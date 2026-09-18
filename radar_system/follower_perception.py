"""Perception ingestion policy for FollowerEngine; no ROS imports."""
import math
from follower_recovery import ScanEvidence
from lidar_track import cluster_points, line_of_sight_gap
from footprint import scan_to_vehicle_frame, optical_to_vehicle, drop_self_hits
from runtime_config import PROFILE


class FollowerPerception:
    def observe_driver(self, d):
        try:
            epoch = d.get('odometry_epoch')
            if epoch is not None and epoch != self.odom_epoch:
                self.odom_epoch = epoch
                self.reset_tracking(keep_odom=False)
            self.driver_armed = bool(d.get('armed', False))
            self.driver_ready = (d.get('ready', '') == 'ready')
            telemetry = d.get('telemetry') or {}
            vel = telemetry.get('velocity')
            now = self.now()
            self.feedback_healthy = False
            age_ms = d.get('age_ms')
            if (isinstance(vel, (list, tuple)) and len(vel) >= 3
                    and isinstance(age_ms, (float, int)) and math.isfinite(age_ms)
                    and 0 <= age_ms <= 1000 * PROFILE['driver']['feedback_timeout_s']):
                vx, wz = float(vel[0]), float(vel[2])
                if math.isfinite(vx) and math.isfinite(wz):
                    self.chassis_speed, self.chassis_yaw_rate = vx, wz
                    self.feedback_stamp = now - age_ms / 1000.0
                    self.feedback_healthy = bool(d.get('connected', False)) and not d.get('holding', False)
        except Exception:
            self.feedback_healthy = False


    def _matches(self, label):
        lbl = (label or '').lower()
        if self.target_class == 'any':
            return True
        if self.target_class in ('person', 'human'):
            return lbl in ('person', 'face')
        return lbl == self.target_class


    def _camera_sees(self, x, y):
        """车体系 (x, y) 处站着的人,相机是否应当能稳定看到(视野边缘留余量)。"""
        cfg = self.cfg
        dx = x - self.camera_mount.x_m
        if not 0.9 <= dx <= cfg.max_follow_distance_m - 0.5:
            return False
        half = math.radians(cfg.camera_hfov_deg / 2 - 6.0)
        rel = math.atan2(y - self.camera_mount.y_m, dx) - self.camera_mount.yaw_rad
        return abs(math.atan2(math.sin(rel), math.cos(rel))) <= half


    def _meas_time(self, stamp, mono_now):
        """消息时间戳(ROS 秒) -> 采集时刻(本节点 monotonic 时基)。

        相机推理有几十~上百毫秒延迟,按「收到时刻」把检测结果和雷达、车身位姿
        对齐会错位,人走得快时相机位置和雷达腿对不上。真实输入缺失或无效时间戳时丢弃；只有显式旧包回放可按到达时刻处理。
        """
        if not stamp:
            return mono_now if self.simulated_odometry else None
        try:
            ros_now = self.ros_time()
        except Exception:
            return mono_now if self.simulated_odometry else None
        lag = ros_now - float(stamp)
        if not -0.05 <= lag <= self.cfg.max_camera_latency_s:
            self.stamp_warnings += 1
            return None if not self.simulated_odometry else mono_now
        return mono_now - max(0.0, lag)


    def observe_targets(self, items):
        self.target_messages += 1
        if not isinstance(items, list):
            return

        now = self.now()
        front = self.cfg.footprint_front_m
        scan_fresh = bool(self.scan_stamp and 0 <= now - self.scan_stamp < PROFILE['safety']['scan_timeout_s'])
        detections = []
        stamp = None
        for item in items:
            if not isinstance(item, dict) or not self._matches(item.get('label')):
                continue
            self.visual_matches += 1
            conf = float(item.get('conf', 0.0) or 0.0)
            if conf < self.cfg.track_low_conf:
                continue
            stamp = stamp or item.get('stamp')
            # 相机能识别人但深度图有空洞时，不能把“人”这个检测也一起
            # 丢掉。优先使用可信的相机深度；深度无效或像素比例不足时，
            # 只在激光扫描新鲜且同方位确有回波时，使用雷达距离兜底。
            raw_z = item.get('z')
            raw_x = item.get('x')
            z = float(raw_z or 0.0)
            x = float(raw_x or 0.0)
            ratio = item.get('depth_ratio')
            range_valid = bool(item.get('range_valid', raw_z is not None))
            camera_ok = (range_valid
                         and self.cfg.min_target_depth_m <= z <= self.cfg.max_follow_distance_m
                         and (ratio is None or float(ratio) >= self.cfg.min_depth_ratio))

            bearing_value = item.get('bearing_rad')
            if bearing_value is not None:
                bearing = float(bearing_value)
            elif z > 0.0:
                bearing = math.atan2(-x, max(z, 0.05))
            else:
                continue

            if camera_ok:
                # 相机俯 15° 装,z 是沿光轴的距离而非水平距离,必须先转到车体系。
                # 误差随目标高度变化,站立的人躯干处可差近 20cm。
                y = float(item.get('y', 0.0) or 0.0)
                px, py, _pz = optical_to_vehicle(x, y, z, self.camera_mount,
                                                 self.cfg.camera_pitch_rad)
                if math.hypot(px, py) < 0.30:
                    continue
                source, depth_sigma = 'camera_depth', None
            else:
                hit = self._line_of_sight(bearing=bearing) if scan_fresh else None
                if hit is None:
                    continue
                # 雷达兜底:取相机视线上最近的雷达点(已在车体系、横向右为正)
                gap, lateral = hit
                if not (0.0 < gap <= self.cfg.max_follow_distance_m):
                    continue
                px, py = gap + front, -lateral
                # 玻璃反光防误检：雷达视线兜底必须校验附近是否存在真实人体尺寸点簇！
                # 平整玻璃窗或平整墙面会被 cluster_points 剔除；若无真实点簇匹配，拒绝当成人体！
                cluster_matched = any(
                    math.hypot(px - c.x, py - c.y) <= 0.40
                    for c in getattr(self, 'latest_clusters', [])
                )
                if not cluster_matched:
                    continue
                source, depth_sigma = 'lidar_fallback', 0.10
                self.lidar_fallback_matches += 1
            detections.append({'x': px, 'y': py, 'conf': conf,
                               'label': item.get('label'), 'range_source': source,
                               'depth_ratio': ratio, 'raw_z': z,
                               'depth_sigma': depth_sigma})

        t_meas = self._meas_time(stamp, now)
        if t_meas is None:
            return
        # 所有检测(含空帧)都交给跟踪器:空帧让轨迹按时老化
        self.people.add_camera(detections, t_meas, now, in_view=self._camera_sees)

        # ---- 相机 / 雷达交叉校验(只看目标视线窄带) ----
        # 旧做法取目标方位 ±10° 扇形(分桶后可达 ±15°)里最近的任何东西,
        # 旁边的椅子/门框被当成人。视线上雷达更近 -> 控制时采信更近的值
        # (可能是人本身也可能是挡在中间的东西,都不该往前冲),
        # 但不改跟踪器里人的位置,转向照常跟人。
        view = self.people.target_view(now)
        if (view is not None and view['source'] == 'camera'
                and view['meta'].get('range_source') == 'camera_depth' and scan_fresh):
            gap = view['x'] - front
            hit = self._line_of_sight(gap=gap, lateral=-view['y'])
            self.los_gap = hit[0] if hit else None
            self.los_time = now
            if hit is not None and gap - hit[0] > self.cfg.range_conflict_m:
                self.range_conflicts += 1   # 仅作遥测:视线上有明显更近的东西
        self.last_conflict = False


    def _line_of_sight(self, gap=None, lateral=None, bearing=None):
        """相机视线上最近的雷达点 -> (车头间距, 横向 右为正) 或 None。"""
        origin = (self.camera_mount.x_m, self.camera_mount.y_m)
        front = self.cfg.footprint_front_m
        if gap is not None:
            target = (gap + front, -lateral)
            beyond = 0.40
        else:
            reach = self.cfg.max_follow_distance_m + front
            heading = bearing + self.camera_mount.yaw_rad   # 相机系方位 -> 车体系
            target = (origin[0] + reach * math.cos(heading),
                      origin[1] + reach * math.sin(heading))
            beyond = 0.0
        hit = line_of_sight_gap(self.scan_points, origin, target, front,
                                self.cfg.los_half_width_m, beyond)
        if hit is None or hit[0] < self.cfg.min_target_depth_m - front:
            return None
        return hit


    def observe_scan(self, msg):
        scan_age = 0.0
        if msg.stamp:
            age_s = self.ros_time() - msg.stamp
            if not -0.1 <= age_s <= PROFILE['safety']['scan_timeout_s']:
                self.scan_evidence = None
                self.scan_stamp = 0.0
                return
            scan_age = max(0.0, age_s)
        elif not self.simulated_odometry:
            self.scan_evidence = None
            self.scan_stamp = 0.0
            return
        n = len(msg.ranges)
        if n == 0:
            self.scan_evidence = None
            self.scan_stamp = 0.0
            return
        self.scan_evidence = ScanEvidence(
            msg.ranges, msg.angle_min, msg.angle_increment,
            max(msg.range_min, self.cfg.scan_min_valid_m), msg.range_max,
            self.lidar_mount, self.footprint, self.cfg.scan_blind_sectors_deg,
            self_hit_skin_m=self.cfg.self_hit_skin_m)
        if (not math.isfinite(msg.angle_min) or not math.isfinite(msg.angle_increment)
                or msg.angle_increment == 0.0):
            self.scan_stamp = 0.0
            self.sectors.clear()
            self.scan_points = []
            return
        # 用 LaserScan 自带的角度字段,不再假设一定是 360 等分
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        cone = math.radians(self.cfg.scan_cone_deg)

        # 按方位分桶存最近距离。只用全向最小值有两个问题:
        # 走廊两侧的墙会把它拉低导致莫名限速;而做相机证伪时需要的是
        # **目标所在方位附近**的距离,不是整个前向扇区的最小值。
        self.sectors.clear()
        nearest = 99.0
        bearings = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or not (msg.range_min <= r <= msg.range_max):
                continue
            if r < self.cfg.scan_min_valid_m:
                continue
            ang = angle_min + i * angle_inc
            ang = math.atan2(math.sin(ang), math.cos(ang))
            self.sectors.add(ang, r)
            bearings.append((ang, r))
            if abs(ang) <= cone and r < nearest:
                nearest = r

        self.min_front_scan = nearest
        # 换算到车体坐标系,供扫掠路径碰撞检查使用。
        # 锥形取最近点只知道"前面多远有东西",不知道那东西是否挡在车宽之内,
        # 也不知道转弯时车体会扫到哪里。
        raw_points = scan_to_vehicle_frame(
            bearings, self.lidar_mount,
            blind_sectors_deg=self.cfg.scan_blind_sectors_deg)
        # 关键:车自己的结构件必须丢弃,不能当成障碍物。
        # 否则它们落在车体轮廓内,corridor_clearance 直接返回 0,车永久停住。
        self.scan_points, dropped = drop_self_hits(
            raw_points, self.footprint, self.cfg.self_hit_skin_m)
        self.self_hits = dropped
        received_at = self.now()
        self.scan_stamp = received_at - scan_age
        # 雷达腿部点簇交给跟踪器: 无论是否配置车尾盲区, 人体跟踪在车尾均不主动屏蔽,
        # 只要在车身几何轮廓之外, 均送入聚类更新, 保证人绕到车尾时跟踪不中断
        tracking_sectors = tuple(s for s in self.cfg.scan_blind_sectors_deg if not (s[0] > 90 and s[1] < -90))
        if tracking_sectors != self.cfg.scan_blind_sectors_deg:
            trk_raw = scan_to_vehicle_frame(bearings, self.lidar_mount, blind_sectors_deg=tracking_sectors)
            trk_clean, _ = drop_self_hits(trk_raw, self.footprint, self.cfg.self_hit_skin_m)
        else:
            trk_clean = self.scan_points
        clusters = cluster_points(trk_clean, origin=(self.lidar_mount.x_m, self.lidar_mount.y_m))
        self.latest_clusters = clusters
        self.people.add_lidar([(c.x, c.y) for c in clusters],
                              self.scan_stamp, received_at)


    def observe_voltage(self, value):
        self.voltage = float(value)

