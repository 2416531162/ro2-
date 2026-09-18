"""Local follow, turning and bounded recovery policy; no actuator access."""
import math
from runtime_config import PROFILE
from motion_safety import BrakeProfile, brake_envelope, yaw_from_steer, clamp
from footprint import in_blind_sector, swept_path_clearance


class FollowerController:
    def step(self):
        now = self.now()
        cfg = self.cfg
        elapsed = self.dt if self.last_control_time is None else now-self.last_control_time
        self.last_control_time = now
        dt = max(0.0, min(elapsed, 0.10))
        feedback_fresh = bool(self.feedback_healthy and self.feedback_stamp
                              and 0 <= now-self.feedback_stamp <= PROFILE['driver']['feedback_timeout_s'])
        # 里程计:底盘反馈断了按静止处理(此时 healthy=False,车本来就会停)
        if self.simulated_odometry:
            self.people.step_odom(now, self.chassis_speed if feedback_fresh else 0.0,
                                  self.chassis_yaw_rate if feedback_fresh else 0.0)
        local_pose = None if self.simulated_odometry else self.people.odom.pose_at(now)
        feedback_fresh = feedback_fresh and (self.simulated_odometry or local_pose is not None)
        self.people.prune_crumbs()
        view = self.people.target_view(now, lidar_after_s=cfg.lidar_handoff_after_s)
        # 当目标由雷达接力跟踪或相机出现短暂丢帧时，允许更宽的超时门限 (0.80s)，防止离开相机画面瞬间掉锁
        is_lidar_tracking = bool(view is not None and (view['source'] == 'lidar' or view.get('camera_age', 0.0) > 0.25))
        timeout_limit = 0.80 if is_lidar_tracking else cfg.target_timeout_s
        have_target = bool(view is not None and view['update_age'] <= timeout_limit
                           and math.hypot(view['x'], view['y']) >= cfg.min_target_range_m)
        if have_target:
            self.last_target_seen = now - view['update_age']
        age = now-self.last_target_seen if self.last_target_seen else 1e9
        self.view = view if have_target else None
        self.lidar_handoff_active = bool(have_target and view['source'] == 'lidar')
        if self.lidar_handoff_active:
            self.lidar_handoff_frames += 1
        self.aim_point = None
        desired_vx = desired_steer = bearing = 0.0
        is_behind = False
        is_turnaround = False
        gap = float('inf')
        cap_follow = cfg.max_speed_mps
        if have_target:
            front = cfg.footprint_front_m
            bearing = math.atan2(view['y'], view['x'])
            person_gap = (math.hypot(max(0.0, view['x'] - front), view['y'])
                          if view['x'] >= 0
                          else math.hypot(view['x'] + cfg.footprint_rear_m, view['y']))
            gap = person_gap
            if (view['source'] == 'camera' and self.los_gap is not None
                    and now - self.los_time <= 0.3):
                gap = min(gap, self.los_gap)
            person_follow_gap = max(person_gap, self._path_remaining(view)) if cfg.follow_breadcrumbs else person_gap
            v_rel = view['v_fwd'] - self.chassis_speed
            z_person = max(person_follow_gap + v_rel * cfg.control_latency_s * 0.5, .05)
            target_ground_speed = view['v_fwd']
            person_error = z_person - cfg.follow_distance_m

            turnaround_reason = 'face_rear_target'
            # 掉头完成判定：如果已经在掉头过程中，检查是否已调转车头（目标位于前方且偏角进入车头前方视野 ±35°）
            turnaround_complete = (
                self.turnaround_phase != 'IDLE' and
                view['x'] > 0.25 and
                abs(bearing) <= math.radians(35.0)
            )
            if turnaround_complete:
                self.turnaround_phase = 'IDLE'
                self.turnaround_phase_start = 0.0

            # 目标位置分类：在车后（x < 0 或超过 rear_target_bearing_deg）
            is_behind = (self.turnaround_phase != 'IDLE') or (
                view['x'] < 0.0 or abs(bearing) > math.radians(cfg.rear_target_bearing_deg))
            if is_behind:
                if cfg.enable_rear_turnaround:
                    # ---- 阿克曼狭窄空间揉库掉头 (K-turn) 状态机 ----
                    # 刚进入掉头：锁定转向回旋方向(+1: 左转逆时针, -1: 右转顺时针)
                    if self.turnaround_phase == 'IDLE':
                        self.turnaround_dir = 1 if bearing >= 0 else -1

                    fwd_turn_steer = self.turnaround_dir * cfg.max_steer_rad
                    fwd_arc_clear = swept_path_clearance(self.scan_points, self.footprint, self.cfg.geometry, fwd_turn_steer)
                    front_dist = self.min_front_scan

                    # 阿克曼倒车揉库特性：前轮必须向反方向打舵(-turnaround_dir)，使车尾向相反方向摆动，车头保持同向旋转！
                    rev_turn_steer = -self.turnaround_dir * cfg.max_steer_rad
                    rev_clear = (self.recovery.clearance(self.scan_evidence, rev_turn_steer, -1,
                                                          current_steer=rev_turn_steer,
                                                          allow_history=True)
                                  if self.scan_evidence is not None else 0.0)

                    # 初始阶段决策：优先前向回旋，若前向受阻则倒车调整
                    if self.turnaround_phase == 'IDLE':
                        if fwd_arc_clear >= 0.70 and front_dist >= 0.55:
                            self.turnaround_phase = 'FORWARD'
                        elif rev_clear >= 0.45:
                            self.turnaround_phase = 'REVERSE'
                        elif fwd_arc_clear >= rev_clear and fwd_arc_clear > cfg.aeb_clearance_m:
                            self.turnaround_phase = 'FORWARD'
                        elif rev_clear > cfg.aeb_clearance_m:
                            self.turnaround_phase = 'REVERSE'
                        else:
                            self.turnaround_phase = 'FORWARD'
                        self.turnaround_phase_start = now

                    phase_elapsed = now - self.turnaround_phase_start

                    # 阶段换向与滞后防抖：
                    # 单阶段保留一个短的最小冲程，避免在雷达噪声下抖动换向。
                    if self.turnaround_phase == 'FORWARD':
                        fwd_blocked = (fwd_arc_clear < 0.45 or front_dist < 0.45)
                        fwd_urgent = (fwd_arc_clear < 0.32 or front_dist < 0.32)
                        rear_safe = (rev_clear >= 0.40)
                        # Open space is enough for a continuous forward arc.
                        # A time based flip made the old controller alternate
                        # forward/reverse forever even when no obstacle existed.
                        if ((phase_elapsed >= cfg.rear_turn_phase_min_s and fwd_blocked)
                                or fwd_urgent) and rear_safe:
                            self.turnaround_phase = 'REVERSE'
                            self.turnaround_phase_start = now
                            phase_elapsed = 0.0

                    elif self.turnaround_phase == 'REVERSE':
                        rev_blocked = (rev_clear < 0.45)
                        rev_urgent = (rev_clear < 0.32)
                        fwd_safe = (fwd_arc_clear >= 0.55 and front_dist >= 0.50)
                        if ((phase_elapsed >= cfg.rear_turn_phase_min_s and rev_blocked)
                                or rev_urgent) and fwd_safe:
                            self.turnaround_phase = 'FORWARD'
                            self.turnaround_phase_start = now
                            phase_elapsed = 0.0

                    is_turnaround = True
                    if self.turnaround_phase == 'FORWARD':
                        desired_steer = fwd_turn_steer
                        turn_speed = min(cfg.rear_turn_speed_mps, cfg.max_speed_mps)
                        if fwd_arc_clear > cfg.aeb_clearance_m and front_dist > cfg.aeb_clearance_m:
                            desired_vx = turn_speed
                        else:
                            desired_vx = 0.0
                        person_requested_vx = desired_vx
                        turnaround_reason = 'k_turn_forward'
                    elif self.turnaround_phase == 'REVERSE':
                        desired_steer = rev_turn_steer
                        rear_is_blind = in_blind_sector(bearing, cfg.scan_blind_sectors_deg)
                        rev_speed = (min(cfg.rear_reverse_speed_mps,
                                         cfg.recovery.blind_speed_mps)
                                     if rear_is_blind else cfg.rear_reverse_speed_mps)
                        if rev_clear > cfg.aeb_clearance_m:
                            desired_vx = -rev_speed
                        else:
                            desired_vx = 0.0
                        person_requested_vx = desired_vx
                        turnaround_reason = 'k_turn_reverse'
                    else:
                        desired_steer = fwd_turn_steer
                        desired_vx = 0.0
                        person_requested_vx = 0.0
                else:
                    # 纯倒车对准模式（兼容旧配置与基础单元测试）
                    rear_clear = (self.recovery.clearance(self.scan_evidence, self.cmd_steer, -1,
                                                          current_steer=self.cmd_steer,
                                                          allow_history=True)
                                  if self.scan_evidence is not None else 0.0)
                    rear_dist = math.hypot(view['x'] + cfg.footprint_rear_m, view['y'])
                    rear_error = rear_dist - cfg.follow_distance_m
                    rear_bearing_error = (math.pi - bearing) if bearing >= 0 else (-math.pi - bearing)

                    if rear_clear >= cfg.obstacle_standoff_m and rear_error > cfg.deadband_m:
                        rev_steer = clamp(cfg.kp_steer * rear_bearing_error, -cfg.max_steer_rad, cfg.max_steer_rad)
                        desired_steer = rev_steer
                        rev_speed = clamp(cfg.kp_distance * rear_error, cfg.creep_floor_mps,
                                          min(cfg.rear_reverse_speed_mps, cfg.recovery.blind_speed_mps))
                        person_requested_vx = -rev_speed
                        desired_vx = person_requested_vx
                    else:
                        desired_steer = clamp(cfg.kp_steer * rear_bearing_error, -cfg.max_steer_rad, cfg.max_steer_rad)
                        person_requested_vx = 0.0
                        desired_vx = 0.0
            else:
                person_requested_vx = 0.0
                if not (abs(person_error) <= cfg.deadband_m and abs(target_ground_speed) < .10):
                    person_requested_vx = max(0.0, cfg.kd_feedforward*max(0.0, target_ground_speed)
                                              + cfg.kp_distance*person_error)
                desired_vx = person_requested_vx

                # 视线上有更近的障碍物：根据障碍物距离平滑减速，贴近 obstacle_standoff_m 时直接归零
                if gap < person_gap:
                    z_obs = max(gap + v_rel * cfg.control_latency_s * 0.5, .05)
                    obs_error = z_obs - cfg.follow_distance_m
                    if obs_error <= cfg.deadband_m or gap <= cfg.obstacle_standoff_m:
                        desired_vx = 0.0
                    else:
                        desired_vx = min(desired_vx, cfg.kp_distance * obs_error)

                ax, ay = self._aim(view)
                self.aim_point = (ax, ay)
                if abs(math.atan2(ay, ax)) > cfg.steer_deadband_rad:
                    desired_steer = self._pursuit_steer(ax, ay)

                # 侧向对准：若人在侧方（偏角 > 20°），以蠕行小速度带动阿克曼车头转正对准人
                if (cfg.enable_pre_steer and abs(bearing) > math.radians(20.0)
                        and desired_vx < cfg.pre_steer_creep_mps):
                    desired_vx = cfg.pre_steer_creep_mps

            cap_follow = min(cfg.max_speed_mps, brake_envelope(gap, cfg.follow_profile))
            if view['update_age'] > 0.2 and not self.lidar_handoff_active:
                cap_follow = min(cap_follow, cfg.coasting_speed_cap)
            if self.lidar_handoff_active:
                cap_follow = min(cap_follow, cfg.lidar_track_speed_cap)
            person_follow_cap = min(cfg.max_speed_mps, brake_envelope(person_gap, cfg.follow_profile))
            if view['update_age'] > 0.2 and not self.lidar_handoff_active:
                person_follow_cap = min(person_follow_cap, cfg.coasting_speed_cap)
            if self.lidar_handoff_active:
                person_follow_cap = min(person_follow_cap, cfg.lidar_track_speed_cap)
            requested_vx = person_requested_vx
            if desired_vx >= 0:
                desired_vx = min(desired_vx, cap_follow)
            else:
                desired_vx = max(desired_vx, -cfg.recovery.speed_mps)
            self._remember_target(view, gap)
        else:
            self.turnaround_phase = 'IDLE'
            self.turnaround_phase_start = 0.0
            requested_vx = desired_vx
            person_follow_cap = cap_follow
            self.latest_raw = None

        scan_fresh = bool(self.scan_stamp and 0 <= now-self.scan_stamp < PROFILE['safety']['scan_timeout_s']
                          and self.scan_evidence is not None and self.scan_evidence.usable)
        low_battery = not math.isfinite(self.voltage) or self.voltage < cfg.battery_min_v
        healthy = (scan_fresh and feedback_fresh and (self.dry_run or self.driver_armed)
                   and not low_battery and not self.last_conflict and elapsed <= .25)
        # 诊断:页面上逐项显示,现场不用再猜为什么不动
        self.diag = {
            "healthy": healthy,
            "scan_ok": scan_fresh,
            "scan_age_ms": round((now - self.scan_stamp) * 1000) if self.scan_stamp else None,
            "feedback_ok": feedback_fresh,
            "odometry_ok": self.simulated_odometry or local_pose is not None,
            "odometry_source": "simulation" if self.simulated_odometry else PROFILE["localization"]["topic"],
            "feedback_age_ms": (round((now - self.feedback_stamp) * 1000)
                                if self.feedback_stamp else None),
            "driver_armed": bool(self.driver_armed),
            "driver_ready": bool(self.driver_ready),
            "loop_interval_ms": round(elapsed * 1000),
            "loop_late": elapsed > .25,
            "loop_compute_ms": self.last_loop_ms,
        }
        result = self.recovery.update(
            now=now, scan=self.scan_evidence, healthy=healthy,
            speed=self.chassis_speed, yaw_rate=self.chassis_yaw_rate,
            target=have_target, gap=person_gap if have_target else gap, bearing=bearing,
            requested_speed=requested_vx, requested_steer=desired_steer,
            current_steer=self.cmd_steer, follow_cap=person_follow_cap if have_target else cap_follow, lost_age=age,
            odom_ok=feedback_fresh, local_pose=local_pose)
        pre_steer = (cfg.enable_pre_steer and healthy and have_target and not is_behind
                     and result.state in ('HOLDING', 'ALIGNING') and cap_follow > 0
                     and abs(desired_steer) > cfg.steer_deadband_rad)
        if pre_steer:
            result.speed = min(cfg.pre_steer_creep_mps, cap_follow)
            result.steer = desired_steer
        if is_behind:
            result.steer = desired_steer
            if is_turnaround:
                result.state, result.reason = 'TURNAROUND', turnaround_reason
                result.speed = desired_vx
            elif person_requested_vx < 0:
                result.state, result.reason = 'REAR_ALIGNING', 'target_behind'
            else:
                result.state, result.reason = 'HOLDING', 'target_behind'
        self.state, self.limit_reason = result.state, result.reason
        if low_battery:
            self.state, self.limit_reason = 'LOW_BATTERY', 'battery'
        elif self.last_conflict:
            self.state, self.limit_reason = 'SENSOR_CONFLICT', 'range_conflict'
        elif not scan_fresh:
            self.state, self.limit_reason = 'RECOVERY_WAIT', 'scan_unavailable'
        elif not feedback_fresh or not (self.dry_run or self.driver_armed):
            self.state, self.limit_reason = 'RECOVERY_WAIT', 'driver_unavailable'
        elif elapsed > .25:
            self.state, self.limit_reason = 'RECOVERY_WAIT', 'loop_late'

        self.steer_limited = abs(result.steer-desired_steer) > 1e-4
        # 目标已经确认在车后时，普通跟随的舒适转角斜坡会让车头近
        # 300ms 才打满舵；这段时间人可能已经离开雷达的短时关联门。
        # 掉头专用响应只提高“打舵”速度，净空/急停检查仍在下方每周期执行。
        steer_rate = (cfg.rear_turn_steer_rate_radps if is_turnaround
                      else cfg.steer_rate_radps)
        max_dsteer = steer_rate*dt
        self.cmd_steer += clamp(result.steer-self.cmd_steer, -max_dsteer, max_dsteer)
        if not self.recovery.active and healthy:
            if is_behind:
                if is_turnaround:
                    if desired_vx >= 0:
                        desired_vx = min(cfg.rear_turn_speed_mps, cfg.max_speed_mps)
                    else:
                        rear_is_blind = in_blind_sector(bearing, cfg.scan_blind_sectors_deg)
                        reverse_cap = (cfg.recovery.blind_speed_mps if rear_is_blind
                                       else cfg.rear_reverse_speed_mps)
                        desired_vx = max(-min(reverse_cap, cfg.max_speed_mps), desired_vx)
                elif person_requested_vx < 0:
                    desired_vx = max(-min(cfg.rear_reverse_speed_mps, cfg.recovery.blind_speed_mps), person_requested_vx)
                else:
                    desired_vx = 0.0
            else:
                desired_vx = max(0.0, min(desired_vx, result.speed, cap_follow))
        else:
            desired_vx = result.speed if healthy else 0.0
        direction = -1 if desired_vx < 0 else 1
        # Gear changes require measured stop, not just a zero software command.
        if desired_vx*self.chassis_speed < -0.002:
            desired_vx = 0.0
            self.state, self.limit_reason = 'RECOVERY_BRAKE', 'wait_stationary'
        if healthy:
            self.path_clearance = self.recovery.clearance(
                self.scan_evidence, self.cmd_steer, direction, self.cmd_steer,
                allow_history=(self.recovery.active and self.recovery.blind_leg) or (is_behind and direction < 0))
            # Recheck the ACTUAL rate-limited steer, not only the selected future arc.
            # Front AEB is not a rear veto; collision checks still include all corners.
            hard = self.path_clearance < cfg.aeb_clearance_m
            if self.motion_direction != direction:
                self.aeb_latched = hard
            elif hard:
                self.aeb_latched = True
            elif self.path_clearance >= cfg.aeb_release_clearance_m:
                self.aeb_latched = False
        else:
            # 数据不可信(雷达/底盘断流、控制周期抖动 >250ms 等)时净空记 0,
            # 由下面的 cap=0 保证不动;但不能据此锁存 AEB —— 旧版在这里把
            # "数据不可信" 当成 "前方有障碍",页面在车静止时也一直报 AEB 硬急停。
            self.path_clearance = 0.0
        self.motion_direction = direction
        cap = abs(desired_vx)
        profile = (BrakeProfile(cfg.decel_capability_mps2, cfg.control_latency_s, .035, .015)
                   if self.recovery.active else cfg.obstacle_profile)
        cap = min(cap, brake_envelope(self.path_clearance, profile))
        if self.aeb_latched:
            cap = 0.0
            if healthy and abs(desired_vx) > 0:
                self.state, self.limit_reason = 'AEB_EMERGENCY', 'aeb_hard'
        if cap == 0:
            self.speed_slew.reset(0.0)
            self.kick.apply(0.0, False, 0.0, now)
            self.cmd_vx = 0.0
        else:
            # Recovery never uses the forward-only breakaway kick. Its speed and
            # distance caps also apply to the final ramp output on every cycle.
            wanted = direction*cap
            if not self.recovery.active and direction > 0 and not pre_steer and not is_turnaround:
                wanted = self.kick.apply(cap, abs(self.chassis_speed) > .03, cap, now)
            accel_limit = (cfg.rear_turn_accel_limit_mps2 if is_turnaround
                            else None)
            self.cmd_vx = direction*min(
                cap, abs(self.speed_slew.step(wanted, dt,
                                              accel_limit=accel_limit)))
            self.speed_slew.reset(self.cmd_vx)
        self.speed_cap = cap
        self.cmd_wz = yaw_from_steer(self.cmd_vx, self.cmd_steer, cfg.geometry)
        self.last_loop_ms = round((self.now() - now) * 1000, 1)
        self.publish_status(now, have_target, age)


    def _aim(self, view):
        """纯追踪预瞄点:沿人走过的路径点,取第一个超过预瞄距离的点。

        直接朝人打舵会切角:人绕过门框/柜子拐弯时,车走直线蹭上去。
        人走过的地方一定过得去人,沿着走更容易过门、拐弯。
        """
        cfg = self.cfg
        target = (view['x'], view['y'])
        if not cfg.follow_breadcrumbs:
            return target
        look = clamp(cfg.pp_lookahead_min_m + cfg.pp_lookahead_gain_s * abs(self.chassis_speed),
                     cfg.pp_lookahead_min_m, cfg.pp_lookahead_max_m)
        for px, py in view['crumbs']:
            if px > 0.3 and math.hypot(px, py) >= look:
                return px, py
        return target


    def _path_remaining(self, view):
        """车头 -> 路径点 -> 人 的折线长度(只计车头前方的路径点)。"""
        front = self.cfg.footprint_front_m
        px, py = front, 0.0
        total = 0.0
        for cx, cy in view['crumbs']:
            if cx <= front:
                continue
            total += math.hypot(cx - px, cy - py)
            px, py = cx, cy
        return total + math.hypot(view['x'] - px, view['y'] - py)


    def _pursuit_steer(self, ax, ay):
        """后轴系预瞄点 -> 前轮转角(与固件 TurnR = L/tan(δ) + 轮距/2 一致)。"""
        geo = self.cfg.geometry
        d2 = ax * ax + ay * ay
        if d2 < 1e-6 or abs(ay) < 1e-6:
            return 0.0
        radius = d2 / (2.0 * abs(ay))
        denom = radius - 0.5 * geo.track_m
        steer = geo.max_steer_rad if denom <= 1e-6 else math.atan(geo.wheelbase_m / denom)
        return math.copysign(min(steer, self.cfg.max_steer_rad), ay)
