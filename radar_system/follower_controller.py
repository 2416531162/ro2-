"""Local follow, turning and bounded recovery policy; no actuator access."""
import math
from runtime_config import PROFILE
from motion_safety import BrakeProfile, brake_envelope, yaw_from_steer, clamp


class FollowerController:
    def step(self):
        now = self.now()
        cfg = self.cfg
        self.mppi_stop_reason = None
        elapsed = self.dt if self.last_control_time is None else now-self.last_control_time
        self.last_control_time = now
        dt = max(0.0, min(elapsed, 0.10))
        feedback_fresh = bool(self.feedback_healthy and self.feedback_stamp
                              and 0 <= now-self.feedback_stamp <= PROFILE['driver']['feedback_timeout_s'])
        scan_fresh = bool(self.scan_stamp is not None and 0 <= now-self.scan_stamp < PROFILE['safety']['scan_timeout_s']
                          and self.scan_evidence is not None and self.scan_evidence.usable)
        # 里程计:底盘反馈断了按静止处理(此时 healthy=False,车本来就会停)
        if self.simulated_odometry:
            self.people.step_odom(now, self.chassis_speed if feedback_fresh else 0.0,
                                  self.chassis_yaw_rate if feedback_fresh else 0.0)
        local_pose = None if self.simulated_odometry else self.people.odom.pose_at(now)
        feedback_fresh = feedback_fresh and (self.simulated_odometry or local_pose is not None)
        self.recovery.last_depth_used = False
        self.recovery.depth_evidence = self.depth_path.view(
            now, self.people.odom.current() if feedback_fresh else None)
        self.people.prune_crumbs()
        view = self.people.target_view(now, lidar_after_s=cfg.lidar_handoff_after_s)
        # 当目标由雷达接力跟踪或相机出现短暂丢帧时，允许更宽的超时门限，防止离开相机画面瞬间掉锁
        is_lidar_tracking = bool(view is not None and (view['source'] == 'lidar' or view.get('camera_age', 0.0) > 0.60))
        timeout_limit = max(cfg.lost_timeout_s, 2.5) if is_lidar_tracking else cfg.target_timeout_s
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
            # 掉头完成判定：人已回到车头前方、进入相机视野（留几度余量）
            cam_bearing = math.atan2(view['y'] - cfg.camera_offset_y_m,
                                     view['x'] - cfg.camera_offset_x_m)
            turnaround_complete = (
                self.turnaround_phase != 'IDLE' and
                view['x'] - cfg.camera_offset_x_m > 0.25 and
                abs(cam_bearing) <= math.radians(cfg.turnaround_exit_deg)
            )
            if turnaround_complete:
                self.turnaround_phase = 'IDLE'
                self.turnaround_phase_start = 0.0
                self.turn_plan = None

            # 目标位置分类：在车后,或从相机看已偏出 rear_target_bearing_deg
            # (近处的人从相机看偏得更多,也更需要先转车头再跟)
            is_behind = (self.turnaround_phase != 'IDLE') or (
                view['x'] < 0.0 or view['x'] - cfg.camera_offset_x_m < 0.10
                or abs(cam_bearing) > math.radians(cfg.rear_target_bearing_deg))
            if is_behind:
                if cfg.enable_rear_turnaround:
                    # ---- 多段前进/倒车掉头:每周期从当前位姿重新搜索 ----
                    is_turnaround = True
                    plan = self._plan_turnaround(view, now)
                    if plan.direction == 0:
                        desired_vx, desired_steer = 0.0, self.cmd_steer
                        self.turnaround_phase = 'BLOCKED'
                        turnaround_reason = 'turnaround_blocked'
                    else:
                        phase = 'FORWARD' if plan.direction > 0 else 'REVERSE'
                        if phase != self.turnaround_phase:
                            self.turnaround_phase, self.turnaround_phase_start = phase, now
                        self.turnaround_dir = plan.turn_dir
                        desired_steer = plan.steer
                        # 本段按剩余行程收油:到了换向点刚好停住。最后一段向前
                        # 转正后直接交给跟随,不必停下。
                        stroke = plan.stroke_m
                        if plan.reached and plan.direction > 0 and len(plan.segments) == 1:
                            stroke += 0.5
                        desired_vx = plan.direction * min(
                            cfg.rear_turn_speed_mps,
                            brake_envelope(stroke, BrakeProfile(cfg.decel_capability_mps2,
                                                                cfg.control_latency_s, 0.0, 0.0)))
                        turnaround_reason = 'k_turn_forward' if plan.direction > 0 else 'k_turn_reverse'
                    person_requested_vx = desired_vx
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
                if getattr(self, 'mppi', None) is not None and not getattr(self, 'mppi_fallback', False):
                    desired_vx, desired_steer = self.solve_mppi(person_gap, scan_fresh)
                    person_requested_vx = desired_vx
                else:
                    desired_vx, desired_steer = self.solve_pure_pursuit(person_gap, bearing)
                    person_requested_vx = desired_vx
                    # 视线上雷达测得的距离更近(通常是人自己的腿,比相机读数更准;
                    # 或雷达没能确认人时的保守处理):按这个距离平滑减速。
                    # 已确认是「挡在中间的障碍物」的回波不在这里,交给选路绕行。
                    if gap < person_gap:
                        z_obs = max(gap + v_rel * cfg.control_latency_s * 0.5, .05)
                        obs_error = z_obs - cfg.follow_distance_m
                        if obs_error <= cfg.deadband_m or gap <= cfg.obstacle_standoff_m:
                            desired_vx = 0.0
                        else:
                            desired_vx = min(desired_vx, cfg.kp_distance * obs_error)
                    # 交给选路/脱困的是「为了走到人身边想要的速度」。
                    # 视线上的回波就是人自己的腿(比相机读数略近)时,减速后的
                    # 值才是真实请求 —— 否则停稳后状态会一直显示「跟随中」。
                    # 明显更近的未确认回波(墙、玻璃后面)不能这样算:那种情况
                    # 车该判定无路可走去脱困绕路,而不是贴着墙当成已经到位。
                    if person_gap - gap <= cfg.los_obstacle_margin_m:
                        person_requested_vx = desired_vx

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
            if is_turnaround:
                # 掉头各段已按剩余行程收油;人不在行进方向上,不按跟随间距限速。
                # 扫掠净空、刹车包络和 AEB 仍在下方对实际舵角逐周期复核。
                pass
            elif desired_vx >= 0:
                desired_vx = min(desired_vx, cap_follow)
            else:
                desired_vx = max(desired_vx, -cfg.recovery.speed_mps)
            self._remember_target(view, gap)
        else:
            self.turnaround_phase = 'IDLE'
            self.turnaround_phase_start = 0.0
            self.turn_plan = None
            requested_vx = desired_vx
            person_follow_cap = cap_follow
            self.latest_raw = None

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
            odom_ok=feedback_fresh, local_pose=local_pose,
            executed_speed=self.cmd_vx,
            target_room=(person_gap - cfg.follow_stop_m) if have_target else None)
        # 预打舵只用于「停着等」(HOLDING)。ALIGNING 是选路为了绕开障碍物
        # 主动偏离人方向的舵角,用预打舵把它改回朝人,等于把车重新指向
        # 障碍物:仿真里车贴着柱子停住、蠕行 0.08m/s 撞净空,然后倒车脱困。
        pre_steer = (cfg.enable_pre_steer and healthy and have_target and not is_behind
                     and result.state == 'HOLDING' and cap_follow > 0
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
                    # 倒车段能否看见由规划器的首段净空与下方扫掠复核决定
                    # (遮挡/未知区域不算空地),不看人落在哪个方位扇区。
                    pass
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
            hard = (self.path_clearance < cfg.aeb_clearance_m and
                    (self.recovery.last_block is None or self.recovery.last_block[0] == 'obstacle'))
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
        if self.recovery.last_depth_used or self.recovery.last_no_echo_used:
            # Inferred clearance (camera or sampled no echo) is creep-only.
            cap = min(cap, .12)
        profile = (BrakeProfile(cfg.decel_capability_mps2, cfg.control_latency_s, .035, .015)
                   if self.recovery.active else cfg.obstacle_profile)
        cap = min(cap, brake_envelope(self.path_clearance, profile))
        if (healthy and cap == 0 and abs(desired_vx) > 0 and
                self.recovery.last_block is not None and
                self.recovery.last_block[0] in ('unknown', 'no_scan')):
            self.state, self.limit_reason = 'OBSERVATION_WAIT', 'insufficient_observation'
        if self.aeb_latched:
            cap = 0.0
            if healthy and abs(desired_vx) > 0:
                block = self.recovery.last_block
                if block is not None and block[0] in ('unknown', 'no_scan'):
                    self.state, self.limit_reason = 'OBSERVATION_WAIT', 'insufficient_observation'
                else:
                    self.state, self.limit_reason = 'AEB_EMERGENCY', 'aeb_hard'
        if self.recovery.last_depth_used:
            ev = self.depth_path.evidence
            if ev is None or not 0 <= self.now()-ev.stamp <= self.depth_path.MAX_AGE_S:
                cap = 0.0
                self.state, self.limit_reason = 'OBSERVATION_WAIT', 'depth_expired_during_control'
        # Recheck at the end of computation: a slow solve must not publish a fresh
        # timestamp on a command based on observations that expired during it.
        end = self.now()
        stale_during_control = scan_fresh and feedback_fresh and (
            not 0 <= end-self.scan_stamp < PROFILE['safety']['scan_timeout_s']
            or not 0 <= end-self.feedback_stamp <= PROFILE['driver']['feedback_timeout_s']
            or (not self.simulated_odometry and self.people.odom.pose_at(end) is None))
        if self.mppi_stop_reason or (self.mppi is not None and end-now > self.dt) or stale_during_control:
            cap = 0.0
            self.state = 'RECOVERY_WAIT'
            self.limit_reason = (self.mppi_stop_reason or
                ('observations_expired_during_control' if stale_during_control else 'control_overrun'))
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


    def _plan_turnaround(self, view, now):
        """人在车后/侧后:搜索前进/倒车满舵段,返回 TurnPlan(见 turnaround.py)。"""
        cfg = self.cfg
        horizon = self.turn_planner.lengths[-1] + self.turn_planner.first_buffer + 0.05

        def first_clear(direction, steer):
            return self.recovery.clearance(self.scan_evidence, steer, direction,
                                           current_steer=self.cmd_steer,
                                           allow_history=direction < 0, horizon=horizon)

        plan = self.turn_planner.plan(
            (view['x'], view['y']), self.scan_points,
            first_clearance=first_clear if self.scan_evidence is not None else (lambda d, a: 0.0),
            current_speed=self.chassis_speed,
            prefer_turn=self.turnaround_dir if self.turnaround_phase != 'IDLE' else None)
        self.turn_plan = plan
        return plan


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


    def solve_pure_pursuit(self, z, bearing):
        """几何解 + 距离 P 律 + 目标速度前馈。确定性,无依赖,默认走这条。"""
        cfg = self.cfg
        vx = steer = 0.0
        v = self.view
        target_ground_speed = v['v_fwd'] if v else self.chassis_speed
        error = z - cfg.follow_distance_m
        if not (abs(error) <= cfg.deadband_m and abs(target_ground_speed) < .10):
            vx = max(0.0, cfg.kd_feedforward * max(0.0, target_ground_speed)
                     + cfg.kp_distance * error)
        if abs(bearing) > cfg.steer_deadband_rad:
            ax, ay = self._aim(v) if v else (z + cfg.footprint_front_m, 0.0)
            self.aim_point = (ax, ay)
            steer = self._pursuit_steer(ax, ay)
        return vx, steer


    def solve_mppi(self, z, scan_fresh):
        """采样优化:跟随、避障、视野保持共用一个代价函数。

        它只产出**参考量**。产出之后照样过刹车包络、扫掠净空、AEB 和脱困
        状态机 —— MPPI 是软约束优化器,加权平均出来的控制可能落在没有任何
        样本占据的区域,把硬安全交给它是拿碰撞保证换一个调参旋钮。

        连续判不可行超过上限就整段回退到纯追踪:偶尔找不到解是正常的,
        一直找不到说明它自己坏了,不该让车一直瘫着。
        """
        cfg = self.cfg
        v = self.view
        if v is not None:
            person = (v['x'], v['y'])
            velocity = (v['v_fwd'], v['v_lat'])
        else:
            person = (z + cfg.footprint_front_m, 0.0)
            velocity = (0.0, 0.0)
        now = self.now()
        if (self._mppi_target_id != self.people.target_id or self._mppi_last_at is None
                or now-self._mppi_last_at > 2.5*self.dt):
            self.mppi.reset()
        self._mppi_target_id, self._mppi_last_at = self.people.target_id, now
        try:
            sol = self.mppi.solve(self.scan_points, person, velocity,
                                  self.chassis_speed, self.cmd_steer,
                                  usable=scan_fresh)
        except Exception:
            self.mppi_fallback = True
            self.mppi_last = None
            self.mppi_stop_reason = 'mppi_error'
            self.mppi.reset()
            return 0.0, self.cmd_steer
        self.mppi_last = sol
        invalid = (not all(math.isfinite(value) for value in (sol.speed, sol.steer, sol.solve_ms))
                   or (sol.feasible and not all(math.isfinite(value)
                                               for value in (sol.cost, sol.min_clearance))))
        too_slow = sol.solve_ms > cfg.mppi_solve_budget_ms
        if sol.feasible and not too_slow and not invalid:
            self.mppi_infeasible_streak = 0
        else:
            self.mppi_infeasible_streak += 1
            if self.mppi_infeasible_streak >= cfg.mppi_fallback_after:
                self.mppi_fallback = True
            self.mppi_stop_reason = ('mppi_invalid' if invalid else
                                    'mppi_timeout' if too_slow else sol.reason)
            self.mppi.reset()
            return 0.0, self.cmd_steer
        return sol.speed, sol.steer
