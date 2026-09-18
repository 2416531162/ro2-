"""Follower status serialization inputs and optional terminal rendering."""
import sys
import math
from runtime_config import PROFILE, profile_hash


class FollowerTelemetry:
    def _remember_target(self, view, gap):
        meta = view['meta']
        source = {'camera': meta.get('range_source', 'camera_depth'),
                  'lidar': 'lidar_track'}.get(view['source'], 'predicted')
        raw_z = meta.get('raw_z')
        self.latest_raw = {'label': view['label'] or 'person',
                           'conf': round(view['conf'], 3),
                           'x': round(-view['y'], 3),
                           'z': round(raw_z, 3) if raw_z else round(gap, 3),
                           'gap_used': round(gap, 3),
                           'range_source': source,
                           'depth_ratio': meta.get('depth_ratio'),
                           'lidar_gap': (round(self.los_gap, 3)
                                         if self.los_gap is not None else None),
                           'track_id': view['id']}


    def _blocked_by(self):
        """路径净空不足时,说明是被什么挡住的(车体系坐标 + 雷达方位)。"""
        block = getattr(self.recovery, 'last_block', None)
        if block is None or self.path_clearance >= 0.30:
            return None
        kind, x, y, at = block
        lx, ly = x - self.lidar_mount.x_m, y - self.lidar_mount.y_m
        out = {"kind": kind, "x": round(x, 3), "y": round(y, 3),
               "lidar_bearing_deg": round(math.degrees(math.atan2(ly, lx)), 1),
               "lidar_range_m": round(math.hypot(lx, ly), 3),
               "after_m": round(at, 2)}
        ev = self.scan_evidence
        if kind == "unknown" and ev is not None:
            # 这个方向上每条光束为什么不算数:none=没回波 near=太近(车身遮挡)
            # self=打在车身上 masked=屏蔽扇区 far=超量程
            rays = ev.explain(x, y)
            counts = {}
            for _deg, _r, cause in rays:
                counts[cause] = counts.get(cause, 0) + 1
            out["ray_causes"] = counts
            out["rays"] = rays
        return out


    def _lidar_track_status(self, now):
        """兼容旧页面字段:目标轨迹被雷达更新过才给出。"""
        v = self.people.target_view(now) if self.people.target_id is not None else None
        if v is None or v['lidar_hits'] == 0:
            return None
        return {"x": round(v['x'], 3), "y": round(v['y'], 3),
                "speed": round(math.hypot(v['v_fwd'], v['v_lat']), 2),
                "since_camera_s": round(v['camera_age'], 2),
                "confident_age_s": round(v['confident_age'], 2),
                "valid": v['confident_age'] <= self.cfg.lidar_handoff_max_s and v['update_age'] <= 0.6}


    def publish_status(self, now, have_target, age):
        target = None
        view = self.view
        if have_target and view is not None:
            if self.latest_raw:
                target = dict(self.latest_raw)
            else:
                target = {'label': view.get('label') or 'person', 'conf': 0.85,
                          'gap_used': round(math.hypot(view['x'], view['y']), 2)}
            target['veh_x'] = round(view['x'], 3)
            target['veh_y'] = round(view['y'], 3)
            target['x'] = round(-view['y'], 3)
            target['y'] = round(view['x'], 3)
            target['smooth_z'] = round(view['x'] - self.cfg.footprint_front_m, 3)
            target['smooth_x'] = round(-view['y'], 3)
            target['closing_rate'] = round(view['v_fwd'] - self.chassis_speed, 3)
            target['ground_speed'] = round(math.hypot(view['v_fwd'], view['v_lat']), 2)
            target['distance'] = target.get('gap_used') or round(math.hypot(view['x'], view['y']), 2)
            target['coasting'] = view['update_age'] > 0.2
            target['sigma_m'] = round(view['sigma'], 3)

            # 计算人相对车头的精确方位角（车头正前方为 0°，左侧为正，右侧为负，车尾为 ±180°）
            bearing_rad = math.atan2(view['y'], view['x'])
            bearing_deg = round(math.degrees(bearing_rad), 1)
            dist_val = round(math.hypot(view['x'], view['y']), 2)

            abs_deg = abs(bearing_deg)
            if abs_deg <= 20.0:
                dir_name, clock_name = "正前方", "12点钟"
            elif bearing_deg > 20.0 and bearing_deg <= 70.0:
                dir_name, clock_name = "左前方", "10点钟"
            elif bearing_deg > 70.0 and bearing_deg <= 110.0:
                dir_name, clock_name = "正左方", "9点钟"
            elif bearing_deg > 110.0 and bearing_deg <= 160.0:
                dir_name, clock_name = "左后方", "8点钟"
            elif abs_deg > 160.0:
                dir_name, clock_name = "正后方", "6点钟"
            elif bearing_deg < -20.0 and bearing_deg >= -70.0:
                dir_name, clock_name = "右前方", "2点钟"
            elif bearing_deg < -70.0 and bearing_deg >= -110.0:
                dir_name, clock_name = "正右方", "3点钟"
            else:
                dir_name, clock_name = "右后方", "4点钟"

            in_cam = bool(view.get('source') == 'camera' and view.get('camera_age', 0.0) < 0.35)
            src_name = "视觉+雷达" if in_cam else "雷达接力"
            summary_text = f"{dir_name} {bearing_deg:+.0f}° ({clock_name}) · {dist_val:.2f}m [{src_name}]"

            target['bearing_deg'] = bearing_deg
            target['direction_name'] = dir_name
            target['clock_name'] = clock_name
            target['in_camera_view'] = in_cam
            target['tracking_source'] = 'camera' if in_cam else 'lidar'
            target['direction_summary'] = summary_text

        payload = {
            "state": self.state,
            "profile_hash": profile_hash(PROFILE),
            "recovery_enabled": self.cfg.recovery.enabled,
            "recovery_phase": self.recovery.phase,
            "recovery_legs": self.recovery.legs,
            "recovery_distance_m": round(self.recovery.total_distance, 3),
            "blind_reverse_used": self.recovery.blind_used,
            "reverse_trail_m": round(self.recovery.trail_length, 2),
            "stall_steer_deg": (round(math.degrees(self.recovery.stall_steer), 1)
                                if self.recovery.stall_steer is not None else None),
            "reverse_budget_m": round(max(0.0, self.recovery.cfg.blind_reverse_m
                                          - self.recovery.blind_distance), 2),
            "recovery_exhausted": self.recovery.exhausted,
            "dry_run": self.dry_run,
            "target": target,
            "target_seen_age_ms": round(age * 1000, 1) if age < 1e8 else None,
            "aeb_min_scan_m": round(self.min_front_scan, 2),
            "path_clearance_m": round(self.path_clearance, 2),
            "self_hits": self.self_hits,
            "footprint_width_m": round(self.footprint.width_m, 2),
            "steer_limited": self.steer_limited,
            "min_gap_needed_m": round(self.footprint.min_gap_needed(), 2),
            "aeb_active": self.aeb_latched,
            "speed_cap_mps": round(self.speed_cap, 3),
            "limit_reason": self.limit_reason,
            "controller": ('pure-pursuit' if getattr(self, 'mppi', None) is None or getattr(self, 'mppi_fallback', False)
                           else 'mppi'),
            "mppi": (None if getattr(self, 'mppi_last', None) is None else {
                "feasible": self.mppi_last.feasible,
                "reason": self.mppi_last.reason,
                "cost": round(self.mppi_last.cost, 1),
                "clearance_m": round(self.mppi_last.min_clearance, 3),
                "solve_ms": round(self.mppi_last.solve_ms, 2),
                "obstacles": self.mppi_last.obstacles,
                "over_budget": bool(self.mppi_last.solve_ms
                                    > self.cfg.mppi_solve_budget_ms),
                "failure_streak": self.mppi_infeasible_streak,
                "fell_back": self.mppi_fallback,
            }),
            "target_locked": bool(have_target),
            "signature_ready": bool(self.lock.signature_fresh(now)) if hasattr(self, 'lock') else False,
            "appearance_rejects": getattr(self.lock, 'rejected_appearance', 0) if hasattr(self, 'lock') else 0,
            "target_id": self.people.target_id,
            "target_switches": self.people.switches,
            "target_reacquires": self.people.reacquires,
            "lidar_ambiguous_frames": self.people.lidar_ambiguous_frames,
            "tracks": self.people.summary(now),
            "outliers_rejected": self.people.rejected,
            "stamp_warnings": self.stamp_warnings,
            "dropped_not_person": self.people.dropped_unseen,
            "aim_point": ([round(self.aim_point[0], 2), round(self.aim_point[1], 2)]
                          if self.aim_point else None),
            "range_conflicts": self.range_conflicts,
            "target_messages": self.target_messages,
            "visual_matches": self.visual_matches,
            "lidar_fallback_matches": self.lidar_fallback_matches,
            "blocked_by": self._blocked_by(),
            "diag": self.diag,
            "depth_path": dict(self.depth_path.status(now),
                               used=self.recovery.last_depth_used),
            "lidar_handoff": self.lidar_handoff_active,
            "lidar_handoff_frames": self.lidar_handoff_frames,
            "lidar_track": self._lidar_track_status(now),
            "voltage_v": round(self.voltage, 2),
            "turnaround_phase": self.turnaround_phase,
            "cmd_vx": round(self.cmd_vx, 3),
            "cmd_wz": round(self.cmd_wz, 3),
            "cmd_steer_deg": round(math.degrees(self.cmd_steer), 1),
            "chassis_speed": round(self.chassis_speed, 3),
            "timestamp": round(now, 3),
        }
        self.status = payload
        self.emit_status(payload)

        if now - self.last_print_time >= 0.20:
            self.last_print_time = now
            self.print_dashboard(payload)


    def print_dashboard(self, s):
        colors = {
            "ALIGNING":       "[ 调整过门姿态 ]",
            "SEARCH_SCAN":    "[ 停车观察目标 ]",
            "SEARCH_TURN":    "[ 转弯搜索 / 掉头 ]",
            "RECOVERY_REVERSE": "[ 限量倒车脱困 ]",
            "RECOVERY_BRAKE": "[ 停稳换向 ]",
            "RECOVERY_WAIT":  "[ 等待可行路径 ]",
            "RECOVERY_EXHAUSTED": "[ 脱困达到上限 ]",
            "OBSERVATION_WAIT": "[ 前方观测不足，等待雷达 ]",
            "PATH_BLOCKED":   "\033[1;33m[ 前方障碍受阻 ]\033[0m",
            "TRACKING":       "\033[1;32m[ 跟踪追随 ]\033[0m",
            "HOLDING":        "\033[1;36m[ 距离锁定 ]\033[0m",
            "REAR_ALIGNING":  "\033[1;36m[ 车后对准倒车 ]\033[0m",
            "TURNAROUND":     "\033[1;36m[ 揉库掉头对准 ]\033[0m",
            "TARGET_BLINK":   "\033[1;33m[ 目标闪断 ]\033[0m",
            "SEARCHING_LOST": "\033[1;35m[ 搜索目标 ]\033[0m",
            "COLLISION_AEB":  "\033[1;41;37m[ 防撞急停 ]\033[0m",
            "AEB_EMERGENCY":  "\033[1;41;37m[ 硬急停 ]\033[0m",
            "SENSOR_CONFLICT": "\033[1;41;37m[ 传感器冲突 ]\033[0m",
            "LOW_BATTERY":    "\033[1;31m[ 低电量 ]\033[0m",
            "STANDBY":        "\033[1;30m[ 待命 ]\033[0m",
        }
        tag = colors.get(s['state'], f"[{s['state']}]")
        t = s['target']
        info = (f"{t['label']} X:{t['smooth_x']:+.2f} Z:{t['smooth_z']:.2f}m "
                f"v:{t['closing_rate']:+.2f}m/s" if t else "未发现目标")
        cap_col = "\033[1;31m" if s['speed_cap_mps'] < 0.2 else "\033[1;32m"
        lock_tag = "\033[1;32m锁定\033[0m" if s['target_locked'] else "\033[1;33m未锁\033[0m"
        limit_tag = "\033[1;33m收\033[0m" if s.get('steer_limited') else " "
        line = (f"\r{'[DRY]' if s['dry_run'] else '[RUN]'} {tag} {lock_tag} "
                f"{info:<44} | 净空 {s['path_clearance_m']:5.2f}m "
                f"| {cap_col}上限 {s['speed_cap_mps']:.2f}\033[0m ({s['limit_reason']:<17}) "
                f"| vx={s['cmd_vx']:+.2f} 舵={s['cmd_steer_deg']:+5.1f}°{limit_tag} "
                f"| 野值{s['outliers_rejected']:>3d} 冲突{s['range_conflicts']:>3d} "
                f"| {s['voltage_v']:.1f}V   ")
        sys.stdout.write(line)
        sys.stdout.flush()
