"""Transport-independent follower engine.

The three policy classes operate on this single state owner. They contain no ROS
clients/publishers, processes or actuator calls. Production feeds measured local
poses; synthetic integration must be explicitly selected by a test/replay.
"""
import math
from motion_safety import SlewLimiter, BreakawayKick, ScanSectors
from follower_recovery import LocalRecovery
from person_tracker import PersonTracker
from runtime_config import PROFILE
from follower_perception import FollowerPerception
from follower_controller import FollowerController
from follower_telemetry import FollowerTelemetry


class FollowerEngine(FollowerPerception, FollowerController, FollowerTelemetry):
    def __init__(self, config, *, now, ros_time, emit_status=lambda _: None,
                 dry_run=False, target_class='person', simulated_odometry=False):
        self.now, self.ros_time, self.emit_status = now, ros_time, emit_status
        self.simulated_odometry = simulated_odometry
        self.odom_epoch = None
        self.status = {}
        self.cfg = config
        self.dry_run = dry_run
        self.target_class = target_class.lower()

        # ---- 感知状态 ----
        self.people = PersonTracker(high_conf=config.track_high_conf,
                                    low_conf=config.track_low_conf,
                                    confirm_hits=config.confirm_frames,
                                    lidar_only_max_s=config.lidar_handoff_max_s,
                                    reacquire_after_s=config.lidar_handoff_after_s,
                                    reacquire_radius_m=config.lidar_reacquire_gate_m,
                                    prefer_distance_m=config.follow_distance_m)
        self.people.lidar_enabled = config.lidar_handoff
        self.view = None               # 本周期目标视图(车体系)
        self.los_gap = None            # 相机视线上雷达测得的车头间距
        self.los_time = 0.0
        self.aim_point = None          # 纯追踪预瞄点(车体系)
        self.stamp_warnings = 0
        self.sectors = ScanSectors(half_fov_deg=60.0, bin_deg=5.0)
        self.footprint = config.footprint
        self.footprint_aeb = config.footprint_aeb
        self.lidar_mount = config.lidar_mount
        self.camera_mount = config.camera_mount
        self.scan_points = []          # 车体坐标系下的雷达点,供扫掠检查用
        self.scan_evidence = None
        self.recovery = LocalRecovery(self.footprint, config.geometry,
                                      config.obstacle_profile, config.recovery)
        self.feedback_stamp = 0.0
        self.feedback_healthy = False
        self.chassis_yaw_rate = 0.0
        self.last_control_time = None
        self.motion_direction = 1
        self.path_clearance = 99.0
        self.self_hits = 0
        self.steer_limited = False     # 本帧是否因净空不足而收了舵
        self.latest_raw = None
        self.last_target_seen = 0.0
        self.min_front_scan = 99.0
        self.scan_stamp = 0.0
        self.voltage = 24.0
        self.range_conflicts = 0
        self.last_conflict = False
        self.target_messages = 0
        self.visual_matches = 0
        self.lidar_fallback_matches = 0
        self.lidar_handoff_active = False
        self.lidar_handoff_frames = 0
        self.latest_clusters = []

        # ---- 执行状态 ----
        self.state = "STANDBY"
        self.limit_reason = "-"
        self.turnaround_phase = 'IDLE'     # 'IDLE', 'FORWARD', 'REVERSE'
        self.turnaround_dir = 1            # +1 (左转 CCW), -1 (右转 CW)
        self.turnaround_phase_start = 0.0
        self.aeb_latched = False
        self.speed_slew = SlewLimiter(config.accel_limit_mps2, config.decel_limit_mps2)
        self.kick = BreakawayKick(config.kick_mps, config.kick_duration_s,
                                  config.creep_floor_mps)
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self.cmd_steer = 0.0
        self.chassis_speed = 0.0
        self.speed_cap = 0.0
        self.last_loop_ms = None
        self.diag = {}

        # ---- 底盘使能 ----
        self.driver_armed = False
        self.driver_ready = False


        self.people.odom.horizon_s = PROFILE['localization']['history_s']
        self.people.odom.strict = not simulated_odometry
        self.people.odom.extrapolation_s = PROFILE['localization']['max_extrapolation_s']
        self.dt = 1.0 / config.control_hz
        self.last_print_time = 0.0

    def reset_tracking(self, keep_odom=True):
        self.people.reset(keep_odom=keep_odom)
        self.recovery = LocalRecovery(self.footprint, self.cfg.geometry,
                                      self.cfg.obstacle_profile, self.cfg.recovery)
        self.turnaround_phase = 'IDLE'
        self.turnaround_phase_start = self.last_target_seen = 0.0
        self.cmd_vx = self.cmd_wz = self.cmd_steer = 0.0
        self.aeb_latched = False
        self.speed_slew.reset(0.0)
        self.last_control_time = None

    def observe_pose(self, pose):
        cfg = PROFILE['localization']
        if pose.frame != PROFILE['frames']['odom'] or pose.child_frame != PROFILE['frames']['base']:
            self.reset_tracking(keep_odom=False)
            return False
        age = self.ros_time() - pose.stamp
        if not math.isfinite(age) or not 0 <= age <= cfg['timeout_s']:
            return False
        buf = self.people.odom
        revision = buf.revision
        accepted = buf.add(self.now() - age, pose.x, pose.y, pose.yaw,
                           cfg['max_speed_m_s'], cfg['max_yaw_rate_rad_s'])
        if buf.revision != revision:
            self.reset_tracking(keep_odom=True)
        return accepted

    def pause(self):
        self.cmd_vx = self.cmd_wz = 0.0
        self.speed_slew.reset(0.0)
        self.state, self.limit_reason = 'PAUSED', 'motion_authority'
        self.publish_status(self.now(), False, 1e9)
