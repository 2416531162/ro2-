#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
「电子跟屁虫」—— 人体 3D 视觉 + 激光雷达智能跟随控制节点 (Person Follower)

订阅
    /camera/ai_detection/targets   String(JSON)  Astra S + YOLOv8 3D 目标
    /scan                          LaserScan     N10P 激光雷达
    /voltage                       Float32       6S 动力电池
    /wheeltec/status               String(JSON)  底盘驱动状态 (含实测车速)
发布
    /cmd_vel                       Twist         速度指令
    /follower/status               String(JSON)  遥测

=============================================================================
本次重写解决的问题:接近目标不减速,直接撞上
=============================================================================

旧版速度律是一个断崖:

    if z > 0.80:  vx = clamp(0.25 + 0.45*(z-0.65), 0.25, 0.65)
    else:         vx = 0.0

`MIN_SPEED_MPS = 0.25` 被写成了**全程速度下限**,于是不管离人多近,
只要还在死区外,车速就永远不低于 0.32 m/s,然后在 0.80 m 处要求瞬间归零。
中间没有任何减速过程。而实际刹停需要:

    感知死时间 (相机+推理+EMA滞后+控制周期) ≈ 0.35 s   -> 0.11 m
    阿克曼车无主动刹车,松油门惯性滑行             -> 0.15~0.30 m
                                             合计 ≈ 0.26~0.41 m

0.80 - 0.4 = 0.40 m,正好撞在 AEB 线上;若此前车速更高则直接撞人。
0.4 m 的 AEB 是布尔锁存,触发时同样只发一个 0,没有任何提前量,救不了。

现在改为三层:
    1. 刹车包络 (主力)  距离换算成允许速度,1.5 m 外就开始收油,0.70 m 自然为零
    2. 分级雷达限速     前方障碍物同样走包络,越近上限越低
    3. 硬急停 (兜底)    越过 0.40 m 无条件发 0

另外把 EMA 换成 alpha-beta 滤波器:EMA 只平滑位置并引入约 240 ms 相位滞后
(读到的距离比真实值偏大),alpha-beta 显式维护速度项,对匀速目标零稳态滞后,
并且顺手给出目标速度用于前馈跟速 —— 车会去**匹配**人的步速,而不是追一下停一下。

转向也一并改了:阿克曼车的前轮转角由固件按 R = Vx/Vz 解算,所以角速度指令
的含义随车速漂移。现在改为先定前轮转角,再按车速反算角速度,转弯半径与车速解耦。

局部自主脱困默认开启:双向选路、停稳换向、限量后退、丢人先观察再弧线搜索。
盲区不是空地;盲区倒车只能沿最近前进路径短退,所有动作受次数/里程/时间上限约束。
"""

import sys
import os
import math
import time
import json
import signal
import argparse
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from motion_safety import (  # noqa: E402
    ChassisGeometry, BrakeProfile, brake_envelope, stopping_distance,
    yaw_from_steer, AlphaBetaTracker, SlewLimiter, BreakawayKick, clamp,
    TargetLock, ScanSectors, reconcile_range,
)
from follower_recovery import LocalRecovery, RecoveryConfig, ScanEvidence  # noqa: E402
from lidar_track import LidarPersonTrack, cluster_points, line_of_sight_gap  # noqa: E402
from footprint import (  # noqa: E402
    VehicleFootprint, SensorMount, scan_to_vehicle_frame, swept_path_clearance,
    limit_steer_for_clearance, optical_to_vehicle, drop_self_hits,
)


# =============================================================================
# 控制参数
# =============================================================================

@dataclass
class FollowerConfig:
    """全部可调参数集中在这里。带 ★ 的必须实车标定。"""

    # ---- 跟随几何 ----
    # 注意:以下距离全部指「**车头最前端**到目标」的间距,不是相机读数。
    # 相机装在车头后方一点且俯装 15°,原始 z 要先做坐标变换才是真实间距。
    follow_distance_m: float = 1.00     # 期望保持的车头-人间距
    follow_stop_m: float = 0.80         # 刹车包络归零点,车头停在这里
    deadband_m: float = 0.12            # 距离死区半宽,±12cm 内不动,防止前后抽搐
    max_follow_distance_m: float = 4.00 # 超过视为脱离
    min_target_depth_m: float = 0.30    # 小于此深度的观测判为噪点

    # ---- 避障 ----
    # 以下均为车头到障碍物的**净空**
    obstacle_standoff_m: float = 0.12   # 正常停车时车头前保留净空 (0.12m 兼顾室内门框防卡死与平滑减速)
    aeb_clearance_m: float = 0.06       # 硬急停线:净空小于此值无条件发 0 (6cm 物理防撞刹停)
    aeb_release_clearance_m: float = 0.10   # 急停解除回差 (10cm 恢复)
    scan_cone_deg: float = 30.0         # 前向检测扇区半角
    scan_min_valid_m: float = 0.15      # 雷达本体盲区
    # 雷达装在车上,周围的相机支架、天线杆、传感器盒会被扫成距离恒定、
    # 永不消失的"障碍物",导致 AEB 一直误触发。两道过滤:
    #   1. 落在车体轮廓内的点一律丢弃(见 footprint.is_self_hit)
    #   2. 明确知道哪些方位有车体结构时,用角度屏蔽更精准
    # 用 radar_system/scan_doctor.py 在空旷处实测,它会直接给出这两个值。
    self_hit_skin_m: float = 0.05       # 车体轮廓外扩多少算自反射 (5cm, 实车自反射点集中在 x<=0.39m, 0.05m 过滤完全且不吃门框)
    scan_blind_sectors_deg: tuple = ((155.0, -130.0),)  # 例: ((-35, -20), (150, 180))

    # ---- 速度 ----
    max_speed_mps: float = 0.55         # ★ 保守起步值,实车验证后再往上加
    creep_floor_mps: float = 0.08       # 低于此速度直接停,避免电机嗡嗡不转
    kick_mps: float = 0.22              # 静摩擦破除脉冲幅值
    kick_duration_s: float = 0.25       # 脉冲时长
    kp_distance: float = 0.60           # 距离误差 P 增益
    kd_feedforward: float = 0.90        # 目标速度前馈系数,1.0 = 完全跟速
    accel_limit_mps2: float = 0.90      # 加速斜坡
    decel_limit_mps2: float = 2.50      # 减速斜坡,刹车永远比加速陡

    # ---- 转向 ----
    max_steer_rad: float = 0.35         # 舵机物理限位
    kp_steer: float = 1.10              # 视线角 -> 前轮转角 增益
    steer_deadband_rad: float = 0.06    # ~3.4°,身体微晃不打舵
    steer_rate_radps: float = 1.20      # 转角变化率限制

    # ---- 刹车物理 ----
    decel_capability_mps2: float = 1.00 # ★ 实测减速度,阿克曼车无主动刹车,别乐观
    control_latency_s: float = 0.35     # ★ 感知到轮子响应的总死时间

    # ---- 目标管理 ----
    min_confidence: float = 0.35
    confirm_frames: int = 3             # 连续 N 帧位置一致才锁定目标
    target_timeout_s: float = 0.30      # 超过这么久没有新观测即视为丢失
    lost_grace_s: float = 0.40          # 短暂遮挡的宽限期,期间减速而非急停
    lock_radius_m: float = 0.55         # 帧间关联半径,超出即认为不是同一个人
    lock_timeout_s: float = 1.50        # 关联不上多久后解锁、允许重选目标
    min_depth_ratio: float = 0.30       # 深度有效像素占比门限,低于此判无效

    # ---- 车体足迹 (★ 全部必须实测,见 docs/TUNING.md 第 9 节) ----
    # 改造前避障只在前向锥形里取最近点,等于把车当成一个点:既不知道车有多宽,
    # 也不知道转弯时车体扫过的是一个比车身更宽的圆环。过门刮轮子就是这么来的。
    # 实测值 (2026-09-16):全宽 0.67 前长 0.67 后长 0.18 轴距 0.54 轮距 0.59
    footprint_front_m: float = 0.67      # 后轴中心 -> 车体最前端(含支架外伸)
    footprint_rear_m: float = 0.18       # 后轴中心 -> 车体最后端
    footprint_half_width_m: float = 0.335  # 中线 -> 轮胎外沿 (全宽 0.67 的一半)
    footprint_margin_m: float = 0.035    # 侧向安全余量 (3.5cm, 全宽 0.67+0.07=0.74m 可顺畅穿过 80~85cm 窄门)
    aeb_margin_m: float = 0.015          # AEB 专属物理急停余量 (1.5cm, 只要车体不发生物理碰撞就不锁死)
    lidar_offset_x_m: float = 0.53       # 后轴中心 -> 雷达,向前为正(基本在前轴线上)
    lidar_offset_y_m: float = 0.0        # 雷达在中线上
    lidar_yaw_rad: float = 0.0           # 雷达 0 度对齐车头
    camera_offset_x_m: float = 0.54      # 后轴中心 -> 相机,实测 2026-09-16
    camera_pitch_rad: float = 0.2618     # 相机俯角,实测 15°(向下为正)。
                                         #   深度 z 沿光轴,俯装时不等于水平距离,
                                         #   且误差随目标高度变化(上方 0.6m 处差 19cm)
    min_path_clearance_m: float = 0.15   # 低于此净空就收舵找更直的路,而不是硬停

    # ---- 传感器交叉校验 ----
    range_conflict_m: float = 1.00      # 相机比雷达远这么多即判为冲突
    coasting_speed_cap: float = 0.15    # 滤波器靠外推滑行时的速度上限
    los_half_width_m: float = 0.25      # 交叉校验只看「车->人」视线两侧这么宽的窄带

    # ---- 雷达接力跟踪 (人走出相机视野后继续用雷达跟) ----
    lidar_handoff: bool = True
    lidar_handoff_after_s: float = 0.25  # 相机超过这么久没看到人,才改由雷达接力
    lidar_handoff_max_s: float = 8.0     # 超过这么久未被相机重新确认,不再相信雷达轨迹
    lidar_track_speed_cap: float = 0.35  # 接力期间身份可信度较低,额外限速

    # ---- 其他 ----
    battery_min_v: float = 21.0
    control_hz: float = 20.0
    enable_pre_steer: bool = False      # 静止预打舵 (见 PROTOCOL.md 8.3),需实车验证
    pre_steer_creep_mps: float = 0.005

    recovery: RecoveryConfig = field(default_factory=RecoveryConfig)
    geometry: ChassisGeometry = field(default_factory=ChassisGeometry)

    @property
    def footprint(self):
        return VehicleFootprint(front_m=self.footprint_front_m,
                                rear_m=self.footprint_rear_m,
                                half_width_m=self.footprint_half_width_m,
                                margin_m=self.footprint_margin_m)

    @property
    def footprint_aeb(self):
        return VehicleFootprint(front_m=self.footprint_front_m,
                                rear_m=self.footprint_rear_m,
                                half_width_m=self.footprint_half_width_m,
                                margin_m=self.aeb_margin_m)

    @property
    def camera_mount(self):
        return SensorMount(x_m=self.camera_offset_x_m, y_m=0.0, yaw_rad=0.0)

    @property
    def lidar_mount(self):
        return SensorMount(x_m=self.lidar_offset_x_m,
                           y_m=self.lidar_offset_y_m,
                           yaw_rad=self.lidar_yaw_rad)

    def __post_init__(self):
        if self.follow_stop_m >= self.follow_distance_m:
            raise ValueError("follow_stop_m 必须小于 follow_distance_m")
        if self.aeb_clearance_m >= self.obstacle_standoff_m:
            raise ValueError("aeb_clearance_m 必须小于 obstacle_standoff_m,"
                             "否则硬急停会先于正常减速触发")
        if self.aeb_release_clearance_m <= self.aeb_clearance_m:
            raise ValueError("急停解除回差必须大于急停线")
        if not -0.6 < self.camera_pitch_rad < 0.6:
            raise ValueError("camera_pitch_rad 超出合理范围 (±34°)")
        if self.camera_offset_x_m > self.footprint_front_m:
            raise ValueError("相机不可能装在车头前面")
        self.geometry.max_steer_rad = self.max_steer_rad

    @property
    def lidar_to_bumper_m(self):
        return max(0.0, self.footprint_front_m - self.lidar_offset_x_m)

    @property
    def follow_profile(self):
        return BrakeProfile(decel_mps2=self.decel_capability_mps2,
                            latency_s=self.control_latency_s,
                            stop_m=self.follow_stop_m,
                            hard_stop_m=self.aeb_clearance_m)

    @property
    def obstacle_profile(self):
        return BrakeProfile(decel_mps2=self.decel_capability_mps2,
                            latency_s=self.control_latency_s,
                            stop_m=self.obstacle_standoff_m,
                            hard_stop_m=self.aeb_clearance_m)


class PersonFollowerNode(Node):

    def __init__(self, config, dry_run=False, target_class="person"):
        super().__init__('person_follower_node')
        self.cfg = config
        self.dry_run = dry_run
        self.target_class = target_class.lower()

        # ---- 感知状态 ----
        self.tracker_z = AlphaBetaTracker(alpha=0.45, beta=0.10,
                                          gate_base_m=0.35, gate_rate_mps=2.5)
        self.tracker_x = AlphaBetaTracker(alpha=0.50, beta=0.08,
                                          gate_base_m=0.30, gate_rate_mps=2.0)
        self.lock = TargetLock(assoc_radius_m=config.lock_radius_m,
                               lost_timeout_s=config.lock_timeout_s,
                               confirm_frames=config.confirm_frames)
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
        self.lidar_track = LidarPersonTrack(handoff_s=config.lidar_handoff_max_s)
        self.last_camera_seen = 0.0
        self.last_scan_time = None
        self.lidar_handoff_active = False
        self.lidar_handoff_frames = 0

        # ---- 执行状态 ----
        self.state = "STANDBY"
        self.limit_reason = "-"
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
        self.last_arm_request = 0.0

        qos_scan = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(String, '/camera/ai_detection/targets', self.on_targets, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_scan)
        self.create_subscription(Float32, '/voltage', self.on_voltage, 10)
        self.create_subscription(String, '/wheeltec/status', self.on_driver_status, 10)

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/follower/status', 10)

        self.cli_arm = self.create_client(SetBool, '/wheeltec/arm')
        self.cli_stop = self.create_client(Trigger, '/wheeltec/stop')
        if not self.dry_run:
            self.arm_chassis(True)

        self.dt = 1.0 / config.control_hz
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.last_print_time = 0.0

        mode = "【仿真演练 DRY-RUN】" if dry_run else "【实车控制 ACTIVE】"
        self.get_logger().info(f">>> 电子跟屁虫就绪 {mode} 目标类别=[{self.target_class}]")
        self.get_logger().info(
            f">>> 保持 {config.follow_distance_m:.2f}m / 包络归零 {config.follow_stop_m:.2f}m "
            f"/ 障碍物留 {config.obstacle_standoff_m:.2f}m "
            f"/ 硬急停 {config.aeb_clearance_m:.2f}m / 极速 {config.max_speed_mps:.2f}m/s")
        self.get_logger().info(
            f">>> 按 a={config.decel_capability_mps2:.2f}m/s^2, T={config.control_latency_s:.2f}s 估算: "
            f"满速刹停需 {stopping_distance(config.max_speed_mps, config.follow_profile):.2f}m")

    # ------------------------------------------------------------------
    # 订阅回调
    # ------------------------------------------------------------------

    def on_driver_status(self, msg):
        try:
            d = json.loads(msg.data)
            self.driver_armed = bool(d.get('armed', False))
            self.driver_ready = (d.get('ready', '') == 'ready')
            telemetry = d.get('telemetry') or {}
            vel = telemetry.get('velocity')
            now = time.monotonic()
            self.feedback_healthy = False
            age_ms = d.get('age_ms')
            if (isinstance(vel, (list, tuple)) and len(vel) >= 3
                    and isinstance(age_ms, (float, int)) and math.isfinite(age_ms)
                    and 0 <= age_ms <= 300):
                vx, wz = float(vel[0]), float(vel[2])
                if math.isfinite(vx) and math.isfinite(wz):
                    self.chassis_speed, self.chassis_yaw_rate = vx, wz
                    self.feedback_stamp = now - age_ms / 1000.0
                    self.feedback_healthy = bool(d.get('connected', False)) and not d.get('holding', False)
            if (not self.dry_run and not self.driver_armed and self.driver_ready
                    and now - self.last_arm_request > 1.5):
                self.arm_chassis(True)
        except Exception:
            self.feedback_healthy = False

    def arm_chassis(self, enable=True):
        if self.dry_run or not self.cli_arm.service_is_ready():
            return
        req = SetBool.Request()
        req.data = enable
        self.last_arm_request = time.monotonic()
        self.cli_arm.call_async(req)

    def trigger_chassis_stop(self):
        if not self.dry_run and self.cli_stop.service_is_ready():
            self.cli_stop.call_async(Trigger.Request())

    def _matches(self, label):
        lbl = (label or '').lower()
        if self.target_class == 'any':
            return True
        if self.target_class in ('person', 'human'):
            return lbl in ('person', 'face')
        return lbl == self.target_class

    def on_targets(self, msg):
        self.target_messages += 1
        try:
            items = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(items, list):
            return

        now = time.monotonic()
        candidates = []
        for item in items:
            if not self._matches(item.get('label')):
                continue
            self.visual_matches += 1
            conf = float(item.get('conf', 0.0) or 0.0)
            if conf < self.cfg.min_confidence:
                continue
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

            lidar_near = None
            source = 'camera_depth'
            if camera_ok:
                # 相机俯 15° 装,z 是沿光轴的距离而非水平距离,必须先转到车体系。
                # 误差随目标高度变化,站立的人躯干处可差近 20cm。
                y = float(item.get('y', 0.0) or 0.0)
                vx_, vy_, _vz = optical_to_vehicle(x, y, z, self.camera_mount,
                                                   self.cfg.camera_pitch_rad)
                gap = vx_ - self.cfg.footprint_front_m      # 车头到人的水平间距
                if gap <= 0.0:
                    continue
                lateral = -vy_
            else:
                scan_fresh = self.scan_stamp and now - self.scan_stamp < 0.5
                hit = self._line_of_sight(bearing=bearing) if scan_fresh else None
                if hit is None:
                    continue
                # 雷达兜底:取相机视线上最近的雷达点,已在车体系、横向右为正
                gap, lateral = hit
                if not (0.0 < gap <= self.cfg.max_follow_distance_m):
                    continue
                lidar_near = gap
                source = 'lidar_fallback'
                self.lidar_fallback_matches += 1

            # z 一律是「车头到目标」的水平间距,x 是横向偏移(右为正)
            candidates.append({'x': lateral, 'z': gap, 'conf': conf,
                               'label': item.get('label'), 'source': source,
                               'bearing': bearing, 'lidar_near': lidar_near,
                               'depth_ratio': ratio, 'raw_z': z})

        # 目标锁定:按运动一致性关联,避免房间里走过第二个人就跟错
        chosen = self.lock.update(candidates, now, self.cfg.follow_distance_m)
        if chosen is None:
            return

        # ---- 相机 / 雷达交叉校验 ----
        # 只看「车 -> 人」这条视线两侧 los_half_width_m 内的雷达点。
        # 旧做法取目标方位 ±10° 扇形(分桶后可达 ±15°)里最近的任何东西,
        # 2m 外扇形宽近 1m,旁边的椅子/门框被当成人:车以为人就在跟前而不走,
        # 差超过 1m 还整帧丢弃,屏幕上有人、车却不动。
        # 现在:视线上雷达更近 -> 采信雷达(保守,可能是人本身也可能是挡在中间的
        # 东西,两种情况都不该往前冲);不再丢弃整帧,转向照常跟人。
        lidar_gap = None
        if chosen.get('source') == 'lidar_fallback':
            z_eff = chosen['z']            # 距离本来就来自雷达
        else:
            if self.scan_stamp and now - self.scan_stamp < 0.5:
                hit = self._line_of_sight(gap=chosen['z'], lateral=chosen['x'])
                lidar_gap = hit[0] if hit else None
            z_eff, occluded = reconcile_range(chosen['z'], lidar_gap,
                                              self.cfg.range_conflict_m)
            if occluded:
                self.range_conflicts += 1   # 仅作遥测:视线上有明显更近的东西
        self.last_conflict = False

        self.last_camera_seen = now
        self.lidar_handoff_active = False
        if self.cfg.lidar_handoff:
            # 让雷达认领这个人:用相机给出的位置(车体系 x 前 y 左)
            self.lidar_track.seed(chosen['z'] + self.cfg.footprint_front_m,
                                  -chosen['x'], now)

        # 目标中断过久,滤波器里的速度估计已失效,重新起算
        if now - self.last_target_seen > self.cfg.lost_grace_s:
            self.tracker_z.reset()
            self.tracker_x.reset()

        self.tracker_z.update(z_eff, now)
        self.tracker_x.update(chosen['x'], now)
        self.last_target_seen = now
        self.latest_raw = {'label': chosen['label'],
                           'conf': round(chosen['conf'], 3),
                           'x': round(chosen['x'], 3),
                           'z': round(chosen.get('raw_z', chosen['z']), 3),
                           'gap_used': round(z_eff, 3),
                           'range_source': chosen.get('source', 'camera_depth'),
                           'depth_ratio': chosen.get('depth_ratio'),
                           'lidar_gap': round(lidar_gap, 3) if lidar_gap is not None else None}

    def _line_of_sight(self, gap=None, lateral=None, bearing=None):
        """相机视线上最近的雷达点 -> (车头间距, 横向 右为正) 或 None。"""
        origin = (self.camera_mount.x_m, self.camera_mount.y_m)
        front = self.cfg.footprint_front_m
        if gap is not None:
            target = (gap + front, -lateral)
            beyond = 0.40
        else:
            reach = self.cfg.max_follow_distance_m + front
            target = (origin[0] + reach * math.cos(bearing),
                      origin[1] + reach * math.sin(bearing))
            beyond = 0.0
        hit = line_of_sight_gap(self.scan_points, origin, target, front,
                                self.cfg.los_half_width_m, beyond)
        if hit is None or hit[0] < self.cfg.min_target_depth_m - front:
            return None
        return hit

    def _lidar_handoff(self, now):
        """相机看不到人时,用雷达轨迹继续喂跟踪滤波器。"""
        cfg = self.cfg
        if (not cfg.lidar_handoff or now - self.last_camera_seen < cfg.lidar_handoff_after_s
                or not self.lidar_track.valid(now)):
            self.lidar_handoff_active = False
            return
        gap = self.lidar_track.x - cfg.footprint_front_m
        lateral = -self.lidar_track.y
        if not (0.0 < gap <= cfg.max_follow_distance_m) or self.lidar_track.x <= 0.0:
            self.lidar_handoff_active = False
            return
        if now - self.last_target_seen > cfg.lost_grace_s:
            self.tracker_z.reset()
            self.tracker_x.reset()
        self.tracker_z.update(gap, now)
        self.tracker_x.update(lateral, now)
        self.last_target_seen = now
        self.lidar_handoff_active = True
        self.lidar_handoff_frames += 1
        # 相机重新看到人时,锁定器要在人现在的位置附近找,而不是在出画前的位置
        if self.lock.locked:
            self.lock.anchor_xz = (lateral, gap)
            self.lock.last_seen = now
        raw = dict(self.latest_raw or {})
        raw.update({'x': round(lateral, 3), 'z': round(gap, 3), 'gap_used': round(gap, 3),
                    'range_source': 'lidar_track', 'lidar_gap': round(gap, 3)})
        raw.setdefault('label', 'person')
        raw.setdefault('conf', 0.0)
        self.latest_raw = raw

    def on_scan(self, msg):
        header = getattr(msg, 'header', None)
        stamp = getattr(header, 'stamp', None)
        if stamp is not None:
            source_ns = stamp.sec * 1000000000 + stamp.nanosec
            if source_ns > 0:
                age_s = (self.get_clock().now().nanoseconds-source_ns)/1e9
                if not -0.1 <= age_s <= 0.5:
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
        self.scan_stamp = time.monotonic()
        if self.cfg.lidar_handoff:
            now = self.scan_stamp
            fresh = bool(self.feedback_healthy and self.feedback_stamp
                         and 0 <= now - self.feedback_stamp <= .30)
            clusters = cluster_points(self.scan_points,
                                      origin=(self.lidar_mount.x_m, self.lidar_mount.y_m))
            self.lidar_track.update(clusters, now,
                                    self.chassis_speed if fresh else 0.0,
                                    self.chassis_yaw_rate if fresh else 0.0)
            self._lidar_handoff(now)
        # 硬急停按当前行进方向判定 (沿 cmd_steer 扫掠路径, 使用物理余量 footprint_aeb)
        active_steer = self.cmd_steer
        corridor_near = swept_path_clearance(
            self.scan_points, self.footprint_aeb, self.cfg.geometry, active_steer)
        if corridor_near < self.cfg.aeb_clearance_m:
            self.aeb_latched = True
        elif corridor_near >= self.cfg.aeb_release_clearance_m:
            self.aeb_latched = False

    def on_voltage(self, msg):
        self.voltage = float(msg.data)

    # ------------------------------------------------------------------
    # 控制主循环
    # ------------------------------------------------------------------

    def control_loop(self):
        now = time.monotonic()
        cfg = self.cfg
        elapsed = self.dt if self.last_control_time is None else now-self.last_control_time
        self.last_control_time = now
        dt = max(0.0, min(elapsed, 0.10))
        age = now-self.last_target_seen if self.last_target_seen else 1e9
        have_target = self.tracker_z.initialized and age <= cfg.target_timeout_s
        desired_vx = desired_steer = bearing = 0.0
        gap = float('inf')
        cap_follow = cfg.max_speed_mps
        if have_target:
            gap = self.tracker_z.position
            z = max(self.tracker_z.predict(cfg.control_latency_s*0.5) or gap, .05)
            bearing = math.atan2(-self.tracker_x.position, z)
            target_ground_speed = self.chassis_speed + self.tracker_z.velocity
            error = z-cfg.follow_distance_m
            if not (abs(error) <= cfg.deadband_m and abs(target_ground_speed) < .10):
                desired_vx = max(0.0, cfg.kd_feedforward*max(0.0, target_ground_speed)
                                 + cfg.kp_distance*error)
            if abs(bearing) > cfg.steer_deadband_rad:
                desired_steer = clamp(cfg.kp_steer*bearing, -cfg.max_steer_rad, cfg.max_steer_rad)
            cap_follow = min(cfg.max_speed_mps, brake_envelope(gap, cfg.follow_profile))
            if self.tracker_z.coasting:
                cap_follow = min(cap_follow, cfg.coasting_speed_cap)
            if self.lidar_handoff_active:
                cap_follow = min(cap_follow, cfg.lidar_track_speed_cap)
            desired_vx = min(desired_vx, cap_follow)

        scan_fresh = bool(self.scan_stamp and 0 <= now-self.scan_stamp < .5
                          and self.scan_evidence is not None and self.scan_evidence.usable)
        feedback_fresh = bool(self.feedback_healthy and self.feedback_stamp
                              and 0 <= now-self.feedback_stamp <= .30)
        low_battery = not math.isfinite(self.voltage) or self.voltage < cfg.battery_min_v
        healthy = (scan_fresh and feedback_fresh and (self.dry_run or self.driver_armed)
                   and not low_battery and not self.last_conflict and elapsed <= .25)
        # 诊断:页面上逐项显示,现场不用再猜为什么不动
        self.diag = {
            "healthy": healthy,
            "scan_ok": scan_fresh,
            "scan_age_ms": round((now - self.scan_stamp) * 1000) if self.scan_stamp else None,
            "feedback_ok": feedback_fresh,
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
            target=have_target, gap=gap, bearing=bearing,
            requested_speed=desired_vx, requested_steer=desired_steer,
            current_steer=self.cmd_steer, follow_cap=cap_follow, lost_age=age)
        pre_steer = (cfg.enable_pre_steer and healthy and have_target
                     and result.state == 'HOLDING' and cap_follow > 0
                     and abs(desired_steer) > cfg.steer_deadband_rad)
        if pre_steer:
            result.speed = min(cfg.pre_steer_creep_mps, cap_follow)
            result.steer = desired_steer
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
        max_dsteer = cfg.steer_rate_radps*dt
        self.cmd_steer += clamp(result.steer-self.cmd_steer, -max_dsteer, max_dsteer)
        desired_vx = result.speed if healthy else 0.0
        direction = -1 if desired_vx < 0 else 1
        # Gear changes require measured stop, not just a zero software command.
        if desired_vx*self.chassis_speed < -0.002:
            desired_vx = 0.0
            self.state, self.limit_reason = 'RECOVERY_BRAKE', 'wait_stationary'
        if healthy:
            self.path_clearance = self.recovery.clearance(
                self.scan_evidence, self.cmd_steer, direction, self.cmd_steer,
                allow_history=self.recovery.active and self.recovery.blind_leg)
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
            if not self.recovery.active and direction > 0 and not pre_steer:
                wanted = self.kick.apply(cap, abs(self.chassis_speed) > .03, cap, now)
            self.cmd_vx = direction*min(cap, abs(self.speed_slew.step(wanted, dt)))
            self.speed_slew.reset(self.cmd_vx)
        self.speed_cap = cap
        self.cmd_wz = yaw_from_steer(self.cmd_vx, self.cmd_steer, cfg.geometry)
        if not self.dry_run:
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = float(self.cmd_vx), float(self.cmd_wz)
            self.pub_cmd_vel.publish(cmd)
        self.last_loop_ms = round((time.monotonic() - now) * 1000, 1)
        self.publish_status(now, have_target, age)

    def _blocked_by(self):
        """路径净空不足时,说明是被什么挡住的(车体系坐标 + 雷达方位)。"""
        block = getattr(self.recovery, 'last_block', None)
        if block is None or self.path_clearance >= 0.30:
            return None
        kind, x, y, at = block
        lx, ly = x - self.lidar_mount.x_m, y - self.lidar_mount.y_m
        return {"kind": kind, "x": round(x, 3), "y": round(y, 3),
                "lidar_bearing_deg": round(math.degrees(math.atan2(ly, lx)), 1),
                "lidar_range_m": round(math.hypot(lx, ly), 3),
                "after_m": round(at, 2)}

    def publish_status(self, now, have_target, age):
        target = None
        if have_target and self.latest_raw:
            target = dict(self.latest_raw)
            target['smooth_z'] = round(self.tracker_z.position, 3)
            target['smooth_x'] = round(self.tracker_x.position, 3)
            target['closing_rate'] = round(self.tracker_z.velocity, 3)
            target['distance'] = target['smooth_z']
            target['coasting'] = self.tracker_z.coasting

        payload = {
            "state": self.state,
            "recovery_enabled": self.cfg.recovery.enabled,
            "recovery_phase": self.recovery.phase,
            "recovery_legs": self.recovery.legs,
            "recovery_distance_m": round(self.recovery.total_distance, 3),
            "blind_reverse_used": self.recovery.blind_used,
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
            "target_locked": self.lock.locked,
            "outliers_rejected": self.tracker_z.rejected_total,
            "range_conflicts": self.range_conflicts,
            "target_messages": self.target_messages,
            "visual_matches": self.visual_matches,
            "lidar_fallback_matches": self.lidar_fallback_matches,
            "blocked_by": self._blocked_by(),
            "diag": self.diag,
            "lidar_handoff": self.lidar_handoff_active,
            "lidar_handoff_frames": self.lidar_handoff_frames,
            "lidar_track": self.lidar_track.status(now),
            "voltage_v": round(self.voltage, 2),
            "cmd_vx": round(self.cmd_vx, 3),
            "cmd_wz": round(self.cmd_wz, 3),
            "cmd_steer_deg": round(math.degrees(self.cmd_steer), 1),
            "chassis_speed": round(self.chassis_speed, 3),
            "timestamp": round(now, 3),
        }
        self.pub_status.publish(String(data=json.dumps(payload, ensure_ascii=False)))

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
            "PATH_BLOCKED":   "\033[1;33m[ 前方无路 ]\033[0m",
            "TRACKING":       "\033[1;32m[ 跟踪追随 ]\033[0m",
            "HOLDING":        "\033[1;36m[ 距离锁定 ]\033[0m",
            "TARGET_BLINK":   "\033[1;33m[ 目标闪断 ]\033[0m",
            "SEARCHING_LOST": "\033[1;35m[ 搜索目标 ]\033[0m",
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

    def stop_robot(self):
        # 信号处理与 finally 都会走到这里,只执行一次,且必须在 destroy_node 之前
        if getattr(self, '_stopped', False):
            return
        self._stopped = True
        self.get_logger().info(">>> 安全刹停中...")
        if self.dry_run:
            return
        stop = Twist()
        for _ in range(15):
            self.pub_cmd_vel.publish(stop)
            time.sleep(0.02)
        self.trigger_chassis_stop()
        self.arm_chassis(False)


def build_config(args):
    cfg = FollowerConfig()
    for name in ('follow_distance_m', 'follow_stop_m', 'max_speed_mps',
                 'decel_capability_mps2', 'control_latency_s',
                 'aeb_clearance_m', 'obstacle_standoff_m',
                 'footprint_margin_m', 'aeb_margin_m', 'min_path_clearance_m'):
        value = getattr(args, name, None)
        if value is not None:
            setattr(cfg, name, value)
    # 转角与俯角按度数传入更顺手,这里转成弧度
    if getattr(args, 'max_steer_deg', None) is not None:
        cfg.max_steer_rad = math.radians(args.max_steer_deg)
    if getattr(args, 'camera_pitch_deg', None) is not None:
        cfg.camera_pitch_rad = math.radians(args.camera_pitch_deg)
    cfg.recovery.enabled = not bool(getattr(args, "no_recovery", False))
    cfg.enable_pre_steer = bool(getattr(args, 'pre_steer', False))
    if getattr(args, 'safe_mode', False):
        # 首次实车验证用:速度压到最低,停车距离放大,先确认逻辑正确再放开
        cfg.max_speed_mps = min(cfg.max_speed_mps, 0.30)
        cfg.follow_distance_m = max(cfg.follow_distance_m, 1.20)
        cfg.follow_stop_m = max(cfg.follow_stop_m, 0.90)
        cfg.obstacle_standoff_m = max(cfg.obstacle_standoff_m, 0.45)
        cfg.decel_capability_mps2 = min(cfg.decel_capability_mps2, 0.70)
    cfg.__post_init__()
    return cfg


def strip_ros_args(argv):
    """去掉 ros2 run 追加的 --ros-args 段,其余参数必须全部可识别。

    旧版 parse_known_args 会静默吞掉写错的参数(如 --max-speed 0.3),
    限速没生效却照常启动,比直接报错危险得多。
    """
    argv = list(argv)
    return argv[:argv.index('--ros-args')] if '--ros-args' in argv else argv


def main():
    p = argparse.ArgumentParser(description="RK3588 电子跟屁虫 - 人体跟随控制节点")
    p.add_argument('--no-recovery', action='store_true', help='关闭自动倒车脱困和丢人搜索')
    p.add_argument('--dry-run', action='store_true',
                   help='仿真演练:照常计算与打印,但不向底盘发指令')
    p.add_argument('--safe-mode', action='store_true',
                   help='首次实车验证用的保守参数组 (低速 + 大停车距离)')
    p.add_argument('--target', type=str, default='person',
                   help='追踪类别: person (默认) / face / any')
    p.add_argument('--follow-distance-m', type=float, default=None, dest='follow_distance_m')
    p.add_argument('--follow-stop-m', type=float, default=None, dest='follow_stop_m')
    p.add_argument('--max-speed-mps', type=float, default=None, dest='max_speed_mps')
    p.add_argument('--decel-mps2', type=float, default=None, dest='decel_capability_mps2',
                   help='★ 实测减速度,标定方法见 docs/TUNING.md')
    p.add_argument('--latency-s', type=float, default=None, dest='control_latency_s',
                   help='★ 实测感知到执行的总死时间')
    p.add_argument('--aeb-clearance-m', type=float, default=None, dest='aeb_clearance_m')
    p.add_argument('--obstacle-standoff-m', type=float, default=None, dest='obstacle_standoff_m')
    p.add_argument('--max-steer-deg', type=float, default=None, dest='max_steer_deg',
                   help='★ 实测满舵角度(度)。轴距 0.54 下它对转弯半径很敏感')
    p.add_argument('--margin-m', type=float, default=None, dest='footprint_margin_m',
                   help='侧向安全余量(米)。过窄门时可临时调小试探')
    p.add_argument('--aeb-margin-m', type=float, default=None, dest='aeb_margin_m',
                   help='AEB专属硬急停侧向物理余量(米),默认0.015')
    p.add_argument('--min-clearance-m', type=float, default=None, dest='min_path_clearance_m',
                   help='最小通行净空门限(米),默认0.15')
    p.add_argument('--camera-pitch-deg', type=float, default=None, dest='camera_pitch_deg',
                   help='相机俯角(度,向下为正),默认 15')
    p.add_argument('--pre-steer', action='store_true',
                   help='静止时用微速度触发预打舵 (PROTOCOL.md 8.3),需实车确认')
    args = p.parse_args(strip_ros_args(sys.argv[1:]))

    cfg = build_config(args)
    try:
        # 由本节点自己处理 SIGINT,保证刹停帧在 ROS 上下文关闭之前发出去
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    node = PersonFollowerNode(cfg, dry_run=args.dry_run, target_class=args.target)

    def on_signal(_sig=None, _frame=None):
        # 只打断 spin;刹停和销毁统一在 finally 里做一次
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n>>> 捕获中断,安全刹停...")
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
