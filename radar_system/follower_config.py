"""跟随参数和 CLI 覆盖：不依赖 ROS，可单独用于标定、回放。"""
import math
from runtime_config import PROFILE
from dataclasses import dataclass, field
from typing import Optional
from motion_safety import ChassisGeometry, BrakeProfile
from follower_recovery import RecoveryConfig
from footprint import VehicleFootprint, SensorMount


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
    obstacle_standoff_m: float = 0.15   # 正常停车时车头前保留净空 (0.15m 兼顾窄门通行与平滑减速)
    aeb_clearance_m: float = 0.08       # 硬急停线:净空小于此值无条件发 0 (8cm 物理防撞硬急停)
    aeb_release_clearance_m: float = 0.14   # 急停解除回差 (14cm 恢复)
    scan_cone_deg: float = 30.0         # 前向检测扇区半角
    scan_min_valid_m: float = 0.15      # 雷达本体盲区
    # 雷达装在车上,周围的相机支架、天线杆、传感器盒会被扫成距离恒定、
    # 永不消失的"障碍物",导致 AEB 一直误触发。两道过滤:
    #   1. 落在车体轮廓内的点一律丢弃(见 footprint.is_self_hit)
    #   2. 明确知道哪些方位有车体结构时,用角度屏蔽更精准
    # 用 radar_system/scan_doctor.py 在空旷处实测,它会直接给出这两个值。
    self_hit_skin_m: float = 0.05       # 车体轮廓外扩多少算自反射 (5cm, 实车自反射点集中在 x<=0.39m, 0.05m 过滤完全且不吃门框)
    scan_blind_sectors_deg: tuple = (
        (155.0, -130.0),  # 车尾屏蔽扇区
        (10.5, 17.5),     # 前向右侧相机支架/线束盲区 (scan_doctor 测得 11.5°~16.5°)
        (30.0, 33.5),     # 前向右侧结构件盲区 (scan_doctor 测得 30.5°~33.0°)
    )

    # ---- 速度 ----
    max_speed_mps: float = 0.45         # ★ 安全巡航速度上限
    creep_floor_mps: float = 0.08       # 低于此速度直接停,避免电机嗡嗡不转
    kick_mps: float = 0.22              # 静摩擦破除脉冲幅值
    kick_duration_s: float = 0.25       # 脉冲时长
    kp_distance: float = 0.60           # 距离误差 P 增益
    kd_feedforward: float = 0.90        # 目标速度前馈系数,1.0 = 完全跟速
    accel_limit_mps2: float = 0.90      # 加速斜坡
    decel_limit_mps2: float = 2.50      # 减速斜坡,刹车永远比加速陡

    # ---- 转向 ----
    max_steer_rad: float = PROFILE["geometry"]["max_steer_rad"]    # 舵机物理限位
    kp_steer: float = 1.10              # 视线角 -> 前轮转角 增益
    steer_deadband_rad: float = 0.06    # ~3.4°,身体微晃不打舵
    steer_rate_radps: float = 1.20      # 转角变化率限制

    # ---- 刹车物理 ----
    decel_capability_mps2: float = PROFILE["safety"]["decel_mps2"]    # ★ 实测减速度,阿克曼车无主动刹车,别乐观
    control_latency_s: float = PROFILE["safety"]["control_latency_s"]    # ★ 感知到轮子响应的总死时间

    # ---- 目标管理 (统一多人跟踪器,见 person_tracker.py) ----
    track_high_conf: float = 0.45       # 高分框:可以新建轨迹
    track_low_conf: float = 0.15        # 低分框:只能延续已确认的轨迹 (ByteTrack)
    confirm_frames: int = 3             # 相机命中 N 次才确认为人
    target_timeout_s: float = 0.30      # 目标超过这么久没有任何观测即视为丢失
    lost_timeout_s: float = 1.5         # 目标丢失超时
    lost_grace_s: float = 0.40          # 短暂遮挡的宽限期,期间减速而非急停
    min_depth_ratio: float = 0.30       # 深度有效像素占比门限,低于此判无效
    max_camera_latency_s: float = 0.60  # 相机时间戳比现在早这么多以上视为不可信，真实模式丢弃
    camera_hfov_deg: float = 58.0       # Astra S 水平视场,用于「在视野里却没看到」的反向证据
    min_target_range_m: float = 0.30    # 目标距车体中心有效滤波下限(小于此距离视为自反射/底盘噪声)

    # ---- 沿人走过的路跟随 (纯追踪) ----
    follow_breadcrumbs: bool = True
    # 预瞄距离(从后轴算)。满舵转弯半径约 1.77m,预瞄太短转得晚、冲出拐角,
    # 太长又会提前切内角。L 型路线仿真:0.6m 冲出 1.0m;1.2m 内切 4cm、冲出 0.4m。
    pp_lookahead_min_m: float = 1.20
    pp_lookahead_gain_s: float = 0.60   # 每 1 m/s 车速增加的预瞄距离
    pp_lookahead_max_m: float = 1.80

    # ---- 车体足迹 (★ 全部必须实测,见 docs/TUNING.md) ----
    # 改造前避障只在前向锥形里取最近点,等于把车当成一个点:既不知道车有多宽,
    # 也不知道转弯时车体扫过的是一个比车身更宽的圆环。过门刮轮子就是这么来的。
    # 实测值 (2026-09-16):全宽 0.67 前长 0.67 后长 0.18 轴距 0.54 轮距 0.59
    footprint_front_m: float = PROFILE["geometry"]["front_m"]    # 后轴中心 -> 车体最前端(含支架外伸)
    footprint_rear_m: float = PROFILE["geometry"]["rear_m"]    # 后轴中心 -> 车体最后端
    footprint_half_width_m: float = PROFILE["geometry"]["half_width_m"]    # 中线 -> 轮胎外沿 (全宽 0.67 的一半)
    footprint_margin_m: float = 0.025    # 侧向安全余量 (2.5cm, 全宽 0.67+0.05=0.72m 可顺畅穿过 80~85cm 窄门)
    aeb_margin_m: float = 0.015          # AEB 专属物理急停余量 (1.5cm, 只要车体不发生物理碰撞就不锁死)
    lidar_offset_x_m: float = PROFILE["sensors"]["lidar_x_m"]    # 后轴中心 -> 雷达,向前为正(基本在前轴线上)
    lidar_offset_y_m: float = PROFILE["sensors"]["lidar_y_m"]    # 雷达在中线上
    lidar_yaw_deg: Optional[float] = None # 雷达偏航角偏差(度,逆时针为正,优先于 lidar_yaw_rad)
    lidar_yaw_rad: float = PROFILE["sensors"]["lidar_yaw_rad"]    # 雷达 0 度对齐车头
    camera_offset_x_m: float = PROFILE["sensors"]["camera_x_m"]    # 后轴中心 -> 相机,实测 2026-09-16
    camera_offset_y_m: float = PROFILE["sensors"]["camera_y_m"]    # 相机偏离中线(左为正);用 calib_check.py 标定
    camera_yaw_rad: float = PROFILE["sensors"]["camera_yaw_rad"]    # 相机水平朝向相对车头(左为正);用 calib_check.py 标定
    camera_pitch_rad: float = PROFILE["sensors"]["camera_pitch_rad"]    # 相机俯角,实测 15°(向下为正)。
                                         #   深度 z 沿光轴,俯装时不等于水平距离,
                                         #   且误差随目标高度变化(上方 0.6m 处差 19cm)
    min_path_clearance_m: float = 0.15   # 低于此净空就收舵找更直的路,而不是硬停

    # ---- 传感器交叉校验 ----
    range_conflict_m: float = 1.00      # 相机比雷达远这么多即判为冲突
    coasting_speed_cap: float = 0.15    # 滤波器靠外推滑行时的速度上限
    los_half_width_m: float = 0.25      # 交叉校验只看「车->人」视线两侧这么宽的窄带

    # ---- 雷达接力跟踪 (人走出相机视野后继续用雷达跟) ----
    lidar_handoff: bool = True
    lidar_handoff_after_s: float = 0.15  # 相机超过这么久没看到人,才改由雷达接力
    lidar_handoff_max_s: float = 8.0     # 超过这么久未被相机重新确认,不再相信雷达轨迹
    lidar_track_speed_cap: float = 0.45  # 已确认目标的雷达接力上限，仍受驱动和防撞限幅
    lidar_reacquire_gate_m: float = 4.00  # 人快速绕到车后时，允许雷达接回原已确认目标的距离门

    # ---- 车后雷达接力后的快速掉头 ----
    # 阿克曼底盘不能原地旋转；在前方扫掠净空合格时，以较高的受限弧线速度转向。
    # 这些值不是绕过防撞，而是 K-turn 的请求上限，最终仍经过共享位姿、ScanGuard 和驱动限幅。
    rear_target_bearing_deg: float = 75.0
    rear_turn_speed_mps: float = 0.45
    rear_reverse_speed_mps: float = 0.20
    rear_turn_phase_min_s: float = 0.35
    # 已确认车后目标的响应斜坡；仍受净空、目标时效和驱动器限幅约束。
    rear_turn_accel_limit_mps2: float = 1.80
    rear_turn_steer_rate_radps: float = PROFILE["driver"]["steering_rate_rad_s"]

    # ---- 其他 ----
    battery_min_v: float = PROFILE["safety"]["battery_min_v"]
    control_hz: float = 20.0
    enable_pre_steer: bool = True       # 开启静止微速打舵转向对准
    pre_steer_creep_mps: float = 0.08   # 0.08 m/s 微动蠕行带动阿克曼转角对准人
    enable_rear_turnaround: bool = False  # 单元测试保持纯倒车模式兼容，实车启动时默认开启

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
        return SensorMount(x_m=self.camera_offset_x_m, y_m=self.camera_offset_y_m,
                           yaw_rad=self.camera_yaw_rad)

    @property
    def lidar_mount(self):
        return SensorMount(x_m=self.lidar_offset_x_m,
                           y_m=self.lidar_offset_y_m,
                           yaw_rad=self.lidar_yaw_rad)

    def __post_init__(self):
        if self.lidar_yaw_deg is not None:
            self.lidar_yaw_rad = math.radians(self.lidar_yaw_deg)
        if self.follow_stop_m >= self.follow_distance_m:
            raise ValueError("follow_stop_m 必须小于 follow_distance_m")
        if self.aeb_clearance_m >= self.obstacle_standoff_m:
            raise ValueError("aeb_clearance_m 必须小于 obstacle_standoff_m,"
                             "否则硬急停会先于正常减速触发")
        if self.aeb_release_clearance_m <= self.aeb_clearance_m:
            raise ValueError("急停解除回差必须大于急停线")
        if not 0 < self.rear_target_bearing_deg < 90:
            raise ValueError("rear_target_bearing_deg 必须在 0~90 度之间")
        if not 0 < self.rear_reverse_speed_mps <= self.rear_turn_speed_mps <= self.max_speed_mps:
            raise ValueError("车后掉头速度必须位于跟随速度上限内")
        if not 0.1 <= self.rear_turn_phase_min_s <= 1.5:
            raise ValueError("rear_turn_phase_min_s 超出安全范围")
        if not 0 < self.rear_turn_accel_limit_mps2 <= 4.0:
            raise ValueError("rear_turn_accel_limit_mps2 超出安全范围")
        if not 0 < self.rear_turn_steer_rate_radps <= 6.0:
            raise ValueError("rear_turn_steer_rate_radps 超出安全范围")
        if not 0.8 <= self.lidar_reacquire_gate_m <= 4.0:
            raise ValueError("lidar_reacquire_gate_m 超出安全范围")
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


def build_config(args):
    cfg = FollowerConfig()
    calibration_args = {
        'decel_capability_mps2': cfg.decel_capability_mps2,
        'control_latency_s': cfg.control_latency_s,
        'max_steer_deg': math.degrees(cfg.max_steer_rad),
        'camera_pitch_deg': math.degrees(cfg.camera_pitch_rad),
        'lidar_yaw_deg': math.degrees(cfg.lidar_yaw_rad),
    }
    for name, expected in calibration_args.items():
        value = getattr(args, name, None)
        if value is not None and not math.isclose(value, expected, abs_tol=1e-6):
            raise ValueError(name + ': 修改统一 RK3588_ROBOT_CONFIG，不能只覆盖跟随端标定')
    for name in ('follow_distance_m', 'follow_stop_m', 'max_speed_mps',
                 'decel_capability_mps2', 'control_latency_s',
                 'aeb_clearance_m', 'obstacle_standoff_m',
                 'footprint_margin_m', 'aeb_margin_m', 'min_path_clearance_m'):
        value = getattr(args, name, None)
        if value is not None:
            setattr(cfg, name, value)
    # 转角与俯角按度数传入更顺手,这里转成弧度
    if getattr(args, 'lidar_yaw_deg', None) is not None:
        cfg.lidar_yaw_deg = args.lidar_yaw_deg
        cfg.lidar_yaw_rad = math.radians(args.lidar_yaw_deg)
    if getattr(args, 'max_steer_deg', None) is not None:
        cfg.max_steer_rad = math.radians(args.max_steer_deg)
    if getattr(args, 'camera_pitch_deg', None) is not None:
        cfg.camera_pitch_rad = math.radians(args.camera_pitch_deg)
    cfg.recovery.enabled = not bool(getattr(args, "no_recovery", False))
    cfg.enable_pre_steer = bool(getattr(args, 'pre_steer', False))
    cfg.enable_rear_turnaround = not bool(getattr(args, 'no_turnaround', False))
    if not getattr(args, 'rear_blind', False):
        # 实车 N10P 雷达高位安装全向无遮挡, 默认移除车尾屏蔽盲区, 启用 360° 全向避障与倒车
        cfg.scan_blind_sectors_deg = tuple(
            s for s in cfg.scan_blind_sectors_deg if not (s[0] > 90 and s[1] < -90)
        )
    if getattr(args, 'safe_mode', False):
        # 首次实车验证用:速度压到最低,停车距离放大,先确认逻辑正确再放开
        cfg.max_speed_mps = min(cfg.max_speed_mps, 0.30)
        cfg.follow_distance_m = max(cfg.follow_distance_m, 1.20)
        cfg.follow_stop_m = max(cfg.follow_stop_m, 0.90)
        cfg.obstacle_standoff_m = max(cfg.obstacle_standoff_m, 0.45)
        cfg.decel_capability_mps2 = min(cfg.decel_capability_mps2, 0.70)
    cfg.rear_turn_speed_mps = min(cfg.rear_turn_speed_mps, cfg.max_speed_mps)
    cfg.rear_reverse_speed_mps = min(cfg.rear_reverse_speed_mps, cfg.rear_turn_speed_mps)
    cfg.__post_init__()
    return cfg
