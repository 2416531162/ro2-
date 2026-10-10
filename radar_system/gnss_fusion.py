"""后轴中心的 RTK / 双天线 / 陀螺 / 轮速融合；不依赖 ROS。填了天线高度时按 IMU 横滚俯仰做倾斜补偿。

时间戳必须已经换算到同一个单调时钟。有限历史内按测量时间重放，
标定统计也参与回放，避免延迟观测改变结果或重复累计样本。
"""
import bisect
import math
from dataclasses import dataclass
from wheel_scale import WheelScale


@dataclass(frozen=True)
class ImuSample:
    t: float
    gyro_z: float
    # IMU 自己算的航向角（弧度，左转为正，上电时为 0 的相对值）；yaw_input = 'angle' 时用它的增量推算航向
    yaw: float = None
    # IMU 算好的横滚、俯仰（弧度，ROS REP-103：横滚左侧抬起为正，俯仰车头压低为正）；倾斜补偿用，没有就不补偿
    roll: float = None
    pitch: float = None


@dataclass(frozen=True)
class WheelSample:
    t: float
    speed: float
    yaw_rate: float
    # 有区间时 t 是起点，speed / yaw_rate 是区间平均；end_* 是终点即时值。
    # 同一轮程作为预测输入替换旧速度区间，不能再当独立位置观测注入。
    end_t: float = None
    end_speed: float = None
    end_yaw_rate: float = None
    source: str = 'legacy'
    time_uncertainty_s: float = 0.0
    sigma_speed_mps: float = 0.0
    epoch: str = ''
    hall_still: bool = None
    speed_basis: str = ''
    sigma_yaw_rate_rps: float = 0.
    end_sigma_speed_mps: float = None


@dataclass(frozen=True)
class GnssFix:
    t: float
    lat_deg: float
    lon_deg: float
    height_m: float
    sol: str
    sigma_n: float
    sigma_e: float
    sigma_u: float
    diff_age_s: float
    sats: int
    position_variance_scale: float = 1.0  # 同历元相对基线可疑时只降位置权重，不否决固定解。
    inertial_only: bool = False         # 圆弧短时失去固定解：仅 IMU/轮速推算，不用浮点位置拉偏轨迹。


@dataclass(frozen=True)
class DualHeading:
    t: float
    heading_deg: float
    pitch_deg: float
    length_m: float
    sol: str
    sigma_heading_deg: float
    sats: int


@dataclass(frozen=True)
class GnssVelocity:
    t: float
    speed_mps: float
    track_deg: float
    sol: str


@dataclass(frozen=True)
class FusionConfig:
    antenna_xy: tuple = (0.345, 0.165)
    # ANT1 相位中心离地高度（车停在平地上量，天线底座到地面再加相位中心高度），m。大于 0 时做倾斜补偿：
    # 按 IMU 的横滚、俯仰把 (antenna_xy, 高度) 这个三维杠杆臂投到水平面，车在横坡上天线偏出去的那段
    # （高 0.4 m、横坡 2° 约 1.4 cm）不再算成车的位置。0 = 不补偿（没量过就不猜）
    antenna_height_m: float = 0.0
    tilt_compensation_enabled: bool = True
    # 横滚或俯仰超过这么多（度）不补偿：划线车不会开这么陡，多半是车被抬起或姿态出错
    max_tilt_deg: float = 15.0
    baseline_yaw_body_deg: float = -90.0
    # 不设基线长度：天线间距不写死、不实测，由 rtk_node 按两根天线固定解的平面实时距离核对测向
    mount_offset_deg: float = 0.0
    wheelbase_m: float = 0.52
    max_steer_rad: float = 0.35
    # 差速车（Gen5）：角速度来自左右轮速差，车速为 0 也能原地转；陀螺合理范围按里程计角速度算，
    # 原地转向不算静止（不拿来估陀螺零偏）。阿克曼车按车速和满舵算（wheelbase_m、max_steer_rad）
    drive_differential: bool = False
    # 两次测向之间转了多少：'rate' 积分陀螺 z 轴角速度（旧车）；'angle' 用 IMU 自己算的航向角的增量
    # （Gen5 的 HDA536T，2026-09-28 用户要求）。两种都只管转了多少，朝哪由双天线定：融合航向 = IMU 航向角 +
    # 双天线不断校准的偏移，零偏状态就是这个偏移的漂移率。
    yaw_input: str = 'rate'
    yaw_resolution_deg: float = 0.01        # IMU 航向角分辨率（HDA536T 0.01°）：相邻两帧的差按它放宽尖峰门限
    # 差速车 + 航向角模式：两帧航向角之差折成的转速在这以内都收，不按里程计卡（原地转最快 0.8 rad/s ≈ 46°/s，再留余量）。
    # 里程计低速时读小：2026-09-30 实车原地转向收尾、低速修方向时 IMU 5–24°/s，里程计只有它的 1/3–1/5，有时还反向；
    # 按里程计卡会把真实转动当尖峰丢掉（一次作业 725 帧丢 201 帧、少算 26°），融合航向转少，正确的测向和固定解
    # 都被当离群值拒掉，最后定位重置、停车。航向角清零 / 传感器重启一帧就是几百 °/s 以上，照样丢。
    # 70 → 150（2026-10-03 实车）：原地转起步驱动器冲过头，指令 0.8 rad/s、两轮读回 1.6 倍，车身实际 81°/s，
    # 而轮速读数还落后 0.1 s 只有 27–40°/s，真实的 IMU 帧按 70°/s 当尖峰丢了 6 帧，航向不确定、预算停在半路
    angle_rate_floor_deg_s: float = 150.0
    # 转过的角度的比例误差（陀螺 / IMU 航向刻度）：转多少，航向不确定度就按这个比例加多少。0 = 不加（旧车）。
    # 不加时原地转 90° 后滤波器自认航向很准，会把比它偏 0.5° 以上的正确测向当离群值一直拒掉
    yaw_scale_sigma: float = 0.0
    # 测向与 IMU 的时间同步误差：测向误差按「角速度 × 这个时间」加大，转得快时少信测向。0 = 不加（旧车）
    heading_sync_s: float = 0.0
    wheel_scale: float = 1.0
    wheel_increment_enabled: bool = False
    wheel_scale_auto: bool = False
    wheel_scale_block_m: float = 8.0
    gyro_noise_deg_s: float = 0.034
    gyro_sample_period_s: float = 0.05
    bias_initial_sigma_deg_s: float = 0.2
    bias_walk_deg_s_sqrt_s: float = 0.001
    yaw_walk_deg_sqrt_s: float = 0.01
    wheel_noise_mps: float = 0.015
    position_walk_m_sqrt_s: float = 0.02
    # 侧向（垂直车头）位置随机游走：车不会横着走，短时间内轮速+陀螺推算的侧向位置很准，RTK 的侧向跳动
    # （实车 2026-09-26：1 s 内侧移 8 cm 又跳回，接收机报的精度一直 1 cm）只慢慢吸收。转弯时轮胎会侧滑，
    # 每 rad/s 角速度再加 cross_walk_turn；沿车头方向仍是 position_walk_m_sqrt_s。
    cross_walk_m_sqrt_s: float = 0.0015
    cross_walk_turn: float = 0.1
    heading_floor_deg: float = 0.1
    fix_floor_m: float = 0.005
    # 固定解观测方差 = max(fix_floor_m, fix_sigma_scale × 接收机报的 σ)²。接收机的 σ（UM982 约 8 mm）是绝对精度，
    # 逐帧抖动实测只有约 1 mm（10-02 直行 0.8 m/s，>2 Hz 抖动 1.06 mm）；按 8 mm 当白噪声处理，横向又只信推算，
    # 融合横向位置要 1 s 多才跟上 RTK，航向有一点误差推算就把位置带偏一截。Gen5 在 line_node 中调小；
    # 只改 RTK 正常时跟得多紧，RTK 断了时推算误差怎么涨（cross_walk 等，推算维持预算按它算）不变
    fix_sigma_scale: float = 1.0
    # 固定解观测 σ 还不小于 fix_jitter_k × 实时估的 RTK 抖动：相邻两帧固定解相对推算的跳动（后一帧新息减前一帧）
    # 按 fix_jitter_tau_s 指数平均。空旷处约 1–2 mm；靠楼多路径时到厘米级，自动少信 RTK、多靠推算，抖完再慢慢恢复。
    # 0 = 不用（旧车）。转动时再加「转速 × 杠杆臂 × heading_sync_s」：测量时刻差一点，天线位置就差这么多
    fix_jitter_k: float = 0.0
    fix_jitter_tau_s: float = 3.0
    float_floor_m: float = 0.15
    innovation_gate_sigma: float = 5.0
    # 固定解位置离群（归一化新息超过这么多 σ）不整个丢掉，按离群程度放大方差：影响有界，
    # 也不会连丢 0.5 s 后一步跳到 RTK 上
    soft_gate_sigma: float = 2.0
    # 固定解连续离群这么久、且差距超过 reset_distance_m：多半是推算错了（车被推动、打滑），照旧对齐到 RTK
    # 并标记重置（控制暂停等操作员）。短时跳动（实车约 1 s）不触发
    outlier_reset_s: float = 2.0
    # 没有固定解时只靠推算维持（2026-09-30 用户：「IMU 航向 5 秒内肯定没问题」）。0 = 旧做法：固定解中断
    # fixed_timeout_s 就不可控，持续离群 outlier_reset_s 后对齐到 RTK 并暂停。> 0：
    # - 没有固定解（掉信号：中转断流、浮点、没解）同样按推算维持最多这么久，超时才不可控、原地停，恢复后按 RTK 对齐
    #   接回（2026-10-01 11 点用户：「我不是让 imu 可以允许维持 5 秒吗」——平板中转常断 0.4–0.8 s，按断 0.5 s 就停，
    #   当天 9 次作业停了 3 次、还因此卡住一次）。当天早上曾改成掉信号断 0.5 s 就原地停：07:42 中转断流 3.85 s，
    #   积压的旧数据到了被当成当前位置把车拉偏 2.5 m；那是时间戳问题（um982.GnssClock 已修），断流期间推算本身没偏。
    #   迟到 history_s 内的数据按测量时刻插回重算，更早的丢掉
    # - 固定解与推算对不上：照旧按软门限用（离得远降权）；持续对不上是推算走偏了，按 RTK 对齐、不停车。
    #   2026-10-05 起双天线位置互检、推算位移互检都不再否决固定解（用户：「原来的检查必须删除」）
    rtk_hold_s: float = 0.0
    # Gen5 单路传感器中断接管。仅替代测量，不替代电机链路的看门狗。
    sensor_failover_enabled: bool = False
    sensor_hold_s: float = 5.0
    sensor_hold_speed_mps: float = 0.3
    gnss_hold_speed_mps: float = 0.35
    wheel_sample_period_s: float = 0.1     # 轮速采样周期 = 1 / 驱动器 tx_hz（2026-10-05 起 10 Hz，原 50 Hz 时 0.02）
    # 卫星地速（多普勒）接管轮速：相邻两帧的速度变化上限、连续有效多久才用
    doppler_accel_limit_mps2: float = 2.0
    doppler_recovery_s: float = 0.2
    doppler_correlation_s: float = 0.05
    # 任务层的喷头推算误差预算；Gen5 在 line_node 中开启，旧车保持原有时限行为。
    # 估计预算 = 协方差的若干 σ + 轮速比例/滑移等模型余量，不是经认证的真实误差上界。
    hold_budget_enabled: bool = False
    hold_watch_s: float = 0.5  # 用户指定：固定解更新间隔达到 500 ms 才进入完整中断预算。
    hold_lateral_limit_m: float = 0.03
    hold_along_limit_m: float = 0.10
    hold_sigma_multiplier: float = 2.0
    hold_wheel_scale_error: float = 0.02
    hold_slip_per_m: float = 0.002
    hold_slip_mps: float = 0.001
    hold_turn_slip_m_per_rad: float = 0.01
    hold_turn_scale_error: float = 0.03
    hold_slow_ratio: float = 0.65
    # 按两根天线位置互检的偏差逐步少信 RTK 位置、多靠 IMU + 轮速：最近 dual_weight_window_s 内互检偏差的最大值 d 超过
    # dual_weight_m 时，固定解位置方差乘 (d / dual_weight_m)²（最多 DUAL_WEIGHT_MAX 倍）。2026-10-01 实车：空旷处偏差
    # p95 1.3–1.4 cm（不受影响），靠楼 p95 5–9 cm，旧做法到 10 cm 才一刀切换推算，3–10 cm 时车跟着卫星晃。0 = 不降权
    dual_weight_m: float = 0.0
    dual_weight_window_s: float = 1.0
    # 推算维持后按 RTK 接回时，位置最快每秒挪这么多（平滑接回，线上不出折角；挪到差 BLEND_DONE_M 以内再照常更新，
    # 不确定度一直照常更新；时限见 BLEND_SPARE）。
    # 2026-10-01 实车靠楼：维持 2.8 s 后一步对齐跳了 8.7 cm。0 = 一步对齐（旧做法）
    rejoin_rate_mps: float = 0.0
    # Gen5：全局滤波状态与连续控制输出分开。未消除的绝对校正作为系统误差交给喷头预算。
    continuous_output: bool = False
    output_position_tau_s: float = 0.25
    output_heading_tau_s: float = 0.20
    output_heading_rate_deg_s: float = 1.0
    # 测向观测存在时间相关性；直行慢校漂移，转弯后及 IMU 失效时及时收敛。
    heading_correlation_s: float = 0.25
    heading_straight_tau_s: float = 2.0
    heading_turn_tau_s: float = 0.15
    heading_turn_rate_deg_s: float = 3.0
    gyro_margin_deg_s: float = 0.5
    stationary_speed_mps: float = 0.002
    stationary_time_s: float = 1.0
    straight_rate_deg_s: float = 0.15
    straight_time_s: float = 0.5
    course_min_speed_mps: float = 0.3
    velocity_noise_mps: float = 0.03
    heading_timeout_s: float = 0.3
    sensor_timeout_s: float = 0.2
    # 轮速超过这么久没更新才算「轮速过期」（wheel_stale，定位不可控、停车）。2026-10-05 用户要求 1 s：CAN 250k 时右轮
    # 反馈会断 0.6 s 左右，驱动器又降到 10 Hz。只管这个判定；推算里旧轮速最多外推、IMU、静止检测等仍按 sensor_timeout_s
    wheel_timeout_s: float = 1.0
    fixed_timeout_s: float = 0.5
    max_diff_age_s: float = 5.0
    min_sats: int = 6
    control_sigma_xy_m: float = 0.03
    control_sigma_yaw_deg: float = 0.5
    reset_distance_m: float = 0.05
    reset_hold_s: float = 0.5
    history_s: float = 1.0
    history_max_samples: int = 256
    max_extrapolation_s: float = 0.2
    calibration_min_distance_m: float = 3.0
    # 没有有效测向时按天线行驶轨迹定车头（_position_course）：直走够这么远才给一次航向。0 = 用 calibration_min_distance_m。
    # init 是航向还没初始化时的第一次，refine 是之后的校正。单天线（2026-10-07 用户：「起步只需要轮子往前动一下就知道车头」）
    # 在 line_node 里给 0.3 / 1.0 m；双天线照旧 3 m（测向断了才用得上）
    course_init_distance_m: float = 0.0
    course_refine_distance_m: float = 0.0
    # 单天线（2026-10-07 用户：「用 imu 的航向角做航向，偏移来修正」）：> 0 时车头不按上面「走直」的轨迹回归定（实车低速车身
    # 一直摆 ±1–4°/s，0.15°/s 的走直门槛过不了），改按 IMU 对齐（_imu_course）：一段的卫星位移和按融合车头（IMU 航向角 +
    # 偏移）+ 轮程推算的位移之间的夹角就是车头偏移的误差。车身摆、转弯、原地转都由 IMU 记着，不要求走直；还没定车头时走
    # course_init_distance_m 定第一次，之后每走这么远修一次。0 = 不用（双天线有测向）
    course_align_distance_m: float = 0.0
    # 判停稳（融合静止检测、任务层 robot_core.standstill）要不要卫星地速也静止：双天线有 BESTVEL 多普勒地速；
    # 单天线没有地速，只看轮子（2026-10-07 用户：「只要轮子不动就行」）
    ground_speed_still: bool = True
    calibration_floor_deg: float = 0.03
    # 在线学双天线测向的固定偏差（安装偏差、多路径带来的慢偏差），从测向里扣掉。Gen5 在 line_node 中开启。
    # 2026-10-02：位置只修位置、不修航向（Schmidt 更新），横向随机游走又很小（防 RTK 横跳），测向偏 b 时推算一直往旁边带、
    # RTK 只能慢慢拉回，融合位置稳定偏向一侧：0.8 m/s 时水泵约偏 1.1 cm/度（全链路仿真，实车「RTK−融合 左向均值」印证）。
    # 学法：同一段路上「轮速 × 融合航向推出来的天线位移」与「RTK 实测的天线位移」方向之差就是测向剩下的偏差
    # （推出来的位移整体转了 b；与车身摆不摆、是不是直线无关，所以不要求严格直行）。天线走够 heading_bias_window_m
    # 比一次，前几段学得快、之后按 heading_bias_gain 累加，单次最多 heading_bias_step_deg、
    # 总共最多 heading_bias_max_deg。融合一直开着（line_node 不重启）就一直记着，下一次作业接着用。
    # 只用固定解、互检通过、没离群、没降权、IMU 能用的段；中间断了、转过 30° 以上就重新开始一段
    heading_bias_learning: bool = False
    heading_bias_window_m: float = 4.0
    heading_bias_gain: float = 0.15
    heading_bias_step_deg: float = 0.2
    heading_bias_max_deg: float = 2.0

    def __post_init__(self):
        if self.yaw_input not in ('rate', 'angle'):
            raise ValueError("yaw_input 必须是 'rate' 或 'angle'")
        for name, value in self.__dict__.items():
            if name == 'yaw_input':
                continue
            if name in ('hold_budget_enabled', 'continuous_output', 'heading_bias_learning', 'wheel_increment_enabled',
                        'wheel_scale_auto', 'sensor_failover_enabled', 'tilt_compensation_enabled', 'ground_speed_still'):
                if type(value) is not bool:
                    raise ValueError(name + ' 必须是布尔值')
                continue
            if name == 'antenna_xy':
                if len(value) != 2 or not all(math.isfinite(v) for v in value):
                    raise ValueError('antenna_xy 必须是两个有限数值')
            elif not math.isfinite(value):
                raise ValueError(name + ' 必须是有限数值')
            elif name in ('yaw_scale_sigma', 'heading_sync_s', 'antenna_height_m', 'rtk_hold_s', 'dual_weight_m', 'fix_jitter_k',
                          'rejoin_rate_mps', 'hold_wheel_scale_error', 'hold_slip_per_m', 'hold_slip_mps',
                          'hold_turn_slip_m_per_rad', 'hold_turn_scale_error',
                          'course_init_distance_m', 'course_refine_distance_m', 'course_align_distance_m'):
                if value < 0:
                    raise ValueError(name + ' 不能为负')
            elif name not in ('baseline_yaw_body_deg', 'mount_offset_deg', 'drive_differential') and value <= 0:
                raise ValueError(name + ' 必须为正')
        if not 0 < self.max_steer_rad < math.pi / 2:
            raise ValueError('max_steer_rad 必须在 (0, pi/2) 内')
        if self.antenna_height_m > 3:
            raise ValueError('antenna_height_m 超过 3 m，单位应是米')
        if self.max_tilt_deg > 45:
            raise ValueError('max_tilt_deg 不能超过 45°')
        if not isinstance(self.history_max_samples, int) or self.history_max_samples < 10:
            raise ValueError('history_max_samples 必须是至少 10 的整数')
        if not 0 < self.hold_slow_ratio < 1:
            raise ValueError('hold_slow_ratio 必须在 0 与 1 之间')
        if max(self.hold_wheel_scale_error, self.hold_turn_scale_error) > 1:
            raise ValueError('推算比例误差必须是 0–1 的比例')
        if self.hold_budget_enabled and self.hold_watch_s > self.fixed_timeout_s:
            raise ValueError('hold_watch_s 不能大于 fixed_timeout_s')


@dataclass(frozen=True)
class FusedPose:
    t: float
    x: float
    y: float
    yaw: float
    speed: float
    yaw_rate: float
    sigma_xy: float
    sigma_yaw: float
    gyro_bias: float
    mount_offset: float
    quality: str
    usable_for_control: bool
    reasons: tuple
    # x、y、yaw 的协方差上三角；由同一历史快照产生，任务层换算到喷头与路线方向。
    covariance_xy_yaw: tuple = ()
    hold_age_s: float = 0.0
    hold_distance_m: float = 0.0
    hold_turn_rad: float = 0.0
    bias_sigma_rad_s: float = 0.0
    correction_xy: tuple = (0.0, 0.0)
    correction_yaw: float = 0.0
    wheel_scale_error: float = None
    wheel_time_uncertainty_s: float = 0.0
    motion_error_m: float = 0.0
    motion_error_mps: float = 0.0
    speed_source: str = 'wheel'
    yaw_source: str = 'imu'
    sensor_hold_age_s: float = 0.0
    prediction_along_walk: float = None
    # None 表示证据缺失或过期，不能判为静止；旧车保留原接口。
    hall_still: bool = None
    gnss_still: bool = None
    motion_evidence: bool = False
    # 停稳要不要卫星地速也静止（FusionConfig.ground_speed_still）；单天线为 False，只看霍尔
    gnss_still_required: bool = True
    # 单天线流动站（FusionConfig.course_align_distance_m > 0）：没有测向、副天线，rtk_trusted 永远不成立，
    # 开喷前检查等按单天线的等价条件判（path_tracker._runup_position_trusted）
    single_antenna: bool = False
    # 卫星可信（见 TRUSTED_FRESH_S）：位置、航向以卫星为准，不报定位错误，任务层不再按推算预算 / 互检停车
    rtk_trusted: bool = False


def _wrap(angle):
    value = (angle + math.pi) % (2 * math.pi) - math.pi
    return math.pi if value == -math.pi else value


_A = 6378137.0
_F = 1 / 298.257223563
_E2 = _F * (2 - _F)
# 航向以双天线为主（2026-09-28 用户：RTK 航向稳定就必须信它，不能因为 IMU 不稳定停车）。
# 差速车 IMU 与轮速里程计转向相反的判据：两边都明显在转、方向相反持续 MISMATCH_S 秒。判出后到重启都不再用 IMU
# （装反、坐标换算错不会自己好），航向交给双天线（两次测向之间用轮速里程计推）；双天线测向有效就照常能开。
# 2026-10-04录包：BESTVEL约5Hz，测量到接收另约20–40ms。
# 停稳地速许可用350ms新鲜窗，避免200ms输入周期的边缘反复清掉300ms停稳计时。
# 这只用于独立静止证据，轮速/IMU失效保护和运动预测的新鲜门限仍由原配置管理。
STATIONARY_DOPPLER_FRESH_S = .35
# 单天线 IMU 对齐（FusionConfig.course_align_distance_m）：已定车头后一段算出的偏移超过这么多度不修（打滑、卫星跳，
# 正常每段只差零点几度）。夹角的不确定度按两端卫星位置的相对误差算：同固定解观测（fix_sigma_scale），接收机报的是
# 绝对精度，一两秒内首尾两点之差小得多；每端至少这么多米（全链路仿真走 0.5 m 实际车头误差中位 0.3°、最差 0.7°，
# 这样给 0.7°）
ALIGN_MAX_DEG = 10.0
ALIGN_FIX_SIGMA_M = 0.003
MISMATCH_WHEEL_RAD_S = 0.2
MISMATCH_IMU_RAD_S = 0.1
MISMATCH_S = 0.2
# 卫星可信（2026-10-05 用户规则：「只要 RTK 满足双固定，测向满足 0.3 到 0.5，就必须不能报任何定位错误」
# 「即使没有轮速和 IMU 的数据，也可以靠 RTK 作业，轮速和 IMU 永远只是辅助」）：
# 主天线固定解（NARROW_INT）、副天线固定解、双天线测向整周固定且基线 0.30–0.50 m，三样都在 TRUSTED_FRESH_S 内来过。
# 这时位置、航向以卫星为准：双天线位置互检、推算位移互检、测向基线带、测向与 IMU 的一致性都不否决卫星，
# 轮速、IMU 断了或对不上也不报定位错误（定位照常可用、等级报 RTK_FIXED）。代价：两根天线一起错（共模跳）抓不住。
# 1.0 s 与 rtk_node 报「测向有效」、line_mission 认副天线固定的时限相同（10 Hz 输出、CAN 偶尔丢帧也不闪）
TRUSTED_FRESH_S = 1.0
TRUSTED_BASELINE_M = (0.30, 0.50)
# 控制输出航向直接对齐（continuous_output）。2026-10-10 用户：「大偏差立即对齐，双天线可信（两个天线都固定、
# 基线 0.35–0.45 m）时，如果车停着、不在喷，或者差值超过 2°，控制用的车头角直接对齐双天线，不再每秒挪 1°」。
# 10-10 实录：开始作业时控制输出的航向比融合差 58°（融合早已跟上双天线），按 output_heading_rate_deg_s 每秒追 1°
# 追了一分多钟：第一趟原地转错方向、偏离中止，第二趟线头偏 23 cm。
# 卫星可信（见上）且最近一帧测向基线在 OUTPUT_SNAP_BASELINE_M 内时：车停着、不在喷（任务层 set_painting），或输出与
# 融合航向差超过 OUTPUT_SNAP_DEG，输出航向直接等于融合航向（以双天线为准，不带单帧测向的噪声）；
# 只有在喷且差不到 OUTPUT_SNAP_DEG 时仍按 output_heading_rate_deg_s 平滑（线上不出折角）
OUTPUT_SNAP_BASELINE_M = (0.35, 0.45)
OUTPUT_SNAP_DEG = 2.0
# 可信时 pose() 最多往后外推这么久（轮速、IMU 都断了时滤波器只靠 10 Hz 的卫星往前走，加上转发延迟常超过 0.2 s）
TRUSTED_EXTRAPOLATION_S = 0.5
# 旧阿克曼兼容恢复；Gen5 用下方多源质量确认后平滑接回，不直接重定航向。
REANCHOR_S = 0.5
REANCHOR_SPREAD_DEG = 0.5
REANCHOR_RATE_DEG_S = 3.0
# 测向和推算差得多、按测向快校之后，这么久内仍按转弯的时间常数快校
HEADING_RECOVERY_S = 1.0
# 测向偏差学习：一段里转过这么多（度）就重新开始（转弯时轮速刻度误差会带进方向）；推算 / RTK 位移长度比在这范围外不用
HEADING_BIAS_TURN_DEG = 30.0
HEADING_BIAS_RATIO = (0.8, 1.25)
HEADING_BIAS_FAST_N = 6
HEADING_BIAS_FAST_GAIN = 0.5
# 双天线测向有效（刚收下过）时，航向不确定度到这么多都照常能开：这时的不确定度多半是原地转向、IMU 断了时按保守
# 模型涨的，航向有测向拉着（与 line_mission 的 GYRO_HOLD_SIGMA_YAW_DEG 相同）
HEADING_HOLD_SIGMA_DEG = 2.0
# 互检偏差给 RTK 位置降权最多这么多倍（方差；标准差 10 倍）：再大就是判卫星错了、按推算维持那一档
DUAL_WEIGHT_MAX = 100.0
# 平滑接回挪到离 RTK 这么近就交回普通更新（剩下的按普通更新慢慢吸收）
BLEND_DONE_M = 0.01
# 接回时离 RTK 超过这么远不平滑，一步对齐：平滑只为推算维持几秒后的几厘米误差不在线上留折角（实车靠楼 8.7 cm）；
# 差得远是车在没有可用 RTK 时被挪过（室内转发器下开出门、轮子没电推车），或推算已经失效。2026-10-01 13:40 实车：
# 从室内开出来差几十米，按 4 cm/s 慢慢挪，十几分钟里融合位置都偏 20–100 m，车每遍跑在不同的地方
BLEND_MAX_M = 0.3
# 平滑接回最多用 BLEND_SPARE × 开始时的差距 / rejoin_rate_mps + 1 s；到时还没挪到：越离越远（推算一直在跑偏）就
# 一步对齐，挪近了只是挪不进（RTK 自己在跳）就交回普通更新
BLEND_SPARE = 1.5
_FIXED = frozenset(('NARROW_INT',))
_FLOAT = frozenset(('NARROW_FLOAT', 'L1_FLOAT', 'IONOFREE_FLOAT'))
_HEADING = frozenset(('L1_INT', 'NARROW_INT', 'WIDE_INT'))
_VELOCITY = _FIXED | _FLOAT | _HEADING | frozenset(
    ('DOPPLER_VELOCITY', 'SINGLE', 'PSRDIFF', 'SOL_COMPUTED'))


def _ecef(lat, lon, height):
    phi, lam = math.radians(lat), math.radians(lon)
    sp, cp = math.sin(phi), math.cos(phi)
    n = _A / math.sqrt(1 - _E2 * sp * sp)
    return ((n + height) * cp * math.cos(lam),
            (n + height) * cp * math.sin(lam),
            (n * (1 - _E2) + height) * sp)


def _geodetic(x, y, z):
    p = math.hypot(x, y)
    phi = math.atan2(z, p * (1 - _E2))
    for _ in range(8):
        sp = math.sin(phi)
        n = _A / math.sqrt(1 - _E2 * sp * sp)
        nxt = math.atan2(z + _E2 * n * sp, p)
        if abs(nxt - phi) < 1e-15:
            phi = nxt
            break
        phi = nxt
    sp, cp = math.sin(phi), math.cos(phi)
    height = p * cp + z * sp - _A * math.sqrt(1 - _E2 * sp * sp)
    return math.degrees(phi), math.degrees(math.atan2(y, x)), height


class _Frame:
    def __init__(self, lat, lon, height):
        if not all(math.isfinite(v) for v in (lat, lon, height)) or not -90 <= lat <= 90 or not -180 <= lon <= 180:
            raise ValueError('原点经纬度或高程无效')
        self.height = height
        self.xyz = _ecef(lat, lon, height)
        phi, lam = math.radians(lat), math.radians(lon)
        sp, cp, sl, cl = math.sin(phi), math.cos(phi), math.sin(lam), math.cos(lam)
        self.east = (-sl, cl, 0.0)
        self.north = (-sp * cl, -sp * sl, cp)
        self.up = (cp * cl, cp * sl, sp)

    def local(self, lat, lon):
        xyz = _ecef(lat, lon, self.height)
        delta = tuple(a - b for a, b in zip(xyz, self.xyz))
        return (sum(a * b for a, b in zip(delta, self.east)),
                sum(a * b for a, b in zip(delta, self.north)))

    def geodetic(self, east, north):
        # 二维接口没有 U；沿原点法线求与同椭球高曲面的交点，不能直接令 U=0。
        base = tuple(o + east * e + north * n for o, e, n in zip(self.xyz, self.east, self.north))
        up = -(east * east + north * north) / (2 * _A)
        for _ in range(5):
            lat, lon, height = _geodetic(*(b + up * u for b, u in zip(base, self.up)))
            delta = height - self.height
            if abs(delta) < 1e-8:
                break
            up -= delta
        return lat, lon


class DualAntennaCheck:
    """主、副天线固定解位置之差与测向基线（长度 × 方向）的偏差，只用来给 RTK 位置降权（FusionConfig.dual_weight_m），
    不判卫星对错、不丢帧、不停车（2026-10-05 用户：「原来的检查必须删除，这种检查是不合理的」）。
    副天线每来一帧，把主天线位置按时间插值到同一时刻再比。按到达顺序处理；融合在主天线固定解到达时读
    recent_dev，把结果跟样本一起存下，历史重放时不变。"""

    def __init__(self, config):
        self.cfg = config
        self._primary = []          # (t, x, y)，最近 2 s
        self._secondary = []        # 还没比过的副天线 (t, x, y)
        self._heads = []            # (t, 北零顺时针 rad, 水平基线长 m)，最近 2 s
        self.last_dev = None        # 最近一次比出的偏差，m
        self.last_t = None          # 最近一次比的时刻
        self.checks = 0
        self._devs = []             # (t, 偏差 m)，最近 5 s 每次比的结果

    def add_heading(self, t, heading_deg, length_m, pitch_deg):
        if not all(math.isfinite(v) for v in (t, heading_deg, length_m)) or length_m <= 0:
            return
        pitch = math.radians(pitch_deg) if math.isfinite(pitch_deg) else 0.0
        self._heads.append((t, math.radians(heading_deg), length_m * math.cos(pitch)))
        self._trim(self._heads, t)

    def add_primary(self, t, x, y):
        self._primary.append((t, x, y))
        self._trim(self._primary, t)
        self._evaluate()

    def add_secondary(self, t, x, y):
        self._secondary.append((t, x, y))
        del self._secondary[:-20]
        self._evaluate()

    @staticmethod
    def _trim(rows, t):
        while rows and rows[0][0] < t - 2.0:
            rows.pop(0)

    def _evaluate(self):
        keep = []
        for t, x2, y2 in self._secondary:
            p = self._primary_at(t)
            if p is None:
                if self._primary and self._primary[-1][0] < t:
                    keep.append((t, x2, y2))            # 主天线这个时刻之后的还没来，等一下
                continue
            h = min(self._heads, key=lambda r: abs(r[0] - t), default=None)
            if h is None or abs(h[0] - t) > 0.1:
                continue
            ex, ey = h[2] * math.sin(h[1]), h[2] * math.cos(h[1])          # 测向基线：主 → 副，东、北
            dev = math.hypot(x2 - p[0] - ex, y2 - p[1] - ey)
            self.last_dev, self.last_t = dev, t
            self._devs.append((t, dev))
            while self._devs and self._devs[0][0] < t - 5.0:
                self._devs.pop(0)
            self.checks += 1
        self._secondary = keep

    def _primary_at(self, t):
        rows = self._primary
        for a, b in zip(rows, rows[1:]):
            if a[0] <= t <= b[0]:
                if b[0] - a[0] > 0.2:
                    return None
                u = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
                return a[1] + u * (b[1] - a[1]), a[2] + u * (b[2] - a[2])
        return None

    def recent_dev(self, t, window):
        """t 前 window 秒内互检偏差的最大值（m）；这段时间没比过为 None。"""
        devs = [d for tt, d in self._devs if t - window <= tt <= t + 1e-9]
        return max(devs) if devs else None

    def status(self):
        return {'dev_m': None if self.last_dev is None else round(self.last_dev, 4), 'checks': self.checks}


class _State:
    def __init__(self, config):
        self.t = None
        self.x = self.y = self.yaw = self.bias = 0.0
        # 对称协方差上三角：xx, xy, xθ, xb, yy, yθ, yb, θθ, θb, bb。
        self.p = (1., 0., 0., 0., 1., 0., 0., math.pi ** 2, 0.,
                  math.radians(config.bias_initial_sigma_deg_s) ** 2)
        self.position_ready = self.yaw_ready = False
        self.speed = self.gyro = self.wheel_yaw = self.odo = 0.0
        self.imu_t = self.wheel_t = self.fixed_t = self.fix_t = -math.inf
        self.imu_valid_t = -math.inf  # 尖峰后的角度参考仍更新，但不能把坏帧当成新鲜可用的运动输入。
        self.heading_t = self.velocity_t = self.reset_t = -math.inf
        self.heading_ok = False
        self.raw_yaw = 0.0
        self.raw_heading_var = 1.0
        self.position_quality = 'INVALID'
        self.fix_reason = 'position_uninitialized'
        self.stationary_since = self.straight_since = None
        self.motion_check_rows = ()
        self.velocity_motion_since = None
        self.wheel_rate_sigma = 0.
        self.hall_still = None
        self.motion_evidence = False
        self.ground_speed = None
        self.ground_t = -math.inf
        self.segment = None
        self.mount_sum_c = self.mount_sum_s = self.mount_weight = 0.0
        self.scale_sum = self.scale_weight = 0.0
        self.cal_samples = self.cal_scale_samples = self.cal_segments = 0
        self.vel_n = 0
        self.vel_c = self.vel_s = self.vel_var_sum = 0.0
        self.heading_rejected = self.fix_rejected = self.gyro_rejected = self.resets = 0
        self.outlier_since = None
        self.course_t = -math.inf
        # 单天线 IMU 对齐（_imu_course）的这一段：(卫星 x, y, 杠杆臂 x, y, 轮程, 车头, 推算 x, y, 走过的路程)；
        # 元组，历史快照回放时跟着恢复
        self.align = None
        self.imu_yaw = None                 # 上一帧 IMU 航向角（yaw_input = 'angle'）
        self.mismatch_since = None          # 差速车：IMU 转向与轮速里程计相反从什么时候开始
        self.imu_mismatch = False           # 确认相反后一直为真，IMU 不再用
        self.rej_since = None               # 测向连续被拒从什么时候开始；rej_ref 第一帧，rej_lo/hi 之后相对它的范围
        self.rej_ref = self.rej_lo = self.rej_hi = 0.0
        self.heading_reset_t = -math.inf
        self.heading_resets = 0
        self.roll = self.pitch = 0.0        # 最近一帧 IMU 姿态（倾斜补偿），tilt_t 是它的时刻
        self.tilt_t = -math.inf
        # 推算维持（rtk_hold_s > 0）：最近一次按 RTK 对齐接回的时刻、次数
        self.rejoin_t = -math.inf
        self.rejoins = 0
        # 平滑接回中（rejoin_rate_mps > 0）；blend_t 上一步的时刻，blend_until 时限，blend_d0 开始时离 RTK 多远
        self.blending = False
        self.blend_t = self.blend_until = -math.inf
        self.blend_d0 = 0.0
        self.weak_t = -math.inf             # 最近一帧按互检偏差降了权的固定解的时刻
        self.position_variance_scale = 1.0
        self.hold_distance = self.hold_turn = 0.0  # 最近一次接受固定解之后，绝对路程及绝对转角（倒车不抵消）。
        self.output_ready = False
        self.output_rejoining = False
        self.output_x = self.output_y = self.output_yaw = 0.0
        self.predicted_turn = self.scale_turn = 0.0
        self.last_turn_t = -math.inf
        self.heading_sample_t = -math.inf   # 上一帧测向的时刻（增益按帧间隔限幅）
        self.heading_recovery_t = -math.inf # 最近一次按测向快校的时刻
        self.heading_innovation = self.heading_test_ratio = 0.0
        self.heading_gain = 0.0
        self.raw_gyro_bias = 0.0
        self.raw_bias_samples = 0
        # 测向偏差在线学习（heading_bias_learning）：dr_x/dr_y 纯推算的后轴位移累计（轮速 × 融合航向，不被 RTK 修正）；
        # heading_bias 学到的测向偏差（测向减去它）；bias_window 这一段的起点
        # (RTK 天线 x, y, 推算天线 x, y, 最近一帧时刻, 起点累计转角)
        self.dr_x = self.dr_y = 0.0
        # 固定解抖动估计（fix_jitter_k）：上一帧固定解的新息 (时刻, 东, 北)，相邻差平方的指数平均（m²）
        self.fix_innov = None
        self.fix_jitter = 0.004 ** 2
        self.heading_bias = 0.0
        self.heading_bias_updates = 0
        self.bias_window = None
        self.wheel_end = None
        self.wheel_source, self.wheel_epoch = 'legacy', ''
        self.wheel_time_uncertainty = self.wheel_sigma = 0.0
        self.wheel_cal = WheelScale(config.wheel_scale, config.hold_wheel_scale_error)
        self.motion_since = None
        self.motion_conflict_t = -math.inf
        self.motion_error = self.motion_error_rate = 0.0
        self.doppler_t = -math.inf
        self.doppler_speed = 0.0
        self.doppler_vector = None
        self.doppler_valid_since = None
        self.doppler_rejected = 0
        # 卫星可信（见 TRUSTED_FRESH_S）：最近处理的那帧卫星（固定解或测向）是否可信、最近一帧可信的时刻
        self.trusted = False
        self.trusted_t = -math.inf
        # 最近一帧有效测向基线在 OUTPUT_SNAP_BASELINE_M 内时是它的时刻，否则 -inf（控制输出航向直接对齐用）
        self.snap_heading_t = -math.inf
        # 卫星可信、IMU 断着时航向直接取测向；sat_rate 是相邻两帧测向算的转速（rad/s），两帧之间按它推
        self.sat_yaw = 0.0
        self.sat_yaw_t = self.sat_rate_t = -math.inf
        self.sat_rate = 0.0


class GnssFusion:
    def __init__(self, config: FusionConfig = FusionConfig()):
        self.config = config
        self._tilt_zero = (0.0, 0.0)  # 仅本进程有效，不写入标定配置
        self._tilt_zeroed = False
        self._frame = None
        self._s = _State(config)
        self._keys, self._events, self._snapshots = [], [], []
        self._base = self._s.__dict__.copy()
        self._base_key = (-math.inf, -1)
        self._last = [-math.inf] * 5
        self._dropped = [0] * 5
        self._wheel_interval_end = -math.inf
        self._reset_seen = self._reset_visible_until = -math.inf
        self._mount = math.radians(config.mount_offset_deg)
        self._baseline = math.radians(config.baseline_yaw_body_deg)
        self._gyro_q = (math.radians(config.gyro_noise_deg_s) ** 2 * config.gyro_sample_period_s
                        + math.radians(config.yaw_walk_deg_sqrt_s) ** 2)
        self._bias_q = math.radians(config.bias_walk_deg_s_sqrt_s) ** 2
        self._dual = DualAntennaCheck(config)
        # 到达顺序上最近一帧主天线 (t, 解)、副天线 (t, 解)、测向 (t, 解, 基线 m)：判卫星可信用（见 TRUSTED_FRESH_S），
        # 结果跟样本一起存下，历史重放时不变
        self._seen_fix = self._seen_fix2 = self._seen_heading = None
        # 任务层每拍告诉融合现在是否在喷（set_painting）：不在喷时控制输出航向直接对齐（见 OUTPUT_SNAP_BASELINE_M）
        self.painting = False

    def set_painting(self, painting):
        self.painting = bool(painting)

    def switch_config(self, config: FusionConfig):
        """换一套配置（单天线 / 双天线：天线杠杆臂差 0.41 m）：滤波状态、历史、标定统计全部重来，本地坐标原点和
        IMU 倾斜零点留着（路线、断点按原点存）。换之前的迟到样本不再回放。只在车停着、没有作业时调用。"""
        frame, tilt, t = self._frame, (self._tilt_zero, self._tilt_zeroed), self._s.t
        self.__init__(config)
        self._frame = frame
        self._tilt_zero, self._tilt_zeroed = tilt
        if t is not None:
            self._base_key = (t, 4)

    def _trust_at(self, t):
        """t 时刻卫星是否可信：主、副天线都是固定解，测向整周固定且基线在 TRUSTED_BASELINE_M 内，都在 TRUSTED_FRESH_S 内。"""
        def fresh(row):
            return row is not None and abs(t - row[0]) <= TRUSTED_FRESH_S
        fix, fix2, head = self._seen_fix, self._seen_fix2, self._seen_heading
        return (fresh(fix) and fix[1] in _FIXED and fresh(fix2) and fix2[1] in _FIXED
                and fresh(head) and head[1] in _HEADING
                and TRUSTED_BASELINE_M[0] <= head[2] <= TRUSTED_BASELINE_M[1])

    def _trusted(self):
        """现在（滤波器当前时刻）卫星是否可信：最近处理的那帧卫星可信，且可信的那帧不超过 TRUSTED_FRESH_S。"""
        s = self._s
        return s.trusted and s.t is not None and s.t - s.trusted_t <= TRUSTED_FRESH_S + 1e-9

    def _mark_trust(self, t, trusted):
        s = self._s
        s.trusted = bool(trusted)
        if trusted:
            s.trusted_t = t

    def set_origin(self, lat_deg, lon_deg, height_m=0.0):
        frame = _Frame(lat_deg, lon_deg, height_m)
        if self._keys or self._s.position_ready:
            raise ValueError('已有观测后不能更换原点；请新建 GnssFusion')
        self._frame = frame

    def reset_imu_source(self):
        """停车换 IMU 时重建推算状态，保留配置和坐标原点；不带入旧源零偏或迟到观测。"""
        frame, wheel_cal = self._frame, self._s.wheel_cal.clear()
        self.__init__(self.config)
        self._frame = frame
        # 轮程比例是底盘标定，换 IMU 不重置；只丢弃尚未完成的直行标定段。
        self._s.wheel_cal = wheel_cal
        self._base = self._s.__dict__.copy()

    def to_local(self, lat_deg, lon_deg):
        if self._frame is None:
            raise ValueError('尚未设置原点')
        if not all(math.isfinite(v) for v in (lat_deg, lon_deg)) or not -90 <= lat_deg <= 90 or not -180 <= lon_deg <= 180:
            raise ValueError('经纬度无效')
        return self._frame.local(lat_deg, lon_deg)

    def to_geodetic(self, x, y):
        if self._frame is None:
            raise ValueError('尚未设置原点')
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError('本地坐标无效')
        return self._frame.geodetic(x, y)

    def add_wheel(self, s: WheelSample):
        values = (s.t, s.speed, s.yaw_rate, s.time_uncertainty_s, s.sigma_speed_mps,s.sigma_yaw_rate_rps)
        interval = s.end_t is not None
        if interval:
            values += (s.end_t,s.end_speed,s.end_yaw_rate)
            if s.end_sigma_speed_mps is not None:values += (s.end_sigma_speed_mps,)
        if (not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)
                or min(s.time_uncertainty_s,s.sigma_speed_mps,s.sigma_yaw_rate_rps) < 0
                or s.source not in ('legacy','speed','encoder')
                or s.speed_basis not in ('','hall','readback_fallback')
                or (s.hall_still is not None and type(s.hall_still) is not bool)
                or (interval and (s.source != 'encoder' or not 0 < s.end_t-s.t <= self.config.sensor_timeout_s))
                or (s.end_sigma_speed_mps is not None and s.end_sigma_speed_mps < 0)
                or s.t < self._wheel_interval_end-1e-9):
            self._dropped[0] += 1
            return False
        accepted = self._add(0, s, replace_wheel=interval)
        if accepted and interval:
            self._wheel_interval_end = s.end_t
        return accepted

    def add_imu(self, s: ImuSample):
        if math.isfinite(s.t) and math.isfinite(s.gyro_z):
            self._add(1, s)

    def add_heading(self, s: DualHeading):
        if math.isfinite(s.t):
            # 拒绝重复/过旧测向之后再维护互检，避免坏样本改变未来质量判断。
            if (s.t <= self._last[2] or (s.t, 2) <= self._base_key
                    or (self._s.t is not None and s.t < self._s.t - self.config.history_s)):
                self._dropped[2] += 1
                return
            if s.sol in _HEADING:
                self._dual.add_heading(s.t, s.heading_deg, s.length_m, s.pitch_deg)
            if math.isfinite(s.length_m):
                self._seen_heading = (s.t, s.sol, s.length_m)
            else:
                self._seen_heading = (s.t, 'NONE', 0.0)
            self._add(2, s, self._trust_at(s.t))     # 卫星是否可信（到达时的）跟样本一起存下

    def add_fix2(self, s: GnssFix):
        """副天线定位（UM982 BESTNAVHA）：只用来和主天线互检，不直接进融合；解的档次记下来判卫星可信。"""
        if math.isfinite(s.t) and (self._seen_fix2 is None or s.t >= self._seen_fix2[0]):
            ok = all(math.isfinite(v) for v in (s.lat_deg, s.lon_deg))
            self._seen_fix2 = (s.t, s.sol if ok else 'NONE')
        if (self._frame is None or not all(math.isfinite(v) for v in (s.t, s.lat_deg, s.lon_deg))
                or s.sol not in _FIXED):
            return
        self._dual.add_secondary(s.t, *self._frame.local(s.lat_deg, s.lon_deg))

    def add_fix(self, s: GnssFix):
        if not math.isfinite(s.t):
            return
        valid = (s.sol in _FIXED | _FLOAT
                 and all(math.isfinite(v) for v in (s.lat_deg, s.lon_deg, s.height_m, s.sigma_n, s.sigma_e))
                 and -90 <= s.lat_deg <= 90 and -180 <= s.lon_deg <= 180
                 and s.sigma_n > 0 and s.sigma_e > 0
                 and 0 <= s.diff_age_s <= self.config.max_diff_age_s
                 and s.sats >= self.config.min_sats)
        # 不让迟到或重复样本偷偷设置原点。
        if (s.t <= self._last[3] or (s.t, 3) <= self._base_key
                or (self._s.t is not None and s.t < self._s.t - self.config.history_s)):
            self._dropped[3] += 1
            return
        self._seen_fix = (s.t, s.sol if valid else 'NONE')
        if self._frame is None and valid:
            self._frame = _Frame(s.lat_deg, s.lon_deg, s.height_m)
        xy = self._frame.local(s.lat_deg, s.lon_deg) if valid else None
        if xy is not None and s.sol in _FIXED:
            # 互检偏差、卫星是否可信（到达时的）跟样本一起存下，历史重放时不变
            self._dual.add_primary(s.t, *xy)
            cfg = self.config
            xy = (xy[0], xy[1], self._dual.recent_dev(s.t, cfg.dual_weight_window_s) if cfg.dual_weight_m > 0 else None,
                  self._trust_at(s.t))
        self._add(3, s, xy)

    def add_velocity(self, s: GnssVelocity):
        if math.isfinite(s.t):
            self._add(4, s)

    def _restore(self, snapshot):
        self._s = _State.__new__(_State)
        self._s.__dict__ = snapshot.copy()

    def _add(self, kind, sample, xy=None, replace_wheel=False):
        key = (sample.t, kind)
        existing = bisect.bisect_left(self._keys, key)
        refine = (replace_wheel and sample.t == self._last[kind]
                  and existing < len(self._keys) and self._keys[existing] == key
                  and self._events[existing][1].end_t is None
                  and self._events[existing][1].epoch == sample.epoch)
        if (sample.t <= self._last[kind] and not refine) or key <= self._base_key:
            self._dropped[kind] += 1
            return False
        if self._s.t is not None and sample.t < self._s.t - self.config.history_s:
            self._dropped[kind] += 1
            return False
        self._last[kind] = sample.t
        if refine:
            del self._keys[existing], self._events[existing], self._snapshots[existing]
        index = bisect.bisect_right(self._keys, key)
        self._keys.insert(index, key)
        self._events.insert(index, (kind, sample, xy))
        self._snapshots.insert(index, None)
        if kind in (0, 1):
            # 新陀螺补齐前一采样间隔，用线性插值积分；转弯起止不能滞后一整帧。
            for j in range(index-1, -1, -1):
                if self._events[j][0] == 1:
                    index = j+1
                    break
        if refine or index < len(self._keys) - 1:
            self._restore(self._snapshots[index - 1] if index else self._base)
        next_imus = [(i, event[1]) for i, event in enumerate(self._events[index:], index) if event[0] == 1]
        next_index = 0
        for i in range(index, len(self._keys)):
            k, s, pos = self._events[i]
            while next_index < len(next_imus) and next_imus[next_index][0] < i:
                next_index += 1
            future = next_imus[next_index] if next_index < len(next_imus) else None
            future_gyro = None
            if future is not None:
                fi, fs = future
                wheel = None
                for j in range(fi-1, -1, -1):
                    if self._events[j][0] == 0:
                        wheel = self._events[j][1]
                        break
                if wheel is not None and fs.t-(wheel.end_t or wheel.t) <= self.config.sensor_timeout_s:
                    bound = self._gyro_bound(wheel.speed, wheel.yaw_rate, fs.t - self._s.imu_t)
                    if abs(self._yaw_rate(fs)-self._s.bias) <= bound:
                        future_gyro = fs
            self._predict(s.t, future_gyro)
            if k == 0:
                self._wheel(s)
            elif k == 1:
                self._imu(s)
            elif k == 2:
                self._heading(s, pos)
                self._snap_output()
            elif k == 3:
                self._fix(s, pos)
            else:
                self._velocity(s)
            self._initialize_output()
            self._snapshots[i] = self._s.__dict__.copy()
        if self._s.reset_t > self._reset_seen:
            # 很晚才收到的重置也必须给当前控制周期留下可见的暂停窗口。
            self._reset_seen = self._s.reset_t
            self._reset_visible_until = self._s.t + self.config.reset_hold_s
        n = max(bisect.bisect_left(self._keys, (self._s.t - self.config.history_s, -1)),
                len(self._keys) - self.config.history_max_samples)
        if n > 0:
            self._base = self._snapshots[n - 1]
            self._base_key = self._keys[n - 1]
            del self._keys[:n]
            del self._events[:n]
            del self._snapshots[:n]
        return True

    def _wheel_replacement(self):
        s, cfg = self._s, self.config
        if cfg.sensor_failover_enabled and self._trusted():
            # 卫星可信：轮速断了就用卫星地速（多普勒）接着推，不要求 IMU、不限时（轮速、IMU 永远只是辅助）
            return (s.t-s.wheel_t > cfg.sensor_timeout_s
                    and s.t-s.doppler_t <= cfg.sensor_timeout_s
                    and s.doppler_valid_since is not None
                    and s.t-s.doppler_valid_since >= cfg.doppler_recovery_s)
        return (cfg.sensor_failover_enabled and math.isfinite(s.wheel_t)
                and cfg.sensor_timeout_s < s.t-s.wheel_t <= cfg.sensor_hold_s
                and s.t-s.doppler_t <= cfg.sensor_timeout_s
                and s.doppler_valid_since is not None
                and s.t-s.doppler_valid_since >= cfg.doppler_recovery_s
                and s.t-s.fixed_t < cfg.hold_watch_s
                and s.t-s.imu_valid_t <= cfg.sensor_timeout_s and not s.imu_mismatch
                and self._has_heading()
                and s.t-s.motion_conflict_t > 1.0)

    def _predict(self, t, future_gyro=None):
        s, cfg = self._s, self.config
        if s.t is None:
            s.t = t
            return
        if s.wheel_end is not None and t >= s.wheel_end[0]:
            end,speed,rate,sigma = s.wheel_end
            s.wheel_end = None
            self._predict(end, future_gyro)
            s.speed,s.wheel_yaw,s.wheel_sigma = speed,rate,sigma
            self._predict(t, future_gyro)
            return
        dt = t - s.t
        if dt <= 0:
            return
        # 输入中断后不无限外推旧油门或旧角速度；不确定性仍随整个 dt 增长。
        dv = min(dt, max(0., s.wheel_t + cfg.sensor_timeout_s - s.t))
        wheel_dv = dv
        replacement = self._wheel_replacement()
        if replacement and self._trusted():
            dv = min(dt, max(0., s.doppler_t+cfg.sensor_timeout_s-s.t))
        elif replacement:
            dv = min(dt, max(0., min(s.doppler_t+cfg.sensor_timeout_s,
                                    s.wheel_t+cfg.sensor_hold_s, s.fixed_t+cfg.hold_watch_s)-s.t))
        speed = s.doppler_speed if replacement else s.speed*s.wheel_cal.applied
        imu_ok = not s.imu_mismatch
        dg = min(dt, max(0., s.imu_valid_t + cfg.sensor_timeout_s - s.t)) if imu_ok else 0.
        w = s.gyro - s.bias
        stationary = self._stationary()
        if not imu_ok:
            pass
        elif cfg.yaw_input == 'angle':
            # 下一帧 IMU 航向角已到：这一段按两帧航向角之差匀速转过去，积分正好等于 IMU 航向角的变化
            if future_gyro is not None and s.imu_t > -math.inf and future_gyro.t > s.imu_t:
                w = self._yaw_rate(future_gyro) - s.bias
        elif future_gyro is not None and s.imu_t > -math.inf and future_gyro.t > s.imu_t and not stationary:
            fraction = (s.t+dg/2-s.imu_t)/(future_gyro.t-s.imu_t)
            w += (future_gyro.gyro_z-s.gyro)*max(0., min(1., fraction))
        angle = w * dg
        # IMU 断了或不再用：差速车用轮速里程计的角速度接着推（原地转时也有），航向照样由双天线定；阿克曼车不推
        do = max(0., wheel_dv - dg) if cfg.drive_differential else 0.
        angle += s.wheel_yaw * do
        if dg < dt and self._trusted():
            # 卫星可信、IMU 断着：IMU 和轮速里程计都没管到的那段按相邻测向算的转速推（最多推到最近测向后 heading_timeout_s）
            angle += s.sat_rate * max(0., min(dt, s.sat_rate_t + cfg.heading_timeout_s - s.t) - dg - do)
        half = angle / 2
        distance = speed * dv
        sinc = math.sin(half) / half if abs(half) > 1e-8 else 1.0
        mid = s.yaw + half
        dx, dy = distance * sinc * math.cos(mid), distance * sinc * math.sin(mid)
        s.x += dx
        s.y += dy
        s.dr_x += dx
        s.dr_y += dy
        s.yaw = _wrap(s.yaw + angle)
        s.odo += (speed / s.wheel_cal.applied) * dv
        s.hold_distance += abs(distance)
        s.hold_turn += abs(angle)
        s.motion_error += s.motion_error_rate * dt
        s.predicted_turn += angle
        if abs(angle) / dt > math.radians(cfg.heading_turn_rate_deg_s):
            s.last_turn_t = t
        a, c, e = -dy, dx, -dg
        b, d = a * e / 2, c * e / 2
        xx, xy, xh, xb, yy, yh, yb, hh, hb, bb = s.p
        q = cfg.position_walk_m_sqrt_s ** 2 * dt
        # 侧向随机游走只在轮速、陀螺都没中断时收紧；中断时推算本身不可信，退回各向同性
        cross = cfg.cross_walk_m_sqrt_s + cfg.cross_walk_turn * abs(angle) / dt
        qc = q if min(dv, dg) < dt else min(q, cross ** 2 * dt)
        qv = (max(cfg.wheel_noise_mps, s.wheel_sigma) * dv) ** 2
        if cfg.sensor_failover_enabled and s.wheel_source == 'encoder' and min(wheel_dv, dg) >= dt:
            # 区间编码器给出已走距离及其误差；不再额外叠加“未知速度积分”的各向同性位置随机游走。
            # 轮径比例与滑移仍由任务预算单独累计，转向误差仍由协方差传播。
            if s.t-s.fixed_t >= cfg.hold_watch_s:
                qv = max(cfg.wheel_noise_mps, s.wheel_sigma)**2 * cfg.wheel_sample_period_s * dv
                # 侧向的轮胎滑移已有任务预算按路程/转角累计；此处仅传播随机残差。
                # 航向误差通过 P 的位置-航向交叉项进入横向，不再用转速重复放大位置白噪声。
                qc = min(q, cfg.cross_walk_m_sqrt_s**2 * dt)
                q = max(qc, qv)
                qv = 0.0
        elif replacement:
            qv = cfg.velocity_noise_mps**2 * .05 * dv
        cs, sn = math.cos(mid), math.sin(mid)
        qa = q - qc + qv                  # 沿车头方向比侧向多出的部分
        # 刻度误差在整段转角上相关，不能因同一运动被更多事件切分而消失。
        next_scale_turn = s.scale_turn + abs(angle)
        qh = self._gyro_q * dt + cfg.yaw_scale_sigma ** 2 * (next_scale_turn ** 2 - s.scale_turn ** 2)
        s.scale_turn = next_scale_turn
        if dg < dt:
            # 没有 IMU 的那段：里程计推的按 1°/√s 再加转角的 10%（原地转时万向轮打滑），什么都没有按 5°/√s
            qh += (math.radians(1.) ** 2 * do + (0.1 * s.wheel_yaw * do) ** 2
                   + math.radians(5.) ** 2 * max(0., dt - dg - do))
        s.p = (
            xx + 2*a*xh + 2*b*xb + a*a*hh + 2*a*b*hb + b*b*bb + qc + qa*cs*cs,
            xy + c*xh + d*xb + a*yh + b*yb + a*c*hh + (a*d+b*c)*hb + b*d*bb + qa*cs*sn,
            xh + e*xb + a*hh + (a*e+b)*hb + b*e*bb,
            xb + a*hb + b*bb,
            yy + 2*c*yh + 2*d*yb + c*c*hh + 2*c*d*hb + d*d*bb + qc + qa*sn*sn,
            yh + e*yb + c*hh + (c*e+d)*hb + d*e*bb,
            yb + c*hb + d*bb,
            hh + 2*e*hb + e*e*bb + qh,
            hb + e*bb,
            bb + self._bias_q * dt)
        if cfg.continuous_output and s.output_ready:
            out_mid = s.output_yaw + half
            s.output_x += distance * sinc * math.cos(out_mid)
            s.output_y += distance * sinc * math.sin(out_mid)
            s.output_yaw = _wrap(s.output_yaw + angle)
            # 绝对校正只按实际经过的时间进入控制输出；同时间戳多帧不会重复挪车。
            delta = _wrap(s.yaw - s.output_yaw)
            # 输出航向按 1°/s 平滑跟融合航向（IMU 接上、卫星回来都不甩头）；只有卫星可信而 IMU 断着时不限速，
            # 这时没有 IMU 推转动，输出要直接跟测向
            trusted = self._trusted()
            if not self._snap_output(stationary):
                limit = math.inf if trusted and dg < dt else math.radians(cfg.output_heading_rate_deg_s) * dt
                s.output_yaw = _wrap(s.output_yaw + max(-limit, min(limit, delta * -math.expm1(-dt / cfg.output_heading_tau_s))))
            ox, oy = s.x - s.output_x, s.y - s.output_y
            fraction = -math.expm1(-dt / cfg.output_position_tau_s)
            cs_out, sn_out = math.cos(s.yaw), math.sin(s.yaw)
            along = ox * cs_out + oy * sn_out
            across = -ox * sn_out + oy * cs_out
            along_step, cross_step = along * fraction, across * fraction
            if (cfg.rejoin_rate_mps > 0 and (s.output_rejoining
                    or s.t - s.fixed_t >= cfg.hold_watch_s)):
                limit = cfg.rejoin_rate_mps * dt
                cross_step = max(-limit, min(limit, cross_step))
                along_step = max(-limit, min(limit, along_step))
            # 正常固定解由滤波与输出时间常数平滑，不套失联接回上限，避免模型误差持续堆积。
            s.output_x += cs_out * along_step - sn_out * cross_step
            s.output_y += sn_out * along_step + cs_out * cross_step
            if math.hypot(s.x - s.output_x, s.y - s.output_y) <= BLEND_DONE_M:
                s.output_rejoining = False
            # 大幅重定位只在轮速确认已经停稳后对齐输出，行驶中先由预算/重置原因停车。
            if s.reset_t > -math.inf and stationary and math.hypot(s.x - s.output_x, s.y - s.output_y) > BLEND_MAX_M:
                s.output_x, s.output_y, s.output_yaw = s.x, s.y, s.yaw
                s.output_rejoining = False
        s.t = t

    def _stationary(self):
        s, cfg = self._s, self.config
        return (s.stationary_since is not None
                and s.t-s.stationary_since >= cfg.stationary_time_s
                and s.t-s.wheel_t <= cfg.sensor_timeout_s)

    def _snap_output(self, stationary=None):
        """控制输出航向能直接对齐就对齐（见 OUTPUT_SNAP_BASELINE_M），返回是否对齐了。推算每一步和每帧测向修正后都调，
        测向刚把融合航向拉回来那一刻输出也跟上，不留到下一步推算。"""
        s = self._s
        if not (self.config.continuous_output and s.output_ready and s.t is not None):
            return False
        if not (self._trusted() and s.t - s.snap_heading_t <= TRUSTED_FRESH_S + 1e-9):
            return False
        if stationary is None:
            stationary = self._stationary()
        if not (stationary or not self.painting or abs(_wrap(s.yaw - s.output_yaw)) > math.radians(OUTPUT_SNAP_DEG)):
            return False
        # 双天线可信且车停着 / 不在喷 / 差超过 OUTPUT_SNAP_DEG：直接对齐
        s.output_yaw = s.yaw
        return True

    def _initialize_output(self):
        s = self._s
        if not s.output_ready and s.position_ready and s.yaw_ready:
            s.output_x, s.output_y, s.output_yaw = s.x, s.y, s.yaw
            s.output_ready = True

    def _observe(self, h, residual, variance, gate=True, update_attitude=True, update_bias=True):
        s = self._s
        xx, xy, xh, xb, yy, yh, yb, hh, hb, bb = s.p
        a, b, c, d = h
        p0 = xx*a + xy*b + xh*c + xb*d
        p1 = xy*a + yy*b + yh*c + yb*d
        p2 = xh*a + yh*b + hh*c + hb*d
        p3 = xb*a + yb*b + hb*c + bb*d
        innovation = a*p0 + b*p1 + c*p2 + d*p3 + variance
        if innovation <= 0 or (gate and residual*residual > self.config.innovation_gate_sigma ** 2 * innovation):
            return False
        k = residual / innovation
        s.x += p0*k
        s.y += p1*k
        if update_attitude:
            s.yaw = _wrap(s.yaw + p2*k)
            if update_bias:
                s.bias += p3*k
        inv = 1 / innovation
        s.p = (max(1e-16, xx-p0*p0*inv), xy-p0*p1*inv, xh-p0*p2*inv, xb-p0*p3*inv,
               max(1e-16, yy-p1*p1*inv), yh-p1*p2*inv, yb-p1*p3*inv,
               max(1e-16, hh-p2*p2*inv) if update_attitude else hh,
               hb-p2*p3*inv if update_attitude else hb,
               max(1e-20, bb-p3*p3*inv) if update_attitude and update_bias else bb)
        return True

    def _wheel(self, sample):
        s, cfg = self._s, self.config
        # 断档才重来（静止判定、标定段）；10 Hz 时丢一帧就是 0.2 s，按至少 3 个采样周期算断档
        if sample.t - s.wheel_t > max(cfg.sensor_timeout_s, 3 * cfg.wheel_sample_period_s):
            s.stationary_since = None
            s.align = None
            self._end_segment()
            s.wheel_cal = s.wheel_cal.clear()
        s.wheel_rate_sigma = sample.sigma_yaw_rate_rps
        s.hall_still = sample.hall_still
        s.motion_evidence = sample.speed_basis in ('hall','readback_fallback')
        still = (sample.hall_still is True and (not cfg.ground_speed_still
                 or 0 <= sample.t-s.ground_t <= max(cfg.sensor_timeout_s,STATIONARY_DOPPLER_FRESH_S) and s.ground_speed <= .02)
                 ) if s.motion_evidence else abs(sample.speed) <= cfg.stationary_speed_mps
        if cfg.drive_differential:          # 差速车原地转时车速为 0，但在转
            still = still and abs(sample.yaw_rate) <= math.radians(cfg.gyro_margin_deg_s)
        if still:
            if s.stationary_since is None:
                s.stationary_since = sample.t
        else:
            s.stationary_since = None
        if s.speed * sample.speed < 0:
            self._end_segment()
            s.wheel_cal = s.wheel_cal.clear()
        if s.wheel_epoch != sample.epoch or s.wheel_source != sample.source:
            s.wheel_cal = s.wheel_cal.clear()
        s.speed, s.wheel_yaw, s.wheel_t = sample.speed, sample.yaw_rate, sample.t
        s.wheel_source, s.wheel_epoch = sample.source, sample.epoch
        s.wheel_time_uncertainty, s.wheel_sigma = sample.time_uncertainty_s, sample.sigma_speed_mps
        s.wheel_end = None
        if sample.end_t is not None:
            s.wheel_t = sample.end_t
            s.wheel_end = (sample.end_t,sample.end_speed,sample.end_yaw_rate,
                           sample.sigma_speed_mps if sample.end_sigma_speed_mps is None else sample.end_sigma_speed_mps)

    def _gyro_bound(self, speed, wheel_yaw, dt=None):
        """陀螺读数（去零偏）的合理上限，超过的算尖峰不用。

        阿克曼车：当前车速下满舵的角速度。差速车：里程计角速度的 1.5 倍（原地转也有），再加上行驶中路面颠簸、
        打滑带来的转动（按车速，量级与阿克曼车满舵相同）。两种都再加 gyro_margin_deg_s。
        """
        cfg = self.config
        v = abs(speed * self._s.wheel_cal.applied)
        if cfg.drive_differential:
            bound = 1.5 * abs(wheel_yaw) + v * math.tan(cfg.max_steer_rad) / cfg.wheelbase_m
        else:
            bound = v * math.tan(cfg.max_steer_rad) / cfg.wheelbase_m
        if cfg.yaw_input == 'angle' and dt is not None and dt > 0:
            # 相邻两帧航向角之差的量化误差（一个分辨率）
            bound += math.radians(cfg.yaw_resolution_deg) / dt
            if cfg.drive_differential:
                # 差速车低速转动时里程计读小，不能拿它卡 IMU 航向角：车身转得到的转速都收
                bound = max(bound, math.radians(cfg.angle_rate_floor_deg_s))
        return bound + math.radians(cfg.gyro_margin_deg_s)

    def _yaw_rate(self, sample):
        """这一帧用于推算的转速：'rate' 就是陀螺 z 轴；'angle' 是与上一帧 IMU 航向角之差 / 时间差
        （上一帧没有航向角、隔太久或这一帧没带航向角时退回陀螺）。"""
        s, cfg = self._s, self.config
        if cfg.yaw_input == 'angle' and sample.yaw is not None and math.isfinite(sample.yaw) \
                and s.imu_yaw is not None and 0 < sample.t - s.imu_t <= cfg.sensor_timeout_s:
            return _wrap(sample.yaw - s.imu_yaw) / (sample.t - s.imu_t)
        # 航向角漂移率与原始陀螺零偏不是同一个量；缺角度时在等效转速中抵消前者。
        return sample.gyro_z - s.raw_gyro_bias + s.bias if cfg.yaw_input == 'angle' else sample.gyro_z

    def _imu(self, sample):
        s, cfg = self._s, self.config
        if s.imu_mismatch:
            return
        # 姿态与航向推算无关：航向角这一帧被当尖峰丢掉，横滚俯仰照样记下
        if sample.roll is not None and sample.pitch is not None \
                and math.isfinite(sample.roll) and math.isfinite(sample.pitch):
            s.roll, s.pitch, s.tilt_t = sample.roll, sample.pitch, sample.t
        fresh_wheel = sample.t - s.wheel_t <= cfg.sensor_timeout_s
        rate = self._yaw_rate(sample)
        yaw = sample.yaw if sample.yaw is not None and math.isfinite(sample.yaw) else None
        self._check_mismatch(sample.t, rate, fresh_wheel)
        bound = self._gyro_bound(s.speed, s.wheel_yaw, sample.t - s.imu_t)
        if fresh_wheel and abs(rate - s.bias) > bound:
            # 航向角模式下跳变（航向清零、传感器重启）也在这里丢掉：下一帧从新的航向角接着算差
            s.gyro_rejected += 1
            s.imu_t, s.imu_yaw = sample.t, yaw
            return
        s.gyro, s.imu_t, s.imu_yaw = rate, sample.t, yaw
        s.imu_valid_t = sample.t
        if s.wheel_source != 'legacy':
            self._motion_check(sample.t, rate-s.bias, fresh_wheel)
        if cfg.yaw_input == 'angle' and fresh_wheel and s.stationary_since is not None \
                and sample.t - s.stationary_since >= cfg.stationary_time_s:
            s.raw_bias_samples += 1
            alpha = max(0.005, 1.0 / s.raw_bias_samples)
            s.raw_gyro_bias += alpha * (sample.gyro_z - s.raw_gyro_bias)
        # 静止时拿读数估零偏只对陀螺角速度有意义；IMU 航向角 0.01° 一格，单帧的差不能当零偏观测，
        # 它的漂移由双天线测向估
        if cfg.yaw_input == 'rate' and fresh_wheel and s.stationary_since is not None \
                and sample.t - s.stationary_since >= cfg.stationary_time_s:
            self._observe((0., 0., 0., 1.), sample.gyro_z - s.bias,
                          math.radians(cfg.gyro_noise_deg_s) ** 2, gate=False)
            s.gyro = s.bias
        if self._straight():
            if s.straight_since is None:
                s.straight_since = sample.t
        else:
            self._end_segment()

    def _check_mismatch(self, t, rate, fresh_wheel):
        """差速车：轮速里程计的角速度可靠（原地转时也有），IMU 的转向与它持续相反（装反、坐标换算错）：
        之后不再用 IMU，航向不确定度放大，下一帧测向直接接管。只看明显在转的时候，停车、慢转不判。"""
        s, cfg = self._s, self.config
        if not cfg.drive_differential:
            return
        if s.wheel_source != 'legacy':
            return  # 轮子打滑也会与 IMU 不一致，不能据此永久判 IMU 装反。
        if (fresh_wheel and abs(s.wheel_yaw) >= MISMATCH_WHEEL_RAD_S and abs(rate) >= MISMATCH_IMU_RAD_S
                and rate * s.wheel_yaw < 0):
            if s.mismatch_since is None:
                s.mismatch_since = t
            if t - s.mismatch_since >= MISMATCH_S:
                s.imu_mismatch = True
                p = list(s.p)
                p[7] += math.radians(30.) ** 2      # 这时的航向已被 IMU 带偏，交给测向重新定
                s.p = tuple(p)
        else:
            s.mismatch_since = None

    def _straight(self):
        s, cfg = self._s, self.config
        return (s.t - s.wheel_t <= cfg.sensor_timeout_s
                and s.t - s.imu_valid_t <= cfg.sensor_timeout_s
                and abs(s.speed * s.wheel_cal.applied) > cfg.course_min_speed_mps
                and abs(s.gyro - s.bias) < math.radians(cfg.straight_rate_deg_s)
                and abs(s.wheel_yaw) < math.radians(cfg.straight_rate_deg_s) + .002)

    def _motion_check(self, t, rate, fresh_wheel):
        """只报告运动矛盾，不指定哪只传感器有错；主机接收时间差计入门限。"""
        s = self._s
        limit = .12 + .3*abs(rate) + 3*s.wheel_time_uncertainty
        difference = abs(rate-s.wheel_yaw)
        if s.motion_evidence and fresh_wheel:
            # 霍尔差分的量化噪声不能被当作持续转向矛盾；对同一 0.3 秒区间积分核对。
            rows = tuple(r for r in s.motion_check_rows if t-r[0] <= .32)+((t,rate-s.wheel_yaw),)
            s.motion_check_rows = rows
            span = rows[-1][0]-rows[0][0]
            if span < .25:
                s.motion_since = None
                return
            difference = abs(sum(.5*(a[1]+b[1])*(b[0]-a[0]) for a,b in zip(rows,rows[1:]))/span)
            # 计数差分误差是两端一格，不能按每帧独立白噪声永久放大门限。
            limit += 3*s.wheel_rate_sigma*self.config.wheel_sample_period_s/span
        else:
            s.motion_check_rows = ()
        if fresh_wheel and difference > limit:
            if s.motion_since is None:
                s.motion_since = t
            if t-s.motion_since >= .3:
                s.motion_conflict_t = t
                s.motion_error_rate = max(s.motion_error_rate, .02 + .5*self.config.wheelbase_m*difference)
                s.wheel_cal = s.wheel_cal.clear()
        else:
            s.motion_since = None
            if t-s.motion_conflict_t > 1.0:
                s.motion_error_rate = 0.0

    def _wheel_scale_fix(self, sample, xy):
        s, cfg = self._s, self.config
        good = (cfg.wheel_scale_auto and s.wheel_source == 'encoder' and self._has_heading()
                and s.t-s.imu_valid_t <= cfg.sensor_timeout_s and s.t-s.wheel_t <= cfg.sensor_timeout_s
                and s.wheel_time_uncertainty <= .05
                and s.outlier_since is None and s.t-s.weak_t > cfg.fixed_timeout_s
                and s.t-max(s.reset_t, s.rejoin_t, s.motion_conflict_t) > 1.0
                and abs(s.speed*s.wheel_cal.applied) >= cfg.course_min_speed_mps
                and abs(s.gyro-s.bias) < math.radians(3) and abs(s.wheel_yaw) < math.radians(5)
                and max(sample.sigma_e, sample.sigma_n) <= .02)
        if not good:
            s.wheel_cal = s.wheel_cal.clear()
            return
        # 使用原始固定解及双天线姿态扣杆臂，轮程保持未标定值；不用融合位置给自己背书。
        heading = s.raw_yaw+self._mount
        ax, ay, _ = self._lever()
        x = xy[0]-math.cos(heading)*ax+math.sin(heading)*ay
        y = xy[1]-math.sin(heading)*ax-math.cos(heading)*ay
        s.wheel_cal = s.wheel_cal.observe(sample.t, s.odo, x, y, heading,
                                         max(cfg.fix_floor_m, sample.sigma_e, sample.sigma_n),
                                         configured=cfg.wheel_scale, distance_m=cfg.wheel_scale_block_m)

    def _straight_ready(self):
        s = self._s
        return (self._straight() and s.straight_since is not None
                and s.t - s.straight_since >= self.config.straight_time_s)

    def _has_heading(self):
        return (self._s.heading_ok
                and self._s.t - self._s.heading_t <= self.config.heading_timeout_s)

    def tilt_angles(self, roll, pitch):
        """弧度；零点只修正横滚、俯仰，不改变 IMU 航向或陀螺。"""
        return _wrap(roll - self._tilt_zero[0]), _wrap(pitch - self._tilt_zero[1])

    def tilt_reference_changed(self):
        """以当前状态封住历史，迟到的旧参考观测不能重新回放。仅在停稳后调用。"""
        if self._s.t is not None:
            self._base_key = (self._s.t, 4)
        self._keys.clear(); self._events.clear(); self._snapshots.clear()
        self._s.segment = None
        self._base = self._s.__dict__.copy()

    def zero_tilt(self, roll, pitch):
        if not all(math.isfinite(v) for v in (roll, pitch)):
            raise ValueError('IMU 姿态无效')
        self._tilt_zero = (roll, pitch)
        self._tilt_zeroed = True
        self.tilt_reference_changed()

    def _lever(self):
        """ANT1 相对后轴中心地面点的水平杠杆臂，车体系 (前, 左)，m；第三项是没做倾斜补偿的原因（做了为 None）。

        三维杠杆臂 (ax, ay, h) 按 REP-103 姿态 R = Rz(航向)·Ry(俯仰)·Rx(横滚) 转到当地水平面：
        先绕 x 横滚、再绕 y 俯仰，取水平分量（航向由调用方再转）。横滚左侧抬起为正，天线往右偏；
        俯仰车头压低为正，天线往前偏。姿态断了、太陡或 IMU 已不用时退回平地杠杆臂，只降级、不停车。
        """
        s, cfg = self._s, self.config
        ax, ay = cfg.antenna_xy
        h = cfg.antenna_height_m
        if not cfg.tilt_compensation_enabled:
            return ax, ay, 'disabled'
        if h <= 0:
            return ax, ay, 'no_height'
        if s.imu_mismatch:
            return ax, ay, 'imu_mismatch'
        if s.t is None or s.t - s.tilt_t > cfg.sensor_timeout_s + 1e-9:
            return ax, ay, 'attitude_stale'
        limit = math.radians(cfg.max_tilt_deg)
        roll, pitch = self.tilt_angles(s.roll, s.pitch)
        if abs(roll) > limit or abs(pitch) > limit:
            return ax, ay, 'too_steep'
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        return ax * cp + (ay * sr + h * cr) * sp, ay * cr - h * sr, None

    def _yaw_observation(self, yaw, variance, gate=True, update_bias=True):
        s = self._s
        if not s.yaw_ready:
            if s.position_ready:
                ax, ay, _ = self._lever()
                s.x += math.cos(s.yaw)*ax - math.sin(s.yaw)*ay - math.cos(yaw)*ax + math.sin(yaw)*ay
                s.y += math.sin(s.yaw)*ax + math.cos(s.yaw)*ay - math.sin(yaw)*ax - math.cos(yaw)*ay
            s.yaw, s.yaw_ready = _wrap(yaw), True
            p = s.p
            s.p = (p[0], p[1], 0., p[3], p[4], 0., p[6], variance, 0., p[9])
            return True
        gain = s.p[7] / (s.p[7] + variance)
        accepted = self._observe((0., 0., 1., 0.), _wrap(yaw - s.yaw), variance,
                                 gate=gate, update_bias=update_bias)
        if accepted:
            s.scale_turn *= 1.0 - gain
        return accepted

    def _heading(self, sample, trusted=False):
        s, cfg = self._s, self.config
        valid = (sample.sol in _HEADING and sample.sats >= cfg.min_sats
                 and all(math.isfinite(v) for v in (sample.heading_deg, sample.pitch_deg,
                                                   sample.length_m, sample.sigma_heading_deg))
                 and sample.sigma_heading_deg >= 0)
        trusted = valid and bool(trusted)
        self._mark_trust(sample.t, trusted)
        s.snap_heading_t = (sample.t if valid and OUTPUT_SNAP_BASELINE_M[0] <= sample.length_m <= OUTPUT_SNAP_BASELINE_M[1]
                            else -math.inf)
        if not valid:
            s.heading_ok = False
            return
        raw = _wrap(math.radians(90. - sample.heading_deg) - self._baseline)
        rate = self._turn_rate()
        var = math.radians(max(cfg.heading_floor_deg, sample.sigma_heading_deg)) ** 2 + (rate * cfg.heading_sync_s) ** 2
        yaw = _wrap(raw + self._mount - s.heading_bias)
        if cfg.drive_differential:
            self._adaptive_heading(sample, yaw, raw, var, trusted)
            return
        if self._yaw_observation(yaw, var):
            s.heading_t, s.heading_ok, s.raw_yaw, s.raw_heading_var = sample.t, True, raw, var
            s.rej_since = None
            return
        s.heading_rejected += 1
        # 稳定的测向必须信：连续被拒、彼此一致、车转得不快，就是推算的航向错了，按测向重定（不停车）
        if s.rej_since is None:
            s.rej_since, s.rej_ref, s.rej_lo, s.rej_hi = sample.t, yaw, 0., 0.
            return
        d = _wrap(yaw - s.rej_ref)
        s.rej_lo, s.rej_hi = min(s.rej_lo, d), max(s.rej_hi, d)
        if s.rej_hi - s.rej_lo > math.radians(REANCHOR_SPREAD_DEG):
            s.rej_since, s.rej_ref, s.rej_lo, s.rej_hi = sample.t, yaw, 0., 0.     # 测向自己在跳：不算稳定，重新数
            return
        if sample.t - s.rej_since >= REANCHOR_S and abs(rate) < math.radians(REANCHOR_RATE_DEG_S):
            s.yaw = yaw
            p = s.p
            s.p = (p[0], p[1], 0., p[3], p[4], 0., p[6], var, 0., p[9])
            s.heading_t, s.heading_ok, s.raw_yaw, s.raw_heading_var = sample.t, True, raw, var
            s.heading_reset_t, s.rej_since = sample.t, None
            s.heading_resets += 1

    def _adaptive_heading(self, sample, yaw, raw, variance, trusted):
        """Gen5：有效测向都用（rtk_node 判有效：整周固定、基线 0.30–0.50 m），不拿 IMU 否决测向
        （2026-10-05 用户：「原来的检查必须删除，这种检查是不合理的」；IMU 只是辅助）。
        增益照常限幅：直行慢校、转弯或和推算差得多时快校；限幅通过放大 R 实现，转弯/接回时不把角度差学成 IMU 漂移率。
        """
        s, cfg = self._s, self.config
        residual = _wrap(yaw - s.yaw)
        innovation = s.p[7] + variance
        s.heading_innovation = residual
        s.heading_test_ratio = residual * residual / max(1e-20, cfg.innovation_gate_sigma ** 2 * innovation)
        elapsed = sample.t - s.heading_sample_t
        s.heading_sample_t = sample.t
        imu_ok = not s.imu_mismatch and sample.t - s.imu_valid_t <= cfg.sensor_timeout_s
        turning = sample.t - s.last_turn_t <= 1.0
        if trusted:
            gap = sample.t - s.sat_yaw_t
            if 0 < gap <= cfg.heading_timeout_s:
                s.sat_rate, s.sat_rate_t = _wrap(yaw - s.sat_yaw) / gap, sample.t
            s.sat_yaw, s.sat_yaw_t = yaw, sample.t
        if trusted and not imu_ok and s.yaw_ready:
            # 卫星可信、IMU 断着（IMU 只是辅助）：航向直接取测向，主天线位置不动、后轴点按新航向改算；
            # 两帧测向之间按测向算的转速推（见 _predict）
            ax, ay, _ = self._lever()
            s.x += math.cos(s.yaw)*ax - math.sin(s.yaw)*ay - math.cos(yaw)*ax + math.sin(yaw)*ay
            s.y += math.sin(s.yaw)*ax + math.cos(s.yaw)*ay - math.sin(yaw)*ax - math.cos(yaw)*ay
            s.yaw = yaw
            p = s.p
            s.p = (p[0], p[1], 0., p[3], p[4], 0., p[6], variance, 0., p[9])
            s.heading_t, s.heading_ok, s.raw_yaw, s.raw_heading_var = sample.t, True, raw, variance
            s.rej_since = None
            return
        if not s.yaw_ready:
            self._yaw_observation(yaw, variance)
            s.heading_t, s.heading_ok, s.raw_yaw, s.raw_heading_var = sample.t, True, raw, variance
            return
        # 和推算差得多（超过 innovation_gate_sigma 个 σ）：错的是推算，承认推算过于自信、按测向快校
        recovering = s.heading_test_ratio > 1.0
        if recovering:
            s.heading_recovery_t = sample.t
        dt = min(0.2, max(0.001, elapsed if math.isfinite(elapsed) else 0.05))
        recently_recovered = sample.t-s.heading_recovery_t <= HEADING_RECOVERY_S
        tau = cfg.heading_turn_tau_s if turning or not imu_ok or recovering or recently_recovered else cfg.heading_straight_tau_s
        max_gain = -math.expm1(-dt / tau)
        variance *= max(1.0, cfg.heading_correlation_s / dt)
        if recovering:
            # 已有独立恢复证据时承认旧推算过于自信；增大对角方差保持半正定，不硬改均值。
            p = list(s.p)
            p[7] = max(p[7], residual ** 2)
            s.p = tuple(p)
            variance = max(variance, residual ** 2 / cfg.soft_gate_sigma ** 2)
        variance = max(variance, s.p[7] * (1.0 / max_gain - 1.0))
        s.heading_gain = s.p[7] / (s.p[7] + variance)
        aligned = not cfg.continuous_output or abs(_wrap(s.yaw - s.output_yaw)) <= math.radians(0.05)
        learn_drift = (imu_ok and not turning and not recovering and aligned
                       and sample.t - s.heading_recovery_t > cfg.heading_straight_tau_s
                       and abs(residual) <= math.radians(0.3))
        self._yaw_observation(yaw, variance, gate=False, update_bias=learn_drift)
        s.heading_t, s.heading_ok, s.raw_yaw, s.raw_heading_var = sample.t, True, raw, variance
        s.rej_since = None

    def _turn_rate(self):
        """现在转得多快（rad/s）：IMU 能用就用 IMU，否则差速车用轮速里程计，都没有为 0。"""
        s, cfg = self._s, self.config
        if not s.imu_mismatch and s.t - s.imu_valid_t <= cfg.sensor_timeout_s:
            return s.gyro - s.bias
        if cfg.drive_differential and s.t - s.wheel_t <= cfg.sensor_timeout_s:
            return s.wheel_yaw
        return 0.

    def _fix(self, sample, xy):
        s, cfg = self._s, self.config
        trusted = xy is not None and sample.sol in _FIXED and len(xy) > 3 and bool(xy[3])
        self._mark_trust(sample.t, trusted)
        if sample.inertial_only and sample.sol not in _FIXED:
            s.position_quality='DEAD_RECKONING'
            s.fix_reason='rtk_float' if xy is not None else 'fix_unavailable'
            s.wheel_cal=s.wheel_cal.clear()
            self._end_segment()
            return
        # 固定解都用：双天线位置互检、推算位移互检都不再否决卫星（2026-10-05 用户：「原来的检查必须删除，
        # 这种检查是不合理的」）。互检偏差只用来给位置降权（dual_weight_m），不丢帧、不停车
        if xy is None or sample.sol not in _FIXED:
            s.wheel_cal = s.wheel_cal.clear()
        if xy is None:
            s.position_quality = 'DEAD_RECKONING'
            s.fix_reason = 'fix_unavailable'
            self._end_segment()
            return
        fixed = sample.sol in _FIXED
        if not fixed:
            s.bias_window = None
        if not fixed:
            s.outlier_since = None
        floor = cfg.fix_floor_m if fixed else cfg.float_floor_m
        scale = cfg.fix_sigma_scale if fixed else 1.0
        if fixed and cfg.fix_jitter_k > 0:
            floor = max(floor, cfg.fix_jitter_k * math.sqrt(s.fix_jitter / 2))
        re, rn = max(floor, scale * sample.sigma_e) ** 2, max(floor, scale * sample.sigma_n) ** 2
        if fixed and cfg.fix_jitter_k > 0:
            sync = self._turn_rate() * math.hypot(*cfg.antenna_xy) * cfg.heading_sync_s
            re, rn = re + sync * sync, rn + sync * sync
        dev = xy[2] if len(xy) > 2 else None
        # 卫星可信而轮速或 IMU 断着：推算没有信息，不拿它给卫星降权（互检降权、离群放大都不做），位置直接跟卫星
        direct = trusted and (sample.t-s.wheel_t > cfg.sensor_timeout_s or s.imu_mismatch
                              or sample.t-s.imu_valid_t > cfg.sensor_timeout_s)
        # 单天线（course_init_distance_m > 0）起步、车头还没定：推算不知道车往哪走，固定解和它对不上不算离群，位置直接跟
        # 固定解，按行驶轨迹定车头才不会被离群判断打断（车头朝哪都一样定得出来）
        direct = direct or (sample.sol in _FIXED and not s.yaw_ready and cfg.course_init_distance_m > 0)
        variance_scale = sample.position_variance_scale
        if not isinstance(variance_scale,(int,float)) or not math.isfinite(variance_scale):
            variance_scale = 1.0
        s.position_variance_scale = 1.0 if direct else max(1.,min(25.,variance_scale))
        re, rn = re*s.position_variance_scale, rn*s.position_variance_scale
        if fixed and not direct and dev is not None and dev > cfg.dual_weight_m > 0:
            # 两根天线对不上一点：卫星位置不太可信，按偏差降权，位置多靠推算（见 FusionConfig.dual_weight_m）
            w = min(DUAL_WEIGHT_MAX, (dev / cfg.dual_weight_m) ** 2)
            re, rn = re * w, rn * w
            s.weak_t = sample.t
        ax, ay, _ = self._lever()           # 含倾斜补偿：天线在坡上偏出去的那段不算车的位置
        cs, sn = math.cos(s.yaw), math.sin(s.yaw)
        lx, ly = cs*ax-sn*ay, sn*ax+cs*ay
        dx, dy = xy[0]-lx-s.x, xy[1]-ly-s.y
        recovering = s.position_ready and fixed and sample.t - s.fixed_t >= cfg.fixed_timeout_s - 1e-9
        if recovering and cfg.continuous_output and cfg.rejoin_rate_mps > 0:
            # 推算维持后卫星回来：不管差多少，控制输出都按 rejoin_rate_mps 平滑挪回卫星（无感接回，线上不出折角）；
            # 差超过 BLEND_MAX_M 的在下面直接对齐
            s.output_rejoining = True
        far = math.hypot(dx, dy) > cfg.reset_distance_m
        persistent = (fixed and s.outlier_since is not None
                      and sample.t - s.outlier_since >= cfg.outlier_reset_s - 1e-9)
        reset = (recovering or persistent) and far
        # 推算维持模式：维持后接回、或与推算持续对不上（推算走偏了），都按 RTK 对齐、不停车；
        # 差得再远也是推算错了，输出在下面直接对齐（不再先停车重定位）
        rejoin = reset and cfg.rtk_hold_s > 0
        if cfg.continuous_output and rejoin:
            s.output_rejoining = True
        if rejoin and cfg.rejoin_rate_mps > 0 and not cfg.continuous_output and not s.blending and math.hypot(dx, dy) <= BLEND_MAX_M:
            s.blending, s.blend_t = True, sample.t - 0.05
            s.blend_d0 = math.hypot(dx, dy)
            s.blend_until = sample.t + BLEND_SPARE * s.blend_d0 / cfg.rejoin_rate_mps + 1.0
            s.rejoins += 1
            self._end_segment()
        if s.blending and not fixed:
            s.blending = False                  # 接回途中又不是固定解了：不再往它挪，照常（浮点解权重很小）
        if s.blending and (sample.t > s.blend_until or math.hypot(dx, dy) > BLEND_MAX_M):
            # 到时限还没挪到 BLEND_DONE_M 以内（或接回途中差到 BLEND_MAX_M 以外，不等时限）：
            # - 比开始接回时离得还远：推算与 RTK 越差越远（比如轮速刻度错了），慢慢挪追不上，不再挂在半路，
            #   按旧做法一步对齐（下面 reset 分支，计一次接回）；
            # - 已经挪近了、只是挪不进 1 cm：多半是 RTK 自己在跳（靠楼多路径），交回普通（降过权的）更新，
            #   不往这帧跳（2026-10-01 回放 11:42 靠楼那圈：RTK 在 1.6–6.9 cm 间来回跳，4 s 都没挪进 1 cm）
            s.blending = False
            if math.hypot(dx, dy) > s.blend_d0:
                reset = rejoin = True
        if s.blending:
            dist = math.hypot(dx, dy)
            if dist > BLEND_DONE_M:
                # 平滑接回：每帧最多挪 rejoin_rate_mps × 帧间隔，挪到差 BLEND_DONE_M 以内再照常更新
                # （差 5 cm 就交回普通更新时，剩下的几厘米 0.2 s 内补完，还是一个小跳）
                step = min(dist, cfg.rejoin_rate_mps * min(0.1, max(0.0, sample.t - s.blend_t)))
                # 不确定度照常按这帧固定解更新，只是位置不一步跟过去：不更新的话接回这几秒 σ 只涨不降，降权一停就因
                # position_uncertain 停车（同一回放：接回途中停了 0.35 s）
                x0, y0 = s.x, s.y
                self._observe((1., 0., -ly, 0.), dx, re, gate=False, update_attitude=False)
                self._observe((0., 1., lx, 0.), dy, rn, gate=False, update_attitude=False)
                s.x = x0 + dx / dist * step
                s.y = y0 + dy / dist * step
                s.blend_t = s.rejoin_t = sample.t
                s.outlier_since = None
                s.fix_t, s.fixed_t, s.fix_reason, s.position_quality = sample.t, sample.t, '', 'RTK_FIXED'
                s.hold_distance = s.hold_turn = s.motion_error = 0.0
                s.bias_window = None
                s.wheel_cal = s.wheel_cal.clear()
                return
            s.blending = False
        if direct and fixed and s.position_ready and not reset:
            # 卫星可信、轮速或 IMU 断着：位置直接取这帧卫星（不确定度取这帧的观测方差），不计接回；
            # 控制输出照常按时间常数平滑跟上，两帧之间有卫星地速时按地速往前推
            s.outlier_since = None
            s.x, s.y = xy[0]-lx, xy[1]-ly
            p = s.p
            s.p = (re+ly*ly*p[7], -lx*ly*p[7], ly*p[7], ly*p[8],
                   rn+lx*lx*p[7], -lx*p[7], -lx*p[8], p[7], p[8], p[9])
        elif not s.position_ready or reset:
            s.outlier_since = None
            s.bias_window = None
            s.x, s.y = xy[0]-lx, xy[1]-ly
            p = s.p
            s.p = (re+ly*ly*p[7], -lx*ly*p[7], ly*p[7], ly*p[8],
                   rn+lx*lx*p[7], -lx*p[7], -lx*p[8], p[7], p[8], p[9])
            s.position_ready = True
            if rejoin:
                s.rejoin_t = sample.t
                s.rejoins += 1
                s.align = None
                self._end_segment()
                if (cfg.continuous_output and s.output_ready
                        and math.hypot(s.x - s.output_x, s.y - s.output_y) > BLEND_MAX_M):
                    s.output_x, s.output_y = s.x, s.y
                    s.output_rejoining = False
            elif reset:
                s.reset_t = sample.t
                s.resets += 1
                s.align = None
                self._end_segment()
        else:
            # 先做联合门限检验，再逐轴更新，防止只接受离群点的半个坐标。
            p = s.p
            se = p[0] - 2*ly*p[2] + ly*ly*p[7] + re
            snn = p[4] + 2*lx*p[5] + lx*lx*p[7] + rn
            sen = p[1] + lx*p[2] - ly*p[5] - lx*ly*p[7]
            det = se*snn - sen*sen
            nis = (snn*dx*dx - 2*sen*dx*dy + se*dy*dy) / det if det > 0 else math.inf
            if not fixed and nis > cfg.innovation_gate_sigma ** 2:
                s.fix_rejected += 1
                s.fix_reason = 'position_outlier'
                return
            if fixed and not direct and nis > cfg.soft_gate_sigma ** 2:
                # 固定解离群：方差按离群程度放大（Huber），离得越远影响越小；持续的偏差几秒内平滑吸收
                inflate = nis / cfg.soft_gate_sigma ** 2
                re, rn = re * inflate, rn * inflate
                s.fix_rejected += 1
                if s.outlier_since is None:
                    s.outlier_since = sample.t
            elif fixed:
                s.outlier_since = None
            if fixed and cfg.fix_jitter_k > 0:
                self._track_jitter(sample.t, dx, dy)
            # 位置用 Schmidt 更新：保留相关性，但不让位置暗中充当低噪声航向。
            self._observe((1., 0., -ly, 0.), dx, re, gate=False, update_attitude=False)
            cs, sn = math.cos(s.yaw), math.sin(s.yaw)
            lx, ly = cs*ax-sn*ay, sn*ax+cs*ay
            self._observe((0., 1., lx, 0.), xy[1]-s.y-ly, rn, gate=False, update_attitude=False)
        s.fix_t, s.fix_reason = sample.t, ''
        s.position_quality = 'RTK_FIXED' if fixed else 'RTK_FLOAT'
        if fixed:
            s.fixed_t = sample.t
            s.hold_distance = s.hold_turn = s.motion_error = 0.0
            self._wheel_scale_fix(sample, xy)
            self._position_course(sample, xy)
            self._learn_heading_bias(sample, xy)
        else:
            s.align = None
            self._end_segment()

    def _track_jitter(self, t, dx, dy):
        """固定解抖动估计（见 FusionConfig.fix_jitter_k）：相邻两帧新息之差的平方按 fix_jitter_tau_s 指数平均。
        隔太久（中间断了）只记下这一帧，不算差。"""
        s, cfg = self._s, self.config
        prev = s.fix_innov
        s.fix_innov = (t, dx, dy)
        if prev is None or not 0 < t - prev[0] <= cfg.fixed_timeout_s:
            return
        d2 = ((dx - prev[1]) ** 2 + (dy - prev[2]) ** 2) / 2      # 每个轴
        a = -math.expm1(-(t - prev[0]) / cfg.fix_jitter_tau_s)
        s.fix_jitter += a * (d2 - s.fix_jitter)

    def _learn_heading_bias(self, sample, xy):
        """测向偏差在线学习（见 FusionConfig.heading_bias_learning）：每帧可用的固定解调一次。"""
        s, cfg = self._s, self.config
        weak = cfg.dual_weight_m > 0 and len(xy) > 2 and xy[2] is not None and xy[2] > cfg.dual_weight_m
        if (not cfg.heading_bias_learning or not s.yaw_ready or not self._has_heading() or s.imu_mismatch
                or s.outlier_since is not None or s.blending or weak):
            s.bias_window = None
            return
        ax, ay, _ = self._lever()
        cs, sn = math.cos(s.yaw), math.sin(s.yaw)
        # 推算的天线位置（与真位置只差一个常数平移，比位移用）
        px, py = s.dr_x + cs*ax - sn*ay, s.dr_y + sn*ax + cs*ay
        w = s.bias_window
        if (w is None or sample.t - w[4] > cfg.fixed_timeout_s
                or abs(s.predicted_turn - w[5]) > math.radians(HEADING_BIAS_TURN_DEG)):
            s.bias_window = (xy[0], xy[1], px, py, sample.t, s.predicted_turn)
            return
        mx, my = xy[0] - w[0], xy[1] - w[1]
        moved = math.hypot(mx, my)
        if moved < cfg.heading_bias_window_m:
            s.bias_window = w[:4] + (sample.t, w[5])
            return
        qx, qy = px - w[2], py - w[3]
        # 轮速刻度明显不对（打滑、被推）的段不用：只比方向，长度差太多说明推算本身不可信
        if HEADING_BIAS_RATIO[0] < math.hypot(qx, qy) / moved < HEADING_BIAS_RATIO[1]:
            residual = _wrap(math.atan2(qy, qx) - math.atan2(my, mx))
            step = math.radians(cfg.heading_bias_step_deg)
            limit = math.radians(cfg.heading_bias_max_deg)
            # 刚开始（前 HEADING_BIAS_FAST_N 段）学得快，之后按 heading_bias_gain 慢慢跟（少把 RTK 自己的慢漂学进来）。
            # 单次调整受 heading_bias_step_deg 限制：测向一下子挪 0.75° 以上会被当成测向跳变（_adaptive_heading）
            gain = HEADING_BIAS_FAST_GAIN if s.heading_bias_updates < HEADING_BIAS_FAST_N else cfg.heading_bias_gain
            s.heading_bias = max(-limit, min(limit, s.heading_bias + max(-step, min(step, gain * residual))))
            s.heading_bias_updates += 1
        s.bias_window = (xy[0], xy[1], px, py, sample.t, s.predicted_turn)

    def _position_course(self, sample, xy):
        s, cfg = self._s, self.config
        if cfg.drive_differential and (s.outlier_since is not None
                or (cfg.dual_weight_m > 0 and len(xy) > 2 and xy[2] is not None and xy[2] > cfg.dual_weight_m)):
            s.align = None
            self._end_segment()
            return  # 降权、离群位置不作为航向冲突的独立恢复证据。
        if cfg.course_align_distance_m > 0:
            self._imu_course(sample, xy)
            return
        if not self._straight_ready():
            if not self._straight():
                self._end_segment()
            return
        # 有符号轮程作自变量，倒车的斜率仍指向车头；在线回归不用存整段轨迹。
        r = s.segment
        if r is None:
            r = (0, s.odo, s.odo, 0., 0., 0., 0., 0., 0., 0., 0, 0., 0., 0., 0.)
        n, start, mo, mx, my, voo, vox, voy, residual_x, residual_y, nh, hc, hs, hv, sigma_sum = r
        n += 1
        do, dx, dy = s.odo-mo, xy[0]-mx, xy[1]-my
        mo += do/n
        mx += dx/n
        my += dy/n
        voo += do*(s.odo-mo)
        vox += do*(xy[0]-mx)
        voy += do*(xy[1]-my)
        residual_x += dx*(xy[0]-mx)
        residual_y += dy*(xy[1]-my)
        if self._has_heading():
            nh += 1
            hc += math.cos(s.raw_yaw)
            hs += math.sin(s.raw_yaw)
            hv += s.raw_heading_var
        sigma_sum += max(cfg.fix_floor_m, sample.sigma_e, sample.sigma_n) ** 2
        s.segment = (n, start, mo, mx, my, voo, vox, voy, residual_x, residual_y,
                     nh, hc, hs, hv, sigma_sum)
        need = cfg.course_refine_distance_m if s.yaw_ready else cfg.course_init_distance_m
        need = need if need > 0 else cfg.calibration_min_distance_m
        if not self._has_heading() and abs(s.odo-start) >= need and voo > 0 and s.t - s.course_t >= 1.:
            angle = math.atan2(voy, vox)
            # 回归窗口有重叠，不能按每个 RTK 帧把它当独立观测注入。
            var = max(math.radians(.2)**2, 4*(sigma_sum/n)/max(voo, 1e-12))
            first = not s.yaw_ready
            if self._yaw_observation(angle, var, update_bias=not cfg.drive_differential):
                s.course_t = s.t
                if first:
                    # 第一次定下车头（单天线起步）：之前推算按错的车头走，位置被拖偏几厘米、要 2 s 才被固定解拉回；
                    # 按这帧固定解和刚定的车头直接对齐
                    ax, ay, _ = self._lever()
                    c, sn = math.cos(s.yaw), math.sin(s.yaw)
                    s.x, s.y = xy[0] - (c*ax - sn*ay), xy[1] - (sn*ax + c*ay)

    def _imu_course(self, sample, xy):
        """单天线定车头 / 修车头偏移（course_align_distance_m > 0）：车头 = IMU 航向角 + 偏移，偏移按卫星位移修。

        一段里按融合车头（还没定时是 IMU 航向角加任意偏移）和轮程逐帧推算后轴中心的位移，加上杠杆臂随车头转的那点，就是
        推算的天线位移；它和这段卫星测到的天线位移只差一个整体转角 = 车头偏移的误差（轮径比例只影响长短，不影响方向）。
        车身摆、弯道、原地转都由 IMU 记着，不要求走直。还没定车头时走 course_init_distance_m 定第一次，之后每走
        course_align_distance_m 修一次；来回走、原地转为主（位移不到要求的一半）不算，已定车头后夹角超过 ALIGN_MAX_DEG
        （打滑、卫星跳）不修。"""
        s, cfg = self._s, self.config
        ax, ay, _ = self._lever()
        c, sn = math.cos(s.yaw), math.sin(s.yaw)
        off = (c*ax - sn*ay, sn*ax + c*ay)
        if s.align is None:
            s.align = (xy[0], xy[1], off[0], off[1], s.odo, s.yaw, 0., 0., 0.)
            return
        fx0, fy0, ox0, oy0, odo, yaw, drx, dry, dist = s.align
        d = s.odo - odo
        mid = yaw + _wrap(s.yaw - yaw) / 2
        drx, dry, dist = drx + d*math.cos(mid), dry + d*math.sin(mid), dist + abs(d)
        need = cfg.course_align_distance_m if s.yaw_ready or cfg.course_init_distance_m <= 0 else cfg.course_init_distance_m
        if dist < need:
            s.align = (fx0, fy0, ox0, oy0, s.odo, s.yaw, drx, dry, dist)
            return
        s.align = None
        pdx, pdy = drx + off[0] - ox0, dry + off[1] - oy0
        fdx, fdy = xy[0] - fx0, xy[1] - fy0
        shift = math.hypot(fdx, fdy)
        if shift < 0.5 * need or math.hypot(pdx, pdy) < 0.5 * need:
            return
        delta = math.atan2(pdx*fdy - pdy*fdx, pdx*fdx + pdy*fdy)
        if s.yaw_ready and abs(delta) > math.radians(ALIGN_MAX_DEG):
            return
        sigma = max(ALIGN_FIX_SIGMA_M, cfg.fix_sigma_scale * max(sample.sigma_n, sample.sigma_e))
        var = (2 * sigma / shift) ** 2
        first = not s.yaw_ready
        if self._yaw_observation(_wrap(s.yaw + delta), var, update_bias=False):
            s.course_t = s.t
            if first:
                # 同 _position_course：第一次定下车头时位置按这帧固定解和刚定的车头直接对齐
                ax, ay, _ = self._lever()
                c, sn = math.cos(s.yaw), math.sin(s.yaw)
                s.x, s.y = xy[0] - (c*ax - sn*ay), xy[1] - (sn*ax + c*ay)

    def _segment_estimate(self):
        s, cfg = self._s, self.config
        r = s.segment
        if r is None or abs(s.odo-r[1]) < cfg.calibration_min_distance_m or r[5] <= 0 or r[0] < 20:
            return None
        n, _, _, _, _, voo, vox, voy, rxx, ryy, nh, hc, hs, hv, sigma_sum = r
        scale = math.hypot(vox, voy) / voo
        if not .8 < scale < 1.2:
            return None
        noise = max(sigma_sum/n, (rxx+ryy-(vox*vox+voy*voy)/voo)/max(1, 2*n-4))
        scale_var = max(1e-10, noise/voo)
        if nh < 20:
            return (None, math.inf, scale, scale_var, 0, n)
        angle = _wrap(math.atan2(voy, vox) - math.atan2(hs, hc))
        var = max(math.radians(cfg.calibration_floor_deg)**2,
                  noise / (voo*scale*scale) + hv/(nh*nh))
        return angle, var, scale, scale_var, nh, n

    def _end_segment(self):
        s = self._s
        estimate = self._segment_estimate()
        if estimate is not None:
            angle, var, scale, scale_var, n, scale_n = estimate
            if angle is not None:
                s.mount_sum_c += math.cos(angle)/var
                s.mount_sum_s += math.sin(angle)/var
                s.mount_weight += 1/var
                s.cal_samples += n
            s.scale_sum += scale/scale_var
            s.scale_weight += 1/scale_var
            s.cal_scale_samples += scale_n
            s.cal_segments += 1
        s.segment = None
        s.straight_since = None

    def _doppler(self, sample):
        """接管速度的独立入口：投影前保留矢量，检查横向残差和速度变化，再刷新可信时间。"""
        s, cfg = self._s, self.config
        imu_fresh = sample.t-s.imu_valid_t <= cfg.sensor_timeout_s and not s.imu_mismatch
        # 卫星可信时航向由测向定，不要求 IMU（IMU 只是辅助，断了照样用地速推）
        trusted = self._trusted()
        valid = (sample.sol == 'DOPPLER_VELOCITY' and math.isfinite(sample.speed_mps)
                 and 0 <= sample.speed_mps <= 3 and math.isfinite(sample.track_deg)
                 and (trusted or (self._has_heading() and imu_fresh)))
        if not valid:
            s.doppler_valid_since = None
            invalid_value = (not math.isfinite(sample.speed_mps) or not 0 <= sample.speed_mps <= 3
                             or not math.isfinite(sample.track_deg))
            if sample.sol == 'DOPPLER_VELOCITY' and invalid_value:
                s.doppler_rejected += 1
                return False
            return None
        course = math.radians(90.-sample.track_deg)
        w = s.gyro-s.bias if imu_fresh else 0.
        ax, ay, _ = self._lever()
        forward = sample.speed_mps*math.cos(course-s.yaw)+w*ay
        lateral = sample.speed_mps*math.sin(course-s.yaw)-w*ax
        c, sn = math.cos(s.yaw), math.sin(s.yaw)
        velocity = (forward*c-lateral*sn, forward*sn+lateral*c)
        noise_gate = max(.15, cfg.innovation_gate_sigma*cfg.velocity_noise_mps)
        kinematic_valid = abs(lateral) <= noise_gate
        valid = kinematic_valid
        fresh_wheel = sample.t-s.wheel_t <= cfg.sensor_timeout_s
        if fresh_wheel:
            valid = valid and abs(forward-s.speed*s.wheel_cal.applied) <= noise_gate+cfg.doppler_accel_limit_mps2*cfg.heading_sync_s
        elif s.doppler_vector is None and not trusted:
            valid = False
        # 卫星可信、地速断过一阵：旧矢量不代表现在，不拿它卡跳变（否则没有轮速时一次拒绝就再也接不上）
        if s.doppler_vector is not None and not (trusted and sample.t-s.doppler_t > cfg.sensor_timeout_s):
            dt = sample.t-s.doppler_t
            # 拒绝帧不能延长门限时段，让持续错误地速过一阵又被当成正确速度。
            if not fresh_wheel or dt <= cfg.sensor_timeout_s:
                jump = math.hypot(velocity[0]-s.doppler_vector[0], velocity[1]-s.doppler_vector[1])
                kinematic_valid = kinematic_valid and jump <= noise_gate+cfg.doppler_accel_limit_mps2*min(cfg.sensor_timeout_s,max(0.,dt))
                valid = valid and kinematic_valid
        if not valid:
            s.doppler_rejected += 1
            s.doppler_valid_since = None
            return kinematic_valid
        if s.doppler_valid_since is None or sample.t-s.doppler_t > cfg.sensor_timeout_s:
            s.doppler_valid_since = sample.t
        s.doppler_speed, s.doppler_vector, s.doppler_t = forward, velocity, sample.t
        return True

    def _velocity(self, sample):
        s, cfg = self._s, self.config
        if cfg.sensor_failover_enabled and self._doppler(sample) is False:
            return  # 地速本身不合理，不能再拿它判定轮子打滑或学习标定。
        # 地速按测量时刻进入同一历史快照；坏解或旧地速不能证明车已停稳。
        if (sample.sol == 'DOPPLER_VELOCITY' and math.isfinite(sample.speed_mps)
                and 0 <= sample.speed_mps <= 3 and sample.t-s.fixed_t <= cfg.hold_watch_s):
            # RTK 位置被双天线互检降权，不代表独立多普勒地速失效。
            # 保留上面的地速运动学校验、固定解新鲜度及 pose() 的 0.35s 过期限制。
            s.ground_speed,s.ground_t = sample.speed_mps,sample.t
        # 只在可信卫星仍在时检查共同空转；失联期间不能虚构这份独立证据。
        if (s.wheel_source != 'legacy' and sample.sol in _VELOCITY
                and math.isfinite(sample.speed_mps) and sample.speed_mps >= 0
                and sample.t-s.fixed_t <= cfg.hold_watch_s and sample.t-s.weak_t > cfg.fixed_timeout_s
                and sample.t-s.wheel_t <= cfg.sensor_timeout_s and abs(s.gyro-s.bias) < .05):
            difference = abs(sample.speed_mps-abs(s.speed*s.wheel_cal.applied))
            if difference > max(.15,.3*sample.speed_mps,3*s.wheel_sigma):
                if s.velocity_motion_since is None:s.velocity_motion_since=sample.t
                if sample.t-s.velocity_motion_since>=.2:
                    s.motion_conflict_t=sample.t
                    s.motion_error_rate=max(s.motion_error_rate,difference)
                    s.wheel_cal=s.wheel_cal.clear()
            else:
                s.velocity_motion_since=None
        else:
            s.velocity_motion_since=None
        if not (sample.sol in _VELOCITY and math.isfinite(sample.speed_mps)
                and math.isfinite(sample.track_deg) and sample.speed_mps > cfg.course_min_speed_mps
                and self._straight_ready()):
            return
        if abs(sample.speed_mps - abs(s.speed * s.wheel_cal.applied)) > max(.15, .3*sample.speed_mps):
            return
        angle = _wrap(math.radians(90.-sample.track_deg) + (math.pi if s.speed < 0 else 0.))
        var = math.atan2(cfg.velocity_noise_mps, sample.speed_mps) ** 2
        s.velocity_t = sample.t
        if self._has_heading():
            offset = _wrap(angle-s.raw_yaw)
            if s.vel_n >= 20 and abs(_wrap(offset-math.atan2(s.vel_s, s.vel_c))) > max(math.radians(5.), cfg.innovation_gate_sigma*math.sqrt(var)):
                return
            s.vel_n += 1
            s.vel_c += math.cos(offset)
            s.vel_s += math.sin(offset)
            s.vel_var_sum += var+s.raw_heading_var
        elif self._yaw_observation(angle, var, update_bias=not cfg.drive_differential):
            s.course_t = sample.t

    def pose(self, t: float) -> FusedPose | None:
        if not math.isfinite(t) or not self._s.position_ready or (not self._keys and self._base['t'] is None):
            return None
        # 卫星可信时多给些外推：轮速、IMU 都断了，滤波器只靠 10 Hz 卫星往前走（见 TRUSTED_EXTRAPOLATION_S）
        horizon = TRUSTED_EXTRAPOLATION_S if self._trusted() else self.config.max_extrapolation_s
        if t > self._s.t + horizon + 1e-9 or t < self._base_key[0]:
            return None
        latest = self._s
        index = bisect.bisect_right(self._keys, (t, 5))
        snapshot = self._snapshots[index-1] if index else self._base
        if snapshot['t'] is None or not snapshot['position_ready']:
            return None
        self._restore(snapshot)
        try:
            self._predict(t)
            return self._pose()
        finally:
            self._s = latest

    def _pose(self):
        s, cfg = self._s, self.config
        reasons = []
        sigma_xy = math.sqrt(max(s.p[0], s.p[4], 0.))
        sigma_yaw = math.sqrt(max(s.p[7], 0.))
        quality = s.position_quality
        if s.t - s.fix_t >= cfg.fixed_timeout_s - 1e-9:
            quality = 'DEAD_RECKONING'
        usable = True
        if not s.yaw_ready:
            quality = 'INVALID'
            reasons.append('heading_uninitialized')
            usable = False
        if not self._has_heading():
            reasons.append('heading_unavailable')
            if s.t - s.course_t <= cfg.heading_timeout_s:
                reasons.append('course_heading')
            else:
                reasons.append('gyro_only')
        holding = False
        if cfg.rtk_hold_s > 0:
            # 多久没有固定解（单点、浮点、转发断流）不超过 rtk_hold_s 照常能开（位置靠轮速 + IMU 航向推算，
            # 航向保持断开时的方向），超过才不可控、原地停，恢复后自动继续（见 FusionConfig.rtk_hold_s；
            # 2026-10-05 用户再次确认：「最多维持 5 秒，5 秒内没恢复正常，停车」）
            since = s.t - s.fixed_t
            if cfg.hold_budget_enabled and cfg.hold_watch_s <= since < cfg.fixed_timeout_s:
                reasons.append('rtk_hold_pending')
            if since >= cfg.fixed_timeout_s - 1e-9:
                if since <= cfg.rtk_hold_s + 1e-9:
                    holding = True
                    reasons.append('rtk_hold')
                else:
                    reasons.append('rtk_fixed_timeout')
                    usable = False
            if s.t - s.rejoin_t <= cfg.reset_hold_s:
                reasons.append('rtk_rejoin')        # 刚按 RTK 对齐接回：只提示，不停车
            if s.t - s.weak_t <= cfg.fixed_timeout_s:
                # 卫星质量差、固定解降了权（位置多靠推算）：只提示，固定解照常在用
                reasons.append('rtk_weighted')
        elif s.t - s.fixed_t >= cfg.fixed_timeout_s - 1e-9:
            reasons.append('rtk_fixed_timeout')
            usable = False
        if s.fix_reason:
            reasons.append(s.fix_reason)
        if quality == 'RTK_FLOAT':
            reasons.append('rtk_float')
        if sigma_xy > cfg.control_sigma_xy_m:
            reasons.append('position_uncertain')
            # 推算维持期间位置不确定度照常在涨，维持的时限由 rtk_hold_s 管。固定解刚断、还没满 fixed_timeout_s（0.5 s）的
            # 那几拍也一样：不确定度约 4.5 cm/s 涨，0.45 s 左右就过 3 cm，原来这几拍判不可控 → 跟踪暂停、停车再起步
            # （2026-10-07 仿真：遮挡多的 100 m 线 16 趟暂停 141 次里 124 次是这个；用户：「不能停车，丝滑切换，也不能停喷」）
            if not holding and not (cfg.rtk_hold_s > 0 and s.t - s.fixed_t <= cfg.rtk_hold_s + 1e-9):
                usable = False
        if sigma_yaw > math.radians(cfg.control_sigma_yaw_deg):
            reasons.append('heading_uncertain')
            # 测向断了、基线不对时 IMU 无缝接上航向（2026-10-05 用户：「能不能无感 imu 接上继续走，不能停」）：
            # IMU 正常就照常能开，只提示；IMU 也不能用时才要有效测向
            imu_ok = not s.imu_mismatch and s.t - s.imu_valid_t <= cfg.sensor_timeout_s
            if not imu_ok and not (self._has_heading() and sigma_yaw <= math.radians(HEADING_HOLD_SIGMA_DEG)):
                usable = False
        # IMU 断了或不再用：双天线测向有效就照常能开（航向以测向为主），没有测向才不可控
        imu_down = s.imu_mismatch or s.t - s.imu_valid_t > cfg.sensor_timeout_s + 1e-9
        if s.imu_mismatch:
            reasons.append('imu_mismatch')
        elif imu_down:
            reasons.append('imu_stale')
        if imu_down and not self._has_heading():
            usable = False
        if s.t - s.heading_reset_t <= cfg.reset_hold_s:
            reasons.append('heading_reset')         # 刚按测向重定了航向，只提示，不停车
        if cfg.tilt_compensation_enabled and cfg.antenna_height_m > 0 and self._lever()[2] is not None:
            reasons.append('tilt_uncompensated')    # 填了天线高度但这会儿没做倾斜补偿：只提示，不停车
        wheel_replaced = self._wheel_replacement()
        if s.t - s.wheel_t > cfg.wheel_timeout_s + 1e-9:
            reasons.append('wheel_hold' if wheel_replaced else 'wheel_stale')
            if not wheel_replaced:
                usable = False
        if cfg.sensor_failover_enabled and imu_down:
            if (s.t-s.imu_valid_t <= cfg.sensor_hold_s and self._has_heading()
                    and s.t-s.fixed_t < cfg.hold_watch_s and s.t-s.wheel_t <= cfg.sensor_timeout_s):
                reasons.append('imu_hold')
            else:
                usable = False
                reasons.append('imu_hold_timeout' if s.t-s.imu_valid_t > cfg.sensor_hold_s else 'sensor_pair_unavailable')
        if s.t-s.motion_conflict_t <= 1.0:
            reasons.append('wheel_motion_conflict')
            if s.t-s.fixed_t >= cfg.hold_watch_s:
                usable = False
        if (s.t - s.reset_t <= cfg.reset_hold_s
                or self._reset_seen <= s.t <= self._reset_visible_until):
            reasons.append('position_reset')
            usable = False
        x, y, yaw = s.x, s.y, s.yaw
        correction, heading_correction = (0.0, 0.0), 0.0
        if cfg.continuous_output and s.output_ready:
            x, y, yaw = s.output_x, s.output_y, s.output_yaw
            correction = (s.x - x, s.y - y)
            heading_correction = _wrap(s.yaw - yaw)
            if math.hypot(*correction) > 0.001 or abs(heading_correction) > math.radians(0.05):
                reasons.append('gnss_alignment')
        trusted = self._trusted() and s.yaw_ready and s.position_ready
        if trusted:
            # 用户规则（见 TRUSTED_FRESH_S）：双固定 + 测向基线 0.30–0.50 m 就以卫星为准，轮速、IMU 断了或对不上、
            # 互检、推算预算都不算定位错误：照常可用，等级报固定解，不带任何原因
            quality, usable, reasons = 'RTK_FIXED', True, []
        return FusedPose(s.t, x, y, _wrap(yaw), s.doppler_speed if wheel_replaced else s.speed*s.wheel_cal.applied,
                         s.wheel_yaw if imu_down and cfg.sensor_failover_enabled else s.gyro-s.bias, sigma_xy, sigma_yaw, s.bias, self._mount,
                         quality, usable, tuple(reasons),
                         (s.p[0], s.p[1], s.p[2], s.p[4], s.p[5], s.p[7]),
                         max(0.0, s.t - s.fixed_t), s.hold_distance, s.hold_turn,
                         math.sqrt(max(0.0, s.p[9])), correction, heading_correction,
                         s.wheel_cal.error if cfg.wheel_scale_auto else None,
                         s.wheel_time_uncertainty, s.motion_error, s.motion_error_rate,
                         'gnss_doppler' if wheel_replaced else 'wheel', 'wheel' if imu_down else 'imu',
                         max(0., s.t-s.wheel_t) if wheel_replaced else max(0., s.t-s.imu_valid_t) if imu_down else 0.,
                         max(cfg.wheel_noise_mps, s.wheel_sigma)*math.sqrt(cfg.wheel_sample_period_s)
                         if cfg.sensor_failover_enabled and s.wheel_source == 'encoder'
                         and s.t-s.wheel_t <= cfg.sensor_timeout_s and not imu_down else None,
                         s.hall_still if 0 <= s.t-s.wheel_t <= cfg.sensor_timeout_s else None,
                         (s.ground_speed <= .02) if 0 <= s.t-s.ground_t <= max(cfg.sensor_timeout_s,STATIONARY_DOPPLER_FRESH_S) else None,
                         s.motion_evidence, gnss_still_required=cfg.ground_speed_still,
                         single_antenna=cfg.course_align_distance_m > 0, rtk_trusted=trusted)

    def antenna_lever(self):
        """现在用的 ANT1 水平杠杆臂（车体系前、左，m，含倾斜补偿）；line_mission 核对定位跳动时用同一个。"""
        ax, ay, _ = self._lever()
        return ax, ay

    def tilt_status(self) -> dict:
        """倾斜补偿的状态：天线高度、是否在补、没补的原因、用的横滚俯仰（度），
        以及补偿量（天线水平位置比平地时偏了多少，车体系前、左，m）。"""
        s, cfg = self._s, self.config
        ax, ay, reason = self._lever()
        active = reason is None
        roll, pitch = self.tilt_angles(s.roll, s.pitch)
        return {'height_m': cfg.antenna_height_m, 'active': active, 'reason': reason,
                'enabled': cfg.tilt_compensation_enabled, 'imu_zero_supported': True,
                'zeroed': self._tilt_zeroed,
                'zero_roll_deg': math.degrees(self._tilt_zero[0]),
                'zero_pitch_deg': math.degrees(self._tilt_zero[1]),
                'roll_deg': math.degrees(roll) if active else None,
                'pitch_deg': math.degrees(pitch) if active else None,
                'forward_m': ax - cfg.antenna_xy[0] if active else None,
                'left_m': ay - cfg.antenna_xy[1] if active else None}

    def calibration(self) -> dict:
        s, cfg = self._s, self.config
        c, si, weight = s.mount_sum_c, s.mount_sum_s, s.mount_weight
        scale_sum, scale_weight = s.scale_sum, s.scale_weight
        samples, segments = s.cal_samples, s.cal_segments
        active = self._segment_estimate()
        if active is not None:
            angle, var, scale, scale_var, n, scale_n = active
            if angle is not None:
                c += math.cos(angle)/var
                si += math.sin(angle)/var
                weight += 1/var
                samples += n
            scale_sum += scale/scale_var
            scale_weight += 1/scale_var
            segments += 1
        source = 'rtk_position'
        if weight == 0 and s.vel_n >= 20:
            var = max(math.radians(cfg.calibration_floor_deg)**2, s.vel_var_sum/s.vel_n**2)
            angle = math.atan2(s.vel_s, s.vel_c)
            c, si, weight, samples = math.cos(angle)/var, math.sin(angle)/var, 1/var, s.vel_n
            source = 'bestvel'
        sigma = math.degrees(math.sqrt(1/weight)) if weight else None
        wheel_sigma = math.sqrt(1/scale_weight) if scale_weight else None
        wheel_samples = s.cal_scale_samples + (active[5] if active else 0)
        return {
            'position_variance_scale': s.position_variance_scale,
            'mount_offset_deg': math.degrees(math.atan2(si, c)) if weight else None,
            'mount_offset_sigma_deg': sigma,
            'mount_offset_ci95_deg': 1.96*sigma if sigma is not None else None,
            'mount_offset_samples': samples,
            'mount_offset_confident': samples >= 100 and sigma is not None and 1.96*sigma <= .2,
            'mount_offset_source': source if weight else None,
            'applied_mount_offset_deg': cfg.mount_offset_deg,
            'wheel_scale': scale_sum/scale_weight if scale_weight else None,
            'wheel_scale_sigma': wheel_sigma,
            'wheel_scale_ci95': 1.96*wheel_sigma if wheel_sigma is not None else None,
            'wheel_scale_samples': wheel_samples,
            'wheel_scale_confident': wheel_samples >= 100 and wheel_sigma is not None and 1.96*wheel_sigma <= .003,
            'wheel_motion': {'source': s.wheel_source, 'applied_scale': s.wheel_cal.applied,
                             'scale_error': s.wheel_cal.error, 'scale_blocks': s.wheel_cal.blocks,
                             'scale_rejected': s.wheel_cal.rejected,
                             'time_uncertainty_s': s.wheel_time_uncertainty,
                             'conflict': s.t is not None and s.t-s.motion_conflict_t <= 1.0,
                             'error_mps': s.motion_error_rate},
            'segments': segments,
            'heading_bias_learned_deg': math.degrees(s.heading_bias) if cfg.heading_bias_learning else None,
            'heading_bias_updates': s.heading_bias_updates,
            'gyro_bias_deg_s': math.degrees(s.bias),
            'gyro_bias_sigma_deg_s': math.degrees(math.sqrt(s.p[9])),
            'raw_gyro_bias_deg_s': math.degrees(s.raw_gyro_bias) if cfg.yaw_input == 'angle' else None,
            'heading_filter': {'innovation_deg': math.degrees(s.heading_innovation),
                               'test_ratio': s.heading_test_ratio, 'gain': s.heading_gain,
                               'bias_kind': 'angle_drift' if cfg.yaw_input == 'angle' else 'gyro_bias'},
            'alignment': {'x_m': s.x - s.output_x if cfg.continuous_output and s.output_ready else 0.0,
                          'y_m': s.y - s.output_y if cfg.continuous_output and s.output_ready else 0.0,
                          'yaw_deg': math.degrees(_wrap(s.yaw - s.output_yaw)) if cfg.continuous_output and s.output_ready else 0.0},
            'rejected': {'wheel': self._dropped[0], 'imu': self._dropped[1],
                         'heading': self._dropped[2], 'fix': self._dropped[3], 'velocity': self._dropped[4],
                         'gyro_spikes': s.gyro_rejected, 'heading_outliers': s.heading_rejected,
                         'position_outliers': s.fix_rejected},
            'position_resets': s.resets,
            'rtk_rejoins': s.rejoins,
            'dual_antenna': self._dual.status(),
            'history_samples': len(self._keys),
        }
