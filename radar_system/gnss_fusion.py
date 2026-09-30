"""后轴中心的 RTK / 双天线 / 陀螺 / 轮速融合；不依赖 ROS。填了天线高度时按 IMU 横滚俯仰做倾斜补偿。

时间戳必须已经换算到同一个单调时钟。有限历史内按测量时间重放，
标定统计也参与回放，避免延迟观测改变结果或重复累计样本。
"""
import bisect
import math
from dataclasses import dataclass


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
    # 都被当离群值拒掉，最后定位重置、停车。航向角清零 / 传感器重启一帧就是几百 °/s 以上，照样丢
    angle_rate_floor_deg_s: float = 70.0
    # 转过的角度的比例误差（陀螺 / IMU 航向刻度）：转多少，航向不确定度就按这个比例加多少。0 = 不加（旧车）。
    # 不加时原地转 90° 后滤波器自认航向很准，会把比它偏 0.5° 以上的正确测向当离群值一直拒掉
    yaw_scale_sigma: float = 0.0
    # 测向与 IMU 的时间同步误差：测向误差按「角速度 × 这个时间」加大，转得快时少信测向。0 = 不加（旧车）
    heading_sync_s: float = 0.0
    wheel_scale: float = 1.0
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
    float_floor_m: float = 0.15
    innovation_gate_sigma: float = 5.0
    # 固定解位置离群（归一化新息超过这么多 σ）不整个丢掉，按离群程度放大方差：影响有界，
    # 也不会连丢 0.5 s 后一步跳到 RTK 上
    soft_gate_sigma: float = 2.0
    # 固定解连续离群这么久、且差距超过 reset_distance_m：多半是推算错了（车被推动、打滑），照旧对齐到 RTK
    # 并标记重置（控制暂停等操作员）。短时跳动（实车约 1 s）不触发
    outlier_reset_s: float = 2.0
    # 卫星位置不可信时只靠推算维持（2026-09-30 用户规则：「IMU 航向 5 秒内肯定没问题」「卫星定位的问题，为什么要信卫星，
    # 此时应立马无缝衔接 IMU 继续走 5 秒」）。0 = 旧做法：固定解中断 fixed_timeout_s 就不可控，持续离群
    # outlier_reset_s 后对齐到 RTK 并暂停。> 0：
    # - 双天线位置互检不过（卫星位置错了）、没有固定解：位置只按轮速 + 航向推算，不用 RTK、不停车，最多这么久；
    #   超时才不可控（停车）。又有可信的固定解（互检过或比不了）就按 RTK 对齐接回，不停车
    # - 固定解与推算对不上但互检过了：照旧按软门限用（离得远降权，推算有轮速刻度误差，不能完全不用）；持续对不上
    #   是推算走偏了，按 RTK 对齐但不停车。互检比不了（没有副天线）时照旧对齐并暂停
    # 2026-09-30 19:37 实车：主天线固定解错了 20–100 cm（接收机照报 NARROW_INT，两根天线位置之差对不上测向基线），
    # 旧做法离群 2 s 后对齐到错的位置，横偏一下跳 14 cm，车拐过去追，5 s 一次
    rtk_hold_s: float = 0.0
    # 双天线位置互检：主、副天线固定解位置之差与测向基线（长度 × 方向）差超过 dual_check_m 就不信固定解，
    # 降到 dual_ok_m 以下保持 dual_ok_s 才恢复；副天线 dual_stale_s 没有可比的数据就不按它判。
    # 2026-09-30 实车：正常时中位数 1–3 cm、最大约 5 cm；出错时 15–107 cm
    dual_check_m: float = 0.10
    dual_ok_m: float = 0.06
    dual_ok_s: float = 1.0
    dual_stale_s: float = 1.0
    gyro_margin_deg_s: float = 0.5
    stationary_speed_mps: float = 0.002
    stationary_time_s: float = 1.0
    straight_rate_deg_s: float = 0.15
    straight_time_s: float = 0.5
    course_min_speed_mps: float = 0.3
    velocity_noise_mps: float = 0.03
    heading_timeout_s: float = 0.3
    sensor_timeout_s: float = 0.2
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
    calibration_floor_deg: float = 0.03

    def __post_init__(self):
        if self.yaw_input not in ('rate', 'angle'):
            raise ValueError("yaw_input 必须是 'rate' 或 'angle'")
        for name, value in self.__dict__.items():
            if name == 'yaw_input':
                continue
            if name == 'antenna_xy':
                if len(value) != 2 or not all(math.isfinite(v) for v in value):
                    raise ValueError('antenna_xy 必须是两个有限数值')
            elif not math.isfinite(value):
                raise ValueError(name + ' 必须是有限数值')
            elif name in ('yaw_scale_sigma', 'heading_sync_s', 'antenna_height_m', 'rtk_hold_s'):
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
        if self.dual_ok_m >= self.dual_check_m:
            raise ValueError('dual_ok_m 必须小于 dual_check_m')


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


def _wrap(angle):
    value = (angle + math.pi) % (2 * math.pi) - math.pi
    return math.pi if value == -math.pi else value


_A = 6378137.0
_F = 1 / 298.257223563
_E2 = _F * (2 - _F)
# 航向以双天线为主（2026-09-28 用户：RTK 航向稳定就必须信它，不能因为 IMU 不稳定停车）。
# 差速车 IMU 与轮速里程计转向相反的判据：两边都明显在转、方向相反持续 MISMATCH_S 秒。判出后到重启都不再用 IMU
# （装反、坐标换算错不会自己好），航向交给双天线（两次测向之间用轮速里程计推）；双天线测向有效就照常能开。
MISMATCH_WHEEL_RAD_S = 0.2
MISMATCH_IMU_RAD_S = 0.1
MISMATCH_S = 0.2
# 双天线测向（整周固定、有效）连续被新息门限拒掉 REANCHOR_S 秒、这些测向彼此差不超过 REANCHOR_SPREAD_DEG
# （稳定）、车转得不快（REANCHOR_RATE_DEG_S 以下，转得快时测向的时间误差大）：是推算的航向错了，按测向重定
REANCHOR_S = 0.5
REANCHOR_SPREAD_DEG = 0.5
REANCHOR_RATE_DEG_S = 3.0
# 双天线测向有效（刚收下过）时，航向不确定度到这么多都照常能开：这时的不确定度多半是原地转向、IMU 断了时按保守
# 模型涨的，航向有测向拉着（与 line_mission 的 GYRO_HOLD_SIGMA_YAW_DEG 相同）
HEADING_HOLD_SIGMA_DEG = 2.0
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
    """主、副天线固定解位置互检：两根天线装在同一车架上，两个固定解位置之差必须等于测向基线（长度 × 方向）。
    对不上说明至少一根天线的固定解错了（接收机照样报 NARROW_INT），这时位置不能信 RTK。

    副天线 5 Hz、主天线和测向 20 Hz（2026-09-30 实车）：副天线每来一帧，把主天线位置按时间插值到同一时刻再比。
    按到达顺序处理（不参与融合的历史重放）；融合在主天线固定解到达时读 state_at，把结果跟样本一起存下。"""

    def __init__(self, config):
        self.cfg = config
        self._primary = []          # (t, x, y)，最近 2 s
        self._secondary = []        # 还没比过的副天线 (t, x, y)
        self._heads = []            # (t, 北零顺时针 rad, 水平基线长 m)，最近 2 s
        self.suspect = False
        self.last_dev = None        # 最近一次比出的偏差，m
        self.last_t = None          # 最近一次比的时刻
        self.checks = self.flags = 0
        self._ok_since = None

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
        cfg, keep = self.cfg, []
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
            self.checks += 1
            if dev > cfg.dual_check_m:
                if not self.suspect:
                    self.flags += 1
                self.suspect, self._ok_since = True, None
            elif dev < cfg.dual_ok_m:
                if self.suspect:
                    self._ok_since = t if self._ok_since is None else self._ok_since
                    if t - self._ok_since >= cfg.dual_ok_s - 1e-9:
                        self.suspect, self._ok_since = False, None
            else:
                self._ok_since = None
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

    def state_at(self, t):
        """t 时刻主天线固定解的互检结果：'suspect'（不可信）/ 'ok'（过了）/ None（最近 dual_stale_s 没比过：
        副天线断了，不按它判）。"""
        if self.last_t is None or t - self.last_t > self.cfg.dual_stale_s:
            return None
        return 'suspect' if self.suspect else 'ok'

    def status(self):
        return {'suspect': self.suspect, 'dev_m': None if self.last_dev is None else round(self.last_dev, 4),
                'checks': self.checks, 'flags': self.flags}


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
        self.heading_t = self.velocity_t = self.reset_t = -math.inf
        self.heading_ok = False
        self.raw_yaw = 0.0
        self.raw_heading_var = 1.0
        self.position_quality = 'INVALID'
        self.fix_reason = 'position_uninitialized'
        self.stationary_since = self.straight_since = None
        self.segment = None
        self.mount_sum_c = self.mount_sum_s = self.mount_weight = 0.0
        self.scale_sum = self.scale_weight = 0.0
        self.cal_samples = self.cal_scale_samples = self.cal_segments = 0
        self.vel_n = 0
        self.vel_c = self.vel_s = self.vel_var_sum = 0.0
        self.heading_rejected = self.fix_rejected = self.gyro_rejected = self.resets = 0
        self.outlier_since = None
        self.course_t = -math.inf
        self.imu_yaw = None                 # 上一帧 IMU 航向角（yaw_input = 'angle'）
        self.mismatch_since = None          # 差速车：IMU 转向与轮速里程计相反从什么时候开始
        self.imu_mismatch = False           # 确认相反后一直为真，IMU 不再用
        self.rej_since = None               # 测向连续被拒从什么时候开始；rej_ref 第一帧，rej_lo/hi 之后相对它的范围
        self.rej_ref = self.rej_lo = self.rej_hi = 0.0
        self.heading_reset_t = -math.inf
        self.heading_resets = 0
        self.roll = self.pitch = 0.0        # 最近一帧 IMU 姿态（倾斜补偿），tilt_t 是它的时刻
        self.tilt_t = -math.inf
        # 推算维持（rtk_hold_s > 0）：最近一次按 RTK 对齐接回的时刻、次数；互检不过丢掉的固定解帧数
        self.rejoin_t = -math.inf
        self.rejoins = self.fix_suspect = 0


class GnssFusion:
    def __init__(self, config: FusionConfig = FusionConfig()):
        self.config = config
        self._frame = None
        self._s = _State(config)
        self._keys, self._events, self._snapshots = [], [], []
        self._base = self._s.__dict__.copy()
        self._base_key = (-math.inf, -1)
        self._last = [-math.inf] * 5
        self._dropped = [0] * 5
        self._reset_seen = self._reset_visible_until = -math.inf
        self._mount = math.radians(config.mount_offset_deg)
        self._baseline = math.radians(config.baseline_yaw_body_deg)
        self._gyro_q = (math.radians(config.gyro_noise_deg_s) ** 2 * config.gyro_sample_period_s
                        + math.radians(config.yaw_walk_deg_sqrt_s) ** 2)
        self._bias_q = math.radians(config.bias_walk_deg_s_sqrt_s) ** 2
        self._dual = DualAntennaCheck(config)

    def set_origin(self, lat_deg, lon_deg, height_m=0.0):
        frame = _Frame(lat_deg, lon_deg, height_m)
        if self._keys or self._s.position_ready:
            raise ValueError('已有观测后不能更换原点；请新建 GnssFusion')
        self._frame = frame

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
        if all(math.isfinite(v) for v in (s.t, s.speed, s.yaw_rate)):
            self._add(0, s)

    def add_imu(self, s: ImuSample):
        if math.isfinite(s.t) and math.isfinite(s.gyro_z):
            self._add(1, s)

    def add_heading(self, s: DualHeading):
        if math.isfinite(s.t):
            if s.sol in _HEADING:
                self._dual.add_heading(s.t, s.heading_deg, s.length_m, s.pitch_deg)
            self._add(2, s)

    def add_fix2(self, s: GnssFix):
        """副天线定位（UM982 BESTNAVHA）：只用来和主天线互检，不直接进融合。"""
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
        if self._frame is None and valid:
            self._frame = _Frame(s.lat_deg, s.lon_deg, s.height_m)
        xy = self._frame.local(s.lat_deg, s.lon_deg) if valid else None
        if xy is not None and s.sol in _FIXED:
            # 互检的结果（到达时的）跟样本一起存下，历史重放时不变
            self._dual.add_primary(s.t, *xy)
            xy = (xy[0], xy[1], self._dual.state_at(s.t) if self.config.rtk_hold_s > 0 else None)
        self._add(3, s, xy)

    def add_velocity(self, s: GnssVelocity):
        if math.isfinite(s.t):
            self._add(4, s)

    def _restore(self, snapshot):
        self._s = _State.__new__(_State)
        self._s.__dict__ = snapshot.copy()

    def _add(self, kind, sample, xy=None):
        key = (sample.t, kind)
        if sample.t <= self._last[kind] or key <= self._base_key:
            self._dropped[kind] += 1
            return
        if self._s.t is not None and sample.t < self._s.t - self.config.history_s:
            self._dropped[kind] += 1
            return
        self._last[kind] = sample.t
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
        if index < len(self._keys) - 1:
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
                if wheel is not None and fs.t-wheel.t <= self.config.sensor_timeout_s:
                    bound = self._gyro_bound(wheel.speed, wheel.yaw_rate, fs.t - self._s.imu_t)
                    if abs(self._yaw_rate(fs)-self._s.bias) <= bound:
                        future_gyro = fs
            self._predict(s.t, future_gyro)
            if k == 0:
                self._wheel(s)
            elif k == 1:
                self._imu(s)
            elif k == 2:
                self._heading(s)
            elif k == 3:
                self._fix(s, pos)
            else:
                self._velocity(s)
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

    def _predict(self, t, future_gyro=None):
        s, cfg = self._s, self.config
        if s.t is None:
            s.t = t
            return
        dt = t - s.t
        if dt <= 0:
            return
        # 输入中断后不无限外推旧油门或旧角速度；不确定性仍随整个 dt 增长。
        dv = min(dt, max(0., s.wheel_t + cfg.sensor_timeout_s - s.t))
        imu_ok = not s.imu_mismatch
        dg = min(dt, max(0., s.imu_t + cfg.sensor_timeout_s - s.t)) if imu_ok else 0.
        w = s.gyro - s.bias
        stationary = (s.stationary_since is not None
                      and s.t-s.stationary_since >= cfg.stationary_time_s
                      and s.t-s.wheel_t <= cfg.sensor_timeout_s)
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
        do = max(0., dv - dg) if cfg.drive_differential else 0.
        angle += s.wheel_yaw * do
        half = angle / 2
        distance = s.speed * cfg.wheel_scale * dv
        sinc = math.sin(half) / half if abs(half) > 1e-8 else 1.0
        mid = s.yaw + half
        dx, dy = distance * sinc * math.cos(mid), distance * sinc * math.sin(mid)
        s.x += dx
        s.y += dy
        s.yaw = _wrap(s.yaw + angle)
        s.odo += s.speed * dv
        a, c, e = -dy, dx, -dg
        b, d = a * e / 2, c * e / 2
        xx, xy, xh, xb, yy, yh, yb, hh, hb, bb = s.p
        q = cfg.position_walk_m_sqrt_s ** 2 * dt
        # 侧向随机游走只在轮速、陀螺都没中断时收紧；中断时推算本身不可信，退回各向同性
        cross = cfg.cross_walk_m_sqrt_s + cfg.cross_walk_turn * abs(angle) / dt
        qc = q if min(dv, dg) < dt else min(q, cross ** 2 * dt)
        qv = (cfg.wheel_noise_mps * dv) ** 2
        cs, sn = math.cos(mid), math.sin(mid)
        qa = q - qc + qv                  # 沿车头方向比侧向多出的部分
        qh = self._gyro_q * dt + (cfg.yaw_scale_sigma * angle) ** 2
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
        s.t = t

    def _observe(self, h, residual, variance, gate=True, update_attitude=True):
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
            s.bias += p3*k
        inv = 1 / innovation
        s.p = (max(1e-16, xx-p0*p0*inv), xy-p0*p1*inv, xh-p0*p2*inv, xb-p0*p3*inv,
               max(1e-16, yy-p1*p1*inv), yh-p1*p2*inv, yb-p1*p3*inv,
               max(1e-16, hh-p2*p2*inv) if update_attitude else hh,
               hb-p2*p3*inv if update_attitude else hb,
               max(1e-20, bb-p3*p3*inv) if update_attitude else bb)
        return True

    def _wheel(self, sample):
        s, cfg = self._s, self.config
        if sample.t - s.wheel_t > cfg.sensor_timeout_s:
            s.stationary_since = None
            self._end_segment()
        still = abs(sample.speed) <= cfg.stationary_speed_mps
        if cfg.drive_differential:          # 差速车原地转时车速为 0，但在转
            still = still and abs(sample.yaw_rate) <= math.radians(cfg.gyro_margin_deg_s)
        if still:
            if s.stationary_since is None:
                s.stationary_since = sample.t
        else:
            s.stationary_since = None
        if s.speed * sample.speed < 0:
            self._end_segment()
        s.speed, s.wheel_yaw, s.wheel_t = sample.speed, sample.yaw_rate, sample.t

    def _gyro_bound(self, speed, wheel_yaw, dt=None):
        """陀螺读数（去零偏）的合理上限，超过的算尖峰不用。

        阿克曼车：当前车速下满舵的角速度。差速车：里程计角速度的 1.5 倍（原地转也有），再加上行驶中路面颠簸、
        打滑带来的转动（按车速，量级与阿克曼车满舵相同）。两种都再加 gyro_margin_deg_s。
        """
        cfg = self.config
        v = abs(speed * cfg.wheel_scale)
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
        return sample.gyro_z

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
                and s.t - s.imu_t <= cfg.sensor_timeout_s
                and abs(s.speed * cfg.wheel_scale) > cfg.course_min_speed_mps
                and abs(s.gyro - s.bias) < math.radians(cfg.straight_rate_deg_s)
                and abs(s.wheel_yaw) < math.radians(cfg.straight_rate_deg_s) + .002)

    def _straight_ready(self):
        s = self._s
        return (self._straight() and s.straight_since is not None
                and s.t - s.straight_since >= self.config.straight_time_s)

    def _has_heading(self):
        return self._s.heading_ok and self._s.t - self._s.heading_t <= self.config.heading_timeout_s

    def _lever(self):
        """ANT1 相对后轴中心地面点的水平杠杆臂，车体系 (前, 左)，m；第三项是没做倾斜补偿的原因（做了为 None）。

        三维杠杆臂 (ax, ay, h) 按 REP-103 姿态 R = Rz(航向)·Ry(俯仰)·Rx(横滚) 转到当地水平面：
        先绕 x 横滚、再绕 y 俯仰，取水平分量（航向由调用方再转）。横滚左侧抬起为正，天线往右偏；
        俯仰车头压低为正，天线往前偏。姿态断了、太陡或 IMU 已不用时退回平地杠杆臂，只降级、不停车。
        """
        s, cfg = self._s, self.config
        ax, ay = cfg.antenna_xy
        h = cfg.antenna_height_m
        if h <= 0:
            return ax, ay, 'no_height'
        if s.imu_mismatch:
            return ax, ay, 'imu_mismatch'
        if s.t is None or s.t - s.tilt_t > cfg.sensor_timeout_s + 1e-9:
            return ax, ay, 'attitude_stale'
        limit = math.radians(cfg.max_tilt_deg)
        if abs(s.roll) > limit or abs(s.pitch) > limit:
            return ax, ay, 'too_steep'
        cr, sr = math.cos(s.roll), math.sin(s.roll)
        cp, sp = math.cos(s.pitch), math.sin(s.pitch)
        return ax * cp + (ay * sr + h * cr) * sp, ay * cr - h * sr, None

    def _yaw_observation(self, yaw, variance):
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
        return self._observe((0., 0., 1., 0.), _wrap(yaw - s.yaw), variance)

    def _heading(self, sample):
        s, cfg = self._s, self.config
        valid = (sample.sol in _HEADING and sample.sats >= cfg.min_sats
                 and all(math.isfinite(v) for v in (sample.heading_deg, sample.pitch_deg,
                                                   sample.length_m, sample.sigma_heading_deg))
                 and sample.sigma_heading_deg >= 0)
        if not valid:
            s.heading_ok = False
            return
        raw = _wrap(math.radians(90. - sample.heading_deg) - self._baseline)
        rate = self._turn_rate()
        var = math.radians(max(cfg.heading_floor_deg, sample.sigma_heading_deg)) ** 2 + (rate * cfg.heading_sync_s) ** 2
        yaw = _wrap(raw + self._mount)
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

    def _turn_rate(self):
        """现在转得多快（rad/s）：IMU 能用就用 IMU，否则差速车用轮速里程计，都没有为 0。"""
        s, cfg = self._s, self.config
        if not s.imu_mismatch and s.t - s.imu_t <= cfg.sensor_timeout_s:
            return s.gyro - s.bias
        if cfg.drive_differential and s.t - s.wheel_t <= cfg.sensor_timeout_s:
            return s.wheel_yaw
        return 0.

    def _fix(self, sample, xy):
        s, cfg = self._s, self.config
        if xy is None:
            s.position_quality = 'DEAD_RECKONING'
            s.fix_reason = 'fix_unavailable'
            self._end_segment()
            return
        fixed = sample.sol in _FIXED
        dual = xy[2] if len(xy) > 2 else None       # 双天线位置互检：'suspect' / 'ok' / None（比不了、没开推算维持）
        if dual == 'suspect':
            # 卫星位置错了（两根天线的固定解对不上测向基线）：这帧不用，位置靠推算；维持多久由 _pose 按 rtk_hold_s 判
            s.fix_suspect += 1
            s.fix_reason = 'rtk_suspect'
            return
        if not fixed:
            s.outlier_since = None
        floor = cfg.fix_floor_m if fixed else cfg.float_floor_m
        re, rn = max(floor, sample.sigma_e) ** 2, max(floor, sample.sigma_n) ** 2
        ax, ay, _ = self._lever()           # 含倾斜补偿：天线在坡上偏出去的那段不算车的位置
        cs, sn = math.cos(s.yaw), math.sin(s.yaw)
        lx, ly = cs*ax-sn*ay, sn*ax+cs*ay
        dx, dy = xy[0]-lx-s.x, xy[1]-ly-s.y
        recovering = s.position_ready and fixed and sample.t - s.fixed_t >= cfg.fixed_timeout_s - 1e-9
        far = math.hypot(dx, dy) > cfg.reset_distance_m
        persistent = (fixed and s.outlier_since is not None
                      and sample.t - s.outlier_since >= cfg.outlier_reset_s - 1e-9)
        reset = (recovering or persistent) and far
        # 推算维持模式：维持后接回、或互检过了还与推算持续对不上（推算走偏了），都按 RTK 对齐但不停车
        rejoin = reset and cfg.rtk_hold_s > 0 and (recovering or dual == 'ok')
        if not s.position_ready or reset:
            s.outlier_since = None
            s.x, s.y = xy[0]-lx, xy[1]-ly
            p = s.p
            s.p = (re+ly*ly*p[7], -lx*ly*p[7], ly*p[7], ly*p[8],
                   rn+lx*lx*p[7], -lx*p[7], -lx*p[8], p[7], p[8], p[9])
            s.position_ready = True
            if rejoin:
                s.rejoin_t = sample.t
                s.rejoins += 1
                self._end_segment()
            elif reset:
                s.reset_t = sample.t
                s.resets += 1
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
            if fixed and nis > cfg.soft_gate_sigma ** 2:
                # 固定解离群：方差按离群程度放大（Huber），离得越远影响越小；持续的偏差几秒内平滑吸收
                inflate = nis / cfg.soft_gate_sigma ** 2
                re, rn = re * inflate, rn * inflate
                s.fix_rejected += 1
                if s.outlier_since is None:
                    s.outlier_since = sample.t
            elif fixed:
                s.outlier_since = None
            # 位置用 Schmidt 更新：保留相关性，但不让位置暗中充当低噪声航向。
            self._observe((1., 0., -ly, 0.), dx, re, gate=False, update_attitude=False)
            cs, sn = math.cos(s.yaw), math.sin(s.yaw)
            lx, ly = cs*ax-sn*ay, sn*ax+cs*ay
            self._observe((0., 1., lx, 0.), xy[1]-s.y-ly, rn, gate=False, update_attitude=False)
        s.fix_t, s.fix_reason = sample.t, ''
        s.position_quality = 'RTK_FIXED' if fixed else 'RTK_FLOAT'
        if fixed:
            s.fixed_t = sample.t
            self._position_course(sample, xy)
        else:
            self._end_segment()

    def _position_course(self, sample, xy):
        s, cfg = self._s, self.config
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
        if not self._has_heading() and abs(s.odo-start) >= cfg.calibration_min_distance_m and voo > 0 and s.t - s.course_t >= 1.:
            angle = math.atan2(voy, vox)
            # 回归窗口有重叠，不能按每个 RTK 帧把它当独立观测注入。
            var = max(math.radians(.2)**2, 4*(sigma_sum/n)/max(voo, 1e-12))
            if self._yaw_observation(angle, var):
                s.course_t = s.t

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

    def _velocity(self, sample):
        s, cfg = self._s, self.config
        if not (sample.sol in _VELOCITY and math.isfinite(sample.speed_mps)
                and math.isfinite(sample.track_deg) and sample.speed_mps > cfg.course_min_speed_mps
                and self._straight_ready()):
            return
        if abs(sample.speed_mps - abs(s.speed * cfg.wheel_scale)) > max(.15, .3*sample.speed_mps):
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
        elif self._yaw_observation(angle, var):
            s.course_t = sample.t

    def pose(self, t: float) -> FusedPose | None:
        if not math.isfinite(t) or not self._s.position_ready or not self._keys:
            return None
        if t > self._s.t + self.config.max_extrapolation_s + 1e-9 or t < self._base_key[0]:
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
            # 推算维持：多久没有可信的固定解。不超过 rtk_hold_s 照常能开（位置靠轮速 + 航向推算），超过才不可控
            since = s.t - s.fixed_t
            if since >= cfg.fixed_timeout_s - 1e-9:
                if since <= cfg.rtk_hold_s + 1e-9:
                    holding = True
                    reasons.append('rtk_hold')
                else:
                    reasons.append('rtk_hold_timeout')
                    usable = False
            if s.t - s.rejoin_t <= cfg.reset_hold_s:
                reasons.append('rtk_rejoin')        # 刚按 RTK 对齐接回：只提示，不停车
        elif s.t - s.fixed_t >= cfg.fixed_timeout_s - 1e-9:
            reasons.append('rtk_fixed_timeout')
            usable = False
        if s.fix_reason:
            reasons.append(s.fix_reason)
        if quality == 'RTK_FLOAT':
            reasons.append('rtk_float')
        if sigma_xy > cfg.control_sigma_xy_m:
            reasons.append('position_uncertain')
            if not holding:                         # 推算维持期间位置不确定度照常在涨，维持的时限由 rtk_hold_s 管
                usable = False
        if sigma_yaw > math.radians(cfg.control_sigma_yaw_deg):
            reasons.append('heading_uncertain')
            if not (self._has_heading() and sigma_yaw <= math.radians(HEADING_HOLD_SIGMA_DEG)):
                usable = False
        # IMU 断了或不再用：双天线测向有效就照常能开（航向以测向为主），没有测向才不可控
        imu_down = s.imu_mismatch or s.t - s.imu_t > cfg.sensor_timeout_s + 1e-9
        if s.imu_mismatch:
            reasons.append('imu_mismatch')
        elif imu_down:
            reasons.append('imu_stale')
        if imu_down and not self._has_heading():
            usable = False
        if s.t - s.heading_reset_t <= cfg.reset_hold_s:
            reasons.append('heading_reset')         # 刚按测向重定了航向，只提示，不停车
        if cfg.antenna_height_m > 0 and self._lever()[2] is not None:
            reasons.append('tilt_uncompensated')    # 填了天线高度但这会儿没做倾斜补偿：只提示，不停车
        if s.t - s.wheel_t > cfg.sensor_timeout_s + 1e-9:
            reasons.append('wheel_stale')
            usable = False
        if (s.t - s.reset_t <= cfg.reset_hold_s
                or self._reset_seen <= s.t <= self._reset_visible_until):
            reasons.append('position_reset')
            usable = False
        return FusedPose(s.t, s.x, s.y, _wrap(s.yaw), s.speed*cfg.wheel_scale,
                         s.gyro-s.bias, sigma_xy, sigma_yaw, s.bias, self._mount,
                         quality, usable, tuple(reasons))

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
        return {'height_m': cfg.antenna_height_m, 'active': active, 'reason': reason,
                'roll_deg': math.degrees(s.roll) if active else None,
                'pitch_deg': math.degrees(s.pitch) if active else None,
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
            'segments': segments,
            'gyro_bias_deg_s': math.degrees(s.bias),
            'gyro_bias_sigma_deg_s': math.degrees(math.sqrt(s.p[9])),
            'rejected': {'wheel': self._dropped[0], 'imu': self._dropped[1],
                         'heading': self._dropped[2], 'fix': self._dropped[3], 'velocity': self._dropped[4],
                         'gyro_spikes': s.gyro_rejected, 'heading_outliers': s.heading_rejected,
                         'position_outliers': s.fix_rejected, 'position_suspect': s.fix_suspect},
            'position_resets': s.resets,
            'rtk_rejoins': s.rejoins,
            'dual_antenna': self._dual.status(),
            'history_samples': len(self._keys),
        }
