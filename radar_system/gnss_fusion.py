"""后轴中心的 RTK / 双天线 / 陀螺 / 轮速融合；不依赖 ROS。

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
    baseline_yaw_body_deg: float = -90.0
    # 不设基线长度：天线间距不写死、不实测，由 rtk_node 按两根天线固定解的平面实时距离核对测向
    mount_offset_deg: float = 0.0
    wheelbase_m: float = 0.52
    max_steer_rad: float = 0.35
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
        for name, value in self.__dict__.items():
            if name == 'antenna_xy':
                if len(value) != 2 or not all(math.isfinite(v) for v in value):
                    raise ValueError('antenna_xy 必须是两个有限数值')
            elif not math.isfinite(value):
                raise ValueError(name + ' 必须是有限数值')
            elif name not in ('baseline_yaw_body_deg', 'mount_offset_deg') and value <= 0:
                raise ValueError(name + ' 必须为正')
        if not 0 < self.max_steer_rad < math.pi / 2:
            raise ValueError('max_steer_rad 必须在 (0, pi/2) 内')
        if not isinstance(self.history_max_samples, int) or self.history_max_samples < 10:
            raise ValueError('history_max_samples 必须是至少 10 的整数')


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
            self._add(2, s)

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
                    bound = abs(wheel.speed*self.config.wheel_scale)*math.tan(self.config.max_steer_rad)/self.config.wheelbase_m
                    bound += math.radians(self.config.gyro_margin_deg_s)
                    if abs(fs.gyro_z-self._s.bias) <= bound:
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
        dg = min(dt, max(0., s.imu_t + cfg.sensor_timeout_s - s.t))
        w = s.gyro - s.bias
        stationary = (s.stationary_since is not None
                      and s.t-s.stationary_since >= cfg.stationary_time_s
                      and s.t-s.wheel_t <= cfg.sensor_timeout_s)
        if future_gyro is not None and s.imu_t > -math.inf and future_gyro.t > s.imu_t and not stationary:
            fraction = (s.t+dg/2-s.imu_t)/(future_gyro.t-s.imu_t)
            w += (future_gyro.gyro_z-s.gyro)*max(0., min(1., fraction))
        angle = w * dg
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
        cross = cfg.cross_walk_m_sqrt_s + cfg.cross_walk_turn * abs(w)
        qc = q if min(dv, dg) < dt else min(q, cross ** 2 * dt)
        qv = (cfg.wheel_noise_mps * dv) ** 2
        cs, sn = math.cos(mid), math.sin(mid)
        qa = q - qc + qv                  # 沿车头方向比侧向多出的部分
        qh = self._gyro_q * dt
        if dg < dt:
            qh += math.radians(5.) ** 2 * (dt - dg)
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
        if abs(sample.speed) <= cfg.stationary_speed_mps:
            if s.stationary_since is None:
                s.stationary_since = sample.t
        else:
            s.stationary_since = None
        if s.speed * sample.speed < 0:
            self._end_segment()
        s.speed, s.wheel_yaw, s.wheel_t = sample.speed, sample.yaw_rate, sample.t

    def _imu(self, sample):
        s, cfg = self._s, self.config
        fresh_wheel = sample.t - s.wheel_t <= cfg.sensor_timeout_s
        bound = abs(s.speed * cfg.wheel_scale) * math.tan(cfg.max_steer_rad) / cfg.wheelbase_m
        bound += math.radians(cfg.gyro_margin_deg_s)
        if fresh_wheel and abs(sample.gyro_z - s.bias) > bound:
            s.gyro_rejected += 1
            s.imu_t = sample.t
            return
        s.gyro, s.imu_t = sample.gyro_z, sample.t
        if fresh_wheel and s.stationary_since is not None and sample.t - s.stationary_since >= cfg.stationary_time_s:
            self._observe((0., 0., 0., 1.), sample.gyro_z - s.bias,
                          math.radians(cfg.gyro_noise_deg_s) ** 2, gate=False)
            s.gyro = s.bias
        if self._straight():
            if s.straight_since is None:
                s.straight_since = sample.t
        else:
            self._end_segment()

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

    def _yaw_observation(self, yaw, variance):
        s = self._s
        if not s.yaw_ready:
            if s.position_ready:
                ax, ay = self.config.antenna_xy
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
        var = math.radians(max(cfg.heading_floor_deg, sample.sigma_heading_deg)) ** 2
        if self._yaw_observation(_wrap(raw + self._mount), var):
            s.heading_t, s.heading_ok, s.raw_yaw, s.raw_heading_var = sample.t, True, raw, var
        else:
            s.heading_rejected += 1

    def _fix(self, sample, xy):
        s, cfg = self._s, self.config
        if xy is None:
            s.position_quality = 'DEAD_RECKONING'
            s.fix_reason = 'fix_unavailable'
            self._end_segment()
            return
        fixed = sample.sol in _FIXED
        if not fixed:
            s.outlier_since = None
        floor = cfg.fix_floor_m if fixed else cfg.float_floor_m
        re, rn = max(floor, sample.sigma_e) ** 2, max(floor, sample.sigma_n) ** 2
        ax, ay = cfg.antenna_xy
        cs, sn = math.cos(s.yaw), math.sin(s.yaw)
        lx, ly = cs*ax-sn*ay, sn*ax+cs*ay
        dx, dy = xy[0]-lx-s.x, xy[1]-ly-s.y
        recovering = s.position_ready and fixed and sample.t - s.fixed_t >= cfg.fixed_timeout_s - 1e-9
        far = math.hypot(dx, dy) > cfg.reset_distance_m
        persistent = (fixed and s.outlier_since is not None
                      and sample.t - s.outlier_since >= cfg.outlier_reset_s - 1e-9)
        reset = (recovering or persistent) and far
        if not s.position_ready or reset:
            s.outlier_since = None
            s.x, s.y = xy[0]-lx, xy[1]-ly
            p = s.p
            s.p = (re+ly*ly*p[7], -lx*ly*p[7], ly*p[7], ly*p[8],
                   rn+lx*lx*p[7], -lx*p[7], -lx*p[8], p[7], p[8], p[9])
            s.position_ready = True
            if reset:
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
        if s.t - s.fixed_t >= cfg.fixed_timeout_s - 1e-9:
            reasons.append('rtk_fixed_timeout')
            usable = False
        if s.fix_reason:
            reasons.append(s.fix_reason)
        if quality == 'RTK_FLOAT':
            reasons.append('rtk_float')
        if sigma_xy > cfg.control_sigma_xy_m:
            reasons.append('position_uncertain')
            usable = False
        if sigma_yaw > math.radians(cfg.control_sigma_yaw_deg):
            reasons.append('heading_uncertain')
            usable = False
        if s.t - s.imu_t > cfg.sensor_timeout_s + 1e-9:
            reasons.append('imu_stale')
            usable = False
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
                         'position_outliers': s.fix_rejected},
            'position_resets': s.resets,
            'history_samples': len(self._keys),
        }
