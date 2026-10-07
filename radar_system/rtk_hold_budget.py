"""喷头在 RTK 推算维持及绝对坐标平滑校准期间的误差预算和停车预估。

预算采用随机协方差及明确配置的系统误差余量，不能保证覆盖不可观测的打滑/共同卫星漂移。
路线只提供误差方向，不作为位置观测。不把车的位置吸附到路线。
"""
import math
from dataclasses import asdict, dataclass


HOLD_REASONS = ('rtk_hold', 'rtk_hold_pending', 'rtk_suspect', 'rtk_hold_timeout', 'rtk_fixed_timeout', 'gnss_displacement_conflict')


class DegradedSpeedLimiter:
    """短时预算波动只收紧；持续允许的限速可缓慢恢复，避免起步被锁在零速。"""
    RECOVERY_S = 1.0
    RECOVERY_ACCEL_MPS2 = .2

    def __init__(self):
        self.reset()

    def reset(self):
        self.limit = None
        self.healthy_since = None
        self.last_time = None
        self.allowed_since = self.allowed_ceiling = None

    def update(self, now, requested, *, healthy, cruise, previous_speed, dt):
        if self.last_time is not None and not 0 <= now-self.last_time <= .2+1e-9:
            self.healthy_since = None  # 没有连续控制观测的时间不能算作定位已稳定。
            self.allowed_since = self.allowed_ceiling = None
        self.last_time = now
        if requested is not None:
            requested = max(0., min(cruise, requested))
            if self.limit is None:
                # 降级发生在低速行驶/起步中时也不能因限速值较大而反向加速。
                self.limit = min(requested, abs(previous_speed))
            else:
                self.limit = min(self.limit, requested)
        if self.limit is None:
            return None
        if not healthy:
            self.healthy_since = None
            if requested is not None and requested > self.limit+1e-9:
                # 仍在降级但每拍都明确允许行驶：连续一秒确认后恢复到窗口内
                # 最保守的限速。更严预算/零速/观测缺口立即打断，不借偶尔好帧加速。
                if (self.allowed_since is None or now < self.allowed_since
                        or requested < self.allowed_ceiling-1e-9):
                    self.allowed_since, self.allowed_ceiling = now, requested
                if now-self.allowed_since >= self.RECOVERY_S-1e-9:
                    self.limit = min(self.allowed_ceiling, requested,
                                     self.limit+self.RECOVERY_ACCEL_MPS2*max(0., min(.2, dt)))
            else:
                self.allowed_since = self.allowed_ceiling = None
            return self.limit
        self.allowed_since = self.allowed_ceiling = None
        if self.healthy_since is None or now < self.healthy_since:
            self.healthy_since = now
        if now-self.healthy_since < self.RECOVERY_S-1e-9:
            return self.limit
        ceiling = cruise if requested is None else requested
        self.limit = min(ceiling, self.limit+self.RECOVERY_ACCEL_MPS2*max(0., min(.2, dt)))
        if self.limit >= cruise-1e-9:
            self.reset()
            return None
        return self.limit


def _correction(pose):
    """绝对估计减连续控制输出；旧接口没有这些字段时表示没有待校准差值。"""
    xy = getattr(pose, 'correction_xy', (0.0, 0.0))
    yaw = getattr(pose, 'correction_yaw', 0.0)
    try:
        if len(xy) != 2 or not all(math.isfinite(v) for v in (*xy, yaw)):
            return None
        return xy[0], xy[1], yaw
    except (TypeError, ValueError, OverflowError):
        return None


def needs_hold_budget(pose):
    """小于提示阈值的校准差也不能漏记；非法数据必须进入预算并停车。"""
    correction = _correction(pose)
    return (correction is None or any(correction)
            or any(r in getattr(pose, 'reasons', ()) for r in HOLD_REASONS))


def alignment_unconfirmed(pose, position_limit_m, yaw_limit_rad):
    """起步确认允许预算内的小校准差；提示标记本身不应把自动恢复锁死。"""
    correction = _correction(pose)
    return (correction is None or math.hypot(correction[0], correction[1]) > position_limit_m
            or abs(correction[2]) > yaw_limit_rad)


@dataclass(frozen=True)
class HoldDecision:
    state: str
    reason: str
    lateral_m: float = 0.0
    along_m: float = 0.0
    stop_lateral_m: float = 0.0
    stop_along_m: float = 0.0
    lateral_limit_m: float = 0.0
    along_limit_m: float = 0.0
    age_s: float = 0.0
    distance_m: float = 0.0
    stop_time_s: float = 0.0
    stop_distance_m: float = 0.0
    speed_limit_mps: float | None = None
    correction_lateral_m: float = 0.0
    correction_along_m: float = 0.0
    correction_yaw_rad: float = 0.0

    @property
    def stop(self):
        return self.state == 'stop'

    def status(self):
        return asdict(self)


def evaluate_hold(pose, cfg, *, nozzle_x, nozzle_y=0.0, route_yaw, cruise_mps,
                  speed_peak, turn_peak, delay_s, decel_mps2, angular_decel):
    """中断预算从最后接受的固定解起算；恢复后的未消除校准差单独计入。

    停车包含已下发指令的执行延迟、制动过程和一个控制周期。

    投影 J P Jᵀ 保留位置/航向交叉项；未来的随机增长以标准差相加取保守估计。
    轮速比例误差按累计路程、IMU 刻度余量按累计转角估计，滑移余量按时间/路程/转角累计。
    """
    if not getattr(cfg, 'hold_budget_enabled', False) or cfg.rtk_hold_s <= 0:
        return None
    correction = _correction(pose)
    if correction is None:
        return HoldDecision('stop', 'rtk_hold_budget_missing', speed_limit_mps=0.0)
    reasons = tuple(getattr(pose, 'reasons', ()))
    holding = any(r in reasons for r in HOLD_REASONS)
    if not holding and not any(correction):
        return None
    if not all(hasattr(pose, name) for name in ('hold_age_s', 'hold_distance_m', 'hold_turn_rad',
                                               'covariance_xy_yaw', 'bias_sigma_rad_s')):
        return HoldDecision('stop', 'rtk_hold_budget_missing', speed_limit_mps=0.0)
    fields = (pose.hold_age_s, pose.hold_distance_m, pose.hold_turn_rad, pose.bias_sigma_rad_s, pose.yaw, nozzle_x, nozzle_y, route_yaw,
              cruise_mps, speed_peak, turn_peak, delay_s, decel_mps2, angular_decel)
    p = pose.covariance_xy_yaw
    if (len(p) != 6 or not all(math.isfinite(v) for v in (*fields, *p))
            or min(pose.hold_age_s, pose.hold_distance_m, pose.hold_turn_rad, pose.bias_sigma_rad_s, cruise_mps,
                   speed_peak, turn_peak, delay_s) < 0 or min(decel_mps2, angular_decel) <= 0):
        return HoldDecision('stop', 'rtk_hold_budget_missing', speed_limit_mps=0.0)
    xx, xy, xh, yy, yh, hh = p
    u, n = (math.cos(route_yaw), math.sin(route_yaw)), (-math.sin(route_yaw), math.cos(route_yaw))
    dx, dy, correction_yaw = correction
    cg, sg = math.cos(pose.yaw + correction_yaw), math.sin(pose.yaw + correction_yaw)
    cl, sl = math.cos(pose.yaw), math.sin(pose.yaw)
    # 同一个喷头在绝对估计和控制输出中的坐标差，用实际杆臂而非小角度近似。
    dx += nozzle_x * (cg - cl) - nozzle_y * (sg - sl)
    dy += nozzle_x * (sg - sl) + nozzle_y * (cg - cl)
    correction_along = abs(dx * u[0] + dy * u[1])
    correction_lateral = abs(dx * n[0] + dy * n[1])
    # 协方差属于绝对估计，喷头雅可比也必须使用绝对航向。
    # 喷头车体系为（前、左）；保留侧置喷头对航向误差的杠杆效应。
    jx = -nozzle_x * sg - nozzle_y * cg
    jy = nozzle_x * cg - nozzle_y * sg
    def variance(direction):
        dx, dy = direction
        a = dx * jx + dy * jy
        return dx*dx*xx + 2*dx*dy*xy + dy*dy*yy + 2*a*(dx*xh + dy*yh) + a*a*hh
    va, vn = variance(u), variance(n)
    if min(xx, yy, hh, va, vn) < -1e-12:
        return HoldDecision('stop', 'rtk_hold_budget_missing', speed_limit_mps=0.0)
    sa, sn, sh = math.sqrt(max(0.0, va)), math.sqrt(max(0.0, vn)), math.sqrt(max(0.0, hh))
    stop_time = delay_s + max(speed_peak / decel_mps2, turn_peak / angular_decel) if max(speed_peak, turn_peak) > 1e-6 else 0.0
    stop_distance = speed_peak * delay_s + speed_peak * speed_peak / (2 * decel_mps2)
    stop_turn = turn_peak * delay_s + turn_peak * turn_peak / (2 * angular_decel)
    signed_relative = math.atan2(math.sin(pose.yaw-route_yaw), math.cos(pose.yaw-route_yaw))
    relative = abs(math.atan2(math.sin(signed_relative + correction_yaw), math.cos(signed_relative + correction_yaw)))
    relative = min(relative, abs(math.pi-relative))  # 前进/倒车使用相同误差轴。
    # 恢复有固定解后，校准差仍计预算，但旧的中断时限与累积推算余量已经结束。
    age = pose.hold_age_s if holding else 0.0
    held_distance = pose.hold_distance_m if holding else 0.0
    held_turn = pose.hold_turn_rad if holding else 0.0
    wheel_error = getattr(pose, 'wheel_scale_error', None)
    wheel_error = cfg.hold_wheel_scale_error if wheel_error is None else wheel_error
    wheel_timing = getattr(pose, 'wheel_time_uncertainty_s', 0.0)
    motion_error = getattr(pose, 'motion_error_m', 0.0)
    motion_rate = getattr(pose, 'motion_error_mps', 0.0)
    if not all(math.isfinite(v) and v >= 0 for v in (wheel_error, wheel_timing, motion_error, motion_rate)):
        return HoldDecision('stop', 'rtk_hold_budget_missing', speed_limit_mps=0.0)

    prediction_walk = getattr(pose, 'prediction_along_walk', None)
    if prediction_walk is not None and (not math.isfinite(prediction_walk) or prediction_walk < 0):
        return HoldDecision('stop', 'rtk_hold_budget_missing', speed_limit_mps=0.0)

    def bounds(time, distance, turn):
        elapsed, travelled, turned = age + time, held_distance + distance, held_turn + turn
        # 停车前继续移动时，尚未消除的航向校准差也会产生位移误差。
        correction_motion_n = distance * max(abs(math.sin(correction_yaw)),
                                             abs(math.sin(signed_relative + correction_yaw) - math.sin(signed_relative)))
        correction_motion_a = distance * abs(math.cos(signed_relative + correction_yaw) - math.cos(signed_relative))
        if not holding:
            # 可信GNSS仍持续校准，不能假设整个制动期间都会失联。保留当前随机协方差，
            # 只外推控制输出尚未吸收的系统校准差；推算中断时才使用下方完整过程余量。
            return (cfg.hold_sigma_multiplier * sn + correction_lateral + correction_motion_n,
                    cfg.hold_sigma_multiplier * sa + correction_along + correction_motion_a)
        # 随机预测噪声沿车头/侧向不同；转动期间用较大的位置游走作停车预测余量。
        along_walk = prediction_walk
        if along_walk is None:
            along_walk = cfg.position_walk_m_sqrt_s
            cross_walk = min(along_walk, cfg.cross_walk_m_sqrt_s+cfg.cross_walk_turn*turn_peak)
        else:
            # 区间轮程与 IMU 接管：速度噪声、航向协方差、滑移余量各算一次。
            # 下方 slip/angle_scale 仍随累计路程、转角增长；不是假定轮胎不会打滑。
            along_walk = max(along_walk, cfg.cross_walk_m_sqrt_s)
            cross_walk = cfg.cross_walk_m_sqrt_s
        ca, cr = math.cos(relative), math.sin(relative)
        qa = math.hypot(along_walk * ca, cross_walk * cr)
        qn = math.hypot(along_walk * cr, cross_walk * ca)
        if turn > 0.01:
            # 维持期间加停车过程中车头转过的角度：沿车头的游走按转到的最大夹角投到横向，转到 90° 时与原来整个当横向一样。
            # 原来超过 0.01 rad 就整个当横向：0.8 m/s 直线上 0.03 rad/s 的正常小修正，停车过程预计转 0.012 rad，
            # 横向游走就从 0.0015 跳到 0.02 m/√s（13 倍），2026-10-03 实车蓝牙 RTK 晚到 0.24 s 预估横向从 2.0 跳到 5.6 cm 判停车
            reach = min(math.pi / 2, relative + turned)
            qa = along_walk
            qn = max(qn, math.hypot(along_walk * math.sin(reach), cross_walk * math.cos(reach)))
        yaw_q = math.radians(cfg.yaw_walk_deg_sqrt_s)**2 + math.radians(cfg.gyro_noise_deg_s)**2 * cfg.gyro_sample_period_s
        yaw_growth = sh + pose.bias_sigma_rad_s * time + math.sqrt(yaw_q * time)
        yaw_along = distance * yaw_growth * math.sin(min(math.pi/2, relative + turn))
        if cfg.sensor_failover_enabled:
            # 将同一个 x/y/yaw 协方差向刹停位置传播一次。旧式 sn + distance*sh
            # 把已经含在 sn 中的航向误差再次当独立最坏值叠加，丢掉了位置-航向相关项。
            signed_distance = math.copysign(distance, pose.speed) if abs(pose.speed) > 1e-6 else distance
            def projected(direction):
                dx, dy = direction
                a = dx*(jx-sg*signed_distance) + dy*(jy+cg*signed_distance)
                return max(0., dx*dx*xx+2*dx*dy*xy+dy*dy*yy+2*a*(dx*xh+dy*yh)+a*a*hh)
            future_yaw = pose.bias_sigma_rad_s*time + math.sqrt(yaw_q*time)
            future_n = distance*future_yaw
            future_a = future_n*math.sin(min(math.pi/2, relative+turn))
            random_a = cfg.hold_sigma_multiplier*math.hypot(math.sqrt(projected(u))+future_a, qa*math.sqrt(time))
            random_n = cfg.hold_sigma_multiplier*math.hypot(math.sqrt(projected(n))+future_n, qn*math.sqrt(time))
        else:
            random_a = cfg.hold_sigma_multiplier * (sa + yaw_along + qa * math.sqrt(time))
            random_n = cfg.hold_sigma_multiplier * (sn + distance * yaw_growth + qn * math.sqrt(time))
        wheel = travelled * wheel_error + speed_peak * wheel_timing
        slip = cfg.hold_slip_mps * elapsed + cfg.hold_slip_per_m * travelled + cfg.hold_turn_slip_m_per_rad * turned
        slip += motion_error + motion_rate*time
        angle_scale = travelled * cfg.hold_turn_scale_error * turned
        # 转弯时轮速刻度误差也可能横向积累，按已转/将转的绝对角度放宽。
        wheel_cross = wheel * math.sin(min(math.pi / 2, relative + turned))
        return (random_n + wheel_cross + slip + angle_scale + correction_lateral + correction_motion_n,
                random_a + wheel + slip + angle_scale + correction_along + correction_motion_a)

    lateral, along = bounds(0.0, 0.0, 0.0)
    stop_lateral, stop_along = bounds(stop_time, stop_distance, stop_turn)
    ratio = max(stop_lateral / cfg.hold_lateral_limit_m, stop_along / cfg.hold_along_limit_m)
    state, reason, cap = 'maintain', '', None
    if holding and age + stop_time >= cfg.rtk_hold_s:
        state, reason, cap = 'stop', 'rtk_hold_braking_timeout', 0.0
    elif ratio >= 1.0:
        state, reason, cap = 'stop', 'rtk_hold_budget_exceeded', 0.0
    elif ratio > cfg.hold_slow_ratio:
        cap = cruise_mps * (1.0 - ratio) / (1.0 - cfg.hold_slow_ratio)
        state, reason = 'slow', 'rtk_hold_budget_slow'
        if cap < 0.05:
            if cfg.sensor_failover_enabled:
                # ratio < 1 已核对当前实际速度的整个刹停过程仍在预算内。
                # 线性限速只是提前减速启发式，不能把小于最小爬行速度误当成预算已越界。
                cap = min(cruise_mps, 0.05)
            else:
                state, reason, cap = 'stop', 'rtk_hold_budget_exceeded', 0.0
    return HoldDecision(state, reason, lateral, along, stop_lateral, stop_along,
                        cfg.hold_lateral_limit_m, cfg.hold_along_limit_m,
                        age, held_distance, stop_time, stop_distance, cap,
                        correction_lateral, correction_along, correction_yaw)
