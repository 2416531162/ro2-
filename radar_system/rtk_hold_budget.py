"""喷头在 RTK 推算维持期间的误差预算和停车预估；纯计算，不读取硬件或更改滤波状态。

预算采用随机协方差及明确配置的系统误差余量，不能保证覆盖不可观测的打滑/共同卫星漂移。
路线只提供误差方向，不作为位置观测。不把车的位置吸附到路线。
"""
import math
from dataclasses import asdict, dataclass


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

    @property
    def stop(self):
        return self.state == 'stop'

    def status(self):
        return asdict(self)


def evaluate_hold(pose, cfg, *, nozzle_x, nozzle_y=0.0, route_yaw, cruise_mps,
                  speed_peak, turn_peak, delay_s, decel_mps2, angular_decel):
    """预算从最后接受的固定解起算。停车包含已下发指令的执行延迟、制动过程和一个控制周期。

    投影 J P Jᵀ 保留位置/航向交叉项；未来的随机增长以标准差相加取保守估计。
    轮速比例误差按累计路程、IMU 刻度余量按累计转角估计，滑移余量按时间/路程/转角累计。
    """
    if not getattr(cfg, 'hold_budget_enabled', False) or cfg.rtk_hold_s <= 0:
        return None
    reasons = tuple(pose.reasons)
    if not any(r in reasons for r in ('rtk_hold', 'rtk_hold_pending', 'rtk_suspect', 'rtk_hold_timeout', 'rtk_fixed_timeout')):
        return None
    if not all(hasattr(pose, name) for name in ('hold_age_s', 'hold_distance_m', 'hold_turn_rad',
                                               'covariance_xy_yaw', 'bias_sigma_rad_s')):
        return HoldDecision('stop', 'rtk_hold_budget_missing', speed_limit_mps=0.0)
    fields = (pose.hold_age_s, pose.hold_distance_m, pose.hold_turn_rad, pose.bias_sigma_rad_s, nozzle_x, nozzle_y, route_yaw,
              cruise_mps, speed_peak, turn_peak, delay_s, decel_mps2, angular_decel)
    p = pose.covariance_xy_yaw
    if (len(p) != 6 or not all(math.isfinite(v) for v in (*fields, *p))
            or min(pose.hold_age_s, pose.hold_distance_m, pose.hold_turn_rad, pose.bias_sigma_rad_s, cruise_mps,
                   speed_peak, turn_peak, delay_s) < 0 or min(decel_mps2, angular_decel) <= 0):
        return HoldDecision('stop', 'rtk_hold_budget_missing', speed_limit_mps=0.0)
    xx, xy, xh, yy, yh, hh = p
    u, n = (math.cos(route_yaw), math.sin(route_yaw)), (-math.sin(route_yaw), math.cos(route_yaw))
    # 喷头车体系为（前、左）；保留侧置喷头对航向误差的杠杆效应。
    jx = -nozzle_x * math.sin(pose.yaw) - nozzle_y * math.cos(pose.yaw)
    jy = nozzle_x * math.cos(pose.yaw) - nozzle_y * math.sin(pose.yaw)
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
    relative = abs(math.atan2(math.sin(pose.yaw-route_yaw), math.cos(pose.yaw-route_yaw)))
    relative = min(relative, abs(math.pi-relative))  # 前进/倒车使用相同误差轴。

    def bounds(time, distance, turn):
        elapsed, travelled, turned = pose.hold_age_s + time, pose.hold_distance_m + distance, pose.hold_turn_rad + turn
        # 随机预测噪声沿车头/侧向不同；转动期间用较大的位置游走作停车预测余量。
        cross_walk = min(cfg.position_walk_m_sqrt_s,
                         cfg.cross_walk_m_sqrt_s + cfg.cross_walk_turn * turn_peak)
        ca, cr = math.cos(relative), math.sin(relative)
        qa = math.hypot(cfg.position_walk_m_sqrt_s * ca, cross_walk * cr)
        qn = math.hypot(cfg.position_walk_m_sqrt_s * cr, cross_walk * ca)
        if turn > 0.01:
            qa = qn = cfg.position_walk_m_sqrt_s
        yaw_q = math.radians(cfg.yaw_walk_deg_sqrt_s)**2 + math.radians(cfg.gyro_noise_deg_s)**2 * cfg.gyro_sample_period_s
        yaw_growth = sh + pose.bias_sigma_rad_s * time + math.sqrt(yaw_q * time)
        yaw_along = distance * yaw_growth * math.sin(min(math.pi/2, relative + turn))
        random_a = cfg.hold_sigma_multiplier * (sa + yaw_along + qa * math.sqrt(time))
        random_n = cfg.hold_sigma_multiplier * (sn + distance * yaw_growth + qn * math.sqrt(time))
        wheel = travelled * cfg.hold_wheel_scale_error
        slip = cfg.hold_slip_mps * elapsed + cfg.hold_slip_per_m * travelled + cfg.hold_turn_slip_m_per_rad * turned
        angle_scale = travelled * cfg.hold_turn_scale_error * turned
        # 转弯时轮速刻度误差也可能横向积累，按已转/将转的绝对角度放宽。
        wheel_cross = wheel * math.sin(min(math.pi / 2, relative + turned))
        return random_n + wheel_cross + slip + angle_scale, random_a + wheel + slip + angle_scale

    lateral, along = bounds(0.0, 0.0, 0.0)
    stop_lateral, stop_along = bounds(stop_time, stop_distance, stop_turn)
    ratio = max(stop_lateral / cfg.hold_lateral_limit_m, stop_along / cfg.hold_along_limit_m)
    state, reason, cap = 'maintain', '', None
    if pose.hold_age_s + stop_time >= cfg.rtk_hold_s:
        state, reason, cap = 'stop', 'rtk_hold_braking_timeout', 0.0
    elif ratio >= 1.0:
        state, reason, cap = 'stop', 'rtk_hold_budget_exceeded', 0.0
    elif ratio > cfg.hold_slow_ratio:
        cap = cruise_mps * (1.0 - ratio) / (1.0 - cfg.hold_slow_ratio)
        state, reason = 'slow', 'rtk_hold_budget_slow'
        if cap < 0.05:
            state, reason, cap = 'stop', 'rtk_hold_budget_exceeded', 0.0
    return HoldDecision(state, reason, lateral, along, stop_lateral, stop_along,
                        cfg.hold_lateral_limit_m, cfg.hold_along_limit_m,
                        pose.hold_age_s, pose.hold_distance_m, stop_time, stop_distance, cap)
