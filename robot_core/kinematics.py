"""Shared symmetric Ackermann steering convention used by the existing chassis.

All callers use the same wheelbase/track model; firmware validation remains a
hardware acceptance item. SI body yaw is derived from speed and steering.
"""
import math
from dataclasses import dataclass
from .config import PROFILE

def clamp(value, low, high):
    return max(low, min(high, value))

@dataclass
class ChassisGeometry:
    """轮趣阿克曼底盘几何参数。

    沿用项目的对称半轮距补偿模型：
    radius = wheelbase / tan(abs(steer)) + track / 2。
    参考固件记录见 wheeltec_protocol/PROTOCOL.md；实际左右轮定义及
    正反向约定仍须结合当前底盘固件验证，不能仅凭此模型推定已验证。
    """
    # 以下为实测值 (2026-09-16),不是估算。改之前先量车。
    wheelbase_m: float = PROFILE["geometry"]["wheelbase_m"]    # AxleSpacing 前后轴距 (前轮轴心 -> 后轮轴心)
    track_m: float = PROFILE["geometry"]["track_m"]    # WheelSpacing 左右轮距 (轮中心距)
    max_steer_rad: float = PROFILE["geometry"]["max_steer_rad"]    # ★ 舵机物理限位,标称 20°,尚未实测确认

    def __post_init__(self):
        if self.wheelbase_m <= 0:
            raise ValueError("wheelbase_m 必须为正")
        if self.track_m < 0:
            raise ValueError("track_m 不能为负")
        if not 0 < self.max_steer_rad < math.pi / 2:
            raise ValueError("max_steer_rad 超出合理范围")

    @property
    def min_turn_radius_m(self):
        """满舵时的转弯半径。"""
        return self.wheelbase_m / math.tan(self.max_steer_rad) + 0.5 * self.track_m


def yaw_from_steer(speed_mps, steer_rad, geometry):
    """已知车速与期望前轮转角,反算应下发给底盘的横摆角速度。

    这是修复「网页上下左右手感不对」的关键:UI 指定的是转角 (方向盘打多少),
    角速度由当前车速实时算出。速度档位怎么换,转弯半径都不变。
    """
    steer = clamp(steer_rad, -geometry.max_steer_rad, geometry.max_steer_rad)
    if abs(steer) < 1e-9 or abs(speed_mps) < 1e-9:
        return 0.0
    radius = geometry.wheelbase_m / math.tan(abs(steer)) + 0.5 * geometry.track_m
    yaw = abs(speed_mps) / radius
    # 倒车时同样的舵角会让车尾朝相反方向摆,符号需要跟随车速方向
    return math.copysign(yaw, steer) * (1.0 if speed_mps >= 0 else -1.0)


def steer_from_yaw(speed_mps, yaw_radps, geometry):
    """已知车速与横摆角速度,反推前轮转角 (固件内部做的就是这一步)。

    用来检查一条 Twist 指令在物理上是否可实现 —— 改造前网页发的
    (vx=0.5, wz=0.8) 换算出来是 24.8°,超过舵机 20° 限位,三个速度档
    全部打满,档位形同虚设。
    """
    if abs(speed_mps) < 1e-9 or abs(yaw_radps) < 1e-9:
        return 0.0
    radius = abs(speed_mps) / abs(yaw_radps)
    denominator = radius - 0.5 * geometry.track_m
    if denominator <= 1e-6:
        steer = geometry.max_steer_rad
    else:
        steer = math.atan(geometry.wheelbase_m / denominator)
    sign = 1.0 if (yaw_radps >= 0) == (speed_mps >= 0) else -1.0
    return math.copysign(clamp(steer, 0.0, geometry.max_steer_rad), sign)


def pure_pursuit_steer(forward_m, left_m, geometry, min_lookahead_m=0.45, gain=1.0):
    """纯追踪:由目标在车体系里的位置直接解出前轮转角。

    曲率 kappa = 2*sin(alpha)/Ld = 2*left/Ld^2。
    按底盘固件左前轮转角定义折算: TurnR = 1/kappa, tan(steer) = wheelbase / (TurnR - track/2)。
    """
    lookahead = math.hypot(forward_m, left_m)
    if lookahead < min_lookahead_m:
        lookahead = min_lookahead_m
    if abs(left_m) < 1e-9 or lookahead < 1e-6:
        return 0.0
    curvature = gain * 2.0 * left_m / (lookahead * lookahead)
    radius = 1.0 / abs(curvature)
    denominator = radius - 0.5 * geometry.track_m
    if denominator <= 1e-6:
        return math.copysign(geometry.max_steer_rad, left_m)
    steer = math.atan(geometry.wheelbase_m / denominator)
    return math.copysign(clamp(steer, 0.0, geometry.max_steer_rad), left_m)
def max_yaw_at_speed(speed_mps, geometry):
    """当前车速下物理上能达到的最大横摆角速度 (满舵)。"""
    if abs(speed_mps) < 1e-9:
        return 0.0
    return abs(speed_mps) / geometry.min_turn_radius_m
