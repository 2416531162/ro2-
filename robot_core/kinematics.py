"""Gen5 差速底盘几何（robot_gen5.json）：两后轮差速、前面自由万向轮，能原地转。

2026-11-23 用户「删阿克曼底盘的代码」：ChassisGeometry 只接受 drive = differential。
yaw_from_steer / steer_from_yaw 还给车上网页控制台的方向键（按方向盘角度换算角速度）和
Wheeltec 驱动基类用，max_yaw_at_speed 给驱动基类用（驱动层另行处理）。
"""
import math
from dataclasses import dataclass
from .config import PROFILE

def clamp(value, low, high):
    return max(low, min(high, value))

@dataclass
class ChassisGeometry:
    """Gen5 差速底盘几何参数（共享配置 robot_gen5.json）。

    track_m 是两驱动轮的轮距；wheelbase_m / max_steer_rad 不参与差速运动学，只给网页方向键的
    方向盘角度换算和融合的陀螺合理上限用。
    """
    wheelbase_m: float = PROFILE["geometry"]["wheelbase_m"]
    track_m: float = PROFILE["geometry"]["track_m"]    # 两驱动轮的轮距（轮中心距）
    max_steer_rad: float = PROFILE["geometry"]["max_steer_rad"]
    # 驱动方式（robot_gen5.json 的 drive 段），只有 differential
    drive: str = PROFILE["drive"]["type"]
    diff_min_radius_m: float = PROFILE["drive"]["min_radius_m"]     # 差速车跟踪圆弧的最小半径（内侧轮还有速度）
    pivot_rate_rad_s: float = PROFILE["drive"]["pivot_rate_rad_s"]  # 原地转向最大角速度（规划估时、场地规划限速、上报 app 用）
    pivot_accel_rad_s2: float = PROFILE["drive"]["pivot_accel_rad_s2"]
    # 作业里跟踪器真正原地转的最高角速度（2026-11-21 用户「原地转不顺……那就不要低速，速度再提高点」）：Gen5 0.8 → 1.5 rad/s，
    # 每个轮子 0.8×0.3375 = 0.27 m/s → 约 0.5 m/s。0.27 m/s 正在驱动器低速跑快区（11-19 架空：≤0.3 m/s 实际快 1.6–1.85×、
    # 右驱 0.31–0.54 测速来回切；0.5 m/s 两轮 100%），11-21 实车 5 次约 170° 原地转实际转速峰值是命令的 1.65–2.3 倍、
    # 匀速段 ±0.16–0.27 rad/s 忽快忽慢。只改作业原地转；规划估时、场地规划、手动摇杆、标定转圈仍按 pivot_rate_rad_s
    job_pivot_rate_rad_s: float = PROFILE["drive"]["job_pivot_rate_rad_s"]
    # 作业原地转收尾减速上限（rad/s²）：减速曲线仍按 pivot_accel 规划，实际允许降得这么快，转得快了也停得准（Gen5 2.0）
    job_pivot_brake_rad_s2: float = PROFILE["drive"]["job_pivot_brake_rad_s2"]

    def __post_init__(self):
        if self.wheelbase_m <= 0:
            raise ValueError("wheelbase_m 必须为正")
        if self.track_m < 0:
            raise ValueError("track_m 不能为负")
        if not 0 < self.max_steer_rad < math.pi / 2:
            raise ValueError("max_steer_rad 超出合理范围")
        if self.drive != "differential":
            raise ValueError("drive 只能是 differential（阿克曼车 2026-11-23 删了）")
        if min(self.diff_min_radius_m, self.pivot_rate_rad_s, self.pivot_accel_rad_s2, self.job_pivot_rate_rad_s) <= 0 \
                or self.job_pivot_brake_rad_s2 < self.pivot_accel_rad_s2:
            raise ValueError("差速底盘的最小半径、原地转向角速度和角加速度必须为正")

    @property
    def differential(self):
        """两轮差速（能原地转）：恒为 True。只剩 Wheeltec/OID 驱动还在读，驱动层清理时一起删。"""
        return True

    @property
    def min_turn_radius_m(self):
        """跟踪圆弧的最小半径（原地转向另算）。"""
        return self.diff_min_radius_m


def differential_wheels(vx, wz, track_m):
    """差速车 (车速, 角速度) → (左轮, 右轮) 线速度；左转 wz > 0 时右轮快。"""
    half = 0.5 * track_m
    return vx - wz * half, vx + wz * half


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
