#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
共享运动安全层 (Motion Safety Layer)
====================================

这是跟随节点与网页遥控共用的唯一一套「速度该是多少」的权威计算。
纯 Python,不依赖 ROS / 串口 / 硬件,因此可以直接跑单元测试。

为什么要有这一层
----------------
改造前 person_follower.py 和 radar_web_server.py 各自拍脑袋算速度,
各自往 /cmd_vel 上发,限幅规则不一致,而且都没考虑两件物理事实:

  1. 感知到指令生效之间有 0.2~0.4 秒的死时间 (相机 + 推理 + 滤波 + 控制周期);
  2. 阿克曼车没有主动刹车,PWM 归零后靠惯性滑行还要跑一段。

所以「到了阈值再停」必然撞。正确做法是把**距离换算成允许的最大速度**,
远处就开始收油,这就是本模块的 brake_envelope()。

坐标与符号约定
--------------
  vx > 0 前进,vx < 0 倒车
  wz > 0 左转,wz < 0 右转
  steer > 0 左打舵,steer < 0 右打舵
"""

import math
from dataclasses import dataclass

__all__ = [
    "ChassisGeometry", "BrakeProfile",
    "brake_envelope", "stopping_distance",
    "yaw_from_steer", "steer_from_yaw", "max_yaw_at_speed",
    "AlphaBetaTracker", "SlewLimiter", "BreakawayKick",
    "clamp",
]


def clamp(value, low, high):
    return low if value < low else (high if value > high else value)


# --------------------------------------------------------------------------
# 1. 底盘几何
# --------------------------------------------------------------------------

@dataclass
class ChassisGeometry:
    """轮趣阿克曼底盘几何参数。

    下位机固件 (见 wheeltec_protocol/PROTOCOL.md 第八节) 的真实算法是:

        TurnR = Vx / Vz
        Angle_Left = atan(AxleSpacing / (TurnR - 0.5 * WheelSpacing))

    注意它算的是**左前轮**转角,不是自行车模型的中心线转角。
    本模块严格复刻这个公式,这样上位机要求的转角和舵机实际转角才对得上。
    """
    wheelbase_m: float = 0.25      # AxleSpacing 前后轴距
    track_m: float = 0.17          # WheelSpacing 左右轮距
    max_steer_rad: float = 0.35    # 舵机物理限位 ≈ 20°

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


def max_yaw_at_speed(speed_mps, geometry):
    """当前车速下物理上能达到的最大横摆角速度 (满舵)。"""
    if abs(speed_mps) < 1e-9:
        return 0.0
    return abs(speed_mps) / geometry.min_turn_radius_m


# --------------------------------------------------------------------------
# 2. 刹车包络 —— 撞人问题的正解
# --------------------------------------------------------------------------

@dataclass
class BrakeProfile:
    """把「还剩多少距离」换算成「现在最多能跑多快」。

    decel_mps2   实测可达的减速度。阿克曼车没有主动刹车,松油门靠滑行,
                 光滑地面上通常只有 0.8~1.5 m/s^2,务必实车标定。
    latency_s    从传感器成像到车轮真正开始减速的总死时间。
                 相机曝光 + YOLO 推理 + 滤波滞后 + 控制周期 + 串口 + 固件。
    stop_m       速度归零点。车头应当停在这个距离上。
    hard_stop_m  硬急停线。越过这条线无条件发 0,是最后一道保险,不是主力。
    """
    decel_mps2: float = 1.0
    latency_s: float = 0.35
    stop_m: float = 0.70
    hard_stop_m: float = 0.40

    def __post_init__(self):
        if self.decel_mps2 <= 0:
            raise ValueError("decel_mps2 必须为正")
        if self.latency_s < 0:
            raise ValueError("latency_s 不能为负")
        if self.hard_stop_m > self.stop_m:
            raise ValueError("hard_stop_m 必须小于等于 stop_m,否则硬急停永远先触发")


def brake_envelope(distance_m, profile):
    """距离 -> 允许的最大前进速度。

    推导:设当前速度 v,死时间 T 内匀速前进 v*T,之后以 a 减速需要 v^2/(2a)。
    要求两段之和不超过可用余量 d - stop_m:

        v*T + v^2/(2a) <= gap
        =>  v <= -a*T + sqrt((a*T)^2 + 2*a*gap)

    这条曲线在 gap=0 处速度为 0,且导数连续,所以车是**平滑收油**而不是断崖刹停。

    >>> p = BrakeProfile(decel_mps2=1.0, latency_s=0.35, stop_m=0.70)
    >>> round(brake_envelope(1.50, p), 3)
    0.962
    >>> round(brake_envelope(0.80, p), 3)
    0.218
    >>> brake_envelope(0.70, p)
    0.0
    >>> brake_envelope(0.30, p)
    0.0
    """
    if not math.isfinite(distance_m):
        return 0.0
    gap = distance_m - profile.stop_m
    if gap <= 0.0:
        return 0.0
    at = profile.decel_mps2 * profile.latency_s
    return max(0.0, math.sqrt(at * at + 2.0 * profile.decel_mps2 * gap) - at)


def stopping_distance(speed_mps, profile):
    """反过来:以当前速度跑,从下决心到停住一共要多少米。用于日志与体检。"""
    v = abs(speed_mps)
    return v * profile.latency_s + v * v / (2.0 * profile.decel_mps2)


# --------------------------------------------------------------------------
# 3. alpha-beta 跟踪滤波器 —— 替换掉有严重相位滞后的 EMA
# --------------------------------------------------------------------------

class AlphaBetaTracker:
    """恒速模型的 alpha-beta 滤波器,同时输出位置与**速度**。

    改造前用的是 EMA: smooth = 0.65*old + 0.35*new。
    EMA 只平滑位置,代价是引入约 2.4 帧的相位滞后 —— 10fps 下就是 240ms,
    车以 0.32 m/s 靠近时,滤波器读到的距离比真实距离**大 7~8 厘米**,
    这部分误差直接变成了撞击。

    alpha-beta 因为显式维护速度项,对匀速运动的目标是**零稳态滞后**,
    而且顺手给出目标速度,可以拿来做前馈跟速。
    """

    def __init__(self, alpha=0.45, beta=0.10, max_dt_s=0.5):
        if not 0 < alpha <= 1:
            raise ValueError("alpha 应在 (0, 1]")
        if not 0 <= beta <= 2:
            raise ValueError("beta 应在 [0, 2]")
        self.alpha = alpha
        self.beta = beta
        self.max_dt_s = max_dt_s
        self.position = None
        self.velocity = 0.0
        self._last_t = None

    def reset(self):
        self.position = None
        self.velocity = 0.0
        self._last_t = None

    @property
    def initialized(self):
        return self.position is not None

    def update(self, measurement, now):
        """喂一帧观测,返回滤波后的位置。"""
        if self.position is None or self._last_t is None:
            self.position = float(measurement)
            self.velocity = 0.0
            self._last_t = now
            return self.position

        dt = now - self._last_t
        self._last_t = now
        if dt <= 0.0:
            return self.position
        if dt > self.max_dt_s:
            # 间隔过长说明目标丢过,速度估计已不可信,重新起算
            self.position = float(measurement)
            self.velocity = 0.0
            return self.position

        predicted = self.position + self.velocity * dt
        residual = measurement - predicted
        self.position = predicted + self.alpha * residual
        self.velocity += self.beta * residual / dt
        return self.position

    def predict(self, horizon_s):
        """外推 horizon_s 秒后的位置。用来抵消剩余的感知死时间。"""
        if self.position is None:
            return None
        return self.position + self.velocity * horizon_s


# --------------------------------------------------------------------------
# 4. 斜坡限幅 —— 加速和减速必须分开
# --------------------------------------------------------------------------

class SlewLimiter:
    """对指令做加减速斜坡限幅。

    刹车永远允许比加速更陡:起步舒适度可以让,刹车距离不能让。
    """

    def __init__(self, accel_limit, decel_limit, initial=0.0):
        if accel_limit <= 0 or decel_limit <= 0:
            raise ValueError("加减速限制必须为正")
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit
        self.value = initial

    def reset(self, value=0.0):
        self.value = value

    def step(self, target, dt):
        if dt <= 0:
            return self.value
        # 「更靠近 0」一律算减速,包括倒车时的减速
        shrinking = abs(target) < abs(self.value) or self.value * target < 0
        limit = (self.decel_limit if shrinking else self.accel_limit) * dt
        delta = clamp(target - self.value, -limit, limit)
        self.value += delta
        if abs(self.value) < 1e-4:
            self.value = 0.0
        return self.value


# --------------------------------------------------------------------------
# 5. 静摩擦破除脉冲 —— 取代原来那个害人的 MIN_SPEED 地板值
# --------------------------------------------------------------------------

class BreakawayKick:
    """起步时给一小段高于静摩擦门限的推力,之后立刻交回正常控制律。

    改造前的写法是 `vx = max(MIN_SPEED_MPS, ...)`,把 0.25 m/s 做成了
    **全程速度下限**。后果是车在死区边界上永远以 0.32 m/s 撞过去,
    这是撞人的直接原因。

    真正需要 0.25 m/s 的只有「从静止起步的那一瞬间」。
    所以把它做成有时限的脉冲,而且脉冲本身仍然要服从刹车包络 ——
    快贴到人了就不该有任何起步冲动。
    """

    def __init__(self, kick_mps=0.22, duration_s=0.25, creep_floor_mps=0.08):
        self.kick_mps = kick_mps
        self.duration_s = duration_s
        self.creep_floor_mps = creep_floor_mps
        self._kick_until = 0.0
        self._was_moving = False

    def apply(self, desired_mps, measured_moving, speed_cap, now):
        """返回修正后的期望速度。

        desired_mps     控制律算出来的期望速度
        measured_moving 车目前是否真的在动 (来自底盘遥测,没有就传 False)
        speed_cap       刹车包络给出的速度上限,脉冲绝不允许突破它
        """
        if desired_mps <= self.creep_floor_mps:
            self._kick_until = 0.0
            self._was_moving = measured_moving
            return 0.0 if desired_mps < self.creep_floor_mps else desired_mps

        if not measured_moving and not self._was_moving:
            if self._kick_until == 0.0:
                self._kick_until = now + self.duration_s
            if now < self._kick_until:
                desired_mps = max(desired_mps, self.kick_mps)
        else:
            self._kick_until = 0.0

        self._was_moving = measured_moving
        return min(desired_mps, speed_cap)


if __name__ == "__main__":
    import doctest
    failures, _ = doctest.testmod(verbose=False)
    geo = ChassisGeometry()
    profile = BrakeProfile()
    print("满舵最小转弯半径: %.3f m" % geo.min_turn_radius_m)
    print("\n距离 -> 允许速度 (刹车包络)")
    for d in (2.0, 1.5, 1.2, 1.0, 0.9, 0.8, 0.75, 0.70, 0.6):
        print("  %.2f m -> %.3f m/s" % (d, brake_envelope(d, profile)))
    print("\n改造前网页各档位的实际前轮转角")
    for name, vx, wz in (("low 左拐", 0.35, 0.60), ("med 左拐", 0.50, 0.80),
                         ("high 左拐", 0.65, 0.95), ("med 前左", 0.75, 0.65)):
        deg = math.degrees(steer_from_yaw(vx, wz, geo))
        cap = math.degrees(geo.max_steer_rad)
        print("  %-10s vx=%.2f wz=%.2f -> %.1f deg%s"
              % (name, vx, wz, deg, "  (已打满)" if deg >= cap - 0.05 else ""))
    raise SystemExit(1 if failures else 0)
