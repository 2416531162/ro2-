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
from runtime_config import PROFILE

__all__ = [
    "ChassisGeometry", "BrakeProfile",
    "brake_envelope", "stopping_distance",
    "yaw_from_steer", "steer_from_yaw", "max_yaw_at_speed", "pure_pursuit_steer",
    "AlphaBetaTracker", "SlewLimiter", "BreakawayKick",
    "TargetLock", "reconcile_range", "ScanSectors",
    "appearance_similarity",
    "clamp",
]


def clamp(value, low, high):
    return low if value < low else (high if value > high else value)


# --------------------------------------------------------------------------
# 1. 底盘几何
# --------------------------------------------------------------------------

from robot_core.kinematics import (ChassisGeometry, yaw_from_steer,
                                   steer_from_yaw, max_yaw_at_speed,
                                   pure_pursuit_steer)



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
    """恒速模型的 alpha-beta 滤波器,同时输出位置与**速度**,并带野值门控。

    改造前用的是 EMA: smooth = 0.65*old + 0.35*new。
    EMA 只平滑位置,代价是引入约 2.4 帧的相位滞后 —— 10fps 下就是 240ms,
    车以 0.32 m/s 靠近时,滤波器读到的距离比真实距离**大 7~8 厘米**,
    这部分误差直接变成了撞击。

    alpha-beta 因为显式维护速度项,对匀速运动的目标是**零稳态滞后**,
    而且顺手给出目标速度,可以拿来做前馈跟速。

    新息门控 (innovation gate)
    -------------------------
    alpha=0.45 意味着**新观测的 45% 会被立刻采信**。没有门控时,深度相机的
    单帧野值会直接穿透滤波器:距离从 0.8m 跳到 3.0m,滤波结果瞬间变成 1.79m,
    刹车包络一看 1.79m 就放行满速 —— 车直接撞上去。

    这类野值在结构光相机上很常见:黑色衣物、逆光、玻璃反光会让 ROI 里只剩
    几个打在背景墙上的像素。所以观测值与预测值偏差超过门限时必须先丢弃,
    靠预测外推滑行;只有连续多帧都偏离,才认定目标真的跳变并重置滤波器。

    滑行期间 coasting 为 True,调用方应当据此降速 —— 外推出来的位置不是观测,
    不该按它全速前进。
    """

    def __init__(self, alpha=0.45, beta=0.10, max_dt_s=0.5,
                 gate_base_m=0.35, gate_rate_mps=2.5, max_rejects=3):
        if not 0 < alpha <= 1:
            raise ValueError("alpha 应在 (0, 1]")
        if not 0 <= beta <= 2:
            raise ValueError("beta 应在 [0, 2]")
        if gate_base_m <= 0 or gate_rate_mps <= 0:
            raise ValueError("门限必须为正")
        self.alpha = alpha
        self.beta = beta
        self.max_dt_s = max_dt_s
        self.gate_base_m = gate_base_m
        self.gate_rate_mps = gate_rate_mps
        self.max_rejects = max_rejects
        self.position = None
        self.velocity = 0.0
        self._last_t = None
        self.rejected_streak = 0
        self.rejected_total = 0
        self.last_accepted = True

    def reset(self):
        self.position = None
        self.velocity = 0.0
        self._last_t = None
        self.rejected_streak = 0
        self.last_accepted = True

    @property
    def initialized(self):
        return self.position is not None

    @property
    def coasting(self):
        """当前位置来自预测外推而非真实观测。"""
        return self.rejected_streak > 0

    def gate_width(self, dt):
        """允许的新息幅度。目标动得快时放宽,但有下限。"""
        return max(self.gate_base_m, self.gate_rate_mps * dt)

    def update(self, measurement, now):
        """喂一帧观测,返回滤波后的位置。被门控拒绝时返回外推位置。"""
        if self.position is None or self._last_t is None:
            self.position = float(measurement)
            self.velocity = 0.0
            self._last_t = now
            self.rejected_streak = 0
            self.last_accepted = True
            return self.position

        dt = now - self._last_t
        if dt <= 0.0:
            return self.position
        self._last_t = now
        if dt > self.max_dt_s:
            # 间隔过长说明目标丢过,速度估计已不可信,重新起算
            self.position = float(measurement)
            self.velocity = 0.0
            self.rejected_streak = 0
            self.last_accepted = True
            return self.position

        predicted = self.position + self.velocity * dt
        residual = measurement - predicted

        if abs(residual) > self.gate_width(dt):
            self.rejected_streak += 1
            self.rejected_total += 1
            self.last_accepted = False
            if self.rejected_streak >= self.max_rejects:
                # 连续偏离,说明是目标真的跳了(或之前跟的就是噪点),重新起算
                self.position = float(measurement)
                self.velocity = 0.0
                self.rejected_streak = 0
            else:
                # 单帧野值:丢弃观测,靠预测滑行
                self.position = predicted
            return self.position

        self.rejected_streak = 0
        self.last_accepted = True
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

    def step(self, target, dt, *, accel_limit=None):
        if dt <= 0:
            return self.value
        # 「更靠近 0」一律算减速,包括倒车时的减速
        shrinking = abs(target) < abs(self.value) or self.value * target < 0
        # 机动可以单独配置起步斜坡，减速沿用全局限制。
        if accel_limit is None:
            accel_limit = self.accel_limit
        if not math.isfinite(accel_limit) or accel_limit <= 0:
            raise ValueError("加速限制必须是有限正数")
        limit = (self.decel_limit if shrinking else accel_limit) * dt
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
        if desired_mps < 0:
            self._kick_until = 0.0
            self._was_moving = measured_moving
            return max(desired_mps, -abs(speed_cap))

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


# --------------------------------------------------------------------------
# 6. 目标锁定 —— 避免"房间里走过第二个人,车就跟着别人走了"
# --------------------------------------------------------------------------

def appearance_similarity(a_height, a_color, b_height, b_color,
                          height_tolerance_m=0.25):
    """两个观测长得有多像,返回 0~1;任何一边没有特征时返回 None。

    两个线索都是「白送」的,不额外占 NPU:

    height_m  bbox 像素高 x 深度 / fy,即**可见部分**的物理高度。
              人的身高一天之内不会变,是最稳的廉价身份线索。但脚被桌子挡住、
              半身入镜时它会突然变小,所以只能当软证据,不能硬判。
    color     上半身 HSV 色调直方图 (L1 归一化)。衣服颜色在一次跟随任务里
              基本不变,而且对距离、角度都不敏感。

    颜色用 Bhattacharyya 系数 sum(sqrt(p*q)):两个分布完全一致时为 1,
    完全不重叠时为 0,对直方图幅值的缩放不敏感。
    """
    scores = []
    if (a_height and b_height and a_height > 0.2 and b_height > 0.2):
        err = abs(a_height - b_height) / max(height_tolerance_m, 1e-6)
        scores.append((0.4, max(0.0, 1.0 - err)))
    if a_color and b_color and len(a_color) == len(b_color):
        sa, sb = sum(a_color), sum(b_color)
        if sa > 1e-6 and sb > 1e-6:
            bc = sum(math.sqrt((p / sa) * (q / sb))
                     for p, q in zip(a_color, b_color))
            scores.append((0.6, clamp(bc, 0.0, 1.0)))
    if not scores:
        return None
    total_w = sum(w for w, _ in scores)
    return sum(w * s for w, s in scores) / total_w


class TargetLock:
    """按运动一致性 + 外观特征做帧间数据关联,锁定同一个人。

    改造前每帧都独立地挑「最正前方 + 最接近期望距离」的检测框,评分一变就换人。
    这是跟随机器人最经典的失效模式:两个人交错走过,车会跟错。

    规则:
      - 未锁定时,连续 confirm_frames 帧都稳定指向同一位置才锁定;
      - 已锁定时,只接受落在**预测位置** assoc_radius_m 内的检测;
      - 接不上就算丢一帧,靠外推维持,超过 lost_timeout_s 才解锁重选。

    ------------------------------------------------------------------
    本次改动 1:锚点做自车运动补偿 + 目标速度外推
    ------------------------------------------------------------------
    锚点存在**车体坐标系**里,而车自己在动。老代码拿上一帧的观测位置直接和
    这一帧比,等于假设车是静止的。实际数字:车以 1.2 rad/s 转向、目标在 2m 处,
    单帧 (0.1s) 光是自车旋转就让目标在车体系里漂 0.24m,加上自车前进 0.055m
    和人自己走的 0.15m,一帧就是 0.45m —— assoc_radius_m=0.55 已经贴边,
    **检测掉一帧就必然掉锁**。所以转弯的时候比直行更容易跟丢,而转弯恰恰是
    跟随最需要它别丢的时候。

    现在每个控制周期用底盘实测的 (speed, yaw_rate) 把锚点搬到当前车体系,
    再叠上目标自己的速度做外推。关联门限比的是「预测位置」和观测的差,
    也就是类注释里一直写着、但代码里从来没做的那件事。

    ------------------------------------------------------------------
    本次改动 2:轻量外观特征,防止重锁时跟错人
    ------------------------------------------------------------------
    老代码解锁后重选目标的评分是 `abs(x)*1.5 + abs(z - 期望距离)` ——
    **谁最正对车头就跟谁**。人转过拐角、被柱子挡两秒、或者路上迎面来个人,
    回来就跟错了。现在候选项可以带 height_m 与 color 直方图:

      - 锁定期间用它们参与关联评分,位置接近但长得不像的会被压下去;
      - 解锁后重选时,只要签名还新鲜,就**必须**长得像才允许锁定,
        宁可继续搜索也不跟一个陌生人走。

    没有这两个字段时行为与改造前完全一致 —— 老的调用方和测试不用改。
    """

    def __init__(self, assoc_radius_m=0.55, lost_timeout_s=3.0, confirm_frames=1,
                 pending_grace_s=1.50, origin_offset_m=0.0,
                 appearance_floor=0.45, appearance_weight_m=0.60,
                 height_tolerance_m=0.25, signature_ttl_s=20.0,
                 max_target_speed_mps=2.5):
        if assoc_radius_m <= 0:
            raise ValueError("assoc_radius_m 必须为正")
        self.assoc_radius_m = assoc_radius_m
        self.lost_timeout_s = lost_timeout_s
        self.confirm_frames = confirm_frames
        self.pending_grace_s = pending_grace_s
        self.origin_offset_m = origin_offset_m
        self.appearance_floor = appearance_floor
        self.appearance_weight_m = appearance_weight_m
        self.height_tolerance_m = height_tolerance_m
        self.signature_ttl_s = signature_ttl_s
        self.max_target_speed_mps = max_target_speed_mps

        self.locked = False
        self.anchor_xz = None       # 预测位置 (含目标速度外推)
        self.last_seen = None
        self.velocity_fl = (0.0, 0.0)   # 目标在车体系下的速度 (前, 左)
        self.sig_height = None
        self.sig_color = None
        self.sig_stamp = None
        self.rejected_appearance = 0
        self._static_xz = None      # 只做自车补偿、不含目标速度的上一次观测
        self._advance_t = None      # 上次做自车补偿的时刻
        self._pending_xz = None
        self._pending_count = 0
        self._pending_last_seen = None

    def reset(self, forget_signature=False):
        """解锁。默认**保留**外观签名 —— 签名正是用来找回同一个人的,
        跟丢的时候把它一起扔了就等于自愿跟错人。"""
        self.locked = False
        self.anchor_xz = None
        self.last_seen = None
        self.velocity_fl = (0.0, 0.0)
        self._static_xz = None
        self._pending_xz = None
        self._pending_count = 0
        self._pending_last_seen = None
        if forget_signature:
            self.sig_height = self.sig_color = self.sig_stamp = None

    @staticmethod
    def _dist(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def age(self, now):
        return None if self.last_seen is None else now - self.last_seen

    # ------------------------------------------------------------------
    # 自车运动补偿
    # ------------------------------------------------------------------

    def _to_body(self, xz):
        """(右正 x, 车头到人的间距 z) -> (后轴中心前向, 左正)"""
        return (xz[1] + self.origin_offset_m, -xz[0])

    def _from_body(self, fl):
        return (-fl[1], fl[0] - self.origin_offset_m)

    def _shift(self, fl, dt, speed, yaw_rate):
        """把车体系里的一个静止点,搬到 dt 之后的车体系里。

        后轴中心沿圆弧走 (speed/yaw_rate 为半径),车体同时转过 yaw_rate*dt。
        直行时退化为直线,分开算避免 0 除。
        """
        fx, fy = fl
        if abs(yaw_rate) > 1e-6:
            radius = speed / yaw_rate
            dtheta = yaw_rate * dt
            dx = radius * math.sin(dtheta)
            dy = radius * (1.0 - math.cos(dtheta))
        else:
            dtheta = 0.0
            dx, dy = speed * dt, 0.0
        cos_t, sin_t = math.cos(-dtheta), math.sin(-dtheta)
        rx, ry = fx - dx, fy - dy
        return (rx * cos_t - ry * sin_t, rx * sin_t + ry * cos_t)

    def advance(self, dt, speed=0.0, yaw_rate=0.0):
        """控制周期推进:锚点跟着自车运动走,并按目标速度外推。"""
        if dt <= 0.0:
            return
        dt = min(dt, 0.5)
        if self._static_xz is not None:
            self._static_xz = self._from_body(
                self._shift(self._to_body(self._static_xz), dt, speed, yaw_rate))
        if self.anchor_xz is not None:
            fx, fy = self._to_body(self.anchor_xz)
            vf, vl = self.velocity_fl
            moved = self._shift((fx + vf * dt, fy + vl * dt), dt, speed, yaw_rate)
            self.anchor_xz = self._from_body(moved)
        if self._pending_xz is not None:
            self._pending_xz = self._from_body(
                self._shift(self._to_body(self._pending_xz), dt, speed, yaw_rate))

    # ------------------------------------------------------------------
    # 外观
    # ------------------------------------------------------------------

    def _similarity(self, cand):
        return appearance_similarity(self.sig_height, self.sig_color,
                                     cand.get('height_m'), cand.get('color'),
                                     self.height_tolerance_m)

    def signature_fresh(self, now):
        return (self.sig_stamp is not None
                and now - self.sig_stamp <= self.signature_ttl_s
                and (self.sig_height is not None or self.sig_color is not None))

    def _learn(self, cand, now, rate=0.15):
        """慢速更新签名。跟得越久越像本人,但单帧坏特征吃不动它。"""
        h = cand.get('height_m')
        if h and h > 0.2:
            self.sig_height = h if self.sig_height is None \
                else (1.0 - rate) * self.sig_height + rate * h
        c = cand.get('color')
        if c:
            if self.sig_color is None or len(self.sig_color) != len(c):
                self.sig_color = list(c)
            else:
                self.sig_color = [(1.0 - rate) * a + rate * b
                                  for a, b in zip(self.sig_color, c)]
            total = sum(self.sig_color)
            if total > 1e-6:
                self.sig_color = [v / total for v in self.sig_color]
        if self.sig_height is not None or self.sig_color is not None:
            self.sig_stamp = now

    # ------------------------------------------------------------------
    # 主入口
    # ------------------------------------------------------------------

    def update(self, candidates, now, prefer_distance_m, ego=None):
        """candidates: [{'x':.., 'z':.., 'conf':.., 'height_m':?, 'color':?}, ...]

        ego: (speed_mps, yaw_rate_radps) 底盘实测量,用于自车运动补偿。
             给 None 时退化成老行为(假设车静止)。

        返回本帧选中的候选项;接不上或尚未确认时返回 None。
        """
        # 补偿的基准是**上次补偿的时刻**,不是上次观测的时刻。
        # 用后者的话,连续几帧关联不上时同一段自车运动会被重复叠加。
        if ego is not None:
            if self._advance_t is not None:
                self.advance(now - self._advance_t, ego[0], ego[1])
            self._advance_t = now

        if self.locked and self.last_seen is not None:
            if now - self.last_seen > self.lost_timeout_s:
                self.reset()            # 签名保留,重锁时还要靠它认人
            else:
                best, best_cost, best_sim = None, float('inf'), None
                for c in candidates:
                    d = self._dist((c['x'], c['z']), self.anchor_xz)
                    if d > self.assoc_radius_m:
                        continue
                    sim = self._similarity(c)
                    # 位置贴得极近时不让外观否决:人低头、转身都会让直方图抖,
                    # 而 0.2m 以内实际上不可能是另一个人。
                    if (sim is not None and sim < self.appearance_floor
                            and d > self.assoc_radius_m * 0.35):
                        self.rejected_appearance += 1
                        continue
                    cost = d + (0.0 if sim is None
                                else self.appearance_weight_m * (1.0 - sim))
                    if cost < best_cost:
                        best, best_cost, best_sim = c, cost, sim
                if best is not None:
                    self._observe(best, now)
                    return best
                return None     # 关联不上,这一帧算丢

        if not candidates:
            # 人体检测偶尔漏一帧很常见。旧逻辑一遇空帧就把连续确认计数
            # 清零，结果是“偶尔能框到人，但永远锁不上”。短空窗保留候选，
            # 超过宽限时间才真正重置，仍可排除闪烁噪点。
            if (self._pending_last_seen is None
                    or now - self._pending_last_seen > self.pending_grace_s):
                self._pending_xz = None
                self._pending_count = 0
                self._pending_last_seen = None
            return None

        # 未锁定:先按外观筛,再挑最正前方且最接近期望距离的
        pool = candidates
        if self.signature_fresh(now):
            matched = [c for c in candidates
                       if (self._similarity(c) or 0.0) >= self.appearance_floor]
            if matched:
                pool = matched
            elif any(self._similarity(c) is not None for c in candidates):
                # 视野里有人,但没一个像本人。宁可继续搜索也不跟陌生人走。
                self.rejected_appearance += 1
                return None

        best = min(pool,
                   key=lambda c: abs(c['x']) * 1.5 + abs(c['z'] - prefer_distance_m))
        xz = (best['x'], best['z'])
        if self._pending_xz is not None and self._dist(xz, self._pending_xz) <= self.assoc_radius_m:
            self._pending_count += 1
        else:
            self._pending_count = 1
        self._pending_xz = xz
        self._pending_last_seen = now

        if self._pending_count >= self.confirm_frames:
            self.locked = True
            self._observe(best, now)
            self._pending_xz = None
            self._pending_count = 0
            self._pending_last_seen = None
            return best
        return None

    def _observe(self, cand, now):
        """接受一次观测:更新速度估计、锚点与外观签名。"""
        xz = (cand['x'], cand['z'])
        if self._static_xz is not None and self.last_seen is not None:
            dt = now - self.last_seen
            if 1e-3 < dt <= 0.5:
                fx, fy = self._to_body(xz)
                px, py = self._to_body(self._static_xz)
                vf = clamp((fx - px) / dt, -self.max_target_speed_mps,
                           self.max_target_speed_mps)
                vl = clamp((fy - py) / dt, -self.max_target_speed_mps,
                           self.max_target_speed_mps)
                ovf, ovl = self.velocity_fl
                self.velocity_fl = (0.5 * ovf + 0.5 * vf, 0.5 * ovl + 0.5 * vl)
        self.anchor_xz = xz
        self._static_xz = xz
        self.last_seen = now
        self._learn(cand, now)


# --------------------------------------------------------------------------
# 7. 相机 / 雷达交叉证伪
# --------------------------------------------------------------------------

def reconcile_range(camera_z, lidar_z, conflict_margin_m=1.00):
    """用雷达读数校验相机距离,返回 (采信的距离, 是否判定为冲突)。

    结构光相机读错时几乎总是**读得更远**(ROI 里的有效像素打在了背景上),
    而雷达在同一方位上会如实报告近处的物体。所以:

      - 雷达更近 -> 一律采信雷达,宁可保守;
      - 差得离谱(超过 conflict_margin_m)-> 额外标记冲突,
        调用方应当把这一帧的相机观测整个作废并停车,而不是接着按相机跟。

    雷达读数无效(None / 非有限值)时原样返回相机值。

    >>> reconcile_range(2.5, 0.7)
    (0.7, True)
    >>> reconcile_range(0.95, 1.20)
    (0.95, False)
    >>> reconcile_range(1.20, 0.95)
    (0.95, False)
    >>> reconcile_range(1.0, None)
    (1.0, False)
    """
    if lidar_z is None or not math.isfinite(lidar_z) or lidar_z <= 0:
        return camera_z, False
    if lidar_z >= camera_z:
        return camera_z, False
    return lidar_z, (camera_z - lidar_z) > conflict_margin_m


class ScanSectors:
    """把一圈激光扫描压成按方位分桶的最近距离,便于按目标方位查询。

    只用全向最小值会被走廊两侧的墙拉低,导致车在长走廊里莫名其妙一直限速;
    而做相机证伪时又需要**目标所在方位附近**的距离,而不是整个前向扇区。
    """

    def __init__(self, half_fov_deg=60.0, bin_deg=5.0):
        if bin_deg <= 0 or half_fov_deg <= 0:
            raise ValueError("扇区参数必须为正")
        self.half_fov = math.radians(half_fov_deg)
        self.bin_rad = math.radians(bin_deg)
        self.n_bins = int(2 * self.half_fov / self.bin_rad) + 1
        self.bins = [float('inf')] * self.n_bins

    def clear(self):
        self.bins = [float('inf')] * self.n_bins

    def _index(self, bearing_rad):
        if abs(bearing_rad) > self.half_fov:
            return None
        return min(self.n_bins - 1,
                   max(0, int((bearing_rad + self.half_fov) / self.bin_rad)))

    def add(self, bearing_rad, range_m):
        i = self._index(bearing_rad)
        if i is not None and range_m < self.bins[i]:
            self.bins[i] = range_m

    def min_near(self, bearing_rad, half_width_rad):
        """查询某个方位 ± half_width 范围内的最近距离,无有效数据返回 None。"""
        lo = self._index(max(-self.half_fov, bearing_rad - half_width_rad))
        hi = self._index(min(self.half_fov, bearing_rad + half_width_rad))
        if lo is None or hi is None:
            return None
        best = min(self.bins[lo:hi + 1])
        return None if math.isinf(best) else best

    def min_within(self, half_width_rad):
        """正前方 ± half_width 的最近距离。"""
        return self.min_near(0.0, half_width_rad)


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
