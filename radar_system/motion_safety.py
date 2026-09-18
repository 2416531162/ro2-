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
    "yaw_from_steer", "steer_from_yaw", "max_yaw_at_speed",
    "AlphaBetaTracker", "SlewLimiter", "BreakawayKick",
    "TargetLock", "reconcile_range", "ScanSectors",
    "clamp",
]


def clamp(value, low, high):
    return low if value < low else (high if value > high else value)


# --------------------------------------------------------------------------
# 1. 底盘几何
# --------------------------------------------------------------------------

from robot_core.kinematics import (ChassisGeometry, yaw_from_steer,
                                   steer_from_yaw, max_yaw_at_speed)


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

class TargetLock:
    """按运动一致性做帧间数据关联,锁定同一个人。

    改造前每帧都独立地挑「最正前方 + 最接近期望距离」的检测框,评分一变就换人。
    这是跟随机器人最经典的失效模式:两个人交错走过,车会跟错。

    规则很简单但有效:
      - 未锁定时,连续 confirm_frames 帧都稳定指向同一位置才锁定;
      - 已锁定时,只接受落在预测位置 assoc_radius_m 内的检测;
      - 接不上就算丢一帧,靠外推维持,超过 lost_timeout_s 才解锁重选。

    后续要更强的身份保持,可以在这层之上叠 FaceNet 外观特征做重识别,
    接口不用改 —— 候选项里多带一个 embedding 字段即可。
    """

    def __init__(self, assoc_radius_m=0.55, lost_timeout_s=1.5, confirm_frames=3,
                 pending_grace_s=0.35):
        if assoc_radius_m <= 0:
            raise ValueError("assoc_radius_m 必须为正")
        self.assoc_radius_m = assoc_radius_m
        self.lost_timeout_s = lost_timeout_s
        self.confirm_frames = confirm_frames
        self.pending_grace_s = pending_grace_s
        self.locked = False
        self.anchor_xz = None       # 已锁定目标的最近一次位置
        self.last_seen = None
        self._pending_xz = None     # 待确认目标
        self._pending_count = 0
        self._pending_last_seen = None

    def reset(self):
        self.locked = False
        self.anchor_xz = None
        self.last_seen = None
        self._pending_xz = None
        self._pending_count = 0
        self._pending_last_seen = None

    @staticmethod
    def _dist(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def age(self, now):
        return None if self.last_seen is None else now - self.last_seen

    def update(self, candidates, now, prefer_distance_m):
        """candidates: [{'x':.., 'z':.., 'conf':..}, ...]

        返回本帧选中的候选项;接不上或尚未确认时返回 None。
        """
        if self.locked and self.last_seen is not None:
            if now - self.last_seen > self.lost_timeout_s:
                self.reset()
            else:
                best, best_d = None, float('inf')
                for c in candidates:
                    d = self._dist((c['x'], c['z']), self.anchor_xz)
                    if d < best_d:
                        best, best_d = c, d
                if best is not None and best_d <= self.assoc_radius_m:
                    self.anchor_xz = (best['x'], best['z'])
                    self.last_seen = now
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

        # 未锁定:挑最正前方且最接近期望距离的,连续几帧稳定才锁
        best = min(candidates,
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
            self.anchor_xz = xz
            self.last_seen = now
            self._pending_xz = None
            self._pending_count = 0
            self._pending_last_seen = None
            return best
        return None


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
