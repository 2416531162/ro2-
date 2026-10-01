#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RTK 路径跟踪：直线段 + 圆弧段首尾相接的路径（小圆、跑道形、8 字等）。纯计算，不依赖 ROS。

控制律是 line_tracker 的推广，同样按"空间"设计（对走过的路程），车速变化时收敛距离不变：
    κ    = κ_ff + κ_fb
    κ_ff = κ_r(s + 预瞄)·cosθe / (1 − κ_r·e)                    路径本身的曲率（前馈），直线上为 0
    κ_fb = −k_θ·(θe − θ_ref) − k_i·∫e ds，θ_ref = −atan(k_e·e)   与直线控制器相同的横偏/航向反馈
小偏差时横偏满足 e'' + 2ζω·e' + ω²·e = 0（' 是对路程求导）。低通只加在反馈项上（挡住定位噪声），
前馈来自路径、没有噪声，不滤，弯道入口不会因滤波变迟。速度按前方曲率限制横向加速度，终点前按减速度刹停。

路径投影只在上一次位置附近找最近点：小圆转第二圈、8 字交叉处不会跳到别的圈上。

底盘执行延迟（实车 2026-09-26 两趟连线路线记录辨识）：指令发出约 0.33 s 后车才开始按新曲率走，实际曲率只有
指令的 0.96（前进）/ 0.90（倒车）。所以控制器不按"现在"的位姿算，而按"已发出、还没生效的指令都执行完"之后
的位姿算（Smith 预估，delay_s），现在算出的指令正好在那时生效；下发曲率除以 steer_gain 补上转不够的部分。
弯道按横向加速度 lat_accel_mps2 限速，满舵弯约 0.3 m/s：延迟期间走过的路程短，入弯出弯的偏差小。

前进、倒车交替的路线（连线路线的倒车补角）由 SectionTracker 按段跟踪：每段一个 PathTracker，段尾停稳再换向。
倒车段在"行进方向"坐标里跟踪（车头 +π、车速取反、控制点前后对调）：后轴中心的运动学在这个坐标里与前进
完全相同，只是曲率取反，所以两个方向用同一套控制律。

坐标：本地 ENU 平面（x 东 y 北），偏航东为 0、逆时针为正；曲率左转为正；横偏在路径左侧为正。
"""
import math
from collections import deque
from dataclasses import dataclass, field, replace

from robot_core.kinematics import ChassisGeometry


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Piece:
    """一段等曲率路径：起点位姿 (x, y, yaw)、长度、曲率（0 为直线），s0 是它在整条路径上的起始路程。"""

    def __init__(self, x, y, yaw, length, kappa, s0):
        self.x, self.y, self.yaw, self.length, self.kappa, self.s0 = x, y, yaw, length, kappa, s0
        self.straight = abs(kappa) < 1e-12

    def pose(self, u):
        """段内路程 u 处的 (x, y, yaw)；直线段的 u 可以超出 [0, length]（外延）。"""
        k, yaw = self.kappa, self.yaw + self.kappa * u
        if self.straight:
            return self.x + u * math.cos(self.yaw), self.y + u * math.sin(self.yaw), self.yaw
        return (self.x + (math.sin(yaw) - math.sin(self.yaw)) / k,
                self.y - (math.cos(yaw) - math.cos(self.yaw)) / k, yaw)

    def project(self, px, py, u_hint, lo, hi):
        """点在 [lo, hi] 段内的最近点；圆弧上取离 u_hint 最近的那一圈。返回 (u, 横偏, 距离)。"""
        if self.straight:
            u = (px - self.x) * math.cos(self.yaw) + (py - self.y) * math.sin(self.yaw)
        else:
            r = 1.0 / self.kappa
            cx, cy = self.x - r * math.sin(self.yaw), self.y + r * math.cos(self.yaw)
            # 点相对圆心的方位 → 圆上最近处的切线方向；与 u_hint 处切线的夹角换成路程
            yaw_near = math.atan2(py - cy, px - cx) + math.copysign(math.pi / 2, self.kappa)
            u0 = max(lo, min(hi, u_hint))
            u = u0 + wrap(yaw_near - (self.yaw + self.kappa * u0)) / self.kappa
        u = max(lo, min(hi, u))
        x, y, yaw = self.pose(u)
        dx, dy = px - x, py - y
        return u, -math.sin(yaw) * dx + math.cos(yaw) * dy, math.hypot(dx, dy)


class Path:
    """从 start=(x, y, yaw) 出发，按 pieces=[(长度, 曲率), …] 依次画出的连续路径（位置、方向都连续）。"""

    def __init__(self, start, pieces, name=''):
        if not pieces:
            raise ValueError('路径是空的')
        self.name = name
        self.pieces = []
        x, y, yaw = start
        s = 0.0
        for length, kappa in pieces:
            if not length > 0 or not math.isfinite(kappa):
                raise ValueError('每段长度必须为正、曲率必须有限')
            p = Piece(x, y, yaw, float(length), float(kappa), s)
            self.pieces.append(p)
            x, y, yaw = p.pose(p.length)
            s += p.length
        self.length = s
        self.kappa_abs_max = max(abs(p.kappa) for p in self.pieces)

    def _piece(self, s):
        for p in self.pieces:
            if s < p.s0 + p.length:
                return p
        return self.pieces[-1]

    def pose(self, s):
        p = self._piece(max(0.0, s))
        return p.pose(s - p.s0)

    def kappa(self, s):
        """路程 s 处的路径曲率；起点前、终点后按首尾段算。"""
        return self._piece(max(0.0, s)).kappa

    def kappa_ahead(self, s, distance):
        """[s, s+distance] 内的最大 |曲率|，弯道限速往前看用。"""
        return max((abs(p.kappa) for p in self.pieces
                    if p.s0 < s + distance and p.s0 + p.length > s), default=0.0)

    def project(self, px, py, s_hint=None, *, back=0.5, ahead=1.5, extend_start=0.0):
        """最近点，返回 (s, 横偏, 路径方向, 路径曲率)。

        有 s_hint 时只在 [s_hint−back, s_hint+ahead] 里找；没有时全程找，同样近的取路程最小的（起点那一圈）。
        首段是直线时可往回外延 extend_start 米（助跑），末段是直线时可往前外延（冲过终点）。
        """
        lo_s, hi_s = (-math.inf, math.inf) if s_hint is None else (s_hint - back, s_hint + ahead)
        last = len(self.pieces) - 1
        found = []
        for i, p in enumerate(self.pieces):
            lo = -extend_start if (i == 0 and p.straight) else 0.0
            hi = p.length + 5.0 if (i == last and p.straight) else p.length
            lo, hi = max(lo, lo_s - p.s0), min(hi, hi_s - p.s0)
            if lo > hi:
                continue
            hint = lo if s_hint is None else s_hint - p.s0
            u, e, d = p.project(px, py, hint, lo, hi)
            found.append((d, p.s0 + u, e, p))
        if not found:
            raise ValueError('路程提示超出路径范围')
        best = min(d for d, *_ in found)
        d, s, e, p = min((c for c in found if c[0] <= best + 1e-3), key=lambda c: c[1])
        return s, e, p.pose(s - p.s0)[2], p.kappa


# ---- 常用路径（start 是入口直线的起点位姿）----

def circle(radius, *, laps=1.0, left=True, lead_in=2.0, lead_out=1.5, start=(0.0, 0.0, 0.0)):
    """入口直线 → 圆（与直线相切）→ 出口直线。"""
    k = (1.0 if left else -1.0) / radius
    return Path(start, [(lead_in, 0.0), (2 * math.pi * radius * laps, k), (lead_out, 0.0)],
                name='circle_r%g%s' % (radius, '' if left else '_cw'))


def stadium(radius, straight, *, laps=1, left=True, lead_in=2.0, lead_out=1.5, start=(0.0, 0.0, 0.0)):
    """跑道形：直道、半圆、直道、半圆，直线与圆弧交替（曲率阶跃）。"""
    k = (1.0 if left else -1.0) / radius
    lap = [(straight, 0.0), (math.pi * radius, k), (straight, 0.0), (math.pi * radius, k)]
    return Path(start, [(lead_in, 0.0)] + lap * laps + [(lead_out, 0.0)], name='stadium_r%g' % radius)


def figure8(radius, *, lead_in=2.0, lead_out=1.5, start=(0.0, 0.0, 0.0)):
    """8 字：左转一整圈接右转一整圈，在交点处曲率从 +1/R 直接翻到 −1/R（最难的一种）。"""
    k = 1.0 / radius
    return Path(start, [(lead_in, 0.0), (2 * math.pi * radius, k), (2 * math.pi * radius, -k), (lead_out, 0.0)],
                name='figure8_r%g' % radius)


# ---- 控制器 ----

@dataclass(frozen=True)
class PathConfig:
    cruise_mps: float = 0.5
    accel_mps2: float = 0.5
    decel_mps2: float = 0.5
    min_mps: float = 0.06            # 终点前的蠕行速度
    lat_accel_mps2: float = 0.05     # 弯道限速：v ≤ √(a/|κ|)，满舵弯（R 1.92 m）约 0.31 m/s
    omega_per_m: float = 2.0         # 与直线控制器相同
    zeta: float = 0.9
    ki_per_m2: float = 0.6
    integral_limit: float = 0.12     # 积分项贡献的曲率上限 1/m
    approach_max_rad: float = math.radians(30.0)
    track_x_m: float = 0.0           # 要压在路径上的点（车体系 x），默认后轴中心
    delay_s: float = 0.33            # 底盘执行延迟（实测）：按已发出的指令把位姿推到这么久之后再算，0 为不用
    steer_gain: float = 0.96         # 实际曲率 / 指令曲率（实测，前进）；下发时除以它
    lead_s: float = 0.03             # 再按当前速度、角速度往前推这么久（定位本身的滞后）
    ff_preview_s: float = 0.05       # 曲率前馈再往前看这么久：舵机从一个角度转到另一个角度要时间
    kappa_tau_s: float = 0.15        # 反馈项低通
    kappa_margin: float = 0.9        # 路径曲率最多用到底盘最大曲率的这个比例，剩下的留给纠偏
    start_max_offset_m: float = 0.5
    start_max_heading_rad: float = math.radians(30.0)
    start_before_m: float = 3.0      # 入口是直线时，允许从起点后方最多这么远起步
    start_min_left_m: float = 0.5    # 离终点不到这么远就不起步（分段路径换向后的短段放宽到 stop_tol_m）
    abort_offset_m: float = 0.5
    abort_heading_rad: float = math.radians(45.0)
    slow_offset_m: float = 0.10      # 横偏超过它开始降速，积分也停
    stop_tol_m: float = 0.02
    # 起步慢走：每段从停着起步后先走 start_creep_m，车速不超过 start_creep_mps（0 为不用）。差速车的万向轮在原地转向、
    # 换向后是横着的，一起步就把车身甩歪（2026-09-30 实车：车头甩 5–8°，线头 1 m 内偏 4–8 cm）；慢走让它先转回来
    start_creep_mps: float = 0.0
    start_creep_m: float = 0.3
    # 倒车段之后往前起步慢走这么远（不比 start_creep_m 短）：倒完车万向轮朝前，往前走要整个翻过来；倒车助跑
    # 留出的这段（route_plan PIVOT_REVERSE_MIN_M − 喷头前伸，0.9 m）先慢走 0.5 m，剩下 0.4 m 加速，喷头到线头时车已走直
    # （2026-10-01 用户：倒退完要慢速前进，让万向轮回正，才能开的直）
    reverse_creep_m: float = 0.0
    reverse_creep_mps: float = 0.0  # 倒车后助跑慢走的车速（0 同 start_creep_mps）
    # 倒车段自己起步慢走多远（None 同 start_creep_m）：倒车只是把车摆到助跑起点、不画线，原地转完万向轮横着，
    # 倒车起步只要它转过来（约 90°）就行，不用慢走满 start_creep_m
    reverse_section_creep_m: float = None
    # 段尾停车按这么久的执行延迟预估车还会走多远（None 同 delay_s）。车速指令比转向快：2026-10-01 实车 26 次
    # 段尾发 0 后实际还走的距离，对应延迟中位 0.33 s（四分位 0.29–0.37 s），按转向的 0.44 s 估多 3–5 cm，
    # 车停短 4–6 cm，原地转 90° 后成了下一段（进圆、矩形下一条边）的横偏
    stop_delay_s: float = None
    # 起步锁车头（差速车）：每段起步后 start_hold_m 内，车头误差的纠正速度（1/s）不低于 start_hold_rate，不够的部分
    # 直接给角速度（不超过 start_hold_max rad/s）。曲率反馈给出的角速度 = 车速 × 曲率，慢走时几乎为 0：2026-09-30
    # 实车原地转完起步，车头按原地转的方向再甩 4–13°（21 次 19 次同向），0.1 m/s 时只给出 0.06 rad/s，压不住，
    # 喷嘴偏 5–10 cm，之后 5 m 才回来。0 为不用
    start_hold_m: float = 0.0
    start_hold_rate: float = 2.0
    start_hold_max: float = 0.3


def differential_config(config=PathConfig()):
    """差速车（Gen5）的跟踪参数：直接下发车速和角速度，前轮是随动万向轮，不会转不够。
    执行延迟按 2026-09-30 实车 IMU 直行辨识：陀螺 ≈ 0.44 s 前的角速度指令 × 1.02–1.04（原来没辨识，
    按 0.1 s 时仿真 0.8 m/s 画直线偏 20 cm 以上，按 0.44 s 约 2 cm）。弯道横向加速度 0.20 m/s²
    （2026-09-30 用户嫌转弯慢 0.08 → 0.12；2026-10-01 再提速：0.12 时实车 R 3.7 m 圆 0.67 m/s 进圆后 p95 0.9–1.3 cm、
    R 2.2 m 圆 0.51 m/s p95 1.5–1.8 cm，余量大；仿真 0.20 时 R 2.2 m p95 2.3 cm，比 0.12 多约 0.5 cm）：
    R 2.2 m 约 0.66 m/s，R 3.7 m 及以上按 0.8 m/s，角球弧 R 1 m 约 0.44 m/s；
    场地规划经 route_limits 用同一个值。起步先按 0.1 m/s 慢走 0.3 m（万向轮回正），倒车段只慢走 0.15 m，
    倒车后往前起步按 0.15 m/s 慢走 0.5 m（整段倒车助跑），起步 0.6 m 内锁车头；段尾停车与转向同一个延迟预估。
    执行延迟 0.44 → 0.30 s（2026-10-01 下午用户把两台驱动器速度环 Kp 1.8 / Ki 30 改成 2.7 / 45）：巡航时指令→陀螺的
    延迟从 0.38 s 降到 0.28 s，车身左右摆从 0.64 Hz 变成 1.0 Hz；按 0.44 s 预估补过头，1 Hz 处环路增益 0.99、相位 −179°
    （已部署代码实测控制器频响 × 摆得最厉害两趟实测的底盘 1.7∠−100°），6 趟里 2 趟摆幅越来越大到 ±7°、角速度 ±0.5 rad/s。
    按 0.30 s：同一底盘增益裕度 1.6；仿真（二阶 + 延迟、过 1 Hz 实测点）30 m 直线喷头 p95 1.79 → 1.68 cm。
    **延迟与驱动器参数是一对**：驱动器改回 1.8 / 30 要把这里改回 0.44（按 0.30 时旧驱动器相位裕度只剩约 20°）；
    驱动器再加增益前先实测。这时 ω 2.5 的增益裕度只剩约 1.27，不用。
    拐角提速（2026-10-01 用户：矩形每个拐角都要倒车太慢，不改硬件、把走法调快）：实车一个拐角约 13.5 s（停稳、原地转
    3 s、倒车 3.4 s、停稳、助跑 0.1 m/s 约 4 s）；原地转不提速（1.0 rad/s 时驱动器速度环冲过头，见 robot_gen5.json），
    改倒车段慢走 0.3 → 0.15 m、倒车 0.3 → 0.5 m/s（DIFF_REVERSE_MPS）、停稳 0.3 → 0.2 s（DIFF_SETTLE_S）、
    助跑慢走 0.1 → 0.15 m/s。"""
    return replace(config, delay_s=0.30, steer_gain=1.0, lat_accel_mps2=0.20, start_creep_mps=0.1, start_hold_m=0.6,
                   reverse_creep_m=0.5, reverse_creep_mps=0.15, reverse_section_creep_m=0.15, stop_delay_s=None)


DIFF_REVERSE_MPS = 0.5      # 差速车倒车段限速（SectionTracker reverse_mps；阿克曼车 0.3）
DIFF_SETTLE_S = 0.2         # 差速车段尾停稳要持续多久才换段（SectionTracker settle_s；阿克曼车 0.3）


@dataclass
class PathState:
    state: str = 'IDLE'              # IDLE / RUNNING / PAUSED / DONE / ABORTED
    reason: str = ''
    e: float = 0.0                   # 横偏，m，路径左侧为正
    heading_err: float = 0.0         # rad，车头相对路径切线
    s: float = 0.0                   # 路程，m
    length: float = 0.0
    kappa_ref: float = 0.0           # 当前处的路径曲率
    v_cmd: float = 0.0
    kappa: float = 0.0               # 下发的曲率
    integral: float = 0.0            # ∫e ds
    stats: dict = field(default_factory=dict)


class PathTracker:
    def __init__(self, path, config=PathConfig(), geometry=None):
        self.cfg = config
        self.geo = geometry or ChassisGeometry()
        self.kappa_max = 1.0 / self.geo.min_turn_radius_m
        usable = config.kappa_margin * self.kappa_max
        if path.kappa_abs_max > usable + 1e-9:
            raise ValueError('路径最小转弯半径 %.2f m，底盘能稳定跟踪的最小半径是 %.2f m'
                             % (1.0 / path.kappa_abs_max, 1.0 / usable))
        self.path = path
        # 控制律按车实际走出的曲率算，最多能走出 steer_gain·kappa_max（下发的仍不超过 kappa_max）
        self.kappa_act_max = config.steer_gain * self.kappa_max
        self.k_theta = 2.0 * config.zeta * config.omega_per_m
        self.k_e = config.omega_per_m ** 2 / self.k_theta
        self.st = PathState(length=path.length)
        self._hint = None
        self._v_prev = 0.0
        self._creep_from = None
        self._kfb = 0.0
        self._sum2 = self._n = self._max = 0.0
        self._clock = 0.0
        self._sent = deque()            # (下发时刻, 车速, 下发角速度)：还在底盘执行延迟里的指令

    def locate(self, x, y, yaw, hint=None):
        """车体控制点相对路径的 (路程, 横偏, 车头误差, 路径曲率)。"""
        px = x + self.cfg.track_x_m * math.cos(yaw)
        py = y + self.cfg.track_x_m * math.sin(yaw)
        s, e, path_yaw, k = self.path.project(px, py, hint, extend_start=self.cfg.start_before_m)
        return s, e, wrap(yaw - path_yaw), k

    def check_start(self, pose, hint=None, resume=False):
        """resume=True（暂停后接着走）不查离终点多近：段尾前暂停的照样接着把这段走完（下一个周期到点就 DONE）。
        2026-10-01 实车：接入段剩 0.14 m 时定位抖了一下暂停，恢复时按「离终点不到 0.5 m 不起步」每秒重试都不过，
        车停着不动、屏幕还显示「行驶中」，直到操作员按停止。"""
        if not pose.usable_for_control:
            return False, 'pose_unusable'
        s, e, he, _ = self.locate(pose.x, pose.y, pose.yaw, hint)
        if abs(e) > self.cfg.start_max_offset_m:
            return False, 'too_far_from_line'
        if abs(he) > self.cfg.start_max_heading_rad:
            return False, 'heading_not_aligned'
        if s < -self.cfg.start_before_m + 1e-6:
            return False, 'too_far_behind_a'
        if not resume and s > self.path.length - self.cfg.start_min_left_m:
            return False, 'already_past_b'
        return True, ''

    def start(self, pose, integral=None):
        # 暂停后继续：在暂停处附近找投影，多圈路线不会跳回第一圈、8 字交叉处不会跳到另一支
        resume = self.st.state == 'PAUSED'
        hint = self._hint if resume else None
        ok, why = self.check_start(pose, hint, resume=resume)
        if not ok:
            if self.st.state != 'PAUSED':
                self.st.state = 'IDLE'
            self.st.reason = why
            return False
        self.st.state, self.st.reason = 'RUNNING', ''
        if integral is not None:
            self.st.integral = integral
        self._hint = self.locate(pose.x, pose.y, pose.yaw, hint)[0]
        self._kfb = 0.0
        self._creep_from = None         # 起步慢走、锁车头从哪里（路程）算起：第一个控制周期记下
        self._v_prev = max(0.0, pose.speed)
        self._sum2 = self._n = self._max = 0.0
        self._clock = 0.0
        self._sent.clear()
        return True

    def pause(self, reason='operator'):
        self.st.state, self.st.reason = 'PAUSED', reason
        self.st.v_cmd = self._v_prev = 0.0
        self._sent.clear()
        return 0.0, 0.0

    def stop(self, reason='operator'):
        if self.st.state in ('RUNNING', 'PAUSED'):
            self.st.state, self.st.reason = 'ABORTED', reason
        self.st.v_cmd = 0.0
        return 0.0, 0.0

    def update(self, pose, dt):
        """返回 (vx, wz)。pose 需有 x, y, yaw, speed, yaw_rate, usable_for_control。"""
        self._clock += dt
        v, kappa, w_hold = self._control(pose, dt)
        wz = v * kappa + w_hold
        self._sent.append((self._clock, v, wz))
        return v, wz

    def predict(self, x, y, yaw, delay=None):
        """已发出、还在执行延迟里的指令都执行完后的 (x, y, yaw)。

        第 i 条指令在 [t_i, t_i+1) 期间有效、晚 delay_s 才作用到车上，所以车接下来 delay_s 内怎么走，由
        [现在 − delay_s, 现在) 期间有效的那些指令决定；车实际角速度 = 下发角速度 × steer_gain。
        delay 给了（不超过 delay_s）就按它推，段尾停车用（见 stop_delay_s）。
        """
        sent = self._sent
        while len(sent) >= 2 and sent[1][0] <= self._clock - self.cfg.delay_s + 1e-9:
            sent.popleft()
        d = self.cfg.delay_s if delay is None else min(delay, self.cfg.delay_s)
        if d <= 0.0:
            return x, y, yaw
        start = self._clock - d
        for i, (t, v, w) in enumerate(sent):
            end = sent[i + 1][0] if i + 1 < len(sent) else self._clock
            a = max(t, start)
            if end <= a or (v == 0.0 and w == 0.0):
                continue
            if v == 0.0:                # 起步锁车头：车没动、只转
                yaw += w * self.cfg.steer_gain * (end - a)
                continue
            x, y, yaw = Piece(x, y, yaw, 0.0, w / v * self.cfg.steer_gain, 0.0).pose(v * (end - a))
        return x, y, yaw

    def _control(self, pose, dt):
        """返回 (车速, 下发曲率, 起步锁车头另加的角速度)。"""
        st, cfg = self.st, self.cfg
        if st.state not in ('RUNNING', 'PAUSED'):
            st.v_cmd = 0.0
            return 0.0, 0.0, 0.0
        if not pose.usable_for_control:
            st.state, st.reason = 'PAUSED', 'pose_unusable'
            st.v_cmd, self._v_prev = 0.0, 0.0
            return 0.0, 0.0, 0.0
        if st.state == 'PAUSED':
            st.reason = 'pose_recovered_waiting_operator'
            return 0.0, 0.0, 0.0

        # 执行延迟：先按已发出的指令把位姿推到 delay_s 之后，现在算的指令正好在那时生效（Smith 预估）；
        # 再按当前速度和角速度推 lead_s（弯道上角速度不为 0，推出去的是一段圆弧）
        x, y, yaw = self.predict(pose.x, pose.y, pose.yaw)
        mid = yaw + 0.5 * pose.yaw_rate * cfg.lead_s
        x += pose.speed * cfg.lead_s * math.cos(mid)
        y += pose.speed * cfg.lead_s * math.sin(mid)
        yaw += pose.yaw_rate * cfg.lead_s
        s, e, he, k_r = self.locate(x, y, yaw, self._hint)
        ds = 0.0 if self._hint is None else max(0.0, s - self._hint)
        self._hint = s
        # 报给操作员、记录、偏离中止用车现在的位置；控制用预估的位置
        s_now, e_now, he_now, _ = self.locate(pose.x, pose.y, pose.yaw, s)
        st.e, st.heading_err, st.s, st.kappa_ref = e_now, he_now, s_now, k_r

        if abs(e_now) > cfg.abort_offset_m or abs(he_now) > cfg.abort_heading_rad:
            st.state, st.reason = 'ABORTED', 'off_line'
            st.v_cmd, self._v_prev = 0.0, 0.0
            return 0.0, 0.0, 0.0
        remaining = self.path.length - s
        if cfg.stop_delay_s is not None and cfg.stop_delay_s < cfg.delay_s:
            # 停车（刹停曲线、到点）按车速指令的延迟预估：比转向的 delay_s 短，按 delay_s 会提前停、停短
            xs, ys, yaws = self.predict(pose.x, pose.y, pose.yaw, cfg.stop_delay_s)
            xs += pose.speed * cfg.lead_s * math.cos(yaws)
            ys += pose.speed * cfg.lead_s * math.sin(yaws)
            remaining = self.path.length - self.locate(xs, ys, yaws, s_now)[0]
        if remaining <= cfg.stop_tol_m:
            st.state, st.reason = 'DONE', ''
            st.v_cmd, self._v_prev = 0.0, 0.0
            return 0.0, 0.0, 0.0

        if self._creep_from is None:
            self._creep_from = s_now
        started = s_now - self._creep_from          # 这次起步后走了多远
        # 起步锁车头这段不积分：起步甩头是一下子的，积进去会在后面几米反向拖着偏
        if abs(e) < cfg.slow_offset_m and started >= cfg.start_hold_m:
            lim = cfg.integral_limit / cfg.ki_per_m2
            st.integral = max(-lim, min(lim, st.integral + e * ds))

        # 前馈：前方 ff_preview_s 处的路径曲率，按横偏、航向误差修正
        k_pre = self.path.kappa(s + max(0.0, pose.speed) * cfg.ff_preview_s)
        k_ff = k_pre * math.cos(he) / max(0.5, 1.0 - k_pre * e)
        theta_ref = -math.atan(self.k_e * e)
        theta_ref = max(-cfg.approach_max_rad, min(cfg.approach_max_rad, theta_ref))
        k_fb = -self.k_theta * (he - theta_ref) - cfg.ki_per_m2 * st.integral
        # 先把反馈限在"前馈之外还剩的曲率"内再低通（与直线控制器一样先限幅后滤波，滤波器里不会积出超限的量）
        k_max = self.kappa_act_max
        k_ff = max(-k_max, min(k_max, k_ff))
        k_fb = max(-k_max - k_ff, min(k_max - k_ff, k_fb))
        self._kfb += (k_fb - self._kfb) * dt / (cfg.kappa_tau_s + dt)
        kappa = max(-k_max, min(k_max, k_ff + self._kfb))
        # 车实际只走出下发曲率的 steer_gain，下发时补上
        kappa = st.kappa = max(-self.kappa_max, min(self.kappa_max, kappa / cfg.steer_gain))

        # 速度：巡航；前方刹车距离内有弯就按横向加速度限速；偏差大降速；终点前刹停
        v = cfg.cruise_mps
        look = v * v / (2.0 * cfg.decel_mps2) + v * cfg.lead_s + 0.3     # s 已是执行延迟之后的位置
        k_ahead = self.path.kappa_ahead(s, look)
        if k_ahead > 1e-6:
            v = min(v, math.sqrt(cfg.lat_accel_mps2 / k_ahead))
        if abs(e) > cfg.slow_offset_m:
            v *= max(0.4, 1.0 - (abs(e) - cfg.slow_offset_m) / 0.3)
        v = min(v, max(cfg.min_mps, math.sqrt(2.0 * cfg.decel_mps2 * max(0.0, remaining - cfg.stop_tol_m))))
        if cfg.start_creep_mps > 0 and started < cfg.start_creep_m:
            v = min(v, cfg.start_creep_mps)
        v = min(v, self._v_prev + cfg.accel_mps2 * dt)
        self._v_prev = v
        st.v_cmd = v
        w_hold = 0.0
        if cfg.start_hold_m > 0 and started < cfg.start_hold_m:
            # 起步锁车头：曲率反馈的车头纠正速度是 车速 × k_theta，不到 start_hold_rate 的差额直接给角速度
            # （车头误差用执行延迟之后的预估，与曲率反馈同一个参考方向）
            gain = max(0.0, cfg.start_hold_rate - v * self.k_theta)
            w_hold = max(-cfg.start_hold_max, min(cfg.start_hold_max, -gain * (he - theta_ref)))

        if s_now >= 0.0:
            self._n += 1
            self._sum2 += e_now * e_now
            self._max = max(self._max, abs(e_now))
            st.stats = dict(rms_m=math.sqrt(self._sum2 / self._n), max_m=self._max, samples=int(self._n))
        return v, kappa, w_hold

    def status(self):
        st = self.st
        return dict(state=st.state, reason=st.reason, e=st.e, heading_err_deg=math.degrees(st.heading_err),
                    s=st.s, length=st.length, kappa_ref=st.kappa_ref, v_cmd=st.v_cmd, kappa=st.kappa,
                    integral=st.integral, trim_kappa=-self.cfg.ki_per_m2 * st.integral, stats=dict(st.stats))


# ---- 前进 / 倒车交替的分段路径 ----

class Section:
    """分段路径里的一段：direction=+1 前进、−1 倒车。path 按行进方向画：倒车段的 yaw 是车头 + π，曲率也按行进方向。"""

    def __init__(self, direction, path):
        if direction not in (1, -1):
            raise ValueError('方向只能是 +1 前进或 −1 倒车')
        self.direction, self.path = direction, path

    def car_pose(self, s):
        """段内路程 s 处车的 (x, y, 车头)。"""
        x, y, yaw = self.path.pose(s)
        return x, y, yaw if self.direction > 0 else wrap(yaw + math.pi)


@dataclass
class _Travel:
    """倒车段交给 PathTracker 的位姿：按行进方向（车头 +π、车速取反）。"""
    x: float
    y: float
    yaw: float
    speed: float
    yaw_rate: float
    usable_for_control: bool


class SectionTracker:
    """分段路径跟踪：每段一个 PathTracker；段尾停稳（车速 < still_mps 持续 settle_s）后换向进下一段。

    接口与 PathTracker 相同（start / check_start / update / stop / status，st 汇总全程），line_mission 两种都能用。
    积分（舵机零位）按前进坐标保存：倒车坐标里曲率反号，零位偏差也反号，换段时跟着变号。
    """

    def __init__(self, sections, config=PathConfig(), geometry=None, *, reverse_mps=0.3, reverse_gain=0.90,
                 still_mps=0.02, settle_s=0.3, switch_timeout_s=5.0, pivot_rate=None, pivot_accel=1.0,
                 pivot_tol=math.radians(1.0), pivot_timeout_s=20.0, pivot_stop_tol=0.005,
                 pivot_min=math.radians(5.0), pivot_retries=2, pivot_floor=0.1):
        """pivot_rate 不为 None（差速车）：下一段起点车头方向与车差 pivot_min 以上时，停稳后原地转过去再开，
        起步时也一样；角速度不超过 pivot_rate、角加速度不超过 pivot_accel。阿克曼车为 None，不原地转。
        原地转仍按 pivot_tol 瞄准；转完差 pivot_min 以内不补转，直接慢速往前开，万向轮回正、车头由跟踪纠正
        （2026-09-30 实车：转完一停车头回弹约 1°，按 1° 补转时小指令推不动，反复补转卡住 27 s）。
        转完仍差 pivot_min 以上最多补转 pivot_retries 次，再不行按 pivot_timeout 中止。没转到位时角速度不低于
        pivot_floor（rad/s，0.1 约轮子 33 erpm；实车 ±5–19 erpm 收尾磨 1.6 s，±14 erpm 补转推不动）。"""
        self.sections = list(sections)
        if not self.sections:
            raise ValueError('路径是空的')
        self.cfg = config
        self.geo = geometry or ChassisGeometry()
        self.still_mps, self.settle_s, self.switch_timeout_s = still_mps, settle_s, switch_timeout_s
        if pivot_rate is not None and not (pivot_rate > 0 and pivot_accel > 0 and 0 < pivot_tol <= pivot_min):
            raise ValueError('原地转向的角速度、角加速度和容差必须为正')
        self.pivot_rate, self.pivot_accel, self.pivot_tol = pivot_rate, pivot_accel, pivot_tol
        self.pivot_min, self.pivot_retries, self.pivot_floor = pivot_min, pivot_retries, pivot_floor
        self.pivot_timeout_s = pivot_timeout_s
        self.trackers = []
        for i, sec in enumerate(self.sections):
            # 「离终点太近不起步」只对整条路线的最后一段有意义；后面还有段的（如 0.3–0.9 m 的接入、倒车助跑段）
            # 照样起步，否则短接入段一起步就报「已经过了路线终点」（2026-10-01 实车）
            cfg = config if len(self.sections) == 1 else \
                replace(config, start_min_left_m=min(config.start_min_left_m, config.stop_tol_m))
            if i > 0 and sec.direction > 0 > self.sections[i - 1].direction:
                cfg = replace(cfg, start_creep_m=max(cfg.start_creep_m, cfg.reverse_creep_m),
                              start_creep_mps=cfg.reverse_creep_mps or cfg.start_creep_mps)
            if getattr(sec,'cruise_mps',None) is not None:
                if not math.isfinite(sec.cruise_mps) or not .02<=sec.cruise_mps<=1.2:raise ValueError('分段速度无效')
                cfg=replace(cfg,cruise_mps=sec.cruise_mps)
            if pivot_rate is not None and i < len(self.sections) - 1:
                # 差速车段尾要停准：原地转 90° 后，停车差的那点前后距离就成了下一条边上的横偏
                cfg = replace(cfg, stop_tol_m=min(cfg.stop_tol_m, pivot_stop_tol))
            if sec.direction < 0:
                # 倒车实测只走出下发曲率的 0.90（前进 0.96），延迟相近
                cfg = replace(cfg, cruise_mps=min(cfg.cruise_mps, reverse_mps), track_x_m=-cfg.track_x_m,
                              steer_gain=reverse_gain)
                if cfg.reverse_section_creep_m is not None:
                    cfg = replace(cfg, start_creep_m=cfg.reverse_section_creep_m)
            self.trackers.append(PathTracker(sec.path, cfg, self.geo))
        self.kappa_max = self.trackers[0].kappa_max
        self.offsets, total = [], 0.0
        for sec in self.sections:
            self.offsets.append(total)
            total += sec.path.length
        self.length = total
        self.cusps = len(self.sections) - 1
        # 倒车换向次数、段与段之间原地转向的次数（差速车；阿克曼车为 0）
        self.reversals = sum(1 for a, b in zip(self.sections, self.sections[1:]) if a.direction != b.direction)
        self.pivots = 0 if pivot_rate is None else sum(
            1 for a, b in zip(self.sections, self.sections[1:])
            if abs(wrap(b.car_pose(0.0)[2] - a.car_pose(a.path.length)[2])) > pivot_min)
        self.index = 0
        self.switching = False          # 段尾已停车指令，等车停稳换向（差速车还可能要原地转向）
        self._next = 1                  # 换段时要开始的那一段（起步前原地转向时是 0）
        self._pivot = None              # 原地转向的目标车头方向；None 为没在转
        self._w = self._pivot_t = 0.0   # 原地转向当前下发的角速度、已经转了多久
        self._turns = 0                 # 这次起步 / 换段已经原地转了几次（补转计数）
        self._turn_clock = 0.0
        self._turn_sent = deque()      # 原地转向预估也记录零速：暂停、停稳等待时旧指令仍可能在执行
        self._still = self._wait = 0.0
        self._integral = 0.0            # 前进坐标
        self._done = [0.0, 0, 0.0]      # 已跑完各段的 横偏平方和、样本数、最大横偏
        self.st = PathState(length=total)

    def _view(self, sec, pose):
        if sec.direction > 0:
            return pose
        return _Travel(pose.x, pose.y, wrap(pose.yaw + math.pi), -pose.speed, pose.yaw_rate,
                       pose.usable_for_control)

    def check_start(self, pose, hint=None):
        i = self.index if self.st.state == 'PAUSED' else 0
        if i == 0 and self.st.state != 'PAUSED' and self.pivot_rate is not None and pose.usable_for_control \
                and self._misaligned(0, pose):
            # 差速车起步先原地转向：车头方向不算，只看位置在不在第一段起点附近
            pose = _Travel(pose.x, pose.y, self._heading(0), pose.speed, pose.yaw_rate, True)
        return self.trackers[i].check_start(self._view(self.sections[i], pose), hint)

    def start(self, pose, integral=None):
        st = self.st
        if st.state == 'PAUSED':
            if self.switching:                  # 换向途中定位丢过：恢复后接着等停稳、换向
                if not pose.usable_for_control:
                    st.reason = 'pose_unusable'
                    return False
                st.state, st.reason = 'RUNNING', ''
                return True
            tr = self.trackers[self.index]
            ok = tr.start(self._view(self.sections[self.index], pose))
            st.state, st.reason = ('RUNNING', '') if ok else ('PAUSED', tr.st.reason)
            return ok
        if integral is not None:
            self._integral = integral
        self.index, self.switching, self._pivot = 0, False, None
        self._turn_clock = 0.0
        self._turn_sent.clear()
        self._done = [0.0, 0, 0.0]
        sec, tr = self.sections[0], self.trackers[0]
        if self.pivot_rate is not None and pose.usable_for_control and self._misaligned(0, pose):
            # 差速车起步：先原地转到第一段的方向（位置照样要在第一段起点附近）
            ok, why = self.check_start(pose)
            if not ok:
                st.state, st.reason = 'IDLE', why
                return False
            self.switching, self._next, self._still, self._wait, self._turns = True, 0, 0.0, 0.0, 0
            st.state, st.reason, st.stats = 'RUNNING', '', {}
            self._sync()
            return True
        if not tr.start(self._view(sec, pose), integral=sec.direction * self._integral):
            st.state, st.reason = 'IDLE', tr.st.reason
            return False
        st.state, st.reason, st.stats = 'RUNNING', '', {}
        self._sync()
        return True

    def pause(self, reason='operator'):
        self.trackers[self.index].pause(reason)
        self.st.state, self.st.reason = 'PAUSED', reason
        self.st.v_cmd = 0.0
        return 0.0, 0.0

    def stop(self, reason='operator'):
        self.trackers[self.index].stop(reason)
        if self.st.state in ('RUNNING', 'PAUSED'):
            self.st.state, self.st.reason = 'ABORTED', reason
        self.st.v_cmd = 0.0
        return 0.0, 0.0

    def update(self, pose, dt):
        """返回 (vx, wz)，vx 倒车为负。pose 需有 x, y, yaw, speed, yaw_rate, usable_for_control。"""
        if self.pivot_rate is None:
            return self._update(pose, dt)
        self._turn_clock += dt
        output = self._update(pose, dt)
        self._turn_sent.append((self._turn_clock, output[1]))
        while len(self._turn_sent) >= 2 and self._turn_sent[1][0] <= self._turn_clock - self.cfg.delay_s + 1e-9:
            self._turn_sent.popleft()
        return output

    def _update(self, pose, dt):
        st = self.st
        if st.state not in ('RUNNING', 'PAUSED'):
            st.v_cmd = 0.0
            return 0.0, 0.0
        if self.switching:
            return self._switch(pose, dt)
        sec, tr = self.sections[self.index], self.trackers[self.index]
        v, w = tr.update(self._view(sec, pose), dt)
        self._sync()
        if tr.st.state == 'DONE':
            self._integral = sec.direction * tr.st.integral
            self._close(tr)
            if self.index == len(self.sections) - 1:
                st.state, st.reason = 'DONE', ''
            else:
                self.switching, self._next, self._still, self._wait, self._turns = True, self.index + 1, 0.0, 0.0, 0
            st.v_cmd = 0.0
            return 0.0, 0.0
        if tr.st.state in ('ABORTED', 'PAUSED'):
            st.state, st.reason = tr.st.state, tr.st.reason
            return 0.0, 0.0
        return sec.direction * v, w

    def _switch(self, pose, dt):
        st = self.st
        st.v_cmd = 0.0
        if st.state == 'PAUSED':
            self._w = 0.0               # 原地转向中暂停：车已停下，继续时从 0 重新加速
            return 0.0, 0.0
        if not pose.usable_for_control:
            st.state, st.reason = 'PAUSED', 'pose_unusable'
            self._w = 0.0
            return 0.0, 0.0
        i = self._next
        if self._pivot is not None:
            return self._turn(pose, dt)
        self._wait += dt
        still = abs(pose.speed) < self.still_mps
        if self.pivot_rate is not None:         # 差速车刚原地转完：转动也要停下来
            still = still and abs(pose.yaw_rate) < 0.05
        self._still = self._still + dt if still else 0.0
        if self._still < self.settle_s - 1e-9:
            if self._wait > self.switch_timeout_s:
                st.state, st.reason = 'ABORTED', 'not_stopped_at_cusp'
            return 0.0, 0.0
        if self.pivot_rate is not None and self._misaligned(i, pose):
            if self._turns > self.pivot_retries:        # 转了 1 + pivot_retries 次还差 pivot_min 以上：车转不过去
                st.state, st.reason = 'ABORTED', 'pivot_timeout'
                return 0.0, 0.0
            self._turns += 1
            self._pivot, self._w, self._pivot_t = self._heading(i), 0.0, 0.0
            return self._turn(pose, dt)
        sec, tr = self.sections[i], self.trackers[i]
        if not tr.start(self._view(sec, pose), integral=sec.direction * self._integral):
            st.state, st.reason = 'ABORTED', 'cusp_' + tr.st.reason
            return 0.0, 0.0
        self.index, self.switching = i, False
        self._sync()
        return 0.0, 0.0

    def _heading(self, i):
        """第 i 段起点的车头方向。"""
        return self.sections[i].car_pose(0.0)[2]

    def _misaligned(self, i, pose):
        """车头与第 i 段起点方向差 pivot_min 以上，要原地转；以内直接开，由跟踪纠正。"""
        return abs(wrap(self._heading(i) - pose.yaw)) > self.pivot_min

    def _turn(self, pose, dt):
        """原地转向：角速度按剩下的角度梯形规划（到目标正好停下），每步按角加速度限幅；转到容差内、
        角速度降到 0 后回去等停稳。返回 (0, wz)。"""
        st = self.st
        self._pivot_t += dt
        if self._pivot_t > self.pivot_timeout_s:
            st.state, st.reason = 'ABORTED', 'pivot_timeout'
            self._pivot, self._w = None, 0.0
            return 0.0, 0.0
        # 与 PathTracker.predict 相同：剩余角度按队列里的角速度全部执行完之后的车头算。
        # 小容差留出模型误差余量；末段收敛放缓到 1.4/s，覆盖 0.3–0.6 s 延迟、0.9–1.2 增益的失配。
        # 最高转速和角加速度不变；仍须经过 _switch 的实测停稳和最终航向检查。
        yaw = pose.yaw
        start = self._turn_clock - self.cfg.delay_s
        sent = self._turn_sent
        for i, (stamp, rate) in enumerate(sent):
            end = sent[i + 1][0] if i + 1 < len(sent) else self._turn_clock
            yaw += rate * self.cfg.steer_gain * max(0.0, end - max(start, stamp))
        err = wrap(self._pivot - yaw)
        tolerance = self.pivot_tol * (0.5 if self.cfg.delay_s > 0 else 1.0)
        if abs(err) <= tolerance:
            want = 0.0
        else:
            # 末段不低于 pivot_floor：实车原地转几 erpm 的小指令轮子推不动，按比例收尾会一直挂着
            want = math.copysign(max(min(self.pivot_floor, self.pivot_rate),
                                     min(self.pivot_rate,
                                         math.sqrt(2.0 * self.pivot_accel * (abs(err) - 0.5 * tolerance)),
                                         (1.4 if self.cfg.delay_s > 0 else 2.0) * abs(err))), err)
        step = self.pivot_accel * dt
        self._w += max(-step, min(step, want - self._w))
        if want == 0.0 and abs(self._w) < 1e-9:
            self._pivot, self._w, self._still, self._wait = None, 0.0, 0.0, 0.0
            return 0.0, 0.0
        return 0.0, self._w

    def _close(self, tr):
        stats = tr.st.stats
        if stats:
            n = stats['samples']
            self._done[0] += stats['rms_m'] ** 2 * n
            self._done[1] += n
            self._done[2] = max(self._done[2], stats['max_m'])

    def _sync(self):
        """把当前段的状态汇总到 st：路程、积分按全程 / 前进坐标换算，曲率按车头方向。"""
        st, sec, tr = self.st, self.sections[self.index], self.trackers[self.index]
        d = sec.direction
        st.e, st.heading_err, st.kappa_ref = tr.st.e, tr.st.heading_err, d * tr.st.kappa_ref
        st.s = self.offsets[self.index] + max(0.0, tr.st.s)
        st.v_cmd, st.kappa, st.integral = d * tr.st.v_cmd, d * tr.st.kappa, d * tr.st.integral
        sum2, n, peak = self._done
        cur = tr.st.stats if not self.switching else {}
        if cur:
            sum2 += cur['rms_m'] ** 2 * cur['samples']
            n += cur['samples']
            peak = max(peak, cur['max_m'])
        st.stats = dict(rms_m=math.sqrt(sum2 / n), max_m=peak, samples=int(n)) if n else {}

    def status(self):
        st = self.st
        return dict(state=st.state, reason=st.reason, e=st.e, heading_err_deg=math.degrees(st.heading_err),
                    s=st.s, length=st.length, kappa_ref=st.kappa_ref, v_cmd=st.v_cmd, kappa=st.kappa,
                    integral=st.integral, trim_kappa=-self.cfg.ki_per_m2 * st.integral, stats=dict(st.stats),
                    gear='reverse' if self.sections[self.index].direction < 0 else 'forward',
                    section=self.index + 1, sections=len(self.sections), switching=self.switching,
                    pivoting=self._pivot is not None)
