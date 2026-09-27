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


def differential_config(config=PathConfig()):
    """差速车（Gen5）的跟踪参数：直接下发车速和角速度，没有前轮转向的执行延迟、也不会转不够。
    轮毂电机的响应实车还没辨识，延迟先按 0.1 s。"""
    return replace(config, delay_s=0.1, steer_gain=1.0)


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
        self._kfb = 0.0
        self._sum2 = self._n = self._max = 0.0
        self._clock = 0.0
        self._sent = deque()            # (下发时刻, 车速, 下发曲率)：还在底盘执行延迟里的指令

    def locate(self, x, y, yaw, hint=None):
        """车体控制点相对路径的 (路程, 横偏, 车头误差, 路径曲率)。"""
        px = x + self.cfg.track_x_m * math.cos(yaw)
        py = y + self.cfg.track_x_m * math.sin(yaw)
        s, e, path_yaw, k = self.path.project(px, py, hint, extend_start=self.cfg.start_before_m)
        return s, e, wrap(yaw - path_yaw), k

    def check_start(self, pose, hint=None):
        if not pose.usable_for_control:
            return False, 'pose_unusable'
        s, e, he, _ = self.locate(pose.x, pose.y, pose.yaw, hint)
        if abs(e) > self.cfg.start_max_offset_m:
            return False, 'too_far_from_line'
        if abs(he) > self.cfg.start_max_heading_rad:
            return False, 'heading_not_aligned'
        if s < -self.cfg.start_before_m + 1e-6:
            return False, 'too_far_behind_a'
        if s > self.path.length - self.cfg.start_min_left_m:
            return False, 'already_past_b'
        return True, ''

    def start(self, pose, integral=None):
        # 暂停后继续：在暂停处附近找投影，多圈路线不会跳回第一圈、8 字交叉处不会跳到另一支
        hint = self._hint if self.st.state == 'PAUSED' else None
        ok, why = self.check_start(pose, hint)
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
        v, kappa = self._control(pose, dt)
        self._sent.append((self._clock, v, kappa))
        return v, v * kappa

    def predict(self, x, y, yaw):
        """已发出、还在执行延迟里的指令都执行完后的 (x, y, yaw)。

        第 i 条指令在 [t_i, t_i+1) 期间有效、晚 delay_s 才作用到车上，所以车接下来 delay_s 内怎么走，由
        [现在 − delay_s, 现在) 期间有效的那些指令决定；车实际曲率 = 下发曲率 × steer_gain。
        """
        d = self.cfg.delay_s
        sent = self._sent
        while len(sent) >= 2 and sent[1][0] <= self._clock - d + 1e-9:
            sent.popleft()
        if d <= 0.0:
            return x, y, yaw
        start = self._clock - d
        for i, (t, v, k) in enumerate(sent):
            end = sent[i + 1][0] if i + 1 < len(sent) else self._clock
            a = max(t, start)
            if end <= a or v == 0.0:
                continue
            x, y, yaw = Piece(x, y, yaw, 0.0, k * self.cfg.steer_gain, 0.0).pose(v * (end - a))
        return x, y, yaw

    def _control(self, pose, dt):
        """返回 (车速, 下发曲率)。"""
        st, cfg = self.st, self.cfg
        if st.state not in ('RUNNING', 'PAUSED'):
            st.v_cmd = 0.0
            return 0.0, 0.0
        if not pose.usable_for_control:
            st.state, st.reason = 'PAUSED', 'pose_unusable'
            st.v_cmd, self._v_prev = 0.0, 0.0
            return 0.0, 0.0
        if st.state == 'PAUSED':
            st.reason = 'pose_recovered_waiting_operator'
            return 0.0, 0.0

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
            return 0.0, 0.0
        remaining = self.path.length - s
        if remaining <= cfg.stop_tol_m:
            st.state, st.reason = 'DONE', ''
            st.v_cmd, self._v_prev = 0.0, 0.0
            return 0.0, 0.0

        if abs(e) < cfg.slow_offset_m:
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
        v = min(v, self._v_prev + cfg.accel_mps2 * dt)
        self._v_prev = v
        st.v_cmd = v

        if s_now >= 0.0:
            self._n += 1
            self._sum2 += e_now * e_now
            self._max = max(self._max, abs(e_now))
            st.stats = dict(rms_m=math.sqrt(self._sum2 / self._n), max_m=self._max, samples=int(self._n))
        return v, kappa

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
                 pivot_tol=math.radians(1.0), pivot_timeout_s=20.0, pivot_stop_tol=0.005):
        """pivot_rate 不为 None（差速车）：下一段起点车头方向与车差 pivot_tol 以上时，停稳后原地转过去再开，
        起步时也一样；角速度不超过 pivot_rate、角加速度不超过 pivot_accel。阿克曼车为 None，不原地转。"""
        self.sections = list(sections)
        if not self.sections:
            raise ValueError('路径是空的')
        self.cfg = config
        self.geo = geometry or ChassisGeometry()
        self.still_mps, self.settle_s, self.switch_timeout_s = still_mps, settle_s, switch_timeout_s
        if pivot_rate is not None and not (pivot_rate > 0 and pivot_accel > 0 and pivot_tol > 0):
            raise ValueError('原地转向的角速度、角加速度和容差必须为正')
        self.pivot_rate, self.pivot_accel, self.pivot_tol = pivot_rate, pivot_accel, pivot_tol
        self.pivot_timeout_s = pivot_timeout_s
        self.trackers = []
        for i, sec in enumerate(self.sections):
            cfg = config if i == 0 else replace(config, start_min_left_m=min(config.start_min_left_m, config.stop_tol_m))
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
            if abs(wrap(b.car_pose(0.0)[2] - a.car_pose(a.path.length)[2])) > pivot_tol)
        self.index = 0
        self.switching = False          # 段尾已停车指令，等车停稳换向（差速车还可能要原地转向）
        self._next = 1                  # 换段时要开始的那一段（起步前原地转向时是 0）
        self._pivot = None              # 原地转向的目标车头方向；None 为没在转
        self._w = self._pivot_t = 0.0   # 原地转向当前下发的角速度、已经转了多久
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
        self._done = [0.0, 0, 0.0]
        sec, tr = self.sections[0], self.trackers[0]
        if self.pivot_rate is not None and pose.usable_for_control and self._misaligned(0, pose):
            # 差速车起步：先原地转到第一段的方向（位置照样要在第一段起点附近）
            ok, why = self.check_start(pose)
            if not ok:
                st.state, st.reason = 'IDLE', why
                return False
            self.switching, self._next, self._still, self._wait = True, 0, 0.0, 0.0
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
                self.switching, self._next, self._still, self._wait = True, self.index + 1, 0.0, 0.0
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
        return abs(wrap(self._heading(i) - pose.yaw)) > self.pivot_tol

    def _turn(self, pose, dt):
        """原地转向：角速度按剩下的角度梯形规划（到目标正好停下），每步按角加速度限幅；转到容差内、
        角速度降到 0 后回去等停稳。返回 (0, wz)。"""
        st = self.st
        self._pivot_t += dt
        if self._pivot_t > self.pivot_timeout_s:
            st.state, st.reason = 'ABORTED', 'pivot_timeout'
            self._pivot, self._w = None, 0.0
            return 0.0, 0.0
        err = wrap(self._pivot - pose.yaw)
        if abs(err) <= self.pivot_tol:
            want = 0.0
        else:
            want = math.copysign(min(self.pivot_rate, math.sqrt(2.0 * self.pivot_accel * (abs(err) - 0.5 * self.pivot_tol)),
                                     2.0 * abs(err)), err)
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
