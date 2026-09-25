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

坐标：本地 ENU 平面（x 东 y 北），偏航东为 0、逆时针为正；曲率左转为正；横偏在路径左侧为正。
"""
import math
from dataclasses import dataclass, field

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
    lat_accel_mps2: float = 0.3      # 弯道限速：v ≤ √(a/|κ|)
    omega_per_m: float = 2.0         # 与直线控制器相同
    zeta: float = 0.9
    ki_per_m2: float = 0.6
    integral_limit: float = 0.12     # 积分项贡献的曲率上限 1/m
    approach_max_rad: float = math.radians(30.0)
    track_x_m: float = 0.0           # 要压在路径上的点（车体系 x），默认后轴中心
    lead_s: float = 0.10             # 延迟补偿：按当前速度、角速度把位姿往前推
    ff_preview_s: float = 0.05       # 曲率前馈再往前看这么久：舵机从一个角度转到另一个角度要时间
    kappa_tau_s: float = 0.15        # 反馈项低通
    kappa_margin: float = 0.9        # 路径曲率最多用到底盘最大曲率的这个比例，剩下的留给纠偏
    start_max_offset_m: float = 0.5
    start_max_heading_rad: float = math.radians(30.0)
    start_before_m: float = 3.0      # 入口是直线时，允许从起点后方最多这么远起步
    abort_offset_m: float = 0.5
    abort_heading_rad: float = math.radians(45.0)
    slow_offset_m: float = 0.10      # 横偏超过它开始降速，积分也停
    stop_tol_m: float = 0.02


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
        self.k_theta = 2.0 * config.zeta * config.omega_per_m
        self.k_e = config.omega_per_m ** 2 / self.k_theta
        self.st = PathState(length=path.length)
        self._hint = None
        self._v_prev = 0.0
        self._kfb = 0.0
        self._sum2 = self._n = self._max = 0.0

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
        if s > self.path.length - 0.5:
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
        return True

    def stop(self, reason='operator'):
        if self.st.state in ('RUNNING', 'PAUSED'):
            self.st.state, self.st.reason = 'ABORTED', reason
        self.st.v_cmd = 0.0
        return 0.0, 0.0

    def update(self, pose, dt):
        """返回 (vx, wz)。pose 需有 x, y, yaw, speed, yaw_rate, usable_for_control。"""
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

        # 延迟补偿：按当前速度和角速度往前推 lead_s（弯道上角速度不为 0，推出去的是一段圆弧）
        yaw = pose.yaw + pose.yaw_rate * cfg.lead_s
        mid = pose.yaw + 0.5 * pose.yaw_rate * cfg.lead_s
        x = pose.x + pose.speed * cfg.lead_s * math.cos(mid)
        y = pose.y + pose.speed * cfg.lead_s * math.sin(mid)
        s, e, he, k_r = self.locate(x, y, yaw, self._hint)
        ds = 0.0 if self._hint is None else max(0.0, s - self._hint)
        self._hint = s
        st.e, st.heading_err, st.s, st.kappa_ref = e, he, s, k_r

        if abs(e) > cfg.abort_offset_m or abs(he) > cfg.abort_heading_rad:
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
        k_ff = max(-self.kappa_max, min(self.kappa_max, k_ff))
        k_fb = max(-self.kappa_max - k_ff, min(self.kappa_max - k_ff, k_fb))
        self._kfb += (k_fb - self._kfb) * dt / (cfg.kappa_tau_s + dt)
        kappa = st.kappa = max(-self.kappa_max, min(self.kappa_max, k_ff + self._kfb))

        # 速度：巡航；前方刹车距离内有弯就按横向加速度限速；偏差大降速；终点前刹停
        v = cfg.cruise_mps
        look = v * v / (2.0 * cfg.decel_mps2) + v * cfg.lead_s + 0.3
        k_ahead = self.path.kappa_ahead(s, look)
        if k_ahead > 1e-6:
            v = min(v, math.sqrt(cfg.lat_accel_mps2 / k_ahead))
        if abs(e) > cfg.slow_offset_m:
            v *= max(0.4, 1.0 - (abs(e) - cfg.slow_offset_m) / 0.3)
        v = min(v, max(cfg.min_mps, math.sqrt(2.0 * cfg.decel_mps2 * max(0.0, remaining - cfg.stop_tol_m))))
        v = min(v, self._v_prev + cfg.accel_mps2 * dt)
        self._v_prev = v
        st.v_cmd = v

        if s >= 0.0:
            self._n += 1
            self._sum2 += e * e
            self._max = max(self._max, abs(e))
            st.stats = dict(rms_m=math.sqrt(self._sum2 / self._n), max_m=self._max, samples=int(self._n))
        return v, v * kappa

    def status(self):
        st = self.st
        return dict(state=st.state, reason=st.reason, e=st.e, heading_err_deg=math.degrees(st.heading_err),
                    s=st.s, length=st.length, kappa_ref=st.kappa_ref, v_cmd=st.v_cmd, kappa=st.kappa,
                    integral=st.integral, trim_kappa=-self.cfg.ki_per_m2 * st.integral, stats=dict(st.stats))
