#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RTK 走直线控制器：让车体上的一个点沿 A→B 直线前进。纯计算，不依赖 ROS。

横向控制按"空间"设计（对走过的距离而不是时间），车速变化时收敛距离不变：
    切入角  θ_ref = -atan(k_e·e)，限幅 approach_max        偏得远时以固定角度切回，不猛打
    曲率    κ = -k_θ·(θe - θ_ref) - k_i·∫e ds             小偏差时等价于 κ = -k_θ·k_e·e - k_θ·θe
k_θ·k_e = ω²、k_θ = 2ζω，ω = 2/m、ζ = 0.9 时约 1.5 m 收敛、几乎不超调。
积分项抵消舵机零位：1° 的零位偏差只靠比例会留下约 1.5 cm 的固定横偏。

坐标：本地 ENU 平面（x 东 y 北），偏航东为 0、逆时针为正；车体系原点后轴中心、x 前 y 左。
"""
import math
from dataclasses import dataclass, field

from robot_core.kinematics import ChassisGeometry


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class LineConfig:
    cruise_mps: float = 0.5
    accel_mps2: float = 0.5
    decel_mps2: float = 0.5
    min_mps: float = 0.06            # 终点前的蠕行速度，保证真的走到 B
    omega_per_m: float = 2.0         # 空间自然频率；再高压扰动更好，但定位噪声更容易抖方向
    zeta: float = 0.9
    ki_per_m2: float = 0.6           # 横偏积分（每米每米）
    integral_limit: float = 0.12     # 积分项贡献的曲率上限 1/m（约 3.6° 舵角）
    approach_max_rad: float = math.radians(30.0)
    track_x_m: float = 0.0           # 要压在线上的点（车体系 x），默认后轴中心
    lead_s: float = 0.10             # 延迟补偿：定位延迟之外的执行延迟
    kappa_tau_s: float = 0.15        # 曲率指令低通：挡住定位噪声直接抖方向，带宽内相位损失约 6°
    start_max_offset_m: float = 0.5
    start_max_heading_rad: float = math.radians(30.0)
    start_before_a_m: float = 3.0    # 允许从 A 点后方最多这么远起步（助跑）
    abort_offset_m: float = 0.5
    abort_heading_rad: float = math.radians(45.0)
    slow_offset_m: float = 0.10      # 横偏超过它开始降速
    stop_tol_m: float = 0.02
    min_line_m: float = 1.0


@dataclass
class LineState:
    state: str = 'IDLE'              # IDLE / RUNNING / PAUSED / DONE / ABORTED
    reason: str = ''
    e: float = 0.0                   # 横偏，m，线左侧为正
    heading_err: float = 0.0         # rad，车头相对直线方向
    s: float = 0.0                   # 沿线距离，m，A 为 0
    length: float = 0.0
    v_cmd: float = 0.0
    kappa: float = 0.0
    integral: float = 0.0            # ∫e ds
    stats: dict = field(default_factory=dict)


class LineTracker:
    def __init__(self, config=LineConfig(), geometry=None):
        self.cfg = config
        self.geo = geometry or ChassisGeometry()
        # 用共享转向模型的最小转弯半径；输出 wz = v·κ，底盘驱动用同一模型换回舵角
        self.kappa_max = 1.0 / self.geo.min_turn_radius_m
        self.k_theta = 2.0 * config.zeta * config.omega_per_m
        self.k_e = config.omega_per_m ** 2 / self.k_theta
        self.a = self.b = None
        self.st = LineState()
        self._last_s = None
        self._v_prev = 0.0
        self._sum2 = self._n = 0.0
        self._max = 0.0
        self._kappa_f = 0.0

    # ---- 任务 ----
    def set_line(self, a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length < self.cfg.min_line_m:
            raise ValueError('A、B 两点太近（%.2f m）' % length)
        self.a, self.b = tuple(a), tuple(b)
        self.u = (dx / length, dy / length)
        self.n = (-self.u[1], self.u[0])
        self.line_yaw = math.atan2(dy, dx)
        self.st = LineState(length=length)

    def geometry_of(self, x, y, yaw):
        """车体控制点相对直线的 (横偏, 车头误差, 沿线距离)。"""
        px = x + self.cfg.track_x_m * math.cos(yaw)
        py = y + self.cfg.track_x_m * math.sin(yaw)
        rx, ry = px - self.a[0], py - self.a[1]
        return (rx * self.n[0] + ry * self.n[1], wrap(yaw - self.line_yaw),
                rx * self.u[0] + ry * self.u[1])

    def check_start(self, pose):
        """能否从当前位姿开始；返回 (可以, 原因)。"""
        if self.a is None:
            return False, 'no_line'
        if not pose.usable_for_control:
            return False, 'pose_unusable'
        e, he, s = self.geometry_of(pose.x, pose.y, pose.yaw)
        if abs(e) > self.cfg.start_max_offset_m:
            return False, 'too_far_from_line'
        if abs(he) > self.cfg.start_max_heading_rad:
            return False, 'heading_not_aligned'
        if s < -self.cfg.start_before_a_m:
            return False, 'too_far_behind_a'
        if s > self.st.length - 0.5:
            return False, 'already_past_b'
        return True, ''

    def start(self, pose, integral=None):
        """开始或在暂停后继续；integral 不给时沿用当前值（暂停继续、或上次学到的舵机零位）。"""
        ok, why = self.check_start(pose)
        if not ok:
            if self.st.state != 'PAUSED':
                self.st.state = 'IDLE'
            self.st.reason = why
            return False
        self.st.state, self.st.reason = 'RUNNING', ''
        if integral is not None:
            self.st.integral = integral
        self._last_s = None
        self._kappa_f = 0.0
        self._v_prev = max(0.0, pose.speed)
        self._sum2 = self._n = self._max = 0.0
        return True

    def stop(self, reason='operator'):
        if self.st.state in ('RUNNING', 'PAUSED'):
            self.st.state, self.st.reason = 'ABORTED', reason
        self.st.v_cmd = 0.0
        return 0.0, 0.0

    # ---- 每个控制周期 ----
    def update(self, pose, dt):
        """返回 (vx, wz)。pose 需有 x, y, yaw, speed, yaw_rate, usable_for_control。"""
        st, cfg = self.st, self.cfg
        if st.state not in ('RUNNING', 'PAUSED'):
            st.v_cmd = 0.0
            return 0.0, 0.0
        if not pose.usable_for_control:
            # 定位不可用：立即停，恢复后由操作员决定是否继续
            st.state, st.reason = 'PAUSED', 'pose_unusable'
            st.v_cmd, self._v_prev = 0.0, 0.0
            return 0.0, 0.0
        if st.state == 'PAUSED':
            # 定位恢复也不自动继续，等操作员再按开始（start 会重新检查起步条件）
            st.reason = 'pose_recovered_waiting_operator'
            return 0.0, 0.0

        # 延迟补偿：按当前速度和角速度往前推 lead_s
        yaw = pose.yaw + pose.yaw_rate * cfg.lead_s
        mid = pose.yaw + 0.5 * pose.yaw_rate * cfg.lead_s
        x = pose.x + pose.speed * cfg.lead_s * math.cos(mid)
        y = pose.y + pose.speed * cfg.lead_s * math.sin(mid)
        e, he, s = self.geometry_of(x, y, yaw)
        st.e, st.heading_err, st.s = e, he, s

        if abs(e) > cfg.abort_offset_m or abs(he) > cfg.abort_heading_rad:
            st.state, st.reason = 'ABORTED', 'off_line'
            st.v_cmd, self._v_prev = 0.0, 0.0
            return 0.0, 0.0
        remaining = st.length - s
        if remaining <= cfg.stop_tol_m:
            st.state, st.reason = 'DONE', ''
            st.v_cmd, self._v_prev = 0.0, 0.0
            return 0.0, 0.0

        # 积分只在前进且贴近直线时累积，并限幅，防止切入阶段积满
        ds = 0.0 if self._last_s is None else max(0.0, s - self._last_s)
        self._last_s = s
        if abs(e) < cfg.slow_offset_m:
            lim = cfg.integral_limit / cfg.ki_per_m2
            st.integral = max(-lim, min(lim, st.integral + e * ds))

        theta_ref = -math.atan(self.k_e * e)
        theta_ref = max(-cfg.approach_max_rad, min(cfg.approach_max_rad, theta_ref))
        kappa = -self.k_theta * (he - theta_ref) - cfg.ki_per_m2 * st.integral
        kappa = max(-self.kappa_max, min(self.kappa_max, kappa))
        self._kappa_f += (kappa - self._kappa_f) * dt / (cfg.kappa_tau_s + dt)
        kappa = st.kappa = self._kappa_f

        # 速度：巡航、偏差降速、终点前按减速度刹停
        v = cfg.cruise_mps
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
                    s=st.s, length=st.length, v_cmd=st.v_cmd, kappa=st.kappa, integral=st.integral,
                    trim_kappa=-self.cfg.ki_per_m2 * st.integral, stats=dict(st.stats))
