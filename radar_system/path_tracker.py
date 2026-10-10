#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RTK 路径跟踪：直线段 + 圆弧段首尾相接的路径（小圆、跑道形、8 字等）。纯计算，不依赖 ROS。

控制律是 line_tracker 的推广，同样按"空间"设计（对走过的路程），车速变化时收敛距离不变：
    κ    = κ_ff + κ_fb
    κ_ff = κ_r(s + 预瞄)·cosθe / (1 − κ_r·e)                    路径本身的曲率（前馈），直线上为 0
    κ_fb = −k_θ·(θe − θ_ref) − k_i·∫e ds，θ_ref = −atan(k_e·e)   与直线控制器相同的横偏/航向反馈
小偏差时横偏满足 e'' + 2ζω·e' + ω²·e = 0（' 是对路程求导）。低通只加在反馈项上（挡住定位噪声），
前馈来自路径、没有噪声，不滤，弯道入口不会因滤波变迟。速度按前方曲率限制横向加速度，终点前按减速度刹停。
差速车的前馈在曲率突变处分两步给（ff_shape_*，躲底盘角速度超调，见 differential_config）。

路径投影只在上一次位置附近找最近点：小圆转第二圈、8 字交叉处不会跳到别的圈上。

底盘执行延迟（实车 2026-09-26 两趟连线路线记录辨识）：指令发出约 0.33 s 后车才开始按新曲率走，实际曲率只有
指令的 0.96（前进）/ 0.90（倒车）。所以控制器不按"现在"的位姿算，而按"已发出、还没生效的指令都执行完"之后
的位姿算（Smith 预估，delay_s），现在算出的指令正好在那时生效；下发曲率除以 steer_gain 补上转不够的部分。
弯道按横向加速度 lat_accel_mps2 限速，满舵弯约 0.3 m/s：延迟期间走过的路程短，入弯出弯的偏差小。

前进、倒车交替的路线（连线路线的倒车补角）由 SectionTracker 按段跟踪：每段一个 PathTracker，换向处停稳；明确标记的相切前进段保持速度、预瞄历史连续交接。
倒车段在"行进方向"坐标里跟踪（车头 +π、车速取反、控制点前后对调）：后轴中心的运动学在这个坐标里与前进
完全相同，只是曲率取反，所以两个方向用同一套控制律。

坐标：本地 ENU 平面（x 东 y 北），偏航东为 0、逆时针为正；曲率左转为正；横偏在路径左侧为正。
"""
from robot_core.standstill import stationary
import math
from types import SimpleNamespace
from motion_speeds import MAX_SECTION_SPEED, TURN_SPEED_MPS, PAINT_ARC_SPEED_MPS, transit_bend_speed
from robot_core.config import CONTROL_DT
from collections import deque
from dataclasses import dataclass, field, replace

from robot_core.kinematics import ChassisGeometry
from line_tracker import lateral_integral_allowed, position_observation_allowed
from heading_control import HeadingControl, HeadingControlConfig
from heading_hold import HoldConfig
from paint_start import PaintStartGuard
from line_cascade import (CascadeConfig, ImuFirstConfig, ImuFirstGuide, imu_first_target, learn_heading_offset,
                          lateral_heading_step, paint_pull, runup_weight)


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

# 静止直线起步：共同提速参考与相对 IMU 稳向；只在绝对方向尚未接管时使用。
LAUNCH_ACCEL_MPS2 = .5
LAUNCH_JERK_MPS3 = 2.0
LAUNCH_HOLD_MAX_S = 3.0
LAUNCH_HOLD_MAX_M = 1.0
LAUNCH_HEADING_LIMIT_RAD = math.radians(2.0)

@dataclass(frozen=True)
class PathConfig:
    cruise_mps: float = 0.5
    accel_mps2: float = 0.5
    decel_mps2: float = 0.5
    run_out_decel_max_mps2: float = 1.0
    min_mps: float = 0.06            # 终点前的蠕行速度
    lat_accel_mps2: float = 0.05     # 弯道限速：v ≤ √(a/|κ|)，满舵弯（R 1.92 m）约 0.31 m/s
    arc_speed_mps: float | None = None  # Gen5 前进圆弧固定后轴速度；起停与故障处理保留。
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
    # 前馈曲率两脉冲整形（ZV）：曲率突变（直线进弧、出弧）时先给 1 − ff_shape_ratio，晚 ff_shape_gap_s 再补上，
    # 躲开底盘角速度的欠阻尼超调；路径本身不变。突变两边有一边 |κ| 超过 ff_shape_kappa_max（贴着最小半径的弧，
    # 少给的前馈靠反馈补不回来）就不整形。0 为不用（阿克曼车）；差速车见 differential_config
    ff_shape_ratio: float = 0.0
    ff_shape_gap_s: float = 0.48
    ff_shape_kappa_max: float = 1.2
    # 仅从直线进入可整形圆弧时覆盖前馈参数；None 保持既有行为，出弧不受影响。
    ff_entry_ratio: float | None = None
    ff_entry_gap_s: float | None = None
    ff_entry_preview_s: float | None = None
    ff_entry_clockwise_only: bool = False
    ff_entry_min_cruise_mps: float = 0.0
    ff_entry_full_circle_only: bool = False
    ff_entry_min_radius_m: float = 0.0
    ff_entry_max_radius_m: float | None = None
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
    # 起步先稳再纠偏（差速车，2026-10-10 用户：「起步先稳再纠偏」）：每段从停着起步、还没到喷线区时，原控制先不拉
    # 横偏、不积分，只保持车头；路线曲率前馈之外的转向限在 ±launch_settle_max_deg_s，车速目标不超过 launch_settle_mps；
    # 走满 launch_settle_min_m 且车身转速连续 launch_settle_calm_s 跟下发转速差在 launch_settle_rate_deg_s 内（两轮同步了），或走满
    # launch_settle_max_m，再在 launch_settle_blend_s 内把限幅放开（横偏纠正、加速照原控制）。
    # 10-10 实录 21 次起步：下发转速峰中位 5°/s，车身实际 13°/s（两轮先后动最多差 0.3 s、各冲过头约 11%），
    # 起步时一边纠横偏一边轮子不同步，摆动峰峰中位 4.7°、最大 7.4°。
    # 按实测拟合的轮子模型（速度环约 1 Hz 欠阻尼、起步随机晚 0–0.4 s）对比（助跑同长）：限 2°/s、交回 1 s 时
    # 起步 3 s 摆动中位少约 25%、最大少 30–40%，线头 2 m 内最大偏差变化在 ±0.15 cm 内；限 1°/s 摆得更小，但起步
    # 车头已偏 5° 时线头差到 3 cm（修车头太慢），不用。0 为不用
    launch_settle_max_m: float = 0.0
    launch_settle_min_m: float = 0.3
    launch_settle_rate_deg_s: float = 2.0
    launch_settle_calm_s: float = 0.3
    launch_settle_max_deg_s: float = 2.0
    launch_settle_mps: float = 0.35
    launch_settle_blend_s: float = 1.0
    # Gen5前进长直线：快速IMU航向内环、慢喷头横偏外环；基础配置关闭以兼容旧车与纯几何调用方。
    straight_imu_enabled: bool = False
    straight_min_length_m: float = 6.0
    straight_settle_m: float = 1.0
    straight_min_speed_mps: float = 0.35
    straight_blend_s: float = 0.8
    straight_outer_k_e: float = 0.6
    straight_outer_k_i: float = 0.05
    straight_outer_tau_s: float = 0.5  # 旧路线外环参数保留兼容；A/B 控制律不再使用偏角低通。
    # 以下趋势试验参数保留配置兼容；共用的 A/B 控制律不使用趋势反馈。
    straight_outer_rate_gain: float = 0.0
    straight_outer_rate_tau_s: float = 0.4
    straight_outer_rate_model_weight: float = 1.0
    straight_integral_return_gate_mps: float = 0.0
    straight_outer_trend_min_speed_mps: float = 0.0
    straight_outer_i_boost: float = 0.0
    straight_outer_tau_scale: float = 1.0
    straight_bias_max_deg: float = 3.0
    straight_offset_tau_s: float = 10.0
    straight_target_rate_deg_s: float = 2.0
    straight_kp: float = 2.2
    straight_ki: float = 1.0
    straight_kd: float = 0.4
    straight_drift_tc: float = 1.5
    straight_nozzle_x_m: float = 0.30
    straight_target_feedforward: bool = True
    straight_target_rate_tau_s: float = 0.20
    straight_prediction_delay_s: float | None = None
    straight_outer_prediction_weight: float = 0.0
    straight_direct_rate_damping: bool = False
    # 长直线一直到本段终点（停车）时，末尾提前这么多米交回原路径跟踪（None：按车速算，至少 1 m；后面接弯道时总按车速算）。
    # 交回后原跟踪按融合航向开，融合航向有零点几度的
    # 慢漂时（测向慢漂），最后这一米多就带偏 1 cm 以上（2026-10-02 全链路仿真：矩形每条边末尾 +0.4 → +1.7 cm）；
    # 车速、停车都还由路径跟踪管，直行内环只管转向，Gen5 一直管到这段末尾
    straight_end_reserve_m: float | None = None
    # 助跑（runup_speed_mps 不为空、还没到线头）不从 0 m 就交给 A/B 稳向：先由原路径跟踪起步锁车头、按横偏纠正，
    # 这次起步走满 start_hold_m 且横偏、车头误差都收进下面两个门槛才渐入 A/B（交过一次就不再收回）。
    # 普通助跑到线头照常渐入；倒车接入未收稳时继续原跟踪，基线不可信时保留原有交接。
    # 2026-10-06 实车：A/B 从助跑 0.15 m 就接管（偏角上限 3°、目标 3°/s），起步偏 2–5 cm、车头 4° 时横偏先涨到 5–8 cm，
    # 线头喷头偏 3.2–5.5 cm（5 趟 3 趟停喷）；10-05 旧交接（0.6 m 后、|e|<5 cm、|车头|<3°）25 趟线头中位 0.84、最大 2.6 cm
    runup_release_e_m: float = 0.02
    runup_release_heading_deg: float = 3.0
    # 接入、转场（不喷）离正式路线（喷线起点）这么近以内不按转场速度：按刹车曲线降到画线速度，准备接入正式路线
    # （2026-10-07 用户：「离点很远也要提速到 1.5，转场和接入都在 3 米内就不要 1.5，要准备接入正式路线，要稳」）
    approach_zone_m: float = 3.0
    # 长直线 IMU 为主、卫星辅助（line_cascade.ImuFirstConfig，2026-10-06 用户）；None 用原 A/B 控制律
    straight_imu_first: ImuFirstConfig | None = None
    # 画圆（整圆那一段）也 IMU 为主（2026-10-06 晚用户：起点终点交接错开 1–2 cm，选「画圆也 IMU 为主」）：原路径跟踪的
    # 弯道控制律不变，只换它看到的位姿——车头 = IMU 偏航 + 接入时学的换算（同 straight_imu_first），横偏 = IMU 推算
    # + 卫星慢慢拟合（可靠才用、突然跳开不跟），沿圆路程按走过的路推、往卫星挪（距离常数 arc_s_tau_m、每米最多 arc_s_rate_m，
    # 车速读数的比例按 arc_scale_tau_m 慢慢学：05:41 回放车速偏大且抖 ±25%，只按 1 cm/m 挪跟不上，推的路程跑到卫星前面 9 cm，
    # R 2.4 m 的圆切线就差 2°）。停车、喷不喷仍按
    # 卫星。05:41 那圈：原地转完偏 4.6 cm 没收稳就起喷（偏内 2.3 cm），起喷时卫星往里跳 1.3 cm、2.4 m 后 0.4 s 内往外跳
    # 3.2 cm（陀螺稳、车走不出来），车跟着卫星走，交界错开 2.5–2.9 cm
    arc_imu_first: bool = False
    # 非整圆作业弧：预走时学习，跨喷线分段连续保持；径向校正与融合航向/进度分开。
    arc_partial_imu_first: bool = False
    arc_partial_min_radius_m: float = 1.0
    arc_partial_rtk_pull_m: float = 0.07
    arc_partial_rtk_error_m: float = 0.004
    arc_partial_rtk_noise_m: float = 0.001
    arc_partial_tight_feedback_scale: float = 3.5
    arc_partial_rtk_feedback_scale: float = 4.5
    arc_partial_heading_scale: float = 2.5
    arc_partial_response_delay_s: float = .15
    arc_partial_response_tau_s: float = .10
    arc_s_rate_m: float = 0.05
    arc_s_tau_m: float = 0.4
    arc_scale_tau_m: float = 2.0
    arc_heading_tau_s: float = 5.0   # 换算跟融合航向的时间常数：原地转完测向还要几秒才回来，短时间按 IMU
    arc_pull_m: float = 1.0          # 横偏往卫星拉的距离常数
    arc_pull_rate_m: float = 0.02    # 每米最多拉这么多
    # 作业圆弧可靠双天线观测持续确认后，径向估计及时跟随；航向仍由 IMU 平滑维持。
    arc_rtk_radial_enabled: bool = True
    arc_rtk_pull_m: float = 0.10
    arc_rtk_pull_rate_m: float = 0.12
    arc_rtk_pair_max_m: float = 0.015
    arc_rtk_settle_s: float = 0.30
    arc_rtk_fresh_s: float = 0.25
    arc_rtk_error_m: float = 0.010  # 差异达到此值时使用完整额外校正。
    arc_rtk_noise_m: float = 0.002  # 毫米级差异沿用原平缓校正，避免放大正常定位噪声。
    arc_rtk_feedback_scale: float = 1.5  # 可靠原始双天线径向观测下的横向修正比例；不改变航向阻尼。
    # 整圆卫星好时的慢修（2026-10-11 用户「修复a」）。10-10 R2 实车圆：换算跟着测向绕圈晃约 0.6°、推算每米漂约 1 cm，按
    # arc_pull_m 1 m 才拉回一半，估计稳定落后卫星 1–2 cm（上面的快修那圈权重平均 0.09：单帧对不上就清零，攒不满）。
    # 固定解、卫星 ≥ fit_full_sats、σ ≤ fit_max_sigma_m、两天线距离误差 ≤ arc_good_pair_max_m、离上次跳点
    # ≥ arc_good_after_jump_m 时，慢修改按 arc_good_pull_*；那圈回放估计与卫星之差 95% 1.8 → 1.2 cm（加姿态零点 0.9），抖动不变。
    # 多路径多半只偏一根天线，两天线一比就退回原慢修（回放主天线慢漂 3 cm：一直快 2.7、只在好时快 1.7、原来 1.8 cm）。
    # 只用于整圆；作业圆弧另有一套
    arc_good_pull_m: float = 0.3
    arc_good_pull_rate_m: float = 0.05
    arc_good_pair_max_m: float = 0.02
    arc_good_after_jump_m: float = 1.0

    accel_jerk_mps3: float = 0.0   # 正向提速的加速度变化上限；0 保留原控制。减速/保护独立优先。
    turn_decel_mps2: float = .25   # 正常弯前舒适减速；故障/停车约束独立优先。
    turn_jerk_mps3: float = .5

    def __post_init__(self):
        if not math.isfinite(self.accel_jerk_mps3) or self.accel_jerk_mps3<0:
            raise ValueError('提速加加速度必须为非负有限值')
        if self.arc_speed_mps is not None and (not math.isfinite(self.arc_speed_mps)
                or not 0 < self.arc_speed_mps <= MAX_SECTION_SPEED):
            raise ValueError('圆弧固定速度必须为空或有效正速度')
        if not math.isfinite(self.run_out_decel_max_mps2) or not 0 < self.run_out_decel_max_mps2 <= 1.0:
            raise ValueError('run_out_decel_max_mps2 必须在 (0,1] 内')
        for name in ('straight_imu_enabled', 'straight_target_feedforward', 'straight_direct_rate_damping',
                     'arc_rtk_radial_enabled', 'arc_partial_imu_first'):
            if type(getattr(self, name)) is not bool:
                raise ValueError(name + ' 必须是布尔值')
        for name in ('turn_decel_mps2', 'turn_jerk_mps3', 'straight_min_length_m', 'straight_blend_s',
                     'straight_outer_k_e', 'straight_outer_tau_s', 'straight_bias_max_deg', 'straight_offset_tau_s',
                     'straight_target_rate_deg_s', 'straight_kp', 'straight_drift_tc', 'straight_target_rate_tau_s',
                     'straight_outer_rate_tau_s', 'straight_outer_tau_scale',
                     'runup_release_e_m', 'runup_release_heading_deg', 'arc_s_rate_m', 'arc_s_tau_m', 'arc_scale_tau_m',
                     'arc_heading_tau_s', 'arc_pull_m', 'arc_pull_rate_m', 'arc_rtk_pull_m',
                     'arc_rtk_pull_rate_m', 'arc_rtk_pair_max_m', 'arc_rtk_settle_s', 'arc_rtk_fresh_s',
                     'arc_rtk_error_m', 'arc_rtk_feedback_scale', 'arc_good_pull_m', 'arc_good_pull_rate_m',
                     'arc_good_pair_max_m', 'arc_good_after_jump_m', 'arc_partial_min_radius_m',
                     'arc_partial_rtk_pull_m', 'arc_partial_rtk_error_m', 'arc_partial_rtk_feedback_scale',
                     'arc_partial_tight_feedback_scale',
                     'arc_partial_heading_scale', 'arc_partial_response_delay_s', 'arc_partial_response_tau_s'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(name + ' 必须是有限正数')
        for name in ('straight_settle_m', 'straight_min_speed_mps', 'straight_outer_k_i', 'straight_ki', 'straight_kd', 'straight_outer_rate_gain',
                     'straight_integral_return_gate_mps', 'straight_outer_trend_min_speed_mps',
                     'straight_outer_i_boost', 'approach_zone_m'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(name + ' 必须是有限非负数')
        if not math.isfinite(self.arc_rtk_noise_m) or not 0 <= self.arc_rtk_noise_m < self.arc_rtk_error_m:
            raise ValueError('arc_rtk_noise_m 必须非负且小于 arc_rtk_error_m')
        if not math.isfinite(self.arc_partial_rtk_noise_m) or not 0 <= self.arc_partial_rtk_noise_m < self.arc_partial_rtk_error_m:
            raise ValueError('arc_partial_rtk_noise_m 必须非负且小于 arc_partial_rtk_error_m')
        if self.straight_imu_first is not None and not isinstance(self.straight_imu_first, ImuFirstConfig):
            raise ValueError('straight_imu_first 必须为空或 ImuFirstConfig')
        if not math.isfinite(self.straight_nozzle_x_m):
            raise ValueError('straight_nozzle_x_m 必须是有限数值')
        if self.straight_prediction_delay_s is not None and (not math.isfinite(self.straight_prediction_delay_s)
                or self.straight_prediction_delay_s < 0):
            raise ValueError('straight_prediction_delay_s 必须为空或有限非负数')
        if not math.isfinite(self.straight_outer_prediction_weight) or not 0 <= self.straight_outer_prediction_weight <= 1:
            raise ValueError('straight_outer_prediction_weight 必须在 0–1 之间')
        if not math.isfinite(self.straight_outer_rate_model_weight) or not 0 <= self.straight_outer_rate_model_weight <= 1:
            raise ValueError('straight_outer_rate_model_weight 必须在 0–1 之间')
        if not math.isfinite(self.ff_shape_ratio) or not 0 <= self.ff_shape_ratio < 1:
            raise ValueError('ff_shape_ratio 必须在 0–1 之间（不含 1）')
        if not math.isfinite(self.ff_shape_gap_s) or self.ff_shape_gap_s < 0:
            raise ValueError('ff_shape_gap_s 必须是有限非负数')
        if not math.isfinite(self.ff_shape_kappa_max) or self.ff_shape_kappa_max <= 0:
            raise ValueError('ff_shape_kappa_max 必须是有限正数')
        if type(self.ff_entry_clockwise_only) is not bool:
            raise ValueError('ff_entry_clockwise_only 必须是布尔值')
        if type(self.ff_entry_full_circle_only) is not bool:
            raise ValueError('ff_entry_full_circle_only 必须是布尔值')
        if not math.isfinite(self.ff_entry_min_cruise_mps) or self.ff_entry_min_cruise_mps < 0:
            raise ValueError('ff_entry_min_cruise_mps 必须是有限非负数')
        if not math.isfinite(self.ff_entry_min_radius_m) or self.ff_entry_min_radius_m < 0:
            raise ValueError('ff_entry_min_radius_m 必须是有限非负数')
        if self.ff_entry_max_radius_m is not None and (not math.isfinite(self.ff_entry_max_radius_m)
                or self.ff_entry_max_radius_m <= self.ff_entry_min_radius_m):
            raise ValueError('ff_entry_max_radius_m 必须为空或大于最小半径的有限数')
        if self.ff_entry_ratio is not None and (not math.isfinite(self.ff_entry_ratio) or not 0 <= self.ff_entry_ratio < 1):
            raise ValueError('ff_entry_ratio 必须为空或在 [0,1) 内')
        for name in ('ff_entry_gap_s', 'ff_entry_preview_s'):
            value = getattr(self, name)
            if value is not None and (not math.isfinite(value) or value < 0):
                raise ValueError(name + ' 必须为空或有限非负数')


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
    横偏积分 ki 0.6 → 2.0 1/m²（2026-10-01 下午）：双天线测向（基线 0.4 m，接收机自报 σ≈1°）每趟偏 −0.5～+1.2° 不等、
    同一条线来回都不一样，不是固定安装偏差（下午 8 趟中位 −0.02°），补固定角度会让 6 趟变差；16:26:24 测向偏 +1.17°，
    车与线平行地偏 −1.1 cm 走完 14 m，0.6 的积分拉不回来。RTK 横偏是准的，积分按它把任何来源的慢偏差（测向、侧滑、
    安装）都吃掉：仿真测向偏 1.1° 时 15 m 直线喷头 p95 2.05 → 1.72 cm（前 5 m 2.25 → 1.84），不偏时 1.51 → 1.56 cm；
    1 Hz 处增益裕度不变（1.57），原地转 + 倒车 + 甩头 8° 起步线头偏差 0.25 → 0.29 cm。用开头几米 RTK 航迹实时估测向偏差
    回放 13 趟没有改善（均方根 0.47 → 0.45°），没做。
    拐角提速（2026-10-01 用户：矩形每个拐角都要倒车太慢，不改硬件、把走法调快）：实车一个拐角约 13.5 s（停稳、原地转
    3 s、倒车 3.4 s、停稳、助跑 0.1 m/s 约 4 s）；原地转不提速（1.0 rad/s 时驱动器速度环冲过头，见 robot_gen5.json），
    改倒车段慢走 0.3 → 0.15 m、倒车 0.3 → 0.5 m/s（DIFF_REVERSE_MPS）、停稳 0.3 → 0.2 s（DIFF_SETTLE_S）、
    助跑慢走 0.1 → 0.15 m/s。
    入弧前馈整形 ff_shape_ratio 0.27 / ff_shape_gap_s 0.48 s（2026-10-04）：当天 R1.5 m 圆（后轴 1.89 m、0.61 m/s）
    入弧后 0.5–1 s 喷头偏进圆内 25–28 mm。不是指令给晚了（Smith 预估已提前 0.38 s，陀螺在后轴到切点前 0.06 s 到一半），
    是底盘角速度超调：要 −0.32 rad/s，陀螺冲到 −0.43 再掉到 −0.14，车头多转 2.4°。同一段 w_cmd → 陀螺辨识为
    纯延迟 0.05 s + 二阶 1.10 Hz、ζ 0.30、增益 1.05（R² 0.93；1 Hz 处 1.75∠−90°，与上面 1.7∠−100° 一致），
    按它取 ZV 两脉冲：K = e^(−ζπ/√(1−ζ²)) = 0.37，后一脉冲占 K/(1+K) = 0.27、晚半个阻尼周期 0.48 s。
    仿真（真 PathTracker + 两轮各一个该二阶速度环，驱动限加速度 1.0 / 角加速度 2.0）复现实车 26 mm；整形后
    R1.5 顺 / 逆、R3、R5、R1 顺时针角弧 6 种几何 × 6 种底盘（ζ 0.25–0.7、0.9–1.35 Hz、纯延迟 0.05–0.15 s、增益 1.00/1.05）
    最大值和 p95 都不变差、用时不变，辨识底盘上 R1.5 顺 25 → 18 mm、逆 31 → 23 mm、R3 21 → 15 mm；直线不受影响。
    后轴半径 < 0.83 m（|κ| > 1.2，喷头在右的 R1 逆时针角弧后轴 0.56 m）不整形：仿真整形后 28 → 30–56 mm。
    前馈平滑（相当于只在前馈里加过渡曲线）、整体晚给前馈都试过：平滑让车更早转、更差；晚给在紧弧上差到 59–85 mm。
    入弧降速也有效（进弧限 0.15 rad/s：R1.5 顺 25 → 17 mm）但车速降到 0.2–0.3 m/s，水泵常开时那一截漆会厚，紧弧上
    几乎停下，没用。"""
    # 2026-10-02 实车试验（用户要求带上 Codex 的长直线双环并打开）：仿真结论喷头横偏 −15%、车身小摆 +15%，需实车数据核对
    # 2026-10-02 17:09–17:18 实车 6 段巡航：车身小摆 0.63 → 0.35 °/s（减半），但喷头横偏 p95 0.47 → 约 1.0 cm，
    # 多出来的主要是整段平行偏在一侧 0.2–0.8 cm。原因是外环太软：测向每趟偏 −0.5～+1.2°（见上面 ki 一段），双环
    # 外环要靠横偏把它扳回来，比例 0.6 时稳态偏 δ/0.6（0.5° → 1.5 cm），积分 0.05 只有原控制（ki/kθ ≈ 0.56）的 1/11，
    # 还被 1 mm 的融合校正门槛冻住一半时间。按实车校准的仿真（轮子每圈起伏 + 测向偏差 + 四种底盘模型）选 比例 1.3、
    # 积分 0.4、外环低通 1.0 s：真喷头 p95 0.8 m/s 0.76–0.80 → 0.42–0.48 cm（原控制 0.44–0.58），车身小摆与现双环相同
    # （最差模型 +6%）；测向偏 2° 时 4.2 → 1.2 cm。外环低通再短、比例再大，小摆就回来了
    ab = CascadeConfig()
    return replace(config, delay_s=0.30, steer_gain=1.0, ki_per_m2=2.0, lat_accel_mps2=0.20, arc_speed_mps=PAINT_ARC_SPEED_MPS, start_creep_mps=0.1, start_hold_m=0.6, launch_settle_max_m=1.0,
                   reverse_creep_m=0.5, reverse_creep_mps=0.15, reverse_section_creep_m=0.15, stop_delay_s=None,
                   ff_shape_ratio=0.27, ff_shape_gap_s=0.48, ff_shape_kappa_max=1.2,
                   # 2026-10-06：已验证常速顺时针完整圆的入圆收益；低速与其他圆/圆弧沿用原前馈。
                   ff_entry_preview_s=0.0, ff_entry_clockwise_only=True, ff_entry_min_cruise_mps=.65,
                   ff_entry_full_circle_only=True, ff_entry_min_radius_m=1.5, ff_entry_max_radius_m=5.0,
                   straight_imu_enabled=True, straight_outer_k_e=ab.k_e, straight_outer_k_i=ab.k_i,
                   straight_min_length_m=ab.min_line_m, straight_settle_m=0.0, straight_min_speed_mps=0.0,
                   straight_kp=ab.hold.kp, straight_ki=ab.hold.ki, straight_kd=ab.hold.kd,
                   straight_drift_tc=ab.hold.drift_tc, straight_bias_max_deg=ab.max_bias_deg,
                   straight_offset_tau_s=ab.offset_tau_s,
                   # 实车霍尔瞬时速度会抖动，撤掉由它驱动的趋势项和增益调度。
                   straight_outer_rate_gain=0.0, straight_outer_i_boost=0.0,
                   straight_outer_tau_scale=1.0, straight_outer_trend_min_speed_mps=0.0,
                   # 复用 A/B 的原样本内层和目标限速，不叠加目标转速前馈。
                   straight_prediction_delay_s=HoldConfig().delay_s, straight_target_feedforward=False,
                   straight_target_rate_deg_s=ab.turn_rate_deg_s,
                   # 2026-10-02 用户：「帮我改一个最稳的」。长直线内环一直管转向到这段末尾（原来提前 1 m 多交回，原跟踪按融合航向开，
                   # 测向慢漂零点几度就把最后一米多带偏 1 cm 以上）。全链路仿真四种底盘 + 延迟 0.40 s 的底盘都不变差、最差一趟小一点；
                   # 车速、停车照旧由路径跟踪管，末尾减速到 straight_min_speed_mps 以下时照旧渐渐交回
                   straight_end_reserve_m=0.05,
                   # 2026-10-06 用户：直线 IMU 为主、卫星辅助，卫星和 IMU 推算对不上就听 IMU（line_cascade.ImuFirstConfig）；
                   # 当晚又定：直接靠 IMU 画线（rtk_trim=False），卫星好的时候只拟合角度和起点横偏、不逐拍拉（rtk_fit）
                   straight_imu_first=ImuFirstConfig(rtk_trim=False, rtk_fit=True), arc_imu_first=True, arc_partial_imu_first=True)


DIFF_REVERSE_MPS = 0.5      # 差速车倒车段限速（SectionTracker reverse_mps；阿克曼车 0.3）
# 普通路线倒车接入后的助跑（SectionTracker reverse_runup）：倒完车往前起步，万向轮要整个翻过来（175° → 5° 在头 0.25 m 里），
# 倒车停在助跑起点的横偏也比往前开过去大（4–5 vs 2–3 mm）。2026-10-07 用户：车在点后面直接倒一点，但「线头不能差」，
# 「2.5 m 没必要，1.5 能不能一样的效果」。仿真（Gen5 拟合底盘，10 种摆位 × 48 种子，起喷后 0.5 m 内最大偏差 中位/p90/最大）：
#   原来掉头接入 2.99/5.5/8.1 mm；照原样倒车 + 1.5 m 助跑 3.25/5.6/7.7（差）；起步慢走 0.1–0.15 m/s 最大 17–24（更差）；
#   助跑 0.4–0.5 m/s 3.4–3.5（差）；多倒 1 m 助跑 2.5 m 2.96/5.2/8.1；
#   助跑仍 1.5 m、起步锁车头 0.6 → 1.0 m（start_hold_m）2.25/4.4/7.9（起喷后 2 m 内中位 4.05 → 3.57），每种摆位中位、最大都不比原来大；
#   1.2 m 再好一点但只给 IMU 直行留 0.3 m 交接，没用。倒车前后差 1.5° 就原地转正几乎没用，没要。
REVERSE_HOLD_M = 1.0
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
    integral_frozen: bool = False


class PathTracker:
    def _straight_stitch(self,pieces,index):
        """两条近共线直线之间的毫米级拼接弧；几何保持原样，控制与速度不按一次真实转弯重启。"""
        if not 0<index<len(pieces)-1:return False
        p=pieces[index]
        spans=self.paint_spans if self.paint_spans is not None else [(self.paint_from_m,self.paint_to_m)]
        if p.s0>=self.path.length or any(max(a,p.s0)<min(b,p.s0+p.length)-1e-9 for a,b in spans):return False
        return (not p.straight and p.length<=.01 and abs(p.length*p.kappa)<=math.radians(.5)
                and pieces[index-1].straight and pieces[index+1].straight
                and min(pieces[index-1].length,pieces[index+1].length)>=.1
                and abs(wrap(pieces[index+1].yaw-pieces[index-1].yaw))<=math.radians(.5))

    def _straight_run_bounds(self,piece):
        pieces=self.path.pieces;lo=hi=pieces.index(piece)
        while lo>=2 and self._straight_stitch(pieces,lo-1):lo-=2
        while hi+2<len(pieces) and self._straight_stitch(pieces,hi+1):hi+=2
        return lo,hi

    def _same_straight_run(self,a,b):
        if a is b:return True
        if a is None or b is None:return False
        pieces=self.path.pieces
        if not a.straight and self._straight_stitch(pieces,pieces.index(a)):a=pieces[pieces.index(a)-1]
        if not b.straight and self._straight_stitch(pieces,pieces.index(b)):b=pieces[pieces.index(b)-1]
        if not a.straight or not b.straight:return False
        lo,hi=self._straight_run_bounds(a)
        return b in self.path.pieces[lo:hi+1]

    def __init__(self, path, config=PathConfig(), geometry=None):
        self.cfg = config
        self.geo = geometry or ChassisGeometry()
        self.kappa_max = 1.0 / self.geo.min_turn_radius_m
        usable = config.kappa_margin * self.kappa_max
        if path.kappa_abs_max > usable + 1e-9:
            raise ValueError('路径最小转弯半径 %.2f m，底盘能稳定跟踪的最小半径是 %.2f m'
                             % (1.0 / path.kappa_abs_max, 1.0 / usable))
        # 规划可把助跑、喷线、干尾拆成相邻共线段。控制上合并为一条直线，避免 0.83 m 助跑被
        # 当作不足 1 m 的独立短线而禁止 IMU 接管，或跨线头时重新初始化直线控制。喷涂边界仍按路程保留。
        pieces=[]
        for piece in path.pieces:
            if piece.straight and pieces and abs(pieces[-1][1])<1e-12:
                pieces[-1]=(pieces[-1][0]+piece.length,0.)
            else:pieces.append((piece.length,piece.kappa))
        self.path = Path(path.pose(0.),pieces,name=path.name) if len(pieces)<len(path.pieces) else path
        self.speed_limit_mps = None
        self._cruise_target = None
        self.continuation = None
        self.predecessor = None
        self._preview_path = None
        self._rolling_entry = False
        self._accel_prev = 0.0
        self._curve_speed_active = False
        self.entry_speed_mps = None
        self.paint_from_m = 0.0
        self.paint_to_m = path.length
        self.paint_spans = None
        self._arc_work_role = True  # 独立 PathTracker 沿用作业路径；分段转场显式关闭。
        self.run_out_m = 0.0
        self._run_out_started = False
        self.paint_offset_limit_m = None
        self.runup_speed_mps = None
        self.runup_start_mps = 0.0
        self._launch_pending = False
        self._launch_ramp = False
        self.approach_to_m = None       # 正式路线（喷线起点）在本段的路程处（可在段尾之后）；None 不管（见 approach_zone_m）
        self.approach_mps = None        # 离它 approach_zone_m 以内最多这么快（画线速度）
        self.runup_distance_m = 0.
        self.paint_start_guard = None
        self._paint_start_beta_history = deque(maxlen=2048)
        self.reverse_runup = False
        self.reverse_straight = False  # 纯直线倒车在行进方向坐标内使用共享 IMU 内环。
        self._runup_released = False    # 这次起步的助跑已收稳、交给 A/B（见 PathConfig.runup_release_e_m）
        self.constant_straight_speed = False
        self.constant_paint_speed = False
        self.paint_speed_ready = False  # 速度计划符合性诊断，不决定喷涂许可。
        self.paint_target_mps = None
        self.paint_expected_mps = None
        self.entry_stabilize_mps = None
        self._entry_speed_active = False
        self._entry_hold_from = 0.
        self._started_at = 0.0          # 这次起步 / 继续时在路线上的路程（画线时往回拉的线头段从这里也重新算）
        self._entry_ramp_released = False
        self._entry_speed_guard = PaintStartGuard()
        self._entry_reference_piece = None
        self._paint_speed_key = None
        self._paint_speed_latched = None
        self.integral_start_m = None    # 长直线助跑只纠偏，真正上线后才学习横偏积分；None 保留原行为
        self.integral_curve_from_m = None   # 喷线圆弧段：这之前的直线部分（进弧助跑）只纠偏、不学积分；None 保留原行为
        self.arc_tail_heading_from_m = None   # 仅部分喷线圆弧启用；整圆和转场不启用
        self._arc_tail_w = 0.0
        # 控制律按车实际走出的曲率算，最多能走出 steer_gain·kappa_max（下发的仍不超过 kappa_max）
        self.kappa_act_max = config.steer_gain * self.kappa_max
        self.k_theta = 2.0 * config.zeta * config.omega_per_m
        self.k_e = config.omega_per_m ** 2 / self.k_theta
        self.st = PathState(length=path.length)
        self._hint = None
        self._v_prev = 0.0
        self._creep_from = None
        self._settle = None             # 起步先稳再纠偏（launch_settle_*）：{'calm', 'done', 'blend'}，每次起步重来
        self._kfb = 0.0
        self._sum2 = self._n = self._max = 0.0
        self._clock = 0.0
        self._sent = deque()            # (下发时刻, 车速, 下发角速度)：还在底盘执行延迟里的指令
        self._imu_now = self._imu = self._imu_lever = None
        self._imu_history = deque(maxlen=64)
        self._straight_imu_rejected = False
        self._straight_nozzle = (config.straight_nozzle_x_m, 0.0)
        self._straight_core = HeadingControl(HeadingControlConfig(
            kp=config.straight_kp, ki=config.straight_ki, kd=config.straight_kd,
            delay_s=config.delay_s if config.straight_prediction_delay_s is None else config.straight_prediction_delay_s,
            drift_tc=config.straight_drift_tc,
            max_wz=0.35, i_max=0.15, direct_rate_damping=config.straight_direct_rate_damping))
        first = config.straight_imu_first
        self._straight_guide = None if first is None else ImuFirstGuide(first)
        self._arc_guide = None if first is None else ImuFirstGuide(first)     # 画圆 IMU 为主（PathConfig.arc_imu_first）
        self._baseline_bad_t = None     # 最近一次双天线基线不在 straight_imu_first 基线范围内的测量时刻
        self._baseline_frame_t = None   # 最近数过的测向帧时刻（任务层每拍都传最近一帧，同一帧只数一次）
        self._baseline_bad_n = 0        # 连续出界的帧数
        self._fit_bad_n = 0             # 同上，按拟合用的宽范围（fit_min_baseline_m–fit_max_baseline_m）
        self._fit_bad_t = None
        self._quality_frame_t = None    # 最近数过的定位帧时刻
        self._quality_bad_n = 0         # 定位质量连续不合格的帧数
        self._quality_bad_t = None      # 最近一次（连续 fit_quality_frames 帧）不合格的时刻
        self._pair_bad_n = 0            # 两天线距离误差连续超限的帧数（单独计，恢复等 fit_pair_settle_s）
        self._pair_bad_t = None
        self._receiver_bad_n = 0        # 只看接收机（σ、卫星数、固定解，不看两天线）连续不合格的帧数：选天线时用
        self._receiver_bad_t = None
        self._secondary = None          # 副天线最近一帧 (测量时刻, x, y, 固定解, 车体系杠杆臂 (前, 左))
        self._primary = None            # 主天线同样保存原始坐标，可靠整圆观测不再叠加融合位置的滞后。
        self._quality = None            # 最近一帧 (σ m, 卫星数, 两天线距离误差 m 或 None, 固定解 True/False 或 None)
        self._imu_alignment = None      # 任务层的 line_cascade.ImuAlignment：上一条线对好的角度
        self._reset_straight()
        self._reset_arc()

    def _reset_straight(self):
        self._straight_weight = 0.0
        self._launch_weight = 0.0
        self._launch_armed = False
        # 两根天线对不上时选哪根（ImuFirstConfig.antenna_*）：offset 副 − 主横偏的固定差（对得上时学），n 学了几帧，
        # delta 最近一帧副天线比主天线（扣掉固定差）多偏多少，t 用过的副天线帧，choice 这拍用的 main / second / None
        # shift：这拍用副天线时，报给 app 的位置要挪多少（世界系 x, y，m），shift_t 是哪一拍算的
        self._antenna = dict(offset=None, n=0, delta=None, t=None, choice=None, shift=None, shift_t=None)
        if self._straight_guide is not None:
            self._straight_guide.reset()
        self._straight_aligned = False
        self._straight_guided = None    # IMU 为主时喷头横偏估计、IMU 车头（含 β）相对线方向：偏离中止、降速、喷线资格用
        self._straight_piece = None
        self._straight_offset = self._straight_target = None
        self._straight_offset_n = 0
        self._straight_bias = self._straight_integral = 0.0
        self._straight_target_rate = 0.0
        self._straight_last_s = None
        self._straight_settled = False
        self._straight_last_w = 0.0
        self._straight_last_e = None
        self._straight_rate_pos = self._straight_rate_model_slow = 0.0
        self._straight_status = {'active': False, 'weight': 0.0, 'eligible': False}
        self._straight_core.reset()

    def _reset_arc(self):
        self._arc_weight = 0.0
        self._arc_piece = None
        self._arc_s = None              # 沿圆路程（按走过的路推、慢慢靠向卫星），m
        self._arc_scale = 1.0           # 车速读数 → 沿圆路程的比例（慢慢学）
        self._arc_off = None            # 融合航向 − IMU 偏航（锁住后按 arc_heading_tau_s 慢慢跟），rad
        self._arc_e = None              # 喷头横偏估计（IMU 推算 + 往卫星拉），m
        self._arc_jump_e = None         # 跳点判定保留原慢修估计，不能被快速校正提前吸收掉。
        self._arc_inertial_sum = 0.0
        self._arc_raw_last = None
        self._arc_raw_hold = False
        self._arc_raw_expected = None
        self._arc_rel = None
        self._arc_travel = 0.0
        self._arc_hist = deque()        # 最近 jump_window_m 内的 (路程, 卫星 − 推算)
        self._arc_hold = None           # 卫星突然跳开、先不拉：[已等的秒数, 跳开时的差]
        self._arc_hold_end = None       # 上次跳点放开时的沿圆路程（arc_good_after_jump_m 内不算卫星好），m
        self._arc_good = False          # 这一拍整圆卫星好、慢修按 arc_good_pull_*
        self._arc_jumps = 0
        self._arc_rtk_since = self._arc_rtk_frame = None
        self._arc_rtk_frames = 0
        self._arc_rtk_weight = 0.0
        self._arc_guided = None         # IMU 为主时喷头横偏估计、车头相对圆切线：偏离中止用
        self._arc_status = {'active': False, 'weight': 0.0}
        if self._arc_guide is not None:
            self._arc_guide.reset()

    def sync(self, now, imu, lever=None):
        """任务层同步同源单调时间和原始IMU(t,yaw,gz)，后轴主天线杠杆臂供坐标换算。"""
        self._imu_now, self._imu_lever = now, lever
        if imu is not None:
            try:
                t, yaw, rate = imu
                valid = (all(math.isfinite(v) for v in (now, t, yaw, rate)) and 0 <= now-t <= 0.2
                         and abs(rate) <= math.radians(70.0))
                if valid and self._imu is not None and imu != self._imu:
                    elapsed = t-self._imu[0]
                    valid = elapsed > 0 and abs(wrap(yaw-self._imu[1])) <= math.radians(70.0)*elapsed + math.radians(.02)
                self._straight_imu_rejected = not valid
                if valid:
                    self._imu = imu
                    if not self._imu_history or t > self._imu_history[-1][0]:
                        self._imu_history.append((t,yaw,rate))
            except (TypeError, ValueError):
                self._straight_imu_rejected = True

    def set_heading_baseline(self, t, length_m):
        """最近一帧双天线测向的测量时刻（与 sync 同一单调时钟）和基线长（m）；直线 IMU 为主时基线连续
        baseline_bad_frames 帧不在 [min_baseline_m, max_baseline_m] 就不用卫星修正（同一帧重复传只数一次）。"""
        first = self.cfg.straight_imu_first
        try:
            if first is None or not (math.isfinite(t) and math.isfinite(length_m)):
                return
            if self._baseline_frame_t is not None and t <= self._baseline_frame_t:
                return
            self._baseline_frame_t = t
            if first.fit_min_baseline_m <= length_m <= first.fit_max_baseline_m:
                self._fit_bad_n = 0
            else:
                self._fit_bad_n += 1
                if self._fit_bad_n >= first.baseline_bad_frames:
                    self._fit_bad_t = t if self._fit_bad_t is None else max(t, self._fit_bad_t)
            if first.min_baseline_m <= length_m <= first.max_baseline_m:
                self._baseline_bad_n = 0
                return
            self._baseline_bad_n += 1
            if self._baseline_bad_n >= first.baseline_bad_frames:
                self._baseline_bad_t = t if self._baseline_bad_t is None else max(t, self._baseline_bad_t)
        except TypeError:
            pass

    def share_imu_alignment(self, alignment):
        """任务层给的 ImuAlignment（跨线、跨作业共用）。"""
        self._imu_alignment = alignment

    def set_rtk_quality(self, t, sigma_m, sats, pair_m=None, fixed=None):
        """最近一帧主天线定位的测量时刻和质量：接收机 σ（hypot(σn, σe)，m）、卫星数、两天线位置距离误差（m，没对齐为空）、
        是不是固定解（不知道为空）。直线卫星纠偏只在这几项都合格时用（见 _quality_good）；同一帧重复传只数一次。"""
        first = self.cfg.straight_imu_first
        try:
            if first is None or not math.isfinite(t) or self._quality_frame_t is not None and t <= self._quality_frame_t:
                return
            self._quality_frame_t = t
            self._quality = (sigma_m, sats, pair_m, fixed)
            receiver_bad = not self._quality_good(sigma_m, sats, pair_m, fixed, pair_required=False)
            self._receiver_bad_n = self._receiver_bad_n + 1 if receiver_bad else 0
            if self._receiver_bad_n >= first.fit_quality_frames:
                self._receiver_bad_t = t if self._receiver_bad_t is None else max(t, self._receiver_bad_t)
            bad = not self._quality_good(sigma_m, sats, pair_m, fixed)
            self._quality_bad_n = self._quality_bad_n + 1 if bad else 0
            if self._quality_bad_n >= first.fit_quality_frames:
                self._quality_bad_t = t if self._quality_bad_t is None else max(t, self._quality_bad_t)
            pair_bad = pair_m is not None and math.isfinite(pair_m) and pair_m > first.fit_max_pair_m
            self._pair_bad_n = self._pair_bad_n + 1 if pair_bad else 0
            if self._pair_bad_n >= first.fit_quality_frames:
                self._pair_bad_t = t if self._pair_bad_t is None else max(t, self._pair_bad_t)
        except TypeError:
            pass

    def _quality_good(self, sigma, sats, pair, fixed, pair_required=True):
        """一帧定位的 σ、卫星数够不够格（ImuFirstConfig.fit_*）。卫星数到 fit_full_sats 只看 σ ≤ fit_max_sigma_m；
        fit_min_sats 到 fit_full_sats 之间（17–19 颗，10-08 用户）要 σ ≤ fit_low_sats_max_sigma_m、固定解，还要两天线距离
        误差 < fit_low_sats_max_pair_m（pair_required=False 时不看这条：两根对不上时另由 _pick_antenna 选）；低于
        fit_min_sats 不信。"""
        first = self.cfg.straight_imu_first
        if not math.isfinite(sigma) or sats is None or sats < first.fit_min_sats:
            return False
        if sats >= first.fit_full_sats:
            return sigma <= first.fit_max_sigma_m
        return (sigma <= first.fit_low_sats_max_sigma_m and fixed is True
                and (not pair_required or pair is not None and math.isfinite(pair) and pair < first.fit_low_sats_max_pair_m))

    def _receiver_quality_ok(self):
        """只看接收机（σ、卫星数、固定解）的 _rtk_quality_ok：不合格之后连续合格 fit_settle_s 才算好。"""
        first, bad = self.cfg.straight_imu_first, self._receiver_bad_t
        return self._imu_now is None or bad is None or self._imu_now - bad > first.fit_settle_s

    def set_secondary_fix(self, t, x, y, fixed, lever):
        """副天线最近一帧：测量时刻、本地坐标（m）、是不是固定解、车体系杠杆臂 (前, 左)（主天线杠杆臂 + 测向基线）。"""
        try:
            if all(math.isfinite(v) for v in (t, x, y, *lever)):
                self._secondary = (float(t), float(x), float(y), fixed is True, (float(lever[0]), float(lever[1])))
        except (TypeError, ValueError):
            pass

    def set_primary_fix(self, t, x, y, fixed, lever):
        """主天线原始固定解与同一车体系杆臂；用于作业圆弧双天线径向观测。"""
        try:
            if (all(math.isfinite(v) for v in (t,x,y,*lever))
                    and (self._primary is None or t >= self._primary[0])):
                self._primary=(float(t),float(x),float(y),fixed is True,(float(lever[0]),float(lever[1])))
        except (TypeError,ValueError):
            pass

    def _imu_yaw_at(self, t):
        """按定位测量时刻取 IMU 航向，避免用当前航向扣过去的天线杆臂。"""
        rows=self._imu_history
        if not rows or t < rows[0][0]-1e-9:
            return None
        for a,b in zip(rows,list(rows)[1:]):
            if a[0] <= t <= b[0]:
                return wrap(a[1]+wrap(b[1]-a[1])*(t-a[0])/(b[0]-a[0]))
        last=rows[-1]
        return wrap(last[1]+last[2]*(t-last[0])) if 0 <= t-last[0] <= .05 else None

    def _arc_raw_radial(self, piece):
        """两天线分别换算到喷头；测量时刻对齐后，径向观测一致才取均值。"""
        if self._primary is None or self._secondary is None or self._arc_off is None:
            return None
        k=piece.kappa
        if abs(k)<1e-9:return None
        cx,cy=piece.x-math.sin(piece.yaw)/k,piece.y+math.cos(piece.yaw)/k
        nx,ny=self._straight_nozzle
        radius=math.hypot(nx,1./k-ny)
        values=[]
        for fix in (self._primary,self._secondary):
            t,x,y,fixed,(ax,ay)=fix
            if not fixed or self._imu_now is None or not 0 <= self._imu_now-t <= self.cfg.arc_rtk_fresh_s:
                return None
            h=self._imu_yaw_at(t)
            if h is None:return None
            h=wrap(h+self._arc_off)
            qx=x+(nx-ax)*math.cos(h)-(ny-ay)*math.sin(h)
            qy=y+(nx-ax)*math.sin(h)+(ny-ay)*math.cos(h)
            values.append(-math.copysign(1.,k)*(math.hypot(qx-cx,qy-cy)-radius))
        return .5*sum(values) if abs(values[0]-values[1]) <= self.cfg.arc_rtk_pair_max_m else None

    def _pick_antenna(self, guide, heading, hint, base_lat, measured):
        """两根天线对不上时用哪根（ImuFirstConfig.antenna_*，10-08 用户「用和 IMU 对得上的那根」）。
        base_lat：主天线（融合位置）推出来的控制点横偏；measured：主天线算的喷头横偏。返回 (main / second, 加到 measured 上的量)
        或 None（照旧）。"""
        first, a, sec = self.cfg.straight_imu_first, self._antenna, self._secondary
        fresh = (sec is not None and sec[3] and self._imu_now is not None
                 and 0.0 <= self._imu_now - sec[0] <= first.antenna_fresh_s)
        if fresh and sec[0] != a['t']:
            t2, x2, y2, _, (lx, ly) = sec
            c, s = math.cos(heading), math.sin(heading)
            _, lat2, _, _ = self.locate(x2 - (c*lx - s*ly), y2 - (s*lx + c*ly), heading, hint)
            d = lat2 - base_lat
            a['t'] = t2
            pair = self._quality[2] if self._quality is not None else None
            if pair is not None and math.isfinite(pair) and pair < first.antenna_pick_pair_m and a['choice'] is None:
                a['offset'] = d if a['offset'] is None else a['offset'] + 0.1*(d - a['offset'])
                a['n'] += 1
            a['delta'] = None if a['offset'] is None or a['n'] < first.antenna_offset_samples else d - a['offset']
        pair = self._quality[2] if self._quality is not None else None
        if pair is None or not math.isfinite(pair):
            disagree = a['choice'] is not None                          # 这帧没比：沿用上一拍
        else:
            disagree = pair >= first.antenna_pick_pair_m
        choice = None
        if disagree and fresh and a['delta'] is not None and guide.e is not None:
            r_main, r_sec = abs(measured - guide.e), abs(measured + a['delta'] - guide.e)
            agree, apart = first.antenna_agree_m, first.antenna_agree_m + first.antenna_margin_m
            if r_sec <= agree and r_main > apart:
                choice = 'second'
            elif r_main <= agree and r_sec > apart:
                choice = 'main'
        if (choice == 'second') != (a['choice'] == 'second'):
            guide.source_changed()
        a['choice'] = choice
        return None if choice is None else (choice, a['delta'] if choice == 'second' else 0.0)

    def position_correction(self):
        """这一拍直线修偏在用副天线时，融合位置该挪多少才和副天线一致（世界系 dx, dy，m）；没在用副天线为 None。
        只给报给 app 的位置用（地图喷线、作业报告），控制照常用融合位置 + 选天线。"""
        a = self._antenna
        return a['shift'] if a['shift'] is not None and a['shift_t'] == self._imu_now else None

    def _antenna_status(self):
        a = self._antenna
        return dict(rtk_antenna=a['choice'], ant2_offset=a['offset'], ant2_delta=a['delta'], ant2_n=a['n'])

    def _rtk_quality_ok(self):
        """卫星定位质量合格（σ、卫星数不合格之后连续合格 fit_settle_s；两天线距离误差超限之后 fit_pair_settle_s）。
        没收到过质量信息当合格（老任务层、仿真）。"""
        first, bad, pair = self.cfg.straight_imu_first, self._quality_bad_t, self._pair_bad_t
        if self._imu_now is None:
            return True
        return ((bad is None or self._imu_now - bad > first.fit_settle_s)
                and (pair is None or self._imu_now - pair > first.fit_pair_settle_s))

    def _baseline_ok(self, fit=False):
        """双天线基线合格（0.40–0.42，学开头换算用）；fit=True 按拟合用的宽范围（0.30–0.50，卫星纠偏用）。"""
        first = self.cfg.straight_imu_first
        bad = self._fit_bad_t if fit else self._baseline_bad_t
        return bad is None or self._imu_now is None or self._imu_now - bad > first.baseline_hold_s

    def set_nozzle(self, forward, left=0.0):
        """实际喷头相对后轴中心；整场使用其规划时同一份偏移。"""
        if not all(math.isfinite(v) for v in (forward, left)):
            raise ValueError('喷头偏移必须是有限数值')
        self._straight_nozzle = (float(forward), float(left))

    def _fresh_straight_imu(self):
        try:
            if self._straight_imu_rejected:
                return None
            t, yaw, rate = self._imu
            if not all(math.isfinite(v) for v in (self._imu_now, t, yaw, rate)):
                return None
            age = self._imu_now - t
            if not 0.0 <= age <= 0.2:
                return None
            return t, wrap(yaw + rate * age), rate
        except (TypeError, ValueError):
            return None

    def locate(self, x, y, yaw, hint=None, *, preview=False):
        """车体控制点相对路径的 (路程, 横偏, 车头误差, 路径曲率)。"""
        px = x + self.cfg.track_x_m * math.cos(yaw)
        py = y + self.cfg.track_x_m * math.sin(yaw)
        s, e, path_yaw, k = (self._preview_path if preview and self._preview_path is not None else self.path).project(px, py, hint, extend_start=self.cfg.start_before_m)
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

    def start(self, pose, integral=None, *, preserve_startup=False):
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
        if not resume:self._run_out_started=False
        if integral is not None:
            self.st.integral = integral
        self._hint = self.locate(pose.x, pose.y, pose.yaw, hint)[0]
        self._started_at = self._hint
        self._kfb = 0.0
        self._arc_tail_w = 0.0
        if not (preserve_startup and resume):
            # 原地短暂停后自动恢复保持已完成的起步段；操作员重新接入/新任务仍重新稳定起步。
            self._creep_from = None
            self._settle = None
        self._runup_released = False
        self._v_prev = max(0.0, pose.speed)
        self._entry_speed_active = (self.entry_stabilize_mps is not None
            and (self._hint < self.paint_from_m or self._v_prev < min(self.entry_stabilize_mps,self.cfg.cruise_mps)-1e-6))
        self._launch_pending = self.runup_start_mps > 0. and (self._hint < self.paint_from_m or self._entry_speed_active)
        self._launch_ramp = False
        self._entry_hold_from=max(self.paint_from_m,self._hint)
        self._entry_ramp_released=False
        self._entry_speed_guard=PaintStartGuard()
        self._entry_reference_piece=None
        if self.paint_start_guard is not None and not (resume and self.paint_start_guard.started):
            self.paint_start_guard=PaintStartGuard()
        self._paint_start_beta_history.clear()
        self._accel_prev = 0.0
        self._rolling_entry = False
        self._curve_speed_active = False
        self._sum2 = self._n = self._max = 0.0
        self._clock = 0.0
        self._sent.clear()
        self._reset_straight()
        # 静止后的前进喷线/纯直线倒车起步先保持相对 IMU 方向；绝对方向仍按质量规则学习。
        self._launch_armed = (self.geo.differential and self.cfg.straight_imu_enabled
                              and self.cfg.straight_imu_first is not None
                              and (self.reverse_straight or
                                   self.entry_stabilize_mps is not None and self.runup_start_mps > 0.)
                              and abs(pose.speed) < .05
                              and self.path._piece(max(0.,min(self.path.length,self._hint))).straight)
        return True

    def pause(self, reason='operator'):
        self.st.state, self.st.reason = 'PAUSED', reason
        self.st.v_cmd = self._v_prev = 0.0
        self._accel_prev = 0.0
        self._sent.clear()
        self._arc_tail_w = 0.0
        self._reset_straight()
        self._reset_arc()
        return 0.0, 0.0

    def stop(self, reason='operator'):
        if self.st.state in ('RUNNING', 'PAUSED'):
            self.st.state, self.st.reason = 'ABORTED', reason
        self.st.v_cmd = 0.0
        self._arc_tail_w = 0.0
        self._reset_straight()
        self._reset_arc()
        return 0.0, 0.0

    def _paint_windows(self):
        if not self.constant_paint_speed:return []
        return self.paint_spans if self.paint_spans is not None else [(self.paint_from_m,self.paint_to_m)]

    def _active_paint_window(self, s, pose=None):
        for i,(a,b) in enumerate(self._paint_windows()):
            last=self.path._piece(max(a,b-1e-8))
            if not a<=s<b+(.05 if last.straight else 0.):continue
            if last.straight and pose is not None:
                remaining,_=self.nozzle_end(pose,*self._straight_nozzle,at=b)
                if remaining<=-.005:continue
            return i,a,b
        return None

    def _planned_paint_speed(self, a, b):
        cruise=self.cfg.cruise_mps if self._cruise_target is None else self._cruise_target
        curved=any(not p.straight and p.s0<b-1e-8 and p.s0+p.length>a+1e-8 for p in self.path.pieces)
        # 同一笔若有直弯混合，进入该笔前已选好能够全程保持的速度，不到弯前临时压低。
        return self.cfg.arc_speed_mps if curved and self.cfg.arc_speed_mps is not None else cruise

    def _entry_paint_target(self, s, cruise):
        """直线起步先以 0.5 走过正式线头后 1 m，稳定后只释放一次平滑提速。"""
        if not self._entry_speed_active:return cruise
        ready=(self._entry_speed_guard.ready if self.cfg.straight_imu_enabled else
               abs(self.st.e)<=PaintStartGuard.OFFSET_M and abs(self.st.heading_err)<=PaintStartGuard.HEADING_RAD)
        if s>=self._entry_hold_from+1. and ready:self._entry_ramp_released=True
        return cruise if self._entry_ramp_released else min(cruise,self.entry_stabilize_mps)

    def _short_runup_here(self, s):
        """短直线助跑留足采样、渐入和稳定时间，不缩短起喷检查或扩大场地。"""
        if self.paint_start_guard is None or self.entry_stabilize_mps is None or s>=self.paint_from_m:return False
        first=self.cfg.straight_imu_first
        if first is None:return False
        target=self.path._piece(self.paint_from_m)
        if not target.straight:return False
        begin,_=self._straight_run_bounds(target)
        run=self.paint_from_m-self.path.pieces[begin].s0
        required=self.entry_stabilize_mps*(first.lock_samples*CONTROL_DT+self.cfg.straight_blend_s
                                         +PaintStartGuard.SETTLE_S+self.cfg.delay_s+.2)
        return 0.<run<required and self._same_straight_run(self.path._piece(max(0.,s)),target)

    def _prepare_entry_window(self, s, window):
        """一条连续路线可以含多条喷线直边；每次绕弯接入新直边都先稳行，同一直线上的虚线空隙不重置。"""
        if self.entry_stabilize_mps is None:return
        upcoming=window or next(((i,a,b) for i,(a,b) in enumerate(self._paint_windows()) if a>s),None)
        if upcoming is None:return
        _,a,b=upcoming
        piece=self.path._piece(a+1e-8)
        if not piece.straight or self._same_straight_run(piece,self._entry_reference_piece):return
        first=self._entry_reference_piece is None
        self._entry_reference_piece=piece
        if first:
            self._entry_hold_from=max(a,self._entry_hold_from)
            return
        self._entry_speed_active=True
        self._entry_hold_from=a
        self._entry_ramp_released=False
        self._entry_speed_guard=PaintStartGuard()

    def set_cruise(self, speed):
        if not math.isfinite(speed) or not .02<=speed<=MAX_SECTION_SPEED:
            raise ValueError('巡航速度无效')
        if self.st.state=='RUNNING':
            self._cruise_target=speed
        else:
            self.cfg=replace(self.cfg,cruise_mps=speed)
            self._cruise_target=None

    def _advance_cruise(self,dt):
        if self._cruise_target is None or self.st.state!='RUNNING':return
        if self._active_paint_window(self.st.s) is not None:return
        delta=self._cruise_target-self.cfg.cruise_mps
        rate=self.cfg.accel_mps2 if delta>0 else self.cfg.decel_mps2
        value=self.cfg.cruise_mps+max(-rate*max(0.,dt),min(rate*max(0.,dt),delta))
        self.cfg=replace(self.cfg,cruise_mps=value)
        if abs(value-self._cruise_target)<1e-9:self._cruise_target=None

    def update(self, pose, dt):
        """返回 (vx, wz)。pose 需有 x, y, yaw, speed, yaw_rate, usable_for_control。"""
        self._clock += dt
        self._advance_cruise(dt)
        steer = None
        if self.st.state == 'RUNNING' and pose.usable_for_control:
            steer = self._arc_first(pose, dt)
        elif self._arc_piece is not None:
            self._reset_arc()
        self._settle_step(pose, dt)
        v, kappa, w_hold = self._control(pose, dt, steer)
        wz = v * kappa + w_hold
        if self.st.state == 'RUNNING' and not (self.speed_limit_mps is not None and self.speed_limit_mps <= 0.0):
            wz = self._straight_control(pose, dt, v, wz)
        else:
            armed = self._launch_armed and self._launch_pending and self.st.state == 'RUNNING'
            self._reset_straight()
            self._launch_armed = armed
            if self.speed_limit_mps is not None and self.speed_limit_mps <= 0.0:
                v = self.st.v_cmd = 0.0
                wz = 0.0
                self._arc_tail_w = 0.0
        if (self.geo.differential and self.st.state == 'RUNNING'
                and (self.reverse_straight or self.runup_speed_mps is not None and self.st.s < self.paint_from_m)
                and self.path._piece(max(0.,self.st.s)).straight):
            # 直线倒车/圆前切线的低速纠正不能用额外锁向量把一轮反向；入弧后交回原转向包络。
            # 2026-10-08 实录：v 仅 0.0006 m/s 时 wz=-0.0805，左右目标 27/-26 erpm。
            bound=min(self.kappa_max*v, 2.*v/self.geo.track_m)
            wz=max(-bound,min(bound,wz))
            if 'final_wz' in self._straight_status:self._straight_status['final_wz']=wz
        if self.st.state == 'RUNNING':
            v, wz = self._launch_settle(dt, v, wz)
        if self.st.state == 'RUNNING':
            self._check_paint_start(pose,dt,v)
        self._sent.append((self._clock, v, wz))
        if (self._straight_weight > 0 or self._launch_weight > 0) and self._imu_now is not None:
            self._straight_core.record_sent(self._imu_now, wz)
            # 内环按原始 IMU 样本时刻预估，保留新鲜度窗口内可能用到的历史。
            self._straight_core.sent_since(self._imu_now-max(self.cfg.delay_s, self._straight_core.cfg.delay_s + .2), self._imu_now)
        if (self._straight_weight > 0 or self._launch_weight > 0 or abs(self._arc_tail_w) > 1e-9) and v > 1e-6:
            self.st.kappa = wz / v
        return v, wz

    def _settling(self):
        """这一拍还在起步稳定段（launch_settle_*，稳住之前）：原控制不拉横偏、不积分。"""
        return self._settle is not None and not self._settle['done']

    def _settle_step(self, pose, dt):
        """每拍控制前更新起步稳定状态（用上一拍的路程）。只管真正从停着起步、还没到喷线区；连续走过段与段的交接
        （场地连续弧、圆弧掉头）不算起步。"""
        cfg, st = self.cfg, self.st
        if cfg.launch_settle_max_m <= 0 or not self.geo.differential or st.state != 'RUNNING':
            return
        if self._settle is None:
            moving = abs(pose.speed) >= 0.05 or self._v_prev >= 0.05
            self._settle = dict(calm=0.0, done=moving, blend=cfg.launch_settle_blend_s if moving else 0.0, started_m=0.0)
        S = self._settle
        if S['done'] or self._creep_from is None:
            return
        started = S['started_m'] = st.s - self._creep_from
        # 两轮同步了 = 车身转速跟上正在执行的下发转速（不是转速本身小）：限幅顶满、车稳稳地按 2°/s 修车头也算稳住。
        # 原来按「转速 < launch_settle_rate_deg_s」判，和限幅一样大，起步车头差 4–5°（< 原地转门槛 5°）时一直顶满、永远
        # 不算稳，稳定段拖满 1 m 不纠横偏（2026-10-11 仿真 R3 圆：喷头被带开 7 cm，开喷后最大 3 cm）
        calm = (abs(pose.speed) > 0.02
                and abs(getattr(pose, 'yaw_rate', math.inf) - self.cfg.steer_gain * self._executing_wz())
                < math.radians(cfg.launch_settle_rate_deg_s))
        S['calm'] = S['calm'] + dt if calm else 0.0
        if (started >= cfg.launch_settle_max_m or self._active_paint_window(st.s) is not None
                or started >= cfg.launch_settle_min_m and S['calm'] >= cfg.launch_settle_calm_s - 1e-9):
            S['done'] = True

    def _executing_wz(self):
        """此刻车上正在执行的下发角速度（delay_s 之前下发、还在生效的那条）；还没有就当 0（刚从停着起步）。"""
        due, w = self._clock - self.cfg.delay_s, 0.0
        for t, _, wz in self._sent:
            if t > due + 1e-9:
                break
            w = wz
        return w

    def _launch_settle(self, dt, v, wz):
        """起步稳定段的转向限制，返回实际下发的 (v, wz)（车速目标在 _control 里已限到 launch_settle_mps）：路线曲率
        前馈之外的转向（这时原控制已不拉横偏，只剩保持车头）限在 ±launch_settle_max_deg_s，起步极慢时也不让一边轮子反转；
        稳住后 launch_settle_blend_s 内把限幅渐渐放开。"""
        cfg, st, S = self.cfg, self.st, self._settle
        if S is None or S['done'] and S['blend'] >= cfg.launch_settle_blend_s - 1e-9:
            return v, wz
        ff = v * st.kappa_ref
        lim = math.radians(cfg.launch_settle_max_deg_s)
        bound = 2. * abs(v) / self.geo.track_m
        limited = max(-bound, min(bound, ff + max(-lim, min(lim, wz - ff))))
        if not S['done']:
            return v, limited
        S['blend'] += dt
        a = min(1.0, S['blend'] / cfg.launch_settle_blend_s)
        return v, limited + a * (wz - limited)

    def _check_paint_start(self,pose,dt,speed):
        """起步检查只控制喷涂许可，不触发车辆停车；未收稳沿原路径继续纠偏。"""
        guard=self.paint_start_guard
        if guard is None:return False
        if guard.started and not (self._entry_speed_active and not self._entry_ramp_released):
            self._straight_status.update(guard.status());return False
        alignment_mark=self._entry_hold_from if self._entry_speed_active else self.paint_from_m
        _,e,heading,_=self.locate(pose.x,pose.y,pose.yaw,alignment_mark)
        nx,ny=self._straight_nozzle
        e+=(nx-self.cfg.track_x_m)*math.sin(heading)+ny*(math.cos(heading)-1.)
        guided=self._straight_guided
        trusted=(self._runup_position_trusted(pose) and self._straight_guide is not None
                 and self._straight_guide.rtk_accepted and self._fresh_straight_imu() is not None)
        beta=None if self._straight_guide is None else self._straight_guide.beta
        # 记录最近 0.75 m 的校准变化供复核。参数可随可靠观测继续更新；放行检查其位置/方向结果是否稳定一致。
        history=self._paint_start_beta_history
        travel=0. if self._straight_guide is None else self._straight_guide.travel
        if beta is not None:history.append((travel,beta))
        while len(history)>1 and travel-history[1][0]>=PaintStartGuard.CALIBRATION_WINDOW_M:history.popleft()
        span=0. if not history else max(b for _,b in history)-min(b for _,b in history)
        observation=dict(trusted=trusted,frame=self._quality_frame_t,weight=self._straight_weight,
            rtk_error=e,guide_error=None if guided is None else guided[0],
            heading_error=None if guided is None else guided[1])
        if self._entry_speed_active and not self._entry_ramp_released:
            self._entry_speed_guard.update(dt,**observation)
        if guard.started:
            self._straight_status.update(guard.status());return False
        guard.update(dt,**observation)
        self._straight_status.update(guard.status())
        self._straight_status['start_calibration_span_deg']=math.degrees(span)
        left=self.paint_from_m-self.st.s
        self._straight_status['start_left_m']=left
        if left<=0. and guard.ready:
            guard.started=True;self._straight_status.update(guard.status())
        return False

    def paint_start_allowed(self):
        return self.paint_start_guard is None or self.paint_start_guard.started

    def _straight_control(self, pose, dt, speed, legacy_w):
        """直线在行进方向坐标内复用稳向与横偏控制；路线仍管理车速、停车及弯道交接。"""
        cfg, st = self.cfg, self.st
        imu = self._fresh_straight_imu()
        if not cfg.straight_imu_enabled or not self.geo.differential or imu is None or 'imu_mismatch' in getattr(pose, 'reasons', ()):
            self._reset_straight()
            return legacy_w
        _, imu_yaw, gyro = imu
        piece = next((p for p in self.path.pieces if p.s0 <= st.s < p.s0 + p.length), None)
        if piece is None and st.s < 0.0:
            piece = self.path.pieces[0]
        if piece is not None and not piece.straight:
            index=self.path.pieces.index(piece)
            if self._straight_stitch(self.path.pieces,index):piece=self.path.pieces[index-1]
        if piece is None or not piece.straight:
            self._reset_straight()
            return legacy_w
        if piece is not self._straight_piece:
            if not self._same_straight_run(piece,self._straight_piece):
                armed = self._launch_armed and self._straight_piece is None
                self._reset_straight()
                self._launch_armed = armed
            self._straight_piece = piece
        if self._straight_guide is not None:
            return self._straight_first(pose, dt, speed, legacy_w, piece, imu_yaw, gyro)
        # 参数源于 A/B；不引入路线外环的低通、速度趋势或目标转速前馈。
        ab = replace(CascadeConfig(), k_e=cfg.straight_outer_k_e, k_i=cfg.straight_outer_k_i,
                     max_bias_deg=cfg.straight_bias_max_deg, offset_tau_s=cfg.straight_offset_tau_s,
                     turn_rate_deg_s=cfg.straight_target_rate_deg_s)
        absolute_offset = wrap(pose.yaw - imu_yaw)
        if self._straight_offset is None:
            self._straight_offset, self._straight_offset_n = absolute_offset, 1
        difference = wrap(absolute_offset - self._straight_offset)
        self._straight_offset, self._straight_offset_n, _ = learn_heading_offset(
            self._straight_offset, self._straight_offset_n, difference, dt, ab.offset_tau_s)

        first_index,last_index=self._straight_run_bounds(piece)
        first_piece,last_piece=self.path.pieces[first_index],self.path.pieces[last_index]
        u=st.s-first_piece.s0
        length=last_piece.s0+last_piece.length-first_piece.s0
        left=last_piece.s0+last_piece.length-st.s
        reserve = max(1.0, speed*cfg.straight_blend_s + speed*cfg.delay_s + speed*speed/(2*cfg.decel_mps2))
        if cfg.straight_end_reserve_m is not None and last_piece is self.path.pieces[-1]:
            reserve = cfg.straight_end_reserve_m
        eligible = (length >= cfg.straight_min_length_m
                    and (u >= cfg.straight_settle_m or first_piece is self.path.pieces[0] and u < 0.)
                    and speed > cfg.straight_min_speed_mps and left > reserve)
        runup_wait = False
        if self.runup_speed_mps is not None and st.s < self.paint_from_m and not self._runup_released:
            # 助跑起步偏差由原跟踪（起步锁车头 + 横偏反馈）收掉再交，见 PathConfig.runup_release_e_m
            started = 0.0 if self._creep_from is None else st.s - self._creep_from
            self._runup_released = (started >= cfg.start_hold_m and abs(st.e) < cfg.runup_release_e_m
                                    and abs(st.heading_err) < math.radians(cfg.runup_release_heading_deg))
            runup_wait = not self._runup_released
            eligible = eligible and not runup_wait
        old_weight = self._straight_weight
        step = dt / cfg.straight_blend_s
        self._straight_weight = max(0.0, min(1.0, old_weight + (step if eligible else -step)))
        if self._straight_weight == 0.0:
            self._straight_target = None
            self._straight_last_s = None
            self._straight_settled = False
            self._straight_status = {'active': False, 'weight': 0.0, 'eligible': eligible, 'control_law': 'ab',
                                     'runup_wait': runup_wait}
            return legacy_w

        reference = wrap(imu_yaw + self._straight_offset)
        x, y = pose.x, pose.y
        if self._imu_lever is not None:
            ax, ay = self._imu_lever
            cf, sf, cr, sr = math.cos(pose.yaw), math.sin(pose.yaw), math.cos(reference), math.sin(reference)
            x += (cf-cr)*ax - (sf-sr)*ay
            y += (sf-sr)*ax + (cf-cr)*ay
        _, e_now, _, _ = self.locate(x, y, reference, st.s)
        nx, ny = self._straight_nozzle
        relative = wrap(reference-piece.yaw)
        e_now += (nx-cfg.track_x_m)*math.sin(relative) + ny*(math.cos(relative)-1.0)
        # 同 A/B，积分路程来自喷头的沿线位置增量，不用瞬时霍尔速度乘时间。
        s_nozzle = (math.cos(piece.yaw)*(x-piece.x) + math.sin(piece.yaw)*(y-piece.y)
                    + nx*math.cos(relative) - ny*math.sin(relative) - (nx-cfg.track_x_m))
        if old_weight == 0.0:
            self._straight_integral = -cfg.ki_per_m2 * st.integral / self.k_theta
            self._straight_target = imu_yaw
            self._straight_last_s = None
            sent = [(self._imu_now-(self._clock-t), w) for t, _, w in self._sent]
            self._straight_core.reset(now=self._imu_now, sent=sent, output=legacy_w)
        distance = 0.0 if self._straight_last_s is None else max(0.0, s_nozzle-self._straight_last_s)
        self._straight_last_s = s_nozzle
        integral = -self._straight_integral / ab.k_i if ab.k_i > 0.0 else 0.0
        previous_target = self._straight_target
        integral, self._straight_bias, self._straight_target, aligning = lateral_heading_step(
            ab, e_now, distance, integral, self._straight_target, piece.yaw, self._straight_offset,
            dt, st.integral_frozen)
        self._straight_integral = -ab.k_i * integral
        self._straight_target_rate = wrap(self._straight_target-previous_target)/dt
        # 仅换回路径积分单位供交接；A/B 自己的积分保留 2.5° 上限。
        st.integral = max(-cfg.integral_limit/cfg.ki_per_m2, min(cfg.integral_limit/cfg.ki_per_m2,
                          -self._straight_integral*self.k_theta/cfg.ki_per_m2))
        cruise = min(cfg.cruise_mps, self.speed_limit_mps) if self.speed_limit_mps is not None else cfg.cruise_mps
        if not self._straight_settled and not aligning and speed >= cruise-1e-6:
            self._straight_settled = True
            self._straight_offset_n = 0
        sample_t, sample_yaw, _ = self._imu
        error = wrap(sample_yaw-self._straight_target)
        moving = (speed >= ab.hold.moving_ratio*cfg.cruise_mps
                  and pose.speed >= ab.hold.moving_ratio*speed)
        inner_w = self._straight_core.step(self._imu_now, error, gyro, sample_t=sample_t,
                                          moving=moving, dt=dt, record=False)
        output = legacy_w + self._straight_weight * (inner_w-legacy_w)
        output = max(-self.kappa_max*speed, min(self.kappa_max*speed, output))
        self._straight_status = dict(active=True, weight=self._straight_weight, eligible=eligible, control_law='ab',
            target_deg=math.degrees(self._straight_target), offset_deg=math.degrees(self._straight_offset),
            bias_deg=math.degrees(self._straight_bias), error_deg=math.degrees(error),
            target_rate_deg_s=math.degrees(self._straight_target_rate), aligning=aligning,
            integral_deg=math.degrees(self._straight_integral), reference_learning=True,
            outer_e=e_now, outer_e_pred=e_now, outer_e_control=e_now, outer_distance_m=distance,
            outer_rate_mps=0.0, integral_return_frozen=False, outer_trend_weight=0.0,
            inner_wz=inner_w, legacy_wz=legacy_w, final_wz=output, **self._straight_core.status())
        return output

    def _runup_position_trusted(self, pose):
        """助跑拟合比沿线拟合更严格：位置、测向均新鲜，当前帧也必须合格。"""
        if not position_observation_allowed(pose):return False
        first = self.cfg.straight_imu_first
        if getattr(pose, 'single_antenna', False):
            # 单天线（2026-10-07）：没有测向、副天线，「卫星可信」（rtk_trusted）和测向帧、两天线距离永远没有，原条件下
            # 开喷前检查永远不过、一条线都画不了。按单天线的等价条件：固定解（PDOP 不合格的帧任务层已按没有解交给融合）、
            # 定位可用、不在推算维持、0.5 s 内有新定位、精度和卫星数合格
            reasons = getattr(pose, 'reasons', ()) or ()
            if (getattr(pose, 'quality', None) != 'RTK_FIXED' or not getattr(pose, 'usable_for_control', False)
                    or 'rtk_hold' in reasons or 'rtk_hold_pending' in reasons
                    or self._imu_now is None or self._quality_frame_t is None
                    or not 0.0 <= self._imu_now - self._quality_frame_t <= 0.5
                    or not self._rtk_quality_ok() or self._quality is None):
                return False
            sigma, sats, pair, fixed = self._quality
            return self._quality_good(sigma, sats, pair, fixed)
        if (not getattr(pose, 'rtk_trusted', False) or getattr(pose, 'quality', None) != 'RTK_FIXED'
                or self._imu_now is None or self._quality_frame_t is None or self._baseline_frame_t is None
                or not 0.0 <= self._imu_now - self._quality_frame_t <= 0.5
                or not 0.0 <= self._imu_now - self._baseline_frame_t <= 0.5
                or self._baseline_bad_n or not self._baseline_ok() or not self._rtk_quality_ok()
                or self._quality is None):
            return False
        sigma, sats, pair, fixed = self._quality
        return (self._quality_good(sigma, sats, pair, fixed)
                and pair is not None and math.isfinite(pair) and pair <= first.fit_max_pair_m)

    def _straight_eligible(self, piece, speed):
        """这拍能不能（继续）交给直线内环：(可以, 助跑还在等收稳)。"""
        cfg, st = self.cfg, self.st
        first_index,last_index=self._straight_run_bounds(piece)
        first_piece,last_piece=self.path.pieces[first_index],self.path.pieces[last_index]
        u=st.s-first_piece.s0
        length=last_piece.s0+last_piece.length-first_piece.s0
        left=last_piece.s0+last_piece.length-st.s
        reserve = max(1.0, speed*cfg.straight_blend_s + speed*cfg.delay_s + speed*speed/(2*cfg.decel_mps2))
        if cfg.straight_end_reserve_m is not None and last_piece is self.path.pieces[-1]:
            reserve = cfg.straight_end_reserve_m
        eligible = (length >= cfg.straight_min_length_m
                    and (u >= cfg.straight_settle_m or first_piece is self.path.pieces[0] and u < 0.)
                    and speed > cfg.straight_min_speed_mps and left > reserve)
        runup_wait = False
        if self.entry_stabilize_mps is not None:
            # 助跑就是正式直线的延长线；IMU 参考就绪便渐入同一控制器，不再等走满固定距离或先自行对准。
            # 开喷仍由独立的起步检查判定。
            self._runup_released=True
        if self.runup_speed_mps is not None and (st.s < self.paint_from_m or self.reverse_runup) and not self._runup_released:
            # 倒车助跑未收稳时，越过线头仍由原跟踪纠偏；这里只管转向交接，不改变喷涂资格。
            started = 0.0 if self._creep_from is None else st.s - self._creep_from
            self._runup_released = (started >= cfg.start_hold_m and abs(st.e) < cfg.runup_release_e_m
                                    and abs(st.heading_err) < math.radians(cfg.runup_release_heading_deg))
            runup_wait = not self._runup_released
            eligible = eligible and not runup_wait
        return eligible, runup_wait

    def _paint_along(self, piece):
        """开喷后在这条直线上走了多远（画线时往回拉的线头段用，line_cascade.paint_pull）：从这段喷线的起点（线头）、
        直线开头（弯道出口）、这次起步 / 继续的位置里最后那个算起。不在喷（转场段、停喷后的收尾）按 0（往回拉照旧）。"""
        s = self.st.s
        spans = self.paint_spans if self.paint_spans is not None else [(self.paint_from_m, self.paint_to_m)]
        head = next((a for a, b in spans if a <= s < b), None)
        if not self._arc_work_role or head is None:
            return 0.0
        lo, _ = self._straight_run_bounds(piece)
        return s - max(head, self.path.pieces[lo].s0, self._started_at)

    def _straight_first(self, pose, dt, speed, legacy_w, piece, imu_yaw, gyro):
        """长直线 IMU 为主、卫星辅助（line_cascade.ImuFirstGuide 的说明）。车速、停车、弯道交接仍归路径跟踪；
        这里只出转向，另把喷头横偏估计留给偏离中止、降速和喷线资格（卫星跳了时它们也按 IMU 推算）。"""
        cfg, st, guide = self.cfg, self.st, self._straight_guide
        first, align = guide.cfg, self._imu_alignment
        # 同一段助跑与喷线在边界上共享控制状态；仅原本有助跑的直线做半米交接。
        entry_weight=(runup_weight(first,st.s-self.paint_from_m)
                      if self.paint_from_m>0. else 0.)
        baseline_ok = self._baseline_ok()
        if guide.token is None and align is not None:
            guide.token = align.begin_line()            # 新的一条线：上一条线拟合好的换算定下来，可带过来
        was_locked = guide.locked
        if not guide.locked and baseline_ok:
            # 接管前按原有样本数学习换算；助跑期间的位置拟合负责校正已锁定的初始误差。
            guide.learn_offset(wrap(pose.yaw - imu_yaw), gyro)
        if self.reverse_runup and st.s >= self.paint_from_m and not baseline_ok:
            # 线头已进坏区时，沿用原有交接；不能继续等待不可信测向把横偏收进新门槛。
            self._runup_released = True
        eligible, runup_wait = self._straight_eligible(piece, speed)
        # 已有可信跨线标定时，本段最少样本确认一致便可开始原有渐入。
        # 仍保留 .8 s 控制交接；没有可信历史或本段尚未确认时照旧累计完整样本。
        if (eligible and not guide.locked and baseline_ok and self.entry_stabilize_mps is not None
                and st.s<self.paint_from_m and align is not None
                and guide.offset_n>=first.offset_min_samples and self._runup_position_trusted(pose)):
            predicted=align.predict(self._imu_now)
            if (predicted is not None and predicted[1]<=math.radians(align.cfg.dual_sigma_deg)
                    and abs(wrap(predicted[0]-guide.offset))<=math.radians(1.)):
                guide.lock_without_baseline(align,self._imu_now,wrap(pose.yaw-imu_yaw))
        if self._launch_armed:
            self._launch_armed = False
            if eligible and not self._rolling_entry:
                self._launch_weight = 1.0
                self._launch_at = self._clock
                # 起步时只取一次线路方向换算，限到 2°；之后的定位抖动不能拖着参考左右变。
                heading = max(-LAUNCH_HEADING_LIMIT_RAD, min(LAUNCH_HEADING_LIMIT_RAD, st.heading_err))
                self._straight_target = wrap(imu_yaw-heading)
                self._straight_core.reset(now=self._imu_now)
        if eligible and not guide.locked and not baseline_ok:
            # 该接管了、开头基线就不对：用这段已攒的或借上一条线对好的角度先开（用户 10-06）
            guide.lock_without_baseline(align, self._imu_now, wrap(pose.yaw - imu_yaw))
        if guide.locked and not was_locked and align is not None:
            # 双天线学的和上一条线带过来的（按时间、转角校准）按误差加权
            guide.blend_start(align.predict(self._imu_now), align.cfg, rate=align.predict_rate(self._imu_now))
        eligible = eligible and guide.locked
        measured = None
        if guide.locked:
            reference = wrap(imu_yaw + guide.offset)
            rel = wrap(reference - piece.yaw)
            heading = wrap(reference + guide.beta)
            x, y = pose.x, pose.y
            if self._imu_lever is not None:
                # 融合用自己的航向从天线倒推后轴；按 IMU 参考航向（含 β）重推，卫星测向跳了也不影响位置
                ax, ay = self._imu_lever
                cf, sf, cr, sr = math.cos(pose.yaw), math.sin(pose.yaw), math.cos(heading), math.sin(heading)
                x += (cf-cr)*ax - (sf-sr)*ay
                y += (sf-sr)*ax + (cf-cr)*ay
            _, base_lat, _, _ = self.locate(x, y, heading, st.s)
            nx, ny = self._straight_nozzle
            h = rel + guide.beta
            measured = base_lat + (nx-cfg.track_x_m)*math.sin(h) + ny*(math.cos(h)-1.0)
            # 卫星纠偏：拟合按用户定的可信规则（固定解 + 基线 0.30–0.50），观测器老办法仍按 0.40–0.42
            # 助跑冻结的是横偏积分，不是不许用可信位置拟合；否则起步锁住的角度误差一直推到线头。
            # 起喷前积分按设计冻结，正常融合校正也可冻结积分；二者都不应阻断可信位置拟合。
            # 所有直线助跑共用当前质量、时效及跳点检查，不能只给倒车接入放行。
            starting = (st.s < self.paint_from_m or self.paint_start_guard is not None
                        and not self.paint_start_guard.started
                        or self._entry_speed_active and not self._entry_ramp_released)
            position_ok = (self._runup_position_trusted(pose)
                           if self.runup_speed_mps is not None and starting
                           else not st.integral_frozen)
            rtk_ok = position_ok and (self._baseline_ok(fit=True) and self._rtk_quality_ok() if first.rtk_fit
                                      else baseline_ok)
            pick = self._pick_antenna(guide, heading, st.s, base_lat, measured) if first.rtk_fit else None
            if pick is not None:
                # 两根天线对不上、有一根和 IMU 推算对得上：用它，两天线的距离误差不再挡；接收机质量照常要合格
                rtk_ok = position_ok and self._baseline_ok(fit=True) and self._receiver_quality_ok()
                measured += pick[1]
            # 用副天线时，融合位置（跟着被干扰的主天线）比副天线多偏的那段沿线法向挪回去，报给 app（地图喷线、作业报告）
            self._antenna['shift'] = None if pick is None or pick[0] != 'second' else (
                -math.sin(piece.yaw) * pick[1], math.cos(piece.yaw) * pick[1])
            self._antenna['shift_t'] = self._imu_now
            # steering：IMU 已经在开（上一拍权重 > 0）；rtk_trim、rtk_fit 都关时这以后卫星不再修正
            guide.update(measured, rel, max(0.0, pose.speed)*dt, dt, (nx, ny), rtk_ok, fast=st.s < self.paint_from_m,
                         steering=self._straight_weight > 0.0)
            if first.rtk_fit and entry_weight>0. and rtk_ok and guide.rtk_accepted:
                # 助跑及线头交接仍只接纳原质量门和跳点门放行的位置；对齐权重平滑退到零。
                guide.pull_to(measured, 1.0 - math.exp(-max(0.0, pose.speed)*dt*entry_weight / first.runup_rtk_tau_m))
            sigma = guide.fit_sigma()
            if (align is not None and first.rtk_fit and guide.rtk_accepted
                    and sigma is not None and sigma <= math.radians(first.carry_fit_sigma_deg)):
                # 卫星拟合好了：记下这条线真实的换算（偏移 + β）和误差，给下一条线带过去、校准随时间 / 转角的变化
                align.update_line(guide.token, self._imu_now, guide.offset + guide.beta, sigma)
                align.update_rate(guide.token, self._imu_now, *guide.fit_rate())
            elif align is not None and (
                    first.rtk_trim and rtk_ok and guide.mode == 'rtk' and guide.travel >= first.fast_m
                    or not first.rtk_trim and not first.rtk_fit and guide.source == 'learned'):
                # 卫星对过、过了对准阶段（只靠 IMU 时：这段自己学到了偏移）：记下这条线对好的角度，下一条线开头基线不对时借
                align.update_line(guide.token, self._imu_now, guide.offset + guide.beta)
        old_weight = self._straight_weight
        step = dt / cfg.straight_blend_s
        self._straight_weight = max(0.0, min(1.0, old_weight + (step if eligible else -step)))
        if self._straight_weight == 0.0:
            if self._launch_weight > 0.0:
                # 参考尚未就绪时不追逐定位横偏；用实际陀螺和已发指令压住启动扰动。
                # 不能无限期锁住起步方向：超时/超过 1 m 后平滑退回原控制。
                if (self._clock-self._launch_at > LAUNCH_HOLD_MAX_S
                        or st.s-self._started_at >= LAUNCH_HOLD_MAX_M):
                    self._launch_weight = max(0.0, self._launch_weight-step)
                if self._launch_weight > 0.0:
                    sample_t, sample_yaw, _ = self._imu
                    error = wrap(sample_yaw-self._straight_target)
                    inner_w = self._straight_core.step(self._imu_now, error, gyro, sample_t=sample_t,
                                                       moving=False, dt=dt, record=False)
                    output = legacy_w + self._launch_weight*(inner_w-legacy_w)
                    output = max(-self.kappa_max*speed, min(self.kappa_max*speed, output))
                    self._straight_status = dict(active=True, weight=0.0, eligible=False,
                        control_law='imu_launch', launch_weight=self._launch_weight,
                        target_deg=math.degrees(self._straight_target), error_deg=math.degrees(error),
                        target_rate_deg_s=0.0, inner_wz=inner_w, legacy_wz=legacy_w, final_wz=output,
                        runup_wait=runup_wait, baseline_ok=baseline_ok, **guide.status(),
                        **self._straight_core.status())
                    return output
            self._straight_target = None
            self._straight_guided = None
            self._straight_status = dict(active=False, weight=0.0, eligible=eligible, control_law='imu_first',
                                         runup_wait=runup_wait, baseline_ok=baseline_ok, **guide.status(),
                                         **self._antenna_status(), **(align.status() if align is not None else {}))
            return legacy_w

        if old_weight == 0.0 and self._launch_weight == 0.0:
            self._straight_target = imu_yaw
            guide.target_rate = 0.0
            self._straight_aligned = False
            sent = [(self._imu_now-(self._clock-t), w) for t, _, w in self._sent]
            self._straight_core.reset(now=self._imu_now, sent=sent, output=legacy_w)
        previous_target = self._straight_target
        # 纠偏增益、限幅、目标角速度/加速度与位置观察器一起连续交接，避免开喷那拍突然松手。
        pull = paint_pull(first, self._paint_along(piece))
        self._straight_target, self._straight_bias, aligning, self._straight_aligned = imu_first_target(
            first, guide, self._straight_target, piece.yaw, dt,cfg.straight_target_rate_deg_s,
            CascadeConfig.align_deg, self._straight_aligned, pull=pull,entry_weight=entry_weight)
        self._straight_target_rate = wrap(self._straight_target-previous_target)/dt
        # β 相当于原 A/B 积分给的车头偏置（−β）：换成路径积分单位，交回原跟踪（进弯）时接着用
        self._straight_integral = -guide.beta
        st.integral = max(-cfg.integral_limit/cfg.ki_per_m2, min(cfg.integral_limit/cfg.ki_per_m2,
                          -self._straight_integral*self.k_theta/cfg.ki_per_m2))
        sample_t, sample_yaw, _ = self._imu
        target_rate = self._straight_target_rate if first.rtk_fit else 0.0
        # IMU 样本与目标按同一时刻比较；HeadingControl 已负责执行延迟的参考前推。
        error = wrap(sample_yaw-self._straight_target + max(0.0, self._imu_now-sample_t)*target_rate)
        moving = (speed >= HoldConfig.moving_ratio*cfg.cruise_mps
                  and pose.speed >= HoldConfig.moving_ratio*speed)
        inner_w = self._straight_core.step(self._imu_now, error, gyro, sample_t=sample_t,
                                          moving=moving, target_rate=target_rate, dt=dt, record=False)
        output = legacy_w + max(self._straight_weight,self._launch_weight) * (inner_w-legacy_w)
        output = max(-self.kappa_max*speed, min(self.kappa_max*speed, output))
        if self._straight_weight >= 1.0:
            self._launch_weight = 0.0
        self._straight_guided = (guide.e, rel + guide.beta)
        self._straight_status = dict(active=True, weight=self._straight_weight, eligible=eligible,
            control_law='imu_first', launch_weight=self._launch_weight, target_deg=math.degrees(self._straight_target),
            bias_deg=math.degrees(self._straight_bias), error_deg=math.degrees(error),
            pull_k_e=pull[0]+entry_weight*(first.runup_k_e-pull[0]),
            pull_max_bias_deg=pull[1]+entry_weight*(first.runup_max_bias_deg-pull[1]),
            entry_blend_weight=round(entry_weight,4),
            target_rate_deg_s=math.degrees(self._straight_target_rate), aligning=aligning, baseline_ok=baseline_ok,
            outer_e=guide.e, outer_e_rtk=measured, inner_wz=inner_w, legacy_wz=legacy_w, final_wz=output,
            **guide.status(), **self._antenna_status(), **self._straight_core.status(),
            **(align.status() if align is not None else {}))
        return output

    def _arc_radial_rtk_weight(self, rtk_ok, dt):
        """只给新鲜固定解且双天线距离持续一致的作业圆弧径向观测加权；不改变航向、进度或跳点保护。"""
        cfg, first = self.cfg, self.cfg.straight_imu_first
        now, quality, secondary = self._imu_now, self._quality, self._secondary
        paired = False
        if (cfg.arc_rtk_radial_enabled and rtk_ok and self._arc_hold is None
                and now is not None and quality is not None and secondary is not None
                and self._quality_frame_t is not None):
            sigma, sats, pair, fixed = quality
            paired = (fixed is True and secondary[3] is True
                and math.isfinite(sigma) and sigma <= first.fit_max_sigma_m
                and sats is not None and sats >= first.fit_min_sats
                and pair is not None and math.isfinite(pair) and 0. <= pair <= cfg.arc_rtk_pair_max_m
                and 0. <= now-self._quality_frame_t <= cfg.arc_rtk_fresh_s
                and 0. <= now-secondary[0] <= first.antenna_fresh_s)
        if not paired:
            self._arc_rtk_since = self._arc_rtk_frame = None
            self._arc_rtk_frames = 0
            self._arc_rtk_weight = 0.0
            return 0.0
        if self._quality_frame_t != self._arc_rtk_frame:
            self._arc_rtk_frame = self._quality_frame_t
            self._arc_rtk_frames += 1
            if self._arc_rtk_since is None:
                self._arc_rtk_since = now
        if self._arc_rtk_frames >= 3 and now-self._arc_rtk_since >= cfg.arc_rtk_settle_s:
            self._arc_rtk_weight = min(1., self._arc_rtk_weight+dt/cfg.straight_blend_s)
        return self._arc_rtk_weight

    def _correct_arc_radial(self, measured, distance, rtk_ok, dt, raw=None, *, partial=False):
        """有界互补校正；不改变用于维持转向的 IMU 航向和圆上进度。"""
        cfg = self.cfg
        weight = self._arc_radial_rtk_weight(rtk_ok and (self._primary is None or raw is not None), dt)
        observed=measured if raw is None else measured+weight*(raw-measured)
        self._arc_observed_e=observed
        noise_m=cfg.arc_partial_rtk_noise_m if partial else cfg.arc_rtk_noise_m
        error_m=cfg.arc_partial_rtk_error_m if partial else cfg.arc_rtk_error_m
        fast_pull_m=cfg.arc_partial_rtk_pull_m if partial else cfg.arc_rtk_pull_m
        weight *= min(1., max(0., (abs(observed-self._arc_e)-noise_m)/(error_m-noise_m)))
        self._arc_good = not partial and self._arc_good_sky()
        slow_m, slow_rate = ((cfg.arc_good_pull_m, cfg.arc_good_pull_rate_m) if self._arc_good
                             else (cfg.arc_pull_m, cfg.arc_pull_rate_m))
        pull_m = slow_m + weight*(fast_pull_m-slow_m)
        pull_rate = slow_rate + weight*(cfg.arc_rtk_pull_rate_m-slow_rate)
        correction = 0.0
        if rtk_ok and self._arc_hold is None:
            lim = pull_rate * distance
            correction = max(-lim, min(lim, (observed-self._arc_e)*min(1.,distance/pull_m)))
            self._arc_e += correction
        return weight, correction

    def _arc_good_sky(self):
        """整圆这一拍卫星好（见 PathConfig.arc_good_pull_m）：最新一帧固定解、卫星数够、σ 小、两天线距离误差小，且离上次跳点放开已走
        够 arc_good_after_jump_m。逐拍判断、不攒权重：差一帧只是这一拍按原慢修。"""
        cfg, first, q = self.cfg, self.cfg.straight_imu_first, self._quality
        if q is None or self._imu_now is None or self._quality_frame_t is None:
            return False
        sigma, sats, pair, fixed = q
        return (fixed is True and sats is not None and sats >= first.fit_full_sats
                and math.isfinite(sigma) and sigma <= first.fit_max_sigma_m
                and pair is not None and math.isfinite(pair) and 0. <= pair <= cfg.arc_good_pair_max_m
                and 0. <= self._imu_now-self._quality_frame_t <= cfg.arc_rtk_fresh_s
                and (self._arc_hold_end is None or self._arc_travel-self._arc_hold_end >= cfg.arc_good_after_jump_m))

    def _arc_first(self, pose, dt):
        """画圆 IMU 为主（PathConfig.arc_imu_first）：返回原弯道控制律这一拍转向用的位姿（x, y, yaw），不在整圆/可用作业弧上或还没
        学好换算时返回 None（按融合位姿开）。非整圆只校正精确喷头半径，保持融合航向与喷头方位；
        整圆互补滤波：车头 = IMU 偏航 + 换算（换算按 arc_heading_tau_s 慢慢跟融合航向），
        喷头横偏 = IMU 推算 + 往卫星拉；新鲜可靠双天线持续一致时用 arc_rtk_pull_* 及时校正，
        其余观测沿用 arc_pull_* 慢修；突然跳开 > jump_m 先不拉，
        回来或 jump_accept_s 后再拉），沿圆路程按走过的路推、慢慢靠向卫星。不估角度 β：圆一圈只有十几米，
        直线那套角度拟合在卫星小台阶上会学偏（05:41 那圈回放学出 −0.5°、推算和卫星差到 10 cm）。"""
        cfg, st, guide = self.cfg, self.st, self._arc_guide
        if not cfg.arc_imu_first or guide is None or not cfg.straight_imu_enabled or not self.geo.differential:
            return None
        imu = self._fresh_straight_imu()
        piece = next((p for p in self.path.pieces if p.s0 <= st.s < p.s0 + p.length), None)
        partial = self._partial_work_arc_piece(piece) if cfg.arc_partial_imu_first else None
        if partial is not None:
            piece = partial
        if (imu is None or piece is None or piece.straight or (partial is None and piece.length * abs(piece.kappa) < 2 * math.pi - 1e-3)
                or 'imu_mismatch' in getattr(pose, 'reasons', ())):
            if self._arc_piece is not None:
                self._reset_arc()
            return None
        if piece is not self._arc_piece:
            self._reset_arc()
            self._arc_piece = piece
        first = guide.cfg
        _, imu_yaw, gyro = imu
        baseline_ok = self._baseline_ok()
        # _arc_first 在 _control 之前执行，不能读取上一拍的 integral_frozen。
        # 定位退化时连同沿圆进度、轮速比例及航向换算一起停止学习，不能只冻结径向修正。
        rtk_ok = lateral_integral_allowed(pose) and self._baseline_ok(fit=True) and self._rtk_quality_ok()
        heading_ok = baseline_ok and rtk_ok and self._arc_hold is None
        # 径向恢复使用成对位置的持续确认；航向换算仍保留上面的长期质量门槛。
        partial_pair_ok=partial is not None and self._partial_arc_pair_ok()
        if partial is not None:
            rtk_ok=lateral_integral_allowed(pose) and self._baseline_ok(fit=True) and partial_pair_ok
        difference = wrap(pose.yaw - imu_yaw)
        if not guide.locked:
            if heading_ok:
                # 圆上本来就转着（v·κ）；挡的是原地转、甩头（融合航向和 IMU 没有滞后，见 ImuFirstConfig.gyro_max_deg_s）
                guide.learn_offset(difference, gyro - pose.speed * piece.kappa)
            if not guide.locked:
                self._arc_status = dict(active=False, weight=0.0, baseline_ok=baseline_ok, first_offset_n=guide.offset_n)
                return None
            self._arc_off = guide.offset
        elif heading_ok:
            self._arc_off = wrap(self._arc_off + wrap(difference - self._arc_off) * min(1.0, dt / cfg.arc_heading_tau_s))
        reference = wrap(imu_yaw + self._arc_off)
        partial_raw = self._arc_raw_radial(piece) if partial is not None and cfg.arc_rtk_radial_enabled else None
        if partial is not None:rtk_ok = rtk_ok and partial_raw is not None
        x, y = pose.x, pose.y
        if self._imu_lever is not None:
            # 同直线：融合用自己的航向从天线倒推后轴，这里按 IMU 参考航向重推
            ax, ay = self._imu_lever
            cf, sf, cr, sr = math.cos(pose.yaw), math.sin(pose.yaw), math.cos(reference), math.sin(reference)
            x += (cf-cr)*ax - (sf-sr)*ay
            y += (sf-sr)*ax + (cf-cr)*ay
        s_rtk, measured, _, _ = self.locate(x, y, reference, st.s)
        distance = max(0.0, pose.speed) * dt
        if self._arc_s is None:
            self._arc_s = s_rtk
        else:
            # 沿圆路程按走过的路（× 学到的比例）推，再往卫星挪：卫星沿圆跳 1 cm 圆切线就转 κ·1 cm（R 2 m 约 0.3°），
            # 一下跳开的按距离慢慢吃进去；车速读数比例不准由比例项慢慢学掉，不留稳态差
            self._arc_s += distance * self._arc_scale
            if rtk_ok and self._arc_hold is None:
                gap = s_rtk - self._arc_s
                lim = cfg.arc_s_rate_m * distance
                self._arc_s += max(-lim, min(lim, gap * distance / cfg.arc_s_tau_m))
                self._arc_scale = max(0.8, min(1.2, self._arc_scale + gap * distance / cfg.arc_scale_tau_m ** 2))
        px, py, pyaw = self.path.pose(self._arc_s)
        rel = wrap(reference - pyaw)
        nx, ny = self._straight_nozzle
        fx = nx - cfg.track_x_m
        lever = fx*math.sin(rel) + ny*(math.cos(rel)-1.0)
        measured += lever
        if self._arc_e is None:
            self._arc_e, self._arc_rel, self._arc_travel = measured, rel, 0.0
            self._arc_jump_e = measured
        else:
            prev = self._arc_rel
            if self._arc_jump_e is None:
                self._arc_jump_e = self._arc_e
            predicted = (distance*math.sin(0.5*(rel+prev)) + fx*(math.sin(rel)-math.sin(prev))
                         + ny*(math.cos(rel)-math.cos(prev)))
            self._arc_e += predicted
            self._arc_jump_e += predicted
            self._arc_inertial_sum += predicted
            if self._arc_raw_expected is not None:
                self._arc_raw_expected += predicted
            self._arc_rel = rel
            self._arc_travel += distance
        r = measured - self._arc_jump_e
        raw = partial_raw if partial is not None else self._arc_raw_radial(piece) if cfg.arc_rtk_radial_enabled else None
        # 共同跳点会让融合暂时冻结修偏，但原始位置跳点检测必须保留前一帧参考。
        # 只检测，不在融合不可信时应用位置校正；整圆保持原来的策略。
        raw_jump_ok=(partial_pair_ok and self._baseline_ok(fit=True)) if partial is not None else rtk_ok
        if raw_jump_ok and raw is not None:
            stamp=(self._primary[0],self._secondary[0])
            prior=self._arc_raw_last
            if prior is None or stamp != prior[0]:
                if prior is not None and self._arc_hold is None:
                    expected=prior[1]+self._arc_inertial_sum-prior[2]
                    if abs(raw-expected)>first.jump_m:
                        self._arc_hold=[0.,raw-expected]
                        self._arc_raw_hold=True
                        self._arc_raw_expected=expected
                        self._arc_jumps+=1
                self._arc_raw_last=(stamp,raw,self._arc_inertial_sum)
        else:
            self._arc_raw_last=None
        hist = self._arc_hist
        if not rtk_ok:
            hist.clear()
            if self._arc_hold is not None:
                self._arc_hold[0] = 0.0   # 连续稳定计时不能跨过定位质量中断
        elif self._arc_hold is None:
            while len(hist) >= 2 and hist[1][0] <= self._arc_travel - first.jump_window_m:
                hist.popleft()
            if self._arc_travel > first.trust_from_m and any(abs(r - rr) > first.jump_m for _, rr in hist):
                self._arc_hold = [0.0, r]             # 突然跳开：先不拉，[已等的秒数, 跳开那一拍的差]
                self._arc_raw_hold = False
                self._arc_raw_expected = None
                self._arc_jumps += 1
                hist.clear()
            else:
                hist.append((self._arc_travel, r))
        if self._arc_hold is not None and rtk_ok:
            residual=r if not self._arc_raw_hold else math.inf if raw is None else raw-self._arc_raw_expected
            self._arc_hold[0] = self._arc_hold[0]+dt if not self._arc_raw_hold or raw is not None else 0.
            if abs(residual) <= first.rejoin_m or self._arc_hold[0] >= first.jump_accept_s:
                self._arc_hold = None                 # 卫星回来和推算对上了，或等够了按卫星慢慢拉
                self._arc_hold_end = self._arc_travel
                self._arc_raw_hold = False
                self._arc_raw_expected = None
        if rtk_ok and self._arc_hold is None:
            lim = cfg.arc_pull_rate_m * distance
            self._arc_jump_e += max(-lim, min(lim, r*min(1.,distance/cfg.arc_pull_m)))
        radial_weight, correction = self._correct_arc_radial(measured, distance, rtk_ok, dt, raw, partial=partial is not None)
        e_track = self._arc_e - lever
        yaw_v = wrap(pyaw + rel)
        cx = px - math.sin(pyaw)*e_track - cfg.track_x_m*math.cos(yaw_v)
        cy = py + math.cos(pyaw)*e_track - cfg.track_x_m*math.sin(yaw_v)
        self._arc_weight = min(1.0, self._arc_weight + dt / cfg.straight_blend_s)
        w = self._arc_weight
        control_rel=rel
        if partial is not None:
            # 短弧直接修正喷头的圆半径；保留融合航向及当前喷头方位，避免把沿弧推算偏差带进转向。
            k=piece.kappa
            ox,oy=piece.x-math.sin(piece.yaw)/k,piece.y+math.cos(piece.yaw)/k
            qx=pose.x+nx*math.cos(pose.yaw)-ny*math.sin(pose.yaw)
            qy=pose.y+nx*math.sin(pose.yaw)+ny*math.cos(pose.yaw)
            dx,dy=qx-ox,qy-oy
            radius=math.hypot(nx,1./k-ny)
            ratio=(radius-math.copysign(1.,k)*self._arc_e)/max(1e-6,math.hypot(dx,dy))-1.
            cx,cy=pose.x+ratio*dx,pose.y+ratio*dy
            yaw_v=pose.yaw
            control_rel=self.locate(pose.x,pose.y,pose.yaw,st.s)[2]
        self._arc_guided = (self._arc_e, control_rel)
        self._arc_status = dict(active=True, weight=w, partial_arc=partial is not None, baseline_ok=baseline_ok, rtk_ok=rtk_ok, arc_s=self._arc_s,
                                rtk_s=s_rtk, e_est=self._arc_e, e_rtk=measured, heading_deg=math.degrees(control_rel),
                                offset_deg=math.degrees(self._arc_off), holding=self._arc_hold is not None,
                                jumps=self._arc_jumps, scale=self._arc_scale,
                                radial_source=('rtk_paired' if radial_weight > 0 else
                                               'rtk_slow' if rtk_ok and self._arc_hold is None else 'imu_hold'),
                                radial_rtk_weight=radial_weight, radial_correction_m=correction, good_sky=self._arc_good,
                                jump_e_est=self._arc_jump_e, e_rtk_raw=raw, e_observed=self._arc_observed_e)
        return SimpleNamespace(x=pose.x + w*(cx-pose.x), y=pose.y + w*(cy-pose.y), yaw=wrap(pose.yaw + w*wrap(yaw_v-pose.yaw)))

    def straight_guide_estimate(self):
        """直线 IMU 为主、内环已接管时的喷头横偏估计（m，线左为正）；其余时候 None（按卫星位置算）。"""
        return None if self._straight_guided is None else self._straight_guided[0]

    def predict(self, x, y, yaw, delay=None):
        """已发出、还在执行延迟里的指令都执行完后的 (x, y, yaw)。

        第 i 条指令在 [t_i, t_i+1) 期间有效、晚 delay_s 才作用到车上，所以车接下来 delay_s 内怎么走，由
        [现在 − delay_s, 现在) 期间有效的那些指令决定；车实际角速度 = 下发角速度 × steer_gain。
        delay 给了就按该时域推，最长为路径/直线内环两者所需的历史，段尾停车仍用原时域。
        """
        sent = self._sent
        retained = max(self.cfg.delay_s, self._straight_core.cfg.delay_s) if self.cfg.straight_imu_enabled else self.cfg.delay_s
        if self.cfg.arc_partial_imu_first:retained=max(retained,self.cfg.arc_partial_response_delay_s)
        while len(sent) >= 2 and sent[1][0] <= self._clock - retained + 1e-9:
            sent.popleft()
        d = self.cfg.delay_s if delay is None else min(delay, retained)
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

    def _predict_partial_motion(self, source, pose):
        """用 IMU 转速和轮速初始化待执行响应，再按指令队列预测；短弧专用。"""
        x,y,h=source.x,source.y,source.yaw
        speed,rate=pose.speed,pose.yaw_rate
        sent=list(self._sent)
        if not sent or self.cfg.delay_s<=0:return self.predict(x,y,h)
        delay=self.cfg.arc_partial_response_delay_s
        tau=self.cfg.arc_partial_response_tau_s
        horizon=min(self.cfg.delay_s,delay+tau)
        elapsed=0.;index=0
        while elapsed<horizon-1e-9:
            dt=min(.01,horizon-elapsed)
            t=self._clock+elapsed+.5*dt-delay
            while index+1<len(sent) and sent[index+1][0]<=t:index+=1
            _,vc,wc=sent[index]
            if t<sent[0][0]:vc=wc=0.
            alpha=1.-math.exp(-dt/tau)
            nv=speed+alpha*(vc-speed);nw=rate+alpha*(wc*self.cfg.steer_gain-rate)
            v,w=.5*(speed+nv),.5*(rate+nw)
            mid=h+.5*w*dt
            x+=v*math.cos(mid)*dt;y+=v*math.sin(mid)*dt;h+=w*dt
            speed,rate=nv,nw;elapsed+=dt
        return x,y,h

    def nozzle_end(self, pose, forward, left, at=None):
        """前进直线段尾的实际喷头剩余距离、沿线速度；不使用执行延迟预估或冻结的 st.s。

        仅供独立的喷漆截止判定，运动 DONE、减速曲线和拐角流程仍由原控制器决定。
        调用方须确认当前在最后一段直线上；圆和倒车不调用。
        """
        x,y,h=self.path.pose(self.path.length if at is None else at)
        c,s=math.cos(h),math.sin(h)
        cp,sp=math.cos(pose.yaw),math.sin(pose.yaw)
        dx=x+forward*c-left*s-(pose.x+forward*cp-left*sp)
        dy=y+forward*s+left*c-(pose.y+forward*sp+left*cp)
        vx=pose.speed*cp-pose.yaw_rate*(forward*sp+left*cp)
        vy=pose.speed*sp+pose.yaw_rate*(forward*cp-left*sp)
        return dx*c+dy*s, vx*c+vy*s

    def _preview_kappa(self, s):
        tr=self
        while s<0 and tr.predecessor is not None:
            tr=tr.predecessor
            s+=tr.path.length
        path=tr._preview_path if tr._preview_path is not None else tr.path
        return path.kappa(s)

    def feedforward_kappa(self, s, speed):
        """曲率前馈用的路径曲率：s（已是执行延迟之后的位置）再往前 ff_preview_s 处的曲率；ff_shape_ratio > 0 时
        与再早 ff_shape_gap_s 的曲率按比例混合（ZV 两脉冲：突变处先给 1 − ratio，过 gap 秒补满）。
        ff_entry_* 可在指定入圆范围减少额外前瞻；圆中段和出弧仍沿用原参数。
        车速范围按后轴巡航配置判断，并在 0.1 m/s 范围平滑渐入。
        """
        cfg = self.cfg
        path = self._preview_path if self._preview_path is not None else self.path
        v = max(0.0, speed)
        ahead = s + v * cfg.ff_preview_s
        k = self._preview_kappa(ahead)
        if cfg.ff_shape_ratio > 0.0:
            k_late = self._preview_kappa(ahead - v * cfg.ff_shape_gap_s)
            if max(abs(k), abs(k_late)) <= cfg.ff_shape_kappa_max:
                k += cfg.ff_shape_ratio * (k_late - k)
        if (self.geo.differential and cfg.ff_shape_ratio > 0.0
                and any(value is not None for value in (cfg.ff_entry_ratio, cfg.ff_entry_gap_s, cfg.ff_entry_preview_s))):
            entry_cruise = cfg.arc_speed_mps if cfg.arc_speed_mps is not None else cfg.cruise_mps
            weight = 1.0 if cfg.ff_entry_min_cruise_mps <= 0.0 else \
                max(0.0, min(1.0, (entry_cruise-cfg.ff_entry_min_cruise_mps)/.1))
            if weight <= 0.0:
                return k
            preview = cfg.ff_preview_s if cfg.ff_entry_preview_s is None else \
                cfg.ff_preview_s+weight*(cfg.ff_entry_preview_s-cfg.ff_preview_s)
            entry_ahead = s + v * preview
            probe = max(ahead, entry_ahead)
            piece = path._piece(max(0.0, probe))
            index = path.pieces.index(piece)
            if (index > 0 and path.pieces[index-1].straight and not piece.straight
                    and abs(piece.kappa) <= cfg.ff_shape_kappa_max):
                if cfg.ff_entry_clockwise_only and piece.kappa>0:
                    return k
                if cfg.ff_entry_full_circle_only and piece.length*abs(piece.kappa)<2*math.pi-1e-6:
                    return k
                nx,ny=self._straight_nozzle
                radius=math.hypot(1./abs(piece.kappa)-math.copysign(1.,piece.kappa)*ny,nx)
                if radius<cfg.ff_entry_min_radius_m-1e-9 or (cfg.ff_entry_max_radius_m is not None
                        and radius>cfg.ff_entry_max_radius_m+1e-9):
                    return k
                ratio = cfg.ff_shape_ratio if cfg.ff_entry_ratio is None else cfg.ff_shape_ratio+weight*(cfg.ff_entry_ratio-cfg.ff_shape_ratio)
                gap = cfg.ff_shape_gap_s if cfg.ff_entry_gap_s is None else cfg.ff_shape_gap_s+weight*(cfg.ff_entry_gap_s-cfg.ff_shape_gap_s)
                current = self._preview_kappa(entry_ahead)
                late = self._preview_kappa(entry_ahead-v*gap)
                if max(abs(current), abs(late)) <= cfg.ff_shape_kappa_max:
                    k = current+ratio*(late-current)
        return k

    def _turn_ramp_distance(self, speed, target):
        """三次平滑速度包络的保守长度：同时约束加速度和加速度变化率。"""
        delta=max(0.,speed-target)
        return max(1.5*delta*speed/self.cfg.turn_decel_mps2,
                   math.sqrt((6*delta*speed*speed+2.25*delta*delta*speed)/self.cfg.turn_jerk_mps3))

    def _work_arc_piece(self, piece):
        """喷涂圆弧及同曲率的预走/收尾属于作业；真正转场弯道独立限速。"""
        if not self._arc_work_role or piece.straight:return False
        pieces=self.path.pieces;i=pieces.index(piece);lo=hi=i
        while lo>0 and abs(pieces[lo-1].kappa-piece.kappa)<1e-9:lo-=1
        while hi+1<len(pieces) and abs(pieces[hi+1].kappa-piece.kappa)<1e-9:hi+=1
        start,end=pieces[lo].s0,pieces[hi].s0+pieces[hi].length
        spans=self.paint_spans if self.paint_spans is not None else [(self.paint_from_m,self.paint_to_m)]
        return any(start<b-1e-8 and end>a+1e-8 for a,b in spans)

    def _partial_arc_feedback_scale(self):
        """按喷头圆半径平滑分配纠偏强度；紧弧保留响应延迟余量，大弧及时纠偏。"""
        piece=self._arc_piece
        if piece is None or piece.straight:return 1.
        nx,ny=self._straight_nozzle
        radius=math.hypot(nx,1./piece.kappa-ny)
        u=max(0.,min(1.,(radius-1.5)/1.5))
        weight=u*u*(3.-2.*u)
        return self.cfg.arc_partial_tight_feedback_scale+weight*(
            self.cfg.arc_partial_rtk_feedback_scale-self.cfg.arc_partial_tight_feedback_scale)

    def _partial_arc_pair_ok(self):
        """短弧径向观测的当前双固定解质量；确认时长由 _arc_radial_rtk_weight 统一处理。"""
        cfg=self.cfg
        if (self._imu_now is None or self._quality is None or self._quality_frame_t is None
                or self._primary is None or self._secondary is None):
            return False
        sigma,sats,pair,fixed=self._quality
        return (fixed is True and 0.<=self._imu_now-self._quality_frame_t<=cfg.arc_rtk_fresh_s
            and all(fix[3] and 0.<=self._imu_now-fix[0]<=cfg.arc_rtk_fresh_s for fix in (self._primary,self._secondary))
            and self._quality_good(sigma,sats,pair,fixed)
            and pair is not None and 0.<=pair<=cfg.arc_rtk_pair_max_m)

    def _partial_work_arc_piece(self, piece):
        """短作业弧沿同圆预走/喷线/收尾共用观测状态；完整圆保持原来的接管时机。"""
        if (piece is None or not self._work_arc_piece(piece)
                or self._primary is None or self._secondary is None
                or self._quality is None or self._quality_frame_t is None
                or not self.cfg.arc_rtk_radial_enabled):
            return None
        nx,ny=self._straight_nozzle
        radius=math.hypot(nx,1./piece.kappa-ny)
        if radius<self.cfg.arc_partial_min_radius_m-1e-9:return None
        pieces=self.path.pieces;i=pieces.index(piece);lo=hi=i
        while lo>0 and abs(pieces[lo-1].kappa-piece.kappa)<1e-9:lo-=1
        while hi+1<len(pieces) and abs(pieces[hi+1].kappa-piece.kappa)<1e-9:hi+=1
        start,end=pieces[lo].s0,pieces[hi].s0+pieces[hi].length
        spans=self.paint_spans if self.paint_spans is not None else [(self.paint_from_m,self.paint_to_m)]
        painted=sum(max(0.,min(b,end)-max(a,start)) for a,b in spans)
        return pieces[lo] if 1e-6<painted*abs(piece.kappa)<2*math.pi-1e-3 else None

    def _curve_target_at(self, s):
        # 连续路径预瞄必须保留下一段的作业/转场身份，不能只看复制后的曲率。
        tracker=self
        while tracker.continuation is not None and s>=tracker.path.length:
            s-=tracker.path.length;tracker=tracker.continuation
        piece=tracker.path._piece(max(0.,s))
        arc=tracker.cfg.arc_speed_mps
        if arc is not None and tracker._work_arc_piece(piece):return arc
        turn=min(TURN_SPEED_MPS,arc if arc is not None else math.inf)
        if tracker.geo.differential:
            # 转场弯：内侧轮不低于 WHEEL_MIN_MPS（10-08 用户），不超过横向加速度（motion_speeds.transit_bend_speed）
            turn=transit_bend_speed(piece.kappa,tracker.geo.track_m,tracker.cfg.lat_accel_mps2,turn)
        return turn

    def _curve_speed(self, s, v):
        """Gen5 用首尾斜率为零的速度包络提前准备过弯，其他底盘保留原制动曲线。
        s 已包含执行延迟预估，额外距离余量使实际到弯口前完成降速；连续分段也统一预瞄。"""
        cfg = self.cfg
        v=max(v,self._v_prev)
        margin = v * cfg.lead_s + 0.3
        reach = s + max(v*v/(2*cfg.decel_mps2),self._turn_ramp_distance(v,0.)) + margin
        limit = math.inf
        preview_pieces=(self._preview_path if self._preview_path is not None else self.path).pieces
        for index,p in enumerate(preview_pieces):
            if p.s0 >= reach:
                break
            if p.s0 + p.length <= s or abs(p.kappa) <= 1e-6 or self._straight_stitch(preview_pieces,index):
                continue
            d = max(0.0, p.s0 - s - margin)
            curve_v2 = cfg.arc_speed_mps**2 if self.geo.differential and cfg.arc_speed_mps is not None else cfg.lat_accel_mps2 / abs(p.kappa)
            if self.geo.differential:
                target=self._curve_target_at(p.s0+min(1e-6,p.length/2))
                length=self._turn_ramp_distance(v,target)
                u=1. if length<=1e-9 else min(1.,d/length)
                allowed=target+max(0.,v-target)*u*u*(3.-2.*u)
            else:allowed=math.sqrt(curve_v2+2.*cfg.decel_mps2*d)
            limit=min(limit,allowed)
        # 换段前预先满足下一段速度/终点约束，不能交接后才突然减速。
        offset=self.path.length
        nxt=self.continuation
        while nxt is not None and offset-s < v*v/(2*cfg.decel_mps2)+margin:
            target=nxt.entry_speed_mps if nxt.entry_speed_mps is not None and nxt.paint_from_m>0 else nxt.cfg.cruise_mps
            if nxt.cfg.arc_speed_mps is not None and not nxt.path.pieces[0].straight:target=nxt._curve_target_at(0.)
            limit=min(limit,math.sqrt(target*target+2*cfg.decel_mps2*max(0.,offset-s-margin)))
            offset+=nxt.path.length
            if nxt.continuation is None and nxt.run_out_m<=0:
                limit=min(limit,max(cfg.min_mps,math.sqrt(2*cfg.decel_mps2*max(0.,offset-s-cfg.stop_tol_m))))
            nxt=nxt.continuation
        return limit

    def _accelerate(self, target, dt, *, smooth_decel=False):
        """提速与正常弯前降速限制加速度变化；保护和停车仍可独立收紧。"""
        if dt<=0:return min(target,self._v_prev)
        if self._launch_pending and target>0.:
            self._launch_pending=False
            self._launch_ramp = (self._launch_armed and not self._rolling_entry
                                 and self.cfg.straight_imu_enabled and self._fresh_straight_imu() is not None)
            if (not self._rolling_entry and self._v_prev<self.runup_start_mps
                    and not self._launch_ramp):
                # 没有新鲜 IMU 的既有起步流程；更低保护限速仍优先。
                self._accel_prev=0.
                return min(target,self.runup_start_mps)
        if self._launch_ramp:
            top=min(target,self.runup_start_mps)
            gap=top-self._v_prev
            if gap>1e-6:
                # 先由共同速度参考连续升至起步档，避免 0→0.35 阶跃放大两轮响应差异。
                # 不在低速抖动区久留；到起步档后接回原有缓慢提速。
                jerk=LAUNCH_JERK_MPS3
                taper=max(0.,math.sqrt((jerk*dt)**2+2*jerk*gap)-jerk*dt)
                accel=min(LAUNCH_ACCEL_MPS2,self._accel_prev+jerk*dt,taper)
                self._accel_prev=accel
                return min(top,self._v_prev+accel*dt)
            self._launch_ramp=False
        gap=target-self._v_prev
        if smooth_decel:
            jerk=self.cfg.turn_jerk_mps3
            direction=1. if gap>=0. else -1.
            taper=max(0.,math.sqrt((jerk*dt)**2+2*jerk*abs(gap))-jerk*dt)
            maximum=self.cfg.accel_mps2 if direction>0 else self.cfg.turn_decel_mps2
            wanted=direction*min(maximum,taper)
            accel=max(self._accel_prev-jerk*dt,min(self._accel_prev+jerk*dt,wanted))
            value=self._v_prev+accel*dt
            if (gap>=0. and self._v_prev<=target<=value or gap<0. and value<=target<=self._v_prev):
                reached=(target-self._v_prev)/dt
                # 弯前包络会随位置缓慢移动，不能为追上它而把加速度一拍截断。
                if abs(reached-self._accel_prev)<=jerk*dt+1e-9:
                    value=target;accel=reached
            self._accel_prev=accel
            return max(0.,value)
        jerk=self.cfg.accel_jerk_mps3
        if gap<=0 or jerk<=0:
            self._accel_prev=0.
            return min(target,self._v_prev+self.cfg.accel_mps2*dt)
        # 求下一拍加速度：还要留出把加速度降回零的速度余量，靠近目标不会硬截成巡航。
        taper=max(0.,math.sqrt((jerk*dt)**2+2*jerk*gap)-jerk*dt)
        accel=min(self.cfg.accel_mps2,self._accel_prev+jerk*dt,taper)
        self._accel_prev=accel
        return min(target,self._v_prev+accel*dt)

    def constant_arc_speed_here(self):
        piece=self.path._piece(max(0.,min(self.path.length,self.st.s)))
        return (self.geo.differential and self.cfg.arc_speed_mps is not None and not piece.straight
                and not self._straight_stitch(self.path.pieces,self.path.pieces.index(piece)))

    def _control(self, pose, dt, steer=None):
        """返回 (车速, 下发曲率, 起步锁车头另加的角速度)。steer：画圆 IMU 为主时转向用的位姿（x, y, yaw），
        其余（报给操作员、喷线、停车）仍按 pose。"""
        st, cfg = self.st, self.cfg
        self.paint_speed_ready=False
        self.paint_target_mps=None
        self.paint_expected_mps=None
        if st.state != 'RUNNING' or not pose.usable_for_control:
            self._arc_tail_w = 0.0
        if st.state not in ('RUNNING', 'PAUSED'):
            st.v_cmd = 0.0
            return 0.0, 0.0, 0.0
        st.integral_frozen = not lateral_integral_allowed(pose)
        if not pose.usable_for_control:
            st.state, st.reason = 'PAUSED', 'pose_unusable'
            st.v_cmd, self._v_prev = 0.0, 0.0
            return 0.0, 0.0, 0.0
        if st.state == 'PAUSED':
            st.reason = 'pose_recovered_waiting_operator'
            return 0.0, 0.0, 0.0

        # 执行延迟：先按已发出的指令把位姿推到 delay_s 之后，现在算的指令正好在那时生效（Smith 预估）；
        # 再按当前速度和角速度推 lead_s（弯道上角速度不为 0，推出去的是一段圆弧）
        src = pose if steer is None else steer
        x, y, yaw = self.predict(src.x, src.y, src.yaw)
        if self._arc_status.get('partial_arc'):
            nx,ny,nh=self._predict_partial_motion(src,pose)
            blend=self._arc_weight
            x+=blend*(nx-x);y+=blend*(ny-y);yaw=wrap(yaw+blend*wrap(nh-yaw))
        mid = yaw + 0.5 * pose.yaw_rate * cfg.lead_s
        x += pose.speed * cfg.lead_s * math.cos(mid)
        y += pose.speed * cfg.lead_s * math.sin(mid)
        yaw += pose.yaw_rate * cfg.lead_s
        s, e, he, k_r = self.locate(x, y, yaw, self._hint, preview=True)
        ds = 0.0 if self._hint is None else max(0.0, s - self._hint)
        self._hint = s
        # 报给操作员、记录、偏离中止用车现在的位置；控制用预估的位置
        s_now, e_now, he_now, _ = self.locate(pose.x, pose.y, pose.yaw, s)
        st.e, st.heading_err, st.s, st.kappa_ref = e_now, he_now, s_now, k_r
        constant_straight = self.constant_straight_speed and abs(k_r) < 1e-9
        constant_arc = self.constant_arc_speed_here()
        paint_window=self._active_paint_window(s_now,pose)
        self._prepare_entry_window(s_now,paint_window)
        if paint_window is not None:
            key,a,b=paint_window
            if self._paint_speed_key!=(key,a,b):
                self._paint_speed_key=(key,a,b)
                self._paint_speed_latched=self._planned_paint_speed(a,b)
            self.paint_target_mps=self._entry_paint_target(s_now,self._paint_speed_latched)
        else:
            self._paint_speed_key=self._paint_speed_latched=None
        constant_tracking = constant_straight or constant_arc or paint_window is not None
        st.integral_frozen = st.integral_frozen or (
            self.integral_start_m is not None and s_now < self.integral_start_m) or (
            self.integral_curve_from_m is not None and s_now < self.integral_curve_from_m
            and self.path._piece(s_now).straight)

        guided = self._straight_guided     # 直线 IMU 为主：上一拍的喷头横偏估计、车头（卫星跳了也按 IMU 推算）
        if guided is None:
            guided = self._arc_guided      # 画圆 IMU 为主：这一拍的
        off = abs(e_now) > cfg.abort_offset_m or abs(he_now) > cfg.abort_heading_rad
        if off and guided is not None:
            off = abs(guided[0]) > cfg.abort_offset_m or abs(guided[1]) > cfg.abort_heading_rad
        if off:
            st.state, st.reason = 'ABORTED', 'off_line'
            st.v_cmd, self._v_prev = 0.0, 0.0
            self._arc_tail_w = 0.0
            return 0.0, 0.0, 0.0
        remaining = self.path.length - s
        if cfg.stop_delay_s is not None and cfg.stop_delay_s < cfg.delay_s:
            # 停车（刹停曲线、到点）按车速指令的延迟预估：比转向的 delay_s 短，按 delay_s 会提前停、停短
            xs, ys, yaws = self.predict(pose.x, pose.y, pose.yaw, cfg.stop_delay_s)
            xs += pose.speed * cfg.lead_s * math.cos(yaws)
            ys += pose.speed * cfg.lead_s * math.sin(yaws)
            remaining = self.path.length - self.locate(xs, ys, yaws, s_now)[0]
        # 越线后已按斜率把指令降到蠕行档，就必须发零结束；不能一边持续发正速，
        # 一边等待实测速度低于阈值。低速反馈偏高时，这种等待会让车永远爬行。
        # 仍保留越线锁存、路径终点和已发减速指令三项约束，不在高速时直接发零。
        if paint_window is None and self.continuation is None and remaining <= cfg.stop_tol_m and (self.run_out_m <= 0 or self._run_out_started and self._v_prev <= cfg.min_mps):
            st.state, st.reason = 'DONE', ''
            st.v_cmd, self._v_prev = 0.0, 0.0
            self._arc_tail_w = 0.0
            return 0.0, 0.0, 0.0

        if self._creep_from is None:
            self._creep_from = s_now
        started = s_now - self._creep_from          # 这次起步后走了多远
        # 起步锁车头这段不积分：起步甩头是一下子的，积进去会在后面几米反向拖着偏
        if (self._straight_weight == 0.0 and not st.integral_frozen and abs(e) < cfg.slow_offset_m and started >= cfg.start_hold_m
                and not self._settling()):
            lim = cfg.integral_limit / cfg.ki_per_m2
            st.integral = max(-lim, min(lim, st.integral + e * ds))

        # 前馈：前方 ff_preview_s 处的路径曲率（曲率突变处按 ff_shape_* 整形），按横偏、航向误差修正
        k_pre = self.feedforward_kappa(s, pose.speed)
        k_ff = k_pre * math.cos(he) / max(0.5, 1.0 - k_pre * e)
        lateral_gain=self.k_e
        if self._arc_guided is not None and self._arc_status.get('e_rtk_raw') is not None:
            scale=self._partial_arc_feedback_scale() if self._arc_status.get('partial_arc') else cfg.arc_rtk_feedback_scale
            lateral_gain*=1.+self._arc_rtk_weight*(scale-1.)
        # 起步稳定段（launch_settle_*）先不拉横偏，只保持车头
        theta_ref = 0.0 if self._settling() else -math.atan(lateral_gain * e)
        theta_ref = max(-cfg.approach_max_rad, min(cfg.approach_max_rad, theta_ref))
        k_fb = -self.k_theta * (he - theta_ref) - cfg.ki_per_m2 * st.integral
        if self._arc_status.get('partial_arc'):
            # 同比例提高航向反馈与已学的积分补偿，保持原有零偏补偿，并随观测器平滑接入。
            k_fb*=1.+self._arc_weight*(cfg.arc_partial_heading_scale-1.)
        # 先把反馈限在"前馈之外还剩的曲率"内再低通（与直线控制器一样先限幅后滤波，滤波器里不会积出超限的量）
        k_max = self.kappa_act_max
        k_ff = max(-k_max, min(k_max, k_ff))
        k_fb = max(-k_max - k_ff, min(k_max - k_ff, k_fb))
        self._kfb += (k_fb - self._kfb) * dt / (cfg.kappa_tau_s + dt)
        kappa = max(-k_max, min(k_max, k_ff + self._kfb))
        # 车实际只走出下发曲率的 steer_gain，下发时补上
        kappa = st.kappa = max(-self.kappa_max, min(self.kappa_max, kappa / cfg.steer_gain))

        # 速度：巡航；前方弯道按刹车曲线减到横向加速度限速；偏差大降速；终点前刹停
        v = cfg.arc_speed_mps if constant_arc else cfg.cruise_mps
        if self.entry_speed_mps is not None and s < self.paint_from_m:
            # s 已包含执行延迟与前视；在进入划线区之前预留减速距离。
            entry_target=self._planned_paint_speed(*self._paint_windows()[0]) if self._paint_windows() else cfg.cruise_mps
            allowed = math.sqrt(entry_target**2 + 2*cfg.decel_mps2*max(0.,self.paint_from_m-s))
            v = min(self.entry_speed_mps,allowed)
        if self.approach_to_m is not None and s < self.approach_to_m:
            # 接入、转场离正式路线 approach_zone_m 以内不按转场速度：按刹车曲线降到画线速度（段短于它就不提速）
            allowed = math.sqrt(self.approach_mps**2
                                + 2*cfg.decel_mps2*max(0., self.approach_to_m-cfg.approach_zone_m-s))
            v = min(v, allowed)
        runup_mark=self._entry_hold_from if self._entry_speed_active else self.paint_from_m
        if (self.runup_speed_mps is not None and (not self._rolling_entry or self._entry_speed_active)
                and s_now < runup_mark):
            # 直线助跑在入圆切点结束；起喷前的沿圆预走已按圆弧速度运行。
            # 若以 paint_from_m 为终点，圆内会再次 0.5→0.3→0.5，积入转向暂态，起喷后仍在回摆。
            runup_to, exit_speed = runup_mark, cfg.cruise_mps
            paint_piece = self.path._piece(runup_mark)
            if paint_piece.straight:
                # 助跑平滑提到 0.5；跨过线头不触发提速，正式线上稳定走过 1 m 后才升到设定值。
                runup_to=0.
                if self._same_straight_run(self.path._piece(max(0.,s_now)),paint_piece):
                    v=min(v,exit_speed,self.entry_stabilize_mps if self.entry_stabilize_mps is not None else exit_speed)
            elif cfg.arc_speed_mps is not None:
                runup_to, exit_speed = paint_piece.s0, cfg.arc_speed_mps
            if s_now < runup_to and not constant_arc:
                target=min(exit_speed,self.runup_speed_mps)
                lead=max(0.,(exit_speed**2-target**2)/(2*cfg.accel_mps2)) + exit_speed*cfg.delay_s + .20
                if runup_to-s_now > lead:
                    zone=max(0.,runup_to-self.runup_distance_m)
                    allowed=math.sqrt(target**2+2*cfg.decel_mps2*max(0.,zone-s))
                    v=min(v,allowed)
                # 更远的接入直线仍按原巡航速度；只在最后一段助跑内限制到入圆速度。
                cap=exit_speed if runup_to-s_now <= self.runup_distance_m else cfg.cruise_mps
                v=min(v,cap)
        painting = any(a <= s_now < b for a,b in self.paint_spans) if self.paint_spans is not None else self.paint_from_m <= s_now < self.paint_to_m
        if not constant_tracking and self.paint_offset_limit_m is not None and painting:
            # 正式直线喷涂：当前/延迟后喷头偏差达到停喷门槛一半就渐降速，
            # 达到门槛时最多 0.45m/s 继续纠偏。保持正速度，不以小横偏暂停任务。
            nx, ny = self._straight_nozzle
            current = e_now+(nx-cfg.track_x_m)*math.sin(he_now)+ny*(math.cos(he_now)-1.)
            predicted = e+(nx-cfg.track_x_m)*math.sin(he)+ny*(math.cos(he)-1.)
            if guided is not None:
                current = predicted = guided[0]
            ratio = min(1., max(0., 2.*max(abs(current),abs(predicted))/self.paint_offset_limit_m-1.))
            v = min(v, cfg.cruise_mps-(cfg.cruise_mps-min(cfg.cruise_mps,.45))*ratio)
        slow_e = e if guided is None else guided[0]
        if not constant_tracking and (self.paint_spans is None or painting) and abs(slow_e) > cfg.slow_offset_m:
            v *= max(0.4, 1.0 - (abs(slow_e) - cfg.slow_offset_m) / 0.3)
        if paint_window is not None or self.continuation is not None:
            pass  # 正式喷线不提前刹车；连续段完成后只停喷，保持前进。
        elif self.run_out_m > 0:
            # 线尾前完全不参与停车限速。只用当前实际喷头位置触发，禁止 Smith 前视触发。
            nozzle_remaining,_=self.nozzle_end(pose,*self._straight_nozzle,at=self.paint_to_m)
            if s_now>=self.paint_to_m-.05 and nozzle_remaining < 0.:
                # 闭合路线起点与终点相邻，转场也可能穿过终点所在直线；仅在最后线尾附近判断真实越线。
                self._run_out_started=True
            if self._run_out_started:
                # 越线后按恒定斜率降低非零指令；不靠提前刹车或高速发零追停车点。
                v=min(v,max(cfg.min_mps,self._v_prev-cfg.run_out_decel_max_mps2*dt))
        else:
            v = min(v, max(cfg.min_mps, math.sqrt(2.0 * cfg.decel_mps2 * max(0.0, remaining - cfg.stop_tol_m))))
        if not constant_tracking and cfg.start_creep_mps > 0 and started < cfg.start_creep_m:
            v = min(v, cfg.start_creep_mps)
        if self._settling():
            # 起步稳定段（launch_settle_*）：稳住之前车速目标不超过 launch_settle_mps，由下面的加速斜坡平顺收住
            v = min(v, cfg.launch_settle_mps)
        if self.constant_paint_speed:
            if paint_window is not None:
                # 覆盖正常的偏差、曲率预瞄、段尾、慢行限速；硬停车/保护约束仍在下面优先执行。
                v=self.paint_target_mps
            else:
                upcoming=next(((a,b) for a,b in self._paint_windows() if a>s_now),None)
                if upcoming is not None:
                    a,b=upcoming;target=self._planned_paint_speed(a,b)
                    v=min(v,math.sqrt(target*target+2*cfg.decel_mps2*max(0.,a-s)))
        # 弯前规划必须在喷线定速之后执行，助跑、正式喷线、转场均不能覆盖它。
        short_runup=self._short_runup_here(s_now)
        if short_runup:v=min(v,TURN_SPEED_MPS)
        requested=v
        curve_limit=self._curve_speed(s,max(v,abs(pose.speed)))
        current_piece=self.path._piece(max(0.,min(self.path.length,s_now)))
        if paint_window is not None and self.cfg.arc_speed_mps is not None and self._work_arc_piece(current_piece):
            # 正式画圆/画弧线保持作业定速；停喷后的转场再平缓收至掉头速度。
            curve_limit=self.cfg.arc_speed_mps
        if (self.geo.differential and not current_piece.straight
                and not self._straight_stitch(self.path.pieces,self.path.pieces.index(current_piece))):
            curve_limit=min(curve_limit,self._curve_target_at(max(0.,s_now)))
        curve_adjusted=curve_limit<requested-1e-6 or self._v_prev>curve_limit+1e-6
        if curve_adjusted and not self._run_out_started:self._curve_speed_active=True
        curve_profile=self._curve_speed_active and not self._run_out_started
        v=min(v,curve_limit)
        protected=False
        if self.speed_limit_mps is not None and (self.constant_paint_speed or not constant_arc or self.speed_limit_mps <= 0.):
            protected=self.speed_limit_mps<v-1e-6
            v = min(v, max(0.0, self.speed_limit_mps))
        v = self._accelerate(v, dt,smooth_decel=(curve_profile or short_runup) and not protected)
        if protected or (not curve_adjusted and abs(v-requested)<=1e-6 and abs(self._accel_prev)<=1e-6):
            self._curve_speed_active=False
        self._v_prev = v
        if v<=0.:self._accel_prev=0.
        st.v_cmd = v
        self.paint_expected_mps=self.paint_target_mps
        if (paint_window is not None and not protected and (
                curve_profile and v>=min(TURN_SPEED_MPS,self.paint_target_mps)-1e-6 or
                self._entry_speed_active and self._entry_ramp_released
                and v>=min(self.entry_stabilize_mps,self.paint_target_mps)-1e-6)):
            # 速度计划诊断；起喷资格不依赖此值。
            self.paint_expected_mps=v
        self.paint_speed_ready=(self.paint_expected_mps is not None and not protected
                                and abs(v-self.paint_expected_mps)<=1e-6)
        if (paint_window is not None and self._entry_ramp_released and not protected
                and abs(v-self._paint_speed_latched)<=1e-6):
            self._entry_speed_active=False
        w_hold = 0.0
        if cfg.start_hold_m > 0 and started < cfg.start_hold_m:
            # 起步锁车头：曲率反馈的车头纠正速度是 车速 × k_theta，不到 start_hold_rate 的差额直接给角速度
            # （车头误差用执行延迟之后的预估，与曲率反馈同一个参考方向）
            gain = max(0.0, cfg.start_hold_rate - v * self.k_theta)
            w_hold = max(-cfg.start_hold_max, min(cfg.start_hold_max, -gain * (he - theta_ref)))

        # 角弧尾端降速后，v*k_theta 的航向纠偏随之变弱；侧置喷头仍会被几度航向误差拉离目标弧。
        # 仅在正式刷弧的最后 0.5 m 补足到 1.4/s，附加量限 0.06 rad/s，并沿用反馈低通平滑进入。
        mark = self.arc_tail_heading_from_m
        target_w = 0.0
        if (self.continuation is None and mark is not None and s_now >= mark and self.path.length-s_now <= .5
                and started >= cfg.start_hold_m and v > 1e-6):
            gain = max(0.0, 1.4-v*self.k_theta)
            target_w = max(-.06, min(.06, -gain*(he-theta_ref)))
        if mark is not None and v > 1e-6:
            self._arc_tail_w += (target_w-self._arc_tail_w)*dt/(max(.15,cfg.kappa_tau_s)+dt)
            # 保留原前馈/反馈，附加纠偏只使用原曲率上限中尚未占用的余量。
            base_w = v*kappa+w_hold
            limit = v*self.kappa_max
            added = (min(self._arc_tail_w, max(0.0,limit-base_w)) if self._arc_tail_w >= 0
                     else max(self._arc_tail_w, min(0.0,-limit-base_w)))
            self._arc_tail_w = added
            w_hold += added
        else:
            self._arc_tail_w = 0.0

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
                    integral=st.integral, trim_kappa=-self.cfg.ki_per_m2 * st.integral, stats=dict(st.stats),
                    integral_frozen=st.integral_frozen, straight_hold=dict(self._straight_status),
                    arc_hold=dict(self._arc_status), launch_settle=dict(self._settle) if self._settle else None)


# ---- 前进 / 倒车交替的分段路径 ----

def paint_window(section, s, tail_extra=0.):
    """当前或下一喷线区间；区间号用于截止锁存，空隙保持不喷。"""
    spans=getattr(section,'paint_spans',None)
    if spans:
        for i,(a,b) in enumerate(spans):
            if s < b+tail_extra:return i,a,b
        return len(spans)-1,*spans[-1]
    end=getattr(section,'paint_to_m',None)
    return 0,getattr(section,'paint_from_m',0.),section.path.length if end is None else end


class Section:
    """分段路径里的一段：direction=+1 前进、−1 倒车。path 按行进方向画：倒车段的 yaw 是车头 + π，曲率也按行进方向。"""

    def __init__(self, direction, path):
        if direction not in (1, -1):
            raise ValueError('方向只能是 +1 前进或 −1 倒车')
        self.direction, self.path = direction, path
        self.paint_to_m = None
        self.paint_spans = None
        self.continuous_after = False

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
    _source: object = field(default=None, repr=False, compare=False)

    def __getattr__(self, name):
        # 坐标变换只改运动学字段，位置 / 测向质量和待接回校正沿用原位姿。
        if self._source is None:
            raise AttributeError(name)
        return getattr(self._source, name)


class SectionTracker:
    """分段路径跟踪：每段一个 PathTracker；段尾停稳（车速 < still_mps 持续 settle_s）后换向进下一段。

    接口与 PathTracker 相同（start / check_start / update / stop / status，st 汇总全程），line_mission 两种都能用。
    积分（舵机零位）按前进坐标保存：倒车坐标里曲率反号，零位偏差也反号，换段时跟着变号。
    """

    def __init__(self, sections, config=PathConfig(), geometry=None, *, reverse_mps=0.3, reverse_gain=0.90,
                 still_mps=0.02, settle_s=0.3, switch_timeout_s=5.0, pivot_rate=None, pivot_accel=1.0,
                 pivot_tol=math.radians(1.0), pivot_timeout_s=20.0, pivot_stop_tol=0.005,
                 pivot_min=math.radians(5.0), pivot_retries=2, pivot_floor=0.1, paint_offset_limit_m=None,
                 constant_straight_speed=False, reverse_runup=False, constant_paint_speed=False):
        """pivot_rate 不为 None（差速车）：下一段起点车头方向与车差 pivot_min 以上时，停稳后原地转过去再开，
        起步时也一样；角速度不超过 pivot_rate、角加速度不超过 pivot_accel。阿克曼车为 None，不原地转。
        原地转仍按 pivot_tol 瞄准；转完差 pivot_min 以内不补转，直接慢速往前开，万向轮回正、车头由跟踪纠正
        （2026-09-30 实车：转完一停车头回弹约 1°，按 1° 补转时小指令推不动，反复补转卡住 27 s）。
        转完仍差 pivot_min 以上最多补转 pivot_retries 次，再不行按 pivot_timeout 中止。没转到位时角速度不低于
        pivot_floor（rad/s，0.1 约轮子 33 erpm；实车 ±5–19 erpm 收尾磨 1.6 s，±14 erpm 补转推不动）。"""
        self.sections = list(sections)
        if type(constant_straight_speed) is not bool:
            raise ValueError('直线定速策略必须是布尔值')
        self.constant_straight_speed = constant_straight_speed
        if type(constant_paint_speed) is not bool:raise ValueError('喷线定速策略必须是布尔值')
        self.constant_paint_speed=constant_paint_speed
        # 普通路线（不是场地作业）的接入：倒车接入后的第一段喷线段起步锁车头 REVERSE_HOLD_M；路线中间拐角的倒车照旧
        self.reverse_runup = bool(reverse_runup)
        self._first_paint = next((k for k, s in enumerate(self.sections) if getattr(s, 'field_role', None) == 'paint'),
                                 len(self.sections))
        if paint_offset_limit_m is not None and (not math.isfinite(paint_offset_limit_m) or paint_offset_limit_m <= 0):
            raise ValueError('喷头偏差门槛必须为有限正数')
        self.speed_limit_mps = None
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
            reverse_straight = (sec.direction < 0 and self.geo.differential and config.straight_imu_enabled
                                and all(p.straight for p in sec.path.pieces))
            # 「离终点太近不起步」只对整条路线的最后一段有意义；后面还有段的（如 0.3–0.9 m 的接入、倒车助跑段）
            # 照样起步，否则短接入段一起步就报「已经过了路线终点」（2026-10-01 实车）
            cfg = config if len(self.sections) == 1 else \
                replace(config, start_min_left_m=min(config.start_min_left_m, config.stop_tol_m))
            if i > 0 and sec.direction > 0 > self.sections[i - 1].direction:
                cfg = replace(cfg, start_creep_m=max(cfg.start_creep_m, cfg.reverse_creep_m),
                              start_creep_mps=cfg.reverse_creep_mps or cfg.start_creep_mps)
            if getattr(sec,'cruise_mps',None) is not None:
                if not math.isfinite(sec.cruise_mps) or not .02<=sec.cruise_mps<=MAX_SECTION_SPEED:raise ValueError('分段速度无效')
                cfg=replace(cfg,cruise_mps=sec.cruise_mps)
            mark=getattr(sec,'paint_from_m',0.)
            end=sec.path.length if sec.paint_to_m is None else sec.paint_to_m
            spans=sec.paint_spans if sec.paint_spans is not None else [(mark,end)]
            if (self.geo.differential and sec.direction>0 and getattr(sec,'field_role',None)=='paint'
                    and cfg.arc_speed_mps is not None
                    and spans and all(any(not p.straight and p.s0<b-1e-8 and p.s0+p.length>a+1e-8
                                          for p in sec.path.pieces) for a,b in spans)):
                cfg=replace(cfg,cruise_mps=cfg.arc_speed_mps)
            if pivot_rate is not None:
                # 差速车段尾要停准：原地转 90° 后，停车差的那点前后距离就成了下一条边上的横偏。
                # 最后一段也一样：连线路线终点不再多走（route_plan.END_PASS_M），水泵要正好停在定点上（2026-10-03 用户）；
                # 原来按 2 cm 容差停、再靠多走的 5 cm 盖过终点，实车水泵停在定点后 6–11 cm
                cfg = replace(cfg, stop_tol_m=min(cfg.stop_tol_m, pivot_stop_tol))
            if sec.direction < 0:
                # 倒车按行进方向换算控制点；差速车传入增益为 1，阿克曼车沿用其倒车补偿。
                cfg = replace(cfg, cruise_mps=min(cfg.cruise_mps, reverse_mps), track_x_m=-cfg.track_x_m,
                              steer_gain=reverse_gain, straight_imu_enabled=reverse_straight,
                              arc_imu_first=False, arc_speed_mps=None)
                if reverse_straight:
                    # 10-08 三趟倒车中段的周期响应约放大 3 倍、滞后约 0.2 s。
                    # 共享内核但使用较缓的倒车参数，直接按实际角速度阻尼，覆盖响应失配。
                    cfg=replace(cfg,straight_kp=1.2,straight_ki=.4,straight_kd=.15,
                                straight_prediction_delay_s=.30,straight_direct_rate_damping=True)
                if cfg.reverse_section_creep_m is not None:
                    cfg = replace(cfg, start_creep_m=cfg.reverse_section_creep_m)
            # 前进喷线直线与圆弧、连续转场共用柔和起步。
            if self.geo.differential and sec.direction>0 and (
                    getattr(sec,'field_role',None)=='paint'
                    or any(not p.straight for p in sec.path.pieces) or sec.continuous_after
                    or i>0 and self.sections[i-1].continuous_after):
                cfg=replace(cfg,accel_mps2=min(cfg.accel_mps2,.25),accel_jerk_mps3=.5)
            tracker=PathTracker(sec.path, cfg, self.geo)
            tracker.reverse_straight=reverse_straight
            if reverse_straight:tracker.set_nozzle(cfg.track_x_m,0.)
            tracker._arc_work_role=getattr(sec,'field_role',None)!='transit'
            tracker.constant_straight_speed = constant_straight_speed and self.geo.differential
            mark=getattr(sec,'paint_from_m',0.)
            end = sec.path.length if sec.paint_to_m is None else sec.paint_to_m
            if not math.isfinite(end) or not 0 <= mark <= end <= sec.path.length:
                raise ValueError('划线范围无效')
            tracker.paint_from_m, tracker.paint_to_m = mark, end
            tracker.constant_paint_speed=(constant_paint_speed and self.geo.differential and sec.direction>0
                                          and getattr(sec,'field_role',None)=='paint')
            if tracker.constant_paint_speed and sec.paint_to_m is None:
                raise ValueError('旧路线缺少停喷终点，请重新规划后作业')
            if tracker.constant_paint_speed and not sec.continuous_after and sec.path.length-end<.10:
                raise ValueError('喷线后缺少不喷缓停段，请重新规划后作业')
            tracker.paint_spans = sec.paint_spans
            if tracker.paint_spans is None and getattr(sec,'field_role',None)=='paint':
                tracker.paint_spans=[(mark,end)]
            if (self.geo.differential and sec.direction>0 and getattr(sec,'field_role',None)=='paint'
                    and end < sec.path.length and all(p.straight for p in sec.path.pieces if p.s0+p.length>end+1e-9)):
                from route_plan import SOLO_CIRCLE_EXIT_M
                paint_tail=sec.path._piece(max(mark,end-1e-8))
                circle_exit=(not paint_tail.straight
                    and abs(paint_tail.kappa)*max(0.,end-max(mark,paint_tail.s0))>=2*math.pi-1e-3
                    and abs(sec.path.length-end-SOLO_CIRCLE_EXIT_M)<1e-7)
                # 仅整圆的 1 m 切线离开段按剩余距离自然减速，避免整米低速爬行；其他线尾沿用原策略。
                if not circle_exit:
                    tracker.run_out_m = sec.path.length-end
            if (constant_straight_speed and self.geo.differential and sec.direction>0
                    and getattr(sec,'field_role',None)=='paint' and mark>0):
                # 10-05 用户定助跑 0.5 m/s、正常不低于 0.3；10-06 晚用户：「起步的助跑是允许降速的，在进入的直线的
                # 时候必须直」→ 0.3（仿真线头 2 m 弯中位 0.55 → 0.37–0.53 cm，到线头仍 0.8 m/s，每次起步多约 0.5 s）
                tracker.runup_speed_mps=.3
                if tracker.path._piece(mark).straight:
                    tracker.runup_start_mps=.35
                    tracker.entry_stabilize_mps=.5
                    tracker.cfg=replace(tracker.cfg,accel_mps2=min(tracker.cfg.accel_mps2,.20))
                else:
                    # 10-08 用户：圆前切线助跑由 0.3 提到 0.35 m/s，入圆仍平滑接到圆弧速度。
                    tracker.runup_speed_mps=.35
                from route_plan import RUN_UP_M
                tracker.runup_distance_m=RUN_UP_M
                if self.reverse_runup and 0 < i == self._first_paint and self.sections[i-1].direction < 0:
                    # 倒车接入后的助跑：保留起步锁车头距离，航向收至 1° 后才交接（横偏仍用原门槛）。
                    tracker.reverse_runup = True
                    tracker.cfg=replace(tracker.cfg,start_hold_m=max(tracker.cfg.start_hold_m,REVERSE_HOLD_M),
                                        runup_release_heading_deg=1.0)
                tracker.paint_from_m=mark
                tracker.cfg=replace(tracker.cfg,straight_settle_m=min(tracker.cfg.straight_settle_m,
                                    max(tracker.cfg.start_hold_m,mark-1.0)))
                if (tracker.cfg.straight_imu_enabled and tracker.cfg.straight_imu_first is not None
                        and tracker.cfg.straight_imu_first.rtk_fit
                        and tracker.path._piece(mark).straight):
                    tracker.paint_start_guard=PaintStartGuard()
                    # 渐入 IMU 与允许开喷分开判断。先在助跑内平滑交接，开喷仍需上面的 2 cm / 1° 检查。
                    tracker.cfg=replace(tracker.cfg,
                        runup_release_e_m=max(tracker.cfg.runup_release_e_m,.03),
                        runup_release_heading_deg=max(tracker.cfg.runup_release_heading_deg,2.0))

            if (self.geo.differential and sec.direction>0 and getattr(sec,'field_role',None)=='paint'
                    and all(p.straight for p in sec.path.pieces)):
                tracker.paint_offset_limit_m=paint_offset_limit_m
                tracker.paint_from_m=mark
            last=sec.path.pieces[-1]
            # 最后一段非整圆刷弧才启用：实际刷弧角度扣除助跑，整圆、直线、倒车与转场均不参与。
            if (self.geo.differential and sec.direction>0 and getattr(sec,'field_role',None)=='paint'
                    and not last.straight):
                if not math.isfinite(mark) or not 0<=mark<=sec.path.length:
                    raise ValueError('圆弧划线起点无效')
                arc_from=max(mark,last.s0)
                sweep=abs(last.kappa)*(sec.path.length-arc_from)
                if 1e-6<sweep<2*math.pi-1e-3:
                    tracker.arc_tail_heading_from_m=arc_from
            entry_speed=getattr(sec,'entry_speed_mps',None)
            if entry_speed is not None:
                mark=getattr(sec,'paint_from_m',0.)
                if (sec.direction<0 or getattr(sec,'field_role',None)!='paint'
                        or not math.isfinite(entry_speed) or not .02<=entry_speed<=MAX_SECTION_SPEED
                        or not math.isfinite(mark) or not 0<=mark<=sec.path.length):
                    raise ValueError('接入速度或划线起点无效')
                tracker.entry_speed_mps,tracker.paint_from_m=entry_speed,mark
            if self._spray_straight(sec, tracker):
                mark = getattr(sec, 'paint_from_m', 0.0)
                if not math.isfinite(mark) or not 0 <= mark <= sec.path.length:
                    raise ValueError('长直线划线起点无效')
                tracker.integral_start_m = mark
            elif self._paint_curve(sec, tracker):
                # 2026-10-11 用户「补上」：进弧助跑的直线部分只纠偏、不学积分（起步车头差 4–5° 带开的暂时偏差积进去，进圆后反向
                # 冲过头，仿真 R2 圆线头 2.3 cm）；进到圆上照常学，补底盘小半径转不够（10-08 实测 R1 弧只转出指令的 79%）
                mark = getattr(sec, 'paint_from_m', 0.0)
                if math.isfinite(mark) and 0 < mark <= sec.path.length:
                    tracker.integral_curve_from_m = mark
            self.trackers.append(tracker)
        self._set_continuations()
        self._set_approach()
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
        self._pivot_direction = 1.0
        self._pivot_remaining = self._pivot_last_yaw = 0.0
        self._pivot_braking = self._pivot_slowing = False
        self._pivot_rate_excess = 0.0
        self._pivot_delay_s = self.cfg.delay_s+.025 if self.cfg.delay_s>0 else 0.0
        self._turns = 0                 # 这次起步 / 换段已经原地转了几次（补转计数）
        self._turn_clock = 0.0
        self._turn_sent = deque()      # 原地转向预估也记录零速：暂停、停稳等待时旧指令仍可能在执行
        self._still = self._wait = 0.0
        self._integral = 0.0            # 前进坐标
        self._paint_integral = None     # 只缓存实际喷线长直线学到的值，不被转场/倒车覆盖
        self._done = [0.0, 0, 0.0]      # 已跑完各段的 横偏平方和、样本数、最大横偏
        self.st = PathState(length=total)

    def _set_continuations(self):
        for i,sec in enumerate(self.sections):
            if type(sec.continuous_after) is not bool:raise ValueError('连续前进标记必须为布尔值')
            if not sec.continuous_after:continue
            if not self.geo.differential or i+1>=len(self.sections):raise ValueError('连续前进缺少可衔接的下一段')
            nxt=self.sections[i+1]
            end,start=sec.car_pose(sec.path.length),nxt.car_pose(0.)
            if (sec.direction!=1 or nxt.direction!=1 or getattr(sec,'pause_after',None)
                    or math.dist(end[:2],start[:2])>.001 or abs(wrap(end[2]-start[2]))>1e-4):
                raise ValueError('连续前进要求位置与方向相切，且没有换向或检查暂停')
            self.trackers[i].continuation=self.trackers[i+1]
            self.trackers[i+1].predecessor=self.trackers[i]
        for tr in self.trackers:
            if tr.continuation is None:continue
            # 覆盖最高 1.5 m/s 的制动/延迟预瞄，不复制整场数千米路线。
            pieces=[(p.length,p.kappa) for p in tr.path.pieces]
            reach=max(10.,tr._turn_ramp_distance(MAX_SECTION_SPEED,0.)+MAX_SECTION_SPEED*(tr.cfg.delay_s+tr.cfg.lead_s)+1.)
            extra=0.;nxt=tr.continuation
            while nxt is not None and extra<reach:
                for p in nxt.path.pieces:
                    length=min(p.length,reach-extra)
                    if length<=1e-9:break
                    pieces.append((length,p.kappa));extra+=length
                nxt=nxt.continuation
            tr._preview_path=Path(tr.path.pose(0.),pieces)

    def _roll_forward(self, pose):
        """实际车位跨过分界才换喷线窗口；控制预瞄可以提前跨界，二者不能混用。"""
        tr=self.trackers[self.index]
        while tr.continuation is not None:
            s,_,_,_=tr.locate(pose.x,pose.y,pose.yaw,min(tr.path.length,tr._hint or 0.))
            if s<tr.path.length-1e-7:break
            nxt=tr.continuation
            if not nxt.start(pose,integral=tr.st.integral):
                tr.pause(nxt.st.reason)
                self.st.state,self.st.reason='PAUSED',nxt.st.reason
                return False
            nxt._v_prev,nxt._accel_prev=tr._v_prev,tr._accel_prev
            nxt._curve_speed_active=tr._curve_speed_active
            # 已按下一笔作业速度连续直行时无需重新降速；从 0.5 圆弧接入更快直线仍须稳行 1 m。
            # 不能仅因“连续衔接”就跳过下一条直线的接入稳定阶段。
            if (nxt._entry_speed_active and tr._v_prev>=
                    nxt._planned_paint_speed(nxt.paint_from_m,nxt.paint_to_m)-1e-6):
                nxt._entry_speed_active=False
            nxt._clock,nxt._sent=tr._clock,deque(tr._sent)
            nxt._kfb=tr._kfb
            nxt._creep_from=(tr._creep_from if tr._creep_from is not None else 0.)-tr.path.length
            nxt._rolling_entry=True
            self._close(tr)
            tr.st.state='DONE'
            self.index+=1
            tr=nxt
        return True

    def _view(self, sec, pose):
        if sec.direction > 0:
            return pose
        return _Travel(pose.x, pose.y, wrap(pose.yaw + math.pi), -pose.speed, pose.yaw_rate,
                       pose.usable_for_control, _source=pose)

    def sync(self, now, imu, lever=None):
        """倒车按行进方向换算航向和杆臂；角速度方向不变。原地转向由外层控制。"""
        for sec,tracker in zip(self.sections,self.trackers):
            reverse=sec.direction<0
            value=imu
            if reverse and imu is not None:
                try:
                    t,yaw,rate=imu
                    value=(t,wrap(yaw+math.pi),rate)
                except (TypeError,ValueError):
                    pass  # 由 PathTracker.sync 按原规则拒绝无效输入。
            arm=None if lever is None else tuple(-v for v in lever) if reverse else lever
            tracker.sync(now,value,arm)

    def set_heading_baseline(self, t, length_m):
        for tracker in self.trackers:
            tracker.set_heading_baseline(t, length_m)

    def set_rtk_quality(self, t, sigma_m, sats, pair_m=None, fixed=None):
        for tracker in self.trackers:
            tracker.set_rtk_quality(t, sigma_m, sats, pair_m, fixed)

    def set_secondary_fix(self, t, x, y, fixed, lever):
        for sec,tracker in zip(self.sections,self.trackers):
            arm=tuple(-v for v in lever) if sec.direction<0 and lever is not None else lever
            tracker.set_secondary_fix(t, x, y, fixed, arm)

    def set_primary_fix(self, t, x, y, fixed, lever):
        for sec,tracker in zip(self.sections,self.trackers):
            arm=tuple(-v for v in lever) if sec.direction<0 and lever is not None else lever
            tracker.set_primary_fix(t,x,y,fixed,arm)

    def position_correction(self):
        return self.trackers[self.index].position_correction() if 0 <= self.index < len(self.trackers) else None

    def share_imu_alignment(self, alignment):
        for sec,tracker in zip(self.sections,self.trackers):
            # 短倒车独立学习参考，不把倒车暂态标定写回正式喷线的跨线参考。
            tracker.share_imu_alignment(alignment if sec.direction>0 else None)

    def set_nozzle(self, forward, left=0.0):
        for tracker in self.trackers:
            if tracker.reverse_straight:tracker.set_nozzle(tracker.cfg.track_x_m,0.)
            else:tracker.set_nozzle(forward,left)

    def check_start(self, pose, hint=None):
        i = self.index if self.st.state == 'PAUSED' else 0
        if i == 0 and self.st.state != 'PAUSED' and self.pivot_rate is not None and pose.usable_for_control \
                and self._misaligned(0, pose):
            # 差速车起步先原地转向：车头方向不算，只看位置在不在第一段起点附近
            pose = _Travel(pose.x, pose.y, self._heading(0), pose.speed, pose.yaw_rate, True, _source=pose)
        return self.trackers[i].check_start(self._view(self.sections[i], pose), hint)

    def start(self, pose, integral=None, *, preserve_startup=False):
        st = self.st
        if st.state == 'PAUSED':
            if self.switching:                  # 换向途中定位丢过：恢复后接着等停稳、换向
                if not pose.usable_for_control:
                    st.reason = 'pose_unusable'
                    return False
                st.state, st.reason = 'RUNNING', ''
                if self._pivot is not None:
                    # 暂停后旧转向过程已中断；等实测停稳再按当前航向重建同一次尝试。
                    self._pivot, self._w, self._still, self._wait = None, 0.0, 0.0, 0.0
                    self._turns = max(0, self._turns-1)
                return True
            tr = self.trackers[self.index]
            options={'preserve_startup':True} if preserve_startup else {}
            ok = tr.start(self._view(self.sections[self.index], pose),**options)
            st.state, st.reason = ('RUNNING', '') if ok else ('PAUSED', tr.st.reason)
            return ok
        if integral is not None:
            self._integral = integral
        self._paint_integral = integral
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
        while len(self._turn_sent) >= 2 and self._turn_sent[1][0] <= self._turn_clock - self._pivot_delay_s + 1e-9:
            self._turn_sent.popleft()
        return output

    def _update(self, pose, dt):
        st = self.st
        if st.state not in ('RUNNING', 'PAUSED'):
            st.v_cmd = 0.0
            return 0.0, 0.0
        if self.switching:
            return self._switch(pose, dt)
        if st.state=='RUNNING' and pose.usable_for_control and not self._roll_forward(pose):
            st.v_cmd=0.
            return 0.,0.
        sec, tr = self.sections[self.index], self.trackers[self.index]
        tr.speed_limit_mps = self.speed_limit_mps
        v, w = tr.update(self._view(sec, pose), dt)
        if (self._spray_straight(sec, tr) and tr.st.state == 'RUNNING'
                and not tr.st.integral_frozen and tr._straight_weight > 0
                and pose.speed >= tr.cfg.straight_min_speed_mps):
            self._paint_integral = tr.st.integral
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
        still = stationary(pose,self.still_mps,yaw_limit=.05 if self.pivot_rate is not None else math.inf)
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
            self._pivot_remaining = wrap(self._pivot-pose.yaw)
            self._pivot_last_yaw = pose.yaw
            self._pivot_direction = math.copysign(1.0,self._pivot_remaining)
            self._pivot_braking = self._pivot_slowing = False
            self._pivot_rate_excess = 0.0
            return self._turn(pose, dt)
        sec, tr = self.sections[i], self.trackers[i]
        # 新接入和倒车后的暂态不是喷线稳态偏置；长直线只接续已在线上学到的积分。
        seed = self._integral
        if i > 0 and self._spray_straight(sec, tr):
            seed = self._paint_integral if self._paint_integral is not None else 0.0
        elif i > 0 and self._paint_curve(sec, tr):
            # 喷线圆弧段（整圆、角球弧、罚球弧，2026-10-11 用户「先做 1」）停车后重新起步也不接上一段的积分：上一段多是半径
            # 0.5 m 上下的转场急弯，积分是那里的转向补偿；带进来在进弧前的直线助跑上就是一个固定的拐弯偏置，比例纠偏顶住它，
            # 喷头被推开 2–3 cm，开喷时还偏 2.2 cm（仿真整场最差的角球弧线头 2.4 cm 即此）。从 0 开始，进弧助跑里照常重新积分。
            # 仿真整场 3 场：角球弧 12 个线头中位 1.06 → 0.78、最差 2.41 → 1.30 cm；罚球弧、中圈、直线、8 字不变
            seed = 0.0
        if not tr.start(self._view(sec, pose), integral=sec.direction * seed):
            st.state, st.reason = 'ABORTED', 'cusp_' + tr.st.reason
            return 0.0, 0.0
        self._integral = seed
        self.index, self.switching = i, False
        self._sync()
        return 0.0, 0.0

    def _set_approach(self):
        """前进的接入 / 转场段（以及喷线段里线头前那截）离正式路线 approach_zone_m 内按画线速度（PathConfig.approach_zone_m）。
        正式路线 = 喷线起点：本段是喷线段就是本段 paint_from_m；转场段紧接前进喷线段时是段尾 + 下一段的 paint_from_m，
        否则（中间要倒车、原地转后再上）按本段段尾算。画线速度取后面最近一个喷线段的速度。"""
        roles = [getattr(sec, 'field_role', None) for sec in self.sections]
        for i, (sec, tr) in enumerate(zip(self.sections, self.trackers)):
            if sec.direction < 0:
                continue
            if roles[i] == 'paint':
                to, speed = max(0., getattr(sec, 'paint_from_m', 0.) or 0.), tr.cfg.cruise_mps
                fast = tr.entry_speed_mps if tr.entry_speed_mps is not None else tr.cfg.cruise_mps
            elif roles[i] == 'transit':
                to = sec.path.length
                nxt = self.sections[i+1] if i+1 < len(self.sections) else None
                if nxt is not None and nxt.direction > 0 and roles[i+1] == 'paint':
                    to += max(0., getattr(nxt, 'paint_from_m', 0.) or 0.)
                speed = next((t.cfg.cruise_mps for r, t in zip(roles[i+1:], self.trackers[i+1:]) if r == 'paint'), None)
                fast = tr.cfg.cruise_mps
            else:
                continue
            if to > 1e-9 and speed is not None and speed < fast - 1e-9:
                tr.approach_to_m, tr.approach_mps = to, speed

    def _heading(self, i):
        """第 i 段起点的车头方向。"""
        return self.sections[i].car_pose(0.0)[2]

    def constant_speed_here(self):
        """直线巡航按段设定速度；原地转、曲线和到点刹停由原控制器处理。"""
        if not self.constant_straight_speed or not self.geo.differential or self.switching:
            return False
        tr = self.trackers[self.index]
        return abs(tr.path.kappa(max(0., min(tr.path.length, tr.st.s)))) < 1e-9

    def constant_arc_speed_here(self):
        if not self.geo.differential or self.switching or self._pivot is not None:
            return False
        return self.sections[self.index].direction > 0 and self.trackers[self.index].constant_arc_speed_here()

    def _spray_straight(self, sec, tracker):
        """仅前进长直线喷线启用积分隔离；圆、短线、转场和旧底盘继续原控制。"""
        return (self.geo.differential and tracker.cfg.straight_imu_enabled and sec.direction > 0
                and getattr(sec, 'field_role', None) == 'paint'
                and sec.path.length >= tracker.cfg.straight_min_length_m
                and all(piece.straight for piece in sec.path.pieces))

    def _paint_curve(self, sec, tracker):
        """差速车前进的喷线段里带弯（整圆、作业圆弧连同它们的助跑）：停车后重新起步时积分从 0 开始，不接转场的。"""
        return (self.geo.differential and tracker.cfg.straight_imu_enabled and sec.direction > 0
                and getattr(sec, 'field_role', None) == 'paint'
                and not all(piece.straight for piece in sec.path.pieces))

    def _misaligned(self, i, pose):
        """车头与第 i 段起点方向差 pivot_min 以上，要原地转；以内直接开，由跟踪纠正。"""
        return abs(wrap(self._heading(i) - pose.yaw)) > self.pivot_min

    def _turn(self, pose, dt):
        """原地转向每次只沿一个方向连续转动、减速收尾；实测停稳后才判断是否需要补转。"""
        st = self.st
        self._pivot_t += dt
        if self._pivot_t > self.pivot_timeout_s:
            st.state, st.reason = 'ABORTED', 'pivot_timeout'
            self._pivot, self._w = None, 0.0
            return 0.0, 0.0
        # 与 PathTracker.predict 相同：剩余角度按队列里的角速度全部执行完之后的车头算。
        # 另留 25 ms 执行器响应余量；末段按 1.8/s 收敛，缩短低速磨角度的时间。
        # 最高转速和角加速度不变；仍须经过 _switch 的实测停稳和最终航向检查。
        self._pivot_remaining -= wrap(pose.yaw-self._pivot_last_yaw)
        self._pivot_last_yaw = pose.yaw
        ahead = 0.0
        start = self._turn_clock - self._pivot_delay_s
        sent = self._turn_sent
        for i, (stamp, rate) in enumerate(sent):
            end = sent[i + 1][0] if i + 1 < len(sent) else self._turn_clock
            ahead += rate * self.cfg.steer_gain * max(0.0, end - max(start, stamp))
        # 指令延迟模型低估实车超调时，以新鲜实测转速补充制动预估；低通过滤单拍抖动。
        expected = sent[0][1]*self.cfg.steer_gain if sent else 0.0
        excess = max(0.0,self._pivot_direction*(pose.yaw_rate-expected))
        self._pivot_rate_excess += (excess-self._pivot_rate_excess)*dt/(.1+dt)
        remaining = self._pivot_direction*(self._pivot_remaining-ahead)
        remaining -= self._pivot_rate_excess*self._pivot_delay_s
        tolerance = self.pivot_tol * (0.5 if self.cfg.delay_s > 0 else 1.0)
        if remaining <= tolerance or self._pivot_direction*self._pivot_remaining <= tolerance:
            self._pivot_braking = True
        if self._pivot_braking:
            want = 0.0
        else:
            # 末段不低于 pivot_floor：实车原地转几 erpm 的小指令轮子推不动，按比例收尾会一直挂着
            magnitude = max(min(self.pivot_floor, self.pivot_rate),
                            min(self.pivot_rate,
                                math.sqrt(2.0*self.pivot_accel*(remaining-.5*tolerance)),
                                (1.8 if self.cfg.delay_s>0 else 2.0)*remaining))
            if magnitude < abs(self._w)-1e-9:
                self._pivot_slowing = True
            if self._pivot_slowing:
                magnitude = min(magnitude,abs(self._w))
            want = self._pivot_direction*magnitude
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
        st.integral_frozen = tr.st.integral_frozen
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
                    pivoting=self._pivot is not None, integral_frozen=st.integral_frozen,
                    constant_straight_speed=self.constant_speed_here(),
                    constant_arc_speed=self.constant_arc_speed_here(),
                    straight_hold=dict(self.trackers[self.index]._straight_status),
                    arc_hold=dict(self.trackers[self.index]._arc_status))
