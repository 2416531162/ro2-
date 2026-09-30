#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RTK 路径跟踪闭环仿真：小圆、跑道形、8 字、app 画的路点路线（带接入段）。控制器是 path_tracker.PathTracker；
app 默认的连线路线（急弯开出去倒车掉头、前进倒车交替）用 run_route 整趟跑 SectionTracker，量"车身中心压没压过全部连线"。

定位来源（source）：
    fused  原始传感器 → 真实融合（默认）：与 line_sim.run_fused 同一套模型——合成 UM982 日志（20 Hz，
           位置慢漂 6 mm + 白噪声 3 mm，航向慢漂 0.1° + 白噪声 0.25°，天线安装偏差 1.5°）、IMU（零偏、噪声、
           量化、尖峰）、轮速（2% 比例误差），经 Um982Stream 和 LineMission 的输入转换送进 gnss_fusion。
           ideal=True 时把传感器噪声、延迟都关掉，只剩控制器和底盘本身的误差。
    truth  直接用底盘真值当位姿（无噪声、无延迟），看控制律和底盘动态的下限。
底盘（chassis）：
    differential  Gen5 差速底盘（两后轮毂电机 + 前自由万向轮，能原地转）：指令延迟 0.44 s（2026-09-30 实车 IMU
              直行辨识：陀螺 ≈ 0.44 s 前的角速度指令 × 1.02–1.04）后换成左右轮目标速度，
              每个轮按 0.6 m/s² 限加速度、不超过 1.2 m/s；
              run_route 按喷头压线、急弯原地转向规划，融合打开 drive_differential。
    measured  实车辨识（默认，2026-09-26 两趟连线路线记录）：指令到曲率的纯延迟 前进 0.35 s / 倒车 0.30 s，
              曲率变化率 3.0 / 0.7 1/m/s，实际曲率 = 指令 × 0.96 / 0.90；车速指令为 0 时固件把前轮回正
              （Akm_Vz_to_Angle），换向停车后要重新打方向。可加舵机零位偏差。
    ideal     旧模型（与 line_sim 相同）：指令延迟 60 ms、舵机 3 rad/s、实际曲率 = 指令，停车时保持舵角。
              比实车乐观得多：同一条连线路线圆弧上 1 cm 对实车 11 cm。
横偏统计用真值：真实后轴中心到路径的距离。只用于离线调参和回归，不代表实车（没有轮胎侧滑）。

    python3 radar_system/tools/path_sim.py                      # 全部场景
    python3 radar_system/tools/path_sim.py circle_r2 figure8_r2 --seeds 8
    python3 radar_system/tools/path_sim.py circle_r2 --source truth
    python3 radar_system/tools/path_sim.py --cover --seeds 4           # 连线路线车身中心压线（掉头补角）
    python3 radar_system/tools/path_sim.py --cover --chassis ideal     # 旧的理想底盘模型
"""
import argparse
import math
from dataclasses import replace
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

from collections import deque  # noqa: E402

from line_sim import CONTROL_DT, DT, NO_POSE, Chassis, FusedSource, GaussMarkov  # noqa: E402
from path_tracker import (PathConfig, PathTracker, SectionTracker, circle, differential_config, figure8,  # noqa: E402
                          stadium, wrap)
import route_plan  # noqa: E402

TRANSIENT_M = 0.6          # 曲率变化后这么长算过渡段，其余算稳态


class MeasuredChassis(Chassis):
    """按实车辨识的转向响应：按曲率建模（纯延迟 → 变化率限幅 → 增益），前进、倒车参数不同。"""
    DELAY_S = (0.35, 0.30)          # 前进、倒车
    KAPPA_RATE = (3.0, 0.7)         # 1/m/s
    GAIN = (0.96, 0.90)             # 实际曲率 / 指令曲率
    KAPPA_LIMIT = 1.0 / 1.35        # 固件 25° 限位对应的最小半径

    def __init__(self, **kw):
        super().__init__(**kw)
        self.kappa = 0.0

    def command(self, t, vx, wz):
        self.queue.append((t + self.DELAY_S[0 if vx >= 0 else 1], vx, wz))

    def step(self, t, dt):
        while self.queue and self.queue[0][0] <= t:
            _, vx, wz = self.queue.popleft()
            self.cmd = (vx, wz)
        vx, wz = self.cmd
        if vx == 0 or self.driver_v * vx < 0:
            self.driver_v = 0.0
        else:
            step = self.accel * dt
            self.driver_v += max(-step, min(step, vx - self.driver_v))
        rev = 1 if vx < 0 else 0
        # 固件：车速指令为 0 时左前轮角 0（回正）；否则按 R = vx/wz 打方向
        target = 0.0 if vx == 0 else self.GAIN[rev] * wz / vx
        target = max(-self.KAPPA_LIMIT, min(self.KAPPA_LIMIT, target))
        dk = self.KAPPA_RATE[rev] * dt
        self.kappa += max(-dk, min(dk, target - self.kappa))
        want = self.driver_v
        dv = (want - self.v) * min(1.0, dt / self.tau)
        speeding_up = abs(want) > abs(self.v) and want * self.v >= 0
        lim = (self.accel if speeding_up else self.decel) * dt
        self.v += max(-lim, min(lim, dv))
        if abs(self.v) < 1e-4 and want == 0:
            self.v = 0.0
        kappa = self.kappa
        if self.quant > 0:
            q = self.quant / self.geo.wheelbase_m
            kappa = round(kappa / q) * q
        kappa += math.tan(self.trim) / self.geo.wheelbase_m
        self.steer = math.atan(self.kappa * self.geo.wheelbase_m)
        self.wz = self.v * kappa
        self.x += self.v * dt * math.cos(self.yaw + self.wz * dt / 2)
        self.y += self.v * dt * math.sin(self.yaw + self.wz * dt / 2)
        self.yaw += self.wz * dt


class DifferentialChassis:
    """Gen5 差速底盘：(vx, wz) 延迟 delay_s 后换成左右轮目标速度（超过 max_wheel 按比例缩），每个轮按
    wheel_accel 限加速度；车速 = 两轮平均，角速度 = 轮速差 / 轮距。前面万向轮随动，不影响运动。"""
    TRACK_M = 0.675                 # Gen5 CAD 后轮轮距
    NOZZLE_X_M = 0.30               # 喷头在后轴前（天线版总装图的横移臂）

    def __init__(self, delay_s=0.44, max_wheel=1.2, wheel_accel=0.6, **_ackermann_only):
        self.x = self.y = self.yaw = 0.0
        self.v = self.wz = self.steer = 0.0
        self.left = self.right = 0.0
        self.delay_s, self.max_wheel, self.wheel_accel = delay_s, max_wheel, wheel_accel
        self.queue, self.cmd = deque(), (0.0, 0.0)

    def command(self, t, vx, wz):
        self.queue.append((t + self.delay_s, vx, wz))

    def step(self, t, dt):
        while self.queue and self.queue[0][0] <= t:
            _, vx, wz = self.queue.popleft()
            self.cmd = (vx, wz)
        vx, wz = self.cmd
        half = 0.5 * self.TRACK_M
        tl, tr = vx - wz * half, vx + wz * half
        scale = min(1.0, self.max_wheel / max(abs(tl), abs(tr), 1e-12))
        step = self.wheel_accel * dt
        self.left += max(-step, min(step, tl * scale - self.left))
        self.right += max(-step, min(step, tr * scale - self.right))
        self.v, self.wz = 0.5 * (self.left + self.right), (self.right - self.left) / self.TRACK_M
        self.x += self.v * dt * math.cos(self.yaw + self.wz * dt / 2)
        self.y += self.v * dt * math.sin(self.yaw + self.wz * dt / 2)
        self.yaw += self.wz * dt


CHASSIS = {'measured': MeasuredChassis, 'ideal': Chassis, 'differential': DifferentialChassis}


class TruthSource:
    """底盘真值直接当位姿：没有噪声和延迟。"""

    def __init__(self):
        self.car = None

    def step(self, t, car):
        self.car = car

    def pose(self, t):
        c = self.car
        return _Pose(c.x, c.y, c.yaw, c.v, c.wz + getattr(c, 'extra_wz', 0.0))


class _Pose:
    quality, sigma_xy, sigma_yaw, reasons, usable_for_control = 'TRUTH', 0.0, 0.0, (), True

    def __init__(self, x, y, yaw, speed, yaw_rate):
        self.x, self.y, self.yaw, self.speed, self.yaw_rate = x, y, yaw, speed, yaw_rate


def _fused_source(seed, ideal, mount_deg, mount_err_deg, sensor_kw, differential=False):
    import gnss_fusion
    rx = dict(pos_drift=0.006, pos_white=0.003, hdg_drift_deg=0.1, hdg_white_deg=0.25, vel_noise=0.02,
              mount_deg=mount_deg, outage=None, heading_outage=None)
    mo = dict(gyro_bias_dps=0.1, gyro_noise_dps=0.034, spikes=True, wheel_error=0.02, imu_delay=0.02,
              wheel_delay=0.03)
    if ideal:
        rx.update(pos_drift=0.0, pos_white=0.0, hdg_drift_deg=0.0, hdg_white_deg=0.0, vel_noise=0.0)
        mo.update(gyro_bias_dps=0.0, gyro_noise_dps=0.0, spikes=False, wheel_error=0.0, imu_delay=0.0,
                  wheel_delay=0.0)
    for k, v in sensor_kw.items():
        (rx if k in rx else mo)[k] = v
    return FusedSource(seed, gnss_fusion.FusionConfig(mount_offset_deg=mount_deg - mount_err_deg,
                                                      drive_differential=differential), rx, mo)


def run(name='circle_r2.5', *, path=None, cruise=0.5, start=(0.0, 0.0, 0.0), source='fused', ideal=False,
        trim_deg=0.0, quant_deg=0.2, terrain=0.0, mount_deg=1.5, mount_err_deg=0.0, config=None, seed=1,
        trace=False, warmup_s=3.0, max_time=None, integral0=None, chassis_model='measured', **sensor_kw):
    """start=(沿路径, 横偏, 车头误差°)，相对路径起点。返回指标字典。"""
    path = path or SCENARIOS[name]['path']()
    rng = random.Random(seed)
    cfg = config or PathConfig(cruise_mps=cruise)
    tracker = PathTracker(path, cfg)
    x0, y0, yaw0 = path.pose(0.0)
    car = CHASSIS[chassis_model](trim_rad=math.radians(trim_deg), steer_quant_rad=math.radians(quant_deg))
    along, lat, hd = start
    car.x = x0 + along * math.cos(yaw0) - lat * math.sin(yaw0)
    car.y = y0 + along * math.sin(yaw0) + lat * math.cos(yaw0)
    car.yaw = yaw0 + math.radians(hd)
    if source == 'truth':
        src, warmup_s = TruthSource(), 0.0
    else:
        src = _fused_source(seed, ideal, mount_deg, mount_err_deg, sensor_kw)
    bumps = GaussMarkov(terrain, 0.5, rng)

    t, next_ctrl, started = 0.0, 0.0, False
    s_true = None
    rows = []
    max_time = max_time or (warmup_s + path.length / cruise * 1.5 + 20.0)
    while t < max_time:
        src.step(t, car)
        if t >= next_ctrl - 1e-9:
            next_ctrl += CONTROL_DT
            pose = src.pose(t) or NO_POSE
            if not started and t >= warmup_s:
                started = tracker.start(pose, integral=integral0)
                if not started and t > warmup_s + 5.0:
                    break
            if started:
                vx, wz = tracker.update(pose, CONTROL_DT)
                car.command(t, vx, wz)
                s_true, e_true, _, k_true = path.project(car.x, car.y, s_true, extend_start=cfg.start_before_m)
                st = tracker.st
                rows.append(dict(
                    t=t, x=car.x, y=car.y, s=s_true, e_true=e_true, e_ctrl=st.e, kappa_ref=k_true,
                    kappa_cmd=st.kappa, steer=car.steer, v=car.v, state=st.state,
                    pos_err=None if pose is NO_POSE else math.hypot(pose.x - car.x, pose.y - car.y),
                    yaw_err=None if pose is NO_POSE else wrap(pose.yaw - car.yaw)))
                if st.state in ('DONE', 'ABORTED') and abs(car.v) < 1e-3:
                    break
        car.step(t, DT)
        if terrain > 0:
            car.extra_wz = bumps.step(DT) * car.v
            car.yaw += car.extra_wz * DT
        t += DT
    return summarize(name, path, tracker, rows, t, trace)


def summarize(name, path, tracker, rows, t, trace=False):
    """横偏（真值）统计：从入口直线结束一直算到终点（含出口直线）；过渡 = 每次曲率变化后 TRANSIENT_M 米内。"""
    loop_a = path.pieces[0].length
    changes = [p.s0 for i, p in enumerate(path.pieces) if i and p.kappa != path.pieces[i - 1].kappa]
    run = [r for r in rows if r['state'] == 'RUNNING']
    loop = [r for r in run if loop_a <= r['s']]
    trans = [r for r in run if any(0.0 <= r['s'] - c <= TRANSIENT_M for c in changes)]
    steady_arc = [r for r in loop if abs(r['kappa_ref']) > 1e-9 and r not in trans]

    def rms(sel):
        return round(math.sqrt(sum(r['e_true'] ** 2 for r in sel) / len(sel)) * 100, 2) if sel else None

    def peak(sel, key='e_true', scale=100.0):
        vals = [abs(r[key]) for r in sel if r[key] is not None]
        return round(max(vals) * scale, 2) if vals else None

    kmax = tracker.kappa_max
    last = rows[-1] if rows else None
    usable = [r for r in run if r['pos_err'] is not None]
    result = dict(
        name=name, path=path.name, state=tracker.st.state, reason=tracker.st.reason,
        radius_m=round(1.0 / path.kappa_abs_max, 2) if path.kappa_abs_max else None,
        rms_cm=rms(loop), max_cm=peak(loop), arc_rms_cm=rms(steady_arc), arc_max_cm=peak(steady_arc),
        trans_max_cm=peak(trans),
        steer_max_deg=round(math.degrees(max((abs(r['steer']) for r in loop), default=0.0)), 1),
        saturated_pct=round(100.0 * sum(abs(r['kappa_cmd']) >= 0.999 * kmax for r in run) / max(1, len(run)), 1),
        end_err_cm=None if last is None else round((last['s'] - path.length) * 100, 1),
        fusion_pos_max_cm=peak(usable, 'pos_err'), fusion_yaw_max_deg=peak(usable, 'yaw_err', math.degrees(1.0)),
        time_s=round(t, 2), integral=tracker.st.integral)
    if trace:
        result['rows'] = rows
    return result


def app_route(points, *, laps=2, turn_radius=2.5, car=(-2.0, -3.0, math.pi / 2), closed=True):
    """app 画的路点路线：从 car 位姿经 Dubins 接入，拐角圆弧过渡（与 line_mission 走路线同一套几何）。"""
    from robot_core.kinematics import ChassisGeometry
    spec = route_plan.parse_spec(dict(kind='points', points=points, closed=closed, laps=laps,
                                      turn_radius=turn_radius))
    return route_plan.plan(spec, car, ChassisGeometry().min_turn_radius_m / PathConfig().kappa_margin)[0]


def app_path(points, *, closed=False, laps=1, car=(0.0, 0.0, 0.0)):
    """连线路线：点按顺序连直线，从 car 位姿先开到第 1 个点（与 line_mission 同一套几何）。"""
    from robot_core.kinematics import ChassisGeometry
    spec = route_plan.parse_spec(dict(kind='path', points=points, closed=closed, laps=laps))
    return route_plan.plan(spec, car, ChassisGeometry().min_turn_radius_m / PathConfig().kappa_margin)[0]


def cover_gap(polyline, track, step=0.01, cell=0.25):
    """连线上每一点到轨迹（点列）的最远距离，米。轨迹点间距要远小于要分辨的误差。"""
    cells = {}
    for q in track:
        cells.setdefault((math.floor(q[0] / cell), math.floor(q[1] / cell)), []).append(q)
    worst = 0.0
    for a, b in zip(polyline, polyline[1:]):
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
        for i in range(n + 1):
            x, y = a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n
            cx, cy = math.floor(x / cell), math.floor(y / cell)
            near = [q for dx in (-1, 0, 1) for dy in (-1, 0, 1) for q in cells.get((cx + dx, cy + dy), ())]
            worst = max(worst, min((math.hypot(x - q[0], y - q[1]) for q in near), default=math.inf))
    return worst


def run_route(name='cover', *, spec, car=(-3.0, 0.0, 0.0), source='fused', ideal=False, trim_deg=0.0, quant_deg=0.2,
              terrain=0.0, mount_deg=1.5, mount_err_deg=0.0, seed=1, warmup_s=3.0, trace=False, config=None,
              chassis_model='measured', **sensor_kw):
    """连线路线整趟（接入 + 掉头补角）：底盘真值的车身中心压没压过全部连线（cover_cm），各类段的真值横偏。

    chassis_model='differential'（Gen5）：喷头压线、急弯原地转向，cover_cm 量的是喷头；另报原地转向次数 pivots。
    """
    from robot_core.kinematics import ChassisGeometry
    from runtime_config import PROFILE
    g = PROFILE['geometry']
    body = (g['front_m'], g['rear_m'], g['half_width_m'])
    diff = chassis_model == 'differential'
    geo = ChassisGeometry(drive='differential', track_m=DifferentialChassis.TRACK_M) if diff else ChassisGeometry()
    nozzle = DifferentialChassis.NOZZLE_X_M if diff else None
    centre = nozzle if diff else 0.5 * (body[0] - body[1])
    rng = random.Random(seed)
    base = PathConfig(cruise_mps=spec.cruise)
    cfg = replace(config, cruise_mps=spec.cruise) if config else (differential_config(base) if diff else base)
    sections, entry_m = route_plan.plan(spec, car, geo.min_turn_radius_m / cfg.kappa_margin, body=body, nozzle_x=nozzle)
    extra = dict(pivot_rate=geo.pivot_rate_rad_s, pivot_accel=geo.pivot_accel_rad_s2, reverse_gain=1.0) if diff else {}
    tracker = SectionTracker(sections, cfg, geo, **extra)
    pivots = route_plan.pivot_count([(sec.direction, sec.path.pose(0.0), [(p.length, p.kappa) for p in sec.path.pieces])
                                     for sec in sections]) if diff else 0
    chassis = CHASSIS[chassis_model](trim_rad=math.radians(trim_deg), steer_quant_rad=math.radians(quant_deg))
    chassis.x, chassis.y, chassis.yaw = car
    if source == 'truth':
        src, warmup_s = TruthSource(), 0.0
    else:
        src = _fused_source(seed, ideal, mount_deg, mount_err_deg, sensor_kw, differential=diff)
    bumps = GaussMarkov(terrain, 0.5, rng)
    t, next_ctrl, started = 0.0, 0.0, False
    track, rows = [], []
    max_time = warmup_s + tracker.length / min(cfg.cruise_mps, 0.3) * 1.5 + 3.0 * tracker.cusps + 8.0 * pivots + 20.0
    while t < max_time:
        src.step(t, chassis)
        if t >= next_ctrl - 1e-9:
            next_ctrl += CONTROL_DT
            pose = src.pose(t) or NO_POSE
            if not started and t >= warmup_s:
                started = tracker.start(pose)
                if not started and t > warmup_s + 5.0:
                    break
            if started:
                vx, wz = tracker.update(pose, CONTROL_DT)
                chassis.command(t, vx, wz)
                index = tracker.index
                sec = sections[index]
                # 各段都不长、不自交：全程找最近点就是真值横偏
                _, e_true, _, k_ref = sec.path.project(chassis.x, chassis.y, extend_start=cfg.start_before_m)
                rows.append(dict(t=t, section=index, direction=sec.direction, straight=abs(k_ref) < 1e-9,
                                 e_true=e_true, switching=tracker.switching, state=tracker.st.state, v=chassis.v))
                if tracker.st.state in ('DONE', 'ABORTED') and abs(chassis.v) < 1e-3:
                    break
        chassis.step(t, DT)
        if terrain > 0:
            chassis.extra_wz = bumps.step(DT) * chassis.v
            chassis.yaw += chassis.extra_wz * DT
        cx, cy = chassis.x + centre * math.cos(chassis.yaw), chassis.y + centre * math.sin(chassis.yaw)
        if started and (not track or math.hypot(cx - track[-1][0], cy - track[-1][1]) >= 0.002):
            track.append((cx, cy))
        t += DT
    moving = [r for r in rows if not r['switching'] and r['state'] == 'RUNNING']

    def peak(sel):
        return round(max((abs(r['e_true']) for r in sel), default=0.0) * 100, 2)
    if spec.kind == 'circle':
        # 图纸上的圆（喷头要画的线），每 2° 一点
        cx, cy = spec.center
        drawn = [(cx + spec.radius * math.cos(math.radians(a)), cy + spec.radius * math.sin(math.radians(a)))
                 for a in range(0, 361, 2)]
    else:
        drawn = route_plan.path_points(spec)
    end = drawn[-1]
    result = dict(
        name=name, state=tracker.st.state, reason=tracker.st.reason, sections=len(sections),
        cover_cm=round(cover_gap(drawn, track) * 100, 2),
        vertex_cm=round(max(min(math.hypot(v[0] - q[0], v[1] - q[1]) for q in track) for v in drawn) * 100, 2),
        fwd_line_max_cm=peak([r for r in moving if r['direction'] > 0 and r['straight'] and r['section'] > 0]),
        rev_max_cm=peak([r for r in moving if r['direction'] < 0]),
        arc_max_cm=peak([r for r in moving if r['direction'] > 0 and not r['straight']]),
        end_err_cm=round(math.hypot(track[-1][0] - end[0], track[-1][1] - end[1]) * 100, 1)
        if track and spec.kind != 'circle' else None,
        entry_m=round(entry_m, 2), drive_m=round(tracker.length, 2), time_s=round(t - warmup_s, 1),
        integral=tracker.st.integral, pivots=pivots)
    if trace:
        result['rows'], result['track'] = rows, track
    return result


def _cover(points, *, closed=False, laps=1, cruise=0.5):
    return route_plan.parse_spec(dict(kind='path', points=points, closed=closed, laps=laps, cruise=cruise))


COVER_SCENARIOS = {
    # 连线路线：直角、60°、钝角、缓弯、窄行往返、折返，车从第一段延长线、侧面、起点前方出发
    'rect_2laps': dict(spec=_cover([[0, 0], [8, 0], [8, 6], [0, 6]], closed=True, laps=2)),
    'rect_side_start': dict(spec=_cover([[0, 0], [8, 0], [8, 6], [0, 6]], closed=True), car=(-1.0, -3.0, math.pi / 2)),
    'rect_fast_trim2': dict(spec=_cover([[0, 0], [8, 0], [8, 6], [0, 6]], closed=True, cruise=0.8), trim_deg=2.0),
    'triangle': dict(spec=_cover([[0, 0], [10, 0], [5, 8.66]], closed=True)),
    'zigzag': dict(spec=_cover([[3, 0], [9, 0], [12, 5], [16, 1], [22, 4]]), car=(0.3, 0.1, 0.0)),
    'obtuse_146': dict(spec=_cover([[0, 0], [8, 0], [2, 4], [2, 12]])),
    'gentle': dict(spec=_cover([[0, 0], [6, 0], [12, 2], [18, 2.5]])),
    'rows_0.8': dict(spec=_cover([[0, 0], [8, 0], [8, 0.8], [0, 0.8], [0, 1.6], [8, 1.6]])),
    'hairpin_176': dict(spec=_cover([[0, 0], [8, 0], [0, 0.5]])),
}


SCENARIOS = {
    'circle_r3': dict(path=lambda: circle(3.0)),
    'circle_r2.5': dict(path=lambda: circle(2.5)),
    'circle_r2': dict(path=lambda: circle(2.0)),
    'circle_r2_cw': dict(path=lambda: circle(2.0, left=False)),
    'circle_r2.5_2laps': dict(path=lambda: circle(2.5, laps=2)),
    'circle_r2.5_fast': dict(path=lambda: circle(2.5), cruise=0.8),
    'circle_r2.5_trim2': dict(path=lambda: circle(2.5), trim_deg=2.0),
    'circle_r2.5_noisy': dict(path=lambda: circle(2.5), pos_drift=0.012, pos_white=0.006, hdg_drift_deg=0.2,
                              hdg_white_deg=0.5),
    'circle_r2.5_offset': dict(path=lambda: circle(2.5), start=(-1.0, 0.30, 10.0)),
    'circle_r2.5_mount1': dict(path=lambda: circle(2.5), mount_err_deg=1.0),
    'stadium_r2.5': dict(path=lambda: stadium(2.5, 4.0)),
    'figure8_r2.5': dict(path=lambda: figure8(2.5)),
    'figure8_r2': dict(path=lambda: figure8(2.0)),
    # app 画的路线：矩形 2 圈、五边形（锐角拐弯）拐弯半径贴近底盘极限
    'route_rect': dict(path=lambda: app_route([[0, 0], [10, 0], [10, 8], [0, 8]])),
    'route_penta_r2': dict(path=lambda: app_route([[0, 0], [12, 0], [15, 9], [4, 11], [-3, 5]], turn_radius=2.0)),
    'route_open_zigzag': dict(path=lambda: app_route([[0, 0], [8, 0], [10, 7], [16, 1], [22, 6]], closed=False,
                                                     laps=1, car=(-3.0, 1.0, -0.3))),
    # 连线路线（app 默认）：操作员点的点按顺序连直线，参考线是车能开的那条
    'path_zigzag': dict(path=lambda: app_path([[3, 0], [9, 0], [12, 5], [16, 1], [22, 4]])),
    'path_close_points': dict(path=lambda: app_path([[3 + 0.1 * i, 0.4 * math.sin(0.15 * i)] for i in range(120)])),
    'path_square_loop': dict(path=lambda: app_path([[3, 0], [11, 0], [11, 7], [3, 7]], closed=True, laps=2)),
}


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('names', nargs='*')
    p.add_argument('--source', choices=('fused', 'truth'), default='fused')
    p.add_argument('--ideal', action='store_true', help='融合链路但传感器无噪声、无延迟')
    p.add_argument('--seeds', type=int, default=1, help='跑几个随机种子，取最差')
    p.add_argument('--cover', action='store_true', help='跑连线路线（倒车补角）的压线场景')
    p.add_argument('--chassis', choices=tuple(CHASSIS), default='measured', help='底盘模型，默认实车辨识')
    args = p.parse_args()
    if args.cover:
        keys = ('state', 'cover_cm', 'vertex_cm', 'fwd_line_max_cm', 'rev_max_cm', 'arc_max_cm', 'end_err_cm',
                'sections', 'drive_m', 'time_s')
        print('%-18s' % 'scenario' + ''.join('%16s' % k for k in keys))
        for n in args.names or list(COVER_SCENARIOS):
            rs = [run_route(n, seed=s, source=args.source, ideal=args.ideal, chassis_model=args.chassis,
                            **COVER_SCENARIOS[n])
                  for s in range(1, args.seeds + 1)]
            worst = {k: ('/'.join(sorted({r[k] for r in rs})) if k == 'state' else max((r[k] for r in rs), key=abs))
                     for k in keys}
            print('%-18s' % n + ''.join('%16s' % (worst[k],) for k in keys))
        return
    keys = ('state', 'rms_cm', 'max_cm', 'arc_rms_cm', 'arc_max_cm', 'trans_max_cm', 'steer_max_deg',
            'saturated_pct', 'end_err_cm', 'fusion_pos_max_cm', 'fusion_yaw_max_deg')
    print('%-20s' % 'scenario' + ''.join('%15s' % k for k in keys))
    for n in args.names or list(SCENARIOS):
        kw = {k: v for k, v in SCENARIOS[n].items() if k != 'path'}
        rs = [run(n, seed=s, source=args.source, ideal=args.ideal, chassis_model=args.chassis, **kw)
              for s in range(1, args.seeds + 1)]

        def worst(k):
            vals = [r[k] for r in rs if isinstance(r[k], (int, float))]
            if k == 'state':
                return '/'.join(sorted({r['state'] for r in rs}))
            return max(vals, key=abs) if vals else None
        print('%-20s' % n + ''.join('%15s' % (worst(k),) for k in keys))


if __name__ == '__main__':
    main()
