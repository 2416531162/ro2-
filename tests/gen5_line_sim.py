#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gen5 定点画线（A→B 直线）闭环仿真：车端真实 LineMission（差速车走 line_cascade.ImuLineTracker）+ 实测底盘模型。

测试 tests/test_line_cascade.py 用它；也可以直接跑出对照表：
    python3 tests/gen5_line_sim.py

底盘（Gen5Chassis，2026-09-30 室内 IMU 直行 + 倒车数据拟合）：车速、角速度指令都晚 delay_s（0.39 s）才执行，
角速度另有 0.05 s 一阶滞后、实际/指令 = gain（1.03）；加速 1.0 m/s²、减速 2.4 m/s²，单轮不超过 1.5 m/s（轮距 0.675 m）。
万向轮 / 地面偏转（Disturbance，角速度扰动，随车速变大）：各频段都是若干正弦（频率、相位随种子），
    0.05–0.15 Hz  0.20°/s × (v/0.4)      0.15–0.3 Hz  0.25°/s × (v/0.4)
    0.3–0.8 Hz    0.30°/s × (v/0.4)      0.8–2 Hz     0.35°/s × (v/0.4)^2.1
    → 0.15–2 Hz 合计 0.4 m/s 约 0.52°/s、0.8 m/s 约 1.7°/s；0.3–0.8 Hz 段 0.8 m/s 是 0.4 m/s 的 2 倍（与实测一致）。
IMU（50 Hz）：偏航 = 真实偏航 + 任意零点 + 漂移（imu_drift_deg_min，陀螺带同样的零偏）+ 0.005° 噪声，0.01° 一格。
融合位姿（控制周期取）：位置 = 真值 + 白噪声 3 mm + 慢漂 6 mm（一阶马尔可夫 20 s）；
    航向 = 真值 + 双天线噪声（白噪声 0.25° 按 0.25 s 平滑，融合约 0.25 s 跟上测向）+ 慢漂 0.1° + 固定偏差 align_deg
    （双天线 / IMU 与真北对齐的残差，外环积分要吃掉它）。
量的是真实喷头（后轴前 0.30 m）到 A→B 线的距离：喷头在 A、B 之间（离两头各 margin_m 以内不算）、车速 > 0.1 m/s。
"""
import dataclasses
import math
import os
import random
import sys
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, ROOT)

from line_mission import LineMission  # noqa: E402
from line_tracker import LineConfig, LineTracker  # noqa: E402
from path_tracker import Path, PathConfig, PathTracker, differential_config  # noqa: E402
from robot_core.config import CONTROL_HZ  # noqa: E402
from robot_core.kinematics import ChassisGeometry  # noqa: E402

DEG = math.pi / 180.0
NOZZLE_X = 0.30
ANTENNA_XY = (0.24, 0.2075)     # 主天线在车体系的位置（车上 ~/rtk/fusion.json 的 antenna_xy）
TRACK_M = 0.675
DT = 0.01                   # 底盘积分步长
IMU_DT = 0.02               # /imu/data 50 Hz
GEN5 = ChassisGeometry(drive='differential', track_m=TRACK_M, diff_min_radius_m=0.5, pivot_rate_rad_s=0.8,
                       pivot_accel_rad_s2=1.0)
# 扰动频段：(低, 高 Hz, 0.4 m/s 时 rms °/s, 车速指数)
BANDS = ((0.05, 0.15, 0.20, 1.0), (0.15, 0.3, 0.25, 1.0), (0.3, 0.8, 0.30, 1.0), (0.8, 2.0, 0.35, 2.1))


class Disturbance:
    """万向轮 / 地面带来的车身偏转角速度（rad/s），随车速变大。"""

    def __init__(self, seed, per_band=8, scale=1.0):
        rng = random.Random(seed)
        self.bands = []
        for lo, hi, rms, power in BANDS:
            waves = [(rng.uniform(lo, hi), rng.uniform(0.0, 2 * math.pi)) for _ in range(per_band)]
            self.bands.append((lo, hi, rms * scale, power, waves))

    def band_rms(self, v, lo, hi):
        """[lo, hi) Hz 内的角速度 rms，°/s（按正弦分量算）。"""
        total = 0.0
        for _, _, rms, power, waves in self.bands:
            amp = rms * (v / 0.4) ** power * math.sqrt(2.0 / len(waves))
            total += sum(amp * amp / 2.0 for f, _ in waves if lo <= f < hi)
        return math.sqrt(total)

    def rate(self, t, v):
        v = abs(v)
        if v < 0.05:
            return 0.0
        out = 0.0
        for _, _, rms, power, waves in self.bands:
            amp = rms * DEG * (v / 0.4) ** power * math.sqrt(2.0 / len(waves))
            out += amp * sum(math.sin(2 * math.pi * f * t + p) for f, p in waves)
        return out


class Gen5Chassis:
    """指令延迟 delay_s（角速度）/ speed_delay_s（车速）、角速度一阶滞后 lag_s、实际/指令 gain，加减速限幅，单轮限速。

    speed_delay_s：None 同 delay_s。2026-10-01 实车 26 次路线段尾停车，发 0 后实际还走的距离按已发指令折算的
    等效延迟中位 0.33 s（四分位 0.29–0.37 s；这个替身车车速延迟 0.39 s 时等效约 0.39 s）；定点画线按 09-30 实测
    0.40 s 调的，默认不改。"""

    def __init__(self, x, y, yaw, *, delay_s=0.39, speed_delay_s=None, lag_s=0.05, gain=1.03, accel=1.0, decel=2.4,
                 max_wheel=1.5, dist=None):
        self.x, self.y, self.yaw = x, y, yaw
        self.v = self.wz = self.wz_act = 0.0
        self.delay_s, self.lag_s, self.gain, self.accel, self.decel = delay_s, lag_s, gain, accel, decel
        self.speed_delay_s = delay_s if speed_delay_s is None else speed_delay_s
        self.max_wheel, self.dist = max_wheel, dist
        self.push = 0.0             # 外力推着转的角速度（测试「被推歪」用），rad/s
        self.queue, self.vqueue, self.cmd = [], [], (0.0, 0.0)

    def command(self, t, vx, wz):
        self.queue.append((t + self.delay_s, wz))
        self.vqueue.append((t + self.speed_delay_s, vx))

    def step(self, t, dt):
        vx, wz = self.cmd
        while self.queue and self.queue[0][0] <= t + 1e-9:
            wz = self.queue.pop(0)[1]
        while self.vqueue and self.vqueue[0][0] <= t + 1e-9:
            vx = self.vqueue.pop(0)[1]
        self.cmd = (vx, wz)
        half = 0.5 * TRACK_M
        scale = min(1.0, self.max_wheel / max(abs(vx - wz * half), abs(vx + wz * half), 1e-12))
        vx, wz = vx * scale, wz * scale
        speeding = abs(vx) > abs(self.v) and vx * self.v >= 0
        lim = (self.accel if speeding else self.decel) * dt
        self.v += max(-lim, min(lim, vx - self.v))
        self.wz_act += (self.gain * wz - self.wz_act) * min(1.0, dt / self.lag_s)
        self.wz = self.wz_act + (self.dist.rate(t, self.v) if self.dist else 0.0) + self.push
        mid = self.yaw + 0.5 * self.wz * dt
        self.x += self.v * dt * math.cos(mid)
        self.y += self.v * dt * math.sin(mid)
        self.yaw += self.wz * dt

    def nozzle(self):
        return self.x + NOZZLE_X * math.cos(self.yaw), self.y + NOZZLE_X * math.sin(self.yaw)


class PoseNoise:
    """融合位姿误差：位置白噪声 + 慢漂，航向双天线噪声（0.25 s 平滑）+ 慢漂 + 固定偏差。"""

    def __init__(self, seed, pos_white=0.003, pos_drift=0.006, yaw_white_deg=0.25, yaw_drift_deg=0.1,
                 yaw_bias_deg=0.0):
        self.r = random.Random(seed)
        self.pos_white, self.pos_drift = pos_white, pos_drift
        self.yaw_white, self.yaw_drift, self.yaw_bias = yaw_white_deg * DEG, yaw_drift_deg * DEG, yaw_bias_deg * DEG
        self.px = self.r.gauss(0, pos_drift)
        self.py = self.r.gauss(0, pos_drift)
        self.ph = self.r.gauss(0, self.yaw_drift)
        self.hf = 0.0

    def step(self, dt):
        """(dx, dy, dyaw)。"""
        a = math.exp(-dt / 20.0)
        b = math.sqrt(1 - a * a)
        self.px = a * self.px + b * self.r.gauss(0, self.pos_drift)
        self.py = a * self.py + b * self.r.gauss(0, self.pos_drift)
        self.ph = a * self.ph + b * self.r.gauss(0, self.yaw_drift)
        self.hf += (self.r.gauss(0, self.yaw_white) - self.hf) * min(1.0, dt / 0.25)
        return (self.r.gauss(0, self.pos_white) + self.px, self.r.gauss(0, self.pos_white) + self.py,
                self.ph + self.hf + self.yaw_bias)


class SimFusion:
    """LineMission 的融合替身：位姿由仿真直接给；经纬度 1e-5°/m 线性映射。"""

    def __init__(self):
        self.current = None

    def pose(self, t):
        return self.current

    def antenna_lever(self):
        return ANTENNA_XY

    def to_geodetic(self, x, y):
        return 30.0 + y * 1e-5, 120.0 + x * 1e-5

    def to_local(self, lat, lon):
        return (lon - 120.0) / 1e-5, (lat - 30.0) / 1e-5

    def __getattr__(self, name):
        if name.startswith('add_'):
            return lambda sample: None
        raise AttributeError(name)


SAMPLES = SimpleNamespace(ImuSample=lambda **kw: kw, WheelSample=lambda **kw: kw, GnssFix=lambda **kw: kw,
                          DualHeading=lambda **kw: kw, GnssVelocity=lambda **kw: kw)


def make_pose(x, y, yaw, speed, yaw_rate, usable=True):
    return SimpleNamespace(x=x, y=y, yaw=yaw, speed=speed, yaw_rate=yaw_rate, usable_for_control=usable,
                           quality='RTK_FIXED' if usable else 'DEAD_RECKONING', sigma_xy=0.01,
                           sigma_yaw=0.1 * DEG, reasons=() if usable else ('rtk_fixed_timeout',))


def percentile(values, q):
    values = sorted(values)
    return values[int(q * (len(values) - 1))]


def run(controller='cascade', *, v=0.8, seed=1, length=40.0, delay_s=0.39, lag_s=0.05, gain=1.03,
        imu_drift_deg_min=0.03, align_deg=0.0, rate_hz=CONTROL_HZ, start=None, dist_scale=1.0, margin_m=0.0,
        imu_gap=None, pose_gap=None, push=None, max_time=None, noise_kw=None, cascade_kw=None, heading_jump=None,
        trace=None, imu_scale=0.0, path_kw=None):
    """跑一条 A=(0,0) → B=(length,0) 的线，返回结果 dict：
    state/message（任务结束时）、p95_cm/max_cm（真实喷头横偏）、rtk_p95_cm/rtk_max_cm（按控制器拿到的融合位姿算的喷头横偏，
    即实车作业记录里能算出来的那个；看不到 RTK 自己的慢漂）、errs/rtk_errs（样本，m，合并几个种子算用）、
    stop_m（喷头停在 B 前后多少，正 = 过了 B）。

    controller：'cascade' = LineMission 差速车定点画线（本次的新控制器）；'old' = 原 LineTracker（对照）；
    'path' = 路线作业的 PathTracker（differential_config，path_kw 可改参数）。
    start：(x, y, 偏航°)，默认 A 后方 2 m、按种子带 ±2 cm / ±1° 的小误差。
    imu_gap=(t0, t1)：这段时间没有 IMU；pose_gap=(t0, t1)：这段时间定位不可用；push=(t0, t1, rad/s)：这段时间被推着转。
    heading_jump=(t0, t1, 度)：这段时间融合航向跳了这么多（双天线测向跳变），IMU 没变。
    融合航向的所有误差（噪声、偏差、跳变）都按天线杠杆臂 ANTENNA_XY 带进融合位置（融合用自己的航向从天线倒推后轴）。
    imu_scale：IMU 刻度误差（0.03 = IMU 转出来的角度比真实大 3%，偏航和陀螺都是）。
    trace：给一个 list 就每个控制周期记一条 (t, mission 阶段, 跟踪状态, 原因, message, 真实车速, 真实喷头横偏)。
    """
    rng = random.Random(seed + 7)
    if start is None:
        start = (-2.0, rng.uniform(-0.02, 0.02), rng.uniform(-1.0, 1.0))
    car = Gen5Chassis(start[0], start[1], start[2] * DEG, delay_s=delay_s, lag_s=lag_s, gain=gain,
                      dist=Disturbance(seed, scale=dist_scale))
    noise = PoseNoise(seed + 100, yaw_bias_deg=align_deg * (1 if rng.random() < 0.5 else -1), **(noise_kw or {}))
    drift = imu_drift_deg_min * DEG / 60.0 * (1 if rng.random() < 0.5 else -1)
    imu_zero = rng.uniform(-math.pi, math.pi)       # HDA536T 上电为 0：与真北差一个任意角
    fusion = SimFusion()
    ctrl_dt = 1.0 / rate_hz

    if controller == 'cascade':
        m = LineMission(fusion, SAMPLES, tracker=LineTracker(LineConfig(), GEN5), nozzle_x=NOZZLE_X)
        m.points = {'A': fusion.to_geodetic(0.0, 0.0), 'B': fusion.to_geodetic(length, 0.0)}
        old = None
    elif controller == 'path':
        m = None
        cfg = dataclasses.replace(differential_config(PathConfig(cruise_mps=v)), **(path_kw or {}))
        old = PathTracker(Path((0.0, 0.0, 0.0), [(length, 0.0)]), cfg, GEN5)
    else:
        m = None
        old = LineTracker(LineConfig(cruise_mps=v), GEN5)
        old.set_line((0.0, 0.0), (length, 0.0))

    t, next_ctrl, next_imu = 0.0, 0.0, 0.0
    started, result, errs, rtk_errs, done_t = False, None, [], [], None
    max_time = max_time or length / v * 2.0 + 30.0
    meas = (0.0, 0.0, 0.0)
    while t < max_time:
        if t >= next_imu - 1e-9:
            next_imu += IMU_DT
            if m is not None and not (imu_gap and imu_gap[0] <= t < imu_gap[1]):
                yaw_imu = car.yaw * (1.0 + imu_scale) + imu_zero + drift * t + rng.gauss(0.0, 0.005 * DEG)
                yaw_imu = math.atan2(math.sin(yaw_imu), math.cos(yaw_imu))
                yaw_imu = round(yaw_imu / (0.01 * DEG)) * 0.01 * DEG
                m.on_imu(t, car.wz * (1.0 + imu_scale) + drift + rng.gauss(0.0, 0.05 * DEG), yaw_imu)
        if t >= next_ctrl - 1e-9:
            next_ctrl += ctrl_dt
            meas = noise.step(ctrl_dt)
            usable = not (pose_gap and pose_gap[0] <= t < pose_gap[1])
            jump = heading_jump[2] * DEG if heading_jump and heading_jump[0] <= t < heading_jump[1] else 0.0
            # 融合航向错了 err（噪声、偏差、跳变）：后轴中心 = 天线 − R(航向 + err)·杠杆臂，
            # 比真值偏 (R(ψ) − R(ψ + err))·杠杆臂
            err = meas[2] + jump
            (ax, ay), c0, s0, c1, s1 = ANTENNA_XY, math.cos(car.yaw), math.sin(car.yaw), \
                math.cos(car.yaw + err), math.sin(car.yaw + err)
            jx, jy = (c0 - c1) * ax - (s0 - s1) * ay, (s0 - s1) * ax + (c0 - c1) * ay
            pose = make_pose(car.x + meas[0] + jx, car.y + meas[1] + jy, car.yaw + err, car.v, car.wz, usable)
            fusion.current = pose
            nx, _ = car.nozzle()
            if margin_m <= nx <= length - margin_m and car.v > 0.1 and result is None:
                rtk_errs.append(abs(pose.y + NOZZLE_X * math.sin(pose.yaw)))
            if m is not None:
                if not started:
                    ok, msg = m.command({'cmd': 'start', 'cruise': v}, t)
                    if not ok:
                        return dict(state='REFUSED', message=msg)
                    if cascade_kw:
                        m.tracker.cfg = dataclasses.replace(m.tracker.cfg, **cascade_kw)
                    started = True
                vx, wz, _ = m.tick(t, 'NAVIGATION')
                state = m.tracker.st.state
                if trace is not None:
                    trace.append((round(t, 3), m.phase, state, m.tracker.st.reason, m.message, car.v, car.nozzle()[1]))
                if m.phase == 'IDLE' and result is None:
                    result = dict(state=state, message=m.message, reason=m.tracker.st.reason)
                    done_t = t
            else:
                if not started:
                    started = old.start(pose)
                    if not started:
                        return dict(state='REFUSED', message=old.st.reason)
                vx, wz = old.update(pose, ctrl_dt)
                if old.st.state in ('DONE', 'ABORTED') and result is None:
                    result = dict(state=old.st.state, message=old.st.reason, reason=old.st.reason)
                    done_t = t
            car.command(t, vx, wz)
            if result is not None and abs(car.v) < 1e-3 and t - done_t > delay_s + 0.2:
                break
        car.push = push[2] if push and push[0] <= t < push[1] else 0.0
        car.step(t, DT)
        nx, ny = car.nozzle()
        if margin_m <= nx <= length - margin_m and car.v > 0.1 and result is None:
            errs.append(abs(ny))
        t += DT
    out = dict(result or dict(state='TIMEOUT', message='', reason=''))
    nx, ny = car.nozzle()
    out.update(t=t, stop_m=nx - length, stop_lateral_m=ny, n=len(errs), errs=errs, rtk_errs=rtk_errs,
               p95_cm=percentile(errs, 0.95) * 100 if errs else None, max_cm=max(errs) * 100 if errs else None,
               rtk_p95_cm=percentile(rtk_errs, 0.95) * 100 if rtk_errs else None,
               rtk_max_cm=max(rtk_errs) * 100 if rtk_errs else None, mission=m)
    return out


def _one(args):
    kind, kw = args
    r = run(kind, **kw)
    for key in ('mission', 'errs', 'rtk_errs'):
        r.pop(key, None)
    return args, r


if __name__ == '__main__':
    from multiprocessing import Pool
    cases = []
    for v in (0.5, 0.8):
        for seed in (1, 2, 3, 4):
            for kind in ('old', 'cascade'):
                cases.append((kind, dict(v=v, seed=seed)))
            cases.append(('cascade', dict(v=v, seed=seed, rate_hz=20.0)))  # 旧控制周期对照
    for seed in (1, 2, 3, 4):
        cases.append(('cascade', dict(v=0.8, seed=seed, length=100.0, imu_drift_deg_min=1.0, align_deg=0.5)))
    with Pool(8) as pool:
        res = pool.map(_one, cases)
    for (kind, kw), r in res:
        print(kind, kw, {k: (round(val, 3) if isinstance(val, float) else val) for k, val in r.items()}, flush=True)
