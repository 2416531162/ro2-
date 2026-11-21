#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""草地模式（2026-11-21 用户「做草地模式」）：11-21 草地路线作业中段偏差 0.5–1 cm、最大 2–2.5 cm，一半是左右轮负载乱跳
把车头拉偏（IMU 看得见），一半是侧滑（IMU 看不见）。拟合的角度（IMU 车头 → 实际走向）按 0.005°/√m 游走，平地合适，
草地上几米就变 0.2° 左右，学得太慢。按走过的路取最近 2 m 两轮电机电流中位数认草地（≥ 5 A 开、≤ 4 A 关；11-21 实录
平地最高 4.3 A，草地一般 8–12 A），草地时线头段之后角度游走放到 0.1°/√m。平地、线头段完全照旧。"""
import math
import os
import random
import sys
import unittest
from collections import deque
from dataclasses import replace
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, 'radar_system'), ROOT]
from line_cascade import DEG, GroundLoad, ImuFirstConfig, ImuFirstGuide  # noqa: E402
from path_tracker import Path as Route, PathConfig, PathTracker, differential_config  # noqa: E402
from robot_core.kinematics import ChassisGeometry  # noqa: E402

CFG = ImuFirstConfig(rtk_trim=False, rtk_fit=True)
GEO = ChassisGeometry(drive='differential', track_m=.675, diff_min_radius_m=.5)
DT, NX, NY = .02, .23, -.4525


def feed(g, amps, metres, step=.01):
    for i in range(int(round(metres / step))):
        g.update(step, amps(i) if callable(amps) else amps)
    return g.grass


class Detect(unittest.TestCase):
    def test_flat_with_load_bumps_is_never_grass(self):
        rng = random.Random(1)
        g = GroundLoad(CFG)
        # 平地 2.2 A 上下 1 A，偶尔一拍 20 A（起步、颠一下）：中位数不动
        self.assertFalse(feed(g, lambda i: 20. if i % 7 == 0 else 2.2 + rng.uniform(-1., 1.), 20.))
        self.assertLess(g.median, 4.)

    def test_grass_recognised_once_the_window_is_full(self):
        g = GroundLoad(CFG)
        self.assertFalse(feed(g, 10., 1.5))                  # 还没走满 2 m
        self.assertIsNone(g.median)
        self.assertTrue(feed(g, 10., .6))
        self.assertAlmostEqual(g.median, 10.)

    def test_hysteresis_between_4_and_5_a(self):
        g = GroundLoad(CFG)
        feed(g, 10., 3.)
        self.assertTrue(feed(g, 4.5, 3.))                    # 4–5 A 之间：保持草地
        self.assertFalse(feed(g, 3.5, 3.))                   # ≤ 4 A：回平地
        self.assertFalse(feed(g, 4.5, 3.))                   # 4–5 A 之间：保持平地

    def test_standing_still_or_pivoting_does_not_count(self):
        g = GroundLoad(CFG)
        feed(g, 2.2, 3.)
        for _ in range(500):
            g.update(0., 20.)                                # 原地转 / 停着时电流大，不进窗口
        self.assertFalse(g.grass)
        self.assertAlmostEqual(g.median, 2.2)

    def test_lost_current_falls_back_to_flat(self):
        g = GroundLoad(CFG)
        feed(g, 10., 3.)
        self.assertFalse(g.update(.01, None))
        self.assertIsNone(g.median)
        self.assertFalse(feed(g, 10., 1.5))                  # 重新走满窗口才认

    def test_config_checks(self):
        with self.assertRaises(ValueError):
            ImuFirstConfig(grass_off_a=6., grass_on_a=5.)
        with self.assertRaises(ValueError):
            ImuFirstConfig(grass_enabled=1)
        with self.assertRaises(ValueError):
            ImuFirstConfig(grass_fit_walk_b_deg=0.)


class Walk(unittest.TestCase):
    def test_fit_angle_walk_follows_grass_weight(self):
        g = ImuFirstGuide(CFG)
        self.assertEqual(g.grass_weight, 0.)
        self.assertAlmostEqual(g._walk_b(), CFG.fit_walk_b_deg * DEG)
        g.grass_weight = 1.
        self.assertAlmostEqual(g._walk_b(), CFG.grass_fit_walk_b_deg * DEG)
        g.grass_weight = .5
        self.assertAlmostEqual(g._walk_b(), (CFG.fit_walk_b_deg + CFG.grass_fit_walk_b_deg) / 2 * DEG)
        g.reset()
        self.assertEqual(g.grass_weight, 0.)


class GM:
    """按距离（或时间）的一阶 Gauss-Markov。"""
    def __init__(self, rng, sigma, length):
        self.rng, self.s, self.L, self.x = rng, sigma, length, (rng.gauss(0, sigma) if sigma > 0 else 0.)

    def step(self, d):
        if self.s <= 0:
            return 0.
        a = math.exp(-d / self.L)
        self.x = a * self.x + self.s * math.sqrt(1 - a * a) * self.rng.gauss(0, 1)
        return self.x


def run(seed, amps, grass=True, crab_deg=.25, length=20., v=.8, paint_from=None):
    """直线 length 米：底盘 0.36 s 纯延迟；地面侧滑（车走的方向 ≠ 车头，IMU 看不见）按距离 Gauss-Markov（6 m），
    卫星横向噪声 4 mm（2 m）。amps：每拍给的两轮电流（None 不给；可按 x 给）；paint_from：从哪开喷（空 = 从头喷）。
    返回每拍 (x, 真喷头横偏, 直线状态)。"""
    rng = random.Random(seed)
    cfg = differential_config(PathConfig(cruise_mps=v))
    cfg = replace(cfg, straight_imu_first=replace(cfg.straight_imu_first, grass_enabled=grass))
    tr = PathTracker(Route((0., 0., 0.), [(length, 0.)]), cfg, GEO)
    tr.set_nozzle(NX, NY)
    if paint_from is not None:
        tr.paint_from_m = paint_from
    crab, rtk = GM(rng, crab_deg * DEG, 6.), GM(rng, .004, 2.)
    x, psi, t, vv, w, n = 0., rng.gauss(0, .3) * DEG, 0., 0., 0., 0.
    y = .003 - NX * math.sin(psi)
    sent = deque([(0., 0., 0.)])

    def pose():
        return NS(x=x, y=y + n, yaw=psi + .2 * DEG, speed=vv, yaw_rate=w, usable_for_control=True, quality='RTK_FIXED',
                  reasons=(), correction_xy=(0., 0.), correction_yaw=0., rtk_trusted=True)

    def inputs():
        tr.sync(t, (t, psi + 1.7, w))
        tr.set_heading_baseline(t, .41)
        tr.set_rtk_quality(t, .01, 28, .01, True)
        if amps is not None:
            a = amps(x) if callable(amps) else amps
            tr.set_motor_current(t, a, a)
    inputs()
    assert tr.start(pose())
    rows = []
    while t < length / v * 2 + 20 and tr.st.state == 'RUNNING':
        inputs()
        vc, wc = tr.update(pose(), DT)
        sent.append((t, vc, wc))
        while len(sent) > 1 and sent[1][0] <= t - .36:
            sent.popleft()
        _, vv, w = sent[0]
        ds = vv * DT
        c = crab.step(abs(ds))
        x += vv * math.cos(psi + c) * DT
        y += vv * math.sin(psi + c) * DT
        psi += w * DT
        t += DT
        n = rtk.step(abs(ds))
        rows.append((x, y + NX * math.sin(psi) + NY * (math.cos(psi) - 1.), dict(tr._straight_status)))
    return rows


class ClosedLoop(unittest.TestCase):
    def test_flat_current_changes_nothing(self):
        flat, none = run(3, 2.2), run(3, None)
        self.assertEqual([r[:2] for r in flat], [r[:2] for r in none])
        self.assertFalse(any(r[2].get('grass') for r in flat))

    def test_grass_switches_on_after_the_line_head_only(self):
        on, off = run(4, 10.), run(4, 10., grass=False)
        weights = [(r[0], r[2].get('grass_weight', 0.)) for r in on]
        self.assertTrue(all(wt == 0. for xx, wt in weights if xx < CFG.head_m))          # 线头段照旧
        self.assertTrue(all(wt == 1. for xx, wt in weights if CFG.head_m + CFG.head_blend_m + .1 < xx < 19.))
        self.assertTrue(any(r[2].get('grass') for r in on))
        head = [r[:2] for r in on if r[0] < CFG.head_m]
        self.assertEqual(head, [r[:2] for r in off][:len(head)])                         # 线头段一模一样

    def test_current_before_painting_is_ignored(self):
        # 助跑（还没喷）电流大不算：11-21 11:17 平地倒车助跑、慢速起步时电流来回跳到 6 A 中位
        rows = run(5, lambda x: 10. if x < 5. else 2.2, paint_from=5.)
        self.assertFalse(any(r[2].get('grass') for r in rows))
        rows = run(5, 10., paint_from=5.)
        self.assertFalse(any(r[2].get('grass') for r in rows if r[0] < 5.))
        self.assertTrue(any(r[2].get('grass') for r in rows if r[0] > 7.5))

    def test_grass_mode_tracks_slip_better(self):
        # 30 m、0.8 m/s、10 趟：中段平均横偏约少 30%，10 趟都更好（20 m 的线草地模式只开十几米，差别被噪声盖住）
        def mid_error(rows):
            es = [abs(e) for xx, e, _ in rows if 6. <= xx <= 29.]
            return sum(es) / len(es)
        on = [mid_error(run(s, 10., length=30.)) for s in range(10)]
        off = [mid_error(run(s, 10., grass=False, length=30.)) for s in range(10)]
        self.assertLess(sum(on), .8 * sum(off), (on, off))
        self.assertGreaterEqual(sum(a < b for a, b in zip(on, off)), 8)


if __name__ == '__main__':
    unittest.main()
