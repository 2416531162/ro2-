#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""助跑限幅期间内环积分不饱和（2026-11-22 原地转进线 151131：助跑里起步有界纠偏把转向卡在 2°/s，
IMU 内环积分照样涨到 0.068 rad/s，开喷后车被这股假偏置往右带 1.5 m，草地线头 6.9 cm）。"""
import math
import os
import random
import sys
import unittest
from collections import deque
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, 'radar_system'), ROOT]

from heading_control import HeadingControl, HeadingControlConfig  # noqa: E402
from path_tracker import Path as Route, PathConfig, Section, SectionTracker, differential_config  # noqa: E402
from robot_core.kinematics import ChassisGeometry  # noqa: E402

DEG, DT = math.pi / 180, .02
NX, NY = .235, -.445
GEO = ChassisGeometry(drive='differential', track_m=.675, diff_min_radius_m=.5,
                      pivot_rate_rad_s=.8, pivot_accel_rad_s2=1.)


class Limited(unittest.TestCase):
    def core(self):
        c = HeadingControl(HeadingControlConfig(kp=2.2, ki=1.0, kd=.4, delay_s=.3))
        c.reset(now=0.)
        return c

    def test_step_pushing_into_the_clamp_is_undone(self):
        c = self.core()
        w = c.step(.02, 3 * DEG, 0., dt=.02, record=False)        # 车头偏左：要右转，积分往正（输出往负）涨
        self.assertGreater(c.integral_step, 0.)
        self.assertLess(w, -.035)
        before = c.integral - c.integral_step
        self.assertTrue(c.limited(-.035 - w))                     # 实际只下发 −0.035：被限在高处
        self.assertAlmostEqual(c.integral, before)
        self.assertEqual(c.integral_step, 0.)

    def test_step_pulling_away_from_the_clamp_is_kept(self):
        c = self.core()
        w = c.step(.02, 3 * DEG, 0., dt=.02, record=False)
        kept = c.integral
        self.assertFalse(c.limited(w - .5 - w))                   # 限在更低处：积分正往回拉，不撤
        self.assertFalse(c.limited(0.))                           # 没限幅
        self.assertFalse(c.limited(5e-7))                         # 混合的浮点误差
        self.assertEqual(c.integral, kept)


def pivot_entry(seed, g=.5, wobble_deg=.7):
    """原地转进线的作业：倒车 1 m 到助跑起点（车头偏 12–20°）→ 小原地转 → 助跑 2 m（0.35 m/s）→ 开喷 0.5 m/s。
    草地：车头只跟上指令的 g、车头扰动有界 GM（wobble_deg，0.3 s）。返回 (开喷时内环积分, 开喷后 2 m 真喷头最大横偏 m)。"""
    rng = random.Random(seed)
    cfg = differential_config(PathConfig(cruise_mps=.5))
    h = (12. + rng.uniform(0, 8)) * DEG * (1 if seed % 2 else -1)
    back = Route((math.cos(h), math.sin(h) + .03 * rng.choice((1, -1)), h + math.pi), [(1.0, 0.)])
    s1 = Section(-1, back)
    s1.field_role = 'transit'
    s2 = Section(1, Route((0., 0., 0.), [(14., 0.)]))
    s2.field_role, s2.paint_from_m, s2.paint_to_m = 'paint', 2.0, 13.5
    tr = SectionTracker([s1, s2], cfg, GEO, pivot_rate=1.5, pivot_accel=1., pivot_brake=2., reverse_gain=1.0,
                        reverse_mps=.35, settle_s=.3, paint_offset_limit_m=.03, constant_straight_speed=True,
                        constant_paint_speed=True, reverse_runup=True)
    tr.set_nozzle(NX, NY)
    x, y = back.pose(0.)[:2]
    psi = h + rng.gauss(0, .3) * DEG
    v = w = t = 0.
    sent = deque([(0., 0., 0.)])
    wd, a = 0., math.exp(-DT / .3)

    def fused():
        return NS(x=x, y=y, yaw=psi + .2 * DEG, speed=v, yaw_rate=w, usable_for_control=True, quality='RTK_FIXED',
                  reasons=(), correction_xy=(0., 0.), correction_yaw=0., rtk_trusted=True)

    def feed():
        tr.sync(t, (t, psi + 1.7, w))
        tr.set_heading_baseline(t, .41)
        tr.set_rtk_quality(t, .01, 28, .01, True)
        tr.set_motor_current(t, 12., 12.)

    feed()
    assert tr.start(fused())
    integral, head = None, []
    while t < 60 and tr.st.state == 'RUNNING':
        feed()
        vc, wc = tr.update(fused(), DT)
        cur = tr.trackers[tr.index] if tr.index == 1 and not tr.switching else None
        sent.append((t, vc, wc))
        while len(sent) > 1 and sent[1][0] <= t - .36:
            sent.popleft()
        _, v, wcmd = sent[0]
        moving = abs(v) > .05
        old = wd
        wd = a * wd + wobble_deg * DEG * math.sqrt(1 - a * a) * rng.gauss(0, 1)
        w = (g * wcmd + (wd - old) / DT) if moving else wcmd
        x += v * math.cos(psi) * DT
        y += v * math.sin(psi) * DT
        psi += w * DT
        t += DT
        if cur is not None and cur.st.s >= 2.0:
            if integral is None:
                integral = cur._straight_core.integral
            head.append(abs(y + NX * math.sin(psi) + NY * (math.cos(psi) - 1.)))
            if cur.st.s >= 4.0:
                break
    return integral, max(head)


class RunupWindup(unittest.TestCase):
    def test_grass_runup_does_not_carry_a_wound_up_integral_into_the_line_head(self):
        # 限幅不撤积分时这几条开喷积分 0.065–0.125 rad/s、线头 6.7–9.5 cm（仿真复现 151131）；撤了以后 ≤0.01、0.9–1.5 cm
        for seed in (2, 4, 9):
            with self.subTest(seed=seed):
                integral, head = pivot_entry(seed)
                self.assertIsNotNone(integral)
                self.assertLess(abs(integral), .02)
                self.assertLess(head, .025)


if __name__ == '__main__':
    unittest.main()
