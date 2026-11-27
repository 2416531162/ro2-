#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""11-24 楼下的线不直（用户「写补丁先仿真」）。两件事：

1. 跨线带过去的 IMU 换算（ImuAlignment）：车停着时 IMU 偏航的变化是 IMU 自己的（06:58 固件静止修正一下拨了 +4.12°，
   陀螺一直 0），换算跟着挪、不算转角；跨线学习（漂移、刻度）删掉（用户「删掉学习」），只带上一条线的。
   原来学成漂移 +0.35°/min、刻度 −0.44%，后面带过去的换算错 1.7–3.4°。
2. 楼下模式（ImuFirstConfig.poor_sky_*）：σ 低通 > 2.5 cm（11-27 1.5 → 2.5，user_rules.POOR_SKY_SIGMA_M）、不是草地，这条直线开头 6 m 以后靠 IMU 和对好的角度走——
   线头段之后不往回拉，拟合角度用到车头上每米最多改 0.08°（user_rules.POOR_SKY_BETA_RATE_DEG）；拟合角度对不上（> 0.6°）、卫星说偏 > 6 cm 走满 1 m、
   认出草地就退回平常。"""
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
from line_cascade import DEG, AlignmentConfig, ImuAlignment, ImuFirstConfig, ImuFirstGuide, paint_pull  # noqa: E402
from path_tracker import Path as Route, PathConfig, PathTracker, differential_config  # noqa: E402
from robot_core.kinematics import ChassisGeometry  # noqa: E402
import user_rules as rules  # noqa: E402

CFG = ImuFirstConfig(rtk_trim=False, rtk_fit=True)
POOR_SIGMA = rules.POOR_SKY_SIGMA_M + .003     # 闭环里接收机报的楼下 σ：比进楼下门槛高一点（11-27 前门槛 1.5 cm 时用的 1.8 cm）
GEO = ChassisGeometry(drive='differential', track_m=.675, diff_min_radius_m=.5)
DT, NX, NY = .02, .23, -.4525


def feed(a, t0, t1, yaw, still=False, hz=50.):
    """t0 → t1 每 1/hz 秒一帧 IMU 偏航（yaw 可以是 t 的函数）。"""
    n = int(round((t1 - t0) * hz))
    for i in range(n + 1):
        t = t0 + i / hz
        a.feed_imu(t, yaw(t) if callable(yaw) else yaw, still=still)
    return t1


class StillShift(unittest.TestCase):
    def test_imu_step_while_standing_moves_the_carried_c(self):
        a = ImuAlignment(AlignmentConfig())
        feed(a, 0., 1., 0.)
        tok = a.begin_line()
        a.update_line(tok, 1., -158. * DEG, .05 * DEG)
        a.begin_line()
        theta = a.theta
        # 停着：固件静止修正把偏航一下拨了 +4.12°（11-24 06:58），陀螺 0
        feed(a, 1.02, 30., lambda t: 0. if t < 10. else 4.12 * DEG, still=True)
        c, sigma = a.predict(30.)
        self.assertAlmostEqual(math.degrees(c), -158. - 4.12, places=6)   # 真车头没变：c = 真 − IMU 跟着挪
        self.assertAlmostEqual(a.theta, theta)                            # 不算转角
        self.assertAlmostEqual(math.degrees(a.still_shift), 4.12, places=6)

    def test_turning_counts_as_turn_and_keeps_c(self):
        a = ImuAlignment(AlignmentConfig())
        feed(a, 0., 1., 0.)
        tok = a.begin_line()
        a.update_line(tok, 1., -158. * DEG, .05 * DEG)
        a.begin_line()
        feed(a, 1.02, 5., lambda t: (t - 1.) * 45. * DEG)                 # 原地转 180°（在动）
        c, _ = a.predict(5.)
        self.assertAlmostEqual(math.degrees(c), -158., places=6)
        self.assertAlmostEqual(math.degrees(a.theta), 180., places=3)
        self.assertEqual(a.still_shift, 0.)

    def test_pending_of_the_current_line_also_moves(self):
        a = ImuAlignment(AlignmentConfig())
        feed(a, 0., 1., 0.)
        tok = a.begin_line()
        a.update_line(tok, 1., 10. * DEG, .05 * DEG)                       # 这条线还没定下来（暂停停着）
        feed(a, 1.02, 3., 1. * DEG, still=True)
        self.assertAlmostEqual(math.degrees(a.value), 9., places=6)


class NoLearning(unittest.TestCase):
    def test_jump_between_lines_is_not_learned(self):
        # 11-24：两条线之间换算差 +1.8°、−3.9°（固件那一拨）——原来学成漂移 / 刻度，后面每条线都带错；现在只带上一条线的
        a = ImuAlignment(AlignmentConfig())
        for t, theta, c in ((0., 0., -155.95), (138., 5., -154.17), (300., -180., -158.05), (480., 0., -157.87)):
            a.theta, a._imu_t = theta * DEG, t
            tok = a.begin_line()
            p = a.predict(t)
            if t == 480.:
                self.assertAlmostEqual(math.degrees(p[0]), -158.05, places=9)
            a.update_line(tok, t, c * DEG, .05 * DEG)
        self.assertFalse(hasattr(a, 'x'))

    def test_uncertainty_still_grows_with_time_and_turns(self):
        a = ImuAlignment(AlignmentConfig())
        a.theta, a._imu_t = 0., 0.
        tok = a.begin_line()
        a.update_line(tok, 0., 0., .05 * DEG)
        a.begin_line()
        s0 = a.predict(1.)[1]
        s1 = a.predict(300.)[1]
        a.theta = 10 * math.pi
        s2 = a.predict(300.)[1]
        self.assertLess(s0, s1)
        self.assertLess(s1, s2)

    def test_config_checks(self):
        with self.assertRaises(ValueError):
            AlignmentConfig(still_max_gyro_deg_s=0.)
        with self.assertRaises(TypeError):
            AlignmentConfig(drift_deg_min=0.)            # 学习删了，没有这项


class MissionStill(unittest.TestCase):
    def mission(self):
        import line_mission as lm
        me = NS(_wheel_still=None, _wheel_t=None, _imu_alignment=ImuAlignment())
        return lm, me

    def test_still_after_half_a_second_of_stopped_wheels(self):
        lm, me = self.mission()
        lm.LineMission._note_wheel(me, 0., 0., 0.)
        self.assertFalse(lm.LineMission._still(me, .2, 0.))              # 还没停够 0.5 s
        lm.LineMission._note_wheel(me, .6, 0., 0.)
        self.assertTrue(lm.LineMission._still(me, .6, 0.))
        self.assertFalse(lm.LineMission._still(me, .6, math.radians(2.)))  # 陀螺在转：不算停（被人推着转、轮子打滑）
        self.assertFalse(lm.LineMission._still(me, 1.5, 0.))             # 轮速断了

    def test_moving_or_pivoting_is_not_still(self):
        lm, me = self.mission()
        lm.LineMission._note_wheel(me, 0., .3, 0.)
        lm.LineMission._note_wheel(me, 1., .3, 0.)
        self.assertFalse(lm.LineMission._still(me, 1., 0.))
        lm.LineMission._note_wheel(me, 2., 0., math.radians(30.))      # 原地转：车速 0、轮子算的转速大
        lm.LineMission._note_wheel(me, 3., 0., math.radians(30.))
        self.assertFalse(lm.LineMission._still(me, 3., 0.))
        lm.LineMission._note_wheel(me, 3.1, float('nan'), 0.)
        self.assertIsNone(me._wheel_still)


class Guide(unittest.TestCase):
    def test_poor_sky_latches_and_grass_or_disabled_blocks(self):
        g = ImuFirstGuide(CFG)
        g.set_poor_sky(False, False)
        self.assertFalse(g.poor_sky)
        g.set_poor_sky(True, False)
        self.assertTrue(g.poor_sky)
        g.set_poor_sky(False, False)                     # 一条线里进了不退（σ 来回跳不来回切）
        self.assertTrue(g.poor_sky)
        g.set_poor_sky(True, True)                       # 认出草地：退，这条线不再进
        self.assertFalse(g.poor_sky)
        self.assertEqual(g.poor_sky_exit, 'grass')
        g.set_poor_sky(True, False)
        self.assertFalse(g.poor_sky)
        g.reset()
        self.assertFalse(g.poor_sky)
        self.assertIsNone(g.poor_sky_exit)
        g = ImuFirstGuide(replace(CFG, poor_sky_enabled=False))
        g.set_poor_sky(True, False)
        self.assertFalse(g.poor_sky)

    def test_pull_after_the_head_only(self):
        for along in (0., 1., CFG.head_m):
            self.assertEqual(paint_pull(CFG, along, 0.), paint_pull(CFG, along))          # 线头段照旧
        self.assertEqual(paint_pull(CFG, CFG.head_m + CFG.head_blend_m + 1., 0.), (0., 0.))
        k, cap = paint_pull(CFG, CFG.head_m + CFG.head_blend_m / 2, 0.)
        self.assertAlmostEqual(k, CFG.head_k_e / 2)
        self.assertAlmostEqual(cap, CFG.head_max_bias_deg / 2)

    def test_config_checks(self):
        with self.assertRaises(ValueError):
            ImuFirstConfig(poor_sky_enabled=1)
        with self.assertRaises(ValueError):
            ImuFirstConfig(poor_sky_pull_scale=1.5)
        for name in ('poor_sky_sigma_m', 'poor_sky_beta_rate_deg', 'poor_sky_exit_deg', 'poor_sky_exit_e_m', 'poor_sky_exit_m'):
            with self.assertRaises(ValueError):
                ImuFirstConfig(**{name: 0.})

    def test_rules_table(self):
        self.assertEqual(CFG.poor_sky_sigma_m, rules.POOR_SKY_SIGMA_M)
        self.assertEqual(CFG.poor_sky_exit_e_m, rules.POOR_SKY_EXIT_E_M)


def guide_line(poor, slope_deg=0., offset_after=None, length=30., seed=1):
    """单独跑 ImuFirstGuide：IMU 直走（rel = 0），卫星横偏 = 真横偏 + 噪声；slope_deg：真走向比 IMU 参考多偏这么多；
    offset_after：(从哪米起, 真横偏多出多少 m)。返回 (每 0.5 m 的 (路程, β°), guide)。"""
    rng = random.Random(seed)
    g = ImuFirstGuide(CFG)
    g.locked, g.offset, g.source = True, 0., 'learned'
    x, out = 0., []
    step = .5 * DT
    while x < length:
        g.set_poor_sky(poor, False)
        true = math.tan(slope_deg * DEG) * x + (offset_after[1] if offset_after and x >= offset_after[0] else 0.)
        g.update(true + rng.gauss(0, .002), 0., step, DT, (NX, NY), True, fast=x < 2.)
        x += step
        if abs(x / .5 - round(x / .5)) < 1e-6:
            out.append((round(x, 2), math.degrees(g.beta)))
    return out, g


class GuideRates(unittest.TestCase):
    def test_slow_angle_after_the_start_only(self):
        # 真走向比 IMU 参考偏 0.2°（不闭环，15 m 横偏到 5 cm，不到出口）：开头 6 m 两边一样按卫星对角度；
        # 楼下模式 6 m 后每米最多改 poor_sky_beta_rate_deg
        off, g0 = guide_line(False, .2, length=15.)
        on, g1 = guide_line(True, .2, length=15.)
        self.assertEqual([b for s, b in off if s <= CFG.fit_start_m - .5], [b for s, b in on if s <= CFG.fit_start_m - .5])
        after = [b for s, b in on if s >= CFG.fit_start_m]
        steps = [abs(b - a) for a, b in zip(after, after[1:])]
        self.assertLessEqual(max(steps), CFG.poor_sky_beta_rate_deg * .5 + 1e-6)
        self.assertTrue(g1.poor_sky)

    def test_big_angle_mismatch_leaves_poor_sky(self):
        # 带过来的角度错 1.5°：拟合看出来、和正在用的差超过 0.6°，退回平常
        _, g = guide_line(True, 1.5)
        self.assertEqual(g.poor_sky_exit, 'angle')
        self.assertFalse(g.poor_sky)

    def test_real_offset_leaves_poor_sky(self):
        # 车真被推出去 8 cm（IMU 看不见）：卫星同一边 > 6 cm 走满 1 m，退回平常往回拉
        _, g = guide_line(True, 0., offset_after=(12., .08))
        self.assertEqual(g.poor_sky_exit, 'offset')
        _, g = guide_line(True, 0., offset_after=(12., .03))               # 3 cm（楼下卫星自己偏也有）：不退
        self.assertTrue(g.poor_sky)
        self.assertIsNone(g.poor_sky_exit)


class GM:
    """按距离的一阶 Gauss-Markov。"""
    def __init__(self, rng, sigma, length):
        self.rng, self.s, self.L, self.x = rng, sigma, length, (rng.gauss(0, sigma) if sigma > 0 else 0.)

    def step(self, d):
        if self.s <= 0:
            return 0.
        a = math.exp(-d / self.L)
        self.x = a * self.x + self.s * math.sqrt(1 - a * a) * self.rng.gauss(0, 1)
        return self.x


def run(seed, sigma_m, poor_enabled=True, rtk_sigma=.012, rtk_len=4., length=30., v=.5, slip=None):
    """直线 length 米、0.36 s 纯延迟底盘；卫星横向误差按距离 Gauss-Markov（rtk_sigma、rtk_len：楼下多路径几米长的慢偏），
    接收机报的 σ = sigma_m。slip：(从哪米起, 车真往左挪多少 m，IMU 看不见)。返回每拍 (x, 真喷头横偏, 直线状态)。"""
    rng = random.Random(seed)
    cfg = differential_config(PathConfig(cruise_mps=v))
    cfg = replace(cfg, straight_imu_first=replace(cfg.straight_imu_first, poor_sky_enabled=poor_enabled))
    tr = PathTracker(Route((0., 0., 0.), [(length, 0.)]), cfg, GEO)
    tr.set_nozzle(NX, NY)
    rtk = GM(rng, rtk_sigma, rtk_len)
    x, psi, t, vv, w, n = 0., rng.gauss(0, .3) * DEG, 0., 0., 0., 0.
    y = .003 - NX * math.sin(psi)
    sent = deque([(0., 0., 0.)])
    pushed = [0.]

    def pose():
        return NS(x=x, y=y + n, yaw=psi + .2 * DEG, speed=vv, yaw_rate=w, usable_for_control=True, quality='RTK_FIXED',
                  reasons=(), correction_xy=(0., 0.), correction_yaw=0., rtk_trusted=True)

    def inputs():
        tr.sync(t, (t, psi + 1.7, w))
        tr.set_heading_baseline(t, .41)
        tr.set_rtk_quality(t, sigma_m, 28, .01, True)
        tr.set_motor_current(t, 2.2, 2.2)
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
        x += vv * math.cos(psi) * DT
        y += vv * math.sin(psi) * DT
        if slip is not None and x >= slip[0] and pushed[0] < slip[1]:
            d = min(slip[1] - pushed[0], .1 * abs(ds))              # 每走 1 m 被推 10 cm，推满为止
            y += d
            pushed[0] += d
        psi += w * DT
        t += DT
        n = rtk.step(abs(ds))
        rows.append((x, y + NX * math.sin(psi) + NY * (math.cos(psi) - 1.), dict(tr._straight_status)))
    return rows


def bow(rows, start=None, end=None):
    """喷线段（线头段之后）去掉首尾连线后的峰峰值，m。"""
    start = CFG.head_m if start is None else start
    pts = [(xx, e) for xx, e, _ in rows if start <= xx <= (end if end is not None else rows[-1][0] - .5)]
    (x0, e0), (x1, e1) = pts[0], pts[-1]
    r = [e - (e0 + (e1 - e0) * (xx - x0) / (x1 - x0)) for xx, e in pts]
    return max(r) - min(r)


class ClosedLoop(unittest.TestCase):
    def test_good_sky_is_unchanged(self):
        on, off = run(3, .0118), run(3, .0118, poor_enabled=False)
        self.assertEqual([r[:2] for r in on], [r[:2] for r in off])
        self.assertFalse(any(r[2].get('first_poor_sky') for r in on))

    def test_1_8_cm_sigma_is_no_longer_poor_sky(self):
        # 11-27 用户「超过 1.5 cm 这个改成 2.5」：56300f6a 那块场地 σ 1.8–1.9 cm 整条线都判成楼下、2 cm 不修；现在照平常走
        self.assertEqual(rules.POOR_SKY_SIGMA_M, .025)
        on, off = run(3, .018), run(3, .018, poor_enabled=False)
        self.assertEqual([r[:2] for r in on], [r[:2] for r in off])
        self.assertFalse(any(r[2].get('first_poor_sky') for r in on))
        self.assertTrue(any(r[2].get('first_poor_sky') for r in run(3, POOR_SIGMA)))

    def test_poor_sky_is_straighter_and_head_unchanged(self):
        # 楼下：卫星 1.2 cm、4 m 长的慢偏，σ 过门槛。8 趟鼓包平均少一截，线头段一样
        on = [run(s, POOR_SIGMA) for s in range(8)]
        off = [run(s, POOR_SIGMA, poor_enabled=False) for s in range(8)]
        b_on, b_off = [bow(r) for r in on], [bow(r) for r in off]
        self.assertLess(sum(b_on), .8 * sum(b_off), (b_on, b_off))
        for a, b in zip(on, off):
            head = [r[:2] for r in a if r[0] < CFG.head_m]
            self.assertEqual(head, [r[:2] for r in b][:len(head)])
        self.assertTrue(all(any(r[2].get('first_poor_sky') for r in rows) for rows in on))

    def test_real_push_is_pulled_back(self):
        # 楼下模式里车真被推出去 8 cm（IMU 看不见）：退回平常、拉回来，线尾离线比不退小得多
        rows = run(2, POOR_SIGMA, rtk_sigma=.003, slip=(12., .08))
        self.assertIn('offset', {r[2].get('first_poor_sky_exit') for r in rows})
        tail = [abs(e) for xx, e, _ in rows if xx > 26.]
        self.assertLess(max(tail), .04)


if __name__ == '__main__':
    unittest.main()
