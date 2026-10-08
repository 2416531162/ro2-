#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""画线时（线头之后）慢慢修（2026-10-08 用户）：10-06 23:47 那条 37.8 m 线中途推算偏右 1 cm 多，往回拉 k_e 0.8 / 最多 1°
车头目标 2 m 内转 +0.55°、3 m 横挪约 2.5 cm，地上画出一个看得见的 S 弯。改成车头最多偏 0.3°（k_e 0.4），
1.5–2 cm 的偏差 4–6 m 内平缓收回、不冲过头；线头段（开喷后 head_m 内）照旧，线头精度不变。"""
import math
import os
import sys
import unittest
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_reverse_runup_handoff as runup  # noqa: E402
from line_cascade import ImuFirstConfig, imu_first_target, paint_pull  # noqa: E402

DT = .02
CFG = ImuFirstConfig(rtk_trim=False, rtk_fit=True)


def recover(e0, pull, v=.5, length=20.):
    """理想内环（车头 = 目标）、推算 = 真实：从 e0 收回。返回 (回到 0.5 cm 内走的路, 最大偏置°, 最小横偏, 任意 2 m 内最大横移)。"""
    g = NS(e=e0, offset=0., beta=0., fast=False, target_rate=0.)
    y, target, s, back, bias_max, y_min, hist = e0, 0., 0., None, 0., e0, []
    for _ in range(int(length / v / DT)):
        g.e = y
        target, bias, _, _ = imu_first_target(CFG, g, target, 0., DT, 3., 1., True, pull=pull)
        bias_max = max(bias_max, abs(bias))
        s += v * DT
        y += v * DT * math.sin(target)
        y_min = min(y_min, y)
        if back is None and abs(y) < .005:
            back = s
        hist.append((s, y))
    move = max(max(y for x, y in hist if a <= x <= a + 2.) - min(y for x, y in hist if a <= x <= a + 2.)
               for a in (i * .1 for i in range(int((length - 2.) / .1))))
    return back, math.degrees(bias_max), y_min, move


class PaintPull(unittest.TestCase):
    def test_line_head_keeps_old_pull_then_blends_to_gentle(self):
        self.assertEqual((CFG.head_k_e, CFG.head_max_bias_deg), (.8, 1.))     # 线头段：和原来一样
        self.assertEqual(paint_pull(CFG, 0.), (.8, 1.))
        self.assertEqual(paint_pull(CFG, CFG.head_m), (.8, 1.))
        k, lim = paint_pull(CFG, CFG.head_m + CFG.head_blend_m / 2)
        self.assertAlmostEqual(k, (.8 + CFG.k_e) / 2)
        self.assertAlmostEqual(lim, (1. + CFG.max_bias_deg) / 2)
        self.assertEqual(paint_pull(CFG, CFG.head_m + CFG.head_blend_m), (CFG.k_e, CFG.max_bias_deg))
        self.assertEqual(paint_pull(CFG, 30.), (CFG.k_e, CFG.max_bias_deg))

    def test_gentle_pull_tilts_at_most_0_3_deg_and_recovers_in_4_to_6_m_without_overshoot(self):
        pull = paint_pull(CFG, 30.)
        for e0, lo, hi in ((.015, 3.5, 6.), (.02, 4., 6.)):
            back, bias, y_min, move = recover(e0, pull)
            self.assertLessEqual(bias, .3 + 1e-9)
            self.assertTrue(lo <= back <= hi, (e0, back))
            self.assertGreater(y_min, 0.)                    # 不冲过头
            self.assertLess(move, .011)                       # 任意 2 m 内横挪不到 1.1 cm
        # 原来 0.8 / 1°：2 cm 2.5 m 就收回，2 m 内横挪 1.4 cm
        back, bias, _, move = recover(.02, (.8, 1.))
        self.assertLess(back, 3.)
        self.assertGreater(move, .013)


class TrackerPull(unittest.TestCase):
    def bias_at(self, s, e=.03, started_at=None, transit=False, paint_to=None):
        tr = runup.tracker()
        tr.runup_speed_mps = None
        if paint_to is not None:
            tr.paint_to_m, tr.paint_spans = paint_to, [(tr.paint_from_m, paint_to)]
        if started_at is not None:
            tr._started_at = started_at
        if transit:
            tr._arc_work_role = False
        g = tr._straight_guide
        g.offset, g.offset_n, g.locked, g.source = 0., 50, True, 'learned'
        for i in range(1, 120):
            t = i * DT
            tr.st.s, tr.st.e, tr.st.heading_err = s + .006 * i, e, 0.
            tr.sync(t, (t, 0., 0.))
            tr.set_heading_baseline(t, .41)
            tr.set_rtk_quality(t, .01, 28, .01, True)
            tr._straight_first(runup.fused(tr, s + .006 * i, e=e), DT, .3, 0., tr.path.pieces[0], 0., 0.)
        return abs(math.degrees(tr._straight_bias)), tr._straight_status

    def test_after_line_head_pulls_gently(self):
        tr = runup.tracker()
        head, _ = self.bias_at(tr.paint_from_m + .5)
        self.assertAlmostEqual(head, 1., places=6)            # 线头段照旧：3 cm 限到 1°
        middle, status = self.bias_at(tr.paint_from_m + 6.)
        self.assertLessEqual(middle, .3 + 1e-6)              # 之后最多 0.3°
        self.assertGreater(middle, .29)
        self.assertEqual((status['pull_k_e'], status['pull_max_bias_deg']), (CFG.k_e, CFG.max_bias_deg))

    def test_restart_mid_line_is_a_new_line_head(self):
        tr = runup.tracker()
        restart = tr.paint_from_m + 10.
        bias, _ = self.bias_at(restart + .5, started_at=restart)
        self.assertAlmostEqual(bias, 1., places=6)            # 暂停后继续处后 head_m 内照旧

    def test_run_out_after_paint_end_keeps_old_pull(self):
        tr = runup.tracker()
        bias, _ = self.bias_at(tr.paint_from_m + 5.2, paint_to=tr.paint_from_m + 5.)
        self.assertAlmostEqual(bias, 1., places=6)            # 停喷后的收尾不在喷：往回拉照旧

    def test_transit_keeps_old_pull(self):
        tr = runup.tracker()
        bias, status = self.bias_at(tr.paint_from_m + 6., transit=True)
        self.assertAlmostEqual(bias, 1., places=6)            # 转场段不喷：往回拉照旧
        self.assertEqual(status['pull_k_e'], .8)


if __name__ == '__main__':
    unittest.main()
