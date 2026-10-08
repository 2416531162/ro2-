#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""助跑段（线头之前，还没喷）更积极地拉回线上（2026-10-08 用户）：推算直接往卫星靠，往回拉 k_e 2.0 / 车头最多 2°，
开喷以后线头段照旧（k_e 0.8 / 1°，之后慢慢修见 test_gentle_correction）。10-06 下午 6 趟直线线头偏 0.5–2.1 cm：
助跑起点就偏 1–3 cm，锁车头 1 m 后只剩 ~1.7 m。"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_reverse_runup_handoff as runup  # noqa: E402
from line_cascade import ImuFirstConfig, ImuFirstGuide  # noqa: E402

DT = .02


class PullTo(unittest.TestCase):
    def test_estimate_moves_toward_satellite_and_fit_follows(self):
        g = ImuFirstGuide(ImuFirstConfig(rtk_trim=False, rtk_fit=True))
        g.offset, g.offset_n, g.locked, g.source = 0., 50, True, 'learned'
        g.update(0., 0., 0., DT, (.24, -.4575), True)
        for _ in range(100):                                    # 0.6 m，tau 0.3 m：靠拢 ~86 %
            g.update(.02, 0., .006, DT, (.24, -.4575), True)
            g.pull_to(.02, 1. - math.exp(-.006 / .3))
        self.assertGreater(g.e, .017)
        c_now = g._fit_now(g._travel)[0]
        self.assertAlmostEqual(g._fit['c'], g.e - (g._fit['e0'] + g._fit['imu']), places=9)
        self.assertGreater(c_now, .015)                        # 拟合的修正量跟着挪了，开喷后接着用


class RunupLaw(unittest.TestCase):
    def bias_at(self, s, e=.03):
        tr = runup.tracker()
        tr.runup_speed_mps = None
        g = tr._straight_guide
        g.offset, g.offset_n, g.locked, g.source = 0., 50, True, 'learned'
        for i in range(1, 120):
            t = i * DT
            tr.st.s, tr.st.e, tr.st.heading_err = s + .006 * i, e, 0.
            tr.sync(t, (t, 0., 0.))
            tr.set_heading_baseline(t, .41)
            tr.set_rtk_quality(t, .01, 28, .01, True)
            tr._straight_first(runup.fused(tr, s + .006 * i, e=e), DT, .3, 0., tr.path.pieces[0], 0., 0.)
        return abs(math.degrees(tr._straight_bias)), g.e

    def test_before_line_head_pulls_harder(self):
        tr = runup.tracker()
        before, e_before = self.bias_at(tr.paint_from_m - 1.2)
        after, _ = self.bias_at(tr.paint_from_m + .5)
        self.assertGreater(before, 1.5)                       # 3 cm：k_e 2.0 → 3.4°，限到 2°
        self.assertLessEqual(after, 1.0 + 1e-6)               # 开喷后线头段照旧最多 1°
        self.assertGreater(e_before, .02)                     # 助跑段推算已经靠到卫星的 3 cm


if __name__ == '__main__':
    unittest.main()
