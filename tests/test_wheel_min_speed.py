#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""差速车轮子最低速度（2026-10-08 用户「这个轮子最低不要低于0.3最好是」）和场地助跑外伸上限 3 m。

10-06 23:44 角弧场地作业：R 1 m 转场弧 0.35 m/s，内侧轮要 0.20 m/s、实际 0.28（驱动器读数不准），车只转出 79 %，
弧上偏 12–17 cm；线头前助跑因外伸限 2 m 缩到 0.83 m，线头偏 7 cm 开喷。"""
import math
import os
import sys
import unittest
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_continuous_field_arcs import GEO  # noqa: E402
from path_tracker import Path as Route, PathConfig, PathTracker, differential_config  # noqa: E402
from motion_speeds import TURN_SPEED_MPS, PAINT_ARC_SPEED_MPS, WHEEL_MIN_MPS  # noqa: E402
import field_planner  # noqa: E402


def target(radius, work=False):
    tr = PathTracker(Route((0., 0., 0.), [(1.0, 0.0)]), differential_config(PathConfig()), GEO)
    piece = NS(kappa=-1 / radius, straight=False, s0=0., length=1.)
    tr._work_arc_piece = lambda p: work
    tr.path = NS(_piece=lambda s: piece, length=10.)
    return tr._curve_target_at(0.)


class TransitArcSpeed(unittest.TestCase):
    def inner(self, radius):
        return target(radius) * (1 - GEO.track_m / 2 / radius)

    def test_inner_wheel_kept_at_minimum(self):
        self.assertEqual(WHEEL_MIN_MPS, .3)
        for radius in (1.5, 2.0, 2.5, 3.0):
            with self.subTest(radius=radius):
                self.assertGreaterEqual(self.inner(radius), WHEEL_MIN_MPS - 1e-9)

    def test_tight_arc_capped_by_lateral_accel(self):
        v = target(1.0)                                         # 内侧轮 0.3 要 0.453 m/s，横向加速度限 0.20 → 0.447
        self.assertAlmostEqual(v, math.sqrt(.20), places=3)
        self.assertGreater(self.inner(1.0), .29)

    def test_wide_arc_unchanged(self):
        self.assertEqual(target(3.0), TURN_SPEED_MPS)

    def test_paint_arc_keeps_its_constant_speed(self):
        self.assertEqual(target(1.0, work=True), PAINT_ARC_SPEED_MPS)


class FieldRunIn(unittest.TestCase):
    def test_run_in_may_extend_three_metres_outside(self):
        cfg = field_planner.Settings(.24, .4575, (.911, .604, .39), .501)
        self.assertEqual(cfg.run_in_outside_max, 3.0)
        self.assertEqual(field_planner.OUTSIDE_SPACE_M, 2.0)     # 转场打分的场外空地没改


if __name__ == '__main__':
    unittest.main()
