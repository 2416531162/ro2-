#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""直线卫星纠偏的卫星数门槛（2026-10-08 用户）：低于 17 颗不信；17–19 颗只要 σ ≤ 2.7 cm（上午定 2.0，下午改 2.7）、
两天线位置对得上（< 3 cm）、固定解就照常信；20 颗以上照旧（σ ≤ 2.5 cm，两天线 ≤ 5 cm 单独数）。

10-06 16:32 那趟（路线作业 83db1b28）：往东北开，靠楼一侧的主天线掉到 17–19 颗，σ ≤ 2.0 cm、两天线 ≤ 2.9 cm、一直固定解，
被 20 颗挡掉，18–26 m 卫星纠偏没开，按 IMU 偏了 2.5 cm。
"""
import inspect
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_imu_first_straight import GEO  # noqa: E402
import test_reverse_runup_handoff as runup  # noqa: E402
from path_tracker import Path as Route, PathConfig, PathTracker, differential_config  # noqa: E402


def tracker():
    tr = PathTracker(Route((0., 0., 0.), [(20., 0.)]), differential_config(PathConfig()), GEO)
    tr.sync(0., (0., 0., 0.))
    return tr


class SatsBand(unittest.TestCase):

    def feed(self, tr, frames, t0=0.):
        for i, q in enumerate(frames):
            tr.set_rtk_quality(t0 + i * .05, *q)
        return t0 + len(frames) * .05

    def test_floor_and_band_in_config(self):
        first = tracker().cfg.straight_imu_first
        self.assertEqual((first.fit_min_sats, first.fit_full_sats), (17, 20))
        self.assertEqual((first.fit_low_sats_max_sigma_m, first.fit_low_sats_max_pair_m), (.027, .03))

    def test_1632_frames_are_trusted(self):
        tr = tracker()
        t = self.feed(tr, [(.019, 17, .029, True), (.018, 18, .022, True), (.019, 19, .014, True)] * 20)
        self.assertIsNone(tr._quality_bad_t)
        tr.sync(t, (t, 0., 0.))
        self.assertTrue(tr._rtk_quality_ok())

    def test_band_needs_all_three(self):
        for bad in ((.028, 18, .01, True),           # σ 超 2.7 cm
                    (.015, 18, .03, True),           # 两天线差到 3 cm
                    (.015, 18, None, True),          # 这帧没有两天线对比，证明不了
                    (.015, 18, .01, False),          # 不是固定解
                    (.015, 18, .01, None)):          # 不知道是不是固定解
            with self.subTest(bad=bad):
                tr = tracker()
                self.feed(tr, [bad, bad])
                self.assertIsNotNone(tr._quality_bad_t)

    def test_2208_frames_are_trusted(self):
        """10-06 22:08 那条线 8–14 m：17–19 颗、σ 2.2–2.4 cm、两天线 ≤ 2.4 cm、固定解（卫星是对的，2.0 时被挡）。"""
        tr = tracker()
        t = self.feed(tr, [(.022, 18, .02, True), (.024, 17, .024, True), (.026, 19, .015, True)] * 20)
        self.assertIsNone(tr._quality_bad_t)
        tr.sync(t, (t, 0., 0.))
        self.assertTrue(tr._rtk_quality_ok())

    def test_below_17_never_trusted(self):
        tr = tracker()
        self.feed(tr, [(.008, 16, .002, True)] * 2)
        self.assertIsNotNone(tr._quality_bad_t)

    def test_20_and_more_unchanged(self):
        tr = tracker()
        t = self.feed(tr, [(.024, 20, .045, None), (.024, 23, .045, False)] * 10)
        self.assertIsNone(tr._quality_bad_t)
        self.assertIsNone(tr._pair_bad_t)
        tr.sync(t, (t, 0., 0.))
        self.assertTrue(tr._rtk_quality_ok())

    def test_runup_trust_uses_the_same_band(self):
        tr = runup.tracker()
        p = runup.fused(tr)
        tr.sync(1., (1., 0., 0.))
        tr.set_heading_baseline(1., .41)
        tr.set_rtk_quality(1., .019, 18, .02, True)
        self.assertTrue(tr._runup_position_trusted(p))
        tr.set_rtk_quality(1.05, .019, 18, .035, True)
        self.assertFalse(tr._runup_position_trusted(p))

    def test_mission_passes_fixed_flag(self):
        import line_mission
        src = inspect.getsource(line_mission.LineMission.on_gnss)
        self.assertIn('pair, sol in RTK_FIXED_SOLS)', src)


if __name__ == '__main__':
    unittest.main()
