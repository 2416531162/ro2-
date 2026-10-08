#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""两根天线对不上时用和 IMU 推算对得上的那根（2026-10-08 用户）。

10-06 22:08 那条线（B→A）15–20 m：主天线（靠里）被干扰，融合位置往左拉了 5–9 cm，两天线距离误差 9–10 cm；副天线和
IMU 都说车没偏，地上实际也没偏。原来两天线误差 > 5 cm 就整段不用卫星。
"""
import math
import os
import sys
import unittest
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_imu_first_straight import GEO  # noqa: E402
import test_reverse_runup_handoff as runup  # noqa: E402
from path_tracker import Path as Route, PathConfig, PathTracker, differential_config  # noqa: E402

LEVER2 = (.24, -.2025)          # 副天线：主天线 (0.24, 0.2075) 往右 0.41 m


class Guide:
    """_pick_antenna 只看推算 e，换来源时清跳点参照。"""
    def __init__(self, e):
        self.e, self.changes = e, 0

    def source_changed(self):
        self.changes += 1


class PickUnit(unittest.TestCase):
    def setUp(self):
        self.tr = PathTracker(Route((0., 0., 0.), [(20., 0.)]), differential_config(PathConfig()), GEO)
        self.t = 0.

    def step(self, guide, main_y, ant2_y, pair, x=5.):
        """车头沿线（航向 0）；主天线算出的控制点横移 main_y，副天线算出的 ant2_y；返回 _pick_antenna 的结果。"""
        self.t += .2
        tr = self.tr
        tr.sync(self.t, (self.t, 0., 0.))
        tr.set_rtk_quality(self.t, .01, 28, pair, True)
        tr.set_secondary_fix(self.t, x + LEVER2[0], ant2_y + LEVER2[1], True, LEVER2)
        _, base, _, _ = tr.locate(x, main_y, 0., x)
        return tr._pick_antenna(guide, 0., x, base, base)

    def learn(self, guide, offset=.005):
        for _ in range(12):
            self.assertIsNone(self.step(guide, 0., offset, .01))      # 对得上：照旧，顺便学固定差

    def test_disturbed_main_uses_second(self):
        g = Guide(0.)
        self.learn(g)
        choice, delta = self.step(g, .06, .005, .08)
        self.assertEqual(choice, 'second')
        self.assertAlmostEqual(abs(delta), .06, places=3)
        self.assertEqual(g.changes, 1)

    def test_disturbed_second_keeps_main(self):
        g = Guide(0.)
        self.learn(g)
        self.assertEqual(self.step(g, 0., .065, .08), ('main', 0.))

    def test_neither_agrees_falls_back(self):
        g = Guide(.03)
        self.learn(g)
        self.assertIsNone(self.step(g, .06, .005, .08))

    def test_not_learned_falls_back(self):
        g = Guide(0.)
        for _ in range(5):
            self.step(g, 0., .005, .01)
        self.assertIsNone(self.step(g, .06, .005, .08))

    def test_stale_second_falls_back(self):
        g = Guide(0.)
        self.learn(g)
        self.t += 1.
        self.tr.sync(self.t, (self.t, 0., 0.))
        self.tr.set_rtk_quality(self.t, .01, 28, .08, True)
        _, base, _, _ = self.tr.locate(5., .06, 0., 5.)
        self.assertIsNone(self.tr._pick_antenna(g, 0., 5., base, base))

    def test_agreeing_antennas_change_nothing(self):
        g = Guide(0.)
        self.learn(g)
        self.assertIsNone(self.step(g, .004, .009, .02))


class StraightWithDisturbedMain(unittest.TestCase):
    """直线上真跑 _straight_first：先 6 s 两根对得上，再 3 s 主天线被拉偏 5 cm、两天线差 8 cm。"""

    def run_line(self, with_second):
        tr = runup.tracker()
        tr.runup_speed_mps = None                                   # 已过线头、正常喷线阶段（助跑另有开喷检查）
        g = tr._straight_guide
        g.offset, g.offset_n, g.locked, g.source = 0., 50, True, 'learned'
        accepted, picks, es, shifts = [], [], [], []
        for i in range(1, 451):
            t = i * .02
            s = 2.0 + .006 * i
            tr.st.s, tr.st.e, tr.st.heading_err = s, 0., 0.
            tr.sync(t, (t, 0., 0.))
            tr.set_heading_baseline(t, .41)
            disturbed = i > 300
            tr.set_rtk_quality(t, .01, 28, .08 if disturbed else .01, True)
            x, y, yaw = tr.path.pose(s)
            if with_second and i % 10 == 0:                         # 副天线 5 Hz，一直是真位置
                c, sn = math.cos(yaw), math.sin(yaw)
                tr.set_secondary_fix(t, x + c * LEVER2[0] - sn * LEVER2[1], y + sn * LEVER2[0] + c * LEVER2[1], True, LEVER2)
            pose = runup.fused(tr, s, e=.05 if disturbed else 0.)  # 融合跟着主天线被拉偏
            tr._straight_first(pose, .02, .3, 0., tr.path.pieces[0], 0., 0.)
            if i > 360:
                accepted.append(g.rtk_accepted)
                picks.append(tr._antenna['choice'])
                es.append(g.e)
                shifts.append(tr.position_correction())
        self.shifts = shifts
        return accepted, picks, es

    def test_second_antenna_keeps_correction_on(self):
        accepted, picks, es = self.run_line(with_second=True)
        self.assertGreater(sum(accepted) / len(accepted), .9)
        self.assertGreater(picks.count('second') / len(picks), .9)
        self.assertLess(max(abs(e) for e in es), .01)                 # 推算没被拉偏的主天线带走

    def test_reported_position_follows_second_antenna(self):
        """报给 app 的位置（地图喷线、作业报告）：融合被主天线拉偏 5 cm，用副天线时挪回 5 cm。"""
        self.run_line(with_second=True)
        used = [sh for sh in self.shifts if sh is not None]
        self.assertGreater(len(used) / len(self.shifts), .9)
        dx, dy = used[-1]
        self.assertAlmostEqual(dx, 0., places=3)
        self.assertAlmostEqual(dy, -.05, places=3)                   # 线沿 +x，融合偏左 5 cm → 往右挪 5 cm

    def test_mission_reports_corrected_position(self):
        import inspect
        import line_mission
        src = inspect.getsource(line_mission.LineMission.status)
        self.assertIn('position_correction', src)
        self.assertIn('antenna_corrected_m', src)

    def test_without_second_antenna_correction_stops_as_before(self):
        accepted, picks, es = self.run_line(with_second=False)
        self.assertLess(sum(accepted) / len(accepted), .1)
        self.assertEqual(set(picks), {None})
        self.assertEqual(set(self.shifts), {None})


if __name__ == '__main__':
    unittest.main()
