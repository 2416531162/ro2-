#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""走直线控制器(radar_system/line_tracker.py)：单元测试 + 闭环仿真验收。不需要 ROS。

仿真定位误差含 RTK 慢漂（车跟的是测得的位置，这部分控制器消不掉），所以真值横偏的门限
比"控制器相对测量值"的误差宽；数字见 radar_system/tools/line_sim.py 的场景表。
"""
import math
import os
import sys
import unittest
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, os.path.join(ROOT, 'radar_system', 'tools'))
sys.path.insert(0, ROOT)

from line_tracker import LineConfig, LineTracker  # noqa: E402
import line_sim  # noqa: E402

SEEDS = range(1, 8)


def pose(x=0.0, y=0.0, yaw_deg=0.0, speed=0.0, usable=True):
    return SimpleNamespace(x=x, y=y, yaw=math.radians(yaw_deg), speed=speed, yaw_rate=0.0,
                           usable_for_control=usable)


def worst(name, key, **extra):
    kw = dict(line_sim.SCENARIOS[name], **extra)
    return max(line_sim.run(name, seed=s, **kw)[key] for s in SEEDS)


class TestGeometryAndStart(unittest.TestCase):

    def setUp(self):
        self.t = LineTracker()
        self.t.set_line((0.0, 0.0), (10.0, 10.0))      # 东北方向 45°

    def test_signs(self):
        e, he, s = self.t.geometry_of(5.0, 6.0, math.radians(50))
        self.assertGreater(e, 0.0)                      # 在线的左侧
        self.assertAlmostEqual(math.degrees(he), 5.0)   # 车头偏左
        self.assertAlmostEqual(s, 11.0 / math.sqrt(2))

    def test_start_checks(self):
        t = LineTracker()
        t.set_line((0.0, 0.0), (20.0, 0.0))
        self.assertEqual(t.check_start(pose(0.0, 0.8)), (False, 'too_far_from_line'))
        self.assertEqual(t.check_start(pose(0.0, 0.0, 40)), (False, 'heading_not_aligned'))
        self.assertEqual(t.check_start(pose(-4.0, 0.0)), (False, 'too_far_behind_a'))
        self.assertEqual(t.check_start(pose(19.8, 0.0)), (False, 'already_past_b'))
        self.assertEqual(t.check_start(pose(usable=False)), (False, 'pose_unusable'))
        self.assertEqual(t.check_start(pose(-1.0, 0.2, 10)), (True, ''))

    def test_short_line_rejected(self):
        with self.assertRaises(ValueError):
            self.t.set_line((0.0, 0.0), (0.5, 0.0))


class TestControlLaw(unittest.TestCase):

    def run_ticks(self, tracker, p, n=40):
        out = None
        for _ in range(n):
            out = tracker.update(p, 0.05)
        return out

    def test_steers_back_toward_line(self):
        t = LineTracker()
        t.set_line((0.0, 0.0), (20.0, 0.0))
        self.assertTrue(t.start(pose(1.0, 0.2, 0, speed=0.5)))
        v, wz = self.run_ticks(t, pose(1.0, 0.2, 0, speed=0.5))
        self.assertGreater(v, 0.0)
        self.assertLess(wz, 0.0)                         # 在左边 → 右转

    def test_curvature_limited_and_speed_capped(self):
        t = LineTracker(LineConfig(cruise_mps=0.8))
        t.set_line((0.0, 0.0), (20.0, 0.0))
        t.start(pose(1.0, -0.45, -25, speed=0.8))
        v, wz = self.run_ticks(t, pose(1.0, -0.45, -25, speed=0.8))
        self.assertLessEqual(abs(wz / v), t.kappa_max + 1e-9)
        self.assertLess(v, 0.8)                          # 偏离大时降速

    def test_unusable_pose_stops_immediately_and_waits_for_operator(self):
        t = LineTracker()
        t.set_line((0.0, 0.0), (20.0, 0.0))
        t.start(pose(1.0, 0.0, 0, speed=0.5))
        self.run_ticks(t, pose(1.0, 0.0, 0, speed=0.5), 5)
        self.assertEqual(t.update(pose(1.0, 0.0, 0, speed=0.5, usable=False), 0.05), (0.0, 0.0))
        self.assertEqual(t.st.state, 'PAUSED')
        # 定位恢复也不自动走
        self.assertEqual(t.update(pose(1.0, 0.0, 0, speed=0.0), 0.05), (0.0, 0.0))
        self.assertEqual(t.st.reason, 'pose_recovered_waiting_operator')
        self.assertTrue(t.start(pose(1.0, 0.0, 0)))
        self.assertEqual(t.st.state, 'RUNNING')

    def test_off_line_aborts(self):
        t = LineTracker()
        t.set_line((0.0, 0.0), (20.0, 0.0))
        t.start(pose(1.0, 0.0, 0, speed=0.5))
        self.assertEqual(t.update(pose(3.0, 0.6, 0, speed=0.5), 0.05), (0.0, 0.0))
        self.assertEqual((t.st.state, t.st.reason), ('ABORTED', 'off_line'))

    def test_operator_stop(self):
        t = LineTracker()
        t.set_line((0.0, 0.0), (20.0, 0.0))
        t.start(pose(1.0, 0.0, 0))
        self.assertEqual(t.stop(), (0.0, 0.0))
        self.assertEqual(t.st.state, 'ABORTED')
        self.assertEqual(t.update(pose(1.0, 0.0, 0), 0.05), (0.0, 0.0))


class TestClosedLoop(unittest.TestCase):
    """20 m 直线、0.5 m/s，底盘含 60 ms 延迟、舵机速率和 0.2° 转角量化。"""

    def test_ideal_has_no_systematic_error(self):
        r = line_sim.run('ideal', **line_sim.SCENARIOS['ideal'])
        self.assertEqual(r['state'], 'DONE')
        self.assertLess(r['max_cm'], 0.1)

    def test_nominal_straightness(self):
        self.assertLess(worst('nominal', 'rms_cm'), 1.0)
        self.assertLess(worst('nominal', 'max_cm'), 2.0)
        self.assertLess(worst('nominal', 'ctrl_rms_cm'), 1.0)

    def test_stops_at_b(self):
        for s in SEEDS:
            r = line_sim.run('nominal', seed=s)
            self.assertEqual(r['state'], 'DONE')
            self.assertLess(abs(r['end_err_cm']), 3.0)

    def test_converges_from_offset_start(self):
        # 起点在 A 后 2.5 m、左偏 30 cm、车头偏 10°；从 A 后 2 m 起计
        self.assertLess(worst('offset_start', 'max_cm'), 2.5)

    def test_faster_cruise(self):
        self.assertLess(worst('fast', 'max_cm'), 2.0)

    def test_servo_trim_is_learned(self):
        self.assertLess(worst('trim_2deg', 'settled_max_cm'), 2.0)
        r = line_sim.run('trim_2deg', **line_sim.SCENARIOS['trim_2deg'])
        self.assertAlmostEqual(r['trim_learned_deg'], -2.0, delta=0.4)
        # 下一趟从学到的零位起步，一开始就直
        again = line_sim.run('trim_2deg', seed=2, integral0=r['integral'], **line_sim.SCENARIOS['trim_2deg'])
        self.assertLess(again['max_cm'], 1.5)

    def test_rough_ground(self):
        # 压力测试：随机曲率 σ 0.03/m 是假设的强扰动（约等于舵角叠加近 1° 的随机晃动），不是实测。
        # 只要求不跑偏、不中止；真实路面的扰动量要到室外测，再决定要不要提高 omega_per_m。
        for s in SEEDS:
            self.assertEqual(line_sim.run('rough', seed=s, **line_sim.SCENARIOS['rough'])['state'], 'DONE')
        self.assertLess(worst('rough', 'max_cm'), 3.5)

    def test_pose_loss_stops_the_car(self):
        for s in SEEDS:
            r = line_sim.run('outage', seed=s, **line_sim.SCENARIOS['outage'])
            self.assertEqual(r['state'], 'PAUSED')
            self.assertIsNotNone(r['paused_stop_s'])
            # 0.5 m/s：60 ms 延迟 + 1.5 m/s² 刹车，约 11 cm
            self.assertLess(r['stop_x'] - r['outage_x'], 0.20)


if __name__ == '__main__':
    unittest.main()
