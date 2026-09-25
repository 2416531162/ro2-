#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""路径跟踪（radar_system/path_tracker.py）测试：几何投影、与直线控制器一致、小圆/8 字闭环仿真。

闭环数字来自 radar_system/tools/path_sim.py（原始传感器 + 真实融合），不代表实车（没有轮胎侧滑）。
"""
import math
import os
import random
import sys
import unittest
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system', 'tools'))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, ROOT)

import path_sim  # noqa: E402
from line_tracker import LineConfig, LineTracker  # noqa: E402
from path_tracker import Path, PathConfig, PathTracker, circle, figure8, stadium  # noqa: E402

SEEDS = (1, 2, 3, 4)


def pose(x=0.0, y=0.0, yaw_deg=0.0, speed=0.0, yaw_rate=0.0, usable=True):
    return SimpleNamespace(x=x, y=y, yaw=math.radians(yaw_deg), speed=speed, yaw_rate=yaw_rate,
                           usable_for_control=usable)


class TestGeometry(unittest.TestCase):

    def test_circle_closes_on_itself(self):
        p = circle(2.0, lead_in=2.0, lead_out=1.5)
        self.assertAlmostEqual(p.length, 2.0 + 4 * math.pi + 1.5)
        a = p.pose(2.0)
        b = p.pose(2.0 + 4 * math.pi)
        for u, v in zip(a[:2], b[:2]):
            self.assertAlmostEqual(u, v, places=9)
        self.assertAlmostEqual(math.cos(a[2]), math.cos(b[2]), places=9)
        # 逆时针：圆心在入口直线左侧 2 m，四分之一圈处车头朝北
        x, y, yaw = p.pose(2.0 + math.pi)
        self.assertAlmostEqual(x, 4.0, places=9)
        self.assertAlmostEqual(y, 2.0, places=9)
        self.assertAlmostEqual(math.degrees(yaw), 90.0, places=6)

    def test_lateral_error_sign_on_arcs(self):
        # 四分之一圈处（4, 2），圆心 (2, 2)：点在圆内 = 逆时针路径的左侧 → 正；顺时针时圆内是右侧 → 负
        s, e, yaw, k = circle(2.0).project(3.9, 2.0)
        self.assertAlmostEqual(e, 0.1, places=9)
        self.assertAlmostEqual(s, 2.0 + math.pi, places=6)
        self.assertAlmostEqual(k, 0.5)
        cw = circle(2.0, left=False)
        s, e, yaw, k = cw.project(3.9, -2.0)
        self.assertAlmostEqual(e, -0.1, places=9)
        self.assertAlmostEqual(k, -0.5)

    def test_projection_stays_on_the_current_lap(self):
        # 两圈的圆：同一个位置，第二圈时不能跳回第一圈；不给提示时取起点那一圈
        p = circle(2.0, laps=2)
        lap = 4 * math.pi
        self.assertAlmostEqual(p.project(2.0, 0.01)[0], 2.0, places=3)
        self.assertAlmostEqual(p.project(2.0, 0.01, 2.0 + lap - 0.1)[0], 2.0 + lap, places=3)
        # 8 字交点：左圈转完、右圈开始都在同一点，按提示取当前这一圈
        f = figure8(2.0)
        self.assertAlmostEqual(f.project(2.0, 0.0, 2.0 + lap - 0.05)[0], 2.0 + lap, places=3)
        s, e, yaw, k = f.project(2.3, 0.0, 2.0 + lap + 0.2)
        # 右圈圆心 (2, −2)：点 (2.3, 0) 对应的弧长 = 2·(π/2 − atan2(2, 0.3))
        self.assertAlmostEqual(s, 2.0 + lap + 2 * (math.pi / 2 - math.atan2(2.0, 0.3)), places=9)
        self.assertEqual(k, -0.5)

    def test_run_up_and_overshoot_extend_the_straights(self):
        p = circle(2.5)
        self.assertAlmostEqual(p.project(-1.2, 0.05, extend_start=3.0)[0], -1.2)
        end = p.pose(p.length)
        self.assertGreater(p.project(end[0] + 0.3 * math.cos(end[2]), end[1] + 0.3 * math.sin(end[2]),
                                     p.length - 0.1)[0], p.length)

    def test_radius_below_chassis_limit_is_rejected(self):
        with self.assertRaisesRegex(ValueError, '最小半径'):
            PathTracker(circle(1.9))
        PathTracker(circle(1.95))


class TestStraightMatchesLineTracker(unittest.TestCase):
    """直线路径上，路径跟踪器必须与已经验证过的直线控制器逐拍一致。"""

    def test_same_commands_on_a_straight_path(self):
        rng = random.Random(5)
        a, b = (1.0, 2.0), (9.0, 8.0)
        yaw0 = math.atan2(b[1] - a[1], b[0] - a[0])
        line = LineTracker(LineConfig(cruise_mps=0.6))
        line.set_line(a, b)
        path = PathTracker(Path((a[0], a[1], yaw0), [(10.0, 0.0)]), PathConfig(cruise_mps=0.6))
        x, y, yaw, v = a[0] - 1.0 * math.cos(yaw0), a[1] - 1.0 * math.sin(yaw0) + 0.1, yaw0 + 0.1, 0.0
        p0 = pose(x, y, math.degrees(yaw))
        self.assertEqual(line.check_start(p0), path.check_start(p0))
        self.assertTrue(line.start(p0) and path.start(p0))
        for _ in range(500):
            p = pose(x + rng.gauss(0, 0.003), y + rng.gauss(0, 0.003), math.degrees(yaw + rng.gauss(0, 0.002)),
                     speed=v, yaw_rate=rng.gauss(0, 0.01))
            out_l, out_p = line.update(p, 0.05), path.update(p, 0.05)
            self.assertAlmostEqual(out_l[0], out_p[0], places=9)
            self.assertAlmostEqual(out_l[1], out_p[1], places=9)
            self.assertEqual(line.st.state, path.st.state)
            v, wz = out_l
            yaw += wz * 0.05
            x += v * 0.05 * math.cos(yaw)
            y += v * 0.05 * math.sin(yaw)
        self.assertEqual(path.st.state, 'DONE')


class TestClosedLoop(unittest.TestCase):

    def test_truth_pose_is_millimetre_level(self):
        # 定位完美时只剩控制律和底盘动态：小圆、跑道、8 字都在几毫米内
        for name in ('circle_r2', 'circle_r2_cw', 'stadium_r2.5', 'figure8_r2.5', 'figure8_r2'):
            with self.subTest(name):
                r = path_sim.run(name, source='truth')
                self.assertEqual(r['state'], 'DONE')
                self.assertLess(r['max_cm'], 0.6)

    def test_small_circles_with_real_fusion(self):
        for name in ('circle_r2', 'circle_r2_cw', 'circle_r2.5_2laps', 'stadium_r2.5', 'figure8_r2.5', 'figure8_r2'):
            with self.subTest(name):
                rs = [path_sim.run(name, seed=s) for s in SEEDS]
                self.assertEqual({r['state'] for r in rs}, {'DONE'})
                self.assertLess(max(r['max_cm'] for r in rs), 2.5)
                self.assertLess(max(r['rms_cm'] for r in rs), 1.2)

    def test_trim_is_learned_on_a_circle(self):
        first = path_sim.run('circle_r2.5_trim2', trim_deg=2.0)
        self.assertLess(first['max_cm'], 3.5)
        again = path_sim.run('circle_r2.5_trim2', trim_deg=2.0, seed=2, integral0=first['integral'])
        self.assertLess(again['max_cm'], first['max_cm'])


if __name__ == '__main__':
    unittest.main()
