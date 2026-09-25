#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""掉头规划器(radar_system/turnaround.py)单元测试:纯几何,不需要 ROS。"""
import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))

from turnaround import TurnaroundPlanner, stroke_time, reach_time  # noqa: E402
from motion_safety import ChassisGeometry, yaw_from_steer  # noqa: E402
from footprint import VehicleFootprint  # noqa: E402

OPEN = lambda d, steer: 2.0  # noqa: E731  雷达认证净空充足


def planner(**kw):
    return TurnaroundPlanner(ChassisGeometry(), VehicleFootprint(margin_m=0.025), **kw)


def wall_x(x, half=3.0):
    return [(x, y / 20.0) for y in range(int(-half * 20), int(half * 20) + 1)]


class TestArcKinematics(unittest.TestCase):

    def test_arc_matches_shared_kinematics_both_directions(self):
        """规划弧线与 yaw_from_steer 数值积分一致:倒车反打舵时车头与前进同向转。"""
        p = planner()
        g = p.geo
        for direction in (1, -1):
            steer = direction * g.max_steer_rad
            x = y = th = 0.0
            ds = 0.001
            for _ in range(1200):
                wz = yaw_from_steer(float(direction), steer, g)
                x += direction * ds * math.cos(th + wz * ds / 2)
                y += direction * ds * math.sin(th + wz * ds / 2)
                th += wz * ds
            end = p._arc((0.0, 0.0, 0.0), direction, 1, 1.2)[0][-1]
            self.assertAlmostEqual(end[0], x, places=3)
            self.assertAlmostEqual(end[1], y, places=3)
            self.assertAlmostEqual(end[2], th, places=3)
            self.assertGreater(th, 0.0)

    def test_time_models(self):
        self.assertAlmostEqual(stroke_time(0.0, 1.0, 1.5, 1.0), 0.0)
        # 足够长时:匀速段 + 两端斜坡
        self.assertAlmostEqual(stroke_time(3.0, 1.0, 1.0, 1.0), 4.0)
        self.assertLess(reach_time(1.0, 1.0, 1.5), stroke_time(1.0, 1.0, 1.5, 1.0))


class TestPlans(unittest.TestCase):

    def test_person_beside_car_reverses_with_counter_steer(self):
        """人贴在车左侧:前进满舵会让方位角变大,应倒车右打舵,一段转正。"""
        plan = planner(view_x_m=0.54).plan((0.0, 1.4), first_clearance=OPEN)
        self.assertTrue(plan.reached)
        self.assertEqual(plan.direction, -1)
        self.assertLess(plan.steer, 0.0)
        self.assertEqual(len(plan.segments), 1)
        self.assertLess(plan.eta_s, 2.5)

    def test_right_side_is_mirror_of_left(self):
        p = planner(view_x_m=0.54)
        left = p.plan((-1.5, 0.8), first_clearance=OPEN)
        right = p.plan((-1.5, -0.8), first_clearance=OPEN)
        self.assertEqual(left.direction, right.direction)
        self.assertAlmostEqual(left.steer, -right.steer)
        self.assertAlmostEqual(left.eta_s, right.eta_s, places=6)

    def test_person_behind_is_reached_within_physical_limit(self):
        """正后方 2m:满舵半径 1.72m 下转 160° 至少要走 ~4.8m,规划应在 7s 内。"""
        plan = planner(view_x_m=0.54).plan((-2.0, 0.0), first_clearance=OPEN)
        self.assertTrue(plan.reached)
        self.assertLess(plan.eta_s, 7.0)
        dirs = [d for d, _, _ in plan.segments]
        self.assertTrue(all(a != b for a, b in zip(dirs, dirs[1:])), dirs)

    def test_blocked_front_starts_with_reverse(self):
        plan = planner().plan((-2.0, 0.6), wall_x(0.75),
                              first_clearance=lambda d, s: 0.0 if d > 0 else 2.0)
        self.assertEqual(plan.direction, -1)

    def test_blocked_rear_starts_forward(self):
        plan = planner().plan((-1.2, 1.0), wall_x(-0.25),
                              first_clearance=lambda d, s: 0.0 if d < 0 else 2.0)
        self.assertEqual(plan.direction, 1)
        self.assertGreater(plan.steer, 0.0)

    def test_first_stroke_never_exceeds_certified_clearance(self):
        p = planner()
        plan = p.plan((0.0, 1.4), first_clearance=lambda d, s: 0.55)
        self.assertLessEqual(plan.stroke_m, 0.55 - p.first_buffer + 1e-9)

    def test_boxed_in_returns_no_motion(self):
        plan = planner().plan((-2.0, 0.5), first_clearance=lambda d, s: 0.05)
        self.assertEqual(plan.direction, 0)

    def test_already_close_obstacle_allows_moving_away(self):
        """车头已贴着障碍物(比余量还近)时,远离它的倒车不应被判死。"""
        plan = planner().plan((-2.0, 0.6), [(0.70, y / 20.0) for y in range(-6, 7)],
                              first_clearance=lambda d, s: 0.0 if d > 0 else 2.0)
        self.assertEqual(plan.direction, -1)

    def test_planned_path_keeps_margin_from_person_and_walls(self):
        p = planner(view_x_m=0.54)
        walls = wall_x(1.6) + [(x / 20.0, -1.4) for x in range(-60, 33)]
        target = (-1.6, 0.5)
        plan = p.plan(target, walls, first_clearance=OPEN)
        self.assertNotEqual(plan.direction, 0)
        pose = (0.0, 0.0, 0.0)
        import numpy as np
        for d, steer, length in plan.segments:
            turn = 1 if (steer > 0) == (d > 0) else -1
            poses, _ = p._arc(pose, d, turn, length)
            self.assertGreaterEqual(float(np.min(p._rect_gap(poses, np.asarray(walls)))),
                                    p.obstacle_margin - 1e-9)
            self.assertGreaterEqual(float(np.min(p._rect_gap(poses, np.asarray([target])))),
                                    p.person_margin - 1e-9)
            pose = tuple(poses[-1])

    def test_moving_forward_prefers_continuing(self):
        """停着时倒车更快;已在前进时换向要先刹停,应改为继续前进。"""
        p = planner(view_x_m=0.54)
        still = p.plan((1.0, 2.5), first_clearance=OPEN, current_speed=0.0)
        moving = p.plan((1.0, 2.5), first_clearance=OPEN, current_speed=0.9)
        self.assertEqual(still.direction, -1)
        self.assertEqual(moving.direction, 1)
        self.assertGreater(moving.eta_s, still.eta_s)

    def test_plan_is_fast_enough_for_control_loop(self):
        """三面墙 + 正后方目标(两个转向都要搜)是最重的情形;开发机约 8ms。

        取中位数防止后台负载抖动误报;门限只拦数量级退化,板端要另测。
        """
        import time
        p = planner(view_x_m=0.54)
        walls = wall_x(2.0) + wall_x(-2.5) + [(x / 20.0, 2.0) for x in range(-50, 41)]
        times = []
        for _ in range(10):
            start = time.perf_counter()
            p.plan((-1.8, 0.2), walls, first_clearance=OPEN)
            times.append(time.perf_counter() - start)
        self.assertLess(sorted(times)[5], 0.020)


if __name__ == '__main__':
    unittest.main()
