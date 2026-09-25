#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跟随灵敏度闭环回归:radar_system/tools/follow_sim.py 的场景 + 验收门限。

仿真带相机 70ms 延迟、N10P 10Hz、腿部摆动、指令 60ms 死区和底盘斜坡,
门限取自 2026-09-24 调参后的结果并留余量。它只能防止算法退化,不能代替实车验收。
"""
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system', 'tools'))

import follow_sim  # noqa: E402


def run(name, **overrides):
    return follow_sim.run_scenario(name, overrides)[1]


class TestKeepsUpWithWalker(unittest.TestCase):

    def test_normal_and_fast_walk_is_followed_without_losing(self):
        """人以 1.0 / 1.3 m/s 走开:不设低速上限,按间距加减速,全程不丢。"""
        for name in ('walk_away_1.0', 'walk_away_1.3'):
            with self.subTest(name):
                m = run(name)
                self.assertEqual(m['tracked'], 1.0, m)
                self.assertEqual(m['lost_events'], 0, m)
                self.assertLess(m['max_gap'], 3.6, m)
                self.assertLess(abs(m['end_gap'] - 1.2), 0.3, m)
                self.assertLess(m['react_s'], 0.8, m)

    def test_sudden_stop_keeps_distance(self):
        m = run('sudden_stop')
        self.assertGreater(m['min_person_clear'], 0.6, m)
        self.assertLess(m['end_gap'], 1.5, m)


class TestLidarHandoff(unittest.TestCase):

    def test_person_leaving_camera_view_is_tracked_by_lidar(self):
        m = run('exit_fov')
        self.assertEqual(m['tracked'], 1.0, m)
        self.assertTrue(m['same_id'], m)
        self.assertGreater(m['lidar_frac'], 0.1, m)
        self.assertLess(m['max_cam_blind_s'], 3.0, m)
        self.assertLess(m['end_bearing'], 10.0, m)

    def test_passer_by_does_not_steal_lidar_track(self):
        m = run('distractor')
        self.assertTrue(m['same_id'], m)
        self.assertEqual(m['tracked'], 1.0, m)

    def test_lidar_track_stays_on_person_among_chair_legs(self):
        """走出视野后贴着椅子腿走、拐弯:轨迹不能被吸到椅子上(同编号也不行)。

        chair_row 在未剔除静止杂物时部分随机种子会偏 2.9m;行人卡尔曼加速度噪声
        取 1.5 时 clutter_handoff 偏 1.9m。多种子都要贴住人。
        """
        for name in ('clutter_handoff', 'chair_row'):
            for seed in (1, 2, 3, 4):
                with self.subTest(name=name, seed=seed):
                    m = follow_sim.run_scenario(name, seed=seed)[1]
                    self.assertTrue(m['same_id'], m)
                    self.assertEqual(m['lost_events'], 0, m)
                    self.assertLess(m['max_track_err'], 0.5, m)
                    self.assertLess(m['max_cam_blind_s'], 3.0, m)


class TestRearTurnaround(unittest.TestCase):

    def test_person_walking_behind_is_faced_again_quickly(self):
        """人绕到车后:多段前进/倒车掉头,相机重新看到人不超过 4.5s,左右对称。"""
        results = {}
        for name in ('walk_behind_left', 'walk_behind_right'):
            with self.subTest(name):
                m = results[name] = run(name)
                self.assertEqual(m['tracked'], 1.0, m)
                self.assertTrue(m['same_id'], m)
                self.assertLess(m['max_cam_blind_s'], 4.5, m)
                self.assertLess(m['end_bearing'], 20.0, m)
                self.assertGreater(m['min_person_clear'], 0.3, m)
                self.assertGreater(m['min_wall'], 0.3, m)
        left, right = results['walk_behind_left'], results['walk_behind_right']
        self.assertLess(abs(left['max_cam_blind_s'] - right['max_cam_blind_s']), 0.5)

    def test_person_walking_away_behind_is_followed_after_turning(self):
        m = run('behind_walk_away')
        self.assertEqual(m['lost_events'], 0, m)
        self.assertTrue(m['same_id'], m)
        self.assertLess(m['max_cam_blind_s'], 6.0, m)
        self.assertLess(m['end_gap'], 1.6, m)
        self.assertGreater(m['min_person_clear'], 0.3, m)


if __name__ == '__main__':
    unittest.main()


class TestOperatorSelection(unittest.TestCase):

    def test_tapping_other_person_switches_target(self):
        """画面里两个人:app 点选另一个人的方位,目标切过去,车转向他。"""
        import math
        x0 = follow_sim.front_start(1.8)
        a = follow_sim.Walker([(0, x0, 0.25), (30, x0, 0.25)])
        b = follow_sim.Walker([(0, x0 + 0.4, -0.9), (30, x0 + 0.4, -0.9)])
        world = follow_sim.World([a, b], room=(-3, -4, 8, 4))
        sim = follow_sim.Sim(world, a)
        sim.run(1.5)
        e = sim.engine
        first = e.people.target_id
        self.assertIsNotNone(first)
        s = follow_sim.SENS
        bearing = math.atan2(-0.9 - s['camera_y_m'], x0 + 0.4 - s['camera_x_m'])
        chosen = e.select_target(bearing)
        self.assertIsNotNone(chosen)
        self.assertNotEqual(chosen, first)
        self.assertEqual(e.people.target_id, chosen)
        self.assertIsNone(e.select_target(math.radians(25)), '空白处点选不应换人')
        self.assertEqual(e.people.target_id, chosen)
        sim.target = b
        sim.run(4.0)
        self.assertEqual(e.people.target_id, chosen)
        last = sim.samples[-1]
        # 阿克曼车停在跟随距离时不能原地转正;要求车头已转向新目标、相机仍看得到他
        self.assertEqual(last['source'], 'camera', last)
        self.assertLess(last['car'][2], -0.1, last)
        self.assertLess(abs(last['gap'] - 1.2), 0.3, last)
