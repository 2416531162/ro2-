#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""工具测试:外参检查(calib_check)与离线回放(bag_replay)。不需要 ROS。"""

import json
import math
import os
import random
import sys
import types
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tests"))
sys.path.insert(0, os.path.join(ROOT, "radar_system"))

import test_follower_node as tfn  # noqa: E402  (安装 ROS 替身并提供仿真工具)
import person_follower as pf  # noqa: E402
from calib_check import CalibCollector, fit_rigid_2d, suggest_mount  # noqa: E402
from bag_replay import ReplayClock, replay  # noqa: E402

String = tfn.String


class TestCalibCheck(unittest.TestCase):

    def simulate(self, theta_deg, tx, ty, noise=0.02, seed=1):
        """真实人位置 = 雷达看到的;相机因外参错误看到的是变换后的位置。"""
        rnd = random.Random(seed)
        col = CalibCollector()
        th = math.radians(theta_deg)
        c, s = math.cos(th), math.sin(th)
        t = 0.0
        for px in (1.8, 3.0):
            for py in (-0.8, 0.0, 0.8):
                for _ in range(15):
                    t += 0.1
                    # 相机结果 p_cam 满足 p_true = R p_cam + t  => p_cam = R^T (p_true - t)
                    dx, dy = px - tx, py - ty
                    cam = (c * dx + s * dy + rnd.gauss(0, noise),
                           -s * dx + c * dy + rnd.gauss(0, noise))
                    col.add_lidar([(px + rnd.gauss(0, noise), py + rnd.gauss(0, noise)),
                                   (px + 2.0, py - 1.5)], t)
                    col.add_camera([(cam[0], cam[1], 0.9)], t + 0.03)
        return col

    def test_recovers_offset_and_yaw(self):
        col = self.simulate(4.0, 0.08, -0.05)
        res = col.solve()
        self.assertAlmostEqual(math.degrees(res['theta']), 4.0, delta=0.5)
        self.assertAlmostEqual(res['tx'], 0.08, delta=0.03)
        self.assertAlmostEqual(res['ty'], -0.05, delta=0.03)
        self.assertLess(res['rms'], 0.05)
        self.assertGreater(res['raw_mean_error_m'], 0.1)

    def test_suggested_mount_reproduces_correction(self):
        """用建议参数重新换算相机结果,应与雷达一致。"""
        from footprint import SensorMount
        th, tx, ty = math.radians(3.0), 0.06, 0.04
        x, y, yaw = suggest_mount(0.54, 0.0, 0.0, th, tx, ty)
        old = SensorMount(x_m=0.54, y_m=0.0, yaw_rad=0.0)
        new = SensorMount(x_m=x, y_m=y, yaw_rad=yaw)
        from footprint import optical_to_vehicle
        for xo, yo, zo in ((0.3, -0.2, 2.0), (-0.5, -0.1, 1.2)):
            p_old = optical_to_vehicle(xo, yo, zo, old, 0.26)
            p_new = optical_to_vehicle(xo, yo, zo, new, 0.26)
            want = (math.cos(th) * p_old[0] - math.sin(th) * p_old[1] + tx,
                    math.sin(th) * p_old[0] + math.cos(th) * p_old[1] + ty)
            self.assertAlmostEqual(p_new[0], want[0], places=9)
            self.assertAlmostEqual(p_new[1], want[1], places=9)

    def test_multiple_people_are_skipped(self):
        col = CalibCollector()
        col.add_lidar([(2.0, 0.0)], 0.0)
        self.assertFalse(col.add_camera([(2.0, 0.0, 0.9), (2.0, 1.0, 0.9)], 0.05))
        self.assertEqual(col.skipped_multi, 1)

    def test_stale_lidar_is_not_paired(self):
        col = CalibCollector()
        col.add_lidar([(2.0, 0.0)], 0.0)
        self.assertFalse(col.add_camera([(2.0, 0.0, 0.9)], 0.5))

    def test_too_few_samples(self):
        self.assertIsNone(CalibCollector().solve())

    def test_fit_needs_two_points(self):
        with self.assertRaises(ValueError):
            fit_rigid_2d([(0, 0)], [(0, 0)])


class TestBagReplay(unittest.TestCase):

    def messages(self, lose_after=None, n=120):
        """合成一段「录包」:人站在保持距离附近(车头约 1m),底盘静止;可让人消失。

        开环数据里底盘不会响应指令,人若站得远,节点会(正确地)判为堵转并开始脱困。
        """
        out = []
        t = 5000.0
        for k in range(n):
            t += 0.05
            status = String(data=json.dumps({
                'armed': True, 'ready': 'ready', 'connected': True, 'holding': False,
                'age_ms': 5, 'telemetry': {'velocity': [0.0, 0.0, 0.0]}}))
            out.append((t, '/wheeltec/status', status))
            gone = lose_after is not None and k >= lose_after
            legs = None if gone else tfn.person_legs(1.75, 0.0)
            out.append((t + 0.005, '/scan', tfn.n10p_scan(extra=legs, half_size=8.0)))
            dets = [] if gone else tfn.camera_person(1.75, 0.0)
            out.append((t + 0.01, '/camera/ai_detection/targets', String(data=json.dumps(dets))))
        return out

    def run_replay(self, msgs, **cfg_overrides):
        clock = ReplayClock(msgs[0][0])
        real = pf.time
        pf.time = clock
        try:
            h = tfn.FollowerHarness(**cfg_overrides)
            return replay(h.node, msgs, clock)
        finally:
            pf.time = real

    def test_replay_tracks_person(self):
        summary = self.run_replay(self.messages())
        self.assertGreater(summary['cycles'], 100)
        share = summary['state_share']
        self.assertGreater(share.get('HOLDING', 0) + share.get('TRACKING', 0), 0.8, summary)
        self.assertEqual(summary['lost_events'], 0)
        self.assertEqual(summary['stamp_warnings'], 0)

    def test_replay_reports_loss(self):
        summary = self.run_replay(self.messages(lose_after=60))
        self.assertEqual(summary['lost_events'], 1, summary)
        states = [s for _, s, _ in summary['transitions']]
        self.assertIn('HOLDING', states)

    def test_replay_uses_bag_clock_for_camera_stamps(self):
        msgs = self.messages()
        # 给检测结果加上「采集时刻比接收早 100ms」的时间戳
        stamped = []
        for t, topic, msg in msgs:
            if topic == '/camera/ai_detection/targets':
                items = json.loads(msg.data)
                for it in items:
                    it['stamp'] = t - 0.10
                msg = String(data=json.dumps(items))
            stamped.append((t, topic, msg))
        summary = self.run_replay(stamped)
        self.assertEqual(summary['stamp_warnings'], 0, summary)
        share = summary['state_share']
        self.assertGreater(share.get('HOLDING', 0) + share.get('TRACKING', 0), 0.8, summary)


if __name__ == '__main__':
    unittest.main()
