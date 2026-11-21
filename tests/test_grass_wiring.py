#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""草地模式接线（2026-11-21）：驱动器 /oid/trace 的 motor_a → line_node → 任务层 on_motor_current → 每拍交给跟踪器
set_motor_current（草地识别见 test_grass_mode）。只验证我们自己的解析和转交，不是 ROS 集成测试。"""
import json
import os
import sys
import unittest
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, ROOT)

import ros_stubs  # noqa: E402

ros_stubs.install()
sys.modules['sensor_msgs.msg'].Imu = ros_stubs._simple('Imu')

import gnss_fusion as gf  # noqa: E402
import line_mission as lm  # noqa: E402
import line_node  # noqa: E402


class Mission:
    def __init__(self):
        self.fusion = SimpleNamespace(config=gf.FusionConfig())
        self.calls = []

    def on_motor_current(self, t, left, right):
        self.calls.append((t, left, right))


def trace(**fields):
    return ros_stubs.String(json.dumps(dict({'t': 1.0, 'v_cmd': .5}, **fields)))


class NodeParsesOidTrace(unittest.TestCase):
    def test_motor_current_passed_with_receive_time(self):
        m = Mission()
        node = line_node.LineNode(m, clock=lambda: 42.0)
        node.on_oid_trace(trace(motor_a=[10.5, -9.8]))
        node.on_oid_trace(trace(motor_a=[None, 3.2]))           # 一个轮读数旧了（驱动器写 null）
        self.assertEqual(m.calls, [(42.0, 10.5, -9.8), (42.0, None, 3.2)])

    def test_bad_frames_are_dropped(self):
        m = Mission()
        node = line_node.LineNode(m, clock=lambda: 1.0)
        for msg in (ros_stubs.String('not json'), ros_stubs.String('[1, 2]'), trace(), trace(motor_a=[1.0]),
                    trace(motor_a='x')):
            node.on_oid_trace(msg)
        self.assertEqual(m.calls, [])


class MissionForwardsToTracker(unittest.TestCase):
    def test_sync_gives_latest_current_to_the_tracker(self):
        calls = []
        tracker = SimpleNamespace(sync=lambda *a: None, set_motor_current=lambda *a: calls.append(a))
        me = SimpleNamespace(_imu=(1.0, 0.0, 0.0), _antenna_lever=lambda: None, _heading_baseline=None, _rtk_quality=None,
                             _attitude=None, _motor=None, _antenna=None, _fix2=None, fusion=SimpleNamespace())
        lm.LineMission._sync_imu(me, tracker, 1.0)
        self.assertEqual(calls, [])                              # 还没有电流：不给
        lm.LineMission.on_motor_current(me, 1.02, 10.0, 11.0)
        lm.LineMission._sync_imu(me, tracker, 1.04)
        self.assertEqual(calls, [(1.02, 10.0, 11.0)])


if __name__ == '__main__':
    unittest.main()
