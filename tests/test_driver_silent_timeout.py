#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gen5 驱动器没回话的判定（2026-10-06 用户：不超过 2 s 没回话不报故障、继续作业）。

驱动节点（OidDriver）在替身 ROS 和模拟驱动器（oid_sim，同一条 CAN 上左 74、右 75）上跑：
右轮驱动器一段时间不回话，看仲裁是否报 driver_silent、车是否停。**不是 ROS 集成测试**。
"""
import functools
import json
import os
import sys
import unittest
from types import SimpleNamespace as NS
from unittest import mock

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'oid_protocol'))
sys.path.insert(0, os.path.join(ROOT, 'wheeltec_protocol'))
sys.path.insert(0, ROOT)

import oid_driver  # noqa: E402
import wheeltec_driver as wd  # noqa: E402
from oid_driver import DiffConfig  # noqa: E402
from oid_link import CanLink  # noqa: E402
from oid_sim import SimOidBus  # noqa: E402
from robot_core.config import load_profile  # noqa: E402

POLES = 15
GEN5 = load_profile(os.path.join(ROOT, 'robot_core', 'robot_gen5.json'))


class TestGen5DriverSilence(unittest.TestCase):

    def setUp(self):
        self.clock = clock = [100.0]
        self.device = SimOidBus(POLES, 74, 75)
        overrides = dict(confirmed=True, pole_pairs=POLES, can_left_id=74, can_right_id=75, lidar_installed=False)
        published = {}

        class Pub:
            def __init__(self, topic):
                self.topic = topic

            def publish(self, msg):
                published.setdefault(self.topic, []).append(msg)

        class FakeNode:
            def __init__(self, name):
                self._params = {}

            def declare_parameter(self, name, value, descriptor=None):
                self._params[name] = overrides.get(name, value)

            def get_parameter(self, name):
                return NS(value=self._params[name])

            def create_publisher(self, typ, topic, qos):
                return Pub(topic)

            def create_subscription(self, *a, **k):
                return None

            def create_service(self, *a, **k):
                return None

            def create_timer(self, *a, **k):
                return None

            def get_clock(self):
                return NS(now=lambda: NS(nanoseconds=int(clock[0] * 1e9), to_msg=lambda: NS(sec=0, nanosec=0)))

        def odometry():
            pose = NS(position=NS(x=0.0, y=0.0), orientation=NS(z=0.0, w=1.0))
            return NS(header=NS(stamp=None, frame_id=''), child_frame_id='',
                      pose=NS(pose=pose, covariance=[0.0] * 36),
                      twist=NS(twist=NS(linear=NS(x=0.0, y=0.0), angular=NS(z=0.0)), covariance=[0.0] * 36))

        box = lambda **k: NS(**k)  # noqa: E731
        stubs = dict(ROS_AVAILABLE=True, Node=FakeNode, ParameterDescriptor=box, Odometry=odometry,
                     Float32=box, String=box, QoSProfile=box, Duration=box,
                     HistoryPolicy=NS(KEEP_LAST=1), DurabilityPolicy=NS(VOLATILE=2), ReliabilityPolicy=NS(BEST_EFFORT=2),
                     SetBool=object, Trigger=object, LaserScan=object,
                     TransformStamped=lambda: NS(header=None, child_frame_id='', transform=NS(
                         translation=NS(x=0.0, y=0.0), rotation=None)),
                     TransformBroadcaster=lambda node: NS(sendTransform=lambda tf: None))
        # 车上按 robot_gen5.json 取反馈超时（DiffConfig 的默认值来自导入时的车型配置，测试环境是旧车）
        patches = [mock.patch.multiple(wd, create=True, **stubs),
                   mock.patch.object(oid_driver, 'PROFILE', GEN5),
                   mock.patch.object(oid_driver, 'DiffConfig', functools.partial(
                       DiffConfig, feedback_timeout_s=GEN5['driver']['feedback_timeout_s'])),
                   mock.patch.object(oid_driver.threading, 'Thread', lambda **k: NS(start=lambda: None, join=lambda timeout: None)),
                   mock.patch.object(oid_driver, 'build_link',
                                     lambda p: CanLink(self.device, (p['can_left_id'], p['can_right_id']), None,
                                                       heartbeat_s=p['heartbeat_s'])),
                   mock.patch('time.monotonic', lambda: clock[0]),
                   mock.patch.multiple(oid_driver.OidDriver, create=True, **{
                       k: v for k, v in vars(FakeNode).items() if callable(v) and not k.startswith('__')})]
        for p in patches:
            p.start()
            self.addCleanup(p.stop)
        self.node = oid_driver.OidDriver()
        self.faults = []

    def step(self, seconds, command=None):
        dt = 1.0 / self.node.config.tx_hz
        end = self.clock[0] + seconds
        while self.clock[0] < end - 1e-9:
            if command is not None:
                data = dict(vx=command[0], wz=command[1], epoch=self.node.authority.epoch,
                            profile_hash=wd.profile_hash(wd.PROFILE), stamp=self.clock[0] - 0.01)
                self.node.on_motion('manual', wd.String(data=json.dumps(data)))
            self.node._io_step(self.clock[0])
            self.faults.extend(self.node.motion_health_faults)
            self.device.advance(dt)
            self.clock[0] += dt

    def drive(self):
        self.step(4.0)                                          # 启动静止 3 s，等反馈
        self.step(1.0, command=(0.3, 0.0))
        self.assertEqual(self.node.authority.mode, 'MANUAL')
        self.faults.clear()

    def test_gen5_profile_allows_two_seconds(self):
        self.assertEqual(GEN5['driver']['feedback_timeout_s'], 2.0)
        self.assertEqual(self.node.config.feedback_timeout_s, 2.0)
        with self.assertRaises(ValueError):
            DiffConfig(feedback_timeout_s=2.1)

    def test_right_driver_silent_for_one_and_half_seconds_keeps_driving(self):
        # 08:14 实车：右轮（CAN）回话断 1.1 s，1 s 判定时报了 driver_silent:右轮 并停了一下
        self.drive()
        self.device.motors[2].powered = False
        self.step(1.5, command=(0.3, 0.0))
        self.assertEqual(self.faults, [])
        self.assertEqual(self.node.authority.mode, 'MANUAL')
        self.assertTrue(self.node.policy.armed)
        self.assertGreater(abs(self.device.motors[1].speed), 50)   # 左轮照常按指令转
        self.device.motors[2].powered = True
        self.step(1.0, command=(0.3, 0.0))
        self.assertEqual(self.faults, [])
        self.assertEqual(self.node.authority.mode, 'MANUAL')
        self.assertGreater(min(abs(m.speed) for m in self.device.motors.values()), 50)

    def test_right_driver_silent_beyond_two_seconds_is_still_a_fault(self):
        self.drive()
        self.device.motors[2].powered = False
        self.step(1.9, command=(0.3, 0.0))
        self.assertEqual(self.faults, [])
        self.step(0.3, command=(0.3, 0.0))
        self.assertIn('driver_silent:右轮', self.faults)
        self.step(1.2, command=(0.3, 0.0))
        self.assertEqual(self.node.authority.mode, 'FAULT')
        self.assertFalse(self.node.policy.armed)
        self.assertLess(abs(self.device.motors[1].speed), 1.0)    # 左轮也停了


if __name__ == '__main__':
    unittest.main()
