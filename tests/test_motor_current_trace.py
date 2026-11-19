#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gen5 驱动器电流进 /oid/trace（2026-10-23：草地原地转卡住 3–8 s，日志里没电流，分不清是不是顶到最大电流）。

左轮 485 每帧读全状态顺带的电机电流、母线电流、占空比、温度存下来；右轮 CAN 每 0.1 s 多查一次电机电流；
/oid/trace 带 motor_a、bus_a、duty、temp_c（[左, 右]），0.5 s 没更新写 null。
驱动节点跑在替身 ROS 和模拟驱动器（oid_sim）上，**不是 ROS 集成测试**。
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

import oid_codec as oc  # noqa: E402
import oid_driver  # noqa: E402
import wheeltec_driver as wd  # noqa: E402
from oid_driver import DiffConfig  # noqa: E402
from oid_link import CanLink, Rs485Link, SplitLink  # noqa: E402
from oid_sim import SimModbusPort, SimOidBus  # noqa: E402
from robot_core.config import load_profile  # noqa: E402

POLES = 20
GEN5 = load_profile(os.path.join(ROOT, 'robot_core', 'robot_gen5.json'))


class TestLinkLoad(unittest.TestCase):

    def test_rs485_status_frame_keeps_current_duty_and_temperature(self):
        port = SimModbusPort(POLES, addresses=(1,))
        m = port.motors[1]
        m.motor_ca, m.bus_ca, m.duty, m.temp_c = -1234, 567, 420, 41
        link = Rs485Link(port, {1: 1})
        link.request(5.0)
        s = link.state[1]
        self.assertAlmostEqual(s.motor_current_a, -12.34)
        self.assertAlmostEqual(s.bus_current_a, 5.67)
        self.assertEqual((s.duty, s.temp_c, s.t_load), (420, 41, 5.0))
        self.assertEqual(sum(1 for r in port.requests if r[1] == 0x04), 1)     # 还是一帧读全状态，没多问

    def test_can_asks_motor_current_about_ten_times_a_second(self):
        bus = SimOidBus(POLES, 74, 75)
        bus.motors[2].motor_ca = 1550
        link = CanLink(bus, {2: 75})
        for i in range(30):                         # 30 Hz 控制周期跑 1 s
            link.request(i / 30)
            link.poll(i / 30)
        queries = [data[1] for can_id, data in bus.sent if can_id == 75 and data[0] == oc.CAN_QUERY]
        self.assertEqual(queries.count(oc.Q_SPEED), 30)
        self.assertEqual(queries.count(oc.Q_POSITION), 30)
        self.assertTrue(9 <= queries.count(oc.Q_MOTOR_CURRENT) <= 11)
        s = link.state[2]
        self.assertAlmostEqual(s.motor_current_a, 15.5)
        self.assertIsNotNone(s.t_load)
        self.assertIsNone(s.bus_current_a)          # CAN 只查电机电流


class TestTraceLoad(unittest.TestCase):
    """本车接法：左轮 485、右轮 CAN。"""

    def setUp(self):
        self.clock = clock = [100.0]
        self.port = SimModbusPort(POLES, addresses=(1,))
        self.bus = SimOidBus(POLES, 74, 75)
        overrides = dict(confirmed=True, pole_pairs=POLES, left_bus='rs485', right_bus='can',
                         rs485_left_id=1, can_right_id=75, lidar_installed=False)
        self.published = published = {}

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

        def build_link(p):
            return SplitLink([Rs485Link(self.port, {1: p['rs485_left_id']}, heartbeat_s=p['heartbeat_s']),
                              CanLink(self.bus, {2: p['can_right_id']}, heartbeat_s=p['heartbeat_s'])])

        box = lambda **k: NS(**k)  # noqa: E731
        stubs = dict(ROS_AVAILABLE=True, Node=FakeNode, ParameterDescriptor=box, Odometry=odometry,
                     Float32=box, String=box, QoSProfile=box, Duration=box,
                     HistoryPolicy=NS(KEEP_LAST=1), DurabilityPolicy=NS(VOLATILE=2), ReliabilityPolicy=NS(BEST_EFFORT=2),
                     SetBool=object, Trigger=object, LaserScan=object,
                     TransformStamped=lambda: NS(header=None, child_frame_id='', transform=NS(
                         translation=NS(x=0.0, y=0.0), rotation=None)),
                     TransformBroadcaster=lambda node: NS(sendTransform=lambda tf: None))
        patches = [mock.patch.multiple(wd, create=True, **stubs),
                   mock.patch.object(oid_driver, 'PROFILE', GEN5),
                   mock.patch.object(oid_driver, 'DiffConfig', functools.partial(
                       DiffConfig, feedback_timeout_s=GEN5['driver']['feedback_timeout_s'])),
                   mock.patch.object(oid_driver.threading, 'Thread', lambda **k: NS(start=lambda: None, join=lambda timeout: None)),
                   mock.patch.object(oid_driver, 'build_link', build_link),
                   mock.patch('time.monotonic', lambda: clock[0]),
                   mock.patch.multiple(oid_driver.OidDriver, create=True, **{
                       k: v for k, v in vars(FakeNode).items() if callable(v) and not k.startswith('__')})]
        for p in patches:
            p.start()
            self.addCleanup(p.stop)
        self.node = oid_driver.OidDriver()

    def step(self, seconds, command=None):
        dt = 1.0 / self.node.config.tx_hz
        end = self.clock[0] + seconds
        while self.clock[0] < end - 1e-9:
            if command is not None:
                data = dict(vx=command[0], wz=command[1], epoch=self.node.authority.epoch,
                            profile_hash=wd.profile_hash(wd.PROFILE), stamp=self.clock[0] - 0.01)
                self.node.on_motion('manual', wd.String(data=json.dumps(data)))
            self.node._io_step(self.clock[0])
            self.port.advance(dt)
            self.bus.advance(dt)
            self.clock[0] += dt

    def last_trace(self):
        return json.loads(self.published['/oid/trace'][-1].data)

    def test_trace_carries_both_wheels_load(self):
        left, right = self.port.motors[1], self.bus.motors[2]
        left.motor_ca, left.bus_ca, left.duty, left.temp_c = 2450, 310, 612, 38
        right.motor_ca = -2380
        self.step(4.0)
        self.step(1.0, command=(0.0, 0.8))                      # 原地转
        rec = self.last_trace()
        self.assertEqual(rec['motor_a'], [24.5, -23.8])
        self.assertEqual(rec['bus_a'], [3.1, None])             # CAN 那台只查电机电流
        self.assertEqual(rec['duty'], [612, None])
        self.assertEqual(rec['temp_c'], [38, None])

    def test_lost_current_replies_become_null(self):
        self.port.motors[1].motor_ca, self.bus.motors[2].motor_ca = 1000, 1100
        self.step(4.0)
        self.step(0.5, command=(0.3, 0.0))
        self.assertEqual(self.last_trace()['motor_a'], [10.0, 11.0])
        send = self.bus.send                                    # 右轮转速照回，只是电流查询没回：不把旧电流当现在的

        def drop_current(can_id, data):
            if not (data[0] == oc.CAN_QUERY and data[1] == oc.Q_MOTOR_CURRENT):
                send(can_id, data)
        self.bus.send = drop_current
        self.step(0.7, command=(0.3, 0.0))
        rec = self.last_trace()
        self.assertGreater(rec['t'], self.clock[0] - 0.1)        # trace 照常在发
        self.assertEqual(rec['motor_a'], [10.0, None])


if __name__ == '__main__':
    unittest.main()
