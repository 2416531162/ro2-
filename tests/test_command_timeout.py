#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gen5 仲裁「指令超时」只算真的没收到（2026-10-08 用户：真正超过 1.5 s 没收到指令才算超时）。

10-06 16:34 实车（路线作业 1e1baed3）：导航指令 50 Hz 准时到，驱动 IO 线程拿锁之前取的时间，比 ROS 线程
刚收到的那条指令还早一点，间隔为负被当成超时，切回空闲停车。这里复现这个先后顺序；再看断指令 1.5 s 内照常走、
超过 1.5 s 停。驱动节点（OidDriver）跑在 test_driver_silent_timeout 的替身 ROS + 模拟驱动器上。
"""
import json
import os
import sys
import unittest
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_driver_silent_timeout as harness  # noqa: E402

wd = harness.wd


class TestGen5CommandTimeout(unittest.TestCase):

    def setUp(self):
        self.h = harness.TestGen5DriverSilence('test_gen5_profile_allows_two_seconds')
        self.h.setUp()
        self.addCleanup(self.h.doCleanups)
        self.node, self.clock = self.h.node, self.h.clock
        self.h.step(4.0)                                        # 启动静止 3 s，等反馈
        res = self.node.select_motion('navigation', NS(data=True), NS(success=None, message=None))
        self.assertTrue(res.success, res.message)

    def send(self, at, lag=0.001):
        """ROS 线程在 at 时刻收到一条 lag 秒前发出的导航指令。"""
        saved, self.clock[0] = self.clock[0], at
        data = dict(vx=0.5, wz=0.0, epoch=self.node.authority.epoch,
                    profile_hash=wd.profile_hash(wd.PROFILE), stamp=at - lag)
        self.node.on_motion('navigation', wd.String(data=json.dumps(data)))
        self.clock[0] = saved

    def run_for(self, seconds, send=True):
        dt = 1.0 / self.node.config.tx_hz
        end = self.clock[0] + seconds
        while self.clock[0] < end - 1e-9:
            if send:
                self.send(self.clock[0])
            self.node._io_step(self.clock[0])
            self.h.device.advance(dt)
            self.clock[0] += dt

    def mode(self):
        return self.node.authority.mode, self.node.authority.reason

    def test_gen5_profile_timeout_is_one_and_half_seconds(self):
        self.assertEqual(harness.GEN5['safety']['command_timeout_s'], 1.5)
        self.assertEqual(self.node.authority.timeout_s, 1.5)

    def test_command_received_just_after_io_read_the_clock_is_not_a_timeout(self):
        self.run_for(1.0)
        self.assertEqual(self.mode(), ('NAVIGATION', 'operator_select'))
        t = self.clock[0]
        self.send(t + 0.004)            # IO 线程已取 now=t，ROS 线程插进来收了一条 1 ms 前发的
        self.node._io_step(t)
        self.assertEqual(self.mode(), ('NAVIGATION', 'operator_select'))
        self.assertGreater(self.node._cmd[0], 0.3)

    def test_gap_up_to_one_and_half_seconds_keeps_driving(self):
        self.run_for(1.0)
        self.run_for(1.4, send=False)
        self.assertEqual(self.mode(), ('NAVIGATION', 'operator_select'))
        self.assertGreater(self.node._cmd[0], 0.3)
        self.run_for(0.5)
        self.assertEqual(self.mode(), ('NAVIGATION', 'operator_select'))

    def test_gap_beyond_one_and_half_seconds_stops(self):
        self.run_for(1.0)
        self.run_for(1.46, send=False)
        self.assertEqual(self.mode(), ('NAVIGATION', 'operator_select'))
        self.run_for(0.1, send=False)
        self.assertEqual(self.mode(), ('IDLE', 'command_timeout'))
        self.assertEqual(self.node._cmd, (0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
