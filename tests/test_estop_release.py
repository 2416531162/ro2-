#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""急停时轮子还在动就让驱动器放空（2026-11-19 地面事故：驱动器零速闭环自己约 1 Hz 振荡，急停只发零速压不住）。

真实 OidDriver 节点跑在替身 ROS 和模拟驱动器（oid_sim）上，不连接、不转动实车轮子。
「零速闭环振荡」用模拟电机表示：转速模式、目标 0 时转速按正弦来回摆（驱动器自己出力）；放空（电流 0）后才停。
"""
import json
import math
import unittest
from types import SimpleNamespace as NS

import test_driver_silent_timeout as fixtures  # 先导入：它把 oid_protocol、wheeltec_protocol 加进 sys.path
import oid_codec as oc  # noqa: E402
import oid_driver  # noqa: E402
import wheeltec_driver as wd  # noqa: E402


def hunt(motor, amplitude_erpm, hz=1.0):
    """模拟驱动器零速保持环振荡：转速模式下目标为 0 时自己来回转；放空、心跳超时或有非零目标时按原来的速度环。"""
    normal = motor.step

    def step(t, dt):
        if motor.alive(t) and motor.speed_mode and not motor.fault and motor.target == 0:
            motor.speed = amplitude_erpm * math.sin(2 * math.pi * hz * t)
            motor.position += motor.speed / motor.pole_pairs * 360.0 / 60.0 * 100.0 * dt
        else:
            normal(t, dt)
    motor.step = step


class TestEstopRelease(unittest.TestCase):
    setUp = fixtures.TestGen5DriverSilence.setUp

    def step(self, seconds, command=None, until=None):
        """推进 seconds 秒；until(node) 为真时提前停，返回停下时距开始的秒数（没停返回 None）。"""
        dt = 1.0 / self.node.config.tx_hz
        start, end = self.clock[0], self.clock[0] + seconds
        while self.clock[0] < end - 1e-9:
            if command is not None:
                data = dict(vx=command[0], wz=command[1], epoch=self.node.authority.epoch,
                            profile_hash=wd.profile_hash(wd.PROFILE), stamp=self.clock[0] - .01)
                self.node.on_motion('manual', wd.String(data=json.dumps(data)))
            self.node._io_step(self.clock[0])
            self.device.advance(dt)
            self.clock[0] += dt
            if until is not None and until(self.node):
                return self.clock[0] - start
        return None

    def drive(self, speed):
        self.step(4.0)                              # 启动静止 3 s，等反馈、停稳放空
        self.step(1.5, (speed, 0.0))
        self.assertEqual(self.node.authority.mode, 'MANUAL')
        self.assertTrue(all(m.speed_mode for m in self.device.motors.values()))

    def current_zero_frames(self):
        return [can_id for can_id, data in self.device.sent if data[0] == oc.CAN_CURRENT]

    def motion(self):
        return self.node.motion_extensions(self.clock[0])

    def test_hunting_driver_is_released_about_half_a_second_after_estop(self):
        self.drive(.3)
        for m in self.device.motors.values():
            hunt(m, self.node.wheels.to_erpm(.4))
        self.node.on_stop(None, NS())
        self.assertEqual(self.node.authority.mode, 'ESTOP')
        self.assertIsNone(self.step(.4, until=lambda n: n.released))     # 先按零速刹车
        self.assertTrue(all(m.speed_mode for m in self.device.motors.values()))
        took = self.step(.5, until=lambda n: n.released)
        self.assertIsNotNone(took, '轮子一直在动，约 0.5 s 应放空')
        self.assertLessEqual(.4 + took, .65)
        self.assertFalse(any(m.speed_mode for m in self.device.motors.values()))    # 驱动器收到电流 0，不再出力
        self.assertEqual(self.motion()['estop_release_reason'], 'wheels_moving')
        self.assertTrue(self.motion()['estop_released'] and self.motion()['wheels_released'])
        self.step(2.0)
        self.assertTrue(self.node.released)                                 # 放空后不再恢复零速闭环
        self.assertFalse(any(m.speed_mode for m in self.device.motors.values()))
        self.assertEqual(self.node.authority.mode, 'ESTOP')
        self.assertLess(max(abs(m.speed) for m in self.device.motors.values()), 1.0)

    def test_parked_car_hunting_since_startup_is_released_by_estop(self):
        # 11-19 事故：开机后零速闭环就振荡，空闲时永远停不稳、不会自动放空（这一条没改）；按急停后约 0.5 s 放空
        for m in self.device.motors.values():
            hunt(m, self.node.wheels.to_erpm(.5))
        self.step(5.0)
        self.assertEqual(self.node.authority.mode, 'IDLE')
        self.assertFalse(self.node.released)
        self.node.on_stop(None, NS())
        took = self.step(1.0, until=lambda n: n.released)
        self.assertIsNotNone(took)
        self.assertLessEqual(took, .7)
        self.assertEqual(self.motion()['estop_release_reason'], 'wheels_moving')
        self.assertFalse(any(m.speed_mode for m in self.device.motors.values()))

    def test_wheels_that_stop_keep_zero_brake_and_release_after_one_second_still(self):
        self.drive(.6)
        self.node.on_stop(None, NS())
        still_at, released_at = None, None
        dt = 1.0 / self.node.config.tx_hz
        for i in range(int(3.0 / dt)):
            self.step(dt)
            t = (i + 1) * dt
            if still_at is None and self.node.policy.stationary_since is not None:
                still_at = t
            if self.node.released:
                released_at = t
                break
            self.assertTrue(all(m.speed_mode for m in self.device.motors.values()))     # 零速刹车
            self.assertFalse(self.motion()['estop_released'])
        self.assertIsNotNone(still_at)
        self.assertIsNotNone(released_at)
        self.assertGreaterEqual(released_at - still_at, oid_driver.RELEASE_STATIONARY_S - dt)
        self.assertGreater(released_at, 1.0)
        self.assertFalse(self.motion()['estop_released'])                   # 走的是原来的停稳放空
        self.assertIsNone(self.motion()['estop_release_reason'])

    def test_hard_stop_from_high_speed_is_normal_braking_not_hunting(self):
        # 1.2 m/s、减速度 2 m/s² 要刹 0.6 s 多：比 0.5 s 长，也不能当成零速压不住提前放空
        self.drive(1.2)
        self.assertGreater(min(abs(m.speed) for m in self.device.motors.values()), self.node.wheels.to_erpm(1.0))
        self.node.on_stop(None, NS())
        self.assertIsNone(self.step(1.0, until=lambda n: n.released))
        self.assertTrue(all(m.speed_mode for m in self.device.motors.values()))
        self.assertLess(max(abs(m.speed) for m in self.device.motors.values()), 1.0)    # 已靠零速刹停
        self.assertIsNotNone(self.step(2.0, until=lambda n: n.released))
        self.assertFalse(self.motion()['estop_released'])

    def test_silent_drivers_get_release_commands(self):
        self.drive(.3)
        self.step(.5, (0.0, 0.0))
        self.node.on_stop(None, NS())
        self.device.motors[2].powered = False                               # 右轮驱动器不回话
        self.device.sent.clear()
        self.step(self.node.config.feedback_timeout_s - .2)
        self.assertNotIn(75, self.current_zero_frames())                    # 不到反馈超时照旧零速刹车
        self.step(.5)
        self.assertEqual(self.motion()['estop_release_reason'], 'driver_silent')
        self.assertTrue(self.node.released)
        self.assertIn(74, self.current_zero_frames())
        self.assertIn(75, self.current_zero_frames())
        sent = len(self.current_zero_frames())
        self.step(1.0)                                                       # 还不回话：隔 0.5 s 重发
        self.assertGreaterEqual(len(self.current_zero_frames()) - sent, 2 * 2)
        heartbeats = [d for _, d in self.device.sent[-20:] if d[0] in (oc.CAN_HEARTBEAT, oc.CAN_SPEED)]
        self.assertFalse(heartbeats)                                          # 放空后不再发心跳和转速

    def test_reset_after_estop_release_restores_drive(self):
        self.drive(.3)
        for m in self.device.motors.values():
            hunt(m, self.node.wheels.to_erpm(.4))
        self.node.on_stop(None, NS())
        self.assertIsNotNone(self.step(1.0, until=lambda n: n.motion_extensions(self.clock[0])['estop_released']))
        self.step(2.0)                                                       # 放空后停下来、确认静止
        for m in self.device.motors.values():                                # 驱动器处理好了（重新上电）：零速不再振荡
            del m.step
        result = self.node.reset_motion(None, NS())
        self.assertTrue(result.success, result.message)
        self.assertEqual(self.node.authority.mode, 'IDLE')
        self.assertFalse(self.motion()['estop_released'])
        self.assertIsNone(self.motion()['estop_release_reason'])
        self.assertTrue(self.node.released)                                  # 没有指令时保持放空
        self.step(1.0, (.3, 0.0))                                            # 再次取得控制：恢复速度闭环
        self.assertEqual(self.node.authority.mode, 'MANUAL')
        self.assertFalse(self.node.released)
        self.assertTrue(all(m.speed_mode for m in self.device.motors.values()))
        self.assertGreater(min(abs(m.speed) for m in self.device.motors.values()), 50)

    def test_reset_is_refused_in_chinese_while_wheels_still_move(self):
        self.drive(.3)
        self.node.on_stop(None, NS())
        result = self.node.reset_motion(None, NS())                          # 车还在滑：复位照常检查
        self.assertFalse(result.success)
        self.assertEqual(result.message, '复位没成功：请等传感器和底盘正常、车停稳后再复位')

    def test_normal_stop_without_estop_is_unchanged(self):
        # 不是急停（普通停车 / 作业暂停）：轮子还在动也不提前放空，照原来零速闭环
        self.drive(.3)
        for m in self.device.motors.values():
            hunt(m, self.node.wheels.to_erpm(.4))
        self.step(2.0, (0.0, 0.0))
        self.assertEqual(self.node.authority.mode, 'MANUAL')
        self.assertFalse(self.node.released)
        self.assertTrue(all(m.speed_mode for m in self.device.motors.values()))
        self.assertFalse(self.motion()['estop_released'])


if __name__ == '__main__':
    unittest.main()
