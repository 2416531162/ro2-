"""真实 OID 驱动节点与 CAN 模拟驱动器；不连接或转动实车轮子。"""
import json
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import test_driver_silent_timeout as fixtures
import wheeltec_driver as wd


class TestAutomaticWheelRelease(unittest.TestCase):
    setUp = fixtures.TestGen5DriverSilence.setUp

    def step(self, seconds, command=None, source='manual'):
        dt = 1.0 / self.node.config.tx_hz
        end = self.clock[0] + seconds
        while self.clock[0] < end - 1e-9:
            if command is not None:
                data = dict(vx=command[0], wz=command[1], epoch=self.node.authority.epoch,
                            profile_hash=wd.profile_hash(wd.PROFILE), stamp=self.clock[0] - .01)
                self.node.on_motion(source, wd.String(data=json.dumps(data)))
            self.node._io_step(self.clock[0])
            self.device.advance(dt)
            self.clock[0] += dt

    def test_idle_hand_pushing_stays_released_without_a_push_command(self):
        self.step(4.0)
        self.assertTrue(self.node.released)
        self.device.sent.clear()
        for _ in range(30):
            for motor in self.device.motors.values():
                motor.speed = 300.0
            self.step(.02)
            self.assertTrue(self.node.released)
            self.assertFalse(any(m.speed_mode for m in self.device.motors.values()))
        self.assertFalse([data for _, data in self.device.sent if data[0] in (0x00, 0x02)])

    def test_fresh_manual_command_restores_drive_and_stop_releases_again(self):
        self.step(4.0)
        self.step(1.0, (.3, 0.0))
        self.assertFalse(self.node.released)
        self.assertTrue(all(m.speed_mode for m in self.device.motors.values()))
        self.assertGreater(min(abs(m.speed) for m in self.device.motors.values()), 50)
        self.step(.02, (0.0, 0.0))
        self.assertFalse(self.node.released)  # 先按原停车斜坡制动。
        self.step(2.0)
        self.assertTrue(self.node.released)

    def test_held_zero_manual_command_releases_without_waiting_for_command_timeout(self):
        self.step(4.0)
        self.step(1.0, (.3, 0.0))
        self.step(2.0, (0.0, 0.0))
        self.assertEqual(self.node.authority.mode, 'MANUAL')
        self.assertTrue(self.node.released)
        self.assertFalse(any(m.speed_mode for m in self.device.motors.values()))
        self.step(1.0, (0.0, .2))  # 新的原地转向也是明确驾驶指令。
        self.assertFalse(self.node.released)
        self.assertNotEqual(self.node._cmd[1], 0.0)

    def test_automatic_job_keeps_control_at_zero_and_releases_after_pause_or_completion(self):
        self.step(4.0)
        self.assertTrue(self.node.select_motion('navigation', NS(data=True), NS()).success)
        self.step(2.0, (.3, 0.0), 'navigation')
        self.step(2.0, (0.0, 0.0), 'navigation')
        self.assertEqual(self.node.authority.mode, 'NAVIGATION')
        self.assertFalse(self.node.released)
        self.assertTrue(all(m.speed_mode for m in self.device.motors.values()))
        self.assertTrue(self.node.select_motion('navigation', NS(data=False), NS()).success)
        self.step(.04)
        self.assertTrue(self.node.released)
        self.assertEqual(self.node.authority.mode, 'IDLE')

    def test_emergency_brakes_a_moving_vehicle_before_releasing(self):
        self.step(4.0)
        self.step(1.0, (.6, 0.0))
        self.node.on_stop(None, NS())
        self.step(.1)
        self.assertEqual(self.node.authority.mode, 'ESTOP')
        self.assertFalse(self.node.released)
        self.assertEqual(self.node._cmd, (0.0, 0.0))
        self.step(2.0)
        self.assertTrue(self.node.released)

    def test_same_epoch_manual_command_cancels_a_stale_release_decision(self):
        self.step(4.0)
        self.step(.5, (.3, 0.0))
        self.step(2.0, (0.0, 0.0))
        self.assertTrue(self.node.released)
        decide = self.node.automatic_release_allowed
        calls = 0
        def racing(now, v, w):
            nonlocal calls
            calls += 1
            if calls == 2:
                epoch = self.node.authority.epoch
                data = dict(vx=.3, wz=0.0, epoch=epoch, stamp=now-.01)
                self.assertTrue(self.node.authority.submit('manual', data, now, now))
                self.assertEqual(epoch, self.node.authority.epoch)
            return decide(now, v, w)
        with mock.patch.object(self.node, 'automatic_release_allowed', racing):
            self.node._io_step(self.clock[0])
        self.assertFalse(self.node.released)
        self.assertTrue(all(m.speed_mode for m in self.device.motors.values()))

    def test_release_status_reports_actual_motor_state_and_preserves_legacy_fields(self):
        self.step(4.0)
        status = self.node.motion_extensions(self.clock[0])
        self.assertTrue(status['auto_release'] and status['wheels_released'])
        self.assertFalse(status['push_mode'])
        self.step(1.0, (.3, 0.0))
        self.assertFalse(self.node.motion_extensions(self.clock[0])['wheels_released'])
