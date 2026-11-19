#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""停车放空（2026-11-19 地面事故：驱动器零速闭环自己约 1 Hz 振荡，急停只发零速压不住；零速参数不改回去，方案一）。

- 开机直接放空，不先发零速；停车刹到轮速很小马上放空（不再等停稳满 1 s）；
- 任何状态发零速时轮子一直在动就放空（零速压不住）；作业中再转故障，作业中断；急停时驱动器不回话也放空。

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
from robot_core import motion_text  # noqa: E402


def hunt(motor, amplitude_erpm, hz=1.0, until=None, offset_erpm=0.0):
    """模拟驱动器零速保持环压不住：转速模式下目标为 0 时自己转，offset + 正弦（offset 0 = 来回摇；offset 比振幅大
    = 一直往一个方向蹭、刹不到零）；模拟时间到 until 为止（None 一直这样）。放空、心跳超时或有非零目标时按原来的速度环。"""
    normal = motor.step

    def step(t, dt):
        if (motor.alive(t) and motor.speed_mode and not motor.fault and motor.target == 0
                and (until is None or t < until)):
            motor.speed = offset_erpm + amplitude_erpm * math.sin(2 * math.pi * hz * t)
            motor.position += motor.speed / motor.pole_pairs * 360.0 / 60.0 * 100.0 * dt
        else:
            normal(t, dt)
    motor.step = step


def overshoot(motor, back_erpm, hz=1.2):
    """模拟 11-19 地上手动停车：目标 0 时先按减速度刹到零，过零后零速闭环往回打（反向起摆、一直来回摇）。"""
    normal = motor.step
    st = dict(sign=None, cross=None)

    def step(t, dt):
        if not (motor.alive(t) and motor.speed_mode and not motor.fault and motor.target == 0):
            st.update(sign=None, cross=None)
            normal(t, dt)
            return
        if st['sign'] is None:
            st['sign'] = (motor.speed > 0) - (motor.speed < 0)
        if st['cross'] is None:
            normal(t, dt)
            if motor.speed * st['sign'] <= 0:
                st['cross'] = t
            return
        motor.speed = -st['sign'] * back_erpm * math.sin(2 * math.pi * hz * (t - st['cross']))
        motor.position += motor.speed / motor.pole_pairs * 360.0 / 60.0 * 100.0 * dt
    motor.step = step


class TestZeroHoldRelease(unittest.TestCase):
    setUp = fixtures.TestGen5DriverSilence.setUp

    def step(self, seconds, command=None, until=None, source='manual'):
        """推进 seconds 秒；until(node) 为真时提前停，返回停下时距开始的秒数（没停返回 None）。"""
        dt = 1.0 / self.node.config.tx_hz
        start, end = self.clock[0], self.clock[0] + seconds
        while self.clock[0] < end - 1e-9:
            if command is not None:
                data = dict(vx=command[0], wz=command[1], epoch=self.node.authority.epoch,
                            profile_hash=wd.profile_hash(wd.PROFILE), stamp=self.clock[0] - .01)
                self.node.on_motion(source, wd.String(data=json.dumps(data)))
            self.node._io_step(self.clock[0])
            self.device.advance(dt)
            self.clock[0] += dt
            if until is not None and until(self.node):
                return self.clock[0] - start
        return None

    def drive(self, speed, source='manual'):
        self.step(4.0)                              # 开机：直接放空、等反馈
        if source == 'navigation':
            self.assertTrue(self.node.select_motion('navigation', NS(data=True), NS()).success)
        self.step(1.5, (speed, 0.0), source=source)
        self.assertEqual(self.node.authority.mode, source.upper())
        self.assertTrue(all(m.speed_mode for m in self.device.motors.values()))

    def hunt_all(self, speed_mps, seconds=None, creep_mps=0.0):
        """creep_mps：零速时顺着现在的转向一直蹭着走（刹不到零），用来测兜底。"""
        until = None if seconds is None else self.device.t + seconds
        for m in self.device.motors.values():
            sign = (m.speed > 0) - (m.speed < 0)
            hunt(m, self.node.wheels.to_erpm(speed_mps), until=until,
                 offset_erpm=sign * self.node.wheels.to_erpm(creep_mps))

    def stop_and_watch(self, seconds, command=None):
        """停车：一拍一拍走到放空为止，返回 (用了多久, 过零后最大反冲 m/s)。"""
        signs = {k: (m.speed > 0) - (m.speed < 0) for k, m in self.device.motors.items()}
        dt = 1.0 / self.node.config.tx_hz
        back, took = 0.0, None
        for i in range(int(seconds / dt)):
            self.step(dt, command)
            back = max([back] + [-signs[k] * self.node.wheels.to_mps(m.speed) for k, m in self.device.motors.items()])
            if self.node.released:
                took = (i + 1) * dt
                break
        return took, back

    def frames(self, cmd):
        return [can_id for can_id, data in self.device.sent if data[0] == cmd]

    def speed_mode(self):
        return [m.speed_mode for m in self.device.motors.values()]

    def max_speed_mps(self):
        return max(abs(self.node.wheels.to_mps(m.speed)) for m in self.device.motors.values())

    def motion(self):
        return self.node.motion_extensions(self.clock[0])

    # ---- 开机、停车 ----

    def test_startup_releases_without_sending_zero_speed(self):
        # 11-19 事故：开机后先发零速，零速闭环振荡、车摇了 3 分钟。现在开机第一拍就放空，从不进零速闭环
        self.hunt_all(.5)
        self.step(.05)
        self.assertTrue(self.node.released)
        self.assertIn(74, self.frames(oc.CAN_CURRENT))
        self.assertIn(75, self.frames(oc.CAN_CURRENT))
        speed_frames = list(self.frames(oc.CAN_SPEED))
        self.step(5.0)
        self.assertEqual(self.node.authority.mode, 'IDLE')
        self.assertTrue(self.node.released)
        self.assertFalse(speed_frames + self.frames(oc.CAN_SPEED))
        self.assertFalse(any(self.speed_mode()))
        self.assertLess(self.max_speed_mps(), .01)
        self.node.on_stop(None, NS())                                         # 停着按急停：下一拍就放空，不进零速闭环
        self.step(.5)
        self.assertTrue(self.node.released)
        self.assertFalse(self.frames(oc.CAN_SPEED))
        self.assertFalse(self.motion()['estop_released'])

    def test_estop_brakes_then_releases_once_wheels_are_slow(self):
        self.drive(.6)
        self.node.on_stop(None, NS())
        dt = 1.0 / self.node.config.tx_hz
        released_at = None
        for i in range(int(3.0 / dt)):
            self.step(dt)
            if self.node.released:
                released_at = (i + 1) * dt
                break
            self.assertTrue(all(self.speed_mode()))                          # 先零速刹车
        self.assertIsNotNone(released_at)
        self.assertGreaterEqual(released_at, .6 / 2.0)                        # 刹车（减速度 2 m/s²）完成以后
        self.assertLess(released_at, 1.0)                                     # 不再等停稳满 1 s
        self.assertLessEqual(self.max_speed_mps(), oid_driver.HALL_STILL_MAX_MPS)
        self.assertFalse(any(self.speed_mode()))
        self.assertFalse(self.motion()['estop_released'])                   # 走的是正常停车放空，不是兜底
        self.assertIsNone(self.motion()['zero_hold_release_reason'])

    def test_manual_stop_releases_once_wheels_are_slow(self):
        self.drive(.6)
        took = self.step(3.0, (0.0, 0.0), until=lambda n: n.released)
        self.assertIsNotNone(took)
        self.assertLess(took, 1.2)                                            # 松手渐停 + 刹到轮速很小
        self.assertEqual(self.node.authority.mode, 'MANUAL')
        self.assertLessEqual(self.max_speed_mps(), .06)
        self.assertIsNone(self.motion()['zero_hold_release_reason'])

    def test_manual_stop_releases_at_zero_before_driver_swings_back(self):
        # 11-19 地上手动停车：驱动器刹到零后零速闭环往回打 0.2–0.4 m/s，车往后蹿；刹到零就放空，不留反冲的时间
        self.drive(.5)
        for m in self.device.motors.values():
            overshoot(m, self.node.wheels.to_erpm(.3))
        took, back = self.stop_and_watch(3.0, (0.0, 0.0))
        self.assertIsNotNone(took)
        self.assertLess(back, .08, '放空前往回打了 %.2f m/s' % back)
        self.assertIsNone(self.motion()['zero_hold_release_reason'])          # 正常停车放空，不是兜底
        self.assertFalse(any(self.speed_mode()))
        self.step(1.0, (0.0, 0.0))
        self.assertTrue(self.node.released)
        self.assertLess(self.max_speed_mps(), .01)

    def test_estop_releases_at_zero_before_driver_swings_back(self):
        self.drive(.5)
        for m in self.device.motors.values():
            overshoot(m, self.node.wheels.to_erpm(.3))
        self.node.on_stop(None, NS())
        took, back = self.stop_and_watch(3.0)
        self.assertIsNotNone(took)
        self.assertGreaterEqual(took, .5 / 2.0 - .05)                         # 先零速刹车到零
        self.assertLess(back, .08, '放空前往回打了 %.2f m/s' % back)
        self.assertFalse(self.motion()['estop_released'])

    def test_hard_stop_from_high_speed_is_normal_braking_not_hunting(self):
        # 1.2 m/s、减速度 2 m/s² 要刹 0.6 s：比 0.5 s 长，也不能当成零速压不住提前放空
        self.drive(1.2)
        self.assertGreater(min(abs(m.speed) for m in self.device.motors.values()), self.node.wheels.to_erpm(1.0))
        self.node.on_stop(None, NS())
        took = self.step(2.0, until=lambda n: n.released)
        self.assertIsNotNone(took)
        self.assertGreaterEqual(took, 1.2 / 2.0)
        self.assertLessEqual(self.max_speed_mps(), .06)
        self.assertFalse(self.motion()['estop_released'])

    # ---- 兜底：零速压不住 ----

    def test_hunting_driver_is_released_about_half_a_second_after_estop(self):
        # 零速压不住、刹不到零（一直顺着蹭）：过了刹车时间还在动约 0.5 s 就放空
        self.drive(.3)
        self.hunt_all(.1, creep_mps=.3)
        self.node.on_stop(None, NS())
        self.assertEqual(self.node.authority.mode, 'ESTOP')
        self.assertIsNone(self.step(.4, until=lambda n: n.released))     # 先按零速刹车
        self.assertTrue(all(self.speed_mode()))
        took = self.step(.5, until=lambda n: n.released)
        self.assertIsNotNone(took, '轮子一直在动，约 0.5 s 应放空')
        self.assertLessEqual(.4 + took, .65)
        self.assertFalse(any(self.speed_mode()))                             # 驱动器收到电流 0，不再出力
        self.assertEqual(self.motion()['estop_release_reason'], 'wheels_moving')
        self.assertEqual(self.motion()['zero_hold_release_reason'], 'wheels_moving')
        self.assertTrue(self.motion()['estop_released'] and self.motion()['wheels_released'])
        self.step(2.0)
        self.assertTrue(self.node.released)                                 # 放空后不再恢复零速闭环
        self.assertFalse(any(self.speed_mode()))
        self.assertEqual(self.node.authority.mode, 'ESTOP')
        self.assertLess(max(abs(m.speed) for m in self.device.motors.values()), 1.0)

    def test_stop_without_reaching_zero_is_released_by_guard(self):
        # 零速压不住、轮子刹不到零（一直顺着蹭）：原来停稳满 1 s 才放空，永远等不到；兜底约 0.5 s 放空。
        # App 断线不再发指令：指令超时（Gen5 1.5 s）后回空闲、直接发零速（没有松手渐停），轮子没机会先刹到零
        self.drive(.3)
        self.hunt_all(.1, creep_mps=.3)
        took = self.step(4.0, until=lambda n: n.released)
        self.assertIsNotNone(took)
        decel = self.node.chassis.wheels.decel_mps2
        self.assertLess(took, self.node.authority.timeout_s + .4 / decel + .3 + .5 + .1)   # 指令超时 + 正常刹车时间 + 0.5 s
        self.assertEqual(self.node.authority.mode, 'IDLE')
        self.assertEqual(self.motion()['zero_hold_release_reason'], 'wheels_moving')
        self.assertFalse(self.motion()['estop_released'])                   # 不是急停
        self.assertFalse(any(self.speed_mode()))
        self.step(1.0)
        self.assertTrue(self.node.released)
        self.step(1.0, (.3, 0.0))                                            # 再推摇杆：恢复速度闭环
        self.assertFalse(self.node.released)
        self.assertTrue(all(self.speed_mode()))
        self.assertGreater(min(abs(m.speed) for m in self.device.motors.values()), 50)

    def test_manual_takeover_from_job_with_hunting_driver_can_drive(self):
        # 作业中手动接管要先等车停稳；零速闭环振荡时原来永远停不稳、接管后开不动
        self.drive(.3, 'navigation')
        self.hunt_all(.4)
        self.device.sent.clear()
        took = self.step(5.0, (.3, 0.0), until=lambda n: n.authority.mode == 'MANUAL' and not n.released
                         and n._cmd[0] > 0.0 and all(self.speed_mode()))
        self.assertIsNotNone(took, '手动接管后应能开起来')
        self.assertIn(74, self.frames(oc.CAN_CURRENT))                      # 等停稳时零速压不住，中间放空过
        self.step(1.0, (.3, 0.0))
        self.assertGreater(min(abs(m.speed) for m in self.device.motors.values()), 50)

    def test_job_zero_hold_hunting_faults_and_releases(self):
        self.drive(.3, 'navigation')
        self.hunt_all(.4)
        took = self.step(4.0, (0.0, 0.0), until=lambda n: n.authority.mode != 'NAVIGATION', source='navigation')
        self.assertIsNotNone(took, '作业中零速压不住应转故障')
        self.assertGreaterEqual(took, oid_driver.ZERO_HOLD_JOB_FAULT_S)    # 作业中判得更严
        self.assertLess(took, oid_driver.ZERO_HOLD_JOB_FAULT_S + .6)
        self.assertEqual(self.node.authority.mode, 'FAULT')
        self.assertEqual(self.node.authority.reason, 'zero_hold_oscillation')
        self.assertTrue(self.node.released)                                 # 同一拍放空
        self.assertFalse(any(self.speed_mode()))
        self.assertEqual(self.motion()['zero_hold_release_reason'], 'wheels_moving')
        self.assertEqual(motion_text.status_text('FAULT', 'zero_hold_oscillation'),
                         '故障锁存：停车锁轮时轮子一直来回动，已松开驱动器、中断作业')
        self.step(2.0)
        for m in self.device.motors.values():                                # 驱动器处理好了：零速不再振荡
            del m.step
        result = self.node.reset_motion(None, NS())
        self.assertTrue(result.success, result.message)
        self.assertEqual(self.node.authority.mode, 'IDLE')

    def test_job_short_rebound_after_stop_keeps_zero_hold(self):
        # 作业中停车后零速闭环回拉一下（约 1 s 就停住）：照旧锁零速，不放空、不停作业
        self.drive(.3, 'navigation')
        self.hunt_all(.4, seconds=1.0)
        self.step(4.0, (0.0, 0.0), source='navigation')
        self.assertEqual(self.node.authority.mode, 'NAVIGATION')
        self.assertFalse(self.node.released)
        self.assertTrue(all(self.speed_mode()))
        self.assertIsNone(self.motion()['zero_hold_release_reason'])

    # ---- 急停：驱动器不回话、复位 ----

    def test_silent_drivers_get_release_commands(self):
        self.drive(.3)
        self.node.on_stop(None, NS())
        self.device.motors[2].powered = False                               # 急停同时右轮驱动器不回话
        self.device.sent.clear()
        self.step(self.node.config.feedback_timeout_s - .2)
        self.assertNotIn(75, self.frames(oc.CAN_CURRENT))                   # 右轮读不到停没停，不到反馈超时照旧零速刹车
        self.step(.5)
        self.assertEqual(self.motion()['estop_release_reason'], 'driver_silent')
        self.assertTrue(self.node.released)
        self.assertIn(74, self.frames(oc.CAN_CURRENT))
        self.assertIn(75, self.frames(oc.CAN_CURRENT))
        sent = len(self.frames(oc.CAN_CURRENT))
        self.step(1.0)                                                       # 还不回话：隔 0.5 s 重发
        self.assertGreaterEqual(len(self.frames(oc.CAN_CURRENT)) - sent, 2 * 2)
        heartbeats = [d for _, d in self.device.sent[-20:] if d[0] in (oc.CAN_HEARTBEAT, oc.CAN_SPEED)]
        self.assertFalse(heartbeats)                                          # 放空后不再发心跳和转速

    def test_reset_after_estop_release_restores_drive(self):
        self.drive(.3)
        self.hunt_all(.1, creep_mps=.3)
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
        self.assertTrue(all(self.speed_mode()))
        self.assertGreater(min(abs(m.speed) for m in self.device.motors.values()), 50)

    def test_reset_is_refused_in_chinese_while_wheels_still_move(self):
        self.drive(.3)
        self.node.on_stop(None, NS())
        result = self.node.reset_motion(None, NS())                          # 车还在滑：复位照常检查
        self.assertFalse(result.success)
        self.assertEqual(result.message, '复位没成功：请等传感器和底盘正常、车停稳后再复位')


if __name__ == '__main__':
    unittest.main()
