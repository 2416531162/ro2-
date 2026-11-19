#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""App「驱动器参数」（2026-11-16）：闭环最大电流（CAN 0x14）、配置表（0x0E）、速度环加减速（0x0A / 0x10）。

真实的 OidDriver / OidChassis / CanLink 跑在替身 ROS 和模拟 CAN 驱动器（oid_sim）上，不连实车、不转轮子。
"""
import json
import os
import sys
import tempfile
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import test_driver_silent_timeout as fixtures
import wheeltec_driver as wd

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))

import app_link  # noqa: E402
import oid_codec as oc  # noqa: E402
import oid_driver  # noqa: E402
from oid_link import CanLink, DriverTuning, OidChassis, WheelConfig  # noqa: E402
from oid_sim import SimOidBus  # noqa: E402


def frames(device, cmd, can_id=None):
    return [data for i, data in device.sent if data[0] == cmd and (can_id is None or i == can_id)]


class Codec(unittest.TestCase):
    def test_max_current_and_config_table_frames_follow_the_manual(self):
        self.assertEqual(oc.can_max_current(100), bytes.fromhex('140064'))     # 手册 7.13：1 A
        self.assertEqual(oc.can_config_table(1), bytes.fromhex('0e01'))        # 手册 7.11：2 号配置表
        for bad in (0, -5):
            with self.assertRaises(ValueError):
                oc.can_max_current(bad)
        for bad in (-1, 4, True, 1.0):
            with self.assertRaises(ValueError):
                oc.can_config_table(bad)


class Parse(unittest.TestCase):
    def test_ranges_limit_and_types(self):
        t = DriverTuning.parse({'max_current_a': 15, 'config_table': 1, 'accel_mps2': .5, 'decel_mps2': None}, 20)
        self.assertEqual(t.as_dict(), {'max_current_a': 15.0, 'config_table': 1, 'accel_mps2': .5, 'decel_mps2': None})
        self.assertEqual(t.max_current_centiamps(), 1500)
        self.assertTrue(DriverTuning.parse({}, 20).empty)
        for bad in ({'max_current_a': 20.5}, {'max_current_a': .2}, {'config_table': 4}, {'config_table': True},
                    {'accel_mps2': 5.0}, {'decel_mps2': .5}, {'max_current_a': float('nan')}, {'accel_mps2': '1'}):
            with self.assertRaises(ValueError, msg=bad):
                DriverTuning.parse(bad, 20)
        with self.assertRaisesRegex(ValueError, '12'):
            DriverTuning.parse({'max_current_a': 15}, 12)       # oid.yaml 往下调的上限
        with self.assertRaisesRegex(ValueError, 'max_current_limit_a'):
            DriverTuning.parse({'max_current_a': 5}, 0)         # 没设上限不许改
        DriverTuning.parse({'config_table': 2, 'accel_mps2': 1.0}, 0)   # 其他几项照常能改


class Chassis(unittest.TestCase):
    def setUp(self):
        self.bus = SimOidBus(15, 74, 75)
        self.wheels = WheelConfig(pole_pairs=15, accel_mps2=1.0, decel_mps2=3.0)
        self.t = 100.0

    def run_steps(self, chassis, seconds, send=True):
        end = self.t + seconds
        while self.t < end - 1e-9:
            chassis.step(self.t, 0.0, 0.0, send=send)
            self.bus.advance(.02)
            self.t += .02

    def test_open_sends_table_then_ramps_then_current_and_uses_tuned_ramps(self):
        tuning = DriverTuning(max_current_a=15.0, config_table=2, accel_mps2=.5)
        chassis = OidChassis(CanLink(self.bus, (74, 75)), self.wheels, feedback_timeout_s=2.0, tuning=tuning)
        chassis.open(self.t)
        per = self.wheels.erpm_per_mps()
        for can_id, motor in ((74, 1), (75, 2)):
            order = [d[0] for i, d in self.bus.sent if i == can_id and d[0] in (0x0E, 0x0A, 0x10, 0x14)]
            self.assertEqual(order, [0x0E, 0x0A, 0x10, 0x14])
            m = self.bus.motors[motor]
            self.assertEqual((m.max_current_ca, m.config_table), (1500, 2))
            self.assertEqual(m.accel, round(.5 * per))
            self.assertEqual(m.decel, round(3.0 * per))                 # 没改的用 oid.yaml
        self.assertEqual(chassis.wheels.accel_mps2, .5)
        self.assertIs(chassis.wheel_aligner.wheels, chassis.wheels)

    def test_empty_tuning_sends_only_yaml_ramps_like_before(self):
        chassis = OidChassis(CanLink(self.bus, (74, 75)), self.wheels, feedback_timeout_s=2.0)
        chassis.open(self.t)
        self.assertFalse(frames(self.bus, 0x14) or frames(self.bus, 0x0E))
        self.assertEqual(len(frames(self.bus, 0x0A)), 2)

    def test_driver_power_cycle_gets_parameters_again_only_when_sending(self):
        chassis = OidChassis(CanLink(self.bus, (74, 75)), self.wheels, feedback_timeout_s=2.0,
                             tuning=DriverTuning(max_current_a=12.5, config_table=1))
        chassis.open(self.t)
        self.run_steps(chassis, 1.0)
        right = self.bus.motors[2]
        right.powered = False
        self.run_steps(chassis, 3.0)
        right.powered, right.max_current_ca, right.config_table = True, None, 0    # 断电重启：回到上位机保存的
        left_before = len(frames(self.bus, 0x14, 74))
        self.run_steps(chassis, .5, send=False)                                     # 停稳放空 / 只收不发：不发
        self.assertIsNone(right.max_current_ca)
        self.run_steps(chassis, .1)
        self.assertEqual((right.max_current_ca, right.config_table), (1250, 1))
        self.assertEqual(len(frames(self.bus, 0x14, 74)), left_before)              # 左轮没断过，不重发
        self.assertEqual(chassis.reapplied, 1)
        sent = len(frames(self.bus, 0x14))
        self.run_steps(chassis, 1.0)
        self.assertEqual(len(frames(self.bus, 0x14)), sent)                          # 只发一次

    def test_tuning_needs_can_link(self):
        link = NS(kind='rs485+can', state={1: None, 2: None})
        with self.assertRaisesRegex(ValueError, 'CAN'):
            OidChassis(link, self.wheels, tuning=DriverTuning(max_current_a=5.0))
        OidChassis(link, self.wheels, tuning=DriverTuning())


class Node(unittest.TestCase):
    """驱动节点：存盘、停稳检查、IO 线程下发、结果、重启后读回。"""
    preset = None

    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.path = os.path.join(tmp.name, 'rtk', 'oid_tuning.json')
        if self.preset is not None:
            os.makedirs(os.path.dirname(self.path))
            with open(self.path, 'w', encoding='utf-8') as f:
                f.write(self.preset)
        patch = mock.patch.dict(oid_driver.PARAMS, tuning_file=self.path, max_current_limit_a=20.0)
        patch.start()
        self.addCleanup(patch.stop)
        fixtures.TestGen5DriverSilence.setUp(self)

    step = fixtures.TestGen5DriverSilence.step

    def tune(self, ident='t1', **values):
        cmd = dict(dict.fromkeys(app_link.DRIVER_TUNE_FIELDS), id=ident, **values)
        self.node.on_tune(wd.String(data=json.dumps(cmd)))
        return self.node.tuning_status()


class NodeTune(Node):
    def test_stopped_change_is_saved_then_sent_and_reported(self):
        self.step(4.0)
        self.assertTrue(self.node.tuning_status()['supported'])
        self.device.sent.clear()
        status = self.tune(max_current_a=16.5, config_table=1)
        self.assertEqual(status['result'], {'id': 't1', 'ok': None, 'message': '已保存，正在发给驱动器'})
        self.assertFalse(status['sent'])
        with open(self.path, encoding='utf-8') as f:
            self.assertEqual(json.load(f)['max_current_a'], 16.5)
        self.step(.04)
        for m in self.device.motors.values():
            self.assertEqual((m.max_current_ca, m.config_table), (1650, 1))
        status = self.node.tuning_status()
        self.assertTrue(status['result']['ok'])
        self.assertTrue(status['sent'])
        self.assertEqual(status['values']['max_current_a'], 16.5)
        self.assertEqual(status['limit_a'], 20.0)

    def test_moving_or_out_of_range_is_rejected_and_nothing_changes(self):
        self.step(4.0)
        self.step(1.0, command=(.3, 0.0))
        status = self.tune(max_current_a=10.0)
        self.assertFalse(status['result']['ok'])
        self.assertIn('停车', status['result']['message'])
        self.assertFalse(os.path.exists(self.path))
        self.step(3.0, command=(0.0, 0.0))
        status = self.tune('t2', max_current_a=25.0)
        self.assertFalse(status['result']['ok'])
        self.assertIn('0.5–20', status['result']['message'])
        self.step(.1)
        self.assertFalse(frames(self.device, 0x14))
        self.assertFalse(os.path.exists(self.path))

    def test_unsupported_when_one_wheel_is_on_485(self):
        self.node.tuning_supported = False
        self.step(4.0)
        status = self.tune(max_current_a=10.0)
        self.assertFalse(status['result']['ok'])
        self.assertIn('CAN', status['result']['message'])

    def test_clearing_stops_sending_and_restores_yaml_ramps(self):
        self.step(4.0)
        self.tune(max_current_a=10.0, accel_mps2=.4)
        self.step(.04)
        per = self.node.wheels.erpm_per_mps()
        self.assertEqual(self.device.motors[1].accel, round(.4 * per))
        self.device.sent.clear()
        self.tune('t2')
        self.step(.04)
        self.assertEqual(self.device.motors[1].accel, round(self.node.wheels.accel_mps2 * per))
        self.assertFalse(frames(self.device, 0x14))
        self.assertTrue(self.node.tuning_status()['sent'])


class NodeRestart(Node):
    preset = json.dumps({'max_current_a': 14.0, 'config_table': 3, 'accel_mps2': None, 'decel_mps2': 2.5})

    def test_saved_parameters_are_sent_when_the_driver_connects(self):
        self.step(.1)
        for m in self.device.motors.values():
            self.assertEqual((m.max_current_ca, m.config_table), (1400, 3))
        status = self.node.tuning_status()
        self.assertTrue(status['sent'])
        self.assertEqual(status['values']['decel_mps2'], 2.5)
        self.assertIsNone(status['error'])


class NodeCorruptFile(Node):
    preset = '{"max_current_a": 99'

    def test_unreadable_file_is_not_sent_and_is_reported(self):
        self.step(.1)
        self.assertFalse(frames(self.device, 0x14))
        self.assertIn('读不出来', self.node.tuning_status()['error'])


class Protocol(unittest.TestCase):
    def test_command_needs_id_and_all_fields(self):
        cmd = app_link.parse_command(json.dumps({'type': 'driver_tune', 'id': 'a', 'max_current_a': 15,
                                                 'config_table': 1, 'accel_mps2': None, 'decel_mps2': 3}))
        self.assertEqual(cmd, {'type': 'driver_tune', 'id': 'a', 'max_current_a': 15.0, 'config_table': 1,
                               'accel_mps2': None, 'decel_mps2': 3.0})
        full = dict(dict.fromkeys(app_link.DRIVER_TUNE_FIELDS), type='driver_tune', id='a')
        for bad in (dict(full, id=None), {k: v for k, v in full.items() if k != 'decel_mps2'},
                    dict(full, config_table=1.0), dict(full, max_current_a=True), dict(full, accel_mps2='1')):
            bad = {k: v for k, v in bad.items() if v is not None or k in app_link.DRIVER_TUNE_FIELDS}
            with self.assertRaises(app_link.CommandError, msg=bad):
                app_link.parse_command(json.dumps(bad))

    def test_telemetry_only_while_fresh(self):
        tuning = {'supported': True, 'limit_a': 20.0, 'values': {'max_current_a': 15.0, 'config_table': 1},
                  'defaults': {'accel_mps2': 1.0, 'decel_mps2': 3.0}, 'sent': True, 'reapplied': 2, 'error': None,
                  'result': {'id': 'a', 'ok': True, 'message': '已发给两台驱动器'}, '_rx_monotonic': 10.0}
        out = app_link.build_telemetry({'oid_tuning': tuning}, now=11.0, seq=1)['drivers']
        self.assertEqual(out['values'], {'max_current_a': 15.0, 'config_table': 1, 'accel_mps2': None, 'decel_mps2': None})
        self.assertEqual(out['result'], {'id': 'a', 'ok': True, 'message': '已发给两台驱动器'})
        self.assertTrue(out['sent'])
        self.assertNotIn('drivers', app_link.build_telemetry({'oid_tuning': tuning}, now=14.0, seq=1))
        self.assertNotIn('drivers', app_link.build_telemetry({}, now=11.0, seq=1))


if __name__ == '__main__':
    unittest.main()
