#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""欧艾迪驱动器 RS485 版（Modbus RTU）：编解码与厂家《RS485 系列编程手册 V1.3》的示例指令逐字节一致；按手册寄存器行为
写的模拟驱动器上跑 Rs485Link；本车实际接法（2026-09-29：左轮 485、右轮 CAN）用 SplitLink 拼起来跑直线、原地转、
一台掉线；按参数建链路；串口一问一答的收帧（杂字节、分段到达、回显、超时）。"""
import math
import os
import sys
import types
import unittest
from unittest import mock

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'oid_protocol'))
sys.path.insert(0, os.path.join(ROOT, 'wheeltec_protocol'))
sys.path.insert(0, ROOT)

import oid_link  # noqa: E402
import oid_modbus as om  # noqa: E402
from oid_driver import DiffConfig, bus_kind, bus_label, build_link  # noqa: E402
from oid_link import CanLink, OidChassis, Rs485Link, SplitLink, WheelConfig  # noqa: E402
from oid_sim import SimModbusPort, SimOidBus  # noqa: E402

H = bytes.fromhex
POLES = 15
WHEELS = WheelConfig(pole_pairs=POLES)


def run(chassis, devices, v, w, seconds, t0=0.0, dt=0.04, send=True):
    t = t0
    while t < t0 + seconds - 1e-9:
        chassis.step(t, v, w, send=send)
        for d in devices:
            d.advance(dt)
        t += dt
    return t


class TestManualFrames(unittest.TestCase):
    """手册第 2.6.3、7、8 节的示例指令（手册里的数字都是十六进制）。"""

    def test_requests(self):
        cases = [
            (om.read_holding(1, 6000, 2), '01 03 17 70 00 02 C0 64'),
            (om.read_input(1, 5000, 2), '01 04 13 88 00 02 F5 65'),
            (om.write_single(1, 6000, 2), '01 06 17 70 00 02 0C 64'),
            (om.write_multiple(1, 6000, [0x64, 0x1234]), '01 10 17 70 00 02 04 00 64 12 34 53 D3'),
            (om.heartbeat(1, 1), '01 06 17 70 00 01 4C 65'), (om.heartbeat(1, 2), '01 06 17 70 00 02 0C 64'),
            (om.heartbeat(1, 3), '01 06 17 70 00 03 CD A4'), (om.heartbeat(1, 4), '01 06 17 70 00 04 8C 66'),
            (om.set_current(1, 100), '01 06 17 72 00 64 2D 8E'), (om.set_current(1, -100), '01 06 17 72 FF 9C 6D FC'),
            (om.set_mode(1, om.MODE_CURRENT), '01 06 17 71 00 00 DC 65'), (om.set_mode(1, om.MODE_SPEED), '01 06 17 71 00 01 1D A5'),
            (om.set_mode(1, om.MODE_IDLE), '01 06 17 71 FF FF DD D5'),
            (om.set_speed(1, 4000), '01 10 17 73 00 02 04 00 00 0F A0 5A E6'),
            (om.set_speed(1, -4000), '01 10 17 73 00 02 04 FF FF F0 60 1B 62'),
            (om.set_speed_accel(1, 2000), '01 10 17 80 00 02 04 00 00 07 D0 13 93'),
            (om.set_speed_decel(1, 800), '01 10 17 89 00 02 04 00 00 03 20 D1 7D'),
            (om.read_input(1, 5000, 1), '01 04 13 88 00 01 B5 64'), (om.read_input(1, 5001, 2), '01 04 13 89 00 02 A4 A5'),
            (om.read_input(1, 5005, 1), '01 04 13 8D 00 01 A5 65'), (om.read_input(1, 5010, 2), '01 04 13 92 00 02 D4 A2'),
        ]
        for got, want in cases:
            self.assertEqual(got, H(want), want)

    def test_replies(self):
        self.assertEqual(om.parse_reply(om.read_holding(1, 6000, 2), H('01 03 04 00 64 13 88 B6 BA')), [100, 5000])
        self.assertEqual(om.parse_reply(om.read_input(1, 5000, 2), H('01 04 04 00 64 13 88 B7 0D')), [100, 5000])
        speed = om.parse_reply(om.read_input(1, 5001, 2), H('01 04 04 00 00 05 41 38 E4'))
        self.assertEqual(om.words_i32(speed), 1345)                          # 4 对极时 336.25 rpm
        position = om.parse_reply(om.read_input(1, 5010, 2), H('01 04 04 00 00 70 DA 5F DF'))
        self.assertEqual(om.words_i32(position), 28890)                      # 288.9°
        self.assertEqual(om.parse_reply(om.read_input(1, 5005, 1), H('01 04 02 00 30 B9 24')), [48])
        self.assertEqual(om.parse_reply(om.read_input(1, 5000, 1), H('01 04 02 00 00 B9 30')), [0])
        self.assertTrue(om.parse_reply(om.write_multiple(1, 6000, [0x64, 0x1234]), H('01 10 17 70 00 02 45 A7')))
        self.assertTrue(om.parse_reply(om.set_mode(1, 1), H('01 06 17 71 00 01 1D A5')))

    def test_errors_and_word_order(self):
        with self.assertRaisesRegex(om.ModbusError, 'CRC'):
            om.parse_reply(om.read_input(1, 5005, 1), H('01 04 02 00 30 B9 25'))
        with self.assertRaisesRegex(om.ModbusError, '非法数据地址'):
            om.parse_reply(om.read_input(1, 5000, 1), om.frame(1, bytes([0x84, 0x02])))
        with self.assertRaisesRegex(om.ModbusError, '站号'):
            om.parse_reply(om.read_input(2, 5005, 1), H('01 04 02 00 30 B9 24'))
        # 手册第 4 节：0x01020304 小端字序发 03 04 01 02
        self.assertEqual(om.i32_words(0x01020304, 'big'), [0x0102, 0x0304])
        self.assertEqual(om.i32_words(0x01020304, 'little'), [0x0304, 0x0102])
        self.assertEqual(om.words_i32(om.i32_words(-4000, 'little'), 'little'), -4000)

    def test_find_reply_in_noise(self):
        req = om.read_input(1, 5005, 1)
        reply = H('01 04 02 00 30 B9 24')
        self.assertEqual(om.find_reply(b'\x00\xff' + reply, req), reply)     # 前面有杂字节
        self.assertIsNone(om.find_reply(reply[:4], req))                     # 还没收全
        self.assertIsNone(om.find_reply(H('01 04 02 00 30 B9 25'), req))     # CRC 不对
        self.assertEqual(om.find_reply(om.frame(1, bytes([0x84, 0x02])), req)[1], 0x84)


class Rs485Case(unittest.TestCase):
    def make(self, word_order='big'):
        port = SimModbusPort(POLES, (1, 2), voltage=24, word_order=word_order)
        return port, OidChassis(Rs485Link(port, (1, 2), word_order=word_order), WHEELS)


class TestRs485Link(Rs485Case):
    """两台都在 485 上（一条 485 两个站号）。"""

    def test_straight_and_pivot(self):
        port, chassis = self.make()
        chassis.open(0.0)
        t = run(chassis, [port], 0.5, 0.0, 4.0)
        o = chassis.odom
        self.assertAlmostEqual(o.v, 0.5, delta=0.01)
        self.assertAlmostEqual(o.y, 0.0, delta=1e-3)
        self.assertGreater(o.x, 1.5)
        self.assertAlmostEqual(chassis.telemetry(t)['voltage'], 24.0)
        self.assertLess(port.motors[1].speed * port.motors[2].speed, 0)    # 镜像安装
        run(chassis, [port], 0.0, 0.0, 1.5, t)
        run(chassis, [port], 0.0, 0.8, 3.0, t + 1.5)
        self.assertAlmostEqual(chassis.odom.w, 0.8, delta=0.02)

    def test_little_endian_words(self):
        port, chassis = self.make('little')
        chassis.open(0.0)
        run(chassis, [port], 0.5, 0.0, 3.0)
        self.assertAlmostEqual(chassis.odom.v, 0.5, delta=0.01)

    def test_speed_then_mode_and_heartbeat_changes_value(self):
        port, chassis = self.make()
        chassis.open(0.0)
        port.requests.clear()
        chassis.step(1.0, 0.5, 0.0)
        to1 = [r for r in port.requests if r[0] == 1]
        # 先写目标转速，再切转速模式（手册的顺序），再读状态
        self.assertEqual(to1[:4], [om.heartbeat(1, 2), om.set_speed(1, 350), om.set_mode(1, om.MODE_SPEED), om.read_status(1)])
        port.requests.clear()
        chassis.step(1.04, 0.5, 0.0)                                        # 已是转速模式：不再写模式
        self.assertNotIn(om.set_mode(1, om.MODE_SPEED), port.requests)
        beats = [r for r in port.requests if r[2:4] == H('17 70')]
        self.assertEqual(beats, [])                                         # 心跳 0.1 s 一次
        chassis.step(1.12, 0.5, 0.0)
        self.assertIn(om.heartbeat(1, 3), port.requests)                    # 每次换个值

    def test_heartbeat_keeps_alive_and_its_loss_frees(self):
        port, chassis = self.make()
        chassis.open(0.0)
        run(chassis, [port], 0.5, 0.0, 3.0)
        self.assertTrue(all(m.alive(port.t) for m in port.motors.values()))
        for _ in range(60):
            port.advance(0.04)
        self.assertLess(abs(port.motors[1].speed), 1.0)

    def test_receive_only_only_reads(self):
        port, chassis = self.make()
        run(chassis, [port], 0.5, 0.0, 1.0, send=False)
        self.assertTrue(port.requests)
        self.assertTrue(all(r[1] in (om.FC_READ_INPUT, om.FC_READ_HOLDING) for r in port.requests))
        self.assertEqual(port.motors[1].target, 0.0)

    def test_power_cycle_restores_speed_mode(self):
        # 驱动器短暂断电重启：模式回到电流、目标清零。每 0.5 s 读一次模式，发现不是转速模式就重新切
        port, chassis = self.make()
        chassis.open(0.0)
        t = run(chassis, [port], 0.5, 0.0, 2.0)
        port.hold[1][om.REG_MODE] = om.MODE_CURRENT
        port.motors[1].speed_mode = False
        t = run(chassis, [port], 0.5, 0.0, 1.5, t)
        self.assertTrue(port.motors[1].speed_mode)
        self.assertGreater(abs(port.motors[1].speed), 300)

    def test_release_frees_both(self):
        port, chassis = self.make()
        chassis.open(0.0)
        run(chassis, [port], 0.5, 0.0, 2.0)
        chassis.release(2.0)
        self.assertFalse(port.motors[1].speed_mode)
        self.assertEqual(port.hold[2][om.REG_CURRENT], 0)
        self.assertEqual(port.hold[2][om.REG_MODE], om.MODE_CURRENT)

    def test_fault_and_silence(self):
        port, chassis = self.make()
        chassis.open(0.0)
        port.motors[2].fault = 16
        t = run(chassis, [port], 0.3, 0.0, 1.0)
        self.assertEqual(chassis.faults(), {2: 16})
        port.motors[1].powered = False
        port.requests.clear()
        t = run(chassis, [port], 0.3, 0.0, 1.0, t)
        telemetry = chassis.telemetry(t)
        self.assertEqual(telemetry['silent'], ['左'])
        self.assertTrue(telemetry['stale'])
        # 连续没应答后只每 0.5 s 试一次，不再每个周期都等超时
        self.assertLessEqual(sum(1 for r in port.requests if r[0] == 1), 6)
        self.assertIn('左轮（站号 1）：没有应答', chassis.link.last_error)
        port.motors[1].powered = True                                        # 恢复：重新切转速模式、继续跑
        t = run(chassis, [port], 0.3, 0.0, 2.0, t)
        self.assertEqual(chassis.telemetry(t)['silent'], [])


class TestSplitLinkThisCar(unittest.TestCase):
    """本车：左轮 485（站号 1，板载 /dev/ttyS3）、右轮 CAN（节点 75）。"""

    def make(self):
        port = SimModbusPort(POLES, (1,))
        can = SimOidBus(POLES, 74, 75)                                       # 74 那台不在 CAN 上
        link = SplitLink([Rs485Link(port, {1: 1}), CanLink(can, {2: 75})])
        return port, can, OidChassis(link, WHEELS)

    def test_kind_and_shape(self):
        port, can, chassis = self.make()
        self.assertEqual(chassis.link.kind, 'rs485+can')
        self.assertEqual(sorted(chassis.link.state), [1, 2])
        with self.assertRaisesRegex(ValueError, '各由一条'):
            SplitLink([Rs485Link(port, {1: 1}), CanLink(can, {1: 74})])

    def test_straight_pivot_and_mirror(self):
        port, can, chassis = self.make()
        chassis.open(0.0)
        t = run(chassis, [port, can], 0.5, 0.0, 4.0)
        o = chassis.odom
        self.assertAlmostEqual(o.v, 0.5, delta=0.01)
        self.assertAlmostEqual(o.w, 0.0, delta=1e-3)
        self.assertGreater(o.x, 1.5)
        self.assertGreater(port.motors[1].speed, 300)                         # 左轮（485）正转往前
        self.assertLess(can.motors[2].speed, -300)                            # 右轮（CAN）镜像
        self.assertEqual(can.motors[1].target, 0.0)                           # CAN 上没有左轮的事
        t = run(chassis, [port, can], 0.0, 0.0, 1.5, t)
        run(chassis, [port, can], 0.0, 0.8, 3.0, t)
        self.assertAlmostEqual(chassis.odom.w, 0.8, delta=0.02)
        self.assertGreater(chassis.link.rx, 0)
        self.assertEqual(chassis.link.errors, 0)

    def test_either_side_dropping_stops_updates(self):
        for side in ('485', 'can'):
            port, can, chassis = self.make()
            chassis.open(0.0)
            t = run(chassis, [port, can], 0.3, 0.0, 2.0)
            if side == '485':
                port.motors[1].powered = False
            else:
                can.motors[2].powered = False
            updates = []
            for _ in range(20):
                for d in (port, can):
                    d.advance(0.04)
                t += 0.04
                updates.append(chassis.step(t, 0.3, 0.0))
            self.assertFalse(any(updates[10:]), side)
            self.assertEqual(chassis.telemetry(t)['silent'], ['左'] if side == '485' else ['右'])


class TestBuildLink(unittest.TestCase):
    PARAMS = dict(left_bus='rs485', right_bus='can', can_channel='can0', can_left_id=1, can_right_id=2,
                  rs485_port='/dev/ttyS3', rs485_baud=115200, rs485_parity='N', rs485_left_id=1, rs485_right_id=2,
                  rs485_word_order='big', rs485_timeout_s=0.03, rs485_echo=False, heartbeat_s=0.1)

    def build(self, **over):
        opened = []

        class Port:
            def __init__(self, *a, **k):
                self.args, self.closed = (a, k), False
                opened.append(self)

            def close(self):
                self.closed = True
        can = mock.MagicMock()
        with mock.patch('oid_driver.SerialPort', Port), mock.patch('oid_driver.SocketCan', can):
            return build_link(dict(self.PARAMS, **over)), opened, can

    def test_this_car(self):
        link, ports, can = self.build()
        self.assertIsInstance(link, SplitLink)
        rs, cn = link.links
        self.assertEqual((rs.addresses, cn.node_ids), ({1: 1}, {2: 2}))
        self.assertEqual(ports[0].args, (('/dev/ttyS3', 115200, 'N'), {'echo': False}))
        can.assert_called_once_with('can0')
        self.assertEqual(bus_kind(self.PARAMS), 'rs485+can')
        self.assertEqual(bus_label(self.PARAMS), '/dev/ttyS3 + can0')
        self.assertEqual(DiffConfig(protocol=bus_kind(self.PARAMS)).protocol, 'rs485+can')

    def test_all_can_and_all_rs485(self):
        link, ports, can = self.build(left_bus='can')
        self.assertIsInstance(link, CanLink)
        self.assertEqual(link.node_ids, {1: 1, 2: 2})
        self.assertEqual(ports, [])
        link, ports, can = self.build(right_bus='rs485')
        self.assertIsInstance(link, Rs485Link)
        self.assertEqual(link.addresses, {1: 1, 2: 2})
        can.assert_not_called()

    def test_bad_values_and_cleanup(self):
        with self.assertRaisesRegex(ValueError, 'left_bus'):
            self.build(left_bus='modbus')
        with self.assertRaises(ValueError):
            DiffConfig(protocol='modbus')
        # CAN 打不开：已打开的串口要关掉
        opened = []

        class Port:
            def __init__(self, *a, **k):
                self.closed = False
                opened.append(self)

            def close(self):
                self.closed = True
        with mock.patch('oid_driver.SerialPort', Port), \
                mock.patch('oid_driver.SocketCan', mock.MagicMock(side_effect=OSError('no can0'))):
            with self.assertRaises(OSError):
                build_link(dict(self.PARAMS))
        self.assertTrue(opened[0].closed)


class TestParamsFile(unittest.TestCase):
    # ROS（rcl 的参数文件解析）按 YAML 1.1：不带引号的 y/n/yes/no/on/off/true/false 都是布尔
    BOOL = {'y': True, 'yes': True, 'on': True, 'true': True, 'n': False, 'no': False, 'off': False, 'false': False}

    def rcl_value(self, raw):
        if raw[:1] in ('"', "'"):
            return raw[1:-1]
        if raw.lower() in self.BOOL:
            return self.BOOL[raw.lower()]
        for cast in (int, float):
            try:
                return cast(raw)
            except ValueError:
                pass
        return raw

    def test_yaml_types_match_the_node(self):
        # ROS 按声明时的类型核对参数文件，类型不一致节点直接起不来（2026-09-29 实车：不带引号的 N 被读成 false）
        import re
        import oid_driver
        values = {}
        with open(os.path.join(ROOT, 'oid_protocol', 'oid.yaml'), encoding='utf-8') as fh:
            for line in fh:
                m = re.match(r'^ {4}(\w+):\s*("[^"]*"|\'[^\']*\'|[^#\s]+)', line)
                if m:
                    values[m.group(1)] = self.rcl_value(m.group(2))
        self.assertGreater(len(values), 20)
        for name, value in values.items():
            self.assertIn(name, oid_driver.PARAMS, name)
            default = oid_driver.PARAMS[name]
            if isinstance(default, bool) or isinstance(value, bool):
                ok = isinstance(value, bool) and isinstance(default, bool)
            else:
                ok = isinstance(value, type(default)) or (isinstance(default, float) and isinstance(value, int))
            self.assertTrue(ok, '%s：文件里是 %r，节点要 %s' % (name, value, type(default).__name__))
        self.assertEqual((values['left_bus'], values['right_bus'], values['rs485_parity']), ('rs485', 'can', 'N'))


class TestSerialPortFraming(unittest.TestCase):
    """SerialPort.transact：发完再等；应答可能分几段到、前面有杂字节、带回显；等不到返回 None。"""

    def port(self, incoming, echo=False):
        class FakeSerial:
            def __init__(self, *a, **k):
                self.chunks, self.written, self.in_waiting = list(incoming), [], 0
                self.out_waiting = 0
                self.write_timeout = k["write_timeout"]

            def reset_input_buffer(self):
                pass

            def write(self, data):
                self.written.append(bytes(data))
                return len(data)

            def flush(self):
                raise AssertionError("unbounded flush must not be called")

            def read(self, n):
                return self.chunks.pop(0) if self.chunks else b''

            def close(self):
                pass
        fake = types.ModuleType('serial')
        fake.Serial = FakeSerial
        with mock.patch.dict(sys.modules, {'serial': fake}):
            return oid_link.SerialPort('/dev/ttyS3', 115200, echo=echo)

    def test_split_reply_with_noise(self):
        req, reply = om.read_input(1, 5005, 1), H('01 04 02 00 30 B9 24')
        p = self.port([b'\xff', reply[:3], reply[3:]])
        self.assertEqual(p.transact(req, 0.2), reply)
        self.assertEqual(p.ser.written, [req])

    def test_echo_is_skipped(self):
        req = om.set_mode(1, om.MODE_SPEED)                                   # 写单个寄存器的应答 = 请求
        self.assertEqual(self.port([req, req], echo=True).transact(req, 0.2), req)
        self.assertIsNone(self.port([req], echo=True).transact(req, 0.05))    # 只有回显、驱动器没回

    def test_timeout(self):
        self.assertIsNone(self.port([]).transact(om.read_status(1), 0.02))

    def test_write_and_drain_are_bounded(self):
        p = self.port([])
        self.assertLessEqual(p.ser.write_timeout, 0.05)
        p.ser.out_waiting = 8
        import time
        begin = time.monotonic()
        with self.assertRaisesRegex(TimeoutError, 'did not drain'):
            p.transact(om.read_status(1), 0.01)
        self.assertLess(time.monotonic() - begin, 0.2)

    def test_partial_write_and_write_failure_abort_the_transaction(self):
        req = om.read_status(1)
        p = self.port([])
        p.ser.write = lambda data: len(data) - 1
        with self.assertRaisesRegex(TimeoutError, 'incomplete write'):
            p.transact(req, 0.02)
        p.ser.write = mock.Mock(side_effect=TimeoutError('write stalled'))
        with self.assertRaisesRegex(TimeoutError, 'write stalled'):
            p.transact(req, 0.02)
        self.assertLessEqual(p.ser.write_timeout, 0.02)


if __name__ == '__main__':
    unittest.main()
