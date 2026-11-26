#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""平板 USB 数据线转发（2026-11-26 用户「先适配下usb直连传」）：

- usb_gadget.ensure：只在 J17 的控制器 fc400000.usb 上建本工具的 CDC-ACM gadget；已经建好绑好时什么也不改；
  控制器不存在 / 被占 / 同名 gadget 不是自己的 → 报错、不动别人的配置，也绝不退到 J16（adb 口）。
- gnss_bridge.TabletUsbSerial：tty 原始模式，二进制原样（回车换行、^C、XON/XOFF 都不转换不吞），不回显给平板；
  平板拔线（EOF / EIO）后关掉、1 s 后重开；建 gadget 失败只写进状态、隔 1 s 再试。
- rtk_node：USB 线车停着且已连续出数据 1 s 时从 CAN 换过来，车动着不换；USB 断了 2 s 照常换回 CAN；
  用 USB 那一路时车端 CORS 暂停、不往 USB 写改正数。
"""
import os
import sys
import tempfile
import threading
import unittest
from pathlib import Path

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, ROOT)

import ros_stubs  # noqa: E402

ros_stubs.install()
for _name in ('Imu', 'NavSatFix', 'NavSatStatus'):
    setattr(sys.modules['sensor_msgs.msg'], _name, ros_stubs._simple(_name))

import gnss_bridge  # noqa: E402
import rtk_node  # noqa: E402
import usb_gadget  # noqa: E402
from um982 import Um982Stream  # noqa: E402


class FakeSys:
    """假的 configfs（usb_gadget）和 /sys/class/udc：两个控制器，J16 被 adb 占着。"""

    def __init__(self, tmp, j17_function=''):
        self.root = Path(tmp, 'usb_gadget')
        self.udcs = Path(tmp, 'udc')
        self.root.mkdir()
        for name, fn in (('fc000000.usb', 'ffs.adb'), (usb_gadget.J17_UDC, j17_function)):
            (self.udcs / name).mkdir(parents=True)
            (self.udcs / name / 'function').write_text(fn + '\n')
            (self.udcs / name / 'state').write_text('not attached\n')
        adb = self.root / 'rockchip'
        (adb / 'functions/ffs.adb').mkdir(parents=True)
        (adb / 'UDC').write_text('fc000000.usb\n')

    def ensure(self, udc=None):
        return usb_gadget.ensure(udc, root=self.root, udcs=self.udcs)

    def gadget(self):
        return self.root / usb_gadget.NAME


class Gadget(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def test_creates_on_j17_and_is_idempotent(self):
        fs = FakeSys(self.tmp.name)
        info = fs.ensure()
        g = fs.gadget()
        self.assertEqual(info['udc'], 'fc400000.usb')
        self.assertTrue(info['created'])
        self.assertEqual((g / 'UDC').read_text().strip(), 'fc400000.usb')
        self.assertEqual((g / 'strings/0x409/serialnumber').read_text().strip(), usb_gadget.SERIAL)
        self.assertTrue((g / 'configs/c.1/acm.usb0').is_symlink())
        self.assertEqual((g / 'configs/c.1/bmAttributes').read_text().strip(), '0xc0')   # 自供电，不从平板取电
        # 内核建 acm 功能时给的端口号（假 configfs 里手动写）
        (g / 'functions/acm.usb0/port_num').write_text('0\n')
        before = sorted((p, p.stat().st_mtime_ns) for p in g.rglob('*') if p.is_file())
        again = fs.ensure()
        self.assertEqual(again, dict(udc='fc400000.usb', serial_device='/dev/ttyGS0', created=False))
        self.assertEqual(before, sorted((p, p.stat().st_mtime_ns) for p in g.rglob('*') if p.is_file()))
        # J16 的 adb 一点没动
        self.assertEqual((fs.root / 'rockchip/UDC').read_text().strip(), 'fc000000.usb')

    def test_rebinds_own_unbound_gadget(self):
        fs = FakeSys(self.tmp.name)
        fs.ensure()
        (fs.gadget() / 'UDC').write_text('\n')
        info = fs.ensure()
        self.assertFalse(info['created'])
        self.assertEqual((fs.gadget() / 'UDC').read_text().strip(), 'fc400000.usb')

    def test_busy_j17_is_left_alone_and_never_falls_back_to_j16(self):
        fs = FakeSys(self.tmp.name, j17_function='configfs-gadget.other')
        with self.assertRaisesRegex(usb_gadget.GadgetError, '已被别的功能占用'):
            fs.ensure()
        self.assertFalse(fs.gadget().exists())
        self.assertEqual((fs.root / 'rockchip/UDC').read_text().strip(), 'fc000000.usb')

    def test_j17_bound_by_other_gadget_is_left_alone(self):
        fs = FakeSys(self.tmp.name)
        other = fs.root / 'g2'
        other.mkdir()
        (other / 'UDC').write_text('fc400000.usb\n')
        with self.assertRaisesRegex(usb_gadget.GadgetError, '已被别的 gadget 占用'):
            fs.ensure()
        self.assertFalse(fs.gadget().exists())

    def test_missing_controller_or_configfs(self):
        fs = FakeSys(self.tmp.name)
        with self.assertRaisesRegex(usb_gadget.GadgetError, '没有 USB 设备控制器'):
            fs.ensure('fc800000.usb')
        self.assertFalse(fs.gadget().exists())
        with self.assertRaisesRegex(usb_gadget.GadgetError, 'configfs 不可用'):
            usb_gadget.ensure(root=Path(self.tmp.name, 'none'), udcs=fs.udcs)

    def test_foreign_gadget_with_same_name_is_not_touched(self):
        fs = FakeSys(self.tmp.name)
        g = fs.gadget()
        (g / 'strings/0x409').mkdir(parents=True)
        (g / 'strings/0x409/serialnumber').write_text('someone-else\n')
        with self.assertRaisesRegex(usb_gadget.GadgetError, '不是本工具建的'):
            fs.ensure()
        with self.assertRaisesRegex(usb_gadget.GadgetError, '不是本工具建的'):
            usb_gadget.disable(root=fs.root)
        self.assertFalse((g / 'UDC').exists())

    def test_env_overrides_controller(self):
        old = os.environ.get(usb_gadget.UDC_ENV)
        os.environ[usb_gadget.UDC_ENV] = 'fc000000.usb'
        try:
            self.assertEqual(usb_gadget.preferred_udc(), 'fc000000.usb')
            fs = FakeSys(self.tmp.name)
            with self.assertRaisesRegex(usb_gadget.GadgetError, '占用'):   # adb 占着也不抢
                fs.ensure()
        finally:
            if old is None:
                os.environ.pop(usb_gadget.UDC_ENV)
            else:
                os.environ[usb_gadget.UDC_ENV] = old


class Pty:
    """一对伪终端：slave 当板子的 /dev/ttyGS0，master 当平板那头。"""

    def __init__(self):
        self.master, self.slave = os.openpty()
        self.path = os.ttyname(self.slave)

    def opener(self, path):
        assert path == self.path
        return os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)

    def close(self):
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass


def drain(link, want, tries=50):
    out = b''
    for _ in range(tries):
        out += link.read(4096)
        if len(out) >= want:
            break
    return out


class UsbSerial(unittest.TestCase):
    def test_binary_passes_unchanged_and_nothing_echoes_back(self):
        pty = Pty()
        self.addCleanup(pty.close)
        link = gnss_bridge.TabletUsbSerial(device=pty.path, opener=pty.opener, timeout=0.05)
        self.addCleanup(link.close)
        self.assertEqual(link.read(), b'')          # 打开（原始模式），还没数据
        self.assertTrue(link.status()['open'])
        data = bytes(range(256)) + b'#BESTPOSA,COM2*12345678\r\n\xaa\x44\x12\x1c\r\r\n\x03\x11\x13\x04'
        os.write(pty.master, data)
        self.assertEqual(drain(link, len(data)), data)
        st = link.status()
        self.assertTrue(st['connected'])
        self.assertEqual(st['bytes_in'], len(data))
        os.set_blocking(pty.master, False)
        try:
            echoed = os.read(pty.master, 4096)
        except BlockingIOError:
            echoed = b''
        self.assertEqual(echoed, b'')

    def test_hangup_closes_and_reopens_after_retry(self):
        first = Pty()
        clock = [10.0]
        link = gnss_bridge.TabletUsbSerial(device=first.path, opener=first.opener, timeout=0.05,
                                           clock=lambda: clock[0])
        self.addCleanup(link.close)
        link.read()
        os.write(first.master, b'abc')
        self.assertEqual(drain(link, 3), b'abc')
        os.close(first.master)                      # 平板拔线：tty 挂断
        for _ in range(20):
            link.read()
            if link.fd is None:
                break
        self.assertIsNone(link.fd)
        self.assertEqual(link.status()['hangups'], 1)
        self.assertFalse(link.status()['connected'])
        self.assertIsNotNone(link.status()['error'])
        os.close(first.slave)
        second = Pty()
        self.addCleanup(second.close)
        link.device = link.path = second.path
        link.opener = second.opener
        self.assertEqual(link.read(), b'')          # 1 s 内不重开
        self.assertIsNone(link.fd)
        clock[0] += gnss_bridge.USB_RETRY_S
        link.read()
        self.assertIsNotNone(link.fd)
        self.assertEqual(link.status()['opens'], 2)
        self.assertIsNone(link.status()['error'])
        os.write(second.master, b'xyz')
        self.assertEqual(drain(link, 3), b'xyz')

    def test_gadget_setup_failure_is_reported_and_retried(self):
        pty = Pty()
        self.addCleanup(pty.close)
        clock = [0.0]
        calls = []

        def setup():
            calls.append(clock[0])
            if len(calls) == 1:
                raise usb_gadget.GadgetError('USB 控制器 fc400000.usb 已被别的功能占用，不动它')
            return dict(udc='fc400000.usb', serial_device=pty.path, created=True)

        link = gnss_bridge.TabletUsbSerial(setup=setup, opener=pty.opener, timeout=0.01, clock=lambda: clock[0])
        self.addCleanup(link.close)
        self.assertEqual(link.read(), b'')
        self.assertIn('占用', link.status()['error'])
        self.assertFalse(link.status()['open'])
        link.read()
        self.assertEqual(len(calls), 1)
        clock[0] += 1.0
        link.read()
        self.assertEqual(len(calls), 2)
        st = link.status()
        self.assertTrue(st['open'])
        self.assertEqual((st['udc'], st['device'], st['error']), ('fc400000.usb', pty.path, None))


class Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class Log:
    def info(self, *_):
        pass

    warning = error = info


def bare_node():
    """不起线程、不开串口的 RtkNode：只测换路。"""
    node = rtk_node.RtkNode.__new__(rtk_node.RtkNode)
    node.stream = Um982Stream()
    node.stream_lock = threading.Lock()
    node.write_lock = threading.Lock()
    node.source, node.source_rx, node.usb_run_since = None, {}, None
    node.pub_raw = node.pub_fix = node.pub_heading = Pub()
    node.ser = None
    node.get_logger = lambda: Log()
    return node


class Arbitration(unittest.TestCase):
    def run_both(self, node, t0, t1, speed, dt=0.05):
        """CAN、USB 两路同时出数据，CAN 那一路的流里接收机测速为 speed。"""
        t = t0
        while t < t1:
            node.on_serial_data(b'', t, 'can')
            if node.source == 'can':
                node.stream.last['vel'] = dict(kind='vel', t=t, t_rx=t, speed=speed)
            node.on_serial_data(b'', t + 0.01, 'tablet_usb')
            t += dt

    def test_usb_takes_over_from_can_only_when_stopped(self):
        node = bare_node()
        node.on_serial_data(b'', 0.0, 'can')
        self.assertEqual(node.source, 'can')
        self.run_both(node, 0.05, 3.0, speed=0.8)          # 车在走：一直不换
        self.assertEqual(node.source, 'can')
        self.run_both(node, 3.0, 3.5, speed=0.01)          # 停下，USB 早已连续 3 s：马上换
        self.assertEqual(node.source, 'tablet_usb')

    def test_usb_needs_one_second_of_steady_data(self):
        node = bare_node()
        node.on_serial_data(b'', 0.0, 'can')
        node.stream.last['vel'] = dict(kind='vel', t=0.0, t_rx=0.0, speed=0.0)
        t = 0.05
        while t < 0.9:
            node.on_serial_data(b'', t, 'can')
            node.stream.last['vel'] = dict(kind='vel', t=t, t_rx=t, speed=0.0)
            node.on_serial_data(b'', t + 0.01, 'tablet_usb')
            t += 0.05
        self.assertEqual(node.source, 'can')
        node.on_serial_data(b'', 2.0, 'tablet_usb')         # 中间断了 1 s：重新数
        self.assertEqual(node.source, 'can')

    def test_back_to_can_after_usb_silent_and_cors_stays_paused(self):
        node = bare_node()
        node.on_serial_data(b'', 0.0, 'can')
        self.run_both(node, 0.05, 1.5, speed=0.0)
        self.assertEqual(node.source, 'tablet_usb')
        self.assertIn('tablet_usb', rtk_node.TABLET_SOURCES)
        node.on_serial_data(b'', 2.5, 'can')                # USB 才停 1 s：不换
        self.assertEqual(node.source, 'tablet_usb')
        node.on_serial_data(b'', 3.6, 'can')                # 停了 2 s 以上：换回 CAN
        self.assertEqual(node.source, 'can')

    def test_receiver_on_car_serial_is_never_preempted(self):
        node = bare_node()
        node.on_serial_data(b'', 0.0, 'usb')
        t = 0.05
        while t < 2.0:
            node.on_serial_data(b'', t, 'usb')
            node.stream.last['vel'] = dict(kind='vel', t=t, t_rx=t, speed=0.0)
            node.on_serial_data(b'', t + 0.01, 'tablet_usb')
            t += 0.05
        self.assertEqual(node.source, 'usb')

    def test_no_corrections_written_to_usb_cable(self):
        node = bare_node()
        node.on_serial_data(b'', 0.0, 'tablet_usb')
        self.assertEqual(node.source, 'tablet_usb')
        written = []
        node.ser = type('S', (), {'write': lambda self, d: written.append(d)})()
        node.write_corrections(b'\xd3\x00\x00')
        self.assertEqual(written, [])


class Status(unittest.TestCase):
    def test_usb_source_status_fields(self):
        node = bare_node()
        node.tablet = node.bt = node.can = None
        node.tablet_error = node.port_error = None
        node.mount_offset_deg, node.baseline_yaw_body_deg = 0.0, -90.0
        node.cors = type('C', (), dict(client=None, configured=False, error=None, enabled=True,
                                       applied_revision=None, paused='tablet'))()
        node.usb = type('U', (), {'status': lambda self: dict(device='/dev/ttyGS0', open=True, connected=True,
                                                              peer=None, udc='fc400000.usb', opens=1, hangups=0,
                                                              bytes_in=123, error=None)})()
        node.on_serial_data(b'', 0.0, 'tablet_usb')
        st = node.status(0.1)
        self.assertEqual(st['source'], 'tablet_usb')
        self.assertTrue(st['port_open'])
        self.assertIsNone(st['port_error'])
        self.assertEqual(st['tablet_usb']['udc'], 'fc400000.usb')
        self.assertTrue(st['tablet_usb']['listening'])
        node.usb = None
        st = node.status(0.1)
        self.assertFalse(st['tablet_usb']['listening'])
        self.assertFalse(st['port_open'])

    def test_app_link_passes_source_and_connection(self):
        import app_link
        gnss = dict(_rx_monotonic=5.0, source='tablet_usb', tablet_usb=dict(connected=True),
                    can=dict(connected=False))
        out = app_link.rtk_telemetry(gnss, 5.5)
        self.assertEqual(out['source'], 'tablet_usb')
        self.assertTrue(out['tablet_connected'])
        gnss['tablet_usb']['connected'] = False
        self.assertFalse(app_link.rtk_telemetry(gnss, 5.5)['tablet_connected'])


if __name__ == '__main__':
    unittest.main()
