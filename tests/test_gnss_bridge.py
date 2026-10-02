#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""平板串口转发（gnss_bridge.TabletSerialServer / BluetoothSerialServer）测试：本机回环 TCP、socketpair，
不需要平板，也不碰 BlueZ。"""
import os
import socket
import sys
import time
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))

import gnss_bridge  # noqa: E402
from gnss_bridge import BluetoothSerialServer, TabletSerialServer  # noqa: E402


def read_until(server, predicate, timeout=2.0):
    got = b''
    deadline = time.time() + timeout
    while time.time() < deadline and not predicate(got):
        got += server.read(4096)
    return got


class TestTabletSerialServer(unittest.TestCase):

    def setUp(self):
        self.connected = 0
        self.server = TabletSerialServer(0, host='127.0.0.1', timeout=0.02, on_connect=self._on_connect)

    def tearDown(self):
        self.server.close()

    def _on_connect(self):
        self.connected += 1

    def _client(self):
        c = socket.create_connection(('127.0.0.1', self.server.port), timeout=2)
        self.addCleanup(c.close)
        read_until(self.server, lambda _: self.server.client is not None)
        return c

    def test_no_tablet_reads_nothing(self):
        self.assertEqual(self.server.read(), b'')
        self.assertEqual(self.server.status()['connected'], False)
        self.assertFalse(hasattr(self.server, 'write'))               # 单向：车不往平板写（差分由平板自己连 CORS）

    def test_bytes_pass_through_unchanged(self):
        c = self._client()
        self.assertEqual(self.connected, 1)
        c.sendall(b'#BESTPOSA,COM1;...*00000000\r\n')
        self.assertEqual(read_until(self.server, lambda d: d.endswith(b'\r\n')), b'#BESTPOSA,COM1;...*00000000\r\n')
        st = self.server.status()
        self.assertTrue(st['connected'])
        self.assertEqual((st['bytes_in'], st['connections']), (29, 1))

    def test_new_connection_replaces_old_and_close_is_noticed(self):
        old = self._client()
        new = socket.create_connection(('127.0.0.1', self.server.port), timeout=2)
        self.addCleanup(new.close)
        read_until(self.server, lambda _: self.server.connections == 2)
        self.assertEqual(self.connected, 2)
        old.settimeout(1)
        self.assertEqual(old.recv(1), b'')                            # 旧连接被车端关掉
        new.sendall(b'x')
        self.assertEqual(read_until(self.server, lambda d: d == b'x'), b'x')
        new.close()
        read_until(self.server, lambda _: self.server.client is None)
        self.assertFalse(self.server.status()['connected'])


class TestBluetoothSerialServer(unittest.TestCase):
    """BlueZ 交来的连接用 socketpair 代替（adopt 收到的就是一条已连好的 socket）。"""

    def setUp(self):
        self.connected = 0
        self.server = BluetoothSerialServer(timeout=0.02, on_connect=self._on_connect, register=False)

    def tearDown(self):
        self.server.close()

    def _on_connect(self):
        self.connected += 1

    def _tablet(self, peer='AA:BB:CC:DD:EE:01'):
        car_end, tablet_end = socket.socketpair()
        self.addCleanup(tablet_end.close)
        self.server.adopt(car_end, peer)
        return tablet_end

    def test_doctests(self):
        import doctest
        self.assertEqual(doctest.testmod(gnss_bridge).failed, 0)

    def test_nothing_connected_reads_nothing(self):
        self.assertEqual(self.server.read(), b'')
        st = self.server.status()
        self.assertEqual((st['connected'], st['registered'], st['channel']), (False, False, gnss_bridge.BT_CHANNEL))
        self.assertFalse(hasattr(self.server, 'write'))               # 单向：车不往平板写

    def test_bytes_pass_through_unchanged(self):
        t = self._tablet()
        self.assertEqual(self.connected, 1)
        t.sendall(b'#HEADINGA,COM1;...*00000000\r\n')
        self.assertEqual(read_until(self.server, lambda d: d.endswith(b'\r\n')), b'#HEADINGA,COM1;...*00000000\r\n')
        st = self.server.status()
        self.assertEqual((st['connected'], st['peer'], st['bytes_in'], st['connections']), (True, 'AA:BB:CC:DD:EE:01', 29, 1))

    def test_new_connection_replaces_old_and_disconnect_is_noticed(self):
        old = self._tablet('AA:BB:CC:DD:EE:01')
        new = self._tablet('AA:BB:CC:DD:EE:02')
        self.assertEqual((self.connected, self.server.peer), (2, 'AA:BB:CC:DD:EE:02'))
        old.settimeout(1)
        self.assertEqual(old.recv(1), b'')                            # 旧连接被车端关掉
        new.sendall(b'x')
        self.assertEqual(read_until(self.server, lambda d: d == b'x'), b'x')
        new.close()
        read_until(self.server, lambda _: self.server.client is None)
        self.assertEqual((self.server.status()['connected'], self.server.peer), (False, None))

    def test_bluez_asks_to_disconnect(self):
        t = self._tablet()
        self.server.drop_peer()
        t.settimeout(1)
        self.assertEqual(t.recv(1), b'')
        self.assertFalse(self.server.status()['connected'])


if __name__ == '__main__':
    unittest.main()
