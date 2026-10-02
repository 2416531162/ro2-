#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车的 WiFi 方式切换（radar_system/car_network.py）和 app 协议里的 network 指令 / 字段。nmcli 用假的，不碰真网卡。"""
import doctest
import json
import os
import sys
import time
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, ROOT)

import car_network as cn  # noqa: E402


class FakeNmcli:
    """记下每条 nmcli 命令；按「连着谁、有哪些配置」回输出。fail 里的子命令返回失败。"""

    def __init__(self, active='pattern', profiles=('pattern', 'vivo-hotspot'), fail=(), security=None):
        self.active = active
        self.profiles = list(profiles)
        self.fail = set(fail)
        self.calls = []
        # 车热点配置现在的 psk / proto / pairwise / group / pmf / band / channel（nmcli -s -g 的输出顺序）
        self.security = list(security or [cn.HOTSPOT_PSK, 'rsn', 'ccmp', 'ccmp', '1', 'a', '36'])
        self.band = None        # 车热点配置的频段（add / modify 里设的）；fail 里有 'up5' 时 5 GHz 起不来

    def __call__(self, args, timeout=None):
        self.calls.append(list(args))
        a = list(args)
        if a[:4] == ['-t', '-f', 'DEVICE,TYPE', 'dev']:
            return True, 'eth0:ethernet\nwlan0:wifi\np2p-dev-wlan0:wifi-p2p\n'
        if a[:3] == ['-t', '-f', 'NAME,TYPE,DEVICE']:
            return True, ('%s:802-11-wireless:wlan0\n' % self.active) if self.active else ''
        if a[:3] == ['-t', '-g', '802-11-wireless.ssid']:
            return True, {'pattern': 'pattern\n', 'vivo-hotspot': 'vivo#iQOO\n'}.get(a[-1], a[-1] + '\n')
        if a[:2] == ['-s', '-g']:
            if a[-1] not in self.profiles:
                return False, 'Error: %s - no such connection profile.' % a[-1]
            return True, '\n'.join(self.security) + '\n'
        if a[:4] == ['-t', '-f', 'NAME', 'con']:
            return True, '\n'.join(self.profiles) + '\n'
        if '802-11-wireless.band' in a and a[:2] in (['con', 'add'], ['con', 'modify']):
            self.band = a[a.index('802-11-wireless.band') + 1]
        if a[:2] == ['con', 'add']:
            if 'add' in self.fail:
                return False, 'Error: failed'
            self.profiles.append(cn.HOTSPOT_CON)
            return True, ''
        if a[:2] == ['con', 'up']:
            if 'up5' in self.fail and self.band == 'a':
                return False, 'Error: Connection activation failed: 802.1X supplicant took too long to authenticate'
            if 'up' in self.fail:
                return False, 'Error: Connection activation failed: AP mode not supported'
            self.active = a[2]
            return True, ''
        if a[:2] == ['con', 'down']:
            if self.active == a[2]:
                self.active = None
            return True, ''
        if a[:2] == ['dev', 'connect']:
            if 'connect' in self.fail:
                return False, 'Error: no network'
            self.active = 'pattern'
            return True, ''
        return True, ''

    def ran(self, *prefix):
        return [c for c in self.calls if c[:len(prefix)] == list(prefix)]


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def network(nm, joined=None, clock=None):
    count = joined if joined is not None else [0]
    net = cn.CarNetwork(nmcli=nm, app_joined=lambda: count[0], clock=clock or Clock(),
                        read_mac=lambda dev: 'a4:7e:fa:3f:a2:10', delay_s=0, spawn=lambda fn: fn())
    net.refresh()
    return net, count


class CarNetworkTest(unittest.TestCase):
    def test_doctests(self):
        self.assertEqual(doctest.testmod(cn).failed, 0)

    def test_reports_client_wifi_and_hotspot_name(self):
        net, _ = network(FakeNmcli(active='pattern'))
        st = net.status()
        self.assertEqual(('client', 'pattern', 'RTKCAR-A2-10', '192.168.42.1'),
                         (st['mode'], st['ssid'], st['hotspot_ssid'], st['hotspot_ip']))
        self.assertNotIn('revert_to', st)
        net2, _ = network(FakeNmcli(active=cn.HOTSPOT_CON, profiles=('pattern', cn.HOTSPOT_CON)))
        self.assertEqual(('hotspot', 'RTKCAR-A2-10'), (net2.status()['mode'], net2.status()['ssid']))
        info = net.hotspot_info()
        self.assertEqual(('RTKCAR-A2-10', cn.HOTSPOT_PSK, '192.168.42.1'), (info['hotspot_ssid'], info['hotspot_psk'], info['hotspot_ip']))
        self.assertGreaterEqual(len(cn.HOTSPOT_PSK), 8)

    def test_switch_to_hotspot_creates_profile_once_and_enables_autoconnect(self):
        nm = FakeNmcli(active='pattern')
        net, _ = network(nm)
        ok, why = net.request('hotspot')
        self.assertTrue(ok, why)
        add = nm.ran('con', 'add')
        self.assertEqual(1, len(add))
        cmd = add[0]
        for key, value in (('802-11-wireless.mode', 'ap'), ('ipv4.method', 'shared'), ('ipv4.addresses', '192.168.42.1/24'),
                           ('ssid', 'RTKCAR-A2-10'), ('wifi-sec.psk', cn.HOTSPOT_PSK), ('autoconnect', 'no'),
                           ('wifi-sec.proto', 'rsn'), ('wifi-sec.pairwise', 'ccmp'), ('wifi-sec.pmf', 'disable'),
                           ('802-11-wireless.band', 'a'), ('802-11-wireless.channel', '36')):
            self.assertEqual(value, cmd[cmd.index(key) + 1], key)
        self.assertIn(['con', 'modify', cn.HOTSPOT_CON, 'connection.autoconnect', 'yes'], nm.calls)
        self.assertIn(['con', 'up', cn.HOTSPOT_CON, 'ifname', 'wlan0'], nm.calls)
        st = net.status()
        self.assertEqual('hotspot', st['mode'])
        self.assertEqual('client', st['revert_to'])
        self.assertIsNone(st['error'])
        # 其它 WiFi 配置一律不动
        self.assertFalse([c for c in nm.calls if c[:2] == ['con', 'modify'] and c[2] != cn.HOTSPOT_CON])
        # 已经是热点：不再切；配置已存在时不重复建
        self.assertEqual(False, net.request('hotspot')[0] if net.revert is None else False)

    def test_switch_back_to_client_turns_hotspot_autoconnect_off(self):
        nm = FakeNmcli(active=cn.HOTSPOT_CON, profiles=('pattern', cn.HOTSPOT_CON))
        net, _ = network(nm)
        self.assertTrue(net.request('client')[0])
        self.assertIn(['con', 'modify', cn.HOTSPOT_CON, 'connection.autoconnect', 'no'], nm.calls)
        self.assertIn(['con', 'down', cn.HOTSPOT_CON], nm.calls)
        self.assertIn(['dev', 'connect', 'wlan0'], nm.calls)
        self.assertEqual(('client', 'pattern'), (net.status()['mode'], net.status()['ssid']))
        self.assertFalse(nm.ran('con', 'add'))

    def test_existing_hotspot_profile_gets_wpa2_only_security(self):
        # 早先建的配置（WPA/TKIP + PMF）切过去时也要改成 WPA2-CCMP、关 PMF，否则有的平板连不上
        nm = FakeNmcli(active='pattern', profiles=('pattern', cn.HOTSPOT_CON))
        net, _ = network(nm)
        self.assertTrue(net.request('hotspot')[0])
        self.assertFalse(nm.ran('con', 'add'))
        mod = [c for c in nm.calls if c[:3] == ['con', 'modify', cn.HOTSPOT_CON] and 'wifi-sec.psk' in c]
        self.assertEqual(1, len(mod))
        for key, value in (('wifi-sec.proto', 'rsn'), ('wifi-sec.group', 'ccmp'), ('wifi-sec.pmf', 'disable')):
            self.assertEqual(value, mod[0][mod[0].index(key) + 1], key)

    def test_hotspot_falls_back_to_2g_when_5g_will_not_start(self):
        # 5 GHz 起不来（驱动 / 管制区域）：配置改回 2.4 GHz 再起，车不能因此没网；原因给 app 看
        nm = FakeNmcli(active='pattern', fail={'up5'})
        net, _ = network(nm)
        self.assertTrue(net.request('hotspot')[0])
        ups = nm.ran('con', 'up', cn.HOTSPOT_CON)
        self.assertEqual(2, len(ups))
        self.assertIn(['con', 'modify', cn.HOTSPOT_CON] + cn.FALLBACK_BAND, nm.calls)
        st = net.status()
        self.assertEqual('hotspot', st['mode'])
        self.assertIn('已改用 2.4 GHz', st['error'])

    def test_sync_profile_moves_old_2g_hotspot_to_5g(self):
        # 早先建在 2.4 GHz 的热点（密码、加密都对）：服务起来时改到 5 GHz 36 信道并重启热点
        nm = FakeNmcli(active=cn.HOTSPOT_CON, profiles=('pattern', cn.HOTSPOT_CON),
                       security=[cn.HOTSPOT_PSK, 'rsn', 'ccmp', 'ccmp', '1', 'bg', '0'])
        net, _ = network(nm)
        self.assertTrue(net.sync_profile())
        mod = nm.ran('con', 'modify', cn.HOTSPOT_CON)[0]
        self.assertEqual(('a', '36'), (mod[mod.index('802-11-wireless.band') + 1], mod[mod.index('802-11-wireless.channel') + 1]))
        self.assertEqual(1, len(nm.ran('con', 'up', cn.HOTSPOT_CON)))
        self.assertEqual(('hotspot', None), (net.status()['mode'], net.status()['error']))

    def test_sync_profile_falls_back_when_5g_will_not_start(self):
        nm = FakeNmcli(active=cn.HOTSPOT_CON, profiles=('pattern', cn.HOTSPOT_CON), fail={'up5'},
                       security=[cn.HOTSPOT_PSK, 'rsn', 'ccmp', 'ccmp', '1', 'bg', '0'])
        net, _ = network(nm)
        self.assertTrue(net.sync_profile())
        self.assertEqual('bg', nm.band)
        self.assertEqual('hotspot', net.status()['mode'])
        self.assertIn('已改用 2.4 GHz', net.status()['error'])

    def test_sync_profile_leaves_matching_hotspot_alone(self):
        nm = FakeNmcli(active=cn.HOTSPOT_CON, profiles=('pattern', cn.HOTSPOT_CON))
        net, _ = network(nm)
        self.assertFalse(net.sync_profile())
        self.assertFalse(nm.ran('con', 'modify'))
        self.assertFalse(nm.ran('con', 'up'))

    def test_sync_profile_fixes_old_password_and_restarts_running_hotspot(self):
        # 早先版本的密码 + WPA/TKIP + PMF：改成这一版的，热点正开着就重启让它生效，hello 报的密码才是真的
        nm = FakeNmcli(active=cn.HOTSPOT_CON, profiles=('pattern', cn.HOTSPOT_CON),
                       security=['rtkcar2026', 'wpa,rsn', 'tkip,ccmp', 'tkip,ccmp', '0'])
        net, _ = network(nm)
        self.assertTrue(net.sync_profile())
        mod = nm.ran('con', 'modify', cn.HOTSPOT_CON)
        self.assertEqual(1, len(mod))
        for key, value in (('wifi-sec.psk', cn.HOTSPOT_PSK), ('wifi-sec.proto', 'rsn'), ('wifi-sec.pmf', 'disable')):
            self.assertEqual(value, mod[0][mod[0].index(key) + 1], key)
        self.assertEqual([['con', 'up', cn.HOTSPOT_CON, 'ifname', 'wlan0']], nm.ran('con', 'up'))
        st = net.status()
        self.assertEqual(('hotspot', False, None), (st['mode'], st['busy'], st['error']))
        self.assertNotIn('revert_to', st)     # 不是切换，不等平板、不切回

    def test_sync_profile_in_client_mode_only_edits_profile(self):
        nm = FakeNmcli(active='pattern', profiles=('pattern', cn.HOTSPOT_CON), security=['rtkcar2026', 'rsn', 'ccmp', 'ccmp', '1'])
        net, _ = network(nm)
        self.assertTrue(net.sync_profile())
        self.assertEqual(1, len(nm.ran('con', 'modify', cn.HOTSPOT_CON)))
        self.assertFalse(nm.ran('con', 'up'))
        self.assertEqual('client', net.status()['mode'])

    def test_sync_profile_without_hotspot_profile_does_nothing(self):
        nm = FakeNmcli(active='pattern')
        net, _ = network(nm)
        self.assertIsNone(net.sync_profile())
        self.assertFalse(nm.ran('con', 'modify'))
        self.assertFalse(nm.ran('con', 'add'))

    def test_no_tablet_within_window_reverts(self):
        clock = Clock()
        nm = FakeNmcli(active='pattern')
        net, joined = network(nm, clock=clock)
        net.request('hotspot')
        clock.t += cn.REVERT_S - 1
        net.tick()
        self.assertEqual('hotspot', net.status()['mode'])
        self.assertEqual(1, net.status()['revert_in_s'])
        clock.t += 2
        net.tick()
        st = net.status()
        self.assertEqual('client', st['mode'])
        self.assertNotIn('revert_to', st)
        self.assertIn('没有平板连上来', st['error'])

    def test_tablet_joining_keeps_new_mode(self):
        clock = Clock()
        net, joined = network(FakeNmcli(active='pattern'), joined=[3], clock=clock)
        net.request('hotspot')
        net.tick()
        self.assertEqual('client', net.status()['revert_to'])   # 切换前的连接不算
        joined[0] += 1
        net.tick()
        clock.t += cn.REVERT_S * 2
        net.tick()
        st = net.status()
        self.assertEqual('hotspot', st['mode'])
        self.assertNotIn('revert_to', st)

    def test_failed_hotspot_goes_back_and_reports_why(self):
        nm = FakeNmcli(active='pattern', fail=('up',))
        net, _ = network(nm)
        self.assertTrue(net.request('hotspot')[0])
        st = net.status()
        self.assertEqual('client', st['mode'])
        self.assertIn('AP mode not supported', st['error'])
        self.assertNotIn('revert_to', st)
        self.assertFalse(st['busy'])

    def test_rejects_bad_requests(self):
        net, _ = network(FakeNmcli(active='pattern'))
        self.assertFalse(net.request('router')[0])
        self.assertEqual((False, '已经是这种方式'), net.request('client'))
        none = cn.CarNetwork(nmcli=lambda a, timeout=None: (True, 'eth0:ethernet\n'), spawn=lambda fn: fn())
        none.refresh()
        self.assertIsNone(none.status()['mode'])
        self.assertFalse(none.request('hotspot')[0])

    def test_status_without_nmcli(self):
        net = cn.CarNetwork(nmcli=lambda a, timeout=None: (False, 'nmcli: not found'))
        net.refresh()
        self.assertEqual((None, None), (net.status()['mode'], net.status()['hotspot_ssid']))


class AppProtocolNetworkTest(unittest.TestCase):
    def setUp(self):
        import app_link
        self.al = app_link

    def test_parse_network_command(self):
        self.assertEqual({'type': 'network', 'id': 'n1', 'mode': 'hotspot'},
                         self.al.parse_command(json.dumps({'type': 'network', 'id': 'n1', 'mode': 'hotspot'})))
        with self.assertRaises(self.al.CommandError):
            self.al.parse_command('{"type":"network","mode":"router"}')

    def test_switch_only_when_parked(self):
        now = time.monotonic()
        idle = {'motion': {'mode': 'IDLE'}, 'line': {'phase': 'IDLE', '_rx_monotonic': now},
                'wheeltec': {'telemetry': {'velocity': [0.0, 0, 0]}}}
        self.assertIsNone(self.al.network_switch_blocker(idle, now))
        self.assertIsNone(self.al.network_switch_blocker({'motion': {'mode': 'ESTOP'}}, now))
        self.assertIn('行驶', self.al.network_switch_blocker(dict(idle, motion={'mode': 'NAVIGATION'}), now))
        self.assertIn('作业', self.al.network_switch_blocker(dict(idle, line={'phase': 'PAUSED', '_rx_monotonic': now}), now))
        self.assertIn('动', self.al.network_switch_blocker(dict(idle, wheeltec={'telemetry': {'velocity': [0.3, 0, 0]}}), now))
        self.assertIn('直行', self.al.network_switch_blocker(dict(idle, straight={'state': 'running'}), now))

    def test_hello_and_telemetry_fields(self):
        from runtime_config import PROFILE
        hello = self.al.build_hello(PROFILE, name='车', network={'hotspot_ssid': 'RTKCAR-A2-10', 'hotspot_psk': '12345678',
                                                                'hotspot_ip': '192.168.42.1'})
        self.assertEqual({'hotspot_ssid': 'RTKCAR-A2-10', 'hotspot_psk': '12345678', 'hotspot_ip': '192.168.42.1'}, hello['network'])
        self.assertIsNone(self.al.build_hello(PROFILE, name='车')['network'])
        tel = self.al.build_telemetry({'network': {'mode': 'hotspot', 'ssid': 'RTKCAR-A2-10', 'hotspot_ssid': 'RTKCAR-A2-10',
                                                   'hotspot_ip': '192.168.42.1', 'busy': False, 'error': None,
                                                   'revert_to': 'client', 'revert_in_s': 90}}, now=1.0, seq=1)
        self.assertEqual('hotspot', tel['network']['mode'])
        self.assertEqual(('client', 90), (tel['network']['revert_to'], tel['network']['revert_in_s']))
        self.assertNotIn('hotspot_psk', tel['network'])
        self.assertNotIn('network', self.al.build_telemetry({}, now=1.0, seq=1))


if __name__ == '__main__':
    unittest.main()
