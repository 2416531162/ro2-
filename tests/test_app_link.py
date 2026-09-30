#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手机 app 连接层 radar_system/app_link.py:WebSocket 帧、指令校验、摇杆换算、遥测编码。"""
import io
import json
import math
import os
import struct
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, ROOT)

import app_link as al  # noqa: E402
sys.path.insert(0, os.path.join(ROOT, 'radar_system', 'tools'))
import export_app_fixtures as fixtures  # noqa: E402
from runtime_config import PROFILE  # noqa: E402
from robot_core.kinematics import ChassisGeometry, steer_from_yaw  # noqa: E402


def ws_with_input(*frames):
    rfile, wfile = io.BytesIO(b''.join(frames)), io.BytesIO()
    return al.WebSocket(rfile, wfile), wfile


def server_frames(data):
    """解析服务端写出的(不带掩码)帧 -> [(opcode, payload)]。"""
    out, i = [], 0
    while i < len(data):
        b0, b1 = data[i], data[i + 1]
        n, i = b1 & 0x7F, i + 2
        if n == 126:
            n, i = struct.unpack('!H', data[i:i + 2])[0], i + 2
        elif n == 127:
            n, i = struct.unpack('!Q', data[i:i + 8])[0], i + 8
        out.append((b0 & 0x0F, data[i:i + n]))
        i += n
    return out


class TestWebSocket(unittest.TestCase):

    def test_accept_key_matches_rfc6455_example(self):
        self.assertEqual(al.accept_key('dGhlIHNhbXBsZSBub25jZQ=='), 's3pPLMBiTxaQ9kYGzzhZRbK+xOo=')

    def test_masked_text_and_long_payload(self):
        long = ('x' * 70000).encode()
        ws, _ = ws_with_input(al.client_frame(al.OP_TEXT, '你好'.encode()),
                              al.client_frame(al.OP_TEXT, long))
        ws.max_message = 100000
        self.assertEqual(ws.recv(), '你好')
        self.assertEqual(len(ws.recv()), 70000)

    def test_ping_is_answered_and_fragments_are_joined(self):
        ws, out = ws_with_input(al.client_frame(al.OP_PING, b'p1'),
                                al.client_frame(al.OP_TEXT, b'ab', fin=False),
                                al.client_frame(al.OP_CONT, b'cd'))
        self.assertEqual(ws.recv(), 'abcd')
        self.assertEqual(server_frames(out.getvalue()), [(al.OP_PONG, b'p1')])

    def test_close_and_eof_raise(self):
        ws, out = ws_with_input(al.client_frame(al.OP_CLOSE, b'\x03\xe8'))
        with self.assertRaises(al.WebSocketClosed):
            ws.recv()
        self.assertEqual(server_frames(out.getvalue())[0][0], al.OP_CLOSE)
        ws, _ = ws_with_input(b'\x81')
        with self.assertRaises(al.WebSocketClosed):
            ws.recv()

    def test_oversize_message_is_refused(self):
        ws, _ = ws_with_input(al.client_frame(al.OP_TEXT, b'y' * 2000))
        ws.max_message = 1000
        with self.assertRaises(al.WebSocketClosed):
            ws.recv()

    def test_send_json_frames(self):
        ws, out = ws_with_input()
        ws.send_json({'type': 'pong', 'n': 1})
        ws.send_text('z' * 300)
        frames = server_frames(out.getvalue())
        self.assertEqual(json.loads(frames[0][1]), {'type': 'pong', 'n': 1})
        self.assertEqual(len(frames[1][1]), 300)
        ws.close()
        with self.assertRaises(al.WebSocketClosed):
            ws.send_text('late')


class TestCommands(unittest.TestCase):

    def test_drive_is_clamped_and_defaults(self):
        c = al.parse_command('{"type":"drive","throttle":3,"steer":-0.4,"seq":7}')
        self.assertEqual((c['throttle'], c['steer'], c['tier'], c['seq']), (1.0, -0.4, 'med', 7))
        c = al.parse_command('{"type":"drive","throttle":0.2,"tier":"warp"}')
        self.assertEqual(c['tier'], 'med')

    def test_invalid_commands_are_rejected(self):
        for text in ('nope', '[]', '{"type":"fly"}', '{"type":"drive","throttle":"1"}',
                     '{"type":"drive","throttle":true}', '{"type":"follow","enable":1}',
                     '{"type":"select_target"}'):
            with self.subTest(text=text), self.assertRaises(al.CommandError):
                al.parse_command(text)
        with self.assertRaises(al.CommandError):
            al.parse_command(json.dumps({'type': 'drive', 'throttle': float('nan')}))

    def test_simple_commands(self):
        self.assertEqual(al.parse_command('{"type":"estop","id":"a"}'), {'type': 'estop', 'id': 'a'})
        self.assertTrue(al.parse_command('{"type":"follow","enable":true}')['enable'])
        self.assertAlmostEqual(al.parse_command('{"type":"select_target","bearing_rad":9}')['bearing_rad'],
                               math.pi)


class TestStraight(unittest.TestCase):
    """IMU 定向直行:指令校验、hello 里的参数、遥测透传。"""

    def test_straight_commands(self):
        self.assertEqual(al.parse_command('{"type":"straight","action":"go","run":3,"seq":17}'),
                         {'type': 'straight', 'action': 'go', 'run': 3, 'seq': 17})
        self.assertEqual(al.parse_command('{"type":"straight","action":"stop","run":3,"id":"s1"}'),
                         {'type': 'straight', 'id': 's1', 'action': 'stop', 'run': 3, 'seq': None})
        for bad in ('{"type":"straight","action":"go"}', '{"type":"straight","action":"go","run":true}',
                    '{"type":"straight","action":"go","run":-1}', '{"type":"straight","action":"go","run":1.5}',
                    '{"type":"straight","action":"left","run":1}'):
            with self.subTest(bad), self.assertRaises(al.CommandError):
                al.parse_command(bad)

    def test_hello_advertises_fixed_speed_and_tolerance(self):
        hello = al.build_hello(PROFILE, name='x', camera={})
        self.assertEqual(hello['straight'], {'speed_mps': 0.8, 'tolerance_deg': 1.0, 'abort_deg': 5.0})

    def test_straight_telemetry_splits_session_and_run(self):
        st = {'imu_ok': True, 'yaw_deg': 30.4, 'run': ('app-1', 3), 'state': 'stopped', 'reason': 'user',
              'target_deg': 30.0, 'error_deg': 0.4, 'max_error_deg': 0.83, 'within_pct': 98.5,
              'elapsed_s': 12.3, 'speed': 0.0, 'wz': float('nan')}
        self.assertEqual(al.straight_telemetry(st), {
            'imu_ok': True, 'state': 'stopped', 'session': 'app-1', 'run': 3, 'reason': 'user',
            'target_deg': 30.0, 'yaw_deg': 30.4, 'error_deg': 0.4, 'max_error_deg': 0.83, 'within_pct': 98.5,
            'elapsed_s': 12.3, 'speed': 0.0, 'wz': None})
        self.assertEqual(al.straight_telemetry({'imu_ok': False, 'yaw_deg': None, 'state': 'idle'}),
                         {'imu_ok': False, 'state': 'idle', 'yaw_deg': None})
        self.assertIsNone(al.straight_telemetry(None))
        tel = al.build_telemetry(dict(fixtures.sample_snapshot(), straight=st), now=1.0, seq=1,
                                 sensors=PROFILE['sensors'])
        self.assertEqual(tel['straight']['run'], 3)
        self.assertNotIn('straight', al.build_telemetry(fixtures.sample_snapshot(), now=1.0, seq=1,
                                                        sensors=PROFILE['sensors']))
        json.dumps(tel, allow_nan=False)


class TestStick(unittest.TestCase):
    tiers = {'low': 0.5, 'med': 0.85, 'high': 1.2}

    def resolve(self, throttle, steer, tier='med'):
        return al.resolve_stick(throttle, steer, tier, self.tiers, 0.6, ChassisGeometry())

    def test_deadband_stops(self):
        self.assertEqual(self.resolve(0.015, 1.0), (0.0, 0.0, 0.0))

    def test_app_creep_command_moves_slowly(self):
        # app 1.9 不分档、固定发 high，刚出死区约 0.029：车以约 3.5 cm/s 挪（旧车最快档 1.2 m/s）
        vx, _, _ = self.resolve(0.029, 0.0, 'high')
        self.assertAlmostEqual(vx, 0.029 * 1.2, places=4)

    def test_reverse_is_scaled_and_steer_is_limited(self):
        geo = ChassisGeometry()
        vx, wz, deg = self.resolve(-1.0, 1.0, 'high')
        self.assertAlmostEqual(vx, -0.72)
        self.assertAlmostEqual(deg, round(math.degrees(geo.max_steer_rad), 1))
        # 转弯半径只由转角决定:从 (vx, wz) 反算回的转角等于摇杆给的转角
        self.assertAlmostEqual(steer_from_yaw(vx, wz, geo), geo.max_steer_rad, places=3)

    def test_half_steer_keeps_radius_across_tiers(self):
        geo = ChassisGeometry()
        a = self.resolve(1.0, 0.5, 'low')
        b = self.resolve(1.0, 0.5, 'high')
        self.assertAlmostEqual(a[0] / a[1], b[0] / b[1], places=2)
        self.assertAlmostEqual(steer_from_yaw(a[0], a[1], geo), 0.5 * geo.max_steer_rad, places=3)


class TestTelemetry(unittest.TestCase):

    def test_scan_roundtrip_centimetres(self):
        r = [0.0, 1.234, float('inf'), float('nan'), 12.0, -1.0]
        back = al.decode_scan(al.encode_scan(r))
        self.assertEqual(back.tolist(), [0.0, 1.23, 0.0, 0.0, 12.0, 0.0])

    def test_locked_person_is_matched_by_camera_bearing(self):
        s = PROFILE['sensors']
        target = {'in_camera_view': True, 'veh_x': s['camera_x_m'] + 2.0, 'veh_y': 0.5}
        want = al.camera_bearing(target['veh_x'], target['veh_y'], s)
        persons = [{'bearing': want + 0.3}, {'bearing': want + 0.02}, {'bearing': None}]
        al.mark_locked(persons, target, s)
        self.assertEqual([p['locked'] for p in persons], [False, True, False])
        al.mark_locked(persons, dict(target, in_camera_view=False), s)
        self.assertFalse(any(p['locked'] for p in persons))

    def snapshot(self, stale=False):
        return fixtures.sample_snapshot(stale)

    def test_telemetry_is_compact_valid_json(self):
        tel = al.build_telemetry(self.snapshot(), now=12.3456, seq=5,
                                 sensors=PROFILE['sensors'])
        text = json.dumps(tel, allow_nan=False)
        self.assertLess(len(text), 4000)
        self.assertEqual(len(tel['persons']), 1)
        self.assertTrue(tel['persons'][0]['locked'])
        self.assertEqual(tel['follow']['target']['id'], 3)
        self.assertEqual(tel['drive']['speed'], 0.42)
        self.assertEqual(len(al.decode_scan(tel['scan']['r'])), 720)

    def test_stale_follower_hides_target_and_tracks(self):
        tel = al.build_telemetry(self.snapshot(stale=True), now=1.0, seq=1,
                                 sensors=PROFILE['sensors'])
        self.assertIsNone(tel['follow']['target'])
        self.assertEqual(tel['follow']['tracks'], [])
        self.assertFalse(tel['persons'][0]['locked'])

    def test_video_hub_only_encodes_for_viewers(self):
        hub = al.VideoHub(max_fps=10.0)
        self.assertFalse(hub.wants_frame(1.0), '没人看时不解码')
        hub.attach()
        self.assertTrue(hub.wants_frame(1.0))
        hub.publish(b'jpg1', 5.0, 1.0)
        self.assertFalse(hub.wants_frame(1.05), '限帧')
        self.assertTrue(hub.wants_frame(1.11))
        self.assertEqual(hub.wait_next(0, 0.1), (1, b'jpg1', 5.0))
        self.assertIsNone(hub.wait_next(1, 0.05))
        hub.detach()
        self.assertEqual(hub.clients, 0)
        part = al.mjpeg_part(b'\xff\xd8x\xff\xd9', 1.5)
        self.assertIn(b'Content-Length: 5', part)
        self.assertIn(b'X-Stamp: 1.500', part)

    def test_hello_and_beacon(self):
        hello = al.build_hello(PROFILE, name='car', camera={'width': 640, 'height': 480,
                                                            'fx': 570.3, 'cx': 319.5})
        self.assertEqual(hello['v'], al.PROTOCOL_VERSION)
        self.assertAlmostEqual(hello['car']['min_turn_radius_m'], ChassisGeometry().min_turn_radius_m, places=3)
        b = json.loads(al.beacon('car', 8088, 'abc'))
        self.assertEqual((b['magic'], b['port']), (al.DISCOVERY_MAGIC, 8088))


class TestRtkLine(unittest.TestCase):

    def test_line_commands(self):
        self.assertEqual(al.parse_command('{"type":"line","action":"mark","point":"b","id":1}'),
                         {'type': 'line', 'action': 'mark', 'point': 'B', 'id': '1'})
        self.assertEqual(al.parse_command('{"type":"line","action":"start","cruise":3}')['cruise'], 1.0)
        self.assertNotIn('cruise', al.parse_command('{"type":"line","action":"start"}'))
        self.assertEqual(al.parse_command('{"type":"line","action":"stop"}'), {'type': 'line', 'action': 'stop'})
        for bad in ('{"type":"line","action":"fly"}', '{"type":"line","action":"mark","point":"C"}',
                    '{"type":"line","action":"start","cruise":"fast"}'):
            with self.assertRaises(al.CommandError):
                al.parse_command(bad)

    def test_route_commands(self):
        route = {'kind': 'points', 'points': [[0, 0], [10.00004, 0], [10, 8]], 'laps': 2}
        cmd = al.parse_command(json.dumps({'type': 'route', 'action': 'start', 'id': 'r1', 'route': route}))
        self.assertEqual((cmd['action'], cmd['id'], cmd['route']['points'][1]), ('start', 'r1', [10.0, 0.0]))
        self.assertEqual(cmd['route']['turn_radius'], 2.5)
        self.assertNotIn('route', al.parse_command('{"type":"route","action":"start"}'))
        self.assertEqual(al.parse_command('{"type":"route","action":"clear"}'), {'type': 'route', 'action': 'clear'})
        for bad in ('{"type":"route","action":"fly"}', '{"type":"route","action":"set"}',
                    '{"type":"route","action":"set","route":{"kind":"star"}}',
                    '{"type":"route","action":"start","route":{"kind":"points","laps":99}}'):
            with self.subTest(bad), self.assertRaises(al.CommandError):
                al.parse_command(bad)

    def test_route_section_passes_through(self):
        fresh = al.line_telemetry(fixtures.sample_route_line(now=100.0), 100.0)
        self.assertEqual((fresh['task'], fresh['route']['laps'], fresh['route']['problem']), ('route', 2, None))
        self.assertEqual(fresh['origin'], [30.123456789, 120.987654321])
        self.assertEqual(fresh['antenna']['sol'], 'NARROW_INT')
        # 旧版 line_node 没有这些字段：app 按"没有路线"处理
        old = al.line_telemetry(fixtures.sample_line(now=100.0), 100.0)
        self.assertEqual((old['task'], old['route'], old['origin']), ('line', None, None))

    def test_rtk_section_goes_stale(self):
        g = fixtures.sample_gnss(now=100.0)
        fresh = al.rtk_telemetry(g, 100.0)
        self.assertEqual((fresh['stale'], fresh['fix'], fresh['heading_ok']), (False, 'FIXED', True))
        # 样本的接收机在平板串口上：车端 CORS 让给平板，没连
        self.assertEqual((fresh['ntrip']['connected'], fresh['ntrip']['paused']), (False, 'tablet'))
        self.assertTrue(fresh['ntrip']['enabled'])
        g['ntrip'].update(enabled=False, connected=False, configured=True, paused=None)
        stopped = al.rtk_telemetry(g, 100.0)['ntrip']
        self.assertTrue(stopped['configured'])
        self.assertFalse(stopped['enabled'])
        self.assertFalse(stopped['connected'])
        self.assertEqual(al.rtk_telemetry(g, 103.0), {'stale': True})
        self.assertEqual(al.rtk_telemetry(None, 100.0), {'stale': True})

    def test_rtk_source_tells_app_where_the_receiver_is(self):
        g = fixtures.sample_gnss(now=100.0)
        rtk = al.rtk_telemetry(g, 100.0)
        self.assertEqual((rtk['source'], rtk['tablet_connected']), ('tablet', True))
        g.update(source='usb', tablet=dict(g['tablet'], connected=False))
        rtk = al.rtk_telemetry(g, 100.0)
        self.assertEqual((rtk['source'], rtk['tablet_connected']), ('usb', False))
        old = {k: v for k, v in g.items() if k not in ('source', 'usb', 'tablet')}   # 旧 RTK 节点不报来源
        self.assertEqual((al.rtk_telemetry(old, 100.0)['source'], al.rtk_telemetry(old, 100.0)['tablet_connected']),
                         (None, False))
        g['source'] = 'nonsense'
        self.assertIsNone(al.rtk_telemetry(g, 100.0)['source'])
        # 车端 CORS 让给平板：app 据此显示「车端 CORS 已暂停（数据来自平板）」
        g['ntrip'] = dict(g.get('ntrip') or {}, paused='tablet')
        self.assertEqual(al.rtk_telemetry(g, 100.0)['ntrip']['paused'], 'tablet')
        g['ntrip']['paused'] = 'other'
        self.assertIsNone(al.rtk_telemetry(g, 100.0)['ntrip']['paused'])
        del g['ntrip']['paused']
        self.assertIsNone(al.rtk_telemetry(g, 100.0)['ntrip']['paused'])

    def test_rtk_heading_yaw_only_when_valid(self):
        self.assertEqual(al.rtk_telemetry(fixtures.sample_gnss(now=100.0), 100.0)['heading']['yaw_deg'], 90.18)
        self.assertIsNone(al.rtk_telemetry(fixtures.sample_bad_heading_gnss(now=100.0), 100.0)['heading']['yaw_deg'])
        g = fixtures.sample_gnss(now=100.0)
        g['heading'] = dict(g['heading'], valid=False)          # 测向无效时车端就算带了角度也不转给 app
        self.assertIsNone(al.rtk_telemetry(g, 100.0)['heading']['yaw_deg'])
        g['heading'] = {k: v for k, v in fixtures.sample_gnss(now=100.0)['heading'].items() if k != 'yaw_deg'}
        self.assertIsNone(al.rtk_telemetry(g, 100.0)['heading']['yaw_deg'])   # 旧车端没这项

    def test_line_attitude_passes_through_and_rejects_bad_numbers(self):
        ln = fixtures.sample_route_line(now=100.0)
        self.assertEqual(al.line_telemetry(ln, 100.0)['attitude'],
                         {'roll_deg': 1.25, 'pitch_deg': -2.4, 'yaw_deg': 12.6, 'source': 'imu_madgwick'})
        old = dict(ln, attitude={'roll_deg': 1.25, 'pitch_deg': -2.4, 'source': 'imu_madgwick'})   # 只有横滚俯仰的车端
        self.assertIsNone(al.line_telemetry(old, 100.0)['attitude']['yaw_deg'])
        self.assertIsNone(al.line_telemetry(dict(ln, attitude=None), 100.0)['attitude'])
        self.assertIsNone(al.line_telemetry(dict(ln, attitude={'roll_deg': float('nan'), 'pitch_deg': 1.0}), 100.0)['attitude'])
        self.assertIsNone(al.line_telemetry(fixtures.sample_line(now=100.0), 100.0)['attitude'])   # 旧车端没这项

    def test_line_section_goes_stale(self):
        ln = fixtures.sample_line(now=100.0)
        fresh = al.line_telemetry(ln, 100.0)
        self.assertEqual((fresh['phase'], fresh['track']['e'], fresh['points']['B']), ('RUNNING', 0.006, [20.0, 0.0]))
        self.assertEqual(al.line_telemetry(ln, 102.0), {'stale': True})
        json.dumps(fresh, allow_nan=False)


class TestAndroidFixtures(unittest.TestCase):

    def test_fixtures_match_current_encoder(self):
        """安卓单元测试解析的样例必须是当前车端编码器的输出。

        改了协议字段后重新生成,并同步到安卓工程(见 export_app_fixtures.py):
            python3 radar_system/tools/export_app_fixtures.py --out <安卓工程>/app/src/test/resources
        """
        for name, msg in fixtures.fixture_messages().items():
            with self.subTest(name), open(os.path.join(fixtures.FIXTURE_DIR, name), encoding='utf-8') as f:
                self.assertEqual(json.load(f), json.loads(json.dumps(msg)))


if __name__ == '__main__':
    unittest.main()
