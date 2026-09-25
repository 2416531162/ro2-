#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手动遥控 HTTP 链路回归测试(真实 HTTP 服务 + ROS 替身,不需要底盘)。

对应的现场问题:手机上点方向键没反应,过一会车才动。
原因是 HTTP/1.1 长连接下 JSON 响应没有 Content-Length,浏览器的 fetch 一直挂着,
同一主机的连接很快被占满,后面的指令(包括刹车)在浏览器里排队。

    python3 tests/test_web_control.py
"""

import http.client
import json
import os
import socket
import sys
import threading
import time
import types
import unittest
from unittest.mock import patch

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tests"))
sys.path.insert(0, os.path.join(ROOT, "radar_system"))

import ros_stubs  # noqa: E402

ros_stubs.install()
sys.modules['std_msgs.msg'].Float32 = ros_stubs._simple('Float32')
if 'std_srvs.srv' not in sys.modules:
    _srv = types.ModuleType('std_srvs.srv')
    _srv.SetBool = _srv.Trigger = type('Srv', (), {'Request': object})
    sys.modules['std_srvs'] = types.ModuleType('std_srvs')
    sys.modules['std_srvs.srv'] = _srv
if 'rclpy.executors' not in sys.modules:
    _ex = types.ModuleType('rclpy.executors')
    _ex.MultiThreadedExecutor = object
    sys.modules['rclpy.executors'] = _ex
if 'rclpy.callback_groups' not in sys.modules:
    _cg = types.ModuleType('rclpy.callback_groups')
    _cg.MutuallyExclusiveCallbackGroup = object
    sys.modules['rclpy.callback_groups'] = _cg
ros_stubs.Node.create_timer = lambda self, period, cb, **k: types.SimpleNamespace(period=period)
ros_stubs.Node.create_client = lambda self, *a, **k: types.SimpleNamespace(
    service_is_ready=lambda: False)

import radar_web_server as web  # noqa: E402
from runtime_config import PROFILE, profile_hash  # noqa: E402


class FakeMotionService:
    def __init__(self, success=True, message='IDLE; select a new task'):
        self.success = success
        self.message = message
        self.calls = 0

    def service_is_ready(self):
        return True

    def call_async(self, _request):
        self.calls += 1
        result = types.SimpleNamespace(success=self.success, message=self.message)
        future = types.SimpleNamespace()
        future.add_done_callback = lambda callback: callback(future)
        future.result = lambda: result
        return future


class TestWebControl(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        web.check_follower_running_cached = lambda: False
        cls.bridge = web.TrackingBridgeNode()
        web.bridge_node = cls.bridge
        cls.server = web.ThreadedHTTPServer(('127.0.0.1', 0), web.RadarHTTPHandler)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        web.bridge_node = None

    def setUp(self):
        self.conn = http.client.HTTPConnection('127.0.0.1', self.server.server_address[1],
                                               timeout=1.0)
        with self.bridge.motion.lock:
            self.bridge.motion.state = {}
            self.bridge.motion.received = None
        self.bridge.cli_reset = types.SimpleNamespace(service_is_ready=lambda: False)

    def tearDown(self):
        self.conn.close()

    def drive(self, direction, seq=None, client='page-a'):
        body = {'direction': direction, 'speed_tier': 'low', 'steer_tier': 'normal'}
        if seq is not None:
            body.update(client_id=client, seq=seq)
        self.conn.request('POST', '/api/manual_drive', json.dumps(body),
                          {'Content-Type': 'application/json'})
        resp = self.conn.getresponse()
        self.assertIsNotNone(resp.getheader('Content-Length'))
        return resp.status, json.loads(resp.read())

    def test_keepalive_responses_complete_without_waiting(self):
        """同一条长连接上连续发 20 条指令,每条都必须立刻读完。"""
        # 手动接口要求新鲜的仲裁状态；HTTP 长连接测试也应提供这项真实前提。
        with self.bridge.motion.lock:
            self.bridge.motion.state = dict(mode='IDLE', epoch='test-epoch', healthy=True,
                                             profile_hash=profile_hash(PROFILE), legacy_commands=False)
            self.bridge.motion.received = web.time.monotonic()
        try:
            for i in range(20):
                status, data = self.drive('forward', seq=i + 1, client='burst')
                self.assertEqual(status, 200)
                self.assertTrue(data['ok'])
        except socket.timeout:
            self.fail("响应体读不完:缺 Content-Length,浏览器 fetch 会一直挂起")

    def test_preflight_has_empty_body(self):
        self.conn.request('OPTIONS', '/api/manual_drive')
        resp = self.conn.getresponse()
        self.assertEqual(resp.getheader('Content-Length'), '0')
        self.assertEqual(resp.read(), b'')
        self.conn.request('POST', '/api/drive_profile', '')
        resp = self.conn.getresponse()
        self.assertEqual(resp.status, 200)
        json.loads(resp.read())

    def test_app_beacon_identifies_the_car(self):
        # app 连车载热点时用它确认网关就是车：内容与 UDP 发现广播相同
        self.conn.request('GET', '/app/beacon')
        resp = self.conn.getresponse()
        self.assertEqual((resp.status, resp.getheader('Content-Type')), (200, 'application/json'))
        beacon = json.loads(resp.read())
        self.assertEqual((beacon['magic'], beacon['name'], beacon['port']), ('rk3588-follow', web.CAR_NAME, web.PORT))

    def test_late_forward_does_not_restart_after_stop(self):
        self.drive('forward', seq=10, client='order')
        self.drive('stop', seq=11, client='order')
        vx, _wz, active, _zero = self.bridge.manual_drive.sample()
        self.assertFalse(active)
        status, data = self.drive('forward', seq=10, client='order')   # 迟到的旧请求
        self.assertTrue(data.get('stale'))
        vx, _wz, active, _zero = self.bridge.manual_drive.sample()
        self.assertFalse(active)
        self.assertEqual(vx, 0.0)

    def set_motion_fault(self, *, healthy=True, waiting_stationary=False):
        with self.bridge.motion.lock:
            self.bridge.motion.state = {
                'mode': 'FAULT',
                'epoch': 'test-epoch',
                'profile_hash': profile_hash(PROFILE),
                'legacy_commands': False,
                'healthy': healthy,
                'waiting_stationary': waiting_stationary,
            }
            self.bridge.motion.received = web.time.monotonic()

    def test_manual_command_resets_recovered_latched_fault(self):
        self.set_motion_fault()
        reset = FakeMotionService()
        self.bridge.cli_reset = reset

        status, data = self.drive('forward', seq=101, client='recover')

        self.assertEqual(status, 200)
        self.assertTrue(data['ok'])
        self.assertEqual(reset.calls, 1)

    def test_manual_command_does_not_reset_active_fault(self):
        self.set_motion_fault(healthy=False)
        reset = FakeMotionService()
        self.bridge.cli_reset = reset

        status, data = self.drive('forward', seq=102, client='active-fault')

        self.assertEqual(status, 409)
        self.assertFalse(data['ok'])
        self.assertIn('仍有故障', data['error'])
        self.assertEqual(reset.calls, 0)

    def test_reset_failed_remains_locked_without_stall_status(self):
        self.set_motion_fault()
        with self.bridge.motion.lock:
            self.bridge.motion.state['reason'] = 'reset_failed'
        reset = FakeMotionService()
        self.bridge.cli_reset = reset

        status, data = self.drive('forward', seq=104, client='reset-failed')

        self.assertEqual(status, 409)
        self.assertIn('底盘复位失败', data['error'])
        self.assertNotIn('运动卡滞', data['error'])
        self.assertEqual(reset.calls, 0)

    def test_stop_never_clears_latched_fault(self):
        self.set_motion_fault()
        reset = FakeMotionService()
        self.bridge.cli_reset = reset

        status, data = self.drive('stop', seq=103, client='stop-fault')

        self.assertEqual(status, 200)
        self.assertTrue(data['ok'])
        self.assertEqual(reset.calls, 0)

    def test_tracking_bridge_has_no_retired_subscriptions(self):
        topics = {topic for topic, _ in self.bridge.subscriptions_}
        # 相机原图/内参给安卓 app 的 /video.mjpg;没人看时回调直接返回,不解码
        # /gnss/status、/line/status 是 RTK 节点和走直线节点的状态，转给 app（不是旧的 /api/cors 路由）
        self.assertEqual(topics, {'/scan', '/camera/ai_detection/targets',
                                 '/camera/ai_detection/status', '/voltage',
                                 '/follower/status', '/wheeltec/status', '/motion/status',
                                 '/camera/rgb/image_raw', '/camera/rgb/camera_info',
                                 '/gnss/status', '/line/status'})

    def test_retired_map_and_rtk_routes_are_gone(self):
        for path in ('/map', '/map2d', '/api/cors', '/api/live_map/state'):
            self.conn.request('GET', path)
            resp = self.conn.getresponse()
            self.assertEqual(resp.status, 404, path)
            resp.read()

    def test_index_page_has_length(self):
        self.conn.request('GET', '/')
        resp = self.conn.getresponse()
        self.assertEqual(int(resp.getheader('Content-Length')), len(resp.read()))

    def test_index_retains_other_fault_banner_without_stall_status(self):
        self.conn.request('GET', '/')
        body = self.conn.getresponse().read().decode('utf-8')
        self.assertIn('id="motionLock"', body)
        self.assertIn('急停已锁存', body)
        self.assertNotIn('运动卡滞已锁存', body)
        self.assertNotIn('motion_stall_latched', body)


class TestAiStaleness(unittest.TestCase):
    """人体识别停了(默认关闭或挂了)以后,不能再给出它最后那一刻的人框和帧率。"""

    def snapshot_with(self, age_s):
        status = {} if age_s is None else {'fps': 19.0, '_rx_monotonic': time.monotonic() - age_s}
        box = [{'label': 'person', 'x1': 1, 'y1': 2, 'x2': 3, 'y2': 4}]
        with patch.dict(web.state, {'ai_status': status, 'ai_targets': box}):
            return web.snapshot_state()

    def test_running_detection_passes_through(self):
        snap = self.snapshot_with(0.5)
        self.assertEqual(snap['ai_status']['fps'], 19.0)
        self.assertEqual(len(snap['ai_targets']), 1)

    def test_stopped_detection_is_blanked(self):
        for age in (None, web.AI_STALE_S + 0.5):
            with self.subTest(age=age):
                snap = self.snapshot_with(age)
                self.assertEqual(snap['ai_status'], {})
                self.assertEqual(snap['ai_targets'], [])

    def test_status_callback_stamps_receipt_time(self):
        with patch.dict(web.state, {'ai_status': {}}):
            web.TrackingBridgeNode.ai_status_cb(None, types.SimpleNamespace(data='{"fps": 18.7}'))
            self.assertTrue(web.ai_online(web.state['ai_status']))
            self.assertFalse(web.ai_online(web.state['ai_status'], now=time.monotonic() + web.AI_STALE_S + 1))
            web.TrackingBridgeNode.ai_status_cb(None, types.SimpleNamespace(data='[1, 2]'))  # 不是对象:忽略
            self.assertEqual(web.state['ai_status']['fps'], 18.7)


if __name__ == '__main__':
    unittest.main()


class TestAppSocket(unittest.TestCase):
    """安卓 app 的 /ws 长连接:握手、遥测、摇杆、急停、断线即停。"""

    @classmethod
    def setUpClass(cls):
        TestWebControl.setUpClass.__func__(cls)

    @classmethod
    def tearDownClass(cls):
        TestWebControl.tearDownClass.__func__(cls)

    def setUp(self):
        import app_link
        self.al = app_link
        with self.bridge.motion.lock:
            self.bridge.motion.state = {'mode': 'IDLE', 'epoch': 'app-epoch',
                                        'profile_hash': profile_hash(PROFILE),
                                        'legacy_commands': False, 'healthy': True}
            self.bridge.motion.received = web.time.monotonic()
        self.bridge.manual_drive.set(0.0, 0.0)
        self.bridge.manual_drive.sample()
        self.sock = socket.create_connection(('127.0.0.1', self.server.server_address[1]), timeout=2)
        key = 'dGhlIHNhbXBsZSBub25jZQ=='
        self.sock.sendall(('GET /ws HTTP/1.1\r\nHost: car\r\nUpgrade: websocket\r\n'
                           'Connection: Upgrade\r\nSec-WebSocket-Key: ' + key +
                           '\r\nSec-WebSocket-Version: 13\r\n\r\n').encode())
        self.reader = self.sock.makefile('rb')
        head = b''
        while not head.endswith(b'\r\n\r\n'):
            head += self.reader.read(1)
        self.assertIn(b' 101 ', head.split(b'\r\n')[0])
        self.assertIn(self.al.accept_key(key).encode(), head)

    def tearDown(self):
        try:
            self.reader.close()
            self.sock.close()
        except OSError:
            pass

    def read_json(self, want=None, limit=40):
        for _ in range(limit):
            b0, b1 = self.reader.read(2)
            n = b1 & 0x7F
            if n == 126:
                n = int.from_bytes(self.reader.read(2), 'big')
            elif n == 127:
                n = int.from_bytes(self.reader.read(8), 'big')
            payload = self.reader.read(n)
            if b0 & 0x0F != self.al.OP_TEXT:
                continue
            msg = json.loads(payload)
            if want is None or msg.get('type') == want:
                return msg
        self.fail('没有收到 ' + str(want))

    def send(self, obj):
        self.sock.sendall(self.al.client_frame(self.al.OP_TEXT, json.dumps(obj).encode()))

    def wait_for(self, predicate, timeout=1.5):
        deadline = web.time.monotonic() + timeout
        while web.time.monotonic() < deadline:
            if predicate():
                return True
            web.time.sleep(0.02)
        return False

    def test_hello_then_telemetry(self):
        hello = self.read_json('hello')
        self.assertEqual(hello['v'], self.al.PROTOCOL_VERSION)
        self.assertGreater(hello['car']['min_turn_radius_m'], 1.0)
        tel = self.read_json('tel')
        self.assertEqual(tel['motion']['mode'], 'IDLE')
        self.assertIn('follow', tel)

    def test_stick_drives_and_disconnect_stops_immediately(self):
        self.read_json('hello')
        self.send({'type': 'drive', 'throttle': 0.5, 'steer': 0.0, 'tier': 'low', 'seq': 1})
        self.assertTrue(self.wait_for(lambda: self.bridge.manual_drive._vx > 0.2))
        self.assertAlmostEqual(self.bridge.manual_drive._vx, 0.5 * PROFILE['manual']['low_mps'])
        self.reader.close()
        self.sock.shutdown(socket.SHUT_RDWR)
        self.sock.close()
        self.assertTrue(self.wait_for(lambda: self.bridge.manual_drive._vx == 0.0),
                        '断线后车端必须立即清零,不等心跳超时')

    def test_ping_invalid_and_select(self):
        self.read_json('hello')
        self.send({'type': 'ping', 't': 42})
        self.assertEqual(self.read_json('pong')['t'], 42)
        self.send({'type': 'drive', 'throttle': 'full'})
        self.assertFalse(self.read_json('ack')['ok'])
        self.send({'type': 'select_target', 'bearing_rad': 0.2, 'id': 's1'})
        ack = self.read_json('ack')
        self.assertTrue(ack['ok'])
        sent = self.bridge.pub_select.sent[-1]
        self.assertAlmostEqual(json.loads(sent.data)['bearing_rad'], 0.2)

    def test_line_commands_forwarded_and_rtk_status_in_telemetry(self):
        self.read_json('hello')
        self.send({'type': 'line', 'action': 'mark', 'point': 'a', 'id': 'l1'})
        ack = self.read_json('ack')
        self.assertEqual((ack['ok'], ack['id'], ack['message']), (True, 'l1', 'forwarded'))
        self.assertEqual(json.loads(self.bridge.pub_line.sent[-1].data), {'cmd': 'mark', 'point': 'A', 'id': 'l1'})
        self.send({'type': 'line', 'action': 'start', 'cruise': 0.4})
        self.read_json('ack')
        self.assertEqual(json.loads(self.bridge.pub_line.sent[-1].data)['cruise'], 0.4)
        self.bridge.gnss_status_cb(ros_stubs.String(data=json.dumps({'rtk': 'FIXED', 'sats': 30, 'heading_ok': True})))
        self.bridge.line_status_cb(ros_stubs.String(data=json.dumps({'phase': 'IDLE', 'points': {'A': [0, 0]}})))
        tel = self.read_json('tel')
        self.assertEqual((tel['rtk']['stale'], tel['rtk']['fix']), (False, 'FIXED'))
        self.assertEqual((tel['line']['stale'], tel['line']['points']), (False, {'A': [0, 0]}))

    def test_estop_zeroes_and_latches(self):
        self.read_json('hello')
        stop = FakeMotionService('ESTOP latched')
        self.bridge.cli_stop = stop
        self.bridge.manual_drive.set(0.4, 0.0)
        self.send({'type': 'estop', 'id': 'e1'})
        ack = self.read_json('ack')
        self.assertTrue(ack['ok'])
        self.assertEqual(ack['id'], 'e1')
        self.assertEqual(stop.calls, 1)
        self.assertEqual(self.bridge.manual_drive._vx, 0.0)
