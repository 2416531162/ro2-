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
import types
import unittest

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
        self.assertEqual(topics, {'/scan', '/camera/ai_detection/targets',
                                 '/camera/ai_detection/status', '/voltage',
                                 '/follower/status', '/wheeltec/status', '/motion/status'})

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


if __name__ == '__main__':
    unittest.main()
