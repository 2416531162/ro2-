#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车载屏幕只读显示（radar_system/car_display.py）：网络解析、平板连接计数、计划路线、事件流内容、只读路由。"""
import gzip
import io
import json
import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, ROOT)

import car_display as cd  # noqa: E402
import route_plan as rp  # noqa: E402
from runtime_config import PROFILE  # noqa: E402

BODY = [0.67, 0.08, 0.335]
LIMITS = dict(min_radius=1.916, max_points=400, max_laps=10, max_entry_m=30.0, body=BODY)
RECT = dict(kind='path', points=[[0, 0], [8, 0], [8, 6], [0, 6]], closed=True, center=None, radius=3.0,
            clockwise=True, laps=1, turn_radius=2.5, cruise=0.5, rev=3, problem=None, length=28.0)


def snapshot(route=RECT, **line):
    """radar_web_server 快照里显示页面要用的部分：刚收到的路线状态。"""
    import time
    now = time.monotonic()
    st = dict(phase='IDLE', message='', cruise=0.5, pose=None, points={}, tracker={}, task='route', origin=[30.0, 120.0],
              antenna=None, route=route, route_limits=LIMITS, _rx_monotonic=now)
    st.update(line)
    return {'hz': 10.0, 'ranges': [1.0] * 720, 'ai_targets': [{'label': 'person', 'x1': 1}], 'line': st,
            'gnss': {'_rx_monotonic': now, 'receiver_ok': True, 'rtk': 'FIXED', 'sats': 30,
                     'ntrip': {'configured': True, 'connected': True, 'bytes': 2048}}}


class FakeNetwork:
    def value(self):
        return {'wifi': 'car-ap', 'addresses': [{'dev': 'wlan0', 'ip': '192.168.2.40'}], 'checked': 1.0}


class FakeHandler:
    def __init__(self, path):
        self.path, self.status, self.headers, self.wfile = path, None, {}, io.BytesIO()
        self.close_connection = False

    def send_response(self, status):
        self.status = status

    def send_header(self, k, v):
        self.headers[k] = v

    def end_headers(self):
        pass

    def send_error(self, status):
        self.status = status


class TestNetwork(unittest.TestCase):

    def test_parse_addresses_and_wifi(self):
        text = ('1: lo    inet 127.0.0.1/8 scope host lo\n'
                '3: wlan0    inet 192.168.2.40/24 brd 192.168.2.255 scope global dynamic wlan0\n'
                '4: docker0    inet 172.17.0.1/16 scope global docker0\n'
                '5: eth0    inet 10.118.61.7/24 scope global eth0\n')
        self.assertEqual(cd.parse_ipv4(text), [('wlan0', '192.168.2.40'), ('eth0', '10.118.61.7')])
        self.assertEqual(cd.parse_wifi('no:别家\nyes:车库 AP\n'), '车库 AP')
        self.assertEqual(cd.parse_wifi('yes:my\\:ap\n'), 'my:ap')
        self.assertIsNone(cd.parse_wifi('no:x\n'))

    def test_sessions_never_go_negative(self):
        s = cd.Sessions()
        s.enter(); s.enter(); s.leave()
        self.assertEqual(s.count(), 1)
        s.leave(); s.leave()
        self.assertEqual(s.count(), 0)


class TestPlan(unittest.TestCase):

    def test_path_plan_is_the_body_centre_line_through_every_corner(self):
        pts = cd.plan_points(RECT, LIMITS)
        self.assertGreater(len(pts), 100)
        self.assertLess(math.hypot(*pts[0]), 1e-3)                        # 车身中心从第 1 个点出发
        for v in ([8, 0], [8, 6], [0, 6]):                                # 车身中心压过每个拐角（按 10 cm 取点）
            self.assertLess(min(math.hypot(p[0] - v[0], p[1] - v[1]) for p in pts), 0.051)
        self.assertLess(math.hypot(pts[-1][0], pts[-1][1] + rp.VERTEX_PASS_M), 1e-3)   # 最后一段往南回到起点，多走 5 cm

    def test_other_kinds_and_bad_input(self):
        circle = dict(RECT, kind='circle', center=[1.0, 2.0], radius=3.0, points=[])
        pts = cd.plan_points(circle, LIMITS)
        self.assertTrue(all(abs(math.hypot(p[0] - 1, p[1] - 2) - 3) < 1e-3 for p in pts))
        self.assertEqual(cd.plan_points(dict(RECT, points=[[0, 0]]), LIMITS), [])
        self.assertEqual(cd.plan_points(None, LIMITS), [])
        self.assertEqual(cd.plan_points(dict(RECT, kind='star'), LIMITS), [])
        self.assertEqual(cd.plan_points(RECT, dict(LIMITS, body=None)), [])


class TestFeed(unittest.TestCase):

    def feed(self, sessions=None):
        return cd.Feed(name='小车', profile=PROFILE, sessions=sessions or cd.Sessions(), network=FakeNetwork())

    def test_message_is_telemetry_without_scan_plus_links(self):
        s = cd.Sessions(); s.enter()
        msg = self.feed(s).message(snapshot(), __import__('time').monotonic())
        json.dumps(msg, allow_nan=False)
        self.assertEqual((msg['type'], msg['apps'], msg['net']['wifi']), ('display', 1, 'car-ap'))
        for gone in ('scan', 'persons', 'follow'):
            self.assertNotIn(gone, msg['tel'])
        self.assertEqual(msg['tel']['rtk']['fix'], 'FIXED')
        self.assertEqual(msg['car']['front_m'], PROFILE['geometry']['front_m'])

    def test_plan_is_sent_once_per_route(self):
        import time
        f = self.feed()
        first = f.message(snapshot(), time.monotonic())
        self.assertGreater(len(first['plan']['points']), 100)
        self.assertNotIn('plan', f.message(snapshot(), time.monotonic()))
        moved = dict(RECT, points=[[0, 0], [9, 0], [9, 6], [0, 6]], rev=4)
        self.assertIn('plan', f.message(snapshot(moved), time.monotonic()))
        cleared = f.message(snapshot(None), time.monotonic())
        self.assertEqual(cleared['plan']['points'], [])


class TestAppView(unittest.TestCase):
    """平板在预览导入的图纸（还没应用到车）：大屏跟着画；那台平板断开就不再画。"""

    FILE = {'name': '操场.csv', 'origin': [30.1, 120.2],
            'route': rp.parse_spec(dict(kind='path', points=[[0, 0], [5, 0], [5, 3]], closed=True)).to_dict(digits=4)}

    def test_command_parsing(self):
        import app_link
        cmd = app_link.parse_command(json.dumps({'type': 'view', 'file': dict(self.FILE, route=dict(kind='path', points=[[0, 0], [5, 0], [5, 3]], closed=True))}))
        self.assertEqual((cmd['file']['name'], cmd['file']['route']['points'][2], cmd['file']['origin']), ('操场.csv', [5.0, 3.0], [30.1, 120.2]))
        self.assertIsNone(app_link.parse_command('{"type": "view", "file": null}')['file'])
        for bad in ({'type': 'view', 'file': 3}, {'type': 'view', 'file': {'route': {'kind': 'star'}}},
                    {'type': 'view', 'file': dict(self.FILE, origin=[100, 0])}):
            with self.subTest(bad=str(bad)[:50]), self.assertRaises(app_link.CommandError):
                app_link.parse_command(json.dumps(bad))

    def test_view_follows_the_last_tablet_and_clears_on_disconnect(self):
        import time
        view = cd.AppView()
        f = cd.Feed(name='小车', profile=PROFILE, sessions=cd.Sessions(), network=FakeNetwork(), view=view)
        self.assertIsNone(f.message(snapshot(), time.monotonic())['view']['file'])

        view.set('app-1', self.FILE)
        shown = f.message(snapshot(), time.monotonic())['view']['file']
        self.assertEqual((shown['name'], shown['count'], shown['points']), ('操场.csv', 3, [[0, 0], [5, 0], [5, 3], [0, 0]]))
        self.assertNotIn('view', f.message(snapshot(), time.monotonic()))        # 没变就不重发
        view.clear('app-2')                                                     # 别的平板断开不影响
        self.assertNotIn('view', f.message(snapshot(), time.monotonic()))
        view.clear('app-1')
        self.assertIsNone(f.message(snapshot(), time.monotonic())['view']['file'])

    def test_multiple_control_points_are_validated_and_reach_display_feed(self):
        import app_link
        import time
        points = [[0, 0], [5, 0], [5, 3], [0, 3]]
        file = dict(self.FILE, control_points=points)
        parsed = app_link.parse_command(json.dumps({'type': 'view', 'file': file}))['file']
        self.assertEqual(parsed['control_points'], points)
        view = cd.AppView()
        view.set('control-point-test', parsed)
        feed = cd.Feed(name='test', profile=PROFILE, sessions=cd.Sessions(), network=FakeNetwork(), view=view)
        shown = feed.message(snapshot(), time.monotonic())['view']['file']
        self.assertEqual(shown['control_points'], points)
        for bad in ([], [[0, 0]], [[0, 0]] * 401, [[0, 0], [1001, 0]], [[0, 0], [True, 0]], [[0, 0], [1]]):
            with self.subTest(points=bad), self.assertRaises(app_link.CommandError):
                app_link.parse_command(json.dumps({'type': 'view', 'file': dict(self.FILE, control_points=bad)}))

    def test_drawn_points_for_every_kind(self):
        circle = rp.parse_spec(dict(kind='circle', center=[1, 2], radius=3)).to_dict()
        pts = cd.drawn_points(circle)
        self.assertEqual(len(pts), 129)
        self.assertTrue(all(abs(math.hypot(p[0] - 1, p[1] - 2) - 3) < 1e-3 for p in pts))
        legacy = rp.parse_spec(dict(kind='points', points=[[0, 0], [10, 0], [10, 8]], closed=False)).to_dict()
        self.assertGreater(len(cd.drawn_points(legacy)), 50)                     # 路点折线按圆弧过弯画
        self.assertEqual(cd.drawn_points({'kind': 'star'}), [])


class TestRoutes(unittest.TestCase):

    def call(self, path):
        h = FakeHandler(path)
        handled = cd.handle(h, snapshot=snapshot, name='小车', profile=PROFILE, sessions=cd.Sessions(),
                            network=FakeNetwork())
        return handled, h

    def test_page_script_and_vehicle(self):
        ok, h = self.call('/display')
        self.assertTrue(ok)
        self.assertIn('text/html', h.headers['Content-Type'])
        self.assertIn(b'/display/display.js', h.wfile.getvalue())
        ok, h = self.call('/display/display.js?v=1')
        self.assertTrue(ok)
        self.assertIn('javascript', h.headers['Content-Type'])
        ok, h = self.call('/display/vehicle.bin')
        self.assertEqual((ok, h.headers.get('Content-Encoding')), (True, 'gzip'))
        self.assertTrue(gzip.decompress(h.wfile.getvalue()).startswith(b'WRIG0001'))

    def test_only_whitelisted_files_are_served(self):
        for path in ('/display/../route_plan.py', '/display/car_display.py', '/displayx', '/display/wheeltec_rtk.wrmesh'):
            with self.subTest(path=path):
                self.assertFalse(self.call(path)[0])


if __name__ == '__main__':
    unittest.main()
