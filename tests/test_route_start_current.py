"""开始作业（route.start_current）：路线不分起点终点，从离车最近的地方开始、跑完整条路线（2026-10-01 用户）。

几何、命令解析、真实 mission 状态；差速车按理想车替身跑一遍，查喷头画满整条线 / 整圈。
"""
import json
import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, 'radar_system'), ROOT, os.path.join(ROOT, 'tests')]
import route_plan as rp
import app_link
from path_tracker import PathConfig, differential_config
from test_differential_drive import DiffCar, diff_tracker, run
from test_line_mission import Base, FakeFusion, SAMPLES
from line_mission import LineMission

BODY = (.67, .08, .335)
D = .3
R_MIN = .556
LINE = dict(kind='path', points=[[0, 0], [10, 0]], closed=False, cruise=.8)
LEGS = dict(kind='path', points=[[0, 0], [20, 0], [20, 10]], closed=False, cruise=.5)
RECT = dict(kind='path', points=[[0, 0], [10, 0], [10, 5], [0, 5]], closed=True, cruise=.8)
CIRCLE = dict(kind='circle', center=[0, 0], radius=2.5, clockwise=True, laps=1, cruise=.5)


def plan(raw, car, radius=R_MIN, nozzle=D):
    return rp.plan_current(rp.parse_spec(raw), car, radius, body=BODY, nozzle_x=nozzle)


def nozzle_at(section, s):
    x, y, yaw = section.car_pose(s)
    return x + D * math.cos(yaw), y + D * math.sin(yaw)


def drive(raw, car_xy_yaw):
    """理想差速车按 plan_current 跑完，返回 (跟踪器, 每步喷头位置)。"""
    x, y, yaw = car_xy_yaw
    sections, _ = plan(raw, (x, y, yaw))
    car = DiffCar(x, y, math.degrees(yaw))
    tr = diff_tracker(sections)
    assert tr.start(car.pose())
    trail = []
    run(tr, car, until=400.0,
        every=lambda t, tr, c: trail.append((c.x + D * math.cos(c.yaw), c.y + D * math.sin(c.yaw))))
    return tr, trail


class OpenLine(unittest.TestCase):

    def test_nozzle_on_first_point_backs_up_then_runs_up(self):
        sections, entry = plan(LINE, (-D, 0, 0))                     # 喷头压在 A、车头朝 B
        self.assertEqual([s.direction for s in sections], [-1, 1])
        self.assertAlmostEqual(-.8, sections[0].car_pose(sections[0].path.length)[0])   # 后轴退到 A 后方 0.8 m
        self.assertAlmostEqual(1.0, entry)                          # 倒 0.5 m + 助跑 0.5 m
        end = sections[-1]
        self.assertAlmostEqual(10.05, nozzle_at(end, end.path.length)[0])               # 画到 B，多开 5 cm

    def test_at_last_point_turns_backs_up_and_paints_back(self):
        sections, entry = plan(LINE, (9.7, 0, 0))                   # 从 A 开过来，喷头停在 B
        self.assertEqual([s.direction for s in sections], [-1, 1])
        self.assertAlmostEqual(math.pi, abs(sections[0].car_pose(0)[2]), places=6)      # 原地掉头
        self.assertAlmostEqual(1.1, sections[0].path.length)        # 后轴从 B 前 0.3 m 退到 B 后方 0.8 m
        self.assertAlmostEqual(1.6, entry)
        x, y = nozzle_at(sections[-1], sections[-1].path.length)
        self.assertAlmostEqual(-.05, x, places=6)
        self.assertAlmostEqual(0, y, places=6)
        self.assertEqual(((0., 0.), (10., 0.)), rp.parse_spec(LINE).points)          # 原图纸不变

    def test_middle_goes_to_nearer_end_and_paints_whole_line(self):
        for car, first, last in (((5.5, 0, 0), 10.0, -.05), ((4.0, 0, 0), 0.0, 10.05)):
            with self.subTest(car=car):
                spec = rp.start_from_nearest(rp.parse_spec(LINE), car, R_MIN, body=BODY, nozzle_x=D)
                self.assertEqual(first, spec.points[0][0])
                sections, entry = plan(LINE, car)
                self.assertGreater(entry, 1.0)
                self.assertAlmostEqual(last, nozzle_at(sections[-1], sections[-1].path.length)[0], places=6)

    def test_off_mark_backs_up_onto_the_line(self):
        # 标点时对准点和喷头差 0.24 m：倒车把后轴带回线上（助跑起点在线上），往前那段的参考线就是这条线
        sections, entry = plan(LINE, (9.7, .24, 0))
        self.assertEqual([s.direction for s in sections], [-1, 1])
        x, y, yaw = sections[-1].car_pose(0)
        self.assertAlmostEqual(10.8, x, places=6)
        self.assertAlmostEqual(0, y, places=6)
        self.assertAlmostEqual(math.pi, abs(yaw), places=6)
        self.assertAlmostEqual(math.hypot(1.1, .24) + .5, entry, places=6)

    def test_already_behind_the_runup_on_the_line_drives_straight_in(self):
        sections, entry = plan(LINE, (-1.5, 0, 0))
        self.assertEqual([1], [s.direction for s in sections])
        self.assertAlmostEqual(1.2, entry)

    def test_multi_point_path_from_last_point_runs_back_through_corners(self):
        sections, _ = plan(LEGS, (20, 9.7, math.pi / 2))            # 喷头在最后一点 (20, 10)
        x, y = nozzle_at(sections[-1], sections[-1].path.length)
        self.assertLess(math.hypot(x + .05, y), .01)

    def test_far_away_is_refused(self):
        with self.assertRaisesRegex(ValueError, '太远'):
            plan(LINE, (0, -100, 0))


class ClosedLoops(unittest.TestCase):

    def test_rectangle_starts_mid_edge_and_closes_the_full_loop(self):
        spec = rp.start_from_nearest(rp.parse_spec(RECT), (4.7, -.1, 0), R_MIN, body=BODY, nozzle_x=D)
        self.assertEqual(((5., 0.), (10., 0.), (10., 5.), (0., 5.), (0., 0.)),
                         tuple((round(x, 6) + 0.0, round(y, 6) + 0.0) for x, y in spec.points))
        self.assertAlmostEqual(30.0, rp.path_length(rp.path_points(spec)))            # 整圈一点不少
        tr, trail = drive(RECT, (4.7, -.1, 0))
        self.assertEqual('DONE', tr.st.state)
        for mid in ((7.5, 0), (10, 2.5), (5, 5), (0, 2.5), (2.5, 0)):                  # 四条边都画到
            self.assertLess(min(math.dist(mid, p) for p in trail), .03, mid)

    def test_rectangle_near_a_corner_starts_at_the_corner(self):
        spec = rp.start_from_nearest(rp.parse_spec(RECT), (9.6, .1, 0), R_MIN, body=BODY, nozzle_x=D)
        self.assertEqual((10., 0.), spec.points[0])
        self.assertEqual(4, len(spec.points))

    def test_two_laps_from_nearest_point(self):
        spec = rp.start_from_nearest(rp.parse_spec(dict(RECT, laps=2)), (5.3, 5.1, math.pi), R_MIN,
                                     body=BODY, nozzle_x=D)
        self.assertEqual((5., 5.), tuple(round(c, 6) for c in spec.points[0]))
        self.assertAlmostEqual(60.0, rp.path_length(rp.path_points(spec)))

    def test_circle_nozzle_near_the_circle_starts_there_and_draws_full_circle(self):
        car = (2.5, -.3, -math.pi / 2)                              # 喷头 (2.5, -0.6) 在圆边上
        sections, entry = plan(CIRCLE, car)
        self.assertEqual([-1, 1], [s.direction for s in sections])  # 喷头在附近：倒车助跑进圆
        r = rp.circle_axle_radius(2.5, D, R_MIN)
        self.assertAlmostEqual(2 * math.pi * r + rp.VERTEX_PASS_M + .5, sections[-1].path.length, places=6)
        tr, trail = drive(CIRCLE, car)
        self.assertEqual('DONE', tr.st.state)
        on = [p for p in trail if abs(math.hypot(*p) - 2.5) < .02]
        angles = sorted(math.degrees(math.atan2(y, x)) for x, y in on)
        gaps = [b - a for a, b in zip(angles, angles[1:])] + [angles[0] + 360 - angles[-1]]
        self.assertLess(max(gaps), 3.0)                             # 整圈都画到，没有缺口

    def test_circle_from_the_centre_draws_the_full_circle(self):
        # 定点画圆：喷头对准圆心。2026-10-01 实车 4 次都没画完一圈就报完成（R2.5 只画了 1.8 m）
        r = rp.circle_axle_radius(2.5, D, R_MIN)
        for yaw in (0.0, 1.0, -2.5):
            with self.subTest(yaw=yaw):
                car = (-D * math.cos(yaw), -D * math.sin(yaw), yaw)
                sections, entry = plan(CIRCLE, car)
                self.assertAlmostEqual(2 * math.pi * r + rp.VERTEX_PASS_M + rp.ENTRY_LEAD_M,
                                       sections[-1].path.length, places=6)
                self.assertLess(entry, 2.5 + rp.ENTRY_LEAD_M + .6)
                tr, _ = drive(CIRCLE, car)
                self.assertEqual('DONE', tr.st.state)

    def test_two_lap_circle(self):
        sections, _ = plan(dict(CIRCLE, laps=2), (2.5, -.3, -math.pi / 2))
        r = rp.circle_axle_radius(2.5, D, R_MIN)
        self.assertAlmostEqual(4 * math.pi * r + rp.VERTEX_PASS_M + .5, sections[-1].path.length, places=6)


class Ackermann(unittest.TestCase):
    """旧的阿克曼车（Wheeltec）同一规则，接入照旧（Reeds-Shepp / Dubins）。"""

    def test_full_route_from_nearest_place(self):
        cases = [
            ('line near B', LINE, (9.0, 0, math.pi)),
            ('rectangle', RECT, (5, .3, 0)),
            ('circle', dict(CIRCLE, radius=3), (3, 0, -math.pi / 2)),
            ('points loop', dict(RECT, kind='points', turn_radius=2.0), (9, 0, 0)),
        ]
        for name, raw, car in cases:
            with self.subTest(name):
                sections, entry = rp.plan_current(rp.parse_spec(raw), car, 1.916, body=BODY, nozzle_x=None)
                self.assertTrue(0 <= entry <= rp.MAX_ENTRY_M)
                for a, b in zip(sections, sections[1:]):
                    self.assertLess(math.dist(a.car_pose(a.path.length)[:2], b.car_pose(0)[:2]), 1e-6)
                if raw['kind'] == 'circle':
                    self.assertAlmostEqual(2 * math.pi * 3, sum(s.path.length for s in sections) - entry, places=6)


class Tracking(unittest.TestCase):

    def test_reverse_runup_creeps_all_the_way(self):
        sections, _ = plan(LINE, (-D, 0, 0))
        tr = diff_tracker(sections)
        # 倒车段只慢走 0.15 m（只是摆车、不画线）；倒车后往前助跑整段 0.5 m 按 0.15 m/s 慢走（2026-10-01 拐角提速）
        self.assertEqual((.15, .5), (tr.trackers[0].cfg.start_creep_m, tr.trackers[1].cfg.start_creep_m))
        self.assertEqual((.1, .15), (tr.trackers[0].cfg.start_creep_mps, tr.trackers[1].cfg.start_creep_mps))
        car = DiffCar(-D, 0, 0)
        self.assertTrue(tr.start(car.pose()))
        fast = []
        run(tr, car, every=lambda t, tr, c: fast.append(c.x) if tr.index == 1 and c.v > .151 else None)
        self.assertGreaterEqual(min(fast), -.8 + .5 - .02)          # 助跑 0.5 m 内不超过 0.15 m/s
        self.assertEqual('DONE', tr.st.state)

    def test_short_entry_section_starts(self):
        # 接入段比 start_min_left_m（0.5 m）短也能起步：只有整条路线的最后一段才查「离终点太近」
        sections, _ = plan(LINE, (-.6, 0, 0))
        self.assertLess(sections[0].path.length, .5)
        self.assertEqual((True, ''), diff_tracker(sections).check_start(DiffCar(-.6, 0, 0).pose()))

    def test_pause_near_the_end_resumes(self):
        # 2026-10-01 10:20 实车：段尾剩 0.14 m 时定位抖了一下，恢复后按「离终点不到 0.5 m 不起步」一直起不了步
        sections, _ = plan(LINE, (-1.5, 0, 0))                     # 只有一段（直接开进来）
        self.assertEqual(1, len(sections))
        tr = diff_tracker(sections)
        car = DiffCar(-1.5, 0, 0)
        self.assertTrue(tr.start(car.pose()))
        paused = []

        def glitch(t, tr, c):
            if not paused and tr.st.s > sections[0].path.length - .2:
                tr.pause('pose_unusable')
                paused.append(t)
                self.assertTrue(tr.start(c.pose()))                  # 恢复：接着把这段走完
        run(tr, car, every=glitch)
        self.assertTrue(paused)
        self.assertEqual('DONE', tr.st.state)

    def test_differential_parameters(self):
        cfg = differential_config(PathConfig())
        self.assertEqual((.30, None, .5, .15, .15), (cfg.delay_s, cfg.stop_delay_s, cfg.reverse_creep_m,
                                                  cfg.reverse_creep_mps, cfg.reverse_section_creep_m))
        self.assertEqual(.8, rp.PIVOT_REVERSE_MIN_M)


    def test_differential_section_options(self):
        from path_tracker import DIFF_REVERSE_MPS, DIFF_SETTLE_S
        from test_differential_drive import DIFF
        from types import SimpleNamespace
        from line_tracker import LineConfig
        m = LineMission(None, SAMPLES, tracker=SimpleNamespace(cfg=LineConfig(), geo=DIFF), body=BODY, nozzle_x=D)
        self.assertEqual((DIFF_REVERSE_MPS, DIFF_SETTLE_S), (.5, .2))
        opts = m.section_options()
        self.assertEqual((opts['reverse_mps'], opts['settle_s']), (.5, .2))


class Commands(unittest.TestCase):

    def test_action_is_explicit_and_preserves_route(self):
        raw = dict(type='route', action='start_current', id='current', route=LEGS)
        command = app_link.parse_command(json.dumps(raw))
        self.assertEqual('start_current', command['action'])
        self.assertEqual([[0., 0.], [20., 0.], [20., 10.]], command['route']['points'])


class Mission(Base):

    def setUp(self):
        super().setUp()
        self.m = LineMission(self.fusion, SAMPLES, route_path=os.path.join(self.tmp.name, 'route.json'))

    def test_start_current_keeps_the_original_route(self):
        self.fusion.current = FakeFusion.make(1, 0, 180)
        ok, why = self.m.command(dict(cmd='route_start_current', route=LEGS), 0)
        self.assertTrue(ok, why)
        self.assertEqual('SELECTING', self.m.phase)
        self.assertEqual(3, len(self.m._route_local().points))
        self.assertAlmostEqual(0, self.m._route_local().points[0][0])
        self.assertGreater(self.m.operations.execution['entry_m'], 0)

    def test_paused_tracker_is_not_reported_as_driving(self):
        self.fusion.current = FakeFusion.make(-5, 0, 0)
        ok, why = self.m.command(dict(cmd='route_start_current', route=LINE), 0)
        self.assertTrue(ok, why)
        self.drive(0, 3)
        self.assertEqual('RUNNING', self.m.phase)
        self.m.tracker.pause('pose_unusable')
        self.m.tick(3.05, 'NAVIGATION')
        self.assertNotEqual('行驶中', self.m.message)
        self.assertIn('停车', self.m.message)


if __name__ == '__main__':
    unittest.main()
