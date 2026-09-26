#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""路线几何（radar_system/route_plan.py）测试：Reeds-Shepp 最短路径、连线路线车身中心压线（急弯开出去倒车掉头）、
路点折线 + 圆弧过弯、圆、Dubins 接入段、参数校验。"""
import math
import os
import random
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, ROOT)

import route_plan as rp  # noqa: E402

MIN_R = 1.72 / 0.9


def spec(**kw):
    base = dict(kind='points', points=[[0, 0], [10, 0], [10, 8], [0, 8]], closed=True, laps=1,
                turn_radius=2.5, cruise=0.5)
    base.update(kw)
    return rp.parse_spec(base)


def run(start, pieces):
    x, y, yaw = start
    for length, kappa in pieces:
        x, y, yaw = rp.advance(x, y, yaw, length, kappa)
    return x, y, yaw


def samples(start, pieces, step=0.05):
    out, (x, y, yaw) = [], start
    for length, kappa in pieces:
        n = max(1, int(length / step))
        for i in range(n):
            out.append(rp.advance(x, y, yaw, length * i / n, kappa))
        x, y, yaw = rp.advance(x, y, yaw, length, kappa)
    out.append((x, y, yaw))
    return out


def seg_dist(p, a, b):
    ax, ay, bx, by = *a, *b
    t = max(0.0, min(1.0, ((p[0] - ax) * (bx - ax) + (p[1] - ay) * (by - ay)) / ((bx - ax) ** 2 + (by - ay) ** 2)))
    return math.hypot(p[0] - ax - t * (bx - ax), p[1] - ay - t * (by - ay))


class TestPointsLoop(unittest.TestCase):

    def test_closed_rectangle_one_lap_ends_on_first_point(self):
        start, pieces = rp.loop_pieces(spec(), MIN_R)
        self.assertEqual(start, (0.0, 0.0, 0.0))
        x, y, yaw = run(start, pieces)
        self.assertAlmostEqual(x, 0.0, places=9)
        self.assertAlmostEqual(y, 0.0, places=9)
        self.assertAlmostEqual(math.degrees(rp.wrap(yaw)), -90.0, places=6)
        # 三个 90° 拐角各比折线短 2R − πR/2；第 1 个路点只跑一圈时不拐弯
        saving = 2 * 2.5 - math.pi * 2.5 / 2
        self.assertAlmostEqual(sum(p[0] for p in pieces), 36.0 - 3 * saving, places=9)
        self.assertEqual(sum(1 for p in pieces if p[1]), 3)
        self.assertTrue(all(abs(k) <= 1 / 2.5 + 1e-12 for _, k in pieces))

    def test_multi_lap_rounds_the_first_corner_between_laps(self):
        _, one = rp.loop_pieces(spec(laps=1), MIN_R)
        start, three = rp.loop_pieces(spec(laps=3), MIN_R)
        saving = 2 * 2.5 - math.pi * 2.5 / 2
        self.assertAlmostEqual(sum(p[0] for p in three), 3 * 36.0 - 11 * saving, places=9)
        self.assertEqual(sum(1 for p in three if p[1]), 11)
        x, y, _ = run(start, three)
        self.assertAlmostEqual(math.hypot(x, y), 0.0, places=8)
        self.assertLess(len(one), len(three))

    def test_path_stays_on_polyline_except_rounded_corners(self):
        pts = [(0, 0), (12, 0), (15, 9), (4, 11), (-3, 5)]
        s = spec(points=[list(p) for p in pts], laps=2)
        start, pieces = rp.loop_pieces(s, MIN_R)
        legs = list(zip(pts, pts[1:] + pts[:1]))
        for x, y, _ in samples(start, pieces):
            # 离折线最远处在拐角圆弧中点：R/cos(Δ/2) − R，这组路点最大拐角约 108°
            self.assertLess(min(seg_dist((x, y), a, b) for a, b in legs), 2.5 / math.cos(math.radians(54)) - 2.5 + 1e-6)

    def test_open_route_ends_on_last_point_along_last_leg(self):
        start, pieces = rp.loop_pieces(spec(points=[[0, 0], [8, 0], [8, 6]], closed=False, laps=3), MIN_R)
        x, y, yaw = run(start, pieces)
        self.assertAlmostEqual(x, 8.0, places=9)
        self.assertAlmostEqual(y, 6.0, places=9)
        self.assertAlmostEqual(yaw, math.pi / 2, places=9)
        self.assertEqual(sum(1 for p in pieces if p[1]), 1)   # 开放路线只跑一次

    def test_collinear_points_add_no_arc(self):
        _, pieces = rp.loop_pieces(spec(points=[[0, 0], [5, 0], [10, 0]], closed=False), MIN_R)
        self.assertEqual(len(pieces), 2)
        self.assertTrue(all(k == 0.0 for _, k in pieces))

    def test_problems_are_reported_in_chinese(self):
        cases = [
            (spec(points=[[0, 0], [10, 0]]), '至少要 3 个路点'),
            (spec(points=[[0, 0]], closed=False), '至少要 2 个路点'),
            (spec(points=[[0, 0], [0.1, 0], [5, 5]]), '离得太近'),
            (spec(points=[[0, 0], [3, 0], [3, 2]], closed=False), '太近：按 2.5 m 半径拐弯'),
            (spec(points=[[0, 0], [10, 0], [0, 0.01]], closed=False), '原路折返'),
            (spec(turn_radius=1.5), '小于车能稳定跟踪'),
            (spec(kind='circle', center=None), '圆心'),
            (spec(kind='circle', center=[0, 0], radius=1.5), '圆半径'),
        ]
        for s, text in cases:
            with self.subTest(text=text), self.assertRaises(ValueError) as ctx:
                rp.loop_pieces(s, MIN_R)
            self.assertIn(text, str(ctx.exception))


class TestCircle(unittest.TestCase):

    def test_starts_south_and_closes(self):
        for cw, yaw0 in ((False, 0.0), (True, math.pi)):
            with self.subTest(clockwise=cw):
                start, pieces = rp.loop_pieces(spec(kind='circle', center=[3, 4], radius=2.5, clockwise=cw,
                                                    laps=2), MIN_R)
                self.assertEqual(start, (3.0, 1.5, yaw0))
                self.assertEqual(len(pieces), 1)
                self.assertAlmostEqual(pieces[0][0], 2 * 2 * math.pi * 2.5)
                self.assertAlmostEqual(pieces[0][1], (-1 if cw else 1) / 2.5)
                for x, y, _ in samples(start, pieces, 0.2):
                    self.assertAlmostEqual(math.hypot(x - 3, y - 4), 2.5, places=9)


class TestDubins(unittest.TestCase):

    def test_random_poses_always_reach_goal(self):
        rng = random.Random(7)
        for _ in range(3000):
            s = (rng.uniform(-15, 15), rng.uniform(-15, 15), rng.uniform(-math.pi, math.pi))
            g = (s[0] + rng.uniform(-6, 6), s[1] + rng.uniform(-6, 6), rng.uniform(-math.pi, math.pi))
            r = rng.choice((1.0, 2.0, 2.5, 3.0))
            pieces = rp.dubins(s, g, r)
            x, y, yaw = run(s, pieces)
            self.assertLess(math.hypot(x - g[0], y - g[1]), 1e-6)
            self.assertLess(abs(rp.wrap(yaw - g[2])), 1e-6)
            self.assertLessEqual(len(pieces), 3)
            self.assertTrue(all(length > 0 and abs(k) in (0.0, 1 / r) for length, k in pieces))

    def test_simple_cases(self):
        self.assertEqual(rp.dubins((1, 2, 0.3), (1, 2, 0.3), 2.5), [])
        self.assertEqual(rp.dubins((0, 0, 0), (10, 0, 0), 2.5), [(10.0, 0.0)])
        # 正后方同向：最短是先转出去再绕回来，总长大于两点距离
        pieces = rp.dubins((0, 0, 0), (-3, 0, 0), 2.5)
        self.assertGreater(sum(p[0] for p in pieces), 3.0)


class TestPlan(unittest.TestCase):

    def test_path_starts_at_car_and_joins_route(self):
        s = spec()
        car = (-3.0, -4.0, math.radians(120))
        sections, entry = rp.plan(s, car, MIN_R)
        self.assertEqual([sec.direction for sec in sections], [1])
        path = sections[0].path
        self.assertEqual(path.pose(0.0), car)
        _, loop = rp.loop_pieces(s, MIN_R)
        self.assertAlmostEqual(path.length, entry + sum(p[0] for p in loop), places=9)
        x, y, yaw = path.pose(entry)
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        self.assertAlmostEqual(rp.wrap(yaw), 0.0, places=6)
        self.assertLessEqual(path.kappa_abs_max, 1 / 2.5 + 1e-12)

    def test_car_too_far_is_refused(self):
        with self.assertRaises(ValueError) as ctx:
            rp.plan(spec(), (80.0, 0.0, 0.0), MIN_R)
        self.assertIn('太远', str(ctx.exception))


BODY = (0.67, 0.08, 0.335)              # 后轴中心到车头、车尾、半宽（robot_core/robot.json）
CENTER = 0.5 * (BODY[0] - BODY[1])     # 车身中心在后轴前 0.295 m


def section_points(sections, step=0.005, x=CENTER):
    """各段车身上一点（默认车身中心）走过的点（按段）。倒车段按行进方向画，车头 = yaw + π。"""
    out = []
    for sec in sections:
        p = sec.path
        n = max(1, int(p.length / step))
        pts = []
        for i in range(n + 1):
            px, py, yaw = sec.car_pose(p.length * i / n)
            pts.append((px + x * math.cos(yaw), py + x * math.sin(yaw)))
        out.append(pts)
    return out


def cover_gap(polyline, sections, step=0.01):
    """连线上每一点到车身中心轨迹的最远距离（压线误差）。"""
    track = [q for pts in section_points(sections) for q in pts]
    cells = {}
    for q in track:
        cells.setdefault((int(math.floor(q[0] / 0.25)), int(math.floor(q[1] / 0.25))), []).append(q)
    worst = 0.0
    for a, b in zip(polyline, polyline[1:]):
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
        for i in range(n + 1):
            x, y = a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n
            cx, cy = int(math.floor(x / 0.25)), int(math.floor(y / 0.25))
            near = [q for dx in (-1, 0, 1) for dy in (-1, 0, 1) for q in cells.get((cx + dx, cy + dy), ())]
            worst = max(worst, min((math.hypot(x - q[0], y - q[1]) for q in near), default=math.inf))
    return worst


def path_spec(points, **kw):
    return rp.parse_spec(dict(dict(kind='path', points=points, closed=False), **kw))


def body_clear(pose, along):
    """整车（四个角）沿 along 方向离原点最近多远：负数表示还有部分车身没开过去。"""
    x, y, yaw = pose
    ux, uy = math.cos(along), math.sin(along)
    return min((x + a * math.cos(yaw) - b * math.sin(yaw)) * ux + (y + a * math.sin(yaw) + b * math.cos(yaw)) * uy
               for a in (-BODY[1], BODY[0]) for b in (-BODY[2], BODY[2]))


SHAPES = {
    'rect': dict(points=[[0, 0], [8, 0], [8, 6], [0, 6]], closed=True, laps=2),
    'zigzag': dict(points=[[3, 0], [9, 0], [12, 5], [16, 1], [22, 4]], closed=False),
    'triangle': dict(points=[[0, 0], [10, 0], [5, 8.66]], closed=True, laps=1),
    'obtuse': dict(points=[[0, 0], [8, 0], [2, 4], [2, 12]], closed=False),       # 146° 急弯
    'gentle': dict(points=[[0, 0], [6, 0], [12, 2], [18, 2.5]], closed=False),   # 18°、13° 缓弯
    'curve': dict(points=[[0.2 * i, 3.0 * math.sin(0.03 * i)] for i in range(60)], closed=False),
    'rows': dict(points=[[0, 0], [8, 0], [8, 0.8], [0, 0.8], [0, 1.6], [8, 1.6]], closed=False),  # 行距 0.8 m 往返
    'hairpin': dict(points=[[0, 0], [8, 0], [0, 0.5]], closed=False),            # 176° 折返
}
CARS = [(-3.0, 0.0, 0.0), (-1.0, -3.0, math.pi / 2), (0.3, 0.02, 0.02), (4.0, 3.0, math.pi)]


class TestReedsShepp(unittest.TestCase):
    """可以倒车的最短路径：每个候选都到得了终点，最短的不比只前进的 Dubins 长，前后对调一样长。"""

    def test_reaches_goal_and_never_longer_than_forward_only(self):
        rng = random.Random(5)
        for _ in range(400):
            s = (rng.uniform(-6, 6), rng.uniform(-6, 6), rng.uniform(-math.pi, math.pi))
            g = (s[0] + rng.uniform(-5, 5), s[1] + rng.uniform(-5, 5), rng.uniform(-math.pi, math.pi))
            r = rng.choice((1.0, 1.9, 2.5))
            paths = rp.reeds_shepp_paths(s, g, r)
            self.assertTrue(paths)
            best = min(sum(length for _, length, _ in p) for p in paths)
            self.assertLessEqual(best, sum(length for length, _ in rp.dubins(s, g, r)) + 1e-9)
            back = min(sum(length for _, length, _ in p) for p in rp.reeds_shepp_paths(g, s, r))
            self.assertAlmostEqual(best, back, places=6)

    def test_turning_on_the_spot_costs_exactly_radius_times_angle(self):
        # 原地转 θ：车头要转 θ，路程至少 R·θ；三段打满方向（前进-倒车-前进）正好做到
        for deg in (45, 90, 150, 180):
            th = math.radians(deg)
            way = rp.reeds_shepp((0, 0, 0), (0, 0, th), MIN_R)
            self.assertAlmostEqual(sum(length for _, length, _ in way), MIN_R * th, places=6)
            self.assertEqual([d for d, _, _ in way], [1, -1, 1])


class TestConnectedPath(unittest.TestCase):
    """连线路线：车身中心要压过全部连线，缓弯直接转过去，急弯先开出拐角、倒车一次、再往前。"""

    def plan(self, points, car=(-3.0, 0.0, 0.0), **kw):
        return rp.plan(path_spec(points, **kw), car, MIN_R, body=BODY)

    def test_every_drawn_point_is_driven_over_by_the_body_centre(self):
        for name, kw in SHAPES.items():
            for car in CARS:
                with self.subTest(shape=name, car=car):
                    spec = rp.parse_spec(dict(kind='path', **kw))
                    sections, entry = rp.plan(spec, car, MIN_R, body=BODY)
                    self.assertLessEqual(entry, rp.MAX_ENTRY_M)
                    self.assertLessEqual(cover_gap(rp.path_points(spec), sections), rp.COVER_TOL_M + 0.006)

    def test_sections_join_and_stay_trackable(self):
        from path_tracker import PathConfig, SectionTracker
        from robot_core.kinematics import ChassisGeometry
        track_r = ChassisGeometry().min_turn_radius_m / PathConfig().kappa_margin     # 与 line_mission 相同
        for name, kw in SHAPES.items():
            with self.subTest(shape=name):
                sections, _ = rp.plan(rp.parse_spec(dict(kind='path', **kw)), (-3.0, 0.0, 0.0), track_r, body=BODY)
                for a, b in zip(sections, sections[1:]):
                    xa, ya, ha = a.car_pose(a.path.length)
                    xb, yb, hb = b.car_pose(0.0)
                    self.assertLess(math.hypot(xa - xb, ya - yb), 1e-9)
                    self.assertLess(abs(rp.wrap(ha - hb)), 1e-9)       # 车头连续，只换行进方向
                    self.assertNotEqual(a.direction, b.direction)
                    self.assertGreaterEqual(a.path.length, rp.MIN_RUN_M - 1e-9)
                SectionTracker(sections)                                 # 曲率都在底盘能跟的范围内

    def test_right_angle_goes_out_reverses_once_and_is_shortest(self):
        sections, entry = self.plan([(0, 0), (6, 0), (6, 6)])
        self.assertAlmostEqual(entry, 3.0 - CENTER)                       # 已在第一段延长线上：直线开过去
        self.assertEqual([s.direction for s in sections], [1, -1, 1])    # 开出去 → 倒车 → 继续开
        # 倒车前整车已开出拐角
        self.assertGreaterEqual(body_clear(sections[0].car_pose(sections[0].path.length), 0.0) - 6.0,
                                rp.CORNER_CLEAR_M - 1e-9)
        # 路程：连线 12 m + 车身中心过拐角 / 终点各 VERTEX_PASS_M + 掉 90° 头的最短路程 R·π/2
        route = rp.cover_summary(path_spec([(0, 0), (6, 0), (6, 6)]), MIN_R, BODY)
        self.assertAlmostEqual(route['drive_m'], 12.0 + 3 * rp.VERTEX_PASS_M + MIN_R * math.pi / 2, places=3)
        self.assertEqual((route['cusps'], route['corners']), (2, 1))
        # 拐角点被车身中心压到
        centre = [q for pts in section_points(sections) for q in pts]
        self.assertLess(min(math.hypot(x - 6, y) for x, y in centre), 0.003)

    def test_every_sharp_angle_goes_out_then_reverses_once(self):
        for deg in list(range(27, 181, 3)) + [180]:
            for side in (1, -1):
                with self.subTest(deg=deg, side=side):
                    th = side * math.radians(deg)
                    pts = [(0, 0), (8, 0), (8 + 8 * math.cos(th), 8 * math.sin(th))]
                    sections, _ = self.plan(pts, car=(-2.0, 0.0, 0.0))
                    self.assertEqual([s.direction for s in sections], [1, -1, 1])
                    self.assertGreaterEqual(body_clear(sections[0].car_pose(sections[0].path.length), 0.0) - 8.0,
                                            rp.CORNER_CLEAR_M - 1e-9)
                    extra = rp.cover_summary(path_spec(pts), MIN_R, BODY)['drive_m'] - 16.0 - 3 * rp.VERTEX_PASS_M
                    self.assertLess(extra, MIN_R * math.radians(deg) + 0.2)   # 最短也得转够 R·θ，最多多 0.2 m
                    self.assertLessEqual(cover_gap(pts, sections), 0.006)

    def test_gentle_corner_is_driven_through_without_stopping(self):
        sections, _ = self.plan([(0, 0), (6, 0), (12, 2)])               # 18°：圆弧离拐角 2.4 cm
        self.assertEqual(len(sections), 1)
        self.assertLessEqual(cover_gap([(0, 0), (6, 0), (12, 2)], sections), rp.COVER_TOL_M)

    def test_short_sides_and_hairpins_are_fine(self):
        # 行距 0.8 m 往返、176° 折返：掉头不需要两边有多长的线
        for pts in ([(0, 0), (6, 0), (6, 1), (0, 1)], [(0, 0), (8, 0), (0, 0.5)]):
            with self.subTest(pts=pts):
                sections, _ = self.plan(pts)
                self.assertLessEqual(cover_gap(pts, sections), 0.006)

    def test_entry_from_car_pose(self):
        pts = [(0, 0), (6, 0)]
        # 压在第一段延长线上、车身中心在起点后方：从垂足直线开过去
        sections, entry = self.plan(pts, car=(-2.0, 0.03, math.radians(3)))
        self.assertEqual(len(sections), 1)
        self.assertAlmostEqual(entry, 2.0 - CENTER)
        self.assertEqual(sections[0].path.pose(0.0), (-2.0, 0.0, 0.0))
        # 远处侧面：最短路径开到起点前对正，不用倒车
        sections, entry = self.plan(pts, car=(-6.0, -4.0, math.pi / 2))
        self.assertEqual([s.direction for s in sections], [1])
        # 车停在起点前面一点：先倒回去再开，不绕大圈
        sections, entry = self.plan(pts, car=(0.3, 0.1, 0.0))
        self.assertEqual(sections[0].direction, -1)
        self.assertLess(entry, 3.0)
        # 第一个点在车正后方：倒车掉头开过去
        sections, entry = self.plan([(-3, 0), (-6, 0)], car=(0.0, 0.0, 0.0))
        self.assertLessEqual(entry, 12.0)
        with self.assertRaises(ValueError) as ctx:
            self.plan(pts, car=(60.0, 0.0, 0.0))
        self.assertIn('太远', str(ctx.exception))

    def test_open_path_is_driven_once_closed_loops_laps(self):
        sq = [[0, 0], [8, 0], [8, 6], [0, 6]]
        open_spec = rp.parse_spec(dict(kind='path', points=sq, closed=False, laps=3))
        self.assertEqual(open_spec.laps, 1)
        self.assertEqual(rp.path_points(open_spec), [tuple(p) for p in sq])
        self.assertAlmostEqual(rp.route_length(open_spec, MIN_R)[0], 22.0)
        loop = rp.parse_spec(dict(kind='path', points=sq, closed=True, laps=2))
        self.assertEqual(len(rp.path_points(loop)), 9)
        self.assertAlmostEqual(rp.route_length(loop, MIN_R)[0], 56.0)
        self.assertEqual(rp.route_length(rp.parse_spec(dict(kind='path', points=[[1, 1]])), MIN_R)[0], None)
        # 两圈 7 个直角都掉头补角，每个换向 2 次、多开 R·π/2；车身中心过每个拐角和终点多走 VERTEX_PASS_M（前进、倒回各一次）
        cover = rp.cover_summary(loop, MIN_R, BODY)
        self.assertEqual((cover['corners'], cover['cusps']), (7, 14))
        self.assertAlmostEqual(cover['drive_m'], 56.0 + 15 * rp.VERTEX_PASS_M + 7 * MIN_R * math.pi / 2, places=3)


class TestParse(unittest.TestCase):

    def test_round_trip_and_defaults(self):
        s = rp.parse_spec({'kind': 'circle', 'center': [1, 2]})
        self.assertEqual((s.radius, s.turn_radius, s.laps, s.cruise, s.clockwise), (3.0, 2.5, 1, 0.5, True))
        self.assertEqual(rp.parse_spec(s.to_dict()), s)
        # 开放路线只跑一次
        self.assertEqual(rp.parse_spec({'kind': 'points', 'closed': False, 'laps': 3}).laps, 1)
        geo = rp.parse_spec({'kind': 'points', 'points': [[30.1, 120.2]]}, geodetic=True)
        self.assertEqual(geo.points, ((30.1, 120.2),))

    def test_bad_input(self):
        bad = [None, [], {'kind': 'star'}, {'kind': 'points', 'points': [[0, 0]] * 401},
               {'kind': 'points', 'points': [[0, float('nan')]]}, {'kind': 'points', 'points': [[0, 2000]]},
               {'kind': 'points', 'points': [[0]]}, {'kind': 'points', 'laps': 0},
               {'kind': 'points', 'laps': 11}, {'kind': 'points', 'laps': True},
               {'kind': 'points', 'closed': 1}, {'kind': 'points', 'turn_radius': 0.5},
               {'kind': 'points', 'cruise': 2.0}, {'kind': 'circle', 'radius': '3'}]
        for d in bad:
            with self.subTest(d=str(d)[:60]), self.assertRaises(ValueError):
                rp.parse_spec(d)


if __name__ == '__main__':
    unittest.main()
