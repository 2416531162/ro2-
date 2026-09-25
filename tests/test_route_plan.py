#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""路线几何（radar_system/route_plan.py）测试：路点折线 + 圆弧过弯、圆、Dubins 接入段、参数校验。"""
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
        path, entry = rp.plan(s, car, MIN_R)
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


def drive(pieces, car):
    x, y, yaw = car
    pts = [(x, y)]
    for length, k in pieces:
        n = max(1, int(length / 0.05))
        for i in range(1, n + 1):
            pts.append(rp.advance(x, y, yaw, length * i / n, k)[:2])
        x, y, yaw = rp.advance(x, y, yaw, length, k)
    return pts, (x, y, yaw)


class TestConnectedPath(unittest.TestCase):
    """连线路线：操作员点的点按连线顺序走直线，不自动连、不闭合就不绕圈。"""

    def test_straight_ahead_needs_no_turning(self):
        pieces, entry, dev = rp.drawn_reference([(3, 0), (10, 0)], (0, 0, 0), MIN_R)
        self.assertEqual(len(pieces), 1)
        self.assertAlmostEqual(pieces[0][0], 10.0, places=9)
        self.assertEqual(pieces[0][1], 0.0)
        self.assertAlmostEqual(dev, 0.0)
        self.assertLess(entry, 3.0)

    def test_close_points_and_side_start_end_on_last_point(self):
        # 10 cm 点距也照走；起点在左前方
        pts = [(4 + 0.1 * i, 2 + 0.3 * math.sin(i * 0.1)) for i in range(80)]
        pieces, entry, dev = rp.drawn_reference(pts, (0, 0, 0), MIN_R)
        _, end = drive(pieces, (0, 0, 0))
        self.assertLess(math.hypot(end[0] - pts[-1][0], end[1] - pts[-1][1]), 0.03)
        self.assertLess(dev, 0.1)
        self.assertTrue(all(abs(k) <= 1 / MIN_R + 1e-12 for _, k in pieces))

    def test_start_reached_without_looping(self):
        # 左后方 117°、正左 5 m：先满舵转到对准再直线开，转过的角度不超过半圈
        for first in ((-2, 4), (0, 5)):
            with self.subTest(first=first):
                pieces, entry, _ = rp.drawn_reference([first, (first[0], first[1] + 5)], (0, 0, 0), MIN_R)
                turned = done = 0.0
                for length, k in pieces:          # 只算开往第 1 个点这一段
                    if done >= entry - 1e-9:
                        break
                    turned += abs(k) * length
                    done += length
                self.assertLess(turned, math.pi)

    def test_unreachable_start_is_refused(self):
        # 正后方、紧贴车侧（在满舵转弯圆里面）：不倒车不绕圈到不了
        for first in ((-3, 0), (0.2, 0.8), (0, 3)):
            with self.subTest(first=first):
                with self.assertRaises(ValueError) as ctx:
                    rp.drawn_reference([first, (first[0] + 3, first[1])], (0, 0, 0), MIN_R)
                self.assertIn('车头前方', str(ctx.exception))

    def test_sharp_corner_is_cut_as_tight_as_the_car_can(self):
        pts = [(2, 0), (6, 0), (6, 8)]
        pieces, _, dev = rp.drawn_reference(pts, (0, 0, 0), MIN_R)
        _, end = drive(pieces, (0, 0, 0))
        self.assertLess(math.hypot(end[0] - 6, end[1] - 8), 0.05)
        self.assertLess(dev, 1.2)                     # 满舵半径 1.92 m 过直角，偏离约 1 m
        self.assertGreater(dev, 0.5)

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

    def test_plan_builds_trackable_path_from_car(self):
        spec = rp.parse_spec(dict(kind='path', points=[[3, 0], [9, 0], [9, 7]], closed=False))
        path, entry = rp.plan(spec, (0.0, 0.0, 0.0), MIN_R)
        self.assertEqual(path.pose(0.0), (0.0, 0.0, 0.0))
        self.assertLessEqual(path.kappa_abs_max, 1 / MIN_R + 1e-12)
        x, y, _ = path.pose(path.length)
        self.assertLess(math.hypot(x - 9, y - 7), 0.05)


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
