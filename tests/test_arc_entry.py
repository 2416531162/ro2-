"""进线只走圆弧（2026-11-27 用户「进线只走圆弧，删掉进线的原地转」；线中拐角、线尾掉头不变）。

11-19 用户「可选圆弧还是原地，圆弧就是可以设置后方有多远的空地，最低要1.5m才允许圆弧」加了圆弧进线；11-27 删了原地转进线：
只往前、按 Dubins 圆弧开到助跑起点车头已顺线，不停车接着助跑；整条接入车身不退到「车在助跑起点时车尾」后方 entry_space_m
以外，空地填得越大自动用越大的半径（最大 3 m，最小 1.25 m）。车已顺着线、不用原地转（直着开进来 / 直着倒到助跑起点）照旧；
后方空地不到 1.5 m（2026-11-27 用户「大于 1.5m 应该也够了吧」）或哪个半径都放不下：不开始，请操作员挪车或把后方空地填大。"""
from dataclasses import replace
from pathlib import Path
import math
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT)]
import route_plan as rp

NOZZLE = (.24, -.4575)
BODY = (.911, .604, .39)
LINE = rp.RouteSpec(kind='path', points=((0., 0.), (12., 0.)), closed=False, cruise=.8)


def line_car(q, side=0., yaw=0.):
    return rp.axle_for((q, side), yaw, *NOZZLE) + (yaw,)


def plan(car, space, spec=LINE):
    info = {}
    secs, entry_m = rp.plan_current(replace(spec, entry_space_m=space), car, .556,
                                    body=BODY, nozzle_x=NOZZLE, info=info)
    return secs, entry_m, info['entry_turn']


def rearmost(secs, entry_m):
    """接入（含助跑）车身四角沿线方向最靠后的坐标，和车在助跑起点时车尾的坐标。"""
    first = secs[-1]
    x0, y0, h0 = rp.axle_for((0., 0.), 0., *NOZZLE) + (0.,)
    worst, done = math.inf, 0.
    for sec in secs:
        for i in range(int(sec.path.length / .02) + 1):
            s = min(sec.path.length, i * .02)
            if done + s > entry_m:
                break
            x, y, yaw = sec.path.pose(s)
            yaw = yaw if sec.direction > 0 else rp.wrap(yaw + math.pi)
            worst = min(worst, rp._body_min_along(x, y, yaw, BODY, h0, x0, y0))
        done += sec.path.length
    g = rp._body_min_along(x0 - rp.RUN_UP_M, y0, 0., BODY, h0, x0, y0)
    return worst, g


class Spec(unittest.TestCase):
    def test_round_trip_and_defaults(self):
        self.assertEqual(rp.ENTRY_TURNS, ('arc',))
        self.assertEqual(rp.RouteSpec().entry_turn, 'arc')
        old = rp.parse_spec(dict(kind='path', points=[[0, 0], [5, 0]], closed=False))
        self.assertEqual((old.entry_turn, old.entry_space_m), ('arc', 0.))        # 旧 app / 旧路线：空地没填
        pivot = rp.parse_spec(dict(LINE.to_dict(), entry_turn='pivot'))          # 旧 app 选的原地转：当圆弧
        self.assertEqual(pivot.entry_turn, 'arc')
        spec = rp.parse_spec(replace(LINE, entry_space_m=2.).to_dict())
        self.assertEqual((spec.entry_turn, spec.entry_space_m), ('arc', 2.))
        with self.assertRaises(ValueError):
            rp.parse_spec(dict(LINE.to_dict(), entry_turn='loop'))
        with self.assertRaises(ValueError):
            rp.parse_spec(dict(LINE.to_dict(), entry_space=-1))


def no_pivot(test, secs, car):
    """接入一下都不原地转：第一段从车头方向起步，段与段之间车头接得上（换向是倒车，不是原地转）。"""
    yaw = car[2]
    for sec in secs:
        head = sec.car_pose(0.)[2]
        test.assertLess(abs(rp.wrap(head - yaw)), rp.ENTRY_PIVOT_MIN_RAD)
        yaw = sec.car_pose(sec.path.length)[2]


class ArcEntry(unittest.TestCase):
    SIDE = line_car(-1., 3., -math.pi / 2)        # 线头侧面 3 m、车头对着线

    def test_below_minimum_space_does_not_start(self):
        self.assertEqual(rp.ENTRY_ARC_MIN_SPACE_M, 1.5)
        for space in (0., 1.0, 1.49):
            with self.subTest(space=space):
                with self.assertRaises(ValueError) as cm:
                    plan(self.SIDE, space)
                self.assertIn('后方空地', str(cm.exception))
                self.assertIn('填大', str(cm.exception))
                self.assertIn('挪', str(cm.exception))

    def test_arc_with_2_m_only_forward_no_stop_and_stays_inside_space(self):
        secs, entry_m, how = plan(self.SIDE, 2.)
        self.assertEqual(how, 'arc')
        self.assertEqual([s.direction for s in secs], [1])                  # 一段往前，中间不停不转
        no_pivot(self, secs, self.SIDE)
        self.assertAlmostEqual(secs[-1].paint_from_m, entry_m, places=6)
        radii = [1 / abs(p.kappa) for p in secs[0].path.pieces if abs(p.kappa) > 1e-9]
        self.assertGreaterEqual(min(radii), rp.ENTRY_ARC_RADII[-1] - 1e-9)
        self.assertEqual(rp.ENTRY_ARC_RADII[-1], 1.25)
        worst, g = rearmost(secs, entry_m)
        self.assertGreaterEqual(worst, g - 2. - 1e-6)
        # 直线助跑至少 ENTRY_ARC_MIN_STRAIGHT_M
        pieces = secs[0].path.pieces
        last_arc = max(i for i, p in enumerate(pieces) if abs(p.kappa) > 1e-9)
        arc_end = sum(p.length for p in pieces[:last_arc + 1])
        self.assertGreaterEqual(secs[0].paint_from_m - arc_end + 1e-6, rp.ENTRY_ARC_MIN_STRAIGHT_M)

    def test_more_space_uses_bigger_radius_and_full_runup(self):
        def radius(space):
            secs = plan(self.SIDE, space)[0]
            return min(1 / abs(p.kappa) for p in secs[0].path.pieces if abs(p.kappa) > 1e-9)
        # 填的空地越大，自动换越大的圆弧（最大 ENTRY_ARC_RADII[0]）
        radii = [radius(s) for s in (2., 2.5, 3., 4., 6., 10.)]
        self.assertEqual(radii, sorted(radii))
        self.assertLess(radii[0], radii[-1])
        self.assertAlmostEqual(radii[-1], rp.ENTRY_ARC_RADII[0])
        secs, entry_m, how = plan(self.SIDE, 5.)
        self.assertEqual(how, 'arc')
        worst, g = rearmost(secs, entry_m)
        self.assertGreaterEqual(worst, g - 5. - 1e-6)

    def test_turning_round_at_the_far_end_needs_more_space(self):
        # 刚画完同一条线、停在线尾外朝外：往前绕回来要更大的空地。1.5 m（最低）放不下、不开始（原来照原地转），
        # 1.75 / 2.0 m 用 R1.25 绕得回来、直线助跑照常 2 m，2.4 m 起 R1.5 也放得下
        car = line_car(12.5, 0., 0.)
        spec = replace(LINE, points=((12., 0.), (0., 0.)))
        for space in (1.0, 1.5):
            with self.subTest(space=space):
                with self.assertRaises(ValueError) as cm:
                    plan(car, space, spec)
                self.assertIn('放不下圆弧', str(cm.exception))
        for space in (1.75, 2., 2.5, 3.):
            with self.subTest(space=space):
                secs, _, how = plan(car, space, spec)
                self.assertEqual(how, 'arc')
                no_pivot(self, secs, car)
        secs, entry_m, _ = plan(car, 2., spec)
        pieces = secs[0].path.pieces
        self.assertAlmostEqual(min(1 / abs(p.kappa) for p in pieces if abs(p.kappa) > 1e-9), 1.25)
        last_arc = max(i for i, p in enumerate(pieces) if abs(p.kappa) > 1e-9)
        self.assertAlmostEqual(secs[0].paint_from_m - sum(p.length for p in pieces[:last_arc + 1]), rp.RUN_UP_M, places=6)

    def test_no_pivot_needed_keeps_straight_reverse(self):
        # 车已在线上顺着线、喷头在线头后面：本来就不转（直接倒到助跑起点），照旧，不绕大圈；空地没填也行
        for space in (0., 5.):
            with self.subTest(space=space):
                car = line_car(-.8)
                secs, entry_m, how = plan(car, space)
                self.assertEqual(how, 'straight')
                self.assertEqual([s.direction for s in secs], [-1, 1])
                no_pivot(self, secs, car)

    def test_on_the_line_behind_the_runup_start_drives_straight_in(self):
        # 车在线头后方、顺着线（车头差不到 5°）：一段直着开进来，空地没填也行
        for yaw in (0., math.radians(3.)):
            with self.subTest(yaw=yaw):
                car = rp.axle_for((-4., 0.), 0., *NOZZLE) + (yaw,)
                secs, entry_m, how = plan(car, 0.)
                self.assertEqual(how, 'straight')
                self.assertEqual([s.direction for s in secs], [1])
                self.assertAlmostEqual(entry_m, secs[0].paint_from_m, places=6)

    def test_facing_away_or_off_the_line_goes_on_an_arc(self):
        # 原来要原地转的：在线上但车头反了、在线头后方但车头偏 30°、喷头在起点附近但车头横着（原来原地转再倒车助跑）
        cars = (rp.axle_for((-4., 0.), math.pi, *NOZZLE) + (math.pi,),
                rp.axle_for((-4., 0.), math.radians(30.), *NOZZLE) + (math.radians(30.),),
                line_car(.2, .1, math.pi / 2))
        for car in cars:
            with self.subTest(car=car):
                with self.assertRaises(ValueError):
                    plan(car, 0.)
                secs, entry_m, how = plan(car, 3.)
                self.assertEqual(how, 'arc')
                self.assertTrue(all(s.direction == 1 for s in secs))
                self.assertEqual(len(secs), 1)
                no_pivot(self, secs, car)
                worst, g = rearmost(secs, entry_m)
                self.assertGreaterEqual(worst, min(g - 3., rp._body_min_along(*car, BODY, 0., *line_car(0.)[:2])) - 1e-6)

    def test_circle_entry_is_an_arc_too(self):
        circle = rp.RouteSpec(kind='circle', center=(0., 0.), radius=3., cruise=.6)
        for car in ((5., 4., math.pi / 2), (0., 0., 0.), (-3.5, .2, 1.)):
            with self.subTest(car=car):
                with self.assertRaises(ValueError):
                    plan(car, 0., circle)
                secs, entry_m, how = plan(car, 2., circle)
                self.assertEqual(how, 'arc')
                self.assertTrue(all(s.direction == 1 for s in secs))
                no_pivot(self, secs, car)


if __name__ == '__main__':
    unittest.main()
