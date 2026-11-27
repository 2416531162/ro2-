"""进线按 app 选的（2026-11-27 用户「原地转也先不要删除，根据app实际选择」；没带按圆弧）。

圆弧进线：只往前、按 Dubins 圆弧开到助跑起点车头已顺线，不停车接着助跑；半径就是 app 设的转弯半径（turn_radius，
「进线直径按照app设置的来」）；后方空地 app 没设就当无限长、不检查（「后方空地默认无限长，除非app自己设置了」），设了只用来
挑圆弧接上线的位置（少一点直线助跑），放不下也照走（「车子又看不见，选择了圆弧，就直接走呀」）。车已顺着线、不用原地转
（直着开进来 / 直着倒到助跑起点）照旧。原地转进线（entry_turn='pivot'）见 test_entry_choice。"""
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
        self.assertEqual(rp.ENTRY_TURNS, ('pivot', 'arc'))
        self.assertEqual((rp.RouteSpec().entry_turn, rp.RouteSpec().turn, rp.RouteSpec().entry_space_m), ('arc', 'arc', math.inf))
        old = rp.parse_spec(dict(kind='path', points=[[0, 0], [5, 0]], closed=False))
        self.assertEqual((old.entry_turn, old.turn, old.entry_space_m), ('arc', 'arc', math.inf))   # 旧 app / 旧路线
        self.assertEqual(old.to_dict()['entry_space'], 0.)                       # 无限长存成 0（没设），读回来还是无限
        self.assertEqual(rp.parse_spec(old.to_dict()).entry_space_m, math.inf)
        pivot = rp.parse_spec(dict(LINE.to_dict(), entry_turn='pivot', turn='pivot'))   # app 选了原地转：照它
        self.assertEqual((pivot.entry_turn, pivot.turn), ('pivot', 'pivot'))
        spec = rp.parse_spec(replace(LINE, entry_space_m=2.).to_dict())
        self.assertEqual(spec.entry_space_m, 2.)
        for bad in (dict(entry_turn='loop'), dict(entry_space=-1)):
            with self.assertRaises(ValueError):
                rp.parse_spec(dict(LINE.to_dict(), **bad))
        import field_planner
        self.assertEqual(field_planner.Settings.__dataclass_fields__['turn'].default, 'arc')


def arc_radii(secs):
    return [1 / abs(p.kappa) for s in secs for p in s.path.pieces[:-1] if abs(p.kappa) > 1e-9]


def runup_straight(sec):
    pieces = sec.path.pieces
    last = max(i for i, p in enumerate(pieces) if abs(p.kappa) > 1e-9 and p.s0 < sec.paint_from_m)
    return sec.paint_from_m - (pieces[last].s0 + pieces[last].length)


class ArcEntry(unittest.TestCase):
    SIDE = line_car(-1., 3., -math.pi / 2)        # 线头侧面 3 m、车头对着线
    FAR = line_car(12.5, 0., 0.)                  # 刚画完同一条线、停在线尾外朝外
    BACK = replace(LINE, points=((12., 0.), (0., 0.)))

    def go(self, car, spec=LINE, **kw):
        info = {}
        secs, entry_m = rp.plan_current(replace(spec, **kw), car, .556, body=BODY, nozzle_x=NOZZLE, info=info)
        return secs, entry_m, info['entry_turn']

    def test_uses_the_app_turn_radius_one_forward_section(self):
        for radius in (1.25, 2., 3.):
            for car, spec in ((self.SIDE, LINE), (self.FAR, self.BACK)):
                with self.subTest(radius=radius, car=car):
                    secs, entry_m, how = self.go(car, spec, turn_radius=radius)
                    self.assertEqual(how, 'arc')
                    self.assertEqual([s.direction for s in secs], [1])          # 一段往前，中间不停不转
                    self.assertAlmostEqual(secs[0].path.pose(0.)[2], car[2])     # 从车头方向起步，不原地转
                    self.assertTrue(all(abs(r - radius) < 1e-9 for r in arc_radii(secs)))
                    self.assertAlmostEqual(secs[0].paint_from_m, entry_m, places=6)

    def test_no_space_set_never_refuses_and_keeps_full_runup(self):
        # 后方空地没设（无限长）：不检查，直线助跑照常 RUN_UP_M；离线头近、车头横着、反着的都直接走圆弧
        cars = (self.SIDE, line_car(-4., 0., math.pi), line_car(-4., 0., math.radians(30.)), line_car(.2, .1, math.pi / 2))
        for car in cars:
            with self.subTest(car=car):
                secs, _, how = self.go(car, turn_radius=1.25)
                self.assertEqual(how, 'arc')
                self.assertEqual(len(secs), 1)
                self.assertAlmostEqual(runup_straight(secs[0]), rp.RUN_UP_M, places=6)

    def test_space_set_moves_the_join_and_still_goes_when_short(self):
        # 设了后方空地：放得下照常（2 m 助跑）；放不下把圆弧接上线的位置往线头挪（直线助跑最少 ENTRY_ARC_MIN_STRAIGHT_M），
        # 挪到头也放不下照走（R2 线尾掉头往回，车身甩到助跑起点车尾后 3.5 m，填 2 m 也走）
        secs, entry_m, _ = self.go(self.SIDE, turn_radius=1.25, entry_space_m=3.)
        self.assertAlmostEqual(runup_straight(secs[0]), rp.RUN_UP_M, places=6)
        secs, entry_m, _ = self.go(self.SIDE, turn_radius=1.25, entry_space_m=2.)
        self.assertAlmostEqual(runup_straight(secs[0]), rp.ENTRY_ARC_MIN_STRAIGHT_M, places=6)
        worst, g = rearmost(secs, entry_m)
        self.assertGreaterEqual(worst, g - 2. - 1e-6)
        secs, _, how = self.go(self.FAR, self.BACK, turn_radius=2., entry_space_m=2.)
        self.assertEqual(how, 'arc')
        self.assertAlmostEqual(runup_straight(secs[0]), rp.ENTRY_ARC_MIN_STRAIGHT_M, places=6)

    def test_no_pivot_needed_keeps_straight_in(self):
        # 车已顺着线：喷头在线头后面直接倒到助跑起点、在助跑起点后方直着开进来，圆弧、原地都一样
        for turn in ('arc', 'pivot'):
            with self.subTest(turn=turn):
                secs, _, how = self.go(line_car(-.8), entry_turn=turn)
                self.assertEqual((how, [s.direction for s in secs]), ('straight', [-1, 1]))
                secs, _, how = self.go(rp.axle_for((-4., 0.), 0., *NOZZLE) + (math.radians(3.),), entry_turn=turn)
                self.assertEqual((how, [s.direction for s in secs]), ('straight', [1]))

    def test_pivot_selected_keeps_pivot_entry(self):
        secs, _, how = self.go(self.SIDE, entry_turn='pivot')
        self.assertEqual(how, 'pivot')
        self.assertFalse(arc_radii(secs))

    def test_circle_follows_the_selection(self):
        circle = rp.RouteSpec(kind='circle', center=(0., 0.), radius=3., cruise=.6, turn_radius=1.5)
        for car in ((5., 4., math.pi / 2), (0., 0., 0.), (-3.5, .2, 1.)):
            with self.subTest(car=car):
                secs, _, how = self.go(car, circle)
                self.assertEqual(how, 'arc')
                self.assertTrue(all(s.direction == 1 for s in secs))
                self.assertAlmostEqual(secs[0].path.pose(0.)[2], car[2])
                self.assertNotEqual(self.go(car, circle, entry_turn='pivot')[2], 'arc')

    def test_corner_turn_follows_the_selection(self):
        rect = rp.RouteSpec(kind='path', points=((0., 0.), (6., 0.), (6., 4.), (0., 4.)), closed=True, cruise=.8,
                            turn_radius=1.5)
        car = rp.axle_for((-4., 0.), 0., *NOZZLE) + (0.,)
        for turn, arcs in (('arc', 3), ('pivot', 0)):
            with self.subTest(turn=turn):
                info = {}
                rp.plan_current(replace(rect, turn=turn), car, .556, body=BODY, nozzle_x=NOZZLE, info=info)
                self.assertEqual(info['arc_turns'], arcs)


if __name__ == '__main__':
    unittest.main()
