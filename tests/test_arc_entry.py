"""进线按 app 选的（2026-11-27 用户「原地转也先不要删除，根据app实际选择」；没带按圆弧）。

圆弧进线：只往前、按 Dubins 圆弧开到助跑起点车头已顺线，不停车接着助跑；半径就是 app 设的转弯半径（turn_radius，
「进线直径按照app设置的来」）；圆弧后直线助跑默认 2 m、app 可调（entry_runup，11-28 用户「保持助跑2m」「这个助跑要在app可调」）；
掉头尽量在场内一侧转（「圆弧在内场转，外场极限转完刚好接上2m助跑」）：几种走法里挑往线头外伸得最少的。后方空地不再影响
走法、放不下也照走（「车子又看不见，选择了圆弧，就直接走呀」）。车已顺着线、不用原地转
（直着开进来 / 直着倒到助跑起点）照旧。原地转进线（entry_turn='pivot'）见 test_entry_choice。"""
from dataclasses import replace
from pathlib import Path
import math
import sys
import unittest
from unittest import mock

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
        spec = rp.parse_spec(replace(LINE, entry_space_m=2., entry_runup_m=3.).to_dict())
        self.assertEqual((spec.entry_space_m, spec.entry_runup_m), (2., 3.))
        self.assertEqual(old.entry_runup_m, rp.RUN_UP_M)                         # app 没带：2 m
        for bad in (dict(entry_turn='loop'), dict(entry_space=-1), dict(entry_runup=.5), dict(entry_runup=6)):
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


_DUBINS_ALL = rp.dubins_all


def shortest_only(start, goal, radius):
    """只留最短的走法（对照：原来按最短走）。"""
    return [min(_DUBINS_ALL(start, goal, radius), key=lambda w: sum(v for v, _ in w))]


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

    def test_runup_follows_the_app_and_never_refuses(self):
        # 圆弧后直线助跑按 app 的 entry_runup（默认 2 m）；离线头近、车头横着、反着的都直接走圆弧，后方空地设多少都不挡
        cars = (self.SIDE, line_car(-4., 0., math.pi), line_car(-4., 0., math.radians(30.)), line_car(.2, .1, math.pi / 2))
        for car in cars:
            for runup in (None, 1., 3.):
                with self.subTest(car=car, runup=runup):
                    kw = {} if runup is None else dict(entry_runup_m=runup)
                    secs, _, how = self.go(car, turn_radius=1.25, entry_space_m=.5, **kw)
                    self.assertEqual(how, 'arc')
                    self.assertEqual(len(secs), 1)
                    self.assertAlmostEqual(runup_straight(secs[0]), runup or rp.RUN_UP_M, places=6)

    def test_turns_on_the_field_side(self):
        # 掉头尽量在场内一侧：车身往线头外伸得不比最短走法多；最远到助跑起点外一个半径再加车身（物理下限）。
        # 画完线尾掉头往回最明显：直径 4 m 最短走法伸出线头外约 6.6 m，现在约 4.8 m（接入长一些）
        def outside(secs):
            sec = secs[0]
            x0, y0, h0 = sec.path.pose(sec.paint_from_m)
            nx, ny = rp.nozzle_at(x0, y0, h0, *rp.nozzle_point(NOZZLE))
            return -min(rp._body_min_along(*sec.path.pose(min(i * .02, sec.paint_from_m)), BODY, h0, nx, ny)
                        for i in range(int(sec.paint_from_m / .02) + 1))
        cars = ((self.SIDE, LINE), (line_car(3., 1.5, math.pi), LINE), (line_car(3., 3., -math.pi / 2), LINE),
                (self.FAR, self.BACK))
        for radius in (1.25, 2.):
            for car, spec in cars:
                with self.subTest(radius=radius, car=car):
                    secs, _, _ = self.go(car, spec, turn_radius=radius)
                    got = outside(secs)
                    with mock.patch.object(rp, 'dubins_all', shortest_only):
                        shortest = outside(self.go(car, spec, turn_radius=radius)[0])
                    self.assertLessEqual(got, shortest + 1e-6)
                    self.assertLess(got, rp.RUN_UP_M + NOZZLE[0] + radius + .65)
        secs, _, _ = self.go(self.FAR, self.BACK, turn_radius=2.)
        with mock.patch.object(rp, 'dubins_all', shortest_only):
            shortest = outside(self.go(self.FAR, self.BACK, turn_radius=2.)[0])
        self.assertLess(outside(secs), shortest - 1.)

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


class LearnOnTheArc(unittest.TestCase):
    """进线圆弧最后 ENTRY_ARC_LEARN_M 米就给后面那条直线攒 IMU 换算样本（融合车头已按 IMU + RTK 位移对好时），转上直线
    不清掉、接着攒（11-28 用户「1.2开始」）。换算照样只从这条线开头的融合车头学。"""

    def tracker(self):
        from types import SimpleNamespace as NS
        from path_tracker import PathConfig, SectionTracker, differential_config
        from robot_core.kinematics import ChassisGeometry
        from live_cruise import label_sections
        secs, entry_m = rp.plan_current(replace(LINE, turn_radius=2.), ArcEntry.SIDE, .556, body=BODY, nozzle_x=NOZZLE)
        label_sections(secs, entry_m, .8)
        route = SectionTracker(secs, differential_config(PathConfig(cruise_mps=.8)),
                               ChassisGeometry(drive='differential', track_m=.675, diff_min_radius_m=.5),
                               constant_straight_speed=True)
        tr = route.trackers[0]
        tr.set_nozzle(*NOZZLE)
        tr.st.state = 'RUNNING'
        ahead = next(p for p in tr.path.pieces if p.straight and p.s0 <= tr.paint_from_m < p.s0 + p.length)
        return tr, ahead, NS

    def run_to(self, tr, NS, s_from, s_to, calibrated, t0=0.):
        t, s = t0, s_from
        while s < s_to:
            s += .01
            t += .02
            x, y, yaw = tr.path.pose(s)
            tr.sync(t, (t, yaw - 1., math.radians(2.)))
            tr.st.s = s
            pose = NS(x=x, y=y, yaw=yaw, speed=.5, yaw_rate=0., usable_for_control=True, quality='RTK_FIXED',
                      rtk_trusted=True, reasons=(), correction_xy=(0., 0.), correction_yaw=0., heading_calibrated=calibrated)
            tr._straight_control(pose, .02, .5, 0.)
        return t

    def test_samples_start_on_the_last_part_of_the_arc(self):
        import path_tracker
        self.assertEqual(path_tracker.ENTRY_ARC_LEARN_M, .5)
        tr, ahead, NS = self.tracker()
        g = tr._straight_guide
        t = self.run_to(tr, NS, ahead.s0 - 2., ahead.s0 - .6, True)
        self.assertIsNone(g.offset)                                   # 圆弧前面还不攒
        t = self.run_to(tr, NS, ahead.s0 - .6, ahead.s0 - .01, True, t)
        n = g.offset_n
        self.assertGreater(n, 30)                                     # 最后 0.5 m 攒上了
        self.assertAlmostEqual(g.offset, 1., places=6)
        self.run_to(tr, NS, ahead.s0 - .01, ahead.s0 + .2, True, t)
        self.assertTrue(g.locked)                                     # 转上直线接着攒、没清掉，很快锁住
        self.assertEqual(g.source, 'learned')

    def test_not_before_the_heading_is_calibrated(self):
        tr, ahead, NS = self.tracker()
        self.run_to(tr, NS, ahead.s0 - .6, ahead.s0 - .01, False)
        self.assertIsNone(tr._straight_guide.offset)                 # 车头还靠测向：照原来，转上直线才学


if __name__ == '__main__':
    unittest.main()
