"""进线可选圆弧（2026-11-19 用户：「可选圆弧还是原地，圆弧就是可以设置后方有多远的空地，最低要1.5m才允许圆弧」）。

entry_turn='arc' 时只往前、按 Dubins 圆弧开到助跑起点车头已顺线，不停车接着助跑；整条接入车身不退到「车在助跑起点时
车尾」后方 entry_space_m 以外，不到 1.5 m 或哪个半径都放不下就照原地转进线。"""
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


def plan(car, space, turn='arc'):
    info = {}
    secs, entry_m = rp.plan_current(replace(LINE, entry_turn=turn, entry_space_m=space), car, .556,
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
        old = rp.parse_spec(dict(kind='path', points=[[0, 0], [5, 0]], closed=False))
        self.assertEqual((old.entry_turn, old.entry_space_m), ('pivot', 0.))      # 旧 app / 旧路线：原地
        spec = rp.parse_spec(replace(LINE, entry_turn='arc', entry_space_m=2.).to_dict())
        self.assertEqual((spec.entry_turn, spec.entry_space_m), ('arc', 2.))
        with self.assertRaises(ValueError):
            rp.parse_spec(dict(LINE.to_dict(), entry_turn='loop'))
        with self.assertRaises(ValueError):
            rp.parse_spec(dict(LINE.to_dict(), entry_space=-1))


class ArcEntry(unittest.TestCase):
    SIDE = line_car(-1., 3., -math.pi / 2)        # 线头侧面 3 m、车头对着线

    def test_below_minimum_space_stays_pivot(self):
        for space in (0., 1.0, 1.49):
            self.assertEqual(plan(self.SIDE, space)[2], 'pivot', space)

    def test_arc_with_1_5_m_only_forward_no_stop_and_stays_inside_space(self):
        secs, entry_m, how = plan(self.SIDE, 1.5)
        self.assertEqual(how, 'arc')
        self.assertEqual([s.direction for s in secs], [1])                  # 一段往前，中间不停不转
        self.assertAlmostEqual(secs[-1].paint_from_m, entry_m, places=6)
        radii = [1 / abs(p.kappa) for p in secs[0].path.pieces if abs(p.kappa) > 1e-9]
        self.assertGreaterEqual(min(radii), 1.5 - 1e-9)
        worst, g = rearmost(secs, entry_m)
        self.assertGreaterEqual(worst, g - 1.5 - 1e-6)
        # 直线助跑至少 ENTRY_ARC_MIN_STRAIGHT_M
        pieces = secs[0].path.pieces
        last_arc = max(i for i, p in enumerate(pieces) if abs(p.kappa) > 1e-9)
        arc_end = sum(p.length for p in pieces[:last_arc + 1])
        self.assertGreaterEqual(secs[0].paint_from_m - arc_end + 1e-6, rp.ENTRY_ARC_MIN_STRAIGHT_M)

    def test_more_space_uses_bigger_radius_and_full_runup(self):
        secs, entry_m, how = plan(self.SIDE, 5.)
        self.assertEqual(how, 'arc')
        radii = [1 / abs(p.kappa) for p in secs[0].path.pieces if abs(p.kappa) > 1e-9]
        self.assertGreater(min(radii), 1.5)
        worst, g = rearmost(secs, entry_m)
        self.assertGreaterEqual(worst, g - 5. - 1e-6)

    def test_turning_round_at_the_far_end_needs_more_space(self):
        # 刚画完同一条线、停在线尾外朝外：往前绕回来要更大的空地，1.5 m 放不下照原地，3 m 能圆弧
        car = line_car(12.5, 0., 0.)
        spec = replace(LINE, points=((12., 0.), (0., 0.)))
        def how(space):
            info = {}
            rp.plan_current(replace(spec, entry_turn='arc', entry_space_m=space), car, .556,
                            body=BODY, nozzle_x=NOZZLE, info=info)
            return info['entry_turn']
        self.assertEqual(how(1.5), 'pivot')
        self.assertEqual(how(3.), 'arc')

    def test_no_pivot_needed_keeps_straight_reverse(self):
        # 车已在线上顺着线、在线头后面：原地方式本来就不转（直接倒到助跑起点），圆弧模式也照用，不绕大圈
        secs, entry_m, how = plan(line_car(-.8), 5.)
        self.assertEqual(how, 'pivot')
        self.assertEqual([s.direction for s in secs], [-1, 1])

    def test_pivot_mode_unchanged_by_space(self):
        a = plan(self.SIDE, 0., 'pivot')
        b = plan(self.SIDE, 5., 'pivot')
        self.assertEqual(a[2], 'pivot')
        self.assertAlmostEqual(a[1], b[1])


if __name__ == '__main__':
    unittest.main()
