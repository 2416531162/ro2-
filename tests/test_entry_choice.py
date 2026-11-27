"""接入走法（2026-10-07 用户）：定点画线车在点后面时直接往后倒一点，不要原地转一圈再转回来；
画圆从接入最省事的地方开始（直接走最近的接入口）。2026-11-27 用户「进线只走圆弧，删掉进线的原地转」：
本来要原地转的接入改走圆弧（只往前、一段不停），后方空地不够不开始；顺线直倒、直开进来照旧。只做纯规划测试。"""
from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'radar_system'), str(ROOT)]
import route_plan as rp
from live_cruise import label_sections

NOZZLE = (.24, -.4575)          # 实车：后轴前 0.24 m、右 0.4575 m
MIN_R = .556


def moves(sections, car):
    """[(原地转角度°), ('前'/'倒', 米)…]：原地转只记 >5°（跟踪器 pivot_min）。"""
    out, yaw = [], car[2]
    for sec in sections:
        x, y, h = sec.path.pose(0.)
        head = h if sec.direction > 0 else rp.wrap(h+math.pi)
        turn = math.degrees(rp.wrap(head-yaw))
        if abs(turn) > 5:
            out.append(turn)
        out.append(('前' if sec.direction > 0 else '倒', sec.path.length))
        end = sec.path.pose(sec.path.length)[2]
        yaw = end if sec.direction > 0 else rp.wrap(end+math.pi)
    return out


def pivots(m):
    return [abs(v) for v in m if not isinstance(v, tuple)]


def approach(m):
    """去掉助跑起点前「转正后直线倒回 RUNUP_BACK_M」那一下（2026-10-11 起步扭头，见 route_plan.RUNUP_BACK_M），
    只看怎么开到转正点：倒车接入的距离限制、往前开还是倒过去，都按原来的规则。"""
    k = [i for i, v in enumerate(m) if isinstance(v, tuple)]
    if (len(k) >= 2 and m[k[-2]][0] == '倒' and abs(m[k[-2]][1] - rp.RUNUP_BACK_M) < 1e-6
            and m[k[-1]][0] == '前' and k[-1] == k[-2] + 1):
        return m[:k[-2]] + m[k[-1]:]
    return m


def line_car(q, side=0., yaw=0.):
    """喷头在 (q, side)、车头 yaw 时的后轴位姿。"""
    return rp.axle_for((q, side), yaw, *NOZZLE) + (yaw,)


LINE = rp.RouteSpec(kind='path', points=((0., 0.), (10., 0.)), closed=False, cruise=.8)


def only_forward_arc(test, sections, car):
    """进线只走圆弧：一段往前、一下都不原地转。"""
    m = moves(sections, car)
    test.assertEqual(pivots(m), [], m)
    test.assertEqual(len(sections), 1, m)
    test.assertEqual(sections[0].direction, 1, m)


class LineEntry(unittest.TestCase):
    def plan(self, car, space=0., spec=LINE):
        sections, entry_m = rp.plan_current(replace(spec, entry_space_m=space), car, MIN_R, nozzle_x=NOZZLE)
        return moves(sections, car), entry_m

    def sections(self, car, space, spec=LINE):
        return rp.plan_current(replace(spec, entry_space_m=space), car, MIN_R, nozzle_x=NOZZLE)[0]

    def test_behind_start_reverses_instead_of_turning_round(self):
        # 喷头在线头后 0.8 / 1.2 m、车头对着线：原来转 180° 开一点再转 180°，现在直接倒到助跑起点
        for q in (-.8, -1.2):
            m, _ = self.plan(line_car(q))
            self.assertEqual(pivots(m), [], (q, m))
            self.assertEqual(m[0][0], '倒', (q, m))
            self.assertAlmostEqual(m[0][1], rp.RUN_UP_M+q, places=6)
            self.assertEqual(m[1][0], '前')

    def test_slightly_off_line_goes_round_on_an_arc(self):
        # 原来小转一下倒到助跑起点、再转正；倒过去要原地转的改走圆弧（往前绕回来），后方空地没填不开始
        car = line_car(-1., .3)
        with self.assertRaises(ValueError):
            self.plan(car)
        only_forward_arc(self, self.sections(car, rp.ENTRY_ARC_MIN_SPACE_M), car)

    def test_past_start_reverses_back(self):
        m, _ = self.plan(line_car(.3))
        self.assertEqual(pivots(m), [], m)
        self.assertEqual(m[0][0], '倒')

    def test_long_reverse_not_used(self):
        # 倒车超过 REVERSE_ENTRY_MAX_M 不倒：往前按圆弧绕回来（车在线外侧，起点在背后 5 m）
        car = line_car(4., 1.5, 0.)
        with self.assertRaises(ValueError):
            self.plan(car)
        only_forward_arc(self, self.sections(car, rp.ENTRY_ARC_MIN_SPACE_M), car)

    def test_runup_keeps_the_existing_reverse_distance_limit(self):
        # 2026-11-19 助跑 3.5 → 2 m：喷头在线头前 0.8 m 时倒回助跑起点只要 2.8 m，在 REVERSE_ENTRY_MAX_M 以内就直线倒回去；
        # 再往前（倒车超过上限）照旧原地掉头开过去
        m, _ = self.plan(line_car(.8))
        back = [v[1] for v in approach(m) if isinstance(v, tuple) and v[0] == '倒']
        self.assertTrue(back and max(back) <= rp.REVERSE_ENTRY_MAX_M + 1e-9, m)
        self.assertEqual(pivots(m), [], m)
        car = line_car(rp.REVERSE_ENTRY_MAX_M - rp.RUN_UP_M + .2)
        with self.assertRaises(ValueError):
            self.plan(car)
        only_forward_arc(self, self.sections(car, rp.ENTRY_ARC_MIN_SPACE_M), car)

    def test_on_extension_facing_out_goes_on_an_arc(self):
        # 车已在线的延长线上、助跑起点后方 2 m，车头横着：原来原地转正一次直接开进来；现在一段圆弧开进来，
        # 车就在助跑起点后面、要往后绕，后方空地 2 m 放不下（R1.25 要 3 m），不开始
        x, y, _ = line_car(-4.)          # 后轴在后轴线的延长线上（车头朝 +y）
        car = (x, y, math.pi/2)
        with self.assertRaises(ValueError) as cm:
            self.plan(car, 2.)
        self.assertIn('放不下圆弧', str(cm.exception))
        only_forward_arc(self, self.sections(car, 3.), car)

    def test_far_entry_still_goes_forward_when_reverse_saves_little(self):
        # 起点正后方、喷头在侧面：倒过去和开过去转的角度一样，照旧往前开
        car = (0., 0., 0.)
        spec = rp.RouteSpec(kind='path', points=((-1.0, 0.), (-6., 0.)), closed=False)
        only_forward_arc(self, self.sections(car, 3., spec), car)


class ReverseRunup(unittest.TestCase):
    """倒车接入后的助跑（线头不能变差）：助跑仍 1.5 m，起步锁车头 0.6 → 1.0 m（只普通路线的接入）。"""
    def tracker(self, car, reverse_runup=True, spec=LINE):
        import os
        os.environ.setdefault('RK3588_ROBOT_CONFIG', str(ROOT/'robot_core/robot_gen5.json'))
        from path_tracker import PathConfig, SectionTracker, differential_config
        from robot_core.kinematics import ChassisGeometry
        geo = ChassisGeometry(drive='differential', track_m=.675, diff_min_radius_m=.5)
        secs, entry_m = rp.plan_current(spec, car, MIN_R, nozzle_x=NOZZLE)
        label_sections(secs, entry_m, .8)
        return SectionTracker(secs, differential_config(PathConfig(cruise_mps=.8)), geo, pivot_rate=.8, pivot_accel=1.,
                              constant_straight_speed=True, reverse_runup=reverse_runup), secs

    def test_reverse_entry_keeps_runup(self):
        secs, entry_m = rp.plan_current(LINE, line_car(-.8), MIN_R, nozzle_x=NOZZLE)
        label_sections(secs, entry_m, .8)
        self.assertEqual([s.direction for s in secs], [-1, 1])
        self.assertAlmostEqual(secs[1].paint_from_m, rp.RUN_UP_M, places=6)

    def test_longer_start_hold_after_reverse_entry_only(self):
        from path_tracker import REVERSE_HOLD_M
        tr, _ = self.tracker(line_car(-.8))
        self.assertAlmostEqual(tr.trackers[1].cfg.start_hold_m, REVERSE_HOLD_M)
        plain, _ = self.tracker(line_car(-.8), reverse_runup=False)        # 场地作业等：照旧
        self.assertAlmostEqual(plain.trackers[1].cfg.start_hold_m, .6)
        fwd, secs = self.tracker(line_car(-rp.RUN_UP_M-.5))                # 不倒车的接入：照旧
        self.assertTrue(all(s.direction > 0 for s in secs))
        self.assertAlmostEqual(fwd.trackers[0].cfg.start_hold_m, .6)

    def test_corner_reverse_in_route_unchanged(self):
        # 路线中间拐角的倒车（原地转 + 倒车回拐角）后的助跑照旧
        rect = rp.RouteSpec(kind='path', points=((0., 0.), (6., 0.), (6., 4.), (0., 4.)), closed=True, cruise=.8)
        tr, secs = self.tracker(line_car(-3.), spec=rect)
        first = next(k for k, s in enumerate(secs) if s.field_role == 'paint')
        later = [k for k in range(first+1, len(secs)) if secs[k].direction > 0 > secs[k-1].direction]
        self.assertTrue(later)
        for k in later:
            self.assertAlmostEqual(tr.trackers[k].cfg.start_hold_m, .6)


class CircleEntry(unittest.TestCase):
    def setUp(self):
        self.spec = rp.RouteSpec(kind='circle', center=(0., 0.), radius=1.5, clockwise=True, laps=1, cruise=.8,
                                 entry_space_m=2.)

    def plan(self, car):
        sections, entry_m = rp.plan_current(self.spec, car, MIN_R, nozzle_x=NOZZLE)
        return sections, moves(sections, car), entry_m

    def test_outside_near_tangent_goes_on_an_arc_with_a_straight_runup(self):
        # 车在圆外 1 m、车头顺着画的方向：原来转一下顺切线开上圆；现在一段圆弧开上切线，直线助跑至少
        # ENTRY_ARC_MIN_STRAIGHT_M 再进圆过渡；后方空地没填不开始
        car = (0., -2.5, math.pi)
        with self.assertRaises(ValueError):
            rp.plan_current(replace(self.spec, entry_space_m=0.), car, MIN_R, nozzle_x=NOZZLE)
        sections, m, entry_m = self.plan(car)
        only_forward_arc(self, sections, car)
        pieces = sections[0].path.pieces
        ramp = len(pieces) - rp.CIRCLE_TRANSITION_STEPS - 2      # 进圆过渡第一小段（后面是过渡、整圈、切线开出）
        self.assertGreaterEqual(pieces[ramp - 1].length, rp.ENTRY_ARC_MIN_STRAIGHT_M - 1e-9)
        self.assertLess(entry_m, 12.)

    def test_far_outside_goes_on_one_arc(self):
        for yaw in (math.pi/2, -math.pi/2):
            sections, m, _ = self.plan((0., -4.5, yaw))
            only_forward_arc(self, sections, (0., -4.5, yaw))

    def test_nozzle_on_circle_no_turn_round(self):
        # 喷头已在圆上、车头顺着画的方向：原来「转 38° 倒 2.8 m 转 9°」；现在倒一点就进圆，不掉头
        car = rp.axle_for((0., -1.5), math.pi, *NOZZLE) + (math.pi,)
        sections, m, entry_m = self.plan(car)
        self.assertLessEqual(len(pivots(m)), 1, m)
        self.assertLess(sum(pivots(m)), 90., m)
        # 进圆过渡让直线和圆相接处往回挪约半个过渡（CIRCLE_TRANSITION_M / 2），倒车最多多这么多
        self.assertLessEqual(sum(v[1] for v in m if isinstance(v, tuple) and v[0] == '倒'), rp.RUN_UP_M + rp.CIRCLE_TRANSITION_M/2 + 1e-6, m)

    def test_start_still_paints_full_circle(self):
        # 换了起点仍是整圈：起喷后到停喷沿后轴圆走满一圈 + 多喷
        for car in ((0., -2.5, math.pi), (0., -4.5, math.pi/2), (0., 0., 0.)):
            sections, _, entry_m = self.plan(car)
            label_sections(sections, entry_m, .8)       # 同开始作业（line_mission）
            sec = sections[-1]
            r = rp.circle_axle_radius(1.5, NOZZLE, MIN_R, -1.)
            self.assertAlmostEqual(sec.paint_to_m - sec.paint_from_m, 2*math.pi*r + rp.CIRCLE_OVERLAP_M*r/1.5, places=6)
            self.assertAlmostEqual(entry_m, sum(s.path.length for s in sections[:-1]) + sec.paint_from_m, places=6)


if __name__ == '__main__':
    unittest.main()
