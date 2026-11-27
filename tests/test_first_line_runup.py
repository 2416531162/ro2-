"""重启后第一条（任务层没有上一条线卫星拟合好的 IMU 换算）助跑里按卫星轨迹定角度
（2026-11-22 用户「就没有办法直接第一条就很好吗」→「做，先仿真」；11-23 用户「不能靠加长助跑解决问题」：
原来第一条加长到 4 m 的助跑删掉，助跑照常 RUN_UP_M）：

- 规划：给了 first_line 的连线路线，接入、助跑和不给时一模一样，只在接上助跑的第一条段上记 first_line；圆不记；
- 任务层：差速车直线 IMU 为主 + 卫星拟合、ImuAlignment 没有记忆 → 开始作业规划给 first_line，有记忆 → 不给；
- 拟合（只在记了 first_line 的这条上，ImuFirstGuide.no_memory）：助跑把推算往卫星拉时不动卡尔曼状态（角度差不被当成位置吃掉）；
  助跑里角度按 fit_runup_beta_rate_deg 用上。有记忆的线照旧。

规划、任务层这两组测的是开机第一条不空跑时（第一段不够 LEARN_M 长等，route_plan._learn_loop 返回 None）的退路：
关掉空跑（LEARN_M = 0）来测（11-25 空跑改成原地掉头开回起点，不再要后方空地，平常总会空跑）。"""
from dataclasses import replace
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT), str(ROOT / 'tests')]
import route_plan as rp
import user_rules as rules
from line_cascade import ImuAlignment, ImuFirstConfig, ImuFirstGuide
from line_mission import IdleTracker, LineMission
from live_cruise import label_sections
from path_tracker import PathConfig, SectionTracker, differential_config
from robot_core.kinematics import ChassisGeometry

NOZZLE = (.24, -.4575)
MIN_R = .556
GEO = ChassisGeometry(drive='differential', track_m=.675, diff_min_radius_m=.5)
LINE = rp.RouteSpec(kind='path', points=((0., 0.), (12., 0.)), closed=False, cruise=.8)
FAR = (5., 4., math.pi / 2)          # 车在线旁边：开到助跑起点、原地转正再助跑


def paint_section(sections):
    """助跑 + 第一段所在的那一段（规划标成要喷的第一段）。"""
    for sec in sections:
        if getattr(sec, 'field_role', None) == 'paint':
            return sec
    raise AssertionError('没有助跑段')


def geometry(sections):
    return [(sec.direction, tuple(round(v, 9) for v in sec.path.pose(0.)),
             tuple((round(p.length, 9), round(p.kappa, 9)) for p in sec.path.pieces)) for sec in sections]


class DryRun(unittest.TestCase):
    """开机第一条空跑（11-25 用户「空跑不要圆弧了，直线原地掉头去起点」）：助跑起点 G 开过线头 LEARN_M，原地掉头、
    直线开回 G，原地转正再助跑画线；全程往前、不走弧线。11-27 起后方空地够时改回圆弧绕回（ArcDryRun），
    后方空地没填（0）时照这样。"""

    def test_turn_back_in_place_to_the_start(self):
        for car in (FAR, (-6., .05, 0.), (14., 1., math.pi)):
            with self.subTest(car=car):
                info = {}
                sections, entry = rp.plan_current(LINE, car, MIN_R, nozzle_x=NOZZLE, info=info, first_line=True)
                _, plain_entry = rp.plan_current(LINE, car, MIN_R, nozzle_x=NOZZLE, first_line=False)
                self.assertEqual(info['learn_m'], rp.LEARN_M)
                k = [getattr(s, 'learn', False) for s in sections].index(True)
                learn, back, paint = sections[k], sections[k + 1], sections[k + 2]
                h0 = paint.path.pose(0.)[2]
                g = paint.path.pose(0.)[:2]
                # 空跑：从 G 顺线开过线头 LEARN_M
                self.assertAlmostEqual(learn.path.length, rp.RUN_UP_M + rp.LEARN_M, places=6)
                self.assertLess(math.dist(learn.path.pose(0.)[:2], g), 1e-6)
                self.assertLess(abs(rp.wrap(learn.path.pose(0.)[2] - h0)), 1e-6)
                # 原地掉头、直线开回 G
                self.assertLess(abs(abs(rp.wrap(back.path.pose(0.)[2] - h0)) - math.pi), 1e-6)
                self.assertAlmostEqual(back.path.length, rp.RUN_UP_M + rp.LEARN_M, places=6)
                self.assertLess(math.dist(back.path.pose(back.path.length)[:2], g), 1e-6)
                for sec in sections[k:k + 2]:
                    self.assertEqual(sec.direction, 1)                                   # 不倒车
                    self.assertTrue(all(abs(p.kappa) < 1e-12 for p in sec.path.pieces))   # 不走弧线
                self.assertFalse(any(s.continuous_after for s in sections))             # 掉头处停车原地转
                self.assertFalse(any(getattr(s, 'first_line', False) for s in sections))  # 画线那段有空跑学好的
                self.assertAlmostEqual(entry - plain_entry, 2 * (rp.LEARN_M + rp.RUN_UP_M), places=6)


class ArcDryRun(unittest.TestCase):
    """空跑改回圆弧绕回（11-27 用户「空跑改成圆弧」）：后方空地够（≥ ENTRY_ARC_MIN_SPACE_M）时空跑过线头 LEARN_M 后只往前
    按圆弧绕回 G、不停车接着助跑；路线选了圆弧进线时开到 G 也走圆弧，全程不停车、不原地转。放不下照原地掉头开回。"""
    BODY = (.911, .604, .39)
    CARS = ((-5., 3., 0.), (22.5, 0., 0.), (-1., 1., 0.), FAR)

    def plan(self, car, space, turn='arc'):
        info = {}
        spec = replace(LINE, points=((0., 0.), (22., 0.)), entry_turn=turn, entry_space_m=space)
        sections, entry = rp.plan_current(spec, car, MIN_R, body=self.BODY, nozzle_x=NOZZLE, info=info, first_line=True)
        return sections, entry, info

    def test_arc_entry_and_arc_return_never_stop(self):
        for car in self.CARS:
            with self.subTest(car=car):
                sections, entry, info = self.plan(car, rules.ENTRY_ARC_MIN_SPACE_M)
                self.assertEqual(info['learn_turn'], 'arc')
                self.assertEqual(info['learn_m'], rp.LEARN_M)
                self.assertTrue(all(s.direction == 1 for s in sections))                 # 不倒车
                self.assertTrue(all(s.continuous_after for s in sections[:-1]))          # 一路不停车、不原地转
                k = [getattr(s, 'learn', False) for s in sections].index(True)
                paint = sections[-1]
                g, h0 = paint.path.pose(0.)[:2], paint.path.pose(0.)[2]
                # 空跑那段开过线头 LEARN_M（G 往前 RUN_UP_M + LEARN_M），之后的圆弧绕回到 G、车头顺线
                learn = sections[k]
                end = learn.path.pose(learn.path.length)
                along = (end[0] - g[0]) * math.cos(h0) + (end[1] - g[1]) * math.sin(h0)
                self.assertAlmostEqual(along, rp.RUN_UP_M + rp.LEARN_M, delta=1e-6)
                self.assertLess(abs(rp.wrap(end[2] - h0)), 1e-6)
                back = sections[-2]
                self.assertLess(math.dist(back.path.pose(back.path.length)[:2], g), 1e-6)
                self.assertLess(abs(rp.wrap(back.path.pose(back.path.length)[2] - h0)), 1e-6)
                radii = [1 / abs(p.kappa) for s in sections for p in s.path.pieces if abs(p.kappa) > 1e-9]
                self.assertGreaterEqual(min(radii), rules.ENTRY_ARC_RADII[-1] - 1e-9)
                self.assertFalse(getattr(paint, 'first_line', False))                    # 画线那段有空跑学好的

    def test_pivot_entry_still_returns_on_an_arc(self):
        # 进线选原地转：开到 G 照原来（停一次），空跑后照样圆弧绕回、不停车接着助跑
        sections, _, info = self.plan(FAR, rules.ENTRY_ARC_MIN_SPACE_M, turn='pivot')
        self.assertEqual(info['learn_turn'], 'arc')
        k = [getattr(s, 'learn', False) for s in sections].index(True)
        self.assertTrue(all(s.continuous_after for s in sections[k:-1]))

    def test_no_space_falls_back_to_turning_in_place(self):
        sections, _, info = self.plan(FAR, 0.)
        self.assertEqual(info['learn_turn'], 'pivot')
        self.assertFalse(any(s.continuous_after for s in sections))


class NoDryRun(unittest.TestCase):
    """关掉开机第一条空跑（route_plan.LEARN_M = 0），测不空跑时的退路。"""

    def setUp(self):
        self._learn_m, rp.LEARN_M = rp.LEARN_M, 0.0

    def tearDown(self):
        rp.LEARN_M = self._learn_m


class Planning(NoDryRun):
    CARS = (FAR, rp.axle_for((-.8, 0.), 0., *NOZZLE) + (0.,), (-6., .05, 0.), (14., 1., math.pi))

    def test_first_line_only_marks_the_section(self):
        # 接入、助跑长度都不变（不加长助跑）：只在接上助跑的第一条段上记 first_line
        for car in self.CARS:
            with self.subTest(car=car):
                a, ea = rp.plan_current(LINE, car, MIN_R, nozzle_x=NOZZLE)
                b, eb = rp.plan_current(LINE, car, MIN_R, nozzle_x=NOZZLE, first_line=True)
                self.assertEqual(geometry(a), geometry(b))
                self.assertAlmostEqual(ea, eb)
                self.assertFalse(any(getattr(sec, 'first_line', False) for sec in a))
                self.assertEqual([getattr(sec, 'first_line', False) for sec in b],
                                 [sec is paint_section(b) for sec in b])
                self.assertFalse(any(hasattr(sec, 'runup_m') for sec in b))

    def test_runup_stays_run_up_m(self):
        sections, _ = rp.plan_current(LINE, FAR, MIN_R, nozzle_x=NOZZLE, first_line=True)
        sec = paint_section(sections)
        head = sec.path.pose(rp.RUN_UP_M)
        self.assertAlmostEqual(head[0] - sec.path.pose(0.)[0], rp.RUN_UP_M, places=6)
        self.assertFalse(hasattr(rp, 'FIRST_RUN_UP_M'))
        car = rp.axle_for((-.8, 0.), 0., *NOZZLE) + (0.,)       # 喷头在线头后 0.8 m：照旧倒 1.2 m 到助跑起点
        reverse = [s for s in rp.plan_current(LINE, car, MIN_R, nozzle_x=NOZZLE, first_line=True)[0] if s.direction < 0]
        self.assertEqual(len(reverse), 1)
        self.assertAlmostEqual(reverse[0].path.length, rp.RUN_UP_M - .8, places=6)

    def test_plan_marks_too(self):
        a, _ = rp.plan(LINE, FAR, MIN_R, nozzle_x=NOZZLE)
        b, _ = rp.plan(LINE, FAR, MIN_R, nozzle_x=NOZZLE, first_line=True)
        self.assertEqual(geometry(a), geometry(b))
        self.assertTrue(paint_section(b).first_line)

    def test_circle_unchanged(self):
        circle = rp.RouteSpec(kind='circle', center=(0., 0.), radius=3., cruise=.6)
        a, ea = rp.plan_current(circle, FAR, MIN_R, nozzle_x=NOZZLE)
        b, eb = rp.plan_current(circle, FAR, MIN_R, nozzle_x=NOZZLE, first_line=True)
        self.assertAlmostEqual(ea, eb)
        self.assertEqual(geometry(a), geometry(b))
        self.assertFalse(any(getattr(sec, 'first_line', False) for sec in b))

    def test_tracker_reads_the_mark(self):
        for flag in (False, True):
            sections, entry = rp.plan_current(LINE, FAR, MIN_R, nozzle_x=NOZZLE, first_line=flag)
            label_sections(sections, entry, .8)
            route = SectionTracker(sections, differential_config(PathConfig(cruise_mps=.8)), GEO,
                                   constant_straight_speed=True)
            tr = route.trackers[sections.index(paint_section(sections))]
            self.assertEqual(tr.runup_distance_m, rp.RUN_UP_M)
            self.assertEqual(tr.first_line, flag)
            self.assertFalse(any(t.first_line for t in route.trackers if t is not tr))


class Memory(NoDryRun):
    def test_remembered(self):
        a = ImuAlignment()
        self.assertFalse(a.remembered())
        a.feed_imu(10., 0.)
        token = a.begin_line()
        a.update_line(token, 10., .01)                          # 只借来的（没经卫星拟合）不算
        self.assertFalse(a.remembered())
        a.feed_imu(10.05, 0.)
        a.update_line(token, 10.05, .01, math.radians(.05))
        a.feed_imu(10.1, 0.)
        self.assertTrue(a.remembered())
        a.feed_imu(15., 0.)                                     # 中间 IMU 断过：转角接不上，也没有
        self.assertFalse(a.remembered())

    def test_too_old(self):
        a = ImuAlignment()
        t = 10.
        a.feed_imu(t, 0.)
        a.update_line(a.begin_line(), t, .01, math.radians(.05))
        while t < 10. + a.cfg.max_age_s - 1.:
            t += .05
            a.feed_imu(t, 0.)
        self.assertTrue(a.remembered())
        while t < 10. + a.cfg.max_age_s + 1.:
            t += .05
            a.feed_imu(t, 0.)
        self.assertFalse(a.remembered())

    def test_mission_flags_first_line(self):
        car = SimpleNamespace(to_geodetic=lambda x, y: (30. + y * 1e-5, 120. + x * 1e-5),
                              to_local=lambda lat, lon: ((lon - 120.) / 1e-5, (lat - 30.) / 1e-5))
        pose = SimpleNamespace(x=FAR[0], y=FAR[1], yaw=FAR[2])
        nozzle = dict(forward=NOZZLE[0], right=-NOZZLE[1])

        def planned(m):
            sections, _, tracker, _ = m._plan_route(LINE, pose, rp.plan_current, nozzle, .8)
            first = paint_section(sections)
            self.assertEqual(rp.RUN_UP_M, tracker.trackers[sections.index(first)].runup_distance_m)
            return getattr(first, 'first_line', False), tracker.trackers[sections.index(first)].first_line
        with tempfile.TemporaryDirectory() as folder:
            m = LineMission(car, SimpleNamespace(), IdleTracker(GEO),
                            route_path=str(Path(folder) / 'route.json'),
                            path_config=differential_config(PathConfig(cruise_mps=.8)), nozzle_x=NOZZLE)
            self.assertTrue(m.first_line_no_memory())
            self.assertEqual(planned(m), (True, True))
            a = m._imu_alignment
            a.feed_imu(5., 0.)
            a.update_line(a.begin_line(), 5., .01, math.radians(.05))
            a.feed_imu(5.05, 0.)
            self.assertFalse(m.first_line_no_memory())
            self.assertEqual(planned(m), (False, False))
            m.path_cfg = replace(m.path_cfg, straight_imu_enabled=False)  # 不用 IMU 为主：没有记忆这回事
            a.obs = a.pending = None
            self.assertFalse(m.first_line_no_memory())
            self.assertFalse(planned(m)[0])
            self.assertNotIn('first_run_in', m.status()['route_limits'])


def guide(no_memory=True):
    g = ImuFirstGuide(replace(ImuFirstConfig(), rtk_trim=False, rtk_fit=True))
    g.offset, g.locked, g.source = 0., True, 'learned'
    g.no_memory = no_memory
    return g


class RunupFit(unittest.TestCase):
    def drive(self, g, slope_deg, metres, pull, fast=True, step=.01):
        """IMU 推算直走（rel 0），卫星横偏按 slope_deg 拉开：起步换算错了 slope_deg。"""
        u = 0.
        while u < metres - 1e-9:
            u += step
            measured = math.tan(math.radians(slope_deg)) * u
            g.update(measured, 0., step, .02, (0., 0.), True, fast=fast)
            if pull:
                g.pull_to(measured, 1. - math.exp(-step / .3))
        return g

    def test_pull_keeps_angle_information(self):
        g = self.drive(guide(), 1.8, 2.0, pull=True)
        self.assertGreater(math.degrees(g._fit['x'][1]), 1.3)   # 原来每拍把卡尔曼的 c 一起拉，只学到 ~0.6°
        self.assertLess(abs(g.e - math.tan(math.radians(1.8)) * 2.0), .005)   # 推算照样靠到卫星

    def test_runup_applies_angle_fast(self):
        g = self.drive(guide(), 1.8, 2.5, pull=True)
        self.assertGreater(math.degrees(g.beta), 1.2)          # 0.5°/m 时到这里最多 ~0.9°

    def test_painting_rate_unchanged(self):
        g = self.drive(guide(), 1.8, 2.5, pull=False, fast=False)
        self.assertLess(math.degrees(g.beta), 1.0)              # 开喷以后照旧限速（直线度）

    def test_lines_with_memory_unchanged(self):
        g = self.drive(guide(no_memory=False), 1.8, 2.5, pull=True)
        self.assertLess(math.degrees(g._fit['x'][1]), 1.0)      # 照旧：助跑的拉近连拟合一起挪
        self.assertLess(math.degrees(g.beta), 1.0)              # 照旧限速
        g = self.drive(guide(no_memory=False), 1.8, 2.5, pull=False)
        self.assertGreater(math.degrees(g._fit['x'][1]), 1.3)   # 拟合看得出来，
        self.assertLess(math.degrees(g.beta), 1.0)              # 助跑里也照旧按 0.5°/m 用上


class TrackerSetsMemory(unittest.TestCase):
    def run_line(self, first_line, remembered):
        from test_reverse_runup_handoff import fused, tracker
        tr = tracker()
        tr.first_line = first_line
        a = ImuAlignment()
        a.feed_imu(.9, 0.)
        if remembered:
            a.update_line(a.begin_line(), .9, .01, math.radians(.05))
        a.feed_imu(1., 0.)
        self.assertEqual(a.remembered(), remembered)
        tr.share_imu_alignment(a)
        tr.sync(1., (1., 0., 0.))
        tr._straight_first(fused(tr), .02, .3, 0., tr.path.pieces[0], 0., 0.)
        self.assertIsNotNone(tr._straight_guide.token)
        return tr._straight_guide.no_memory

    def test_planned_first_line_counts_as_no_memory(self):
        # 接入那段直线自己拟合好记进了记忆，规划时定了加长助跑的第一条照样按没有记忆
        self.assertTrue(self.run_line(True, True))
        self.assertTrue(self.run_line(True, False))

    def test_other_lines_unchanged(self):
        # 不是规划出的第一条（有记忆的线、场地作业）照旧，没有记忆也不改
        self.assertFalse(self.run_line(False, True))
        self.assertFalse(self.run_line(False, False))


if __name__ == '__main__':
    unittest.main()
