#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""差速底盘（Gen5：两后轮差速、前面自由万向轮，能原地转）：共享配置 drive 段、手动摇杆原地转、
连线路线喷头压线 + 急弯原地转向（与 app PivotRoute 同一规则）、分段跟踪的原地转向、融合在原地转时不丢航向。

阿克曼车（默认配置）的行为不变，由原有测试覆盖；这里只在显式给差速几何时走差速分支。
"""
import json
import math
import os
import sys
import tempfile
import unittest
from dataclasses import replace
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system', 'tools'))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, ROOT)

import app_link as al  # noqa: E402
import path_sim  # noqa: E402
import route_plan as rp  # noqa: E402
from line_tracker import LineConfig, LineTracker  # noqa: E402
from path_tracker import Path, PathConfig, PathTracker, Section, SectionTracker, differential_config  # noqa: E402
from robot_core.config import DEFAULT_PATH, load_profile  # noqa: E402
from robot_core.kinematics import ChassisGeometry, differential_wheels  # noqa: E402

GEN5 = os.path.join(ROOT, 'robot_core', 'robot_gen5.json')
DIFF = ChassisGeometry(drive='differential', track_m=0.675, diff_min_radius_m=0.5, pivot_rate_rad_s=0.8,
                       pivot_accel_rad_s2=1.0)
NOZZLE = 0.30
RADIUS = DIFF.min_turn_radius_m / PathConfig().kappa_margin          # 与 line_mission 相同
SQUARE = [(0.0, 0.0), (8.0, 0.0), (8.0, 6.0), (0.0, 6.0)]


def pose(x=0.0, y=0.0, yaw_deg=0.0, speed=0.0, yaw_rate=0.0, usable=True):
    return SimpleNamespace(x=x, y=y, yaw=math.radians(yaw_deg), speed=speed, yaw_rate=yaw_rate,
                           usable_for_control=usable)


def seg_dist(p, a, b):
    ax, ay, bx, by = *a, *b
    t = max(0.0, min(1.0, ((p[0] - ax) * (bx - ax) + (p[1] - ay) * (by - ay)) / ((bx - ax) ** 2 + (by - ay) ** 2)))
    return math.hypot(p[0] - ax - t * (bx - ax), p[1] - ay - t * (by - ay))


def nozzle_track(sections, step=0.01):
    """各段按规划开时喷头走过的点（原地转向时喷头绕后轴转，不算画线）。"""
    out = []
    for d, start, pieces in sections:
        x, y, yaw = start
        for length, k in pieces:
            n = max(1, int(length / step))
            for i in range(n + 1):
                px, py, pyaw = rp.advance(x, y, yaw, length * i / n, k)
                head = pyaw if d > 0 else pyaw + math.pi
                out.append((px + NOZZLE * math.cos(head), py + NOZZLE * math.sin(head)))
            x, y, yaw = rp.advance(x, y, yaw, length, k)
    return out


def off_line(points, track):
    """喷头轨迹离连线最远多少米（终点多走的那段不算）。"""
    (ax, ay), (bx, by) = points[-2], points[-1]
    ux, uy = bx - ax, by - ay
    return max(min(seg_dist(p, a, b) for a, b in zip(points, points[1:])) for p in track
               if (p[0] - bx) * ux + (p[1] - by) * uy <= 0.0)


def bend(deg, leg=6.0):
    """直线 leg 米后左转 deg 度（负为右转）再走 leg 米。"""
    a = math.radians(deg)
    return [(0.0, 0.0), (leg, 0.0), (leg + leg * math.cos(a), leg * math.sin(a))]


class DiffCar:
    """理想差速车：指令即速度，车速为 0 时照样按角速度原地转。"""

    def __init__(self, x=0.0, y=0.0, yaw_deg=0.0):
        self.x, self.y, self.yaw, self.v, self.w = x, y, math.radians(yaw_deg), 0.0, 0.0

    def pose(self):
        return pose(self.x, self.y, math.degrees(self.yaw), self.v, self.w)

    def step(self, vx, wz, dt):
        self.v, self.w = vx, wz
        self.x += vx * dt * math.cos(self.yaw + wz * dt / 2)
        self.y += vx * dt * math.sin(self.yaw + wz * dt / 2)
        self.yaw += wz * dt


class TestProfile(unittest.TestCase):

    def test_default_car_is_ackermann_and_gen5_is_differential(self):
        self.assertEqual(load_profile()['drive']['type'], 'ackermann')
        gen5 = load_profile(GEN5)
        self.assertEqual(gen5['drive']['type'], 'differential')
        self.assertEqual((gen5['geometry']['front_m'], gen5['geometry']['rear_m'], gen5['geometry']['track_m']),
                         (0.911, 0.604, 0.675))
        self.assertEqual(gen5['drive']['nozzle_x_m'], 0.30)

    def test_old_site_profile_without_drive_section_loads_as_ackermann(self):
        old = json.loads(DEFAULT_PATH.read_text())
        old.pop('drive')
        with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as fh:
            json.dump(old, fh)
        try:
            self.assertEqual(load_profile(fh.name)['drive']['type'], 'ackermann')
        finally:
            os.unlink(fh.name)

    def test_bad_drive_values_are_rejected(self):
        for change in (dict(type='tank'), dict(pivot_rate_rad_s=5.0), dict(nozzle_x_m=2.0), dict(min_radius_m=0)):
            bad = json.loads(DEFAULT_PATH.read_text())
            bad['drive'].update(change)
            with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as fh:
                json.dump(bad, fh)
            try:
                with self.assertRaises(ValueError, msg=change):
                    load_profile(fh.name)
            finally:
                os.unlink(fh.name)

    def test_geometry(self):
        self.assertFalse(ChassisGeometry().differential)
        self.assertTrue(DIFF.differential)
        self.assertEqual(DIFF.min_turn_radius_m, 0.5)
        left, right = differential_wheels(0.0, 0.8, 0.675)
        self.assertAlmostEqual(left, -0.27)
        self.assertAlmostEqual(right, 0.27)


class TestManualStick(unittest.TestCase):
    TIERS = {'low': 0.3, 'med': 0.6, 'high': 1.0}

    def stick(self, throttle, steer, tier='high'):
        return al.resolve_stick(throttle, steer, tier, self.TIERS, 0.6, DIFF)

    def test_sideways_turns_in_place(self):
        self.assertEqual(self.stick(0.0, 1.0), (0.0, 0.8, 0.0))       # 往左打：原地左转
        self.assertEqual(self.stick(0.0, -1.0), (0.0, -0.8, 0.0))
        self.assertEqual(self.stick(0.0, 1.0, 'low'), (0.0, 0.24, 0.0))   # 慢档转得也慢
        self.assertEqual(self.stick(0.01, 0.015), (0.0, 0.0, 0.0))     # 死区（2%）
        self.assertEqual(self.stick(0.029, 0.0), (0.029, 0.0, 0.0))    # app 刚出死区的指令：2.9 cm/s 慢慢挪

    def test_forward_arcs_and_reverse_keeps_the_steering_feel(self):
        vx, wz, deg = self.stick(1.0, 0.5, 'med')
        self.assertEqual((vx, wz, deg), (0.6, 0.24, 0.0))
        vx, wz, _ = self.stick(-1.0, 0.5, 'med')
        self.assertEqual((vx, wz), (-0.36, -0.24))                     # 倒车往左打：车尾往左

    def test_ackermann_still_ignores_pure_sideways(self):
        self.assertEqual(al.resolve_stick(0.0, 1.0, 'med', self.TIERS, 0.6, ChassisGeometry()), (0.0, 0.0, 0.0))

    def test_hello_reports_drive_and_nozzle(self):
        car = al.build_hello(load_profile(GEN5), name='gen5')['car']
        self.assertEqual((car['drive'], car['nozzle_x_m'], car['min_turn_radius_m']), ('differential', 0.30, 0.5))
        self.assertEqual(al.build_hello(load_profile(), name='old')['car']['drive'], 'ackermann')


class TestMotionAuthorityPivot(unittest.TestCase):
    """控制权仲裁：手动指令车速 0 原来一律当停车；差速车（allow_pivot）角速度不为 0 是原地转。"""

    def authority(self, allow):
        sys.path.insert(0, os.path.join(ROOT, 'wheeltec_protocol'))
        from motion_authority import MotionAuthority
        a = MotionAuthority(allow_pivot=allow)
        a.health(True, True, 1.0)
        a.health(True, True, 1.31)
        return a

    @staticmethod
    def manual(a, vx, wz, t):
        return a.submit('manual', dict(vx=vx, wz=wz, stamp=t, epoch=a.epoch), t, t)

    def test_turn_in_place_is_manual_drive_only_on_the_differential_car(self):
        old = self.authority(False)
        self.assertTrue(self.manual(old, 0.0, 0.5, 2.0))
        self.assertEqual(old.mode, 'IDLE')                  # 阿克曼车：还是停车
        diff = self.authority(True)
        self.assertTrue(self.manual(diff, 0.0, 0.5, 2.0))
        self.assertEqual(diff.mode, 'MANUAL')
        diff.health(True, True, 2.1)
        diff.health(True, True, 2.45)
        self.assertTrue(self.manual(diff, 0.0, 0.5, 2.5))
        out = diff.output(2.5)
        self.assertEqual((out.vx, out.wz), (0.0, 0.5))
        self.assertTrue(self.manual(diff, 0.0, 0.0, 2.6))     # 两个都为 0 照样是停车
        out = diff.output(2.6)
        self.assertEqual((out.vx, out.wz), (0.0, 0.0))
        self.assertEqual(diff.mode, 'MANUAL')                # 回中：就地停、还在手动，再推马上走
        diff.output(3.0)
        self.assertEqual((diff.mode, diff.reason), ('IDLE', 'command_timeout'))   # 松手不再发：超时回空闲


class TestPivotPlan(unittest.TestCase):

    def test_corner_reverse_keeps_nozzle_geometry_and_covers_a_short_next_leg(self):
        # 喷头仍在后轴前 0.30 m：转完退 1.20 m（2026-10-01 用户由 0.5 → 0.8 → 1.2 m），喷头在线头前 0.90 m 助跑，
        # 前进重新经过线头。下一边只有 0.10 m，不能把它减成负长度或漏掉。
        sections, corners = rp._pivot_cover([(0.0, 0.0), (2.0, 0.0), (2.0, 0.1)], RADIUS, NOZZLE)
        self.assertEqual(corners, 1)
        self.assertEqual([s[0] for s in sections], [1, -1, 1])
        reverse, forward = sections[1:]
        self.assertAlmostEqual(sum(p[0] for p in reverse[2]), 1.20)
        self.assertAlmostEqual(forward[1][0], 2.0)
        self.assertAlmostEqual(forward[1][1], -1.20)
        self.assertAlmostEqual(forward[1][2], math.pi / 2)
        self.assertAlmostEqual(forward[1][1] + NOZZLE, -0.90)
        x, y, yaw = forward[1]
        for length, kappa in forward[2]:
            x, y, yaw = rp.advance(x, y, yaw, length, kappa)
        self.assertAlmostEqual(x + NOZZLE * math.cos(yaw), 2.0)
        self.assertAlmostEqual(y + NOZZLE * math.sin(yaw), 0.1 + rp.VERTEX_PASS_M)
        track = nozzle_track(sections, step=0.001)
        for target in [(2.0, 0.0), (2.0, 0.1)]:
            self.assertLess(min(math.dist(target, p) for p in track), 0.001)

    def test_square_nozzle_passes_every_edge_and_corner(self):
        spec = rp.parse_spec(dict(kind='path', points=[list(p) for p in SQUARE], closed=True))
        sections, corners = rp._pivot_route_sections(spec, RADIUS, NOZZLE)
        self.assertEqual(corners, 3)                                   # 最后回到起点不再转
        self.assertEqual(rp.pivot_count(sections), 3)
        self.assertEqual([d for d, _, _ in sections], [1, -1, 1, -1, 1, -1, 1])
        track = nozzle_track(sections)
        drawn = SQUARE + [SQUARE[0]]
        for a, b in zip(drawn, drawn[1:]):
            n = int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.01)
            for i in range(n + 1):
                q = (a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n)
                self.assertLess(min(math.hypot(q[0] - x, q[1] - y) for x, y in track), 0.006, q)
        summary = rp.cover_summary(spec, RADIUS, None, nozzle_x=NOZZLE)
        self.assertEqual((summary['corners'], summary['pivots'], summary['cusps']), (3, 3, 6))

    def test_nozzle_between_the_wheels_only_turns_in_place(self):
        sections, corners = rp._pivot_cover(SQUARE, RADIUS, 0.0)
        self.assertEqual(corners, 2)
        self.assertEqual({d for d, _, _ in sections}, {1})             # 不倒车
        self.assertEqual(rp.pivot_count(sections), 2)

    def test_gentle_bend_is_an_arc_without_stopping(self):
        # 喷头在两后轮中间、转 10°：按缓弯半径圆弧转过去离拐角 < 1 cm，不停车
        sections, corners = rp._pivot_cover(bend(10.0, 5.0), RADIUS, 0.0)
        self.assertEqual((corners, len(sections)), (0, 1))
        self.assertTrue(any(abs(k) > 0 for _, k in sections[0][2]))

    def test_gentle_bends_keep_the_nozzle_on_the_line(self):
        # 喷头在后轴前 0.30 m：缓弯时后轴被喷头拖着走，喷头不甩出去（原来后轴按圆弧转，20° 甩出约 7 cm）
        limit = math.degrees(math.atan(NOZZLE / RADIUS))
        for deg in (2.0, 10.0, 20.0, limit - 0.5, -(limit - 0.5)):
            sections, corners = rp._pivot_cover(bend(deg), RADIUS, NOZZLE)
            self.assertEqual((corners, len(sections)), (0, 1), deg)
            self.assertLessEqual(max(abs(k) for _, k in sections[0][2]), 1.0 / RADIUS + 1e-9, deg)
            self.assertLess(off_line(bend(deg), nozzle_track(sections, step=0.002)), 0.0015, deg)
            # 拖过拐角后收回到边上：最后一片是直线，和图纸那条边重合
            x, y, yaw = sections[0][1]
            for length, k in sections[0][2][:-1]:
                x, y, yaw = rp.advance(x, y, yaw, length, k)
            self.assertEqual(sections[0][2][-1][1], 0.0)
            self.assertAlmostEqual(rp.wrap(yaw - math.radians(deg)), 0.0, places=9)
            self.assertLess(seg_dist((x, y), bend(deg)[1], bend(deg)[2]), 1e-3)
        # 超过 atan(喷头 / 缓弯半径) 后轴转不过来：停车原地转向、倒车让喷头回到拐角
        sections, corners = rp._pivot_cover(bend(limit + 0.5), RADIUS, NOZZLE)
        self.assertEqual(corners, 1)
        self.assertEqual([d for d, _, _ in sections], [1, -1, 1])

    def test_polylined_arc_is_pulled_through_without_stopping(self):
        # 罚球弧（R 9.15 m、106°）和角球弧（R 1 m）按弦高 5 mm 折线导出：每个点都是小角度缓弯，喷头全程压线
        for radius, deg, sag in ((9.15, 106.0, 0.005), (1.0, 90.0, 0.012)):
            step = 2 * math.acos(1 - sag / radius)
            n = math.ceil(math.radians(deg) / step)
            pts = [(radius * math.sin(math.radians(deg) * i / n), radius - radius * math.cos(math.radians(deg) * i / n))
                   for i in range(n + 1)]
            sections, corners = rp._pivot_cover(pts, RADIUS, NOZZLE)
            self.assertEqual((corners, len(sections)), (0, 1), radius)
            self.assertLess(off_line(pts, nozzle_track(sections, step=0.002)), 0.0015, radius)

    def test_entry_turns_drives_and_turns(self):
        route = rp.pivot_cover(SQUARE, RADIUS, NOZZLE)
        entry, entry_m = rp.pivot_entry(route[0], (-3.0, 2.0, math.radians(30)), NOZZLE)
        goal = (-NOZZLE - rp.ENTRY_LEAD_M, 0.0)
        self.assertAlmostEqual(entry_m, math.hypot(goal[0] + 3.0, goal[1] - 2.0) + rp.ENTRY_LEAD_M, places=9)
        first, second = entry[0], entry[1]
        self.assertAlmostEqual(first[1][2], math.atan2(-2.0, goal[0] + 3.0), places=9)   # 先转向对准
        self.assertEqual(second[1], (goal[0], goal[1], 0.0))                           # 再转到第一段方向
        # 已在第一段延长线上：直线开过去
        on_line, m = rp.pivot_entry(route[0], (-2.0, 0.01, 0.0), NOZZLE)
        self.assertEqual(len(on_line), 1)
        self.assertAlmostEqual(m, 2.0 - NOZZLE, places=9)

    def test_plan_uses_pivot_rules_only_when_asked(self):
        spec = rp.parse_spec(dict(kind='path', points=[[0, 0], [6, 0], [6, 5]], closed=False))
        diff, _ = rp.plan(spec, (-2.0, 0.0, 0.0), RADIUS, nozzle_x=NOZZLE)
        old, _ = rp.plan(spec, (-2.0, 0.0, 0.0), 1.72 / 0.9, body=(0.67, 0.08, 0.335))
        self.assertEqual([s.direction for s in diff], [1, -1, 1])
        # 差速车急弯原地转（倒 1.2 m 再进）仍比阿克曼车掉头补角短（倒 0.8 m 时短 1.4 m 以上，现在约 0.7 m）
        self.assertGreater(sum(s.path.length for s in old), sum(s.path.length for s in diff) + 0.5)


def run(tracker, car, until=200.0, dt=0.05, every=None):
    t, rows = 0.0, []
    while t < until and tracker.st.state in ('RUNNING', 'PAUSED'):
        vx, wz = tracker.update(car.pose(), dt)
        # 喷头相对正在跟踪那一段的横偏：后轴横偏 + 喷头在前方 NOZZLE 处因车头误差多偏的
        st = tracker.st
        rows.append((t, vx, wz, tracker.index, tracker.switching, tracker.status()['pivoting'],
                     st.e + NOZZLE * math.sin(st.heading_err)))
        if every:
            every(t, tracker, car)
        car.step(vx, wz, dt)
        t += dt
    return rows


def diff_tracker(sections, **kw):
    # 理想车替身指令立刻生效：不预估执行延迟（与 test_path_tracking 的 IDEAL 相同）
    cfg = replace(differential_config(PathConfig(cruise_mps=0.5)), delay_s=0.0)
    return SectionTracker(sections, cfg, DIFF, pivot_rate=0.8, pivot_accel=1.0, reverse_gain=1.0, **kw)


class TestPivotTracking(unittest.TestCase):

    def test_square_with_entry_turns_in_place_and_paints_on_the_line(self):
        spec = rp.parse_spec(dict(kind='path', points=[list(p) for p in SQUARE], closed=True))
        car = DiffCar(-3.0, 2.0, 30.0)
        sections, _ = rp.plan(spec, (car.x, car.y, car.yaw), RADIUS, nozzle_x=NOZZLE)
        tr = diff_tracker(sections)
        self.assertEqual(tr.check_start(car.pose()), (True, ''))       # 车头不对也能起步：先原地转
        self.assertTrue(tr.start(car.pose()))
        self.assertTrue(tr.switching)
        rows = run(tr, car)
        self.assertEqual(tr.st.state, 'DONE')
        # 原地转向时车速为 0、只有角速度，且不超过 0.8 rad/s
        turning = [r for r in rows if r[5]]
        self.assertTrue(turning)
        self.assertTrue(all(r[1] == 0.0 and abs(r[2]) <= 0.8 + 1e-9 for r in turning))
        starts = sum(1 for a, b in zip(rows, rows[1:]) if b[5] and not a[5])
        self.assertEqual(starts, 1 + tr.pivots)                         # 起步一次 + 每个急弯
        # 路线本身（接入段之后）喷头横偏不超过 1.5 cm（理想车）：原地转前停车差 ≤ 5 mm、转完车头差 ≤ 1°
        # （喷头在前 0.30 m，约 5 mm）是设计容差。急弯前喷头会开过拐角 NOZZLE 再原地转、倒车回来，
        # 装了喷阀以后这一小段要关喷
        painting = [r for r in rows if not r[4] and r[3] >= 1]
        self.assertLess(max(abs(r[6]) for r in painting), 0.015)

    def test_pause_while_turning_resumes_from_standstill(self):
        secs = [Section(1, Path((0.0, 0.0, 0.0), [(1.0, 0.0)])), Section(1, Path((1.0, 0.0, math.pi / 2), [(1.0, 0.0)]))]
        tr = diff_tracker(secs)
        car = DiffCar()
        tr.start(car.pose())
        paused = []

        def every(t, tracker, c):
            if tracker.status()['pivoting'] and not paused:
                paused.append(t)
                tracker.pause('operator')
                c.w = 0.0
                tracker.start(c.pose())          # 操作员马上继续
        run(tr, car, every=every)
        self.assertTrue(paused)
        self.assertEqual(tr.st.state, 'DONE')
        self.assertAlmostEqual(math.degrees(car.yaw), 90.0, delta=1.5)

    def test_car_that_does_not_turn_times_out(self):
        secs = [Section(1, Path((0.0, 0.0, math.pi / 2), [(1.0, 0.0)]))]
        tr = diff_tracker(secs, pivot_timeout_s=3.0)
        car = DiffCar()
        tr.start(car.pose())
        t = 0.0
        while t < 10.0 and tr.st.state == 'RUNNING':
            tr.update(car.pose(), 0.05)           # 车不动
            t += 0.05
        self.assertEqual((tr.st.state, tr.st.reason), ('ABORTED', 'pivot_timeout'))

    def test_ackermann_tracker_never_turns_in_place(self):
        secs = [Section(1, Path((0.0, 0.0, 0.0), [(1.0, 0.0)])), Section(1, Path((1.0, 0.0, math.pi / 2), [(1.0, 0.0)]))]
        tr = SectionTracker(secs, PathConfig())
        self.assertEqual(tr.pivots, 0)
        self.assertEqual(tr.check_start(pose(yaw_deg=90.0)), (False, 'heading_not_aligned'))


class TestFusedClosedLoop(unittest.TestCase):
    """原始传感器 → 真实融合 → 跟踪 → Gen5 差速底盘（path_sim.DifferentialChassis）。"""

    def test_square_route_with_real_fusion(self):
        spec = rp.parse_spec(dict(kind='path', points=[list(p) for p in SQUARE], closed=True))
        worst = 0.0
        for seed in (1, 2):
            r = path_sim.run_route(spec=spec, car=(-3.0, 1.0, math.radians(40)), chassis_model='differential', seed=seed)
            self.assertEqual(r['state'], 'DONE', r)
            self.assertEqual(r['pivots'], 4)
            worst = max(worst, r['cover_cm'])
        self.assertLess(worst, 3.0)                                     # 与阿克曼车同一标准

    def test_pivot_needs_the_differential_gyro_bounds(self):
        # 按阿克曼算陀螺上限（车速 0 → 只有 0.5°/s）会把原地转的陀螺全当尖峰丢掉，航向对不上，定位不可用
        orig = path_sim._fused_source
        try:
            path_sim._fused_source = lambda *a, differential=False, **k: orig(*a, differential=False, **k)
            spec = rp.parse_spec(dict(kind='path', points=[[0, 0], [4, 0], [4, 3]], closed=False))
            r = path_sim.run_route(spec=spec, car=(-2.0, 0.0, 0.0), chassis_model='differential', seed=1)
        finally:
            path_sim._fused_source = orig
        self.assertNotEqual(r['state'], 'DONE')


class TestPivotCircle(unittest.TestCase):
    """现场定点画圆（Gen5）：喷头画半径 R 的圆——后轴走 √(R²−d²) 的同心圆；车在圆心时原地转向接入；太小时说明原因。"""

    D = 0.30
    MIN = 0.556

    def circle(self, r=2.0, cw=False, laps=1, center=(1.0, 2.0)):
        return rp.parse_spec(dict(kind='circle', center=list(center), radius=r, clockwise=cw, laps=laps, cruise=0.5))

    def test_nozzle_is_exactly_on_the_drawn_circle(self):
        for cw in (False, True):
            spec = self.circle(cw=cw, laps=2)
            start, pieces = rp.loop_pieces(spec, self.MIN, self.D)
            path = Path(start, pieces)
            for f in (0.0, 0.2, 0.5, 0.8, 0.999):
                x, y, yaw = path.pose(path.length * f)
                nx, ny = x + self.D * math.cos(yaw), y + self.D * math.sin(yaw)
                self.assertAlmostEqual(math.hypot(nx - 1.0, ny - 2.0), 2.0, places=9)
            # 两圈再多走 5 cm，盖过起点
            self.assertAlmostEqual(path.length, 2 * 2 * math.pi * math.sqrt(4.0 - 0.09) + rp.VERTEX_PASS_M, places=9)
        # 不给喷头时仍是图纸上的圆（车载屏幕、续画用；阿克曼车后轴走 R）
        start, pieces = rp.loop_pieces(self.circle(), self.MIN)
        self.assertAlmostEqual(1 / pieces[0][1], 2.0)

    def test_entry_from_the_centre_pivots_then_drives_straight(self):
        spec = self.circle()
        car = (1.0 - self.D, 2.0, 0.0)                  # 喷头在圆心
        sections, entry_m = rp.plan(spec, car, self.MIN, nozzle_x=self.D)
        self.assertEqual(len(sections), 2)
        first, second = sections
        self.assertEqual([p.kappa for p in first.path.pieces], [0.0])            # 原地转向后直线开到起点前
        self.assertNotAlmostEqual(first.path.pose(0)[2], car[2])                 # 起步先原地转（跟踪器完成）
        self.assertEqual(second.path.pieces[0].kappa, 0.0)                         # 转正后开 ENTRY_LEAD_M（1 m）进圆
        self.assertAlmostEqual(second.path.pieces[-1].kappa, 1 / math.sqrt(4.0 - 0.09))
        self.assertLess(entry_m, 3.0 + (rp.ENTRY_LEAD_M - 0.5))     # 对正段 2026-09-30 由 0.5 m 加到 1 m
        length, problem = rp.route_length(spec, self.MIN, nozzle_x=self.D)
        self.assertIsNone(problem)
        self.assertAlmostEqual(length, second.path.pieces[-1].length)

    def test_too_small_circle_says_why(self):
        _, problem = rp.route_length(self.circle(r=0.6), self.MIN, nozzle_x=self.D)
        self.assertIn('后轴要转 0.52 m 的圆', problem)
        _, problem = rp.route_length(self.circle(r=0.6), self.MIN, nozzle_x=0.65)     # 喷头比半径还远
        self.assertIn('要大于喷头到后轴', problem)
        self.assertIsNone(rp.route_length(self.circle(r=0.7), self.MIN, nozzle_x=self.D)[1])

    def test_closed_loop_nozzle_draws_the_circle(self):
        # 底盘仿真闭环：车（喷头）停在圆心、车头斜着，原地转向接入后画圆；图纸圆上离喷头轨迹最远不超过 1 cm
        spec = self.circle(r=2.0, center=(0.0, 0.0))
        car = (-self.D * math.cos(0.5), -self.D * math.sin(0.5), 0.5)
        res = path_sim.run_route('circle', spec=spec, car=car, source='truth', chassis_model='differential')
        self.assertEqual(res['state'], 'DONE')
        self.assertLess(res['cover_cm'], 1.0)
        # 真实融合（RTK 噪声、双天线、IMU、轮速）、顺时针两圈、1 m 小圆
        res = path_sim.run_route('circle_small', spec=self.circle(r=1.0, cw=True, laps=2, center=(0.0, 0.0)),
                                 car=(-self.D, 0.0, 0.0), source='fused', chassis_model='differential')
        self.assertEqual(res['state'], 'DONE')
        self.assertLess(res['cover_cm'], 2.0)


class TestMission(unittest.TestCase):

    def setUp(self):
        from test_line_mission import SAMPLES, FakeFusion
        from line_mission import LineMission
        self.tmp = tempfile.TemporaryDirectory()
        self.fusion = FakeFusion()
        self.m = LineMission(self.fusion, SAMPLES, tracker=LineTracker(LineConfig(), DIFF), nozzle_x=NOZZLE,
                             route_path=os.path.join(self.tmp.name, 'route.json'))

    def tearDown(self):
        self.tmp.cleanup()

    def test_route_limits_and_cover_describe_the_differential_car(self):
        route = dict(kind='path', points=[[0, 0], [6, 0], [6, 5]], closed=False)
        self.assertTrue(self.m.command({'cmd': 'route_set', 'route': route}, 0.0)[0])
        st = self.m.status()
        self.assertEqual((st['route_limits']['drive'], st['route_limits']['nozzle_x']), ('differential', NOZZLE))
        self.assertAlmostEqual(st['route_limits']['min_radius'], RADIUS, places=3)
        self.assertEqual((st['route']['cover']['pivots'], st['route']['cover']['cusps']), (1, 2))

    def test_start_current_at_last_marked_point_runs_back_to_the_first(self):
        """2026-10-01 用户：标 A、B 两点后车停在 B 上点「从当前位置开始」：原地掉头、倒车，
        喷头从 B 一路画到 A。车按跟踪器设的执行延迟执行。"""
        from test_line_mission import FakeFusion
        route = dict(kind='path', points=[[0, 0], [6, 0]], closed=False, cruise=0.5)
        self.fusion.current = FakeFusion.make(6.0 - NOZZLE, 0.0, 0.0)  # 从 A 开过来，喷头停在 B
        ok, msg = self.m.command({'cmd': 'route_start_current', 'route': route}, 0.0)
        self.assertTrue(ok, msg)
        self.assertEqual([s.direction for s in self.m.tracker.sections], [-1, 1])
        t, sent, vx, wz, gears, nozzle = 0.0, [], 0.0, 0.0, set(), []
        while t < 200.0:
            out = self.m.tick(t, 'NAVIGATION')
            p = self.fusion.current
            if out[0] < 0:
                gears.add('reverse')
            sent.append((t + self.m.path_cfg.delay_s, out[0], out[1]))
            while sent and sent[0][0] <= t + 1e-9:
                _, vx, wz = sent.pop(0)
            yaw = p.yaw + wz * 0.05
            self.fusion.current = FakeFusion.make(p.x + vx * 0.05 * math.cos(yaw), p.y + vx * 0.05 * math.sin(yaw),
                                                  math.degrees(yaw), speed=vx)
            self.fusion.current.yaw_rate = wz
            q = self.fusion.current
            if vx > 0.05 and math.cos(q.yaw) < -0.99:                   # 往 A 画的时候喷头离线多远
                nozzle.append((q.x + NOZZLE * math.cos(q.yaw), q.y + NOZZLE * math.sin(q.yaw)))
            t += 0.05
            if self.m.phase == 'IDLE' and out[2] is False:
                break
        self.assertEqual(self.m.message, '路线完成')
        self.assertIn('reverse', gears)
        q = self.fusion.current
        self.assertLess(abs(q.x + NOZZLE * math.cos(q.yaw)), 0.1)          # 喷头画到 A 附近停
        on_line = [y for x, y in nozzle if 0.5 < x < 5.5]
        self.assertTrue(on_line)
        self.assertLess(max(abs(y) for y in on_line), 0.02)

    def test_route_start_turns_in_place_first(self):
        from test_line_mission import FakeFusion
        route = dict(kind='path', points=[[0, 0], [6, 0], [6, 5]], closed=False, cruise=0.5)
        self.fusion.current = FakeFusion.make(-2.0, 1.0, 120.0)        # 车头朝西北，不在线上
        ok, msg = self.m.command({'cmd': 'route_start', 'route': route}, 0.0)
        self.assertTrue(ok, msg)
        self.assertIn('原地转向', msg)
        self.assertEqual(self.m.tracker.pivot_rate, DIFF.pivot_rate_rad_s)
        self.assertEqual(self.m.path_cfg.delay_s, 0.30)            # 2026-10-01 驱动器 Kp 2.7 / Ki 45 后实测 0.28 s
        self.assertEqual(self.m.path_cfg.lat_accel_mps2, 0.20)       # 用户嫌转弯慢：09-30 0.08 → 0.12，10-01 → 0.20
        t, turned = 0.0, False
        sent, vx, wz = [], 0.0, 0.0                 # 车按跟踪器设的执行延迟执行指令
        while t < 200.0:
            out = self.m.tick(t, 'NAVIGATION')
            p = self.fusion.current
            if self.m.phase == 'RUNNING' and self.m.tracker.status().get('pivoting'):
                turned = True
                self.assertEqual(out[0], 0.0)
            sent.append((t + self.m.path_cfg.delay_s, out[0], out[1]))
            while sent and sent[0][0] <= t + 1e-9:
                _, vx, wz = sent.pop(0)
            yaw = p.yaw + wz * 0.05
            self.fusion.current = FakeFusion.make(p.x + vx * 0.05 * math.cos(yaw), p.y + vx * 0.05 * math.sin(yaw),
                                                  math.degrees(yaw), speed=vx)
            self.fusion.current.yaw_rate = wz
            t += 0.05
            if self.m.phase == 'IDLE' and out[2] is False:
                break
        self.assertTrue(turned)
        self.assertEqual(self.m.message, '路线完成')
        # 喷头停在终点后 5 cm 附近：按执行延迟预估，预估位置离终点 stop_tol_m 就停，
        # 实际停在线头前约 1.4 cm（离「线头后 5 cm」约 6.4 cm；原来按 0.1 s、车没延迟时约 4 cm）
        p = self.fusion.current
        self.assertLess(math.hypot(p.x + NOZZLE * math.cos(p.yaw) - 6.0, p.y + NOZZLE * math.sin(p.yaw) - 5.05), 0.07)



class TestStartCreep(unittest.TestCase):
    """差速车每段起步先慢走 0.3 m（≤ 0.1 m/s），万向轮在原地转向、换向后是横着的，一起步会把车身甩歪
    （2026-09-30 实车：车头甩 5–8°，线头 1 m 内偏 4–8 cm）。"""

    @staticmethod
    def drive(cfg, seconds=6.0):
        tr = PathTracker(Path((0.0, 0.0, 0.0), [(10.0, 0.0)]), cfg, DIFF)
        x, v, out = 0.0, 0.0, []
        pose = lambda: SimpleNamespace(x=x, y=0.0, yaw=0.0, speed=v, yaw_rate=0.0, usable_for_control=True)  # noqa: E731
        assert tr.start(pose())
        for _ in range(int(seconds / 0.05)):
            v, _ = tr.update(pose(), 0.05)
            out.append((x, v))
            x += v * 0.05
        return out

    def test_first_0p3_m_is_slow_then_cruises(self):
        cfg = differential_config(PathConfig(cruise_mps=0.8, delay_s=0.0))
        self.assertEqual((cfg.start_creep_mps, cfg.start_creep_m), (0.1, 0.3))
        out = self.drive(cfg)
        self.assertLessEqual(max(v for x, v in out if x < 0.29), 0.1 + 1e-9)
        self.assertGreater(max(v for x, v in out), 0.79)
        old = self.drive(replace(cfg, start_creep_mps=0.0))
        self.assertGreater(max(v for x, v in old if x < 0.29), 0.2)          # 不慢走时照旧按斜坡加速


if __name__ == '__main__':
    unittest.main()
