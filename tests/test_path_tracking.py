#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""路径跟踪（radar_system/path_tracker.py）测试：几何投影、与直线控制器一致、小圆/8 字闭环仿真、
前进倒车交替的分段跟踪（连线路线倒车补角）。

闭环数字来自 radar_system/tools/path_sim.py（原始传感器 + 真实融合），底盘按实车辨识的转向响应（指令约 0.35 s
后才生效、实际曲率只有指令的 0.96 / 0.90），不代表实车（没有轮胎侧滑）。
"""
import math
import os
import random
import sys
import unittest
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system', 'tools'))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, ROOT)

import path_sim  # noqa: E402
from line_tracker import LineConfig, LineTracker  # noqa: E402
from path_tracker import Path, PathConfig, PathTracker, Section, SectionTracker, circle, figure8, stadium  # noqa: E402

SEEDS = (1, 2, 3, 4)
# 理想运动学替身（指令立刻生效、曲率照走）用的控制器设置：不预估执行延迟、不补转向增益
IDEAL = dict(delay_s=0.0, steer_gain=1.0)


def pose(x=0.0, y=0.0, yaw_deg=0.0, speed=0.0, yaw_rate=0.0, usable=True):
    return SimpleNamespace(x=x, y=y, yaw=math.radians(yaw_deg), speed=speed, yaw_rate=yaw_rate,
                           usable_for_control=usable)


class TestGeometry(unittest.TestCase):

    def test_circle_closes_on_itself(self):
        p = circle(2.0, lead_in=2.0, lead_out=1.5)
        self.assertAlmostEqual(p.length, 2.0 + 4 * math.pi + 1.5)
        a = p.pose(2.0)
        b = p.pose(2.0 + 4 * math.pi)
        for u, v in zip(a[:2], b[:2]):
            self.assertAlmostEqual(u, v, places=9)
        self.assertAlmostEqual(math.cos(a[2]), math.cos(b[2]), places=9)
        # 逆时针：圆心在入口直线左侧 2 m，四分之一圈处车头朝北
        x, y, yaw = p.pose(2.0 + math.pi)
        self.assertAlmostEqual(x, 4.0, places=9)
        self.assertAlmostEqual(y, 2.0, places=9)
        self.assertAlmostEqual(math.degrees(yaw), 90.0, places=6)

    def test_lateral_error_sign_on_arcs(self):
        # 四分之一圈处（4, 2），圆心 (2, 2)：点在圆内 = 逆时针路径的左侧 → 正；顺时针时圆内是右侧 → 负
        s, e, yaw, k = circle(2.0).project(3.9, 2.0)
        self.assertAlmostEqual(e, 0.1, places=9)
        self.assertAlmostEqual(s, 2.0 + math.pi, places=6)
        self.assertAlmostEqual(k, 0.5)
        cw = circle(2.0, left=False)
        s, e, yaw, k = cw.project(3.9, -2.0)
        self.assertAlmostEqual(e, -0.1, places=9)
        self.assertAlmostEqual(k, -0.5)

    def test_projection_stays_on_the_current_lap(self):
        # 两圈的圆：同一个位置，第二圈时不能跳回第一圈；不给提示时取起点那一圈
        p = circle(2.0, laps=2)
        lap = 4 * math.pi
        self.assertAlmostEqual(p.project(2.0, 0.01)[0], 2.0, places=3)
        self.assertAlmostEqual(p.project(2.0, 0.01, 2.0 + lap - 0.1)[0], 2.0 + lap, places=3)
        # 8 字交点：左圈转完、右圈开始都在同一点，按提示取当前这一圈
        f = figure8(2.0)
        self.assertAlmostEqual(f.project(2.0, 0.0, 2.0 + lap - 0.05)[0], 2.0 + lap, places=3)
        s, e, yaw, k = f.project(2.3, 0.0, 2.0 + lap + 0.2)
        # 右圈圆心 (2, −2)：点 (2.3, 0) 对应的弧长 = 2·(π/2 − atan2(2, 0.3))
        self.assertAlmostEqual(s, 2.0 + lap + 2 * (math.pi / 2 - math.atan2(2.0, 0.3)), places=9)
        self.assertEqual(k, -0.5)

    def test_run_up_and_overshoot_extend_the_straights(self):
        p = circle(2.5)
        self.assertAlmostEqual(p.project(-1.2, 0.05, extend_start=3.0)[0], -1.2)
        end = p.pose(p.length)
        self.assertGreater(p.project(end[0] + 0.3 * math.cos(end[2]), end[1] + 0.3 * math.sin(end[2]),
                                     p.length - 0.1)[0], p.length)

    def test_radius_below_chassis_limit_is_rejected(self):
        with self.assertRaisesRegex(ValueError, '最小半径'):
            PathTracker(circle(1.9))
        PathTracker(circle(1.95))


class TestStraightMatchesLineTracker(unittest.TestCase):
    """直线路径上，路径跟踪器必须与已经验证过的直线控制器逐拍一致。"""

    def test_same_commands_on_a_straight_path(self):
        rng = random.Random(5)
        a, b = (1.0, 2.0), (9.0, 8.0)
        yaw0 = math.atan2(b[1] - a[1], b[0] - a[0])
        line = LineTracker(LineConfig(cruise_mps=0.6))
        line.set_line(a, b)
        # 直线控制器没有执行延迟预估和增益补偿：关掉这两项、延迟补偿取同样的 lead_s 时两者逐拍一致
        path = PathTracker(Path((a[0], a[1], yaw0), [(10.0, 0.0)]),
                           PathConfig(cruise_mps=0.6, lead_s=LineConfig().lead_s, **IDEAL))
        x, y, yaw, v = a[0] - 1.0 * math.cos(yaw0), a[1] - 1.0 * math.sin(yaw0) + 0.1, yaw0 + 0.1, 0.0
        p0 = pose(x, y, math.degrees(yaw))
        self.assertEqual(line.check_start(p0), path.check_start(p0))
        self.assertTrue(line.start(p0) and path.start(p0))
        for _ in range(500):
            p = pose(x + rng.gauss(0, 0.003), y + rng.gauss(0, 0.003), math.degrees(yaw + rng.gauss(0, 0.002)),
                     speed=v, yaw_rate=rng.gauss(0, 0.01))
            out_l, out_p = line.update(p, 0.05), path.update(p, 0.05)
            self.assertAlmostEqual(out_l[0], out_p[0], places=9)
            self.assertAlmostEqual(out_l[1], out_p[1], places=9)
            self.assertEqual(line.st.state, path.st.state)
            v, wz = out_l
            yaw += wz * 0.05
            x += v * 0.05 * math.cos(yaw)
            y += v * 0.05 * math.sin(yaw)
        self.assertEqual(path.st.state, 'DONE')


class TestClosedLoop(unittest.TestCase):

    def test_truth_pose_is_millimetre_level(self):
        # 定位完美时只剩控制律和底盘动态：小圆、跑道几毫米；8 字交点曲率从 +1/R 翻到 −1/R，
        # 转向要从一边打到另一边，R 2 m（贴着最小半径 1.92 m）约 2 cm
        limits = {'circle_r2': 0.6, 'circle_r2_cw': 0.6, 'stadium_r2.5': 0.6, 'figure8_r2.5': 1.0, 'figure8_r2': 2.6}
        for name, limit in limits.items():
            with self.subTest(name):
                r = path_sim.run(name, source='truth')
                self.assertEqual(r['state'], 'DONE')
                self.assertLess(r['max_cm'], limit)

    def test_small_circles_with_real_fusion(self):
        limits = {'circle_r2': 2.5, 'circle_r2_cw': 2.5, 'circle_r2.5_2laps': 2.5, 'stadium_r2.5': 2.5,
                  'figure8_r2.5': 2.5, 'figure8_r2': 4.2}
        for name, limit in limits.items():
            with self.subTest(name):
                rs = [path_sim.run(name, seed=s) for s in SEEDS]
                self.assertEqual({r['state'] for r in rs}, {'DONE'})
                self.assertLess(max(r['max_cm'] for r in rs), limit)
                self.assertLess(max(r['rms_cm'] for r in rs), 1.2)

    def test_delay_prediction_is_what_makes_tight_curves_work(self):
        # 同一台实测底盘：不预估执行延迟、不补增益、弯道不降速（改之前的设置），8 字偏差是现在的 10 倍
        old = PathConfig(delay_s=0.0, steer_gain=1.0, lead_s=0.10, lat_accel_mps2=0.3)
        for name in ('figure8_r2', 'circle_r2'):
            with self.subTest(name):
                now = path_sim.run(name, source='truth')
                before = path_sim.run(name, source='truth', config=old)
                self.assertLess(now['max_cm'] * 3.0, before['max_cm'])

    def test_old_ideal_chassis_model_is_still_available(self):
        # 旧的理想底盘（60 ms 延迟、曲率照走）配旧设置：仍是毫米级，只用来对比
        r = path_sim.run('circle_r2', source='truth', chassis_model='ideal',
                         config=PathConfig(lead_s=0.10, lat_accel_mps2=0.3, **IDEAL))
        self.assertEqual(r['state'], 'DONE')
        self.assertLess(r['max_cm'], 0.6)

    def test_trim_is_learned_on_a_circle(self):
        first = path_sim.run('circle_r2.5_trim2', trim_deg=2.0)
        self.assertLess(first['max_cm'], 3.5)
        again = path_sim.run('circle_r2.5_trim2', trim_deg=2.0, seed=2, integral0=first['integral'])
        self.assertLess(again['max_cm'], first['max_cm'])


class TestDelayCompensation(unittest.TestCase):
    """执行延迟预估（Smith）、转向增益补偿、弯道限速：按实车 2026-09-26 辨识的底盘响应。"""

    def test_prediction_replays_the_commands_still_in_flight(self):
        from collections import deque
        tr = PathTracker(Path((0.0, 0.0, 0.0), [(10.0, 0.0)]), PathConfig(delay_s=0.3, steer_gain=0.9))
        # 现在 t=1.0：延迟窗口 [0.7, 1.0)。0.5 发的直行指令在 0.7–0.8 仍有效，0.8 发的弯道指令在 0.8–1.0 有效
        tr._clock = 1.0
        tr._sent = deque([(0.2, 0.9, 0.3), (0.5, 0.4, 0.0), (0.8, 0.4, 0.5)])
        x, y, yaw = tr.predict(0.0, 0.0, 0.0)
        ex, ey, eyaw = Path((0.04, 0.0, 0.0), [(0.08, 0.45)]).pose(0.08)     # 直行 0.04 m + 实际曲率 0.45 走 0.08 m
        self.assertAlmostEqual(x, ex, places=9)
        self.assertAlmostEqual(y, ey, places=9)
        self.assertAlmostEqual(yaw, eyaw, places=9)
        self.assertEqual(len(tr._sent), 2)                                     # 0.2 发的已完全生效，丢掉

    def test_reported_error_is_where_the_car_is_now(self):
        from collections import deque
        tr = PathTracker(Path((0.0, 0.0, 0.0), [(10.0, 0.0)]), PathConfig())
        p = pose(1.0, 0.10, 0.0, speed=0.3)
        self.assertTrue(tr.start(p))
        tr._sent = deque([(-1.0, 0.3, -0.5)])                                 # 在途指令正往线上打方向
        tr.update(p, 0.05)
        self.assertAlmostEqual(tr.st.e, 0.10, places=9)                       # 操作员看到的是现在的横偏

    def test_curvature_is_divided_by_the_steering_gain(self):
        path = circle(2.5, lead_in=1.0)
        on_arc = path.pose(3.0)
        p = pose(on_arc[0], on_arc[1], math.degrees(on_arc[2]), speed=0.3, yaw_rate=0.3 / 2.5)
        for gain in (1.0, 0.96):
            tr = PathTracker(path, PathConfig(delay_s=0.0, lead_s=0.0, ff_preview_s=0.0, steer_gain=gain))
            self.assertTrue(tr.start(p))
            tr.update(p, 0.05)
            self.assertAlmostEqual(tr.st.kappa, 0.4 / gain, places=6)

    def test_reverse_sections_use_the_reverse_gain(self):
        secs = [straight(0.0, 0.0, 0.0, 2.0), straight(2.0, 0.0, 0.0, 1.0, direction=-1)]
        tr = SectionTracker(secs, PathConfig())
        self.assertEqual([t.cfg.steer_gain for t in tr.trackers], [PathConfig().steer_gain, 0.90])

    def test_full_lock_arcs_are_driven_slowly(self):
        # 满舵弯（路线最小半径 1.92 m）约 0.31 m/s：0.33 s 延迟里只走 10 cm
        r = path_sim.run('circle_r2', source='truth', trace=True)
        arc = [row for row in r['rows'] if abs(row['kappa_ref']) > 0.4 and row['state'] == 'RUNNING']
        self.assertLess(max(row['v'] for row in arc), 0.33)


def straight(x, y, yaw_deg, length, direction=1):
    """一段直线；倒车段按行进方向画（yaw = 车头 + 180°）。"""
    yaw = math.radians(yaw_deg + (0.0 if direction > 0 else 180.0))
    return Section(direction, Path((x, y, yaw), [(length, 0.0)]))


class Car:
    """理想运动学替身：指令即速度，可选惯性（换向前要等它停下）。"""

    def __init__(self, x=0.0, y=0.0, yaw_deg=0.0, coast=0.0):
        self.x, self.y, self.yaw, self.v, self.w, self.coast = x, y, math.radians(yaw_deg), 0.0, 0.0, coast
        self.usable = True

    def pose(self):
        return pose(self.x, self.y, math.degrees(self.yaw), self.v, self.w, self.usable)

    def step(self, vx, wz, dt):
        self.v = vx if vx != 0.0 or not self.coast else self.v * self.coast
        self.w = wz if vx != 0.0 else 0.0
        self.yaw += self.w * dt
        self.x += self.v * dt * math.cos(self.yaw)
        self.y += self.v * dt * math.sin(self.yaw)


def run_sections(tracker, car, dt=0.05, until=120.0, every=None):
    t, out = 0.0, []
    while t < until and tracker.st.state in ('RUNNING', 'PAUSED'):
        p = car.pose()
        vx, wz = tracker.update(p, dt)
        out.append((t, vx, tracker.index, tracker.switching, car.x, car.y))
        if every:
            every(t, tracker, car)
        car.step(vx, wz, dt)
        t += dt
    return out


class TestSections(unittest.TestCase):
    """前进 / 倒车交替：倒车段在行进方向坐标里用同一套控制律，段尾停稳再换向。"""

    def test_reverse_straight_converges_like_forward(self):
        tr = SectionTracker([straight(10.0, 0.0, 0.0, 8.0, direction=-1)], PathConfig(cruise_mps=0.5, **IDEAL),
                            reverse_gain=1.0)
        car = Car(10.0, 0.08, 3.0)                                  # 偏左 8 cm、车头偏 3°
        self.assertTrue(tr.start(car.pose()))
        log = run_sections(tr, car)
        self.assertEqual(tr.st.state, 'DONE')
        self.assertTrue(all(vx <= 0.0 for _, vx, *_ in log))
        self.assertLessEqual(max(-vx for _, vx, *_ in log), 0.3 + 1e-9)       # 倒车限速
        self.assertLess(abs(car.y), 0.005)
        self.assertAlmostEqual(car.x, 2.0, delta=0.03)
        self.assertEqual(tr.status()['gear'], 'reverse')

    def test_cusp_waits_until_stopped(self):
        secs = [straight(0.0, 0.0, 0.0, 3.0), straight(3.0, 0.0, 0.0, 2.0, direction=-1)]
        tr = SectionTracker(secs, PathConfig(cruise_mps=0.5, **IDEAL), reverse_gain=1.0)
        car = Car(coast=0.8)                                        # 停车指令后还要滑一会儿
        self.assertTrue(tr.start(car.pose()))
        log = run_sections(tr, car)
        self.assertEqual(tr.st.state, 'DONE')
        switch = [row for row in log if row[3]]
        self.assertTrue(switch and all(row[1] == 0.0 for row in switch))
        first_back = next(i for i, row in enumerate(log) if row[1] < 0)
        self.assertGreaterEqual(sum(1 for row in log[:first_back] if row[3]) * 0.05, tr.settle_s - 1e-9)
        self.assertAlmostEqual(car.x, 1.0, delta=0.03)
        self.assertAlmostEqual(tr.st.s, tr.length, delta=0.03)

    def test_integral_keeps_its_meaning_across_gears(self):
        # 舵机零位按前进坐标保存；倒车段的控制器里反号
        secs = [straight(0.0, 0.0, 0.0, 2.0), straight(2.0, 0.0, 0.0, 1.5, direction=-1)]
        tr = SectionTracker(secs, PathConfig())
        car = Car()
        self.assertTrue(tr.start(car.pose(), integral=0.04))
        self.assertAlmostEqual(tr.trackers[0].st.integral, 0.04)
        seen = []
        run_sections(tr, car, every=lambda t, tr_, c: seen.append((tr_.index, tr_.st.integral,
                                                                      tr_.trackers[1].st.integral)))
        rev = [row for row in seen if row[0] == 1]
        self.assertTrue(rev)
        self.assertAlmostEqual(rev[0][2], -rev[0][1], places=9)
        self.assertAlmostEqual(tr.status()['trim_kappa'], -PathConfig().ki_per_m2 * tr.st.integral)

    def test_pose_loss_while_switching_pauses_and_resumes(self):
        secs = [straight(0.0, 0.0, 0.0, 2.0), straight(2.0, 0.0, 0.0, 1.0, direction=-1)]
        tr = SectionTracker(secs, PathConfig())
        car = Car()
        tr.start(car.pose())
        while not tr.switching:
            car.step(*tr.update(car.pose(), 0.05), 0.05)
        car.usable = False
        self.assertEqual(tr.update(car.pose(), 0.05), (0.0, 0.0))
        self.assertEqual(tr.st.state, 'PAUSED')
        car.usable = True
        self.assertEqual(tr.update(car.pose(), 0.05), (0.0, 0.0))          # 恢复后也要等操作员
        self.assertTrue(tr.start(car.pose()))
        run_sections(tr, car)
        self.assertEqual(tr.st.state, 'DONE')

    def test_car_that_never_stops_aborts_at_the_cusp(self):
        secs = [straight(0.0, 0.0, 0.0, 2.0), straight(2.0, 0.0, 0.0, 1.0, direction=-1)]
        tr = SectionTracker(secs, PathConfig())
        car = Car(coast=1.0)                                        # 速度一直不降
        tr.start(car.pose())
        run_sections(tr, car, until=30.0)
        self.assertEqual((tr.st.state, tr.st.reason), ('ABORTED', 'not_stopped_at_cusp'))

    def test_route_with_reversing_corners_in_closed_loop(self):
        import route_plan
        spec = route_plan.parse_spec(dict(kind='path', points=[[0, 0], [8, 0], [8, 6], [0, 6]], closed=True))
        worst = 0.0
        for seed in SEEDS[:2]:
            r = path_sim.run_route(spec=spec, car=(-3.0, 0.0, 0.0), seed=seed)
            self.assertEqual(r['state'], 'DONE')
            worst = max(worst, r['cover_cm'])
        self.assertLess(worst, 3.0)                                 # 真实融合（含 RTK 噪声）下压线误差 < 3 cm


if __name__ == '__main__':
    unittest.main()
