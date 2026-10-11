"""独立画圆收尾：交点后多喷 0.1 m，停喷后沿圆走 0.3 m，再沿切线离开 1 m 停车。
起喷（2026-10-06 用户）：直线助跑后先沿圆走 1 m 不喷，再从原来的起点开始喷。只做纯逻辑测试。"""
from collections import deque
from dataclasses import replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'radar_system'), str(ROOT)]
import route_plan as rp
from live_cruise import label_sections
from path_tracker import (Path as Route, PathConfig, Section, SectionTracker, differential_config,
                          DIFF_REVERSE_MPS, DIFF_SETTLE_S)
from route_operations import RouteOperations
from robot_core.kinematics import ChassisGeometry

GEO = ChassisGeometry(drive='differential', track_m=.675, diff_min_radius_m=.5)
NOZZLE = (.24, -.4575)          # 实车：后轴前 0.24 m、右 0.4575 m
MIN_R = .556
DT = .02


def spec(radius=2., clockwise=True, laps=1):
    return rp.RouteSpec(kind='circle', center=(0., 0.), radius=radius, clockwise=clockwise, laps=laps, cruise=.8)


def nozzle(sec, s):
    return rp.nozzle_at(*sec.car_pose(s), *NOZZLE)


def swept(sec, a, b, sp, step=.01):
    """喷头从路程 a 到 b 沿图纸上的圆转过的弧长（顺着画的方向为正）。"""
    sign = -1. if sp.clockwise else 1.
    n = max(1, math.ceil((b-a)/step))
    total, prev = 0., nozzle(sec, a)
    for i in range(1, n+1):
        q = nozzle(sec, a+(b-a)*i/n)
        d = math.atan2(q[1], q[0])-math.atan2(prev[1], prev[0])
        total += sign*math.atan2(math.sin(d), math.cos(d))*sp.radius
        prev = q
    return total


def cars(sp):
    """喷头压在圆上（倒车助跑接入）和离圆较远（原地转向接入）两种起步位置。"""
    start, _ = rp.loop_pieces(sp, MIN_R, NOZZLE)
    x0, y0, h0 = start
    return [(x0+.1*math.cos(h0), y0+.1*math.sin(h0), h0),
            (x0-3.*math.cos(h0)+1.*math.sin(h0), y0-3.*math.sin(h0)-1.*math.cos(h0), h0+.4)]


class CirclePlan(unittest.TestCase):
    def check(self, sp, sections):
        sec = sections[-1]
        self.assertIsNotNone(sec.paint_to_m)
        exit_start = sec.path.pieces[-1].s0
        self.assertAlmostEqual(sec.path.length-exit_start, 1.0, places=9)
        mark = getattr(sec, 'paint_from_m', 0.)
        # 距离按实际喷头圆计算，顺逆时针的后轴半径不同。
        self.assertAlmostEqual(swept(sec, mark, sec.paint_to_m, sp),
                               2*math.pi*sp.radius*sp.laps+.1, delta=.003)
        self.assertAlmostEqual(swept(sec, sec.paint_to_m, exit_start, sp), .3, delta=.003)
        self.assertAlmostEqual(math.hypot(*nozzle(sec, sec.paint_to_m)), sp.radius, places=6)
        for s in (sec.paint_to_m+.01, (sec.paint_to_m+exit_start)/2, exit_start-1e-6):
            self.assertAlmostEqual(math.hypot(*nozzle(sec, s)), sp.radius, places=6)
            self.assertAlmostEqual(sec.path.kappa(s), sec.path.kappa(mark+.1), places=12)
        start = sec.car_pose(exit_start)
        finish = sec.car_pose(sec.path.length)
        self.assertAlmostEqual(finish[0], start[0]+math.cos(start[2]), places=9)
        self.assertAlmostEqual(finish[1], start[1]+math.sin(start[2]), places=9)
        self.assertAlmostEqual(finish[2], start[2], places=9)
        for s in (exit_start+.01, exit_start+.25, sec.path.length):
            self.assertAlmostEqual(sec.path.kappa(min(s, sec.path.length-1e-6)), 0., places=12)

    def test_plan_current_paints_past_junction_then_runs_out(self):
        for radius in (1.5, 2., 3.):
            for clockwise in (True, False):
                for laps in (1, 2):
                    sp = spec(radius, clockwise, laps)
                    for car in cars(sp):
                        with self.subTest(radius=radius, clockwise=clockwise, laps=laps, car=car):
                            sections, entry_m = rp.plan_current(sp, car, MIN_R, nozzle_x=NOZZLE)
                            label_sections(sections, entry_m, sp.cruise)
                            self.assertEqual(sections[-1].field_role, 'paint')
                            self.check(sp, sections)

    def test_plan_circle_same_tail(self):
        sp = spec()
        start, _ = rp.loop_pieces(sp, MIN_R, NOZZLE)
        x0, y0, h0 = start
        car = (x0-3.*math.cos(h0), y0-3.*math.sin(h0), h0)
        sections, entry_m = rp.plan(sp, car, MIN_R, nozzle_x=NOZZLE)
        label_sections(sections, entry_m, sp.cruise)
        self.check(sp, sections)

    def test_runs_one_metre_on_circle_before_painting(self):
        for radius in (1.5, 2., 3.):
            for clockwise in (True, False):
                sp = spec(radius, clockwise)
                for car in cars(sp):
                    with self.subTest(radius=radius, clockwise=clockwise, car=car):
                        sections, entry_m = rp.plan_current(sp, car, MIN_R, nozzle_x=NOZZLE)
                        label_sections(sections, entry_m, sp.cruise)
                        sec = sections[-1]
                        mark = sec.paint_from_m
                        k = sec.path.kappa(mark+.1)
                        arc = rp.CIRCLE_RUN_IN_M/(abs(k)*sp.radius)
                        # 起喷前的 1 m 已经在同一个圆上：曲率不变、喷头压在图纸圆上
                        self.assertGreaterEqual(mark, arc+rp.CIRCLE_TRANSITION_M+rp.RUN_UP_M-1e-9)
                        for s in (mark-arc+1e-6, mark-arc/2, mark-1e-6):
                            self.assertAlmostEqual(sec.path.kappa(s), k, places=12)
                            self.assertAlmostEqual(math.hypot(*nozzle(sec, s)), sp.radius, places=6)
                        self.assertAlmostEqual(swept(sec, mark-arc, mark, sp), rp.CIRCLE_RUN_IN_M, delta=.003)
                        # 再往前是进圆过渡：曲率从 0 一直往圆曲率增，不突变
                        lead = mark-arc
                        ks = [sec.path.kappa(lead-rp.CIRCLE_TRANSITION_M+u) for u in [i/50*rp.CIRCLE_TRANSITION_M for i in range(1, 50)]]
                        self.assertTrue(all(abs(b) >= abs(a)-1e-12 and a*k >= 0 for a, b in zip(ks, ks[1:])))
                        self.assertLess(abs(ks[0]), abs(k)*.1+1e-9)
                        self.assertGreater(abs(ks[-1]), abs(k)*.9-1e-9)
                        self.assertLessEqual(max(abs(b-a) for a, b in zip([0.]+ks, ks+[k])), abs(k)/rp.CIRCLE_TRANSITION_STEPS+1e-9)
                        # 再往前是直线助跑
                        self.assertAlmostEqual(sec.path.kappa(lead-rp.CIRCLE_TRANSITION_M-.05), 0., places=12)
                        # 起喷点没挪：还是离车最近的那一点
                        run = rp.start_from_nearest(sp, car, MIN_R, nozzle_x=NOZZLE)
                        start, _ = rp.loop_pieces(run, MIN_R, NOZZLE)
                        for a, b in zip(nozzle(sec, mark), rp.nozzle_at(*start, *NOZZLE)):
                            self.assertAlmostEqual(a, b, places=6)

    def test_plan_circle_runs_one_metre_on_circle_too(self):
        sp = spec()
        start, _ = rp.loop_pieces(sp, MIN_R, NOZZLE)
        x0, y0, h0 = start
        car = (x0-3.*math.cos(h0)+1.*math.sin(h0), y0-3.*math.sin(h0)-1.*math.cos(h0), h0+.4)
        sections, entry_m = rp.plan(sp, car, MIN_R, nozzle_x=NOZZLE)
        label_sections(sections, entry_m, sp.cruise)
        sec = sections[-1]
        arc = rp.CIRCLE_RUN_IN_M/(abs(sec.path.kappa(sec.paint_from_m+.1))*sp.radius)
        self.assertAlmostEqual(swept(sec, sec.paint_from_m-arc, sec.paint_from_m, sp), rp.CIRCLE_RUN_IN_M, delta=.003)
        for a, b in zip(nozzle(sec, sec.paint_from_m), rp.nozzle_at(x0, y0, h0, *NOZZLE)):
            self.assertAlmostEqual(a, b, places=6)

    def test_route_length_unchanged(self):
        sp = spec()
        _, loop = rp.loop_pieces(sp, MIN_R, NOZZLE)
        self.assertAlmostEqual(rp.route_length(sp, MIN_R, NOZZLE)[0], sum(v for v, _ in loop))


class _Pose:
    usable_for_control, quality, sigma_xy, sigma_yaw, reasons, rtk_trusted = True, 'RTK_FIXED', .006, .002, (), True

    def __init__(self, x, y, yaw, speed, yaw_rate):
        self.x, self.y, self.yaw, self.speed, self.yaw_rate = x, y, yaw, speed, yaw_rate


class CircleDrive(unittest.TestCase):
    def test_keeps_arc_speed_through_paint_end_and_stops_in_run_out(self):
        sp = spec()
        start, loop = rp.loop_pieces(sp, MIN_R, NOZZLE)
        # 车停在沿圆助跑起点的切线后方：直线开过去、不原地转，只有一段
        (_, (x0, y0, h0), _), = rp._circle_run_in(sp, [(1, start, loop)])[0]
        back = rp.RUN_UP_M + .2
        car = (x0-back*math.cos(h0), y0-back*math.sin(h0), h0)
        sections, entry_m = rp.plan(sp, car, MIN_R, nozzle_x=NOZZLE)
        label_sections(sections, entry_m, sp.cruise)
        self.assertEqual(len(sections), 1)
        sec = sections[0]
        tr = SectionTracker(sections, differential_config(PathConfig(cruise_mps=.8)), GEO,
                            pivot_rate=GEO.pivot_rate_rad_s, pivot_accel=GEO.pivot_accel_rad_s2, reverse_gain=1.,
                            reverse_mps=DIFF_REVERSE_MPS, settle_s=DIFF_SETTLE_S, paint_offset_limit_m=.03,
                            constant_straight_speed=True)
        # 理想底盘：指令纯延迟 0.3 s 后照走
        x, y, yaw = car
        v = w = 0.
        queue = deque([(0., 0.)]*15)
        self.assertTrue(tr.start(_Pose(x, y, yaw, 0., 0.)))
        rows = []
        for _ in range(int(120/DT)):
            vx, wz = tr.update(_Pose(x, y, yaw, v, w), DT)
            rows.append((tr.trackers[0].st.s, vx))
            if tr.st.state == 'DONE':
                break
            queue.append((vx, wz))
            v, w = queue.popleft()
            x += v*DT*math.cos(yaw+w*DT/2)
            y += v*DT*math.sin(yaw+w*DT/2)
            yaw += w*DT
        self.assertEqual(tr.st.state, 'DONE')
        arc = tr.trackers[0].cfg.arc_speed_mps
        # 交点前后一直按圆弧定速画，不在喷线范围里刹车
        painted = [vx for s, vx in rows if sec.paint_from_m+1. <= s < sec.paint_to_m]
        self.assertTrue(painted)
        self.assertGreaterEqual(min(painted), arc-1e-9)
        # 停在多走的那截里：停喷后才刹车，开出 0.3 m 以上才停
        self.assertGreater(rows[-1][0], sec.paint_to_m+.3)
        self.assertLessEqual(rows[-1][0], sec.path.length+.05)


class RepairLap(unittest.TestCase):
    def test_repair_lap_moves_circle_paint_end(self):
        k = -.5
        lap = 2*math.pi/abs(k)
        length = lap+.2+rp.CIRCLE_RUN_OUT_M
        sec = Section(1, Route((0., 0., 0.), [(length, k)]))
        sec.field_role, sec.paint_from_m, sec.paint_to_m = 'paint', 0., length-rp.CIRCLE_RUN_OUT_M
        tr = SectionTracker([sec], replace(differential_config(PathConfig()), arc_speed_mps=None), GEO)
        tr.st.state = tr.trackers[0].st.state = 'RUNNING'
        source = Section(1, Route((0., 0., 0.), [(length, k)]))
        source.paint_to_m = sec.paint_to_m
        ops = RouteOperations.__new__(RouteOperations)
        ops.m = NS(tracker=tr, pose=None, message='')
        ops.record = {'circle_spray': {'lap_m': lap, 'extra_added': False, 'error': None}, 'total_m': length, 'plan': []}
        ops.decode = lambda record: [source]
        ops.encode = lambda sections: [dict(paint_to=s.paint_to_m, length=s.path.length) for s in sections]
        ops.save = lambda: None
        ops.execution, ops.runtime_entry = None, 0.
        ops._append_spray_repair()
        end = length-rp.CIRCLE_RUN_OUT_M+lap
        self.assertAlmostEqual(tr.sections[0].paint_to_m, end)
        self.assertAlmostEqual(tr.trackers[0].paint_to_m, end)
        self.assertAlmostEqual(tr.trackers[0].paint_spans[-1][1], end)
        self.assertAlmostEqual(ops.record['plan'][0]['paint_to'], end)
        self.assertAlmostEqual(tr.sections[0].path.length-end, rp.CIRCLE_RUN_OUT_M)


if __name__ == '__main__':
    unittest.main()
