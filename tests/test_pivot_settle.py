#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""差速车原地转完不补小角度（SectionTracker pivot_min 5°）。全程只用数学仿真。

2026-09-30 20:50–20:59 实车 5 次路线作业：起步原地转完一停，车头回弹约 1.2°（轮胎 / 万向轮被扭着），停在差
1–2° 处；按 1° 补转时每次只发 ±14 erpm 左右的小指令，车推一下又弹回来，反复补转，起步卡 12–27 s（最多 20 次），
要人去摆万向轮才走。用户规则：差 5° 以上才原地转；转完 5° 以内直接两轮慢速往前开，万向轮回正、车头纠正到 1° 以内。
"""
import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, ROOT)

from gen5_line_sim import GEN5, NOZZLE_X, Gen5Chassis, make_pose  # noqa: E402
import route_plan as rp  # noqa: E402
from path_tracker import Path, PathConfig, Section, SectionTracker, differential_config, wrap  # noqa: E402

DEG = math.pi / 180.0


class SpringyChassis(Gen5Chassis):
    """Gen5Chassis（实测延迟、增益）+ 实车原地转的两个毛病：
    停下回弹：原地转时轮胎 / 万向轮被扭着，最多存 springback_deg；转动指令停了按 0.15 s 放掉，车头往回转；
    小指令推不动：停着原地转、角速度指令小于 stiction（rad/s）时轮子不转（实车 ±14 erpm ≈ 0.04 rad/s 推不动）。"""

    def __init__(self, *a, springback_deg=1.2, stiction=0.06, whip_deg=0.0, whip_s=0.8, speed_delay_s=0.33, **kw):
        # 车速延迟按 2026-10-01 实车路线段尾停车实测（见 Gen5Chassis）
        super().__init__(*a, speed_delay_s=speed_delay_s, **kw)
        self.limit, self.stiction = springback_deg * DEG, stiction
        self.wound = 0.0
        # 起步甩头：原地转完从停着往前开，车头按上次原地转的方向再甩 whip_deg（whip_s 内，半正弦）。
        # 2026-09-30 实车 21 次段起步 19 次同向，4–13°，多数 6–9°，发生在开始动的头 0.8 s
        self.whip, self.whip_s = whip_deg * DEG, whip_s
        self.turn_sign, self.whip_t0 = 0, None

    def command(self, t, vx, wz):
        if abs(vx) < 1e-6 and abs(wz) < self.stiction:
            wz = 0.0
        super().command(t, vx, wz)

    def step(self, t, dt):
        before, was_moving = self.yaw, abs(self.cmd[0]) > 1e-6
        super().step(t, dt)
        if abs(self.cmd[0]) < 1e-6 and abs(self.cmd[1]) > 0.05:
            self.turn_sign = 1 if self.cmd[1] > 0 else -1
        if self.whip and self.cmd[0] > 1e-6 and not was_moving and self.turn_sign:
            self.whip_t0, self.whip_dir, self.turn_sign = t, self.turn_sign, 0
        if self.whip_t0 is not None:
            tau = t - self.whip_t0
            if tau < self.whip_s:
                rate = self.whip_dir * self.whip * math.pi / (2.0 * self.whip_s) * math.sin(math.pi * tau / self.whip_s)
                self.yaw += rate * dt
                self.wz += rate
            else:
                self.whip_t0 = None
        if abs(self.cmd[0]) < 1e-6 and abs(self.cmd[1]) > 1e-9 and abs(self.v) < 0.05:
            self.wound = max(-self.limit, min(self.limit, self.wound + self.yaw - before))
        else:
            back = self.wound * min(1.0, dt / 0.15)
            self.wound -= back
            self.yaw -= back
            self.wz -= back / dt


def drive(car, sections, seconds=60.0, **kw):
    """20 Hz 控制、10 ms 底盘步长。返回 (跟踪器, 行)：行 = (t, 换段中, 原地转中, 真车头, 本段路程, 喷头横偏)。"""
    cfg = differential_config(PathConfig(cruise_mps=0.5))
    tr = SectionTracker(sections, cfg, GEN5, pivot_rate=0.8, pivot_accel=1.0, reverse_gain=1.0, **kw)
    assert tr.start(make_pose(car.x, car.y, car.yaw, car.v, car.wz))
    rows = []
    for i in range(int(seconds / 0.01)):
        t = i * 0.01
        if i % 5 == 0:
            v, w = tr.update(make_pose(car.x, car.y, car.yaw, car.v, car.wz), 0.05)
            car.command(t, v, w)
            st = tr.st
            rows.append((t, tr.switching, tr.status()['pivoting'], car.yaw, tr.trackers[tr.index].st.s,
                         st.e + NOZZLE_X * math.sin(st.heading_err)))
            if st.state not in ('RUNNING', 'PAUSED'):
                break
        car.step(t, 0.01)
    return tr, rows


def turns(rows):
    return sum(1 for a, b in zip(rows, rows[1:]) if b[2] and not a[2])


def straight(yaw):
    return [Section(1, Path((0.0, 0.0, yaw), [(3.0, 0.0)]))]


class TestPivotSettle(unittest.TestCase):

    def test_start_turn_then_drives_off_without_topping_up(self):
        """起步原地转 90°：停下回弹后差 1–2°，不补转，直接往前开；开出 0.5 m 车头差 1° 以内。"""
        tr, rows = drive(SpringyChassis(0.0, 0.0, 0.0), straight(math.pi / 2))
        self.assertEqual((tr.st.state, turns(rows)), ('DONE', 1))
        left = next(r[0] for r in rows if not r[1])
        self.assertLess(left, 8.0)                                  # 转 90° 约 3 s + 停稳
        settled = next(r[3] for r in rows if not r[1])
        self.assertGreater(abs(wrap(settled - math.pi / 2)), 0.5 * DEG)   # 确实停在差一点的地方（回弹）
        self.assertLess(abs(wrap(settled - math.pi / 2)), 5.0 * DEG)
        self.assertTrue(all(abs(wrap(r[3] - math.pi / 2)) < 1.0 * DEG for r in rows if not r[1] and r[4] > 0.5))

    def test_old_one_degree_rule_gets_stuck(self):
        """按 1° 补转（旧规则）：回弹后的小角度推不动，补转到次数用完只能中止（以前不限次数，一直卡着）。"""
        tr, rows = drive(SpringyChassis(0.0, 0.0, 0.0), straight(math.pi / 2), pivot_min=1.0 * DEG)
        self.assertEqual((tr.st.state, tr.st.reason), ('ABORTED', 'pivot_timeout'))
        self.assertEqual(turns(rows), 3)

    def test_small_misalignment_drives_straight_off(self):
        """起步车头差 4°：不原地转，直接慢速开，跟踪纠正到 1° 以内。"""
        tr, rows = drive(SpringyChassis(0.0, 0.0, math.pi / 2 - 4.0 * DEG), straight(math.pi / 2))
        self.assertEqual((tr.st.state, turns(rows)), ('DONE', 0))
        self.assertTrue(all(abs(wrap(r[3] - math.pi / 2)) < 1.0 * DEG for r in rows if r[4] > 0.6))

    def test_car_that_springs_back_too_far_gives_up_after_two_top_ups(self):
        """转完总差 5° 以上（回弹 8°）：补转 2 次还不行按 pivot_timeout 中止，不无限卡着。"""
        tr, rows = drive(SpringyChassis(0.0, 0.0, 0.0, springback_deg=8.0), straight(math.pi / 2))
        self.assertEqual((tr.st.state, tr.st.reason), ('ABORTED', 'pivot_timeout'))
        self.assertEqual(turns(rows), 3)
        self.assertLess(rows[-1][0], 30.0)

    def test_route_with_sharp_corners_no_top_ups(self):
        """接入 + 方形路线（急弯原地转、退 0.5 m 再开回拐角）：每处只转一次，喷头压线。"""
        spec = rp.parse_spec(dict(kind='path', points=[[0, 0], [4, 0], [4, 3], [0, 3]], closed=False))
        car = SpringyChassis(-2.0, -1.5, 0.6)
        sections, _ = rp.plan(spec, (car.x, car.y, car.yaw), GEN5.min_turn_radius_m / 0.9, nozzle_x=NOZZLE_X)
        tr, rows = drive(car, sections, seconds=200.0)
        self.assertEqual(tr.st.state, 'DONE')
        self.assertEqual(turns(rows), 1 + tr.pivots)                # 起步一次 + 每个急弯一次，不补转
        painting = [abs(r[5]) for r in rows if not r[1] and r[4] > 0.3]
        self.assertLess(max(painting), 0.03)


def paint_straight(whip_deg, **cfg_kw):
    """接入（原地转、直线、原地转、引入 1 m）后画 20 m 直线（y = 0），起步甩头 whip_deg（按原地转的方向）。
    返回 (状态, 引入段喷嘴最大偏, 线头 0–7 m 最大偏, 7 m 后最大偏)，cm。"""
    from dataclasses import replace
    spec = rp.parse_spec(dict(kind='path', points=[[0, 0], [20, 0]], closed=False, cruise=0.8))
    car = SpringyChassis(-3.0, 2.0, 1.0, whip_deg=whip_deg)
    sections, _ = rp.plan(spec, (car.x, car.y, car.yaw), GEN5.min_turn_radius_m / 0.9, nozzle_x=NOZZLE_X)
    cfg = replace(differential_config(PathConfig(cruise_mps=0.8)), **cfg_kw)
    tr = SectionTracker(sections, cfg, GEN5, pivot_rate=0.8, pivot_accel=1.0, reverse_gain=1.0)
    assert tr.start(make_pose(car.x, car.y, car.yaw, car.v, car.wz))
    rows = []
    for i in range(int(120 / 0.01)):
        t = i * 0.01
        if i % 5 == 0:
            v, w = tr.update(make_pose(car.x, car.y, car.yaw, car.v, car.wz), 0.05)
            car.command(t, v, w)
            if tr.st.state not in ('RUNNING', 'PAUSED'):
                break
            if tr.index == len(tr.sections) - 1 and not tr.switching:
                rows.append((car.x + NOZZLE_X * math.cos(car.yaw), car.y + NOZZLE_X * math.sin(car.yaw)))
        car.step(t, 0.01)
    worst = lambda a, b: 100 * max(abs(y) for x, y in rows if a <= x < b)   # noqa: E731
    return tr.st.state, worst(-1.0, 0.0), worst(0.0, 7.0), worst(7.0, 19.5)


class TestStartHold(unittest.TestCase):
    """起步锁车头（start_hold_m）：原地转完起步车头再甩 8°（实车常见值），慢走时曲率反馈压不住。"""

    def test_whip_is_held_before_the_line_starts(self):
        for whip in (8.0, -8.0):
            with self.subTest(whip=whip):
                state, lead, head, rest = paint_straight(whip)
                self.assertEqual(state, 'DONE')
                self.assertLess(head, 0.6)          # 画线头 7 m 内（不锁 1.3–1.5 cm）
                self.assertLess(rest, 0.3)

    def test_without_hold_the_line_head_is_off(self):
        state, lead, head, rest = paint_straight(8.0, start_hold_m=0.0)
        self.assertEqual(state, 'DONE')
        self.assertGreater(head, 1.0)

    def test_no_whip_no_harm(self):
        state, lead, head, rest = paint_straight(0.0)
        self.assertEqual(state, 'DONE')
        self.assertLess(max(lead, head, rest), 1.0)


if __name__ == '__main__':
    unittest.main()
