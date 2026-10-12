"""同一 3.5 m 助跑：轮子起动滞后/欠阻尼下，有界自适应纠偏和位置/航向收稳交接。
与完全不用起步稳定策略对比，保留摆动、线头精度与不反转单轮的检查。"""
from collections import deque
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
import math
import random
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT)]
import route_plan as rp
from path_tracker import Path as Route, PathConfig, Section, SectionTracker, differential_config
from robot_core.kinematics import ChassisGeometry

DT, TRACK, NX, NY = .02, .675, .24, -.4575
GEO = ChassisGeometry(drive='differential', track_m=TRACK, diff_min_radius_m=.5)


class Wheel:
    """轮子速度环：二阶欠阻尼（ζ 0.10–0.25、0.8–1.3 Hz），起步随机晚 0–0.3 s 才动（10-10 实录左右轮先后动最多差 0.3 s）。"""
    def __init__(self, rng):
        self.wn, self.z = 2 * math.pi * rng.uniform(.8, 1.3), rng.uniform(.10, .25)
        self.delay, self.v, self.a, self.since = rng.uniform(0., .3), 0., 0., None

    def step(self, cmd, t):
        if self.since is None:
            if abs(cmd) < 1e-4:
                return 0.
            self.since = t
        if t - self.since < self.delay:
            return 0.
        self.a += (self.wn ** 2 * (cmd - self.v) - 2 * self.z * self.wn * self.a) * DT
        self.v += self.a * DT
        return self.v


def run(seed, e0, h0, settle):
    rng = random.Random(seed)
    run_up = rp.RUN_UP_M
    sec = Section(1, Route((0., 0., 0.), [(run_up + 12., 0.)]))
    sec.field_role, sec.paint_from_m, sec.paint_to_m = 'paint', run_up, run_up + 11.5
    cfg = differential_config(PathConfig(cruise_mps=.8))
    if not settle:
        cfg = replace(cfg, launch_settle_max_m=0.)
    tr = SectionTracker([sec], cfg, GEO, constant_straight_speed=True, constant_paint_speed=True).trackers[0]
    tr.set_nozzle(NX, NY)
    left, right = Wheel(rng), Wheel(rng)
    x, y, h, v, w = 0., e0, math.radians(h0), 0., 0.
    sent = deque([(0., 0.)] * 4)
    pose = lambda: NS(x=x, y=y, yaw=h, speed=v, yaw_rate=w, usable_for_control=True, quality='RTK_FIXED', rtk_trusted=True,
                      reasons=(), correction_xy=(0., 0.), correction_yaw=0., sigma_xy=.005, sigma_yaw=.005)
    assert tr.start(pose())
    rows = []
    for i in range(int(40 / DT)):
        t = (i + 1) * DT
        tr.sync(t, (t, h, w))
        if i % 5 == 0:
            tr.set_heading_baseline(t, .412)
            tr.set_rtk_quality(t, .008, 30, .005)
        vc, wc = tr.update(pose(), DT)
        sent.append((vc, wc))
        cv, cw = sent.popleft()
        vl, vr = left.step(cv - cw * TRACK / 2, t), right.step(cv + cw * TRACK / 2, t)
        v, w = (vl + vr) / 2, (vr - vl) / TRACK
        x += v * DT * math.cos(h + w * DT / 2)
        y += v * DT * math.sin(h + w * DT / 2)
        h += w * DT
        rows.append(dict(t=t, s=tr.st.s, h=math.degrees(h), vc=vc, wc=wc, settle=tr._settling(),
                         noz=100 * (y + NX * math.sin(h) - NY * (1 - math.cos(h)))))
        if tr.st.s > run_up + 3 or tr.st.state != 'RUNNING':
            break
    first = [r for r in rows if r['t'] <= 3.]
    trend = lambda r: first[0]['h'] + (first[-1]['h'] - first[0]['h']) * (r['t'] - first[0]['t']) / (first[-1]['t'] - first[0]['t'])
    swing = [r['h'] - trend(r) for r in first]
    head = [abs(r['noz']) for r in rows if run_up <= r['s'] <= run_up + 2.]
    return dict(swing=max(swing) - min(swing), head=max(head), rows=rows)


CASES = [(seed, e0, h0) for seed in range(6) for e0 in (-.02, .02) for h0 in (-4., -1., 1., 4.)]


class LaunchSettle(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.old = [run(*c, settle=False) for c in CASES]
        cls.new = [run(*c, settle=True) for c in CASES]

    def test_run_up_is_three_point_five_metres(self):
        # 2026-10-11 用户：助跑暂时 2.8 → 3.5 m
        self.assertEqual(rp.RUN_UP_M, 3.5)

    def test_start_swing_is_smaller(self):
        median = lambda rs: sorted(r['swing'] for r in rs)[len(rs) // 2]
        self.assertLess(median(self.new), .85 * median(self.old))
        self.assertLess(max(r['swing'] for r in self.new), .8 * max(r['swing'] for r in self.old))

    def test_line_head_is_not_worse(self):
        # 线头规则（用户）：线头精度不能变差；模型里允许 0.2 cm 的数值差
        self.assertLessEqual(max(r['head'] for r in self.new), max(r['head'] for r in self.old) + .2)
        median = lambda rs: sorted(r['head'] for r in rs)[len(rs) // 2]
        self.assertLessEqual(median(self.new), median(self.old) + .05)

    def test_settle_limits_turn_speed_and_never_reverses_a_wheel(self):
        for res in self.new:
            settling = [r for r in res['rows'] if r['settle']]
            self.assertTrue(settling)
            prev=0.
            for r in res['rows']:
                if r['settle']:
                    # 交接中失去收稳条件时允许按既有减速度退回，不能继续提速。
                    self.assertLessEqual(r['vc'], max(.35,prev) + 1e-9)
                    self.assertLessEqual(abs(r['wc']), math.radians(4.) + 1e-9)
                self.assertLessEqual(abs(r['wc']), 2 * abs(r['vc']) / TRACK + 1e-9)
                prev=r['vc']
            self.assertLess(settling[-1]['s'],rp.RUN_UP_M+.05)

    def test_wheel_response_alone_cannot_release_a_misaligned_start(self):
        # 两轮即使跟得上指令，也须位置与方向收稳才提速；不能把“稳稳地往偏处走”当作完成起步。
        class Steady:
            """跟得上指令的轮子，右轮带 ±1.8 mm/s 交替的小抖动（车身转速约 ±0.15°/s，实车总有）"""
            made = 0
            def __init__(self, rng):
                self.jitter, self.k = .0018 if Steady.made % 2 else 0., 0
                Steady.made += 1
            def step(self, cmd, t):
                self.k += 1
                return cmd + (self.jitter if self.k % 2 else -self.jitter) * (abs(cmd) > 1e-4)
        from unittest.mock import patch
        with patch(__name__ + '.Wheel', Steady):
            for e0 in (-.02, .02):
                for h0 in (-8., -4.5, 4.5, 8.):
                    rows = run(0, e0, h0, settle=True)['rows']
                    first=next(i for i,r in enumerate(rows) if r['settle'])
                    released=next(r for r in rows[first+1:] if not r['settle'])
                    if released['s']<rp.RUN_UP_M:
                        self.assertLess(abs(released['h']),1.8)
                        self.assertLess(abs(released['noz']),2.2)

    def test_moving_handover_is_not_a_start(self):
        sec = Section(1, Route((0., 0., 0.), [(10., 0.)]))
        sec.field_role, sec.paint_from_m, sec.paint_to_m = 'paint', 3., 9.5
        tr = SectionTracker([sec], differential_config(PathConfig(cruise_mps=.8)), GEO,
                            constant_straight_speed=True, constant_paint_speed=True).trackers[0]
        p = NS(x=0., y=.02, yaw=0., speed=.6, yaw_rate=0., usable_for_control=True, quality='RTK_FIXED', rtk_trusted=True,
               reasons=(), correction_xy=(0., 0.), correction_yaw=0., sigma_xy=.005, sigma_yaw=.005)
        self.assertTrue(tr.start(p))
        tr.update(p, DT)
        self.assertFalse(tr._settling())


if __name__ == '__main__':
    unittest.main()
