"""起步有界纠偏只在大偏差时放宽（2026-11-22 用户「只在大偏差时放宽」）。

草丛起步一侧轮子被拖住，车头扭 10–15°：可信直线上车头偏 ≥ 5° 或喷头横偏 ≥ 5 cm 才从 4°/s 放到 8°/s，
扳回来更快；平地正常起步（车头偏几度、横偏 2 cm）跟原来一模一样。"""
from collections import deque
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
import math
import random
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT), str(ROOT / 'tests')]
import route_plan as rp
from path_tracker import Path as Route, PathConfig, Section, SectionTracker, differential_config
from test_launch_settle import DT, GEO, NX, NY, TRACK, Wheel

OLD = dict(launch_big_heading_deg=1000., launch_big_e_m=1000.)   # 不放宽 = 改之前


class Dragged(Wheel):
    """草里被拖住的轮子：动起来后头 drag_s 秒只转出速度环的 frac（11-22 下午 185721：右轮 0.7 s 时 0.10、左轮 0.31 m/s）。"""
    def __init__(self, rng, drag_s, frac):
        super().__init__(rng)
        self.drag_s, self.frac = drag_s, frac

    def step(self, cmd, t):
        out = super().step(cmd, t)
        if self.since is not None and t - self.since - self.delay < self.drag_s:
            return out * self.frac
        return out


def run(seed, e0, h0, drag=None, **over):
    rng = random.Random(seed)
    run_up = rp.RUN_UP_M
    sec = Section(1, Route((0., 0., 0.), [(run_up + 12., 0.)]))
    sec.field_role, sec.paint_from_m, sec.paint_to_m = 'paint', run_up, run_up + 11.5
    cfg = replace(differential_config(PathConfig(cruise_mps=.8)), **over)
    tr = SectionTracker([sec], cfg, GEO, constant_straight_speed=True, constant_paint_speed=True).trackers[0]
    tr.set_nozzle(NX, NY)
    left, right = Wheel(rng), Wheel(rng)
    if drag == 'L':
        left = Dragged(rng, .8, .2)
    elif drag == 'R':
        right = Dragged(rng, .8, .2)
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
    return rows


def back_within(rows, deg):
    """车头扭到最大以后，第一次回到 deg 以内的时刻。"""
    k = max(range(len(rows)), key=lambda i: abs(rows[i]['h']))
    return next(r['t'] for r in rows[k:] if abs(r['h']) < deg)


class LaunchBigError(unittest.TestCase):
    def test_small_start_error_is_unchanged(self):
        # 平地正常起步：车头偏 ≤ 3°、横偏 2 cm，从不到放宽门槛，每一拍指令和改之前一样
        for seed in range(4):
            for e0 in (-.02, .02):
                for h0 in (-3., 3.):
                    new, old = run(seed, e0, h0), run(seed, e0, h0, **OLD)
                    self.assertEqual(len(new), len(old))
                    for a, b in zip(new, old):
                        self.assertAlmostEqual(a['wc'], b['wc'], places=9)
                        self.assertAlmostEqual(a['vc'], b['vc'], places=9)

    def test_dragged_wheel_twist_turns_back_faster_within_eight(self):
        head = lambda rows: max(abs(r['noz']) for r in rows if rp.RUN_UP_M <= r['s'] <= rp.RUN_UP_M + 2.)
        heads_new, heads_old = [], []
        for seed in range(4):
            for side in ('L', 'R'):
                new, old = run(seed, 0., 0., side), run(seed, 0., 0., side, **OLD)
                twist = max(abs(r['h']) for r in old)
                self.assertGreater(twist, 5.)                                # 真扭出了大偏差（5–11°）
                fast = [abs(r['wc']) for r in new if r['settle']]
                self.assertLessEqual(max(fast), math.radians(8.) + 1e-9)   # 不超过 8°/s
                for r in new:
                    self.assertLessEqual(abs(r['wc']), 2 * abs(r['vc']) / TRACK + 1e-9)   # 照样不让单轮反转
                if twist >= 8.:
                    self.assertGreater(max(fast), math.radians(6.))        # 放宽了
                    self.assertLess(back_within(new, 2.), back_within(old, 2.) - .3)
                heads_new.append(head(new)); heads_old.append(head(old))
        # 线头：8 次里最差和中位都要比不放宽好（单次可能有 ±0.5 cm 的起伏）
        median = lambda xs: sorted(xs)[len(xs) // 2]
        self.assertLess(max(heads_new), max(heads_old) - 2.)
        self.assertLess(median(heads_new), median(heads_old))

    def test_big_lateral_error_also_widens(self):
        rows = run(0, .08, 0.)
        self.assertGreater(max(abs(r['wc']) for r in rows if r['settle']), math.radians(4.5))


if __name__ == '__main__':
    unittest.main()
