#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""直线卫星横偏改用原始双天线固定解（11-27 用户「直线控制改用原始 RTK 算横偏，不再用融合位置；两根对得上才用」，
user_rules.STRAIGHT_RAW_*，PathTracker._straight_raw_lateral）。

10-10 20:03 那条 48 m 线（地上实测前半段往右鼓约 10 cm）：楼下双天线测向被带偏 +2–3.6°，融合位置跟着推偏、
比 RTK 偏左 4–5 cm；直线拿融合位置当卫星横偏，β 往右修，车本来是直的被修成往右走。这里用 0.36 s 延迟的车模型复现：
融合位置在 6–26 m 慢慢偏左再回来，两根天线的原始解是对的。"""
import math
import os
import random
import sys
import unittest
from collections import deque
from dataclasses import replace
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, 'radar_system'), ROOT]
from path_tracker import Path as Route, PathConfig, PathTracker, differential_config  # noqa: E402
from robot_core.kinematics import ChassisGeometry  # noqa: E402
import user_rules as rules  # noqa: E402

GEO = ChassisGeometry(drive='differential', track_m=.675, diff_min_radius_m=.5)
DT, NX, NY = .02, .23, -.4525
MAIN, SECOND = (.24, .2075), (.24, -.2025)        # 两根天线车体系杠杆臂（前, 左），测向基线 0.41 m 朝右


def fused_bias(x, peak=.05):
    """融合位置比真位置多偏左多少（m）：6 m 起 10 m 内偏到 peak，再 10 m 回到 0（复现 48 m 线测向被带偏那段）。"""
    if x < 6. or x > 26.:
        return 0.
    return peak * (x - 6.) / 10. if x < 16. else peak * (26. - x) / 10.


def run(raw=True, feed_raw=True, length=36., v=.5, sigma_m=.018, peak=.05, pair_error=0., seed=4,
        antenna_noise=.002, latency=.04):
    """直线 length 米、0.36 s 纯延迟底盘。融合位置 = 真位置 + fused_bias；原始解 = 真天线位置 + 噪声（pair_error：
    副天线多偏左这么多，可以是走到哪米的函数）。返回 (每拍 (x, 真喷头横偏, 直线状态), tracker)。"""
    rng = random.Random(seed)
    cfg = differential_config(PathConfig(cruise_mps=v))
    cfg = replace(cfg, straight_imu_first=replace(cfg.straight_imu_first, raw_lateral=raw))
    tr = PathTracker(Route((0., 0., 0.), [(length, 0.)]), cfg, GEO)
    tr.set_nozzle(NX, NY)
    x, psi, t, vv, w = 0., .1 * math.pi / 180., 0., 0., 0.
    y = .003 - NX * math.sin(psi)
    sent = deque([(0., 0., 0.)])
    past = deque([(0., x, y, psi)])

    def pose():
        return NS(x=x, y=y + fused_bias(x, peak), yaw=psi + .2 * math.pi / 180., speed=vv, yaw_rate=w,
                  usable_for_control=True, quality='RTK_FIXED', reasons=(), correction_xy=(0., 0.),
                  correction_yaw=0., rtk_trusted=True)

    def antenna(state, lever, extra=0.):
        _, bx, by, h = state
        ax, ay = lever
        extra = extra(bx) if callable(extra) else extra
        return (bx + math.cos(h) * ax - math.sin(h) * ay + rng.gauss(0, antenna_noise),
                by + math.sin(h) * ax + math.cos(h) * ay + rng.gauss(0, antenna_noise) + extra)

    def inputs():
        tr.sync(t, (t, psi + 1.7, w))
        tr.set_heading_baseline(t, .41)
        tr.set_rtk_quality(t, sigma_m, 28, .01, True)
        tr.set_motor_current(t, 2.2, 2.2)
        if feed_raw and int(round(t / DT)) % 5 == 0:                  # 10 Hz，测量时刻比此刻早 latency
            old = next((s for s in reversed(past) if s[0] <= t - latency), past[0])
            tr.set_primary_fix(old[0], *antenna(old, MAIN), True, MAIN)
            tr.set_secondary_fix(old[0], *antenna(old, SECOND, pair_error), True, SECOND)
    inputs()
    assert tr.start(pose())
    rows = []
    while t < length / v * 2 + 20 and tr.st.state == 'RUNNING':
        inputs()
        vc, wc = tr.update(pose(), DT)
        sent.append((t, vc, wc))
        while len(sent) > 1 and sent[1][0] <= t - .36:
            sent.popleft()
        _, vv, w = sent[0]
        x += vv * math.cos(psi) * DT
        y += vv * math.sin(psi) * DT
        psi += w * DT
        t += DT
        past.append((t, x, y, psi))
        while len(past) > 100:
            past.popleft()
        rows.append((x, y + NX * math.sin(psi) + NY * (math.cos(psi) - 1.), dict(tr._straight_status)))
    return rows, tr


def worst(rows, start=4., end=None):
    end = rows[-1][0] - .5 if end is None else end
    return max(abs(e) for xx, e, _ in rows if start <= xx <= end)


class RawLateral(unittest.TestCase):
    def test_rules_table(self):
        first = differential_config(PathConfig()).straight_imu_first
        self.assertIs(first.raw_lateral, rules.STRAIGHT_RAW_LATERAL)
        self.assertEqual(first.raw_pair_m, rules.STRAIGHT_RAW_PAIR_M)
        self.assertEqual(first.raw_hold_s, rules.STRAIGHT_RAW_HOLD_S)
        self.assertEqual(first.raw_fresh_s, rules.STRAIGHT_RAW_FRESH_S)
        with self.assertRaises(ValueError):
            replace(first, raw_lateral=1)
        for name in ('raw_pair_m', 'raw_hold_s', 'raw_fresh_s'):
            with self.assertRaises(ValueError):
                replace(first, **{name: 0.})

    def test_raw_lateral_matches_the_true_nozzle(self):
        # 融合位置没偏时：原始解换算出来的喷头横偏和真值差在毫米级（含 40 ms 测量延迟按 IMU 往前推）
        rows, _ = run(peak=0., antenna_noise=0.)
        diffs = [abs(s['raw_e'] - e) for xx, e, s in rows if s.get('raw_e') is not None and xx > 5.]
        self.assertGreater(len(diffs), 500)
        self.assertLess(max(diffs), .002)
        self.assertTrue(all(s.get('raw_mode') for _, _, s in rows if s.get('active')))

    def test_fused_offset_no_longer_bends_the_line(self):
        # 复现 48 m 线：融合位置偏左 5 cm。老做法（融合位置）被带着往右修，新做法（原始双天线）基本不动
        old, _ = run(raw=False)
        new, _ = run(raw=True)
        w_old, w_new = worst(old), worst(new)
        self.assertGreater(w_old, .025, w_old)
        self.assertLess(w_new, .012, w_new)
        self.assertLess(w_new, .4 * w_old, (w_new, w_old))

    def test_same_as_before_when_fused_agrees_with_raw(self):
        # 融合位置和原始解一致（好天、没被测向带偏）：两种做法走出来的线几乎一样
        old, _ = run(raw=False, peak=0.)
        new, _ = run(raw=True, peak=0.)
        self.assertLess(abs(worst(old) - worst(new)), .003, (worst(old), worst(new)))

    def test_antennas_disagree_means_no_satellite(self):
        # 两根天线一直对不上（副天线多偏 3 cm > 2 cm）：这条线卫星都不用（同卫星不可信，按 IMU 推算走）
        rows, _ = run(pair_error=.03)
        active = [s for xx, _, s in rows if s.get('active') and xx > 5.]
        self.assertTrue(active)
        self.assertTrue(all(s.get('raw_e') is None for s in active))
        self.assertTrue(all(abs(s['raw_pair_m']) > rules.STRAIGHT_RAW_PAIR_M for s in active
                            if s.get('raw_pair_m') is not None))
        self.assertFalse(any(s.get('first_rtk_accepted') for s in active))

    def test_short_disagreement_in_the_middle(self):
        # 楼下那段两根天线对不上 4 m（12–16 m，融合位置正偏着）：这段按 IMU 走，前后照常用原始解，线不被带偏
        rows, _ = run(pair_error=lambda bx: .03 if 12. <= bx <= 16. else 0.)
        gap = [s for xx, _, s in rows if 12.5 <= xx <= 15.5 and s.get('active')]
        self.assertTrue(gap and all(s.get('raw_e') is None for s in gap))
        self.assertLess(worst(rows), .012)

    def test_short_gap_holds_the_last_value(self):
        # 刚对上过：raw_hold_s 内沿用（按 IMU 往前推），超过就不用
        rows, tr = run(length=12.)
        first = tr.cfg.straight_imu_first
        ages = [s['raw_age_s'] for _, _, s in rows if s.get('raw_age_s') is not None]
        self.assertTrue(ages)
        self.assertLessEqual(max(ages), .1 + 1e-6)     # 10 Hz 两根都对得上：最多隔一帧
        self.assertEqual(first.raw_hold_s, rules.STRAIGHT_RAW_HOLD_S)

    def test_without_raw_fixes_falls_back_to_fused(self):
        # 从没收到过两根天线的原始解（单天线车、老任务层、仿真）：照旧用融合位置
        old, _ = run(raw=False)
        nofeed, _ = run(raw=True, feed_raw=False)
        self.assertEqual([r[:2] for r in old], [r[:2] for r in nofeed])
        self.assertFalse(any(s.get('raw_mode') for _, _, s in nofeed))


if __name__ == '__main__':
    unittest.main()
