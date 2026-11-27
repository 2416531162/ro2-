"""整圆车头换算按两根天线原始轨迹、径向往回拉一律拉向原始解（11-27 用户「修复」画圆，user_rules.ARC_TRACK_*、
ARC_RAW_OBSERVATION，PathTracker._arc_track_offset）。

56300f6a 那圈 R3 圆：换算原来按 5 s 跟融合航向，楼下测向偏，换算比真值偏 +0.16–1.28°，IMU 推算每米往外漂 0.3–2.1 cm，
慢修拉不回来；两根天线对不上时又退回拉向偏了的融合位置。控制以为喷头比实际靠外平均 1.4 cm，圆小了 1.6 cm。
这里按真实几何喂两根天线（后轴 + 车头转过的杠杆臂），融合航向偏 +1°。"""
from dataclasses import replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT)]
from path_tracker import PathConfig, PathTracker, circle, differential_config  # noqa: E402
from robot_core.kinematics import ChassisGeometry  # noqa: E402
import user_rules as rules  # noqa: E402

DT, V, R = .02, .5, 2.4
GEO = ChassisGeometry(drive='differential', track_m=.675, diff_min_radius_m=.5)
MAIN, SECOND = (.24, .2075), (.24, -.2025)
IMU_OFF = 1.7                    # IMU 偏航 = 真车头 + 1.7 rad


def tracker(**cfg):
    c = replace(differential_config(PathConfig(cruise_mps=V)), **cfg)
    tr = PathTracker(circle(R, left=False, lead_in=2., lead_out=1.), c, GEO)
    tr.set_nozzle(.24, -.4575)
    x, y, h = tr.path.pose(0.)
    tr.start(NS(x=x, y=y, yaw=h, speed=0., yaw_rate=0., usable_for_control=True,
                quality='RTK_FIXED', reasons=(), correction_xy=(0., 0.)))
    return tr


def run(steps=900, fused_bias_deg=1., fused_offset=0., main_offset=0., second_offset=0., second=True, pair=.005, **cfg):
    """沿圆走 steps 拍（后轴真轨迹在路径上）。融合航向偏 fused_bias_deg、融合位置往左（圆外）偏 fused_offset；
    两根天线按真车头算出位置，再各自往左偏 main_offset / second_offset（多路径）。second=False：只有主天线。
    pair：接收机报的两天线距离误差（> arc_rtk_pair_max_m 时双天线快修不开，56300f6a 那圈多数时候这样）。"""
    tr = tracker(**cfg)
    out = []
    for i in range(1, steps + 1):
        now = i * DT
        s = 2.5 + V * now
        x, y, h = tr.path.pose(s)
        tr.st.s = s
        w = -V / R
        tr.sync(now, (now, h + IMU_OFF, w))
        tr.set_heading_baseline(now, .412)
        tr.set_rtk_quality(now, .01, 28, pair, True)
        c, sn = math.cos(h), math.sin(h)
        for (ax, ay), off, setter in ((MAIN, main_offset, tr.set_primary_fix),
                                      (SECOND, second_offset, tr.set_secondary_fix if second else None)):
            if setter is None:
                continue
            setter(now, x + c * ax - sn * ay - sn * off, y + sn * ax + c * ay + c * off, True, (ax, ay))
        pose = NS(x=x - sn * fused_offset, y=y + c * fused_offset, yaw=h + math.radians(fused_bias_deg), speed=V,
                  yaw_rate=w, usable_for_control=True, quality='RTK_FIXED', reasons=(), correction_xy=(0., 0.),
                  rtk_trusted=True)
        tr._arc_first(pose, DT)
        out.append(dict(tr._arc_status))
    return out, tr


def tail(rows, key='e_est', n=300):
    vals = [r[key] for r in rows[-n:] if r.get(key) is not None]
    return sum(vals) / len(vals)


class Rules(unittest.TestCase):
    def test_rules_table(self):
        cfg = differential_config(PathConfig())
        self.assertIs(cfg.arc_track_heading, rules.ARC_TRACK_HEADING)
        self.assertIs(cfg.arc_raw_observation, rules.ARC_RAW_OBSERVATION)
        self.assertEqual((cfg.arc_track_chord_m, cfg.arc_track_tau_m, cfg.arc_track_min_m),
                         (rules.ARC_TRACK_CHORD_M, rules.ARC_TRACK_TAU_M, rules.ARC_TRACK_MIN_M))
        for name in ('arc_track_chord_m', 'arc_track_tau_m', 'arc_track_min_m'):
            with self.assertRaises(ValueError):
                replace(cfg, **{name: 0.})
        for name in ('arc_track_heading', 'arc_raw_observation'):
            with self.assertRaises(ValueError):
                replace(cfg, **{name: 1})


class TrackHeading(unittest.TestCase):
    def test_biased_fused_heading_no_longer_drags_the_estimate(self):
        # 融合航向偏 +1°、双天线快修和好天慢修都不开（两天线距离误差 2.5 cm）：原来换算跟着偏、推算每米往外漂约 1.7 cm，慢修落后 1 cm
        # 以上；现在换算按天线轨迹，估计贴着真值
        old, _ = run(arc_track_heading=False, pair=.025)
        new, tr = run(pair=.025)
        self.assertGreater(abs(tail(old)), .01)
        self.assertLess(abs(tail(new)), .003)
        self.assertEqual(new[-1]['offset_source'], 'track')
        self.assertAlmostEqual(math.degrees(tr._arc_off), math.degrees(-IMU_OFF), delta=.15)
        self.assertGreater(abs(math.degrees(_wrap(old[-1]['offset_deg'] - math.degrees(-IMU_OFF)))), .7)

    def test_track_takes_over_only_after_arc_track_min_m(self):
        rows, _ = run(steps=900)
        first = next(i for i, r in enumerate(rows) if r.get('offset_source') == 'track')
        self.assertGreaterEqual(rows[first]['track_m'], rules.ARC_TRACK_MIN_M - 1e-9)
        self.assertTrue(all(r.get('offset_source') != 'track' for r in rows[:first]))

    def test_one_antenna_only_keeps_following_fused_heading(self):
        # 只有一根天线的原始解：分不出车头和那根天线自己的慢漂，照旧跟融合航向
        rows, _ = run(second=False)
        self.assertTrue(all(r.get('offset_source') != 'track' for r in rows))
        self.assertTrue(all(r.get('radial_obs') == 'fused' for r in rows if r.get('active')))

    def test_unbiased_fused_heading_is_about_the_same(self):
        old, _ = run(fused_bias_deg=0., arc_track_heading=False)
        new, _ = run(fused_bias_deg=0.)
        self.assertLess(abs(tail(new) - tail(old)), .003)


class RawObservation(unittest.TestCase):
    def test_disagreeing_antennas_pull_to_their_mean_not_the_fused_position(self):
        # 两根天线各偏 ±2 cm（对不上，> arc_rtk_pair_max_m）、平均在线上；融合位置偏外 2 cm。原来慢慢拉向融合位置，
        # 现在拉向两根平均（只慢修，不快修）
        args = dict(fused_offset=.02, main_offset=.02, second_offset=-.02, fused_bias_deg=0.)
        old, _ = run(arc_raw_observation=False, arc_track_heading=False, **args)
        new, _ = run(arc_track_heading=False, **args)
        self.assertGreater(tail(old), .012)
        self.assertLess(abs(tail(new)), .004)
        self.assertEqual(new[-1]['radial_obs'], 'raw_mean')
        self.assertEqual(new[-1]['radial_rtk_weight'], 0.)
        self.assertIsNone(new[-1]['e_rtk_raw'])

    def test_agreeing_antennas_are_used_even_while_the_fast_pull_is_off(self):
        # 两根对得上、融合位置偏外 2 cm：原来快修权重攒满前拉向融合位置，现在拉向原始解
        args = dict(fused_offset=.02, fused_bias_deg=0.)
        old, _ = run(arc_raw_observation=False, arc_track_heading=False, arc_rtk_radial_enabled=True, **args)
        new, _ = run(arc_track_heading=False, **args)
        self.assertEqual(new[-1]['radial_obs'], 'raw')
        self.assertLess(abs(tail(new)), abs(tail(old)))
        self.assertLess(abs(tail(new)), .004)


def _wrap(deg):
    return math.remainder(deg, 360.)


if __name__ == '__main__':
    unittest.main()
