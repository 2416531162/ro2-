"""整圆卫星好时慢修加快（2026-10-11 用户「修复a」，PathConfig.arc_good_pull_*）：卫星好时估计更紧跟卫星；卫星不够好、
刚跳过点、作业圆弧都照原慢修。10-10 R2 实车圆回放见 PathConfig 注释。"""
from dataclasses import replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'radar_system'), str(ROOT)]
from path_tracker import PathConfig, PathTracker, circle, differential_config
from robot_core.kinematics import ChassisGeometry

DT = .02
GEO = ChassisGeometry(drive='differential', track_m=.675, diff_min_radius_m=.5)


def tracker(good=True):
    # 双天线快修关掉，只看慢修
    cfg = replace(differential_config(PathConfig(cruise_mps=.5)), arc_rtk_radial_enabled=False)
    if not good:
        cfg = replace(cfg, arc_good_pair_max_m=1e-9)
    tr = PathTracker(circle(2.4, left=False, lead_in=2., lead_out=1.), cfg, GEO)
    tr.set_nozzle(.24, -.4575)
    x, y, h = tr.path.pose(0.)
    tr.start(NS(x=x, y=y, yaw=h, speed=0., yaw_rate=0., usable_for_control=True,
                quality='RTK_FIXED', reasons=(), correction_xy=(0., 0.)))
    return tr


def feed(tr, i, *, offset=0., pair=.005, sigma=.01, sats=24, fixed=True):
    now = i*DT
    s = 2.5+.5*now
    x, y, h = tr.path.pose(s)
    x -= math.sin(h)*offset
    y += math.cos(h)*offset
    tr.st.s = s
    tr.sync(now, (now, h+1.7, -.5/2.4))
    tr.set_heading_baseline(now, .412)
    tr.set_rtk_quality(now, sigma, sats, pair, fixed)
    tr.set_secondary_fix(now, x, y, fixed, (.24, -.2025))
    pose = NS(x=x, y=y, yaw=h, speed=.5, yaw_rate=-.5/2.4, usable_for_control=True,
              quality='RTK_FIXED', reasons=(), correction_xy=(0., 0.), rtk_trusted=True)
    tr._arc_first(pose, DT)
    return dict(tr._arc_status)


def slip(good=True, **quality):
    """侧向滑移 5 cm（2 s 内），车头角不变：IMU 推算看不到，只有卫星看得到。返回最后的 (卫星 − 估计, 状态)。"""
    tr = tracker(good)
    for i in range(1, 201):
        feed(tr, i)
    for i in range(201, 351):
        st = feed(tr, i, offset=.05*min(1., (i-200)/100), **quality)
    return st['e_rtk']-st['e_est'], st


class GoodSkyPull(unittest.TestCase):
    def test_good_sky_follows_satellite_much_closer(self):
        old, _ = slip(good=False)
        new, st = slip()
        self.assertTrue(st['good_sky'])
        self.assertGreater(abs(old), .02)            # 原慢修落后 2 cm 以上
        self.assertLess(abs(new), .25*abs(old))

    def test_not_good_enough_keeps_the_original_slow_pull(self):
        old, _ = slip(good=False)
        for quality in (dict(pair=.025), dict(pair=None), dict(sats=19), dict(sigma=.026)):
            with self.subTest(**quality):
                gap, st = slip(**quality)
                self.assertFalse(st['good_sky'])
                if quality.get('sigma') is None and quality.get('pair', .005) is not None and quality.get('sats', 24) >= 20:
                    self.assertAlmostEqual(gap, old, places=9)

    def test_jump_waits_a_metre_before_good_sky_again(self):
        tr = tracker()
        for i in range(1, 201):
            feed(tr, i)
        held = released = None
        for i in range(201, 801):
            st = feed(tr, i, offset=.04)             # 突然跳 4 cm 一直不回来：先挡、等够了再认
            if st['holding'] and held is None:
                held = i
            if held and not st['holding'] and released is None:
                released = i
                self.assertFalse(st['good_sky'])
            if released and not st['good_sky']:
                self.assertLess((i-released)*DT*.5, 1.+.02)
        self.assertIsNotNone(held); self.assertIsNotNone(released)
        self.assertTrue(st['good_sky'])

    def test_work_arcs_keep_the_original_slow_pull(self):
        tr = tracker()
        for i in range(1, 201):
            feed(tr, i)
        tr._correct_arc_radial(tr._arc_e+.02, .01, True, DT, partial=True)
        self.assertFalse(tr._arc_good)
        tr._correct_arc_radial(tr._arc_e+.02, .01, True, DT)
        self.assertTrue(tr._arc_good)


if __name__ == '__main__':
    unittest.main()
