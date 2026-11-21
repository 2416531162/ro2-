"""一根天线被污染就用另一根定位（2026-11-21 用户：「左卫星被污染了，右卫星其实还是好的，但是基线已经 0.3 m 了，
我认为还能继续用」）。gnss_fusion FusionConfig.secondary_*：两根对不上时拿纯推算判谁对，用对的那根；
主天线浮点、副天线固定时直接用副天线，不进推算维持。

车沿 +x 以 0.8 m/s 直走（车头 0），10 Hz。主天线（左）在后轴前 0.235、左 0.205，副天线（右）在它右边 0.41 m。
测向在被污染时按基线出界不给融合（rtk_node 判无效）。"""
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT)]
import gnss_fusion as gf

ANT = (.235, .205)
BASE = .41


def config(on=True):
    return gf.FusionConfig(drive_differential=True, yaw_input='angle', continuous_output=True,
                           antenna_xy=ANT, baseline_yaw_body_deg=-90., dual_weight_m=.02,
                           fix_floor_m=.002, fix_sigma_scale=.25, fix_jitter_k=2.0, rtk_hold_s=9.0,
                           rejoin_rate_mps=.04, output_position_tau_s=.10,
                           secondary_baseline_m=BASE if on else 0.)


class Rig:
    def __init__(self, on=True, speed=.8, lag=0.):
        self.f = gf.GnssFusion(config(on))
        self.f.set_origin(31., 120.)
        self.t, self.speed, self.lag = 0., speed, lag

    def x_at(self, t):
        return self.speed * t

    def step(self, n=1, left=(0., 0.), left_sol='NARROW_INT', right=(0., 0.), heading=True):
        out = []
        for _ in range(n):
            self.t = t = round(self.t + .1, 6)
            f = self.f
            f.add_wheel(gf.WheelSample(t, self.speed, 0.))
            f.add_imu(gf.ImuSample(t, 0., 0.))
            if heading:
                f.add_heading(gf.DualHeading(t, 180., 0., BASE, 'NARROW_INT', .3, 30))
            t2 = round(t - self.lag, 6)
            lat2, lon2 = f.to_geodetic(self.x_at(t2) + ANT[0] + right[0], ANT[1] - BASE + right[1])
            f.add_fix2(gf.GnssFix(t2, lat2, lon2, 0., 'NARROW_INT', .01, .01, .02, 1., 30))
            lat, lon = f.to_geodetic(self.x_at(t) + ANT[0] + left[0], ANT[1] + left[1])
            f.add_fix(gf.GnssFix(t, lat, lon, 0., left_sol, .01, .01, .02, 1., 30))
            p = f.pose(t)
            out.append(p)
        return out

    def ready(self):
        return self.step(60)            # 6 s 两根都好：学固定差


def lateral_cm(poses):
    return max(abs(p.y) for p in poses) * 100


def ramp(rig, total, n, **kw):
    out = []
    for i in range(1, n + 1):
        out += rig.step(left=(0., total * i / n), **kw)
    return out


class SecondaryPosition(unittest.TestCase):
    def test_agreeing_antennas_use_main(self):
        rig = Rig()
        poses = rig.ready()
        self.assertEqual({p.position_antenna for p in poses}, {'main'})
        cal = rig.f.calibration()['secondary_position']
        self.assertGreaterEqual(cal['samples'], 20)
        self.assertLess(max(abs(v) for v in cal['offset_m']), .002)
        self.assertLess(lateral_cm(poses[-10:]), .3)

    def test_left_jump_uses_right(self):
        for on in (True, False):
            rig = Rig(on)
            rig.ready()
            poses = rig.step(40, left=(0., .05), heading=False)
            if on:
                self.assertGreater(sum(p.position_antenna == 'second' for p in poses) / len(poses), .9)
                self.assertLess(lateral_cm(poses), 1.0)
            else:
                self.assertGreater(lateral_cm(poses), 3.0)          # 原来：跟着被污染的左天线偏

    def test_left_slow_drift_uses_right(self):
        """左天线 3 s 内慢慢漂开 6 cm 再停住：两根差不到 3 cm 分不出谁错，融合跟着偏到约 3 cm；之后按纯推算
        认出是左天线、换右天线，回到 1 cm 以内。"""
        for on in (True, False):
            rig = Rig(on)
            rig.ready()
            poses = ramp(rig, .06, 30, heading=False) + rig.step(30, left=(0., .06), heading=False)
            if on:
                self.assertLess(lateral_cm(poses), 3.5)
                self.assertLess(lateral_cm(poses[-20:]), 1.0)
                self.assertEqual(poses[-1].position_antenna, 'second')
            else:
                self.assertGreater(lateral_cm(poses), 4.0)

    def test_right_polluted_keeps_main(self):
        rig = Rig()
        rig.ready()
        poses = rig.step(40, right=(0., .05), heading=False)
        self.assertEqual({p.position_antenna for p in poses}, {'main'})
        self.assertLess(lateral_cm(poses), 1.0)

    def test_left_float_right_fixed_keeps_rtk(self):
        """左天线浮点（还偏 20 cm）12 s，超过 9 s 推算维持：用右天线照常 RTK，不停。"""
        for on in (True, False):
            rig = Rig(on)
            rig.ready()
            poses = rig.step(120, left=(0., .2), left_sol='NARROW_FLOAT', heading=False)
            if on:
                self.assertTrue(all(p.usable_for_control for p in poses))
                self.assertNotIn('DEAD_RECKONING', {p.quality for p in poses})
                self.assertGreater(sum(p.position_antenna == 'second' for p in poses) / len(poses), .95)
                self.assertLess(lateral_cm(poses), 1.0)
            else:
                self.assertFalse(poses[-1].usable_for_control)      # 原来：推算维持到时停车

    def test_back_to_main_when_antennas_agree(self):
        rig = Rig()
        rig.ready()
        rig.step(30, left=(0., .05), heading=False)
        poses = rig.step(30)
        self.assertEqual(poses[-1].position_antenna, 'main')
        self.assertLess(lateral_cm(poses), 1.0)

    def test_secondary_late_by_50ms(self):
        """副天线晚 50 ms（0.8 m/s 走 4 cm）：按车速对齐，两根照样对得上，不误换。"""
        rig = Rig(lag=.05)
        poses = rig.ready()
        self.assertEqual({p.position_antenna for p in poses}, {'main'})
        cal = rig.f.calibration()['secondary_position']
        self.assertLess(max(abs(v) for v in cal['offset_m']), .003)
        poses = rig.step(40, left=(0., .05), heading=False)
        self.assertGreater(sum(p.position_antenna == 'second' for p in poses) / len(poses), .9)
        self.assertLess(lateral_cm(poses), 1.0)

    def test_off_without_baseline(self):
        rig = Rig(on=False)
        rig.ready()
        poses = rig.step(20, left_sol='NARROW_FLOAT', heading=False)
        self.assertEqual({p.position_antenna for p in poses}, {'main'})


if __name__ == '__main__':
    unittest.main()
