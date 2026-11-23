"""双天线测向偏差在线学习（gnss_fusion FusionConfig.heading_bias_learning）不再被位置降权的 2 cm 互检卡住。

2026-11-23 实车 30 分钟：两根天线位置互检偏差中位 2.5 cm、只有 29% 的时间 ≤2 cm，学习要连续 4 m 都在 2 cm 以内，
一次没学；测向实际只偏 0.7°、却按安装偏差 1.7° 补，每条线开头的方向系统性偏约 1°。现在学习按
heading_bias_dev_max_m（5 cm）挡：两根天线之间基本不变的 2–3 cm 差照样学，明显被污染的段照样不学。

车沿 +x 以 0.8 m/s 直走（真车头 0），10 Hz；测向比真车头偏 1°（接收机报 179°）。"""
import dataclasses
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT), str(ROOT / 'tests')]
import gnss_fusion as gf  # noqa: E402
from test_secondary_position import ANT, BASE, config  # noqa: E402


def drive(pair_offset_m, bias_deg=1.0, seconds=60., speed=.8, **overrides):
    """副天线固定解比真实位置沿车头多出 pair_offset_m（两根天线互检偏差基本不变）。返回 (融合, 最后的位姿)。"""
    f = gf.GnssFusion(dataclasses.replace(config(), heading_bias_learning=True, **overrides))
    f.set_origin(31., 120.)
    t, pose = 0., None
    for _ in range(int(seconds * 10)):
        t = round(t + .1, 6)
        x = speed * t
        f.add_wheel(gf.WheelSample(t, speed, 0.))
        f.add_imu(gf.ImuSample(t, 0., 0.))
        f.add_heading(gf.DualHeading(t, 180. - bias_deg, 0., BASE, 'NARROW_INT', .3, 30))
        lat2, lon2 = f.to_geodetic(x + ANT[0] + pair_offset_m, ANT[1] - BASE)
        f.add_fix2(gf.GnssFix(t, lat2, lon2, 0., 'NARROW_INT', .01, .01, .02, 1., 30))
        lat, lon = f.to_geodetic(x + ANT[0], ANT[1])
        f.add_fix(gf.GnssFix(t, lat, lon, 0., 'NARROW_INT', .01, .01, .02, 1., 30))
        pose = f.pose(t) or pose
    return f, pose


class HeadingBiasLearning(unittest.TestCase):
    def test_default_limit_is_5cm_and_position_weighting_stays_2cm(self):
        cfg = config()
        self.assertEqual((cfg.heading_bias_dev_max_m, cfg.dual_weight_m), (.05, .02))

    def test_learns_with_steady_3cm_pair_difference(self):
        f, pose = drive(.03)
        cal = f.calibration()
        self.assertGreaterEqual(cal['heading_bias_updates'], 6)
        self.assertAlmostEqual(cal['heading_bias_learned_deg'], 1.0, delta=.15)
        self.assertLess(abs(math.degrees(pose.yaw)), .15)          # 融合车头回到真车头

    def test_old_2cm_limit_never_learned(self):
        """原来的门槛（= dual_weight_m 2 cm）：同样 3 cm 的差，一次不学，车头一直偏 1°。"""
        f, pose = drive(.03, heading_bias_dev_max_m=.02)
        self.assertEqual(f.calibration()['heading_bias_updates'], 0)
        self.assertGreater(abs(math.degrees(pose.yaw)), .8)

    def test_polluted_pair_still_blocks_learning(self):
        f, _ = drive(.08)
        self.assertEqual(f.calibration()['heading_bias_updates'], 0)


if __name__ == '__main__':
    unittest.main()
