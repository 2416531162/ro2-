"""控制输出航向直接对齐（2026-10-10 用户）：双天线可信（两个天线都固定、基线 0.35–0.45 m）时，车停着、不在喷，
或者差值超过 2°，控制用的车头角直接对齐，不再每秒挪 1°；只有在喷且差不到 2° 时仍平滑。

10-10 实录 16:05:00 / 16:05:30：开始作业时控制输出的航向比融合差 58° / 27°（融合已跟上双天线），按 1°/s 追了
一分多钟。这里让 IMU 假装转了一个角度（双天线说没转）：融合按测向很快拉回，给控制的输出原来按 1°/s 慢慢追，
看它什么情况下直接对齐。"""
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT)]
import gnss_fusion as gf

ANT = (.24, .2075)          # 主天线在后轴前、左（Gen5 fusion.json）
BASE = .41


class Rig:
    """Gen5 融合配置；车沿 +x（车头 0）以 speed 直走，卫星、轮速、IMU 都 10 Hz。"""
    def __init__(self):
        self.f = gf.GnssFusion(gf.FusionConfig(drive_differential=True, yaw_input='angle', continuous_output=True,
                                               antenna_xy=ANT, baseline_yaw_body_deg=-90.))
        self.f.set_origin(31., 120.)
        self.t, self.x, self.imu = 0., 0., 0.

    def step(self, speed=0., baseline=BASE, fix2='NARROW_INT', n=1, imu_deg_s=0.):
        for _ in range(n):
            self.t = round(self.t + .1, 6)
            self.x += speed * .1
            self.imu += math.radians(imu_deg_s) * .1
            t, f = self.t, self.f
            f.add_wheel(gf.WheelSample(t, speed, 0.))
            f.add_imu(gf.ImuSample(t, math.radians(imu_deg_s), self.imu))
            f.add_heading(gf.DualHeading(t, 180., 0., baseline, 'NARROW_INT', .3, 30))
            lat, lon = f.to_geodetic(self.x + ANT[0], ANT[1])
            f.add_fix(gf.GnssFix(t, lat, lon, 0., 'NARROW_INT', .01, .01, .02, 1., 30))
            lat2, lon2 = f.to_geodetic(self.x + ANT[0], ANT[1] - BASE)
            f.add_fix2(gf.GnssFix(t, lat2, lon2, 0., fix2, .01, .01, .02, 1., 30))
        return f.pose(self.t)

    def fake_turn(self, deg, **kw):
        """IMU 1 s 内假装转了 deg（车实际直走、双天线不变），返回最后的位姿。"""
        return self.step(n=10, imu_deg_s=deg, **kw)


def gap_deg(pose):
    return abs(math.degrees(pose.correction_yaw))


class OutputHeadingSnap(unittest.TestCase):
    def ready(self, speed=0.):
        rig = Rig()
        p = rig.step(speed, n=30)
        self.assertTrue(p.rtk_trusted, p.reasons)
        self.assertLess(gap_deg(p), .05)
        return rig

    def test_old_behaviour_reproduced_when_rule_not_met(self):
        # 在喷、基线 0.47（卫星可信但不在 0.35–0.45）：仍按 1°/s 追，假转 20° 后差十几度
        rig = self.ready(.8)
        rig.f.set_painting(True)
        rig.step(.8, baseline=.47, n=3)
        p = rig.fake_turn(20., speed=.8, baseline=.47)
        self.assertTrue(p.rtk_trusted)
        self.assertGreater(gap_deg(p), 10.)

    def test_stationary_snaps_even_while_painting_flag_set(self):
        rig = self.ready()
        rig.f.set_painting(True)
        p = rig.fake_turn(58.)
        self.assertLess(gap_deg(p), .05)

    def test_moving_not_painting_snaps(self):
        rig = self.ready(.8)
        rig.f.set_painting(False)
        p = rig.fake_turn(27., speed=.8)
        self.assertLess(gap_deg(p), .05)

    def test_moving_painting_large_gap_snaps(self):
        rig = self.ready(.8)
        rig.f.set_painting(True)
        p = rig.fake_turn(20., speed=.8)
        self.assertLessEqual(gap_deg(p), 2.)

    def test_moving_painting_small_gap_still_smooth(self):
        # IMU 一帧跳 1.8°（差不到 2°）：在喷时输出照旧每秒最多追 1°，不在喷时直接对齐
        for painting in (True, False):
            with self.subTest(painting=painting):
                rig = self.ready(.8)
                rig.f.set_painting(painting)
                gaps = [gap_deg(rig.step(.8, imu_deg_s=18.))]
                for _ in range(10):
                    gaps.append(gap_deg(rig.step(.8)))
                if painting:
                    self.assertGreater(min(gaps[1:8]), .5)     # 没有被直接抹平
                    self.assertLessEqual(max(gaps), 2.)
                    for a, b in zip(gaps[1:], gaps[2:]):
                        self.assertLessEqual(a - b, .1 + 1e-6)   # 每 0.1 s 最多追 0.1°
                else:
                    self.assertLess(max(gaps), .05)

    def test_baseline_outside_035_045_does_not_snap(self):
        rig = self.ready(.8)
        rig.f.set_painting(False)
        rig.step(.8, baseline=.47, n=3)          # 0.30–0.50 仍算卫星可信，但不在 0.35–0.45
        p = rig.fake_turn(20., speed=.8, baseline=.47)
        self.assertTrue(p.rtk_trusted)
        self.assertGreater(gap_deg(p), 10.)

    def test_secondary_not_fixed_does_not_snap(self):
        rig = self.ready(.8)
        rig.f.set_painting(False)
        rig.step(.8, fix2='NARROW_FLOAT', n=15)   # 副天线浮点超过 1 s：不算双固定
        p = rig.fake_turn(20., speed=.8, fix2='NARROW_FLOAT')
        self.assertFalse(p.rtk_trusted)
        self.assertGreater(gap_deg(p), 10.)


if __name__ == '__main__':
    unittest.main()
