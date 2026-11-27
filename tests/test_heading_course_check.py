"""测向被楼面反射带偏时融合不跟着走（11-27 用户「改」，gnss_fusion FusionConfig.heading_course_*，user_rules.HEADING_COURSE_*）。

10-10 20:03 那条 48 m 线：楼下测向在 s 12–28 m 比实际行驶方向偏 +2.7–3.6°，IMU 是直的；融合航向按直行 2 s 时间常数
跟过去偏了 2–3°，融合位置跟着推偏、RTK 被当离群降权。这里车沿 +x 以 0.5 m/s 直走（真车头 0），IMU、轮速、固定解都对，
测向在 6–24 m 偏 +3°。"""
import dataclasses
import math
import random
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT), str(ROOT / 'tests')]
import gnss_fusion as gf  # noqa: E402
import user_rules as rules  # noqa: E402
from test_secondary_position import ANT, BASE, config  # noqa: E402


def bump(x, peak=3.0, start=6., end=24., ramp=2.):
    """测向在 start–end 米比真车头多偏多少度（两头各 ramp 米过渡）。"""
    if x < start or x > end:
        return 0.
    return peak * min(1., (x - start) / ramp, (end - x) / ramp)


def drive(check=True, dual=bump, seconds=70., speed=.5, imu=True, seed=3, fix_noise=.003, **overrides):
    """返回 (融合, [(x, 融合输出车头°, 融合输出横偏 m)])。"""
    rng = random.Random(seed)
    f = gf.GnssFusion(dataclasses.replace(config(False), heading_course_check=check, **overrides))
    f.set_origin(31., 120.)
    t, rows = 0., []
    for i in range(int(seconds * 20)):
        t = round(t + .05, 6)
        x = speed * t
        f.add_wheel(gf.WheelSample(t, speed, 0.))
        if imu:
            f.add_imu(gf.ImuSample(t, 0., 0.))
        if i % 2 == 0:
            f.add_heading(gf.DualHeading(t, 180. - dual(x), 0., BASE, 'NARROW_INT', .3, 30))
        lat, lon = f.to_geodetic(x + ANT[0] + rng.gauss(0, fix_noise), ANT[1] + rng.gauss(0, fix_noise))
        f.add_fix(gf.GnssFix(t, lat, lon, 0., 'NARROW_INT', .01, .01, .02, 1., 30))
        pose = f.pose(t)
        if pose is not None:
            rows.append((x, math.degrees(pose.yaw), pose.y))
    return f, rows


def worst(rows, lo=5., hi=30.):
    return (max(abs(r[1]) for r in rows if lo <= r[0] <= hi), max(abs(r[2]) for r in rows if lo <= r[0] <= hi))


class HeadingCourseCheck(unittest.TestCase):
    def test_rules_table(self):
        cfg = gf.FusionConfig()
        self.assertIs(cfg.heading_course_check, rules.HEADING_COURSE_CHECK)
        self.assertEqual((cfg.heading_course_window_m, cfg.heading_course_dual_s, cfg.heading_course_suspect_deg,
                          cfg.heading_course_release_deg, cfg.heading_course_agree_deg, cfg.heading_course_base_tau_s),
                         (rules.HEADING_COURSE_WINDOW_M, rules.HEADING_COURSE_DUAL_S, rules.HEADING_COURSE_SUSPECT_DEG,
                          rules.HEADING_COURSE_RELEASE_DEG, rules.HEADING_COURSE_AGREE_DEG, rules.HEADING_COURSE_BASE_TAU_S))
        with self.assertRaises(ValueError):
            gf.FusionConfig(heading_course_check=1)
        with self.assertRaises(ValueError):
            gf.FusionConfig(heading_course_release_deg=2.)
        with self.assertRaises(ValueError):
            gf.FusionConfig(heading_course_base_tau_s=0.)

    def test_bent_dual_heading_no_longer_drags_the_fusion(self):
        # 复现 48 m 线：测向中段偏 3°。原来融合车头跟过去约 3°、位置推偏约 4 cm；现在只在刚偏起来那 1 m（还没判上，
        # 测向要攒 2 s 平均）被拖不到 1°，判上以后拉回来、≤ 0.3°，位置 ≤ 1 cm
        _, old = drive(check=False)
        f, new = drive(check=True)
        yaw_old, y_old = worst(old)
        yaw_new, y_new = worst(new)
        self.assertGreater(yaw_old, 2.5, yaw_old)
        self.assertGreater(y_old, .03, y_old)
        self.assertLess(yaw_new, 1., yaw_new)
        self.assertLess(worst(new, lo=9.)[0], .3)
        self.assertLess(y_new, .01, y_new)
        self.assertLess(y_new, y_old)
        cal = f.calibration()['heading_filter']
        self.assertGreater(cal['course_replaced'], 0)

    def test_follows_dual_again_after_it_recovers(self):
        # 测向回来以后不再判：后半段融合车头回到真车头附近
        f, rows = drive(check=True)
        tail = [abs(r[1]) for r in rows if r[0] > 30.]
        self.assertLess(max(tail), .3)
        self.assertFalse(f.calibration()['heading_filter']['course_suspect'])

    def test_good_dual_is_unchanged(self):
        # 测向没偏：开不开这个判据，融合输出一样
        _, old = drive(check=False, dual=lambda x: 0.)
        f, new = drive(check=True, dual=lambda x: 0.)
        self.assertEqual(old, new)
        self.assertEqual(f.calibration()['heading_filter']['course_replaced'], 0)

    def test_steady_offset_from_the_start_is_left_to_bias_learning(self):
        # 一上来测向就偏 1.5°（安装偏差那种，不是中途被带偏）：IMU 参照跟着融合一起偏，不判，照旧
        _, old = drive(check=False, dual=lambda x: 1.5)
        f, new = drive(check=True, dual=lambda x: 1.5)
        self.assertEqual(old, new)
        self.assertEqual(f.calibration()['heading_filter']['course_replaced'], 0)

    def test_without_imu_trusts_the_dual_heading(self):
        # IMU 断着（10-05 规矩：没有 IMU 也能靠卫星作业）：照旧全信测向
        _, old = drive(check=False, imu=False)
        f, new = drive(check=True, imu=False)
        self.assertEqual(old, new)
        self.assertEqual(f.calibration()['heading_filter']['course_replaced'], 0)


if __name__ == '__main__':
    unittest.main()
