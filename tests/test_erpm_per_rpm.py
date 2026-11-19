"""驱动器转速换算比 erpm_per_rpm：和磁极对数分开（2026-11-19 磁极对数 10、实测换算比约 5）。"""
import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'oid_protocol'))

from oid_link import WheelConfig  # noqa: E402
from robot_core.wheel_motion import HALL_STEPS_PER_ELECTRICAL_TURN, WheelMotionAligner  # noqa: E402


class ErpmPerRpm(unittest.TestCase):
    def test_default_is_pole_pairs(self):
        w = WheelConfig(wheel_radius_m=0.194, pole_pairs=20)
        rpm = 0.3 / (2 * math.pi * 0.194) * 60
        self.assertEqual(w.to_erpm(0.3), round(rpm * 20))

    def test_measured_ratio_drives_speed_not_pole_pairs(self):
        w = WheelConfig(wheel_radius_m=0.194, pole_pairs=10, erpm_per_rpm=5.0)
        rpm = 0.3 / (2 * math.pi * 0.194) * 60          # 14.77 圈 / 分
        self.assertEqual(w.to_erpm(0.3), round(rpm * 5))
        self.assertAlmostEqual(w.to_mps(w.to_erpm(0.3)), 0.3, places=2)
        # 11-19 实测：295 erpm 时电机每分钟 59.7 圈 → 按 5 换算轮速 1.2 m/s，不是 0.3
        self.assertAlmostEqual(w.to_mps(295), 295 / 5 / 60 * 2 * math.pi * 0.194, places=6)

    def test_position_and_hall_step_follow_pole_pairs(self):
        w = WheelConfig(wheel_radius_m=0.194, pole_pairs=10, erpm_per_rpm=5.0)
        self.assertAlmostEqual(w.centideg_to_m(36000), 2 * math.pi * 0.194)   # 驱动器位置是电机轴机械角
        a = WheelMotionAligner(w)
        self.assertAlmostEqual(a.step, w.centideg_to_m(36000 / (HALL_STEPS_PER_ELECTRICAL_TURN * 10)))

    def test_bad_ratio_rejected(self):
        for bad in (-1.0, 101.0, float('nan')):
            with self.assertRaises(ValueError):
                WheelConfig(pole_pairs=10, erpm_per_rpm=bad)


if __name__ == '__main__':
    unittest.main()
