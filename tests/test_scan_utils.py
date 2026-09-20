#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""雷达扇区 helpers 的回归测试。

验证扇区跨越 0 度、无有效回波以及 angle_min 偏移等边界。

    python3 tests/test_scan_utils.py
"""

import array
import os
import sys
import time
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "radar_system"))

from scan_utils import (  # noqa: E402
    clean_ranges, sector_min,
)

try:
    import numpy as np
    HAVE_NUMPY = True
except ImportError:
    HAVE_NUMPY = False


@unittest.skipUnless(HAVE_NUMPY, "需要 numpy")
class TestCleanRanges(unittest.TestCase):

    def test_marks_out_of_band_as_nan(self):
        raw = array.array('f', [0.05, 1.5, 30.0, 2.0])
        clean, ok = clean_ranges(raw, 0.1, 12.0)
        self.assertTrue(np.isnan(clean[0]))     # 低于下界
        self.assertTrue(np.isnan(clean[2]))     # 高于上界
        self.assertEqual(list(ok), [False, True, False, True])

    def test_marks_inf_and_nan(self):
        raw = array.array('f', [float('inf'), float('nan'), 1.0])
        clean, ok = clean_ranges(raw, 0.1, 12.0)
        self.assertTrue(np.isnan(clean[0]))
        self.assertTrue(np.isnan(clean[1]))
        self.assertTrue(bool(ok[2]))

    def test_accepts_plain_list(self):
        clean, ok = clean_ranges([1.0, 2.0], 0.1, 12.0)
        self.assertEqual(int(ok.sum()), 2)


@unittest.skipUnless(HAVE_NUMPY, "需要 numpy")
class TestSectorMin(unittest.TestCase):

    def _ring(self, n=360):
        """构造一圈:每一度的距离等于 5.0,便于按索引放置特征点。"""
        return np.full(n, 5.0, dtype=np.float32)

    def test_picks_minimum_in_range(self):
        r = self._ring()
        r[90] = 1.2
        self.assertAlmostEqual(sector_min(r, 75, 105), 1.2, places=3)

    def test_ignores_outside_the_sector(self):
        r = self._ring()
        r[180] = 0.3                      # 正后方有东西
        self.assertAlmostEqual(sector_min(r, 75, 105), 5.0, places=3)

    def test_wraps_across_zero(self):
        """正前方扇区 345°~15° 跨越 0 度,这是最容易写错的一条。"""
        r = self._ring()
        r[350] = 0.8
        self.assertAlmostEqual(sector_min(r, 345, 15), 0.8, places=3)
        r = self._ring()
        r[5] = 0.6
        self.assertAlmostEqual(sector_min(r, 345, 15), 0.6, places=3)

    def test_wrap_sector_excludes_the_far_side(self):
        r = self._ring()
        r[180] = 0.1
        self.assertAlmostEqual(sector_min(r, 345, 15), 5.0, places=3)

    def test_all_invalid_returns_sentinel(self):
        r = np.full(360, np.nan, dtype=np.float32)
        self.assertEqual(sector_min(r, 0, 90), 99.0)

    def test_empty_array_returns_sentinel(self):
        self.assertEqual(sector_min(np.array([], dtype=np.float32), 0, 90), 99.0)

    def test_full_circle(self):
        r = self._ring()
        r[200] = 0.4
        self.assertAlmostEqual(sector_min(r, 0, 360), 0.4, places=3)

    def test_works_with_non_360_point_scans(self):
        """N10P 之类的雷达一圈不一定是 360 个点。"""
        n = 897
        r = np.full(n, 5.0, dtype=np.float32)
        r[int(90 / 360.0 * n) + 1] = 1.1
        self.assertAlmostEqual(sector_min(r, 75, 105), 1.1, places=3)


@unittest.skipUnless(HAVE_NUMPY, "需要 numpy")
class TestAngleMinHandling(unittest.TestCase):
    """实车 bug 回归:忽略 angle_min 会让整圈数据旋转,前后颠倒。

    LaserScan 的第 0 个光束指向 msg.angle_min,不是 0°。N10P 按惯例发布
    angle_min = -π,这时光束序号 0 指向**车尾**。

    改造前写的是 `int(deg / 360 * n)`,等于假设 angle_min = 0,于是:
      - 「正前方测距」读的其实是车尾
      - 雷达装在车头,往后扫到自己的车身(实测 0.17m)
      - 这个 0.17m 被当成正前方的障碍物 -> AEB 一直硬刹停
      - 而人眼看前方明明空无一物
    """

    def _ring(self, n=720):
        """构造一圈,每 0.5° 一个光束,全部 5.0m。"""
        return np.full(n, 5.0, dtype=np.float32)

    def _index_of(self, deg, n=720, amin_deg=-180.0):
        return int(round((deg - amin_deg) / 360.0 * n)) % n

    def test_front_query_with_negative_angle_min(self):
        """angle_min=-180 时,真实正前方的光束在数组中间,不在开头。"""
        r = self._ring()
        r[self._index_of(0.0)] = 1.5          # 正前方放一个近点
        self.assertAlmostEqual(
            sector_min(r, 345, 15, angle_min_deg=-180.0), 1.5, places=3)

    def test_rear_self_reflection_is_not_seen_as_front(self):
        """核心回归:车尾的自反射不能被算进前向扇区。"""
        r = self._ring()
        for deg in (165.0, 177.0, 190.0):     # 实测自反射所在方位
            r[self._index_of(deg)] = 0.17
        front = sector_min(r, 345, 15, angle_min_deg=-180.0)
        self.assertGreater(front, 1.0, "前方应当是干净的")
        rear = sector_min(r, 165, 195, angle_min_deg=-180.0)
        self.assertAlmostEqual(rear, 0.17, places=3, msg="车尾应当看得到自反射")

    def test_ignoring_angle_min_flips_front_and_rear(self):
        """量化旧写法的后果:不传 angle_min,前后完全颠倒。"""
        r = self._ring()
        for deg in (165.0, 177.0, 190.0):
            r[self._index_of(deg)] = 0.17

        correct = sector_min(r, 345, 15, angle_min_deg=-180.0)
        buggy = sector_min(r, 345, 15)        # 旧写法:默认 angle_min=0
        self.assertGreater(correct, 1.0)
        self.assertAlmostEqual(buggy, 0.17, places=3,
                               msg="旧写法会把车尾的自己当成正前方障碍物")

    def test_zero_angle_min_is_unaffected(self):
        """angle_min 本来就是 0 的雷达,行为不变。"""
        r = self._ring()
        r[self._index_of(0.0, amin_deg=0.0)] = 1.5
        self.assertAlmostEqual(sector_min(r, 345, 15, angle_min_deg=0.0),
                               1.5, places=3)

    def test_all_four_quadrants_map_correctly(self):
        for deg, name in ((0.0, "前"), (90.0, "左"), (180.0, "后"), (270.0, "右")):
            r = self._ring()
            r[self._index_of(deg)] = 0.8
            got = sector_min(r, deg - 10, deg + 10, angle_min_deg=-180.0)
            self.assertAlmostEqual(got, 0.8, places=3, msg=f"{name}方位映射错误")


if __name__ == "__main__":
    unittest.main(verbosity=2)
