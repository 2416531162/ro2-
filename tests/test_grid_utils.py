#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""雷达扇区与占据栅格 helpers 的回归测试。

这两段逻辑原来内联在 radar_web_server.py 的回调里,没法测,而且都有容易
写错的边界情况:扇区跨越 0 度、地图全空、点数爆炸。

    python3 tests/test_grid_utils.py
"""

import array
import os
import sys
import time
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "radar_system"))

from grid_utils import (  # noqa: E402
    clean_ranges, sector_min, downsample_step, extract_grid_points,
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


class TestDownsampleStep(unittest.TestCase):

    def test_small_map_keeps_finest_step(self):
        self.assertEqual(downsample_step(200, 200, 12000), 2)

    def test_step_grows_with_map_size(self):
        steps = [downsample_step(w, w, 12000) for w in (200, 400, 800, 1600)]
        self.assertEqual(steps, sorted(steps))
        self.assertLess(steps[0], steps[-1])

    def test_respects_upper_limit(self):
        self.assertLessEqual(downsample_step(100000, 100000, 12000, limit=32), 32)

    def test_point_budget_is_respected(self):
        for w in (200, 400, 800, 1200, 2000):
            st = downsample_step(w, w, 12000)
            self.assertLessEqual((w // st) * (w // st), 12000 * 2,
                                 f"{w}x{w} step={st} 超出点数预算")


@unittest.skipUnless(HAVE_NUMPY, "需要 numpy")
class TestExtractGridPoints(unittest.TestCase):

    def _grid(self, w, h, fill=-1):
        return array.array('b', [fill] * (w * h))

    def test_classifies_obstacles_and_free_space(self):
        w = h = 8
        g = self._grid(w, h)
        g[2 * w + 4] = 100
        g[6 * w + 2] = 0
        obstacles, frees = extract_grid_points(g, w, h, 1)
        self.assertIn([4, 2], obstacles)
        self.assertIn([2, 6], frees)

    def test_unknown_cells_are_dropped(self):
        w = h = 8
        obstacles, frees = extract_grid_points(self._grid(w, h), w, h, 1)
        self.assertEqual(obstacles, [])
        self.assertEqual(frees, [])

    def test_coordinates_are_in_original_grid_units(self):
        """抽样之后坐标必须乘回去,前端不需要知道抽样倍率。"""
        w = h = 16
        g = self._grid(w, h)
        g[8 * w + 4] = 100
        obstacles, _ = extract_grid_points(g, w, h, 4)
        self.assertIn([4, 8], obstacles)

    def test_degenerate_inputs_are_safe(self):
        self.assertEqual(extract_grid_points(self._grid(4, 4), 0, 4, 2), ([], []))
        self.assertEqual(extract_grid_points(self._grid(4, 4), 4, 4, 0), ([], []))

    def test_truncated_data_is_rejected(self):
        """消息声明的尺寸与实际数据不符时不能崩,也不能读越界。"""
        self.assertEqual(extract_grid_points(self._grid(4, 4), 16, 16, 2), ([], []))

    def test_numpy_and_pure_python_paths_agree(self):
        import grid_utils
        w = h = 24
        g = self._grid(w, h)
        for i in (10, 57, 200, 401, 500):
            g[i] = 100 if i % 2 else 0
        fast = extract_grid_points(g, w, h, 2)
        saved = grid_utils.np
        try:
            grid_utils.np = None                    # 强制走纯 Python 分支
            slow = extract_grid_points(g, w, h, 2)
        finally:
            grid_utils.np = saved
        self.assertEqual(sorted(fast[0]), sorted(slow[0]))
        self.assertEqual(sorted(fast[1]), sorted(slow[1]))


@unittest.skipUnless(HAVE_NUMPY, "需要 numpy")
class TestPerformanceBudget(unittest.TestCase):
    """回归保护:这段代码持有 GIL,慢下来会拖垮同进程的网页遥控。"""

    def test_large_map_stays_fast(self):
        w = h = 1200
        g = array.array('b', [(-1, 0, 0, 0, 100)[i % 5] for i in range(w * h)])
        step = downsample_step(w, h, 12000)
        t0 = time.perf_counter()
        obstacles, frees = extract_grid_points(g, w, h, step)
        elapsed_ms = (time.perf_counter() - t0) * 1000
        # 改造前同样的地图固定 step=2,要 180ms 且输出 28.8 万点。
        # 这里放宽到 150ms 是给多进程高负载下的 RK3588 留足余量。
        self.assertLess(elapsed_ms, 150.0, f"耗时 {elapsed_ms:.1f}ms,过慢")
        self.assertLess(len(obstacles) + len(frees), 12000 * 3)


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
