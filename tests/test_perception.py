#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""感知层加固的回归测试:野值门控、目标锁定、相机/雷达证伪、稳健深度。

每条测试都对应一个能让车撞人或跟错人的具体场景。
不需要 ROS、不需要相机、不需要实车:

    python3 tests/test_perception.py
"""

import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "radar_system"))

from motion_safety import (  # noqa: E402
    AlphaBetaTracker, TargetLock, ScanSectors, reconcile_range,
    BrakeProfile, brake_envelope,
)


# =============================================================================
# 野值门控 —— 「单帧坏深度让车全速冲出去」
# =============================================================================

class TestInnovationGate(unittest.TestCase):

    def _settled(self, closing_rate=-0.4, start=2.0, frames=40, dt=0.1):
        """先喂一段干净的匀速接近数据,让滤波器收敛。"""
        t = AlphaBetaTracker(alpha=0.45, beta=0.10)
        z = start
        for i in range(frames):
            z += closing_rate * dt
            t.update(z, i * dt)
        return t, z, frames * dt, dt

    def test_single_outlier_is_rejected(self):
        """0.8m 处突然读到 3.0m(ROI 打在背景墙上),必须被挡住。"""
        tracker, z, t, dt = self._settled()
        before = tracker.position
        tracker.update(z + 2.2, t)          # 野值:突然远了 2.2 米
        self.assertFalse(tracker.last_accepted)
        self.assertTrue(tracker.coasting)
        # 位置只按预测外推,不能被野值拉走
        self.assertAlmostEqual(tracker.position, before + tracker.velocity * dt,
                               delta=0.01)

    def test_outlier_would_have_unlocked_full_speed_without_gate(self):
        """量化没有门控时的后果:滤波位置被拉远,刹车包络直接放行满速。"""
        profile = BrakeProfile(decel_mps2=1.0, latency_s=0.35, stop_m=0.70)
        true_z, bad_z, alpha = 0.80, 3.00, 0.45

        # 无门控的 alpha-beta:新观测的 45% 被立刻采信
        naive = true_z + alpha * (bad_z - true_z)
        self.assertGreater(brake_envelope(naive, profile), 0.55,
                           "无门控时包络会放行满速")

        # 有门控:观测被拒,位置基本不动,包络仍然把车压住
        gated = AlphaBetaTracker(alpha=alpha, beta=0.10)
        gated.update(true_z, 0.0)
        gated.update(true_z - 0.04, 0.1)
        gated.update(bad_z, 0.2)
        self.assertFalse(gated.last_accepted)
        self.assertLess(brake_envelope(gated.position, profile), 0.30)

    def test_persistent_jump_eventually_accepted(self):
        """目标真的瞬移(换人/重新捕获)时,连续几帧后必须接受,不能永远卡住。"""
        tracker, z, t, dt = self._settled()
        for k in range(1, 5):
            tracker.update(z + 2.2, t + k * dt)
        self.assertAlmostEqual(tracker.position, z + 2.2, delta=0.05)
        self.assertFalse(tracker.coasting)

    def test_legitimate_fast_approach_not_rejected(self):
        """门控不能误伤:人快步走来(2 m/s 接近)应当全程被接受。"""
        tracker = AlphaBetaTracker(alpha=0.45, beta=0.10)
        dt, z = 0.1, 4.0
        rejects = 0
        for i in range(30):
            z -= 2.0 * dt
            tracker.update(z, i * dt)
            if not tracker.last_accepted:
                rejects += 1
        self.assertLessEqual(rejects, 2, "正常快速接近不该被门控频繁拒绝")

    def test_gate_widens_with_longer_interval(self):
        tracker = AlphaBetaTracker(gate_base_m=0.35, gate_rate_mps=2.5)
        self.assertAlmostEqual(tracker.gate_width(0.1), 0.35, places=6)
        self.assertAlmostEqual(tracker.gate_width(0.4), 1.00, places=6)

    def test_counts_are_reported(self):
        tracker, z, t, dt = self._settled()
        tracker.update(z + 2.2, t)
        self.assertEqual(tracker.rejected_total, 1)


# =============================================================================
# 目标锁定 —— 「房间里走过第二个人,车跟错了」
# =============================================================================

def person(x, z, conf=0.9):
    return {'x': x, 'z': z, 'conf': conf, 'label': 'person'}


class TestTargetLock(unittest.TestCase):

    def setUp(self):
        self.lock = TargetLock(assoc_radius_m=0.55, lost_timeout_s=1.5,
                               confirm_frames=3)

    def _acquire(self, x=0.0, z=1.5, t0=0.0):
        for i in range(3):
            got = self.lock.update([person(x, z)], t0 + i * 0.1, 0.9)
        return got

    def test_requires_consecutive_frames_to_lock(self):
        self.assertIsNone(self.lock.update([person(0.0, 1.5)], 0.0, 0.9))
        self.assertIsNone(self.lock.update([person(0.0, 1.5)], 0.1, 0.9))
        self.assertIsNotNone(self.lock.update([person(0.0, 1.5)], 0.2, 0.9))
        self.assertTrue(self.lock.locked)

    def test_flickering_detection_does_not_lock(self):
        """位置跳来跳去的检测(噪点)不该被锁定。"""
        for i in range(8):
            x = 1.5 if i % 2 else -1.5
            self.lock.update([person(x, 2.0)], i * 0.1, 0.9)
        self.assertFalse(self.lock.locked)

    def test_does_not_switch_to_a_closer_stranger(self):
        """核心场景:锁定 A 之后,一个更近更正的陌生人 B 走进画面,不能换人。"""
        self._acquire(x=0.0, z=2.0)
        a, b = person(0.05, 1.95), person(-0.02, 0.9)   # B 更近、更居中
        got = self.lock.update([a, b], 0.4, 0.9)
        self.assertEqual(got['z'], a['z'], "应当继续跟 A,而不是被 B 抢走")

    def test_returns_none_when_association_fails(self):
        self._acquire(x=0.0, z=2.0)
        far = person(2.5, 2.0)          # 离锚点 2.5m,超出关联半径
        self.assertIsNone(self.lock.update([far], 0.4, 0.9))
        self.assertTrue(self.lock.locked, "关联失败只算丢一帧,不应立刻解锁")

    def test_unlocks_after_timeout_and_can_reacquire(self):
        self._acquire(x=0.0, z=2.0, t0=0.0)
        self.lock.update([], 2.5, 0.9)              # 超过 lost_timeout
        self.assertFalse(self.lock.locked)
        for i in range(3):
            got = self.lock.update([person(1.0, 1.2)], 3.0 + i * 0.1, 0.9)
        self.assertIsNotNone(got)
        self.assertTrue(self.lock.locked)

    def test_tracks_target_that_moves_steadily(self):
        self._acquire(x=0.0, z=2.0)
        z, t = 2.0, 0.3
        for _ in range(20):
            z -= 0.05
            t += 0.1
            got = self.lock.update([person(0.0, z), person(1.2, 1.0)], t, 0.9)
            self.assertIsNotNone(got)
            self.assertAlmostEqual(got['z'], z, places=6)

    def test_empty_candidates_is_safe(self):
        self.assertIsNone(self.lock.update([], 0.0, 0.9))

    def test_one_missed_frame_does_not_reset_acquisition(self):
        self.assertIsNone(self.lock.update([person(0.0, 1.5)], 0.0, 0.9))
        self.assertIsNone(self.lock.update([], 0.1, 0.9))
        self.assertIsNone(self.lock.update([person(0.02, 1.48)], 0.2, 0.9))
        got = self.lock.update([person(0.01, 1.49)], 0.3, 0.9)
        self.assertIsNotNone(got)
        self.assertTrue(self.lock.locked)

    def test_long_detection_gap_resets_acquisition(self):
        self.lock.update([person(0.0, 1.5)], 0.0, 0.9)
        self.lock.update([], 0.5, 0.9)
        self.assertIsNone(self.lock.update([person(0.0, 1.5)], 0.6, 0.9))
        self.assertFalse(self.lock.locked)


# =============================================================================
# 相机 / 雷达交叉证伪 —— 「相机读远了,雷达明明看见近处有东西」
# =============================================================================

class TestReconcileRange(unittest.TestCase):

    def test_lidar_closer_wins(self):
        used, conflict = reconcile_range(2.50, 0.70)
        self.assertEqual(used, 0.70)
        self.assertTrue(conflict, "差 1.8m 应当判为冲突")

    def test_small_disagreement_is_not_a_conflict(self):
        """腿比躯干近一点是正常的,不该动不动报冲突。"""
        used, conflict = reconcile_range(1.20, 0.95)
        self.assertEqual(used, 0.95)
        self.assertFalse(conflict)

    def test_lidar_farther_keeps_camera(self):
        """雷达平面以上的躯干被相机看到、雷达扫不到,属正常。"""
        used, conflict = reconcile_range(0.95, 1.20)
        self.assertEqual(used, 0.95)
        self.assertFalse(conflict)

    def test_missing_lidar_passes_through(self):
        for bad in (None, float('nan'), 0.0, -1.0):
            used, conflict = reconcile_range(1.0, bad)
            self.assertEqual(used, 1.0)
            self.assertFalse(conflict)

    def test_conflict_margin_is_configurable(self):
        _, conflict = reconcile_range(2.0, 1.3, conflict_margin_m=1.0)
        self.assertFalse(conflict)
        _, conflict = reconcile_range(2.0, 1.3, conflict_margin_m=0.5)
        self.assertTrue(conflict)


# =============================================================================
# 分扇区雷达 —— 「走廊两侧的墙不该把速度拉低」
# =============================================================================

class TestScanSectors(unittest.TestCase):

    def setUp(self):
        self.s = ScanSectors(half_fov_deg=60.0, bin_deg=5.0)

    def test_corridor_walls_do_not_affect_forward_query(self):
        self.s.add(math.radians(55), 0.45)      # 左墙很近
        self.s.add(math.radians(-55), 0.45)     # 右墙很近
        self.s.add(0.0, 3.00)                   # 正前方空旷
        self.assertAlmostEqual(self.s.min_within(math.radians(10)), 3.00, places=6)

    def test_query_near_bearing(self):
        self.s.add(math.radians(30), 0.80)
        self.assertAlmostEqual(
            self.s.min_near(math.radians(30), math.radians(6)), 0.80, places=6)
        self.assertIsNone(
            self.s.min_near(math.radians(-30), math.radians(6)))

    def test_keeps_minimum_per_bin(self):
        self.s.add(0.0, 2.0)
        self.s.add(0.01, 1.2)
        self.s.add(0.02, 2.5)
        self.assertAlmostEqual(self.s.min_within(math.radians(3)), 1.2, places=6)

    def test_out_of_field_is_ignored(self):
        self.s.add(math.radians(120), 0.1)      # 车身后方
        self.assertIsNone(self.s.min_within(math.radians(60)))

    def test_clear_resets(self):
        self.s.add(0.0, 1.0)
        self.s.clear()
        self.assertIsNone(self.s.min_within(math.radians(10)))


# =============================================================================
# 稳健深度提取 —— 「5 个背景像素决定了距离」
# =============================================================================

try:
    import numpy as np
    HAVE_NUMPY = True
except ImportError:
    HAVE_NUMPY = False


class _Depth:
    """只取 ai_3d_detector 里的深度提取常量与算法,避免测试依赖 ROS/cv2。

    这些常量与 Detector3D 类中的定义保持一致;如果改了那边,这里也要改,
    test_constants_match_detector 会在两边失配时报警。
    """
    DEPTH_MIN_MM = 150.0
    DEPTH_MAX_MM = 6000.0
    DEPTH_INSET = 0.20
    DEPTH_PERCENTILE = 20.0
    DEPTH_MIN_VALID_RATIO = 0.30
    DEPTH_MIN_PIXELS = 60

    @classmethod
    def robust_depth(cls, depth, x1, y1, x2, y2):
        h, w = depth.shape[:2]
        bw, bh = x2 - x1, y2 - y1
        if bw <= 0 or bh <= 0:
            return None, 0.0
        ix1 = max(0, int(x1 + bw * cls.DEPTH_INSET))
        ix2 = min(w, int(x2 - bw * cls.DEPTH_INSET))
        iy1 = max(0, int(y1 + bh * cls.DEPTH_INSET))
        iy2 = min(h, int(y2 - bh * cls.DEPTH_INSET))
        if ix2 - ix1 < 2 or iy2 - iy1 < 2:
            return None, 0.0
        roi = depth[iy1:iy2, ix1:ix2]
        if roi.size < cls.DEPTH_MIN_PIXELS:
            return None, 0.0
        mask = (roi > cls.DEPTH_MIN_MM) & (roi < cls.DEPTH_MAX_MM)
        valid = roi[mask]
        ratio = float(len(valid)) / float(roi.size)
        if ratio < cls.DEPTH_MIN_VALID_RATIO or len(valid) < cls.DEPTH_MIN_PIXELS:
            return None, ratio
        return float(np.percentile(valid, cls.DEPTH_PERCENTILE)) / 1000.0, ratio


@unittest.skipUnless(HAVE_NUMPY, "需要 numpy")
class TestRobustDepth(unittest.TestCase):

    def test_sparse_background_pixels_are_rejected(self):
        """旧版的致命场景:ROI 几乎全空,只有几个像素打在 4m 外的背景墙上。

        旧版 len(valid) > 4 就采信,得出"人在 4m 外"并全速前进。
        新版要求有效占比达标,这一帧应当被判为无效观测。
        """
        depth = np.zeros((240, 320), dtype=np.uint16)
        depth[100:104, 150:154] = 4000          # 16 个背景像素
        z, ratio = _Depth.robust_depth(depth, 120, 60, 200, 200)
        self.assertIsNone(z, "有效像素太稀疏时必须拒绝")
        self.assertLess(ratio, _Depth.DEPTH_MIN_VALID_RATIO)

    def test_normal_person_gives_sane_depth(self):
        depth = np.full((240, 320), 800, dtype=np.uint16)
        z, ratio = _Depth.robust_depth(depth, 120, 60, 200, 200)
        self.assertAlmostEqual(z, 0.80, places=3)
        self.assertAlmostEqual(ratio, 1.0, places=3)

    def test_percentile_biases_toward_the_nearer_surface(self):
        """一半前景(0.8m)一半背景(3.5m)时,必须偏向近的那个。

        中位数在这种分布上会落在两者之间甚至偏向远处,把人判得比实际远 ——
        车就会多往前开一段。第 20 百分位保证宁近勿远。
        """
        depth = np.empty((240, 320), dtype=np.uint16)
        depth[:, :160] = 800
        depth[:, 160:] = 3500
        z, _ = _Depth.robust_depth(depth, 60, 40, 260, 200)
        self.assertLess(z, 1.0, "应当采信近处的前景,而不是被背景拉远")

    def test_out_of_range_values_are_excluded(self):
        depth = np.full((240, 320), 9000, dtype=np.uint16)   # 超出量程的垃圾值
        depth[60:200, 120:200] = 1000
        z, _ = _Depth.robust_depth(depth, 120, 60, 200, 200)
        self.assertAlmostEqual(z, 1.00, places=3)

    def test_all_garbage_is_rejected(self):
        depth = np.full((240, 320), 9000, dtype=np.uint16)
        z, _ = _Depth.robust_depth(depth, 120, 60, 200, 200)
        self.assertIsNone(z)

    def test_zero_depth_holes_are_rejected(self):
        depth = np.zeros((240, 320), dtype=np.uint16)
        z, ratio = _Depth.robust_depth(depth, 120, 60, 200, 200)
        self.assertIsNone(z)
        self.assertEqual(ratio, 0.0)

    def test_tiny_box_is_rejected(self):
        depth = np.full((240, 320), 800, dtype=np.uint16)
        z, _ = _Depth.robust_depth(depth, 100, 100, 104, 104)
        self.assertIsNone(z)

    def test_constants_match_detector(self):
        """本测试里的常量必须与 ai_3d_detector.py 保持一致。"""
        import re
        with open(os.path.join(ROOT, "radar_system", "ai_3d_detector.py")) as fh:
            src = fh.read()
        for name in ("DEPTH_MIN_MM", "DEPTH_MAX_MM", "DEPTH_INSET",
                     "DEPTH_PERCENTILE", "DEPTH_MIN_VALID_RATIO",
                     "DEPTH_MIN_PIXELS"):
            m = re.search(rf"^    {name} = ([0-9.]+)", src, re.M)
            self.assertIsNotNone(m, f"{name} 未在检测器中找到")
            self.assertAlmostEqual(float(m.group(1)), float(getattr(_Depth, name)),
                                   places=6, msg=f"{name} 两边不一致")


if __name__ == "__main__":
    unittest.main(verbosity=2)
