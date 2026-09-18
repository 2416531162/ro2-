#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跟踪人这一层的回归测试:纯追踪转向律、自车运动补偿、轻量外观重识别。

每条测试都对应一个实车上看得见的行为,不需要 ROS、不需要相机:

    python3 tests/test_tracking.py
"""

import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "radar_system"))

from motion_safety import (  # noqa: E402
    ChassisGeometry, TargetLock, pure_pursuit_steer, yaw_from_steer,
    appearance_similarity,
)


def polar(distance_m, bearing_rad):
    """把「多远、偏多少」换成纯追踪要的 (前向, 左向)。"""
    return distance_m * math.cos(bearing_rad), distance_m * math.sin(bearing_rad)


# =============================================================================
# 纯追踪 —— 「跟人画龙、左右摇摆」
# =============================================================================

class TestPurePursuit(unittest.TestCase):

    def setUp(self):
        self.geo = ChassisGeometry()

    def test_straight_ahead_is_zero_steer(self):
        self.assertEqual(pure_pursuit_steer(2.0, 0.0, self.geo), 0.0)

    def test_sign_follows_target_side(self):
        self.assertGreater(pure_pursuit_steer(2.0, 0.4, self.geo), 0.0)
        self.assertLess(pure_pursuit_steer(2.0, -0.4, self.geo), 0.0)

    def test_same_bearing_needs_less_steer_when_farther(self):
        """老式子 kp*bearing 的核心错误:同样的视线角,远近给一样的舵。

        远处那个给多了,车冲过头再反打 —— 这就是画龙。
        """
        near = pure_pursuit_steer(*polar(1.6, 0.30), geometry=self.geo)
        far = pure_pursuit_steer(*polar(3.5, 0.30), geometry=self.geo)
        self.assertLess(far, near)
        self.assertLess(far, 0.5 * near)

    def test_matches_closed_form_geometry(self):
        """解出来的转角经 yaw_from_steer 换回半径,必须还原纯追踪的 R。"""
        fx, fy = polar(2.4, 0.25)
        steer = pure_pursuit_steer(fx, fy, self.geo)
        self.assertLess(abs(steer), self.geo.max_steer_rad)
        speed = 0.4
        yaw = yaw_from_steer(speed, steer, self.geo)
        radius_cmd = speed / abs(yaw)
        radius_geom = (fx * fx + fy * fy) / (2.0 * abs(fy))     # Ld^2 / (2*Ld*sin a)
        self.assertAlmostEqual(radius_cmd, radius_geom, places=6)

    def test_never_exceeds_servo_limit(self):
        for bearing in (0.1, 0.5, 1.0, 1.4):
            for dist in (0.3, 0.8, 2.0, 6.0):
                steer = pure_pursuit_steer(*polar(dist, bearing), geometry=self.geo)
                self.assertLessEqual(abs(steer), self.geo.max_steer_rad + 1e-9)

    def test_close_target_does_not_explode(self):
        """贴脸时 Ld^2 会让曲率爆掉。钳住前视距离,不准变成原地乱转。"""
        steer = pure_pursuit_steer(0.05, 0.02, self.geo)
        self.assertLessEqual(abs(steer), self.geo.max_steer_rad + 1e-9)

    def test_gain_scales_aggressiveness(self):
        fx, fy = polar(3.0, 0.2)
        soft = pure_pursuit_steer(fx, fy, self.geo, gain=0.5)
        hard = pure_pursuit_steer(fx, fy, self.geo, gain=1.0)
        self.assertLess(soft, hard)


# =============================================================================
# 自车运动补偿 —— 「一转弯就掉锁」
# =============================================================================

class TestEgoCompensation(unittest.TestCase):

    def _lock(self, **kw):
        kw.setdefault('confirm_frames', 1)
        kw.setdefault('origin_offset_m', 0.67)
        return TargetLock(**kw)

    @staticmethod
    def _turn_sim(lock, frame_dt, yaw=1.2, speed=0.0, z0=2.0, frames=6,
                  origin=0.67, use_ego=True):
        """人站着不动,车原地转向。返回关联失败的帧数。

        静止目标在旋转的车体系里的坐标是精确可算的,所以补偿做对了就应当
        一帧都不丢;做错了误差会立刻超出关联半径。
        """
        t, x, z = 0.0, 0.0, z0
        ego = (speed, yaw) if use_ego else None
        lock.update([{'x': x, 'z': z, 'conf': .9}], t, 1.0, ego=(0.0, 0.0))
        misses = 0
        for _ in range(frames):
            t += frame_dt
            fx, fy = z + origin, -x
            dth = yaw * frame_dt
            dx = (speed / yaw) * math.sin(dth) if abs(yaw) > 1e-9 else speed * frame_dt
            dy = (speed / yaw) * (1.0 - math.cos(dth)) if abs(yaw) > 1e-9 else 0.0
            c, s_ = math.cos(-dth), math.sin(-dth)
            rx, ry = fx - dx, fy - dy
            nfx, nfy = rx * c - ry * s_, rx * s_ + ry * c
            x, z = -nfy, nfx - origin
            if lock.update([{'x': x, 'z': z, 'conf': .9}], t, 1.0, ego=ego) is None:
                misses += 1
        return misses

    def test_turning_at_full_frame_rate_stays_locked(self):
        self.assertEqual(self._turn_sim(self._lock(), 0.1), 0)

    def test_dropped_frame_while_turning_used_to_break_the_lock(self):
        """真正的失效场景:转弯时检测掉一帧。

        车 1.2 rad/s、人在 2m 处,0.2s 的间隔光靠自车旋转就让目标在车体系里
        移动约 0.64m —— 超过 0.55m 的关联半径。补偿前必掉锁,补偿后一帧不丢。
        这就是"转弯比直行更容易跟丢"的机理。
        """
        self.assertGreater(self._turn_sim(self._lock(), 0.2, use_ego=False), 0,
                           "不补偿本来就该掉锁,否则这个测试没有意义")
        self.assertEqual(self._turn_sim(self._lock(), 0.2, use_ego=True), 0)

    def test_driving_and_turning_together_stays_locked(self):
        """边走边转 —— 跟人拐弯时的真实工况。"""
        self.assertEqual(self._turn_sim(self._lock(), 0.15, yaw=0.9, speed=0.45), 0)

    def test_repeated_misses_do_not_double_count_ego_motion(self):
        """连续几帧关联不上时,同一段自车运动不能被反复叠加到锚点上。"""
        lock = self._lock()
        lock.update([{'x': 0.0, 'z': 2.0, 'conf': .9}], 0.0, 1.0, ego=(0.0, 0.0))
        anchor0 = lock.anchor_xz
        for i in range(1, 5):
            lock.update([], i * 0.1, 1.0, ego=(0.5, 0.0))
        drift = abs(lock.anchor_xz[1] - anchor0[1])
        # 0.4s x 0.5m/s = 0.2m,允许目标速度外推带来的少量额外量
        self.assertLess(drift, 0.35, "锚点漂得比自车实际走过的还多,说明重复叠加了")

    def test_target_velocity_is_estimated(self):
        """人以约 1 m/s 远离时,锚点外推应当跟得上,速度估计方向正确。"""
        lock = self._lock()
        t, z = 0.0, 1.5
        lock.update([{'x': 0.0, 'z': z, 'conf': .9}], t, 1.0, ego=(0.0, 0.0))
        for _ in range(5):
            t += 0.1
            z += 0.10
            lock.update([{'x': 0.0, 'z': z, 'conf': .9}], t, 1.0, ego=(0.0, 0.0))
        vf, _vl = lock.velocity_fl
        self.assertGreater(vf, 0.6)
        self.assertLess(vf, 1.5)

    def test_moving_forward_does_not_shift_lateral_anchor(self):
        lock = self._lock()
        lock.update([{'x': 0.3, 'z': 2.0, 'conf': .9}], 0.0, 1.0, ego=(0.0, 0.0))
        lock.update([], 0.1, 1.0, ego=(0.5, 0.0))
        self.assertAlmostEqual(lock.anchor_xz[0], 0.3, places=6)
        self.assertAlmostEqual(lock.anchor_xz[1], 2.0 - 0.05, places=6)


# =============================================================================
# 外观重识别 —— 「解锁后跟了个陌生人走」
# =============================================================================

RED = [0.9, 0.05, 0.05] + [0.0] * 9
BLUE = [0.0] * 8 + [0.9, 0.05, 0.05, 0.0]


class TestAppearanceSimilarity(unittest.TestCase):

    def test_none_when_no_features(self):
        self.assertIsNone(appearance_similarity(None, None, None, None))

    def test_identical_colors_score_one(self):
        self.assertAlmostEqual(appearance_similarity(None, RED, None, RED), 1.0,
                               places=6)

    def test_disjoint_colors_score_zero(self):
        self.assertLess(appearance_similarity(None, RED, None, BLUE), 0.05)

    def test_scale_invariant(self):
        doubled = [v * 2 for v in RED]
        self.assertAlmostEqual(appearance_similarity(None, RED, None, doubled),
                               1.0, places=6)

    def test_height_difference_lowers_score(self):
        same = appearance_similarity(1.70, None, 1.70, None)
        different = appearance_similarity(1.70, None, 1.40, None)
        self.assertAlmostEqual(same, 1.0, places=6)
        self.assertLess(different, 0.2)


class TestReIdentification(unittest.TestCase):

    def _lock(self):
        return TargetLock(confirm_frames=1, lost_timeout_s=0.5,
                          origin_offset_m=0.67)

    def _me(self, x, z):
        return {'x': x, 'z': z, 'conf': .9, 'height_m': 1.70, 'color': RED}

    def _stranger(self, x, z):
        return {'x': x, 'z': z, 'conf': .9, 'height_m': 1.45, 'color': BLUE}

    def test_signature_is_learned_while_locked(self):
        lock = self._lock()
        for i in range(4):
            lock.update([self._me(0.0, 1.2)], i * 0.1, 1.0, ego=(0.0, 0.0))
        self.assertTrue(lock.signature_fresh(0.4))
        self.assertAlmostEqual(lock.sig_height, 1.70, places=2)

    def test_stranger_walking_through_does_not_steal_the_lock(self):
        """经典失效:两个人交错走过。陌生人离锚点更近时也不准被接上。"""
        lock = self._lock()
        for i in range(4):
            lock.update([self._me(0.0, 1.2)], i * 0.1, 1.0, ego=(0.0, 0.0))
        got = lock.update([self._stranger(0.05, 1.2), self._me(0.30, 1.25)],
                          0.5, 1.0, ego=(0.0, 0.0))
        self.assertIsNotNone(got)
        self.assertAlmostEqual(got['x'], 0.30, places=6)

    def test_relock_refuses_a_stranger(self):
        """跟丢之后视野里只剩陌生人 —— 宁可继续搜索也不跟他走。"""
        lock = self._lock()
        for i in range(4):
            lock.update([self._me(0.0, 1.2)], i * 0.1, 1.0, ego=(0.0, 0.0))
        self.assertTrue(lock.locked)
        for _ in range(6):
            self.assertIsNone(lock.update([self._stranger(0.0, 1.1)], 5.0, 1.0,
                                          ego=(0.0, 0.0)))
        self.assertFalse(lock.locked)

    def test_relock_accepts_the_right_person(self):
        lock = self._lock()
        for i in range(4):
            lock.update([self._me(0.0, 1.2)], i * 0.1, 1.0, ego=(0.0, 0.0))
        lock.update([], 5.0, 1.0, ego=(0.0, 0.0))       # 超时解锁
        self.assertFalse(lock.locked)
        got = lock.update([self._stranger(0.0, 1.0), self._me(0.6, 1.4)],
                          5.1, 1.0, ego=(0.0, 0.0))
        self.assertIsNotNone(got, "本人就在视野里,应该认出来")
        self.assertAlmostEqual(got['x'], 0.6, places=6)

    def test_signature_expires_so_the_robot_is_not_stuck_forever(self):
        """签名过期后必须允许重新认人,否则换件外套就永远锁不上了。"""
        lock = TargetLock(confirm_frames=1, lost_timeout_s=0.5,
                          signature_ttl_s=2.0, origin_offset_m=0.67)
        for i in range(4):
            lock.update([self._me(0.0, 1.2)], i * 0.1, 1.0, ego=(0.0, 0.0))
        lock.update([], 5.0, 1.0, ego=(0.0, 0.0))
        self.assertFalse(lock.signature_fresh(5.0))
        got = lock.update([self._stranger(0.0, 1.1)], 5.1, 1.0, ego=(0.0, 0.0))
        self.assertIsNotNone(got)

    def test_featureless_candidates_behave_exactly_as_before(self):
        """老检测器不发 height/color。此时必须退化成纯几何,不能拒绝一切。"""
        lock = self._lock()
        plain = {'x': 0.0, 'z': 1.2, 'conf': .9}
        for i in range(4):
            lock.update([plain], i * 0.1, 1.0, ego=(0.0, 0.0))
        self.assertTrue(lock.locked)
        self.assertFalse(lock.signature_fresh(0.4))
        self.assertIsNotNone(lock.update([{'x': 0.05, 'z': 1.25, 'conf': .9}],
                                         0.5, 1.0, ego=(0.0, 0.0)))

    def test_very_close_detection_is_not_vetoed_by_appearance(self):
        """人低头、转身会让直方图抖。贴在锚点上的检测不准被外观否决。"""
        lock = self._lock()
        for i in range(4):
            lock.update([self._me(0.0, 1.2)], i * 0.1, 1.0, ego=(0.0, 0.0))
        shaky = {'x': 0.02, 'z': 1.21, 'conf': .9, 'height_m': 1.70,
                 'color': BLUE}
        self.assertIsNotNone(lock.update([shaky], 0.5, 1.0, ego=(0.0, 0.0)))


if __name__ == "__main__":
    unittest.main(verbosity=2)
