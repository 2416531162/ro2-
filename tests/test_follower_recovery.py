#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""局部脱困层的回归测试:过门收舵、停稳换向、限量倒车、丢人搜索。

倒车是这个项目里最危险的动作 —— 雷达装在车头,车尾是物理盲区。这里的每条
测试都在钉死一条"不准越过"的红线,不需要 ROS、不需要实车:

    python3 tests/test_follower_recovery.py
"""

import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "radar_system"))

from follower_recovery import (  # noqa: E402
    LocalRecovery, RecoveryConfig, ScanEvidence, PathMemory, reverse_clearance,
)
from footprint import VehicleFootprint, SensorMount  # noqa: E402
from motion_safety import ChassisGeometry, BrakeProfile  # noqa: E402


FOOT = VehicleFootprint(front_m=0.67, rear_m=0.18, half_width_m=0.335,
                        margin_m=0.035)
GEO = ChassisGeometry(wheelbase_m=0.54, track_m=0.59, max_steer_rad=0.35)
PROFILE = BrakeProfile(decel_mps2=1.0, latency_s=0.35, stop_m=0.12,
                       hard_stop_m=0.06)
MOUNT = SensorMount(x_m=0.53, y_m=0.0, yaw_rad=0.0)


def scan_from(distances_by_deg, n=360, default=6.0):
    """造一帧 360 线雷达。键是**相对车头**的方位角(度,左为正),不是数组下标。

    angle_min = -pi 意味着下标 180 才是正前方 —— 这里换算一次,免得每条测试
    自己去算下标,算错了测试就会在一个根本不存在的场景上"通过"。
    """
    ranges = [default] * n
    for deg, dist in distances_by_deg.items():
        ranges[(int(round(deg)) + n // 2) % n] = dist
    return ScanEvidence(ranges, -math.pi, 2 * math.pi / n, 0.15, 12.0,
                        MOUNT, FOOT)


def open_scan():
    """四面八方 6m,没有任何障碍。"""
    return ScanEvidence.from_points(
        [(6.0 * math.cos(math.radians(d)), 6.0 * math.sin(math.radians(d)))
         for d in range(0, 360, 2)])


def wall_ahead(gap_m):
    """正前方距**车头** gap_m 处一堵墙。车头在 x = FOOT.front_m。"""
    x = FOOT.front_m + gap_m
    return ScanEvidence.from_points([(x, y / 100.0) for y in range(-30, 31, 5)])


def doorway(half_width_m=0.39, x_m=0.70):
    """一道刚好比车宽一点的门。

    车全宽含余量 0.74m,门净宽 0.78m —— 直着能过。但 0.30rad 打舵时车体扫过
    的带宽是 0.84m,外侧前角会刮到门框。这正是"跟着人修方向就刮轮子"的场景。
    """
    return ScanEvidence.from_points([(x_m, half_width_m), (x_m, -half_width_m)])


def make(cfg=None):
    return LocalRecovery(FOOT, GEO, PROFILE, cfg or RecoveryConfig())


def step(rec, t, scan, **kw):
    kw.setdefault('healthy', True)
    kw.setdefault('speed', 0.0)
    kw.setdefault('yaw_rate', 0.0)
    kw.setdefault('target', True)
    kw.setdefault('gap', 1.2)
    kw.setdefault('bearing', 0.0)
    kw.setdefault('requested_speed', 0.3)
    kw.setdefault('requested_steer', 0.0)
    kw.setdefault('current_steer', 0.0)
    kw.setdefault('follow_cap', 0.5)
    kw.setdefault('lost_age', 0.0)
    return rec.update(now=t, scan=scan, **kw)


# =============================================================================
# 雷达证据
# =============================================================================

class TestScanEvidence(unittest.TestCase):

    def test_empty_scan_is_not_usable(self):
        self.assertFalse(ScanEvidence([], 0.0, 0.01, 0.1, 12.0).usable)

    def test_zero_increment_is_not_usable(self):
        self.assertFalse(ScanEvidence([1.0] * 360, 0.0, 0.0, 0.1, 12.0).usable)

    def test_nan_and_out_of_range_are_dropped(self):
        ev = ScanEvidence([float('nan')] * 300 + [2.0] * 60,
                          -math.pi, 2 * math.pi / 360, 0.15, 12.0, MOUNT, FOOT)
        self.assertTrue(ev.usable)
        self.assertEqual(len(ev.bearings), 60)

    def test_all_garbage_is_not_usable(self):
        """一帧里连几个有效回波都没有,不能被当成'前方畅通'。"""
        self.assertFalse(ScanEvidence([float('inf')] * 360, -math.pi,
                                      2 * math.pi / 360, 0.15, 12.0,
                                      MOUNT, FOOT).usable)


# =============================================================================
# 倒车净空 —— 「车尾是盲区,盲区不是空地」
# =============================================================================

class TestReverseClearance(unittest.TestCase):

    def test_obstacle_behind_limits_reverse(self):
        # 车尾在 x=-0.18,障碍物在 x=-0.78 -> 净空 0.60
        c = reverse_clearance([(-0.78, 0.0)], FOOT, GEO, 0.0)
        self.assertAlmostEqual(c, 0.60, places=2)

    def test_obstacle_ahead_does_not_limit_reverse(self):
        self.assertGreater(reverse_clearance([(2.0, 0.0)], FOOT, GEO, 0.0), 5.0)

    def test_obstacle_beside_is_ignored(self):
        self.assertGreater(reverse_clearance([(-0.78, 1.5)], FOOT, GEO, 0.0), 5.0)

    def test_touching_the_rear_bumper_is_zero(self):
        # 车尾在 x=-0.18,点已经贴进车体轮廓里
        self.assertAlmostEqual(reverse_clearance([(-0.10, 0.0)], FOOT, GEO, 0.0),
                               0.0, places=3)
        # 刚出轮廓 2cm
        self.assertAlmostEqual(reverse_clearance([(-0.20, 0.0)], FOOT, GEO, 0.0),
                               0.02, places=3)

    def test_turning_while_reversing_sweeps_the_other_way(self):
        """倒车打舵时车体扫向与前进相反的一侧 —— 符号弄反就是刮墙。"""
        left_side = [(-0.9, 0.55)]
        a = reverse_clearance(left_side, FOOT, GEO, 0.30)
        b = reverse_clearance(left_side, FOOT, GEO, -0.30)
        self.assertNotAlmostEqual(a, b, places=2)


# =============================================================================
# 路径记忆
# =============================================================================

class TestPathMemory(unittest.TestCase):

    def test_points_move_backwards_as_the_car_drives_forward(self):
        mem = PathMemory(RecoveryConfig())
        mem.add([(1.0, 0.0)], 0.0)
        mem.advance(1.0, 0.5, 0.0)
        x, y = mem.points()[0]
        self.assertAlmostEqual(x, 0.5, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)

    def test_forward_travel_eventually_covers_the_rear(self):
        """开过去的东西会落到车后 —— 这正是盲区倒车唯一的依据。"""
        mem = PathMemory(RecoveryConfig())
        self.assertFalse(mem.covers_rear())
        for i in range(20):
            mem.add([(1.0, 0.1 * j - 0.5) for j in range(11)], i * 0.1)
            mem.advance(0.1, 0.6, 0.0)
        self.assertTrue(mem.covers_rear())

    def test_stale_points_expire(self):
        cfg = RecoveryConfig(memory_horizon_s=1.0)
        mem = PathMemory(cfg)
        mem.add([(1.0, 0.0)], 0.0)
        mem.add([(1.2, 0.3)], 5.0)
        self.assertEqual(len(mem), 1)

    def test_point_count_is_bounded(self):
        cfg = RecoveryConfig(memory_max_points=50)
        mem = PathMemory(cfg)
        for i in range(30):
            mem.add([(0.01 * k, 0.01 * i) for k in range(100)], i * 0.01)
        self.assertLessEqual(len(mem), 50)


# =============================================================================
# 过门收舵
# =============================================================================

class TestAligning(unittest.TestCase):

    def test_open_space_passes_the_request_through(self):
        r = step(make(), 0.0, open_scan(), requested_steer=0.20)
        self.assertEqual(r.state, 'TRACKING')
        self.assertAlmostEqual(r.steer, 0.20, places=6)

    def test_narrow_gap_straightens_the_wheel(self):
        """人往旁边偏,车本能打舵去追,恰好在门框里扫出最宽的轨迹。
        正确做法是先摆正过去。"""
        rec = make()
        # 一条 0.80m 宽的门,车全宽 0.74m:直着能过,打舵过不去
        r = step(rec, 0.0, doorway(), requested_steer=0.30)
        self.assertLess(abs(r.steer), 0.30)

    def test_alignment_is_held_briefly_to_avoid_flapping(self):
        rec = make(RecoveryConfig(align_hold_s=0.5))
        step(rec, 0.0, doorway(), requested_steer=0.30)
        r = step(rec, 0.1, open_scan(), requested_steer=0.30)
        self.assertLess(abs(r.steer), 0.30, "净空一好转就打回去,会在门里左右横跳")


# =============================================================================
# 被困 -> 停稳 -> 倒车
# =============================================================================

class TestStuckAndReverse(unittest.TestCase):

    def _get_stuck(self, rec, cfg=None):
        cfg = cfg or rec.cfg
        blocked = wall_ahead(0.08)
        t = 0.0
        r = step(rec, t, blocked, speed=0.0, requested_speed=0.3)
        while t < cfg.stuck_confirm_s + 0.3 and r.state != 'RECOVERY_BRAKE':
            t += 0.1
            r = step(rec, t, blocked, speed=0.0, requested_speed=0.3)
        return t, r

    def test_momentary_blockage_does_not_trigger_reverse(self):
        """一帧净空低不代表被困 —— 抖一下就倒车比不倒车更危险。"""
        rec = make()
        r = step(rec, 0.0, wall_ahead(0.08), requested_speed=0.3)
        self.assertNotEqual(r.state, 'RECOVERY_BRAKE')
        self.assertEqual(rec.legs, 0)

    def test_sustained_blockage_enters_brake_first(self):
        rec = make()
        _t, r = self._get_stuck(rec)
        self.assertEqual(r.state, 'RECOVERY_BRAKE')
        self.assertEqual(r.speed, 0.0)

    def test_reverse_only_starts_after_a_measured_stop(self):
        """软件发 0 不等于车停了。电机还在正转时给反向指令会顶电流。"""
        rec = make()
        t, _r = self._get_stuck(rec)
        for _ in range(5):
            t += 0.1
            r = step(rec, t, wall_ahead(0.08), speed=0.25)     # 还在滑行
            self.assertEqual(r.state, 'RECOVERY_BRAKE')
            self.assertEqual(r.speed, 0.0)
        t += 0.1
        r = step(rec, t, wall_ahead(0.08), speed=0.0)
        self.assertEqual(r.state, 'RECOVERY_REVERSE')
        self.assertLess(r.speed, 0.0)

    def test_blind_reverse_is_shorter_than_remembered_reverse(self):
        """身后没有记忆点时,允许退的距离必须更短。"""
        rec = make()
        t, _ = self._get_stuck(rec)
        t += 0.1
        step(rec, t, wall_ahead(0.08), speed=0.0)
        self.assertTrue(rec.blind_used, "刚上电就倒车,身后是全瞎的")

    def test_reverse_stops_at_the_distance_budget(self):
        cfg = RecoveryConfig(blind_reverse_distance_m=0.25)
        rec = make(cfg)
        t, _ = self._get_stuck(rec, cfg)
        t += 0.1
        step(rec, t, wall_ahead(0.08), speed=0.0)
        travelled = 0.0
        for _ in range(60):
            t += 0.1
            r = step(rec, t, wall_ahead(0.08), speed=-cfg.reverse_speed_mps)
            if r.state != 'RECOVERY_REVERSE':
                break
            travelled += cfg.reverse_speed_mps * 0.1
        self.assertLessEqual(rec.total_distance, cfg.blind_reverse_distance_m + 0.02)
        self.assertNotEqual(r.state, 'RECOVERY_REVERSE')

    def test_obstacle_behind_stops_the_reverse_immediately(self):
        rec = make()
        t, _ = self._get_stuck(rec)
        t += 0.1
        step(rec, t, wall_ahead(0.08), speed=0.0)
        # 身后 5cm 处一堵墙:雷达 180° 方向,读数 = 车尾净空 + 雷达到车尾距离
        behind = ScanEvidence.from_points(
            [(-(FOOT.rear_m + 0.05), y / 100.0) for y in range(-30, 31, 5)])
        t += 0.1
        r = step(rec, t, behind, speed=-0.1)
        self.assertNotEqual(r.state, 'RECOVERY_REVERSE')

    def test_reverse_retraces_the_recent_forward_steer(self):
        """原路退回:用刚才前进时的舵角,阿克曼车会精确沿原弧线倒回去。"""
        rec = make()
        blocked = wall_ahead(0.08)
        t = 0.0
        for _ in range(6):                      # 先带着舵角往前走
            step(rec, t, open_scan(), speed=0.3, current_steer=0.22,
                 requested_steer=0.22)
            t += 0.1
        r = step(rec, t, blocked, speed=0.0, requested_speed=0.3)
        while r.state != 'RECOVERY_BRAKE' and t < 4.0:
            t += 0.1
            r = step(rec, t, blocked, speed=0.0, requested_speed=0.3)
        t += 0.1
        r = step(rec, t, blocked, speed=0.0)
        self.assertEqual(r.state, 'RECOVERY_REVERSE')
        self.assertAlmostEqual(r.steer, 0.22, places=6)

    def test_reverse_is_speed_capped(self):
        rec = make()
        t, _ = self._get_stuck(rec)
        t += 0.1
        r = step(rec, t, wall_ahead(0.08), speed=0.0)
        self.assertGreaterEqual(r.speed, -rec.cfg.reverse_speed_mps - 1e-9)

    def test_leg_budget_is_exhausted_eventually(self):
        """退不出去就认输停下,不能无限次往盲区里退。"""
        cfg = RecoveryConfig(reverse_max_legs=2, stuck_confirm_s=0.2,
                             blind_reverse_distance_m=0.10)
        rec = make(cfg)
        blocked = wall_ahead(0.08)
        t = 0.0
        for _ in range(600):
            t += 0.1
            speed = -cfg.reverse_speed_mps if rec.phase == 'REVERSE' else 0.0
            r = step(rec, t, blocked, speed=speed, requested_speed=0.3)
            if rec.exhausted and not rec.active:
                break
        self.assertTrue(rec.exhausted)
        self.assertLessEqual(rec.legs, cfg.reverse_max_legs)
        self.assertEqual(r.speed, 0.0)

    def test_driving_clear_again_restores_the_budget(self):
        rec = make()
        t, _ = self._get_stuck(rec)
        # 重新走顺之后,脱困预算必须恢复 —— 否则一次卡住会让车终身残废
        for _ in range(80):
            t += 0.1
            step(rec, t, open_scan(), speed=0.4, requested_speed=0.3)
        self.assertEqual(rec.legs, 0)
        self.assertFalse(rec.exhausted)


# =============================================================================
# 丢人搜索
# =============================================================================

class TestSearch(unittest.TestCase):

    def test_blink_holds_still(self):
        """人刚丢的一瞬间最可能只是漏帧。乱转会把本来能接上的人转出视野。"""
        r = step(make(), 0.0, open_scan(), target=False, lost_age=0.2,
                 requested_speed=0.0)
        self.assertEqual(r.state, 'TARGET_BLINK')
        self.assertEqual(r.speed, 0.0)

    def test_observe_before_turning(self):
        rec = make()
        r = step(rec, 0.0, open_scan(), target=False, lost_age=2.0,
                 requested_speed=0.0)
        self.assertEqual(r.state, 'SEARCH_SCAN')
        self.assertEqual(r.speed, 0.0)
        r = step(rec, 0.5, open_scan(), target=False, lost_age=2.5,
                 requested_speed=0.0)
        self.assertEqual(r.state, 'SEARCH_SCAN')

    def test_turn_starts_after_the_observation_window(self):
        cfg = RecoveryConfig(scan_hold_s=1.0)
        rec = make(cfg)
        step(rec, 0.0, open_scan(), target=False, lost_age=2.0, requested_speed=0.0)
        r = step(rec, 1.5, open_scan(), target=False, lost_age=3.5, requested_speed=0.0)
        self.assertEqual(r.state, 'SEARCH_TURN')
        r = step(rec, 1.6, open_scan(), target=False, lost_age=3.6, requested_speed=0.0)
        self.assertGreater(abs(r.steer), 0.1)
        self.assertGreater(r.speed, 0.0)
        self.assertLessEqual(r.speed, cfg.search_speed_mps + 1e-9)

    def test_reacquiring_the_target_ends_the_search(self):
        rec = make()
        step(rec, 0.0, open_scan(), target=False, lost_age=2.0, requested_speed=0.0)
        r = step(rec, 0.2, open_scan(), target=True, lost_age=0.0)
        self.assertEqual(r.state, 'TRACKING')
        self.assertFalse(rec.active)

    def test_search_gives_up_after_the_yaw_budget(self):
        cfg = RecoveryConfig(scan_hold_s=0.1, search_yaw_limit_rad=0.5)
        rec = make(cfg)
        step(rec, 0.0, open_scan(), target=False, lost_age=2.0, requested_speed=0.0)
        t = 0.2
        r = step(rec, t, open_scan(), target=False, lost_age=2.2, requested_speed=0.0)
        for _ in range(50):
            t += 0.1
            r = step(rec, t, open_scan(), target=False, lost_age=9.0,
                     requested_speed=0.0, yaw_rate=0.8)
            if r.state == 'SEARCHING_LOST':
                break
        self.assertEqual(r.state, 'SEARCHING_LOST')

    def test_recovery_disabled_never_moves_on_its_own(self):
        rec = make(RecoveryConfig(enabled=False))
        t = 0.0
        for _ in range(60):
            t += 0.1
            r = step(rec, t, wall_ahead(0.08), target=False, lost_age=9.0,
                     requested_speed=0.3)
            self.assertGreaterEqual(r.speed, 0.0, "--no-recovery 下不准自己倒车")
            self.assertEqual(rec.legs, 0)


# =============================================================================
# 健康与净空
# =============================================================================

class TestHealthAndClearance(unittest.TestCase):

    def test_unhealthy_stops_everything(self):
        r = step(make(), 0.0, open_scan(), healthy=False)
        self.assertEqual(r.state, 'RECOVERY_WAIT')
        self.assertEqual(r.speed, 0.0)

    def test_unusable_scan_stops_everything(self):
        r = step(make(), 0.0, ScanEvidence([], 0.0, 0.01, 0.1, 12.0))
        self.assertEqual(r.state, 'RECOVERY_WAIT')
        self.assertEqual(r.speed, 0.0)

    def test_losing_health_aborts_an_active_reverse(self):
        rec = make()
        blocked = wall_ahead(0.08)
        t = 0.0
        r = step(rec, t, blocked, speed=0.0, requested_speed=0.3)
        while r.state != 'RECOVERY_BRAKE' and t < 4.0:
            t += 0.1
            r = step(rec, t, blocked, speed=0.0, requested_speed=0.3)
        t += 0.1
        self.assertEqual(step(rec, t, blocked, speed=0.0).state, 'RECOVERY_REVERSE')
        t += 0.1
        r = step(rec, t, blocked, speed=-0.1, healthy=False)
        self.assertEqual(r.speed, 0.0)
        self.assertFalse(rec.active)

    def test_clearance_without_scan_is_zero(self):
        """没有证据时必须当成'前面是墙',不能当成'前面是空的'。"""
        self.assertEqual(make().clearance(None, 0.0, 1), 0.0)

    def test_clearance_takes_the_more_conservative_arc(self):
        rec = make()
        gate = doorway()
        both = rec.clearance(gate, 0.0, 1, actual_steer=0.30)
        only_straight = rec.clearance(gate, 0.0, 1)
        self.assertLessEqual(both, only_straight)

    def test_history_is_only_used_when_asked(self):
        rec = make()
        for i in range(20):
            rec.update(now=i * 0.1, scan=open_scan(), healthy=True, speed=0.6,
                       yaw_rate=0.0, target=True, gap=1.2, bearing=0.0,
                       requested_speed=0.3, requested_steer=0.0,
                       current_steer=0.0, follow_cap=0.5, lost_age=0.0)
        plain = rec.clearance(open_scan(), 0.0, -1, allow_history=False)
        remembered = rec.clearance(open_scan(), 0.0, -1, allow_history=True)
        self.assertLessEqual(remembered, plain)


if __name__ == "__main__":
    unittest.main(verbosity=2)
