#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘驱动内独立防撞层(scan_guard)测试。不需要 ROS、串口。"""

import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "wheeltec_protocol"))

from scan_guard import GuardConfig, ScanGuard  # noqa: E402
from wheeltec_driver import Config, ControlPolicy  # noqa: E402

BINS = 720
INC = 2 * math.pi / BINS
LIDAR_X = 0.53


def scan_with(points):
    """车体系障碍点 -> 720 格扫描(其余方向无回波)。"""
    ranges = [math.inf] * BINS
    for x, y in points:
        dx, dy = x - LIDAR_X, y
        a = math.atan2(dy, dx) % (2 * math.pi)
        i = int(round(a / INC)) % BINS
        ranges[i] = min(ranges[i], math.hypot(dx, dy))
    return ranges


def guard_with(points, now=0.0, **cfg):
    g = ScanGuard(GuardConfig(**cfg))
    g.update_scan(scan_with(points), 0.0, INC, 0.15, 12.0, now)
    return g


class TestScanGuard(unittest.TestCase):

    def test_no_scan_is_passthrough(self):
        g = ScanGuard()
        self.assertEqual(g.limit(0.8, False, 1.0), (0.8, None))
        self.assertEqual(g.last_reason, "no_scan_passthrough")

    def test_clear_path_is_untouched(self):
        g = guard_with([(4.0, 0.0)])
        self.assertEqual(g.limit(0.5, False, 0.1), (0.5, None))

    def test_obstacle_ahead_slows_then_stops(self):
        g = guard_with([(0.67 + 0.40, 0.0)])
        v, why = g.limit(1.2, False, 0.1)
        self.assertEqual(why, "guard_slow")
        self.assertLess(v, 1.2)
        # 刹车距离必须在剩余空间内
        c = g.cfg
        self.assertLessEqual(c.latency_s * v + v * v / (2 * c.decel_m_s2) + c.stop_margin_m, 0.40 + 1e-9)
        g = guard_with([(0.67 + 0.03, 0.0)])
        self.assertEqual(g.limit(0.3, False, 0.1), (0.0, "guard_stop"))

    def test_obstacle_beside_corridor_is_ignored(self):
        g = guard_with([(1.0, 0.60)])
        self.assertEqual(g.limit(0.5, False, 0.1), (0.5, None))

    def test_turning_widens_corridor(self):
        g = guard_with([(0.67 + 0.10, 0.44)])
        self.assertEqual(g.limit(0.5, False, 0.1)[1], None)
        v, why = g.limit(0.5, True, 0.1)
        self.assertEqual(why, "guard_slow")
        self.assertLess(v, 0.2)

    def test_reverse_checks_rear_only(self):
        g = guard_with([(-0.18 - 0.03, 0.0), (0.67 + 3.0, 0.0)])
        self.assertEqual(g.limit(-0.2, False, 0.1), (0.0, "guard_stop"))
        self.assertEqual(g.limit(0.5, False, 0.1), (0.5, None))

    def test_obstacle_behind_does_not_block_forward(self):
        g = guard_with([(-0.25, 0.0)])
        self.assertEqual(g.limit(0.5, False, 0.1), (0.5, None))

    def test_self_hits_are_ignored(self):
        g = guard_with([(0.685, 0.10), (0.30, 0.35)])     # 车头 1.5cm / 车侧 1.5cm 内
        self.assertEqual(g.points, [])

    def test_stale_scan_caps_speed(self):
        g = guard_with([(4.0, 0.0)], now=0.0)
        v, why = g.limit(0.8, False, 1.0)
        self.assertEqual((v, why), (0.15, "guard_scan_stale"))
        self.assertEqual(g.limit(-0.8, False, 1.0)[0], -0.15)

    def test_disabled(self):
        g = guard_with([(0.70, 0.0)], enabled=False)
        self.assertEqual(g.limit(0.5, False, 0.1), (0.5, None))

    def test_invalid_config(self):
        with self.assertRaises(ValueError):
            GuardConfig(decel_m_s2=0.0)


class TestSampledNoEcho(unittest.TestCase):
    def scan(self, *, marker=True, fill=math.inf):
        ranges = [5.0] * BINS
        sampled = [True] * BINS
        # One unsampled rear slot advertises the publisher's sampling mask.
        sampled[BINS // 2] = False
        ranges[BINS // 2] = math.inf
        for i in list(range(100)) + list(range(BINS - 99, BINS)):
            ranges[i] = fill
            sampled[i] = True
        g = ScanGuard()
        g.update_scan(ranges, 0., INC, .15, 12., 0., sampled=sampled if marker else None)
        return g, ranges, sampled

    def test_sampled_no_echo_is_far_but_creep_only(self):
        g, _, _ = self.scan()
        self.assertTrue(g._observed_free(1.2, 0.)[0])
        self.assertTrue(g._observed_free(1.2, 0.)[1])
        self.assertEqual(g.limit(.4, 0., .1), (.12, 'guard_no_echo'))
        self.assertTrue(g.last_no_echo_used)

    def test_old_publisher_and_unsampled_sector_are_not_far(self):
        g, ranges, sampled = self.scan(marker=False)
        self.assertFalse(g._observed_free(1.2, 0.)[0])
        self.assertEqual(g.limit(.4, 0., .1)[0], 0.)
        for i in list(range(100)) + list(range(BINS - 99, BINS)):
            sampled[i] = False
        g.update_scan(ranges, 0., INC, .15, 12., .2, sampled=sampled)
        self.assertFalse(g._observed_free(1.2, 0.)[0])
        self.assertEqual(g.limit(.4, 0., .3)[0], 0.)

    def test_invalid_echoes_and_self_hits_do_not_clear_path(self):
        for fill in (-math.inf, math.nan, .17):
            with self.subTest(fill=fill):
                g, _, _ = self.scan(fill=fill)
                self.assertFalse(g._observed_free(1.2, 0.)[0])
                self.assertEqual(g.limit(.4, 0., .1)[0], 0.)
        g, ranges, sampled = self.scan()
        ranges[BINS // 2] = .2  # rear ray hits the chassis itself
        sampled[BINS // 2] = True
        g.update_scan(ranges, 0., INC, .15, 12., .1, sampled=sampled)
        self.assertFalse(g._observed_free(-.4, 0.)[0])

    def test_real_near_obstacle_wins_over_no_echo(self):
        g, ranges, sampled = self.scan()
        ranges[0] = .16  # x=.69m, just beyond the front bumper
        g.update_scan(ranges, 0., INC, .15, 12., .1, sampled=sampled)
        v, why = g.limit(.4, 0., .2)
        self.assertEqual(v, 0.)
        self.assertEqual(why, 'guard_stop')

    def test_rear_sampled_no_echo_does_not_justify_reverse(self):
        g, ranges, sampled = self.scan()
        for i in range(200, 521):
            ranges[i] = math.inf
            sampled[i] = True
        g.update_scan(ranges, 0., INC, .15, 12., .1, sampled=sampled)
        self.assertFalse(g._observed_free(-.4, 0.)[0])
        self.assertEqual(g.limit(-.3, 0., .2)[0], 0.)

    def test_no_finite_echo_or_stale_scan_stops(self):
        g, ranges, sampled = self.scan()
        self.assertEqual(g.limit(.4, 0., .6), (0., 'guard_scan_stale'))
        self.assertFalse(g.last_no_echo_used)
        g.update_scan([math.inf] * BINS, 0., INC, .15, 12., .7,
                      sampled=[True] * BINS)
        self.assertIsNone(g.scan_time)
        self.assertEqual(g.limit(.4, 0., .8), (0., 'guard_scan_unavailable'))


class TestPolicyWithGuard(unittest.TestCase):

    def policy(self, points):
        cfg = Config(protocol="twist", protocol_confirmed=True, receive_only=False,
                     max_speed_m_s=1.2, acceleration_m_s2=2.0)
        p = ControlPolicy(cfg, 0.0)
        p.link(True, 0.0)
        for k in range(6):
            p.feedback({"velocity": (0.0, 0.0, 0.0)}, 3.0 + k * 0.01)
        ok, _ = p.arm(3.1)
        self.assertTrue(ok)
        g = guard_with(points, now=3.1)
        p.speed_filter = lambda speed, turn, now: g.limit(speed, abs(turn) > 0.05, now)
        return p, g

    def test_guard_stops_output_immediately(self):
        p, g = self.policy([(0.67 + 0.03, 0.0)])
        self.assertTrue(g.points, "车头 3cm 的障碍物必须可见")
        p.command("twist", 0.8, 0.0, 3.12)
        p.feedback({"velocity": (0.0, 0.0, 0.0)}, 3.12)
        p.tick(3.14)
        self.assertEqual(p.output[0], 0.0)
        self.assertEqual(p.guard_reason, "guard_stop")
        self.assertTrue(p.armed, "防撞只清零输出,不解除使能")

    def test_guard_slowdown_applies_without_ramp_delay(self):
        p, g = self.policy([(4.0, 0.0)])
        p.command("twist", 1.0, 0.0, 3.12)
        now = 3.12
        for _ in range(60):
            now += 0.02
            g.update_scan(scan_with([(4.0, 0.0)]), 0.0, INC, 0.15, 12.0, now)
            p.feedback({"velocity": (0.9, 0.0, 0.0)}, now)
            p.command("twist", 1.0, 0.0, now)
            p.tick(now)
        self.assertGreater(p.output[0], 0.9)
        # 突然出现障碍:下一帧输出就降到允许速度,不等加减速斜坡
        g.update_scan(scan_with([(0.67 + 0.30, 0.0)]), 0.0, INC, 0.15, 12.0, now)
        now += 0.02
        p.feedback({"velocity": (0.9, 0.0, 0.0)}, now)
        p.command("twist", 1.0, 0.0, now)
        p.tick(now)
        self.assertLessEqual(p.output[0], g.allowed_speed(0.30) + 1e-9)
        self.assertEqual(p.guard_reason, "guard_slow")

    def test_no_filter_keeps_old_behaviour(self):
        cfg = Config(protocol="twist", protocol_confirmed=True, receive_only=False,
                     max_speed_m_s=1.2, acceleration_m_s2=2.0)
        p = ControlPolicy(cfg, 0.0)
        self.assertIsNone(p.speed_filter)


if __name__ == "__main__":
    unittest.main()
