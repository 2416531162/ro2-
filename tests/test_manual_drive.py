#!/usr/bin/env python3
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "radar_system"))

from manual_drive import ManualDriveLatch  # noqa: E402


class TestManualDriveLatch(unittest.TestCase):
    def test_50hz_stream_stays_active_for_click_pulse(self):
        latch = ManualDriveLatch(timeout_s=0.75)
        latch.set(0.85, 0.0, now=10.0)
        samples = [latch.sample(now=10.0 + i * 0.02) for i in range(31)]
        self.assertEqual(len(samples), 31)
        self.assertTrue(all(active for _, _, active, _ in samples))
        self.assertTrue(all(abs(vx - 0.85) < 1e-9 for vx, _, _, _ in samples))

    def test_explicit_stop_is_immediate_and_published_once(self):
        latch = ManualDriveLatch(timeout_s=0.75)
        latch.set(0.55, 0.2, now=1.0)
        latch.set(0.0, 1.0, now=1.2)  # zero linear speed also clears impossible yaw
        self.assertEqual(latch.sample(now=1.2), (0.0, 0.0, False, True))
        self.assertEqual(latch.sample(now=1.22), (0.0, 0.0, False, False))

    def test_watchdog_expires_to_one_zero_frame(self):
        latch = ManualDriveLatch(timeout_s=0.75)
        latch.set(0.30, 0.0, now=2.0)
        self.assertTrue(latch.sample(now=2.75)[2])
        self.assertEqual(latch.sample(now=2.751), (0.0, 0.0, False, True))
        self.assertEqual(latch.sample(now=2.80), (0.0, 0.0, False, False))

    def test_late_move_after_newer_command_is_dropped(self):
        """网络乱序:seq=5 的前进晚于 seq=6 的刹车到达,不能再把车开起来。"""
        latch = ManualDriveLatch(timeout_s=0.75)
        self.assertTrue(latch.accept("page", 6, 0.0, 0.0))
        self.assertFalse(latch.accept("page", 5, 0.85, 0.0))
        self.assertFalse(latch.accept("page", 6, 0.85, 0.0))
        self.assertTrue(latch.accept("page", 7, 0.85, 0.0))

    def test_late_stop_is_still_applied(self):
        """迟到的刹车宁可多停一下,也不丢弃。"""
        latch = ManualDriveLatch(timeout_s=0.75)
        self.assertTrue(latch.accept("page", 9, 0.85, 0.0))
        self.assertTrue(latch.accept("page", 8, 0.0, 0.0))

    def test_sessions_and_legacy_clients_are_independent(self):
        latch = ManualDriveLatch(timeout_s=0.75)
        self.assertTrue(latch.accept("phone", 100, 0.5, 0.0))
        self.assertTrue(latch.accept("screen", 1, 0.5, 0.0))
        self.assertTrue(latch.accept(None, None, 0.5, 0.0))


if __name__ == "__main__":
    unittest.main()
