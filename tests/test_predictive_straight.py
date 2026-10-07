"""可信 RTK 及时、平顺纠偏；质量中断不能积累纠偏或跨线标定证据。"""
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT)]
from line_cascade import ImuAlignment, ImuFirstConfig, ImuFirstGuide, imu_first_target, wrap
from test_imu_first_straight import closed_loop
from test_late_correction import run

DT = .02
DEG = math.pi / 180.


def intermittent_offset(interrupted):
    g = ImuFirstGuide(ImuFirstConfig(rtk_fit=True, rtk_trim=False))
    g.offset, g.offset_n, g.locked, g.source = 0., 50, True, 'learned'
    g.update(0., 0., 0., DT, (.24, -.4575), True)
    s = 0.
    while s < 16.:
        s += .016
        measured = .02 if 12. <= s < 12.32 or 14. <= s < 14.32 else 0.
        trusted = not (12.32 <= s < 14.) if interrupted else True
        g.update(measured, 0., .016, DT, (.24, -.4575), trusted)
    return g


class ObservationContinuity(unittest.TestCase):
    def test_position_step_does_not_create_a_large_heading_calibration(self):
        # 车没有转向，仅卫星位置出现 1.5 cm 的台阶；应及时跟进位置，不把它全解释为角度。
        g, rows = run(lambda s: 0. if s < 12. else .015*min(1., (s-12.)/.5), 20.)
        self.assertGreater(g.persists, 0)
        self.assertLess(max(abs(beta) for _, _, _, beta in rows), .15)
        self.assertLess(max(abs(sat-est) for s, sat, est, _ in rows if 14.5 < s < 16.), .005)

    def test_two_short_excursions_do_not_become_persistent_across_quality_loss(self):
        for interrupted in (False, True):
            with self.subTest(interrupted=interrupted):
                self.assertEqual(intermittent_offset(interrupted).persists, 0)

    def test_rejected_jump_breaks_persistent_offset_evidence(self):
        g = intermittent_offset(False)
        # 一个有效残差点不能跨过跳点拒绝区累计到下一点。
        g._persist_check(g.travel, g._fit_now(g.travel)[0] + .02)
        self.assertIsNotNone(g._persist)
        g.update(.25, 0., .016, DT, (.24, -.4575), True)
        self.assertFalse(g.rtk_accepted)
        self.assertIsNone(g._persist)

    def test_quality_loss_restarts_jump_recovery_timing(self):
        g = intermittent_offset(False)
        cfg = g.cfg
        # 短暂稳定后质量中断，不能把中断两边的计时拼成 jump_accept_s。
        g.update(.05, 0., .016, DT, (.24, -.4575), True)
        self.assertEqual(g.mode, 'imu')
        for _ in range(int(cfg.jump_accept_s/DT)-5):
            g.update(.05, 0., .016, DT, (.24, -.4575), True)
        self.assertEqual(g.jumps_accepted, 0)
        g.update(.05, 0., .016, DT, (.24, -.4575), False)
        for _ in range(10):
            g.update(.05, 0., .016, DT, (.24, -.4575), True)
        self.assertEqual(g.jumps_accepted, 0)
        self.assertEqual(g.mode, 'imu')

    def test_bad_quality_does_not_calibrate_the_next_line(self):
        alignment = ImuAlignment()
        updates = []
        original = alignment.update_line
        def record(token, now, *args):
            updates.append(now)
            return original(token, now, *args)
        with patch.object(alignment, 'update_line', side_effect=record):
            tr, _, _ = closed_loop('imu_fit', length=40., seconds=70., alignment=alignment,
                quality=lambda t: (.008, 30, .005) if t < 20. else (.04, 15, .08))
        self.assertEqual(tr.st.state, 'DONE')
        self.assertTrue(updates)
        self.assertLess(max(updates), 20.1)


class SmoothReference(unittest.TestCase):
    def test_target_rate_and_acceleration_remain_bounded_at_reversals_and_wrap(self):
        cfg = ImuFirstConfig(rtk_trim=False, rtk_fit=True)
        for origin in (0., math.pi - .001):
            g = ImuFirstGuide(cfg)
            g.offset, g.locked, g.fast = 0., True, False
            target, prev_rate = origin, 0.
            for i in range(700):
                g.e = .02 if i < 100 else -.02 if i < 200 else 0.
                previous = target
                target, _, _, _ = imu_first_target(cfg, g, target, origin, DT, 3., 1., True)
                rate = wrap(target-previous)/DT
                self.assertLessEqual(abs(rate), cfg.rate_deg_s*DEG + 1e-10)
                self.assertLessEqual(abs(rate-prev_rate), cfg.target_accel_deg_s2*DEG*DT + 1e-10)
                prev_rate = rate
            self.assertLess(abs(wrap(target-origin)), .001*DEG)

    def test_heading_error_is_corrected_without_waiting_for_large_cross_track_error(self):
        # 与原简化底盘模型成对复现：0.9° 初始换算误差，旧外环/固定目标内环 3 m 后最大约 1.78 cm。
        tr, rows, _ = closed_loop('imu_fit', length=25., seconds=70., dual_bias_deg=.9)
        self.assertEqual(tr.st.state, 'DONE')
        self.assertLess(max(abs(e) for _, s, e in rows if 3. <= s <= 24.), .015)


if __name__ == '__main__':
    unittest.main()
