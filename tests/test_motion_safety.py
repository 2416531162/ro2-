#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""无硬件回归测试:刹车包络、阿克曼换算、驱动状态机。

不需要 ROS、不需要串口、不需要实车。跑法:

    python3 tests/test_motion_safety.py
    # 或
    python3 -m unittest discover tests -v
"""

import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "radar_system"))
sys.path.insert(0, os.path.join(ROOT, "wheeltec_protocol"))

from motion_safety import (  # noqa: E402
    ChassisGeometry, BrakeProfile, brake_envelope, stopping_distance,
    yaw_from_steer, steer_from_yaw, max_yaw_at_speed,
    AlphaBetaTracker, SlewLimiter, BreakawayKick,
)
from wheeltec_driver import Config, ControlPolicy, build_frame, STOP_FRAME  # noqa: E402


# =============================================================================
# 刹车包络 —— 直接对应「没提前减速就撞上」这个 bug
# =============================================================================

class TestBrakeEnvelope(unittest.TestCase):

    def setUp(self):
        self.p = BrakeProfile(decel_mps2=1.0, latency_s=0.35,
                              stop_m=0.70, hard_stop_m=0.40)

    def test_zero_at_and_inside_stop_distance(self):
        self.assertEqual(brake_envelope(0.70, self.p), 0.0)
        self.assertEqual(brake_envelope(0.50, self.p), 0.0)
        self.assertEqual(brake_envelope(0.0, self.p), 0.0)

    def test_monotonic_increasing_with_distance(self):
        """距离越远允许越快,而且必须单调 —— 不允许出现旧版那种断崖。"""
        distances = [0.7 + 0.05 * i for i in range(40)]
        speeds = [brake_envelope(d, self.p) for d in distances]
        for a, b in zip(speeds, speeds[1:]):
            self.assertLessEqual(a, b)

    def test_no_speed_cliff(self):
        """相邻 1cm 之间的速度跳变必须很小。

        旧版在 0.80m 处从 0.32 m/s 直接跳到 0,这个 bug 会被这条测试抓住。
        """
        for i in range(200):
            d = 0.70 + 0.01 * i
            jump = abs(brake_envelope(d + 0.01, self.p) - brake_envelope(d, self.p))
            self.assertLess(jump, 0.05, f"{d:.2f}m 处速度跳变 {jump:.3f} 过大")

    def test_envelope_speed_actually_stops_in_time(self):
        """核心安全性质:以包络允许的速度行驶,刹停距离不得超过可用余量。"""
        for i in range(1, 60):
            d = 0.70 + 0.05 * i
            v = brake_envelope(d, self.p)
            need = stopping_distance(v, self.p)
            gap = d - self.p.stop_m
            self.assertLessEqual(need, gap + 1e-6,
                                 f"在 {d:.2f}m 以 {v:.3f}m/s 行驶需要 {need:.3f}m,只有 {gap:.3f}m")

    def test_old_behaviour_would_have_crashed(self):
        """复现旧版参数,证明它会撞进硬急停区。

        旧速度律: vx = clamp(0.25 + 0.45*(z-0.65), 0.25, 0.65),z<=0.80 时直接归零。
        关键在于 0.80m 这一刻车速可以高达 MAX_SPEED_MPS —— 人在 1.54m 外时车就
        已经跑满 0.65 m/s,人一停步,车带着满速冲进死区边界才开始想刹车。
        """
        OLD_MAX_SPEED = 0.65
        OLD_DEADBAND_MAX = 0.80
        OLD_AEB = 0.40

        stop_at = OLD_DEADBAND_MAX - stopping_distance(OLD_MAX_SPEED, self.p)
        self.assertLess(stop_at, OLD_AEB,
                        f"旧版应当越过 {OLD_AEB}m 硬急停线,实际停在 {stop_at:.3f}m")

        # 即便只是死区边界上的最低速度,余量也只剩十几厘米,经不起标定误差
        edge_speed = max(0.25, 0.25 + 0.45 * (OLD_DEADBAND_MAX - 0.65))
        edge_stop = OLD_DEADBAND_MAX - stopping_distance(edge_speed, self.p)
        self.assertLess(edge_stop - OLD_AEB, 0.25,
                        "死区边界速度的安全余量也过小")

    def test_new_envelope_never_reaches_hard_stop(self):
        """新版:从任意距离出发,按包络行驶都不会碰到硬急停线。"""
        for i in range(1, 80):
            d = 0.70 + 0.05 * i
            v = min(0.55, brake_envelope(d, self.p))    # 0.55 = 新的 max_speed_mps
            stop_at = d - stopping_distance(v, self.p)
            self.assertGreater(stop_at, self.p.hard_stop_m,
                               f"从 {d:.2f}m 以 {v:.3f}m/s 出发会停在 {stop_at:.3f}m")

    def test_new_behaviour_stops_clear(self):
        """新版:在同一个 0.80m 位置,包络允许的速度能安全停在 stop_m 上。"""
        v = brake_envelope(0.80, self.p)
        self.assertLessEqual(0.80 - stopping_distance(v, self.p),
                             self.p.stop_m + 1e-6)
        self.assertGreater(0.80 - stopping_distance(v, self.p),
                           self.p.hard_stop_m)

    def test_rejects_inconsistent_profile(self):
        with self.assertRaises(ValueError):
            BrakeProfile(stop_m=0.30, hard_stop_m=0.60)
        with self.assertRaises(ValueError):
            BrakeProfile(decel_mps2=0.0)

    def test_non_finite_distance_is_safe(self):
        self.assertEqual(brake_envelope(float('nan'), self.p), 0.0)


# =============================================================================
# 阿克曼转向 —— 对应「网页上下左右手感不对」
# =============================================================================

class TestAckermann(unittest.TestCase):

    def setUp(self):
        self.geo = ChassisGeometry(wheelbase_m=0.25, track_m=0.17, max_steer_rad=0.35)

    def test_roundtrip_steer_yaw(self):
        for speed in (0.2, 0.55, 0.85, 1.2):
            for deg in (-20, -14, -8, -3, 0, 3, 8, 14, 20):
                steer = math.radians(deg)
                wz = yaw_from_steer(speed, steer, self.geo)
                back = steer_from_yaw(speed, wz, self.geo)
                self.assertAlmostEqual(back, steer, places=6,
                                       msg=f"v={speed} steer={deg}deg 往返不一致")

    def test_turn_radius_independent_of_speed(self):
        """同一个转角档,换速度档不应该改变转弯半径 —— 这是本次改动的目的。"""
        steer = math.radians(14)
        radii = []
        for speed in (0.30, 0.55, 0.85):
            wz = yaw_from_steer(speed, steer, self.geo)
            radii.append(speed / wz)
        for r in radii[1:]:
            self.assertAlmostEqual(r, radii[0], places=9)

    def test_old_ui_values_all_saturated(self):
        """证明旧版三个速度档的「左拐」全部打满,档位形同虚设。"""
        old = {'low': (0.35, 0.60), 'med': (0.50, 0.80), 'high': (0.65, 0.95)}
        angles = {}
        for tier, (vx, wz) in old.items():
            angles[tier] = steer_from_yaw(vx, wz, self.geo)
            self.assertAlmostEqual(angles[tier], self.geo.max_steer_rad, places=6,
                                   msg=f"{tier} 档理应已经打满")
        self.assertEqual(len(set(round(a, 9) for a in angles.values())), 1)

    def test_yaw_never_exceeds_physical_limit(self):
        for speed in (0.1, 0.5, 1.0, 1.3):
            wz = yaw_from_steer(speed, 10.0, self.geo)   # 请求一个荒谬的大转角
            self.assertLessEqual(abs(wz), max_yaw_at_speed(speed, self.geo) + 1e-9)

    def test_zero_speed_gives_zero_yaw(self):
        # 固件在 Vx==0 时强制舵机归中,而且驱动层会拒绝原地转向指令
        self.assertEqual(yaw_from_steer(0.0, 0.3, self.geo), 0.0)

    def test_reverse_sign_convention(self):
        """倒车时同样的舵角,车身横摆方向相反。"""
        fwd = yaw_from_steer(+0.5, math.radians(15), self.geo)
        rev = yaw_from_steer(-0.5, math.radians(15), self.geo)
        self.assertGreater(fwd, 0)
        self.assertLess(rev, 0)
        self.assertAlmostEqual(abs(fwd), abs(rev), places=9)


# =============================================================================
# 滤波与限幅
# =============================================================================

class TestFilters(unittest.TestCase):

    def test_alpha_beta_tracks_constant_velocity_without_lag(self):
        """匀速接近的目标,alpha-beta 应收敛到零稳态滞后;EMA 做不到。"""
        tracker = AlphaBetaTracker(alpha=0.45, beta=0.10)
        dt, v_true = 0.1, -0.4          # 每秒接近 0.4m
        z = 3.0
        for i in range(120):
            z += v_true * dt
            tracker.update(z, i * dt)
        self.assertAlmostEqual(tracker.position, z, delta=0.02)
        self.assertAlmostEqual(tracker.velocity, v_true, delta=0.05)

    def test_ema_lag_is_real(self):
        """量化旧版 EMA 的滞后,说明它为什么会让车读到偏大的距离。"""
        dt, v_true = 0.1, -0.4
        z, ema = 3.0, 3.0
        for _ in range(120):
            z += v_true * dt
            ema = 0.65 * ema + 0.35 * z
        self.assertGreater(ema - z, 0.05, "EMA 读到的距离应当明显偏大")

    def test_predict_extrapolates(self):
        tracker = AlphaBetaTracker()
        for i in range(60):
            tracker.update(2.0 - 0.3 * i * 0.1, i * 0.1)
        ahead = tracker.predict(0.2)
        self.assertLess(ahead, tracker.position)

    def test_slew_brakes_faster_than_it_accelerates(self):
        s = SlewLimiter(accel_limit=0.9, decel_limit=2.5)
        s.step(1.0, 0.05)
        self.assertAlmostEqual(s.value, 0.045, places=6)
        s.reset(0.5)
        s.step(0.0, 0.05)
        self.assertAlmostEqual(s.value, 0.375, places=6)

    def test_breakaway_kick_respects_envelope(self):
        """起步脉冲绝不允许突破刹车包络 —— 快贴到人了就不该有起步冲动。"""
        kick = BreakawayKick(kick_mps=0.22, duration_s=0.25, creep_floor_mps=0.08)
        out = kick.apply(desired_mps=0.20, measured_moving=False, speed_cap=0.10, now=0.0)
        self.assertLessEqual(out, 0.10)

    def test_breakaway_kick_helps_from_rest(self):
        kick = BreakawayKick(kick_mps=0.22, duration_s=0.25, creep_floor_mps=0.08)
        out = kick.apply(desired_mps=0.12, measured_moving=False, speed_cap=0.60, now=0.0)
        self.assertAlmostEqual(out, 0.22, places=6)

    def test_below_creep_floor_commands_zero(self):
        """取代旧版 MIN_SPEED 地板值:太慢就干脆停,而不是硬撑 0.25m/s。"""
        kick = BreakawayKick(creep_floor_mps=0.08)
        self.assertEqual(kick.apply(0.03, False, 0.6, 0.0), 0.0)


# =============================================================================
# 驱动状态机 —— 对应「网页前进后退极慢」
# =============================================================================

def make_policy(**overrides):
    cfg = dict(protocol="twist", protocol_confirmed=True, receive_only=False,
               wheelbase_m=0.25, max_speed_m_s=1.30, max_yaw_rate_rad_s=1.0,
               acceleration_m_s2=3.5, steering_rate_rad_s=2.5,
               cmd_timeout_s=0.5, feedback_timeout_s=0.30, tx_hz=50.0)
    cfg.update(overrides)
    policy = ControlPolicy(Config(**cfg), 0.0)
    policy.link(True, 0.0)
    return policy


def feed_stationary(policy, t, frames=6):
    for i in range(frames):
        policy.feedback({"velocity": [0.0, 0.0, 0.0]}, t + i * 0.05)
    return t + frames * 0.05


class TestDriverRecovery(unittest.TestCase):

    def test_arms_after_stationary_telemetry(self):
        p = make_policy()
        t = feed_stationary(p, 4.0)
        ok, reason = p.arm(t)
        self.assertTrue(ok, reason)

    def test_single_backlog_does_not_disarm(self):
        """一次串口 backlog 只是 hold,车仍然 armed,不需要停车重新使能。

        旧版在这里直接 stop(),而重新 arm 又要求 5 帧静止遥测,于是形成
        「开动 -> 锁停 -> 滑行到停 -> 重新使能 -> 再开动」的顿挫循环,
        表现就是网页上前进后退慢得离谱。
        """
        p = make_policy()
        t = feed_stationary(p, 4.0)
        self.assertTrue(p.arm(t)[0])
        p.hold("serial_output_backlog", t)
        self.assertTrue(p.armed, "瞬时故障不应当解除使能")
        self.assertTrue(p.holding)
        self.assertEqual(p.tick(t + 0.01), STOP_FRAME)
        p.release_hold()
        self.assertTrue(p.armed)
        self.assertFalse(p.holding)

    def test_persistent_fault_escalates_to_disarm(self):
        p = make_policy(feedback_grace_s=0.5)
        t = feed_stationary(p, 4.0)
        self.assertTrue(p.arm(t)[0])
        p.hold("serial_output_backlog", t)
        p.tick(t + 0.9)          # 超过 grace
        self.assertFalse(p.armed, "持续故障必须升级为硬解除使能")

    def test_moving_vehicle_survives_one_stale_telemetry_frame(self):
        p = make_policy(feedback_grace_s=1.0)
        t = feed_stationary(p, 4.0)
        self.assertTrue(p.arm(t)[0])
        p.command("twist", 0.8, 0.0, t)
        p.tick(t + 0.02)
        p.feedback({"velocity": [0.5, 0.0, 0.0]}, t + 0.05)   # 车在动
        p.tick(t + 0.40)         # 遥测停了 350ms,超过 feedback_timeout
        self.assertTrue(p.armed, "一次遥测抖动不该解除使能")
        p.feedback({"velocity": [0.5, 0.0, 0.0]}, t + 0.45)   # 遥测回来了
        p.tick(t + 0.46)
        self.assertFalse(p.holding, "遥测恢复后应当自动放行")

    def test_rejected_command_keeps_arm(self):
        """原地转向指令会被拒绝,但不该把底盘打成未使能。"""
        p = make_policy()
        t = feed_stationary(p, 4.0)
        self.assertTrue(p.arm(t)[0])
        with self.assertRaises(ValueError):
            p.command("twist", 0.0, 0.5, t)
        self.assertTrue(p.armed, "被拒绝的单条指令不应当解除使能")

    def test_dt_cap_allows_slower_than_nominal_loop(self):
        """dt 上限曾被写成标称周期,循环稍慢加速斜坡就被拉长。"""
        p = make_policy(tx_hz=50.0, acceleration_m_s2=3.5)
        t = feed_stationary(p, 4.0)
        self.assertTrue(p.arm(t)[0])
        p.command("twist", 1.0, 0.0, t)
        p.last_tick = t
        p.tick(t + 0.05)                       # 真实间隔 50ms,是标称的 2.5 倍
        self.assertGreater(p.output[0], 3.5 * (1 / 50.0) * 1.5,
                           "应当按真实经过时间累加,而不是被压成标称周期")

    def test_hard_fault_still_requires_stationary_rearm(self):
        """安全兜底没有被削弱:真故障仍然要求车停稳才能重新使能。"""
        p = make_policy()
        t = feed_stationary(p, 4.0)
        self.assertTrue(p.arm(t)[0])
        p.stop("feedback_lost")
        p.feedback({"velocity": [0.4, 0.0, 0.0]}, t + 0.1)   # 车还在动
        self.assertNotEqual(p.ready(t + 0.2), "ready")
        ok, _ = p.arm(t + 0.2)
        self.assertFalse(ok)


class TestWireFormat(unittest.TestCase):
    """帧格式未被本次改动影响 —— 对照 PROTOCOL.md 第二、三节。"""

    def test_stop_frame_matches_builder(self):
        self.assertEqual(build_frame(0.0, 0.0, 0), STOP_FRAME)

    def test_frame_length_and_checksum(self):
        frame = build_frame(0.85, -0.42, 0)
        self.assertEqual(len(frame), 11)
        self.assertEqual(frame[0], 0x7B)
        self.assertEqual(frame[-1], 0x7D)
        checksum = 0
        for b in frame[:9]:
            checksum ^= b
        self.assertEqual(frame[9], checksum)

    def test_units_are_millimetres_per_second(self):
        frame = build_frame(0.85, 0.0, 0)
        self.assertEqual(int.from_bytes(frame[3:5], 'big', signed=True), 850)

    def test_pre_steer_frame_from_protocol_doc(self):
        """PROTOCOL.md 8.3 记录的微速度预打舵帧:7b 00 00 00 05 00 00 00 14 6e 7d"""
        # 注意:文档此前把校验字节记成 6e,实际按第三节规则算出来是 6a。
        # 本次一并勘误,这条测试就是那个勘误的回归保护。
        self.assertEqual(build_frame(0.005, 0.020, 0).hex(" "),
                         "7b 00 00 00 05 00 00 00 14 6a 7d")


if __name__ == "__main__":
    unittest.main(verbosity=2)
