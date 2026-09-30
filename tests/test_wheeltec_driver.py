#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""旧阿克曼车（Wheeltec 底盘）无硬件回归测试:转角 / 角速度换算、驱动状态机、串口帧格式。

不需要 ROS、不需要串口、不需要实车。跑法:

    python3 tests/test_wheeltec_driver.py
    # 或
    python3 -m unittest discover tests -v
"""

import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "wheeltec_protocol"))

from robot_core.kinematics import (  # noqa: E402
    ChassisGeometry, yaw_from_steer, steer_from_yaw, max_yaw_at_speed,
)
from wheeltec_driver import Config, ControlPolicy, build_frame, STOP_FRAME  # noqa: E402


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

    def test_fresh_zero_speed_feedback_does_not_latch_motion(self):
        """持续不动交给上层有限脱困；驱动只因真正的健康故障停车。"""
        p = make_policy()
        t = feed_stationary(p, 4.0)
        self.assertTrue(p.arm(t)[0])
        for i in range(1, 101):
            now = t + i * 0.02
            p.feedback({"velocity": [0.0, 0.0, 0.0]}, now)
            p.command("twist", 0.85, 0.0, now)
            frame = p.tick(now)
            self.assertNotEqual(frame, STOP_FRAME)
            self.assertTrue(p.armed)
        self.assertGreater(p.output[0], 0.06)
        self.assertEqual(p.reason, "command_active")

        # Removing the speed mismatch latch must not weaken feedback loss.
        self.assertEqual(p.tick(now + 0.31), STOP_FRAME)
        self.assertTrue(p.holding)
        self.assertEqual(p.tick(now + 1.4), STOP_FRAME)
        self.assertFalse(p.armed)

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
