#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gen5 融合用 IMU 自己算的航向角（yaw_input = 'angle'，2026-09-28 用户要求）：双天线定朝哪，两次测向之间转了多少
按 HDA536T 航向角的增量算。航向以双天线为主（同日用户要求）：测向稳定就必须信它，IMU 出问题只降级、不停车，
没有测向时才不可控。仿真 Gen5 原地转 90°（0.8 rad/s），IMU 航向角上电为 0、0.01° 一格。"""
import math
import os
import sys
import unittest
from unittest import mock

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, ROOT)

import ros_stubs  # noqa: E402

ros_stubs.install()
sys.modules['sensor_msgs.msg'].Imu = ros_stubs._simple('Imu')

import gnss_fusion as gf  # noqa: E402
import line_node  # noqa: E402

GEN5 = dict(drive_differential=True, yaw_input='angle', yaw_scale_sigma=0.03, heading_sync_s=0.02, yaw_walk_deg_sqrt_s=0.1)
PIVOT_AT = 6.0


def _pivot_rate(t, angle=math.pi / 2, rate=0.8, acc=1.0):
    """原地转向角速度（梯形），rad/s。"""
    ta = rate / acc
    tc = (angle - rate * ta) / rate
    x = t - PIVOT_AT
    if x < 0 or x > 2 * ta + tc:
        return 0.0
    return acc * x if x < ta else rate if x < ta + tc else rate - acc * (x - ta - tc)


def pivot_rate(t, angle=math.pi / 2, rate=0.8, acc=1.0):
    return _pivot_rate(t, angle, rate, acc)



def simulate(config, T=12.0, heading0_deg=60.0, imu_scale=1.0, imu_sign=1.0, gyro_z=None,
             rtk_heading=lambda t: True, rtk_offset=lambda t: 0.0, imu_on=lambda t: True, yaw_jump=None, marks=(),
             odom_scale=1.0):
    """直行 → 停车 → 原地转 90° → 直行。IMU 航向角 = 上电 0 起、按 imu_scale 转过的角度、0.01° 量化；
    gyro_z 给了就用它当陀螺读数（证明推算用的是航向角），yaw_jump=(t, 度) 模拟航向清零；imu_on(t) 为假时这一帧 IMU
    没来；rtk_offset(t) 加在测向上（度）；odom_scale 是里程计角速度 / 真实转速（低速转动时读小）。返回 (融合, {时刻: (融合航向与真值差°, 位姿)})。"""
    f = gf.GnssFusion(gf.FusionConfig(**config))
    f.set_origin(30.0, 120.0, 0.0)
    ax, ay = f.config.antenna_xy
    x = y = 0.0
    yaw = math.radians(heading0_deg)
    imu_yaw = 0.0
    dt, t, k = 0.004, 0.0, 0
    out = {}
    while t < T - 1e-9:
        w = pivot_rate(t)
        v = 0.5 if (t < PIVOT_AT - 1.0 or t > PIVOT_AT + 3.5) else 0.0
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += w * dt
        imu_yaw += imu_sign * imu_scale * w * dt
        if yaw_jump and abs(t - yaw_jump[0]) < dt / 2:
            imu_yaw += math.radians(yaw_jump[1])
        t += dt
        k += 1
        if k % 5 == 0:          # IMU、轮速 50 Hz
            q = gf._wrap(round(math.degrees(imu_yaw) / 0.01) * math.radians(0.01))
            g = imu_sign * w if gyro_z is None else gyro_z
            f.add_wheel(gf.WheelSample(t, v, w * odom_scale))
            if imu_on(t):
                f.add_imu(gf.ImuSample(t, g, q if config.get('yaw_input') == 'angle' else None))
        if k % 10 == 0:         # RTK 位置、测向 25 Hz
            px = x + math.cos(yaw) * ax - math.sin(yaw) * ay
            py = y + math.sin(yaw) * ax + math.cos(yaw) * ay
            lat, lon = f.to_geodetic(px, py)
            f.add_fix(gf.GnssFix(t, lat, lon, 0.0, 'NARROW_INT', 0.01, 0.01, 0.02, 1.0, 30))
            if rtk_heading(t):
                f.add_heading(gf.DualHeading(t, (180.0 - math.degrees(yaw) - rtk_offset(t)) % 360.0, 0.0, 0.4, 'NARROW_INT', 0.1, 30))
        for m in marks:
            if abs(t - m) < dt / 2:
                p = f.pose(t)
                out[m] = (math.degrees(gf._wrap(p.yaw - yaw)), p)
    return f, out


class AngleInput(unittest.TestCase):
    def test_heading_follows_imu_angle_not_gyro(self):
        # 陀螺读数一直是 0：转过去全靠 IMU 航向角；转向中、转完后测向都断了，航向只能来自航向角
        f, out = simulate(GEN5, gyro_z=0.0, rtk_heading=lambda t: t < PIVOT_AT, marks=(PIVOT_AT + 3.0, PIVOT_AT + 5.0))
        for m, (err, p) in out.items():
            self.assertLess(abs(err), 0.05, m)          # 0.01° 量化，转 90° 后照样对
            self.assertIn('gyro_only', p.reasons)
        # 同样的数据在陀螺模式下转不过去
        f, out = simulate(dict(drive_differential=True), gyro_z=0.0, rtk_heading=lambda t: t < PIVOT_AT,
                          marks=(PIVOT_AT + 3.0,))
        self.assertGreater(abs(out[PIVOT_AT + 3.0][0]), 80.0)

    def test_power_on_zero_is_anchored_by_dual_antenna(self):
        # IMU 航向角上电为 0，车头其实朝 60°：朝哪由双天线定
        f, out = simulate(GEN5, marks=(3.0,))
        err, p = out[3.0]
        self.assertLess(abs(err), 0.05)
        self.assertTrue(p.usable_for_control, p.reasons)

    def test_pivot_with_3pct_scale_error_is_pulled_back_by_rtk(self):
        # IMU 航向刻度大 3%：转 90° 多算 2.7°；转向时航向不确定度按刻度加大，测向照样收下，转完就回来
        f, out = simulate(GEN5, imu_scale=1.03, marks=(PIVOT_AT + 3.0, PIVOT_AT + 4.0))
        self.assertLess(abs(out[PIVOT_AT + 3.0][0]), 0.2)
        self.assertLess(abs(out[PIVOT_AT + 4.0][0]), 0.05)
        self.assertTrue(out[PIVOT_AT + 4.0][1].usable_for_control)
        self.assertLess(f._s.heading_rejected, 5)

    def test_rtk_lockout_is_resolved_by_reanchor_without_stopping(self):
        # 不按刻度加不确定度（旧的做法）：3% 会把正确的测向拒掉；测向稳定、车一停转，0.5 s 后按测向重定，一直能开
        cfg = dict(GEN5, yaw_scale_sigma=0.0, heading_sync_s=0.0, yaw_walk_deg_sqrt_s=0.01)
        f, out = simulate(cfg, imu_scale=1.03, marks=(PIVOT_AT + 2.9, PIVOT_AT + 4.0, PIVOT_AT + 5.0))
        self.assertGreater(abs(out[PIVOT_AT + 2.9][0]), 2.0)            # 刚转完：确实被拒着
        self.assertLess(abs(out[PIVOT_AT + 4.0][0]), 0.1)
        self.assertGreaterEqual(f._s.heading_resets, 1)
        for m, (err, p) in out.items():
            self.assertTrue(p.usable_for_control, (m, p.reasons))

    def test_imu_is_kept_when_odometry_under_reads_the_turn(self):
        # 2026-09-30 实车：原地转向收尾、低速修方向时里程计只报 IMU 的 1/3–1/5。按「1.5 倍里程计角速度」卡 IMU，
        # 真实转动被当尖峰丢掉（一次作业 725 帧丢 201 帧、少算 26°），融合航向转少，正确的测向、固定解都被拒，
        # 最后定位重置停车（「开始就报定位跳」）。差速车航向角模式 70°/s 以内都收
        f, out = simulate(GEN5, odom_scale=0.25, marks=tuple(PIVOT_AT + d for d in (0.5, 1.0, 2.0, 3.0, 5.0)))
        self.assertLessEqual(f._s.gyro_rejected, 1)
        self.assertEqual(f._s.heading_rejected, 0)
        self.assertEqual(f._s.resets, 0)
        for m, (err, p) in out.items():
            self.assertLess(abs(err), 1.0, m)
            self.assertTrue(p.usable_for_control, (m, p.reasons))

    def test_yaw_reset_jump_is_dropped(self):
        # 直行中航向角突然跳 37°（航向清零、传感器重启）：这一帧的差当尖峰丢掉，下一帧从新的角度接着算
        f, out = simulate(GEN5, yaw_jump=(3.0, 37.0), rtk_heading=lambda t: t < 2.9, marks=(4.0,))
        self.assertLess(abs(out[4.0][0]), 0.05)
        self.assertGreaterEqual(f._s.gyro_rejected, 1)

    def test_reversed_imu_is_dropped_and_rtk_takes_over(self):
        # IMU 装反（或坐标换算错）：原地转向时与轮速里程计方向相反，0.5 s 内判出，之后不再用 IMU；航向交给测向
        # （两次测向之间按里程计推），测向有效就照常能开，不停车
        f, out = simulate(GEN5, imu_sign=-1.0, T=14.0, marks=(PIVOT_AT + 0.8, PIVOT_AT + 3.0, PIVOT_AT + 7.0))
        for m, (err, p) in out.items():
            self.assertIn('imu_mismatch', p.reasons)
            self.assertTrue(p.usable_for_control, (m, p.reasons))
            self.assertLess(abs(err), 0.5, m)
        # 陀螺模式的差速车同样核对
        f, out = simulate(dict(drive_differential=True), imu_sign=-1.0, marks=(PIVOT_AT + 0.8,))
        self.assertIn('imu_mismatch', out[PIVOT_AT + 0.8][1].reasons)
        # 没有测向时就没有航向来源了：不可控
        f, out = simulate(GEN5, imu_sign=-1.0, rtk_heading=lambda t: t < PIVOT_AT + 1.0, marks=(PIVOT_AT + 2.0,))
        self.assertFalse(out[PIVOT_AT + 2.0][1].usable_for_control)

    def test_imu_dropout_does_not_stop_while_rtk_heading_is_valid(self):
        # IMU 从 3 s 起断了、原地转向时也没有：测向有效就照常能开，转向按里程计推、测向拉住
        f, out = simulate(GEN5, imu_on=lambda t: t < 3.0, marks=(4.0, PIVOT_AT + 1.0, PIVOT_AT + 3.0, PIVOT_AT + 5.0))
        for m, (err, p) in out.items():
            self.assertIn('imu_stale', p.reasons)
            self.assertTrue(p.usable_for_control, (m, p.reasons))
            self.assertLess(abs(err), 0.5, m)
        self.assertLess(abs(out[PIVOT_AT + 5.0][0]), 0.1)
        # IMU 和测向都没了：不可控
        f, out = simulate(GEN5, imu_on=lambda t: t < 3.0, rtk_heading=lambda t: t < 4.0, marks=(4.5,))
        self.assertFalse(out[4.5][1].usable_for_control)
        self.assertIn('imu_stale', out[4.5][1].reasons)

    def test_half_turn_without_imu_does_not_stop(self):
        # 原地转 180°、IMU 从 3 s 起就断了：转向中按保守模型涨的航向不确定度有测向拉着，全程能开
        with mock.patch.object(sys.modules[__name__], 'pivot_rate',
                               lambda t, angle=math.pi, rate=0.8, acc=1.0: _pivot_rate(t, angle, rate, acc)):
            marks = tuple(PIVOT_AT + x / 10 for x in range(0, 60, 2))
            f, out = simulate(GEN5, T=14.0, imu_on=lambda t: t < 3.0, marks=marks)
        self.assertTrue(all(p.usable_for_control for _, p in out.values()),
                        [(m, p.reasons) for m, (_, p) in out.items() if not p.usable_for_control])
        self.assertLess(max(abs(e) for e, _ in out.values()), 0.5)

    def test_rtk_heading_leads(self):
        # 3 s 起测向整体偏 0.3°（IMU 照实）：航向以测向为主，0.5 s 内就跟过去
        f, out = simulate(GEN5, rtk_offset=lambda t: 0.3 if t >= 3.0 else 0.0, marks=(3.5, 5.0))
        self.assertGreater(out[3.5][0], 0.25)
        self.assertAlmostEqual(out[5.0][0], 0.3, delta=0.05)

    def test_short_or_unstable_rtk_glitches_are_not_followed(self):
        # 测向跳 3° 只持续 0.2 s：当离群值丢掉，不重定
        f, out = simulate(GEN5, rtk_offset=lambda t: 3.0 if 3.0 <= t < 3.2 else 0.0, marks=(3.1, 4.0))
        self.assertLess(abs(out[3.1][0]), 0.5)
        self.assertLess(abs(out[4.0][0]), 0.05)
        self.assertEqual(f._s.heading_resets, 0)
        # 测向在 ±2° 之间来回跳（不稳定）：不重定
        f, out = simulate(GEN5, rtk_offset=lambda t: (2.0 if int(t * 25) % 2 else -2.0) if t >= 3.0 else 0.0, marks=(4.5,))
        self.assertEqual(f._s.heading_resets, 0)
        self.assertLess(abs(out[4.5][0]), 1.0)

    def test_stable_rtk_step_is_followed_even_when_large(self):
        # 测向突然整体偏 2° 并一直保持（稳定）：比门限大，先被拒，0.5 s 后按测向重定，一直能开
        f, out = simulate(GEN5, rtk_offset=lambda t: 2.0 if t >= 3.0 else 0.0, marks=(3.2, 4.0))
        self.assertAlmostEqual(out[4.0][0], 2.0, delta=0.05)
        self.assertGreaterEqual(f._s.heading_resets, 1)
        self.assertTrue(all(p.usable_for_control for _, p in out.values()))

    def test_config_validation(self):
        self.assertRaises(ValueError, gf.FusionConfig, yaw_input='quaternion')
        self.assertRaises(ValueError, gf.FusionConfig, yaw_scale_sigma=-0.01)
        self.assertEqual(gf.FusionConfig().yaw_input, 'rate')          # 旧车默认不变


class NodeWiring(unittest.TestCase):
    class Mission:
        def __init__(self, yaw_input):
            self.fusion = type('F', (), {'config': gf.FusionConfig(yaw_input=yaw_input)})()
            self.calls = []

        def on_imu(self, t, gz, yaw=None, roll=None, pitch=None):
            self.calls.append((gz, yaw))

    @staticmethod
    def imu(yaw_deg=None, gz=0.3):
        msg = sys.modules['sensor_msgs.msg'].Imu()
        msg.angular_velocity.z = gz
        if yaw_deg is None:
            msg.orientation_covariance[0] = -1.0
        else:
            msg.orientation.x = msg.orientation.y = 0.0
            msg.orientation.z, msg.orientation.w = math.sin(math.radians(yaw_deg) / 2), math.cos(math.radians(yaw_deg) / 2)
        return msg

    def test_angle_mode_takes_imu_data_only(self):
        m = self.Mission('angle')
        node = line_node.LineNode(m)
        node.on_imu(self.imu())                         # /imu：不进融合
        node.on_attitude(self.imu(yaw_deg=30.0))        # /imu/data：航向角 + 陀螺
        node.on_attitude(self.imu(yaw_deg=None))        # 这一帧没带姿态：只给陀螺
        self.assertEqual(len(m.calls), 2)
        self.assertAlmostEqual(m.calls[0][1], math.radians(30.0))
        self.assertIsNone(m.calls[1][1])

    def test_rate_mode_is_unchanged(self):
        m = self.Mission('rate')
        node = line_node.LineNode(m)
        node.on_imu(self.imu())
        node.on_attitude(self.imu(yaw_deg=30.0))        # 旧车：/imu/data 只显示
        self.assertEqual(m.calls, [(0.3, None)])

    def test_gen5_config_uses_imu_angle(self):
        import runtime_config
        with mock.patch.dict(runtime_config.PROFILE['drive'], {'type': 'differential'}):
            cfg = line_node.load_fusion_config(gf, '/nonexistent/fusion.json')
        self.assertEqual((cfg.yaw_input, cfg.yaw_scale_sigma, cfg.heading_sync_s, cfg.yaw_walk_deg_sqrt_s),
                         ('angle', 0.03, 0.02, 0.1))
        with mock.patch.dict(runtime_config.PROFILE['drive'], {'type': 'ackermann'}):
            cfg = line_node.load_fusion_config(gf, '/nonexistent/fusion.json')
        self.assertEqual((cfg.yaw_input, cfg.yaw_scale_sigma, cfg.heading_sync_s), ('rate', 0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
