#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""倾斜补偿：填了天线高度（FusionConfig.antenna_height_m）时，融合按 IMU 的横滚、俯仰把 ANT1 的三维杠杆臂
(antenna_xy, 高度) 投到水平面。车在横坡上天线偏出去的那段（高 0.5 m、横坡 3° 约 2.6 cm）不再算成车的位置。
姿态断了、太陡、IMU 已不用时退回平地杠杆臂：只提示 tilt_uncompensated，不停车。"""
import math
import os
import random
import sys
import unittest
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'radar_system', 'tools'))

import ros_stubs  # noqa: E402

ros_stubs.install()
sys.modules['sensor_msgs.msg'].Imu = ros_stubs._simple('Imu')

import app_link as al  # noqa: E402
import export_app_fixtures as fixtures  # noqa: E402
import gnss_fusion as gf  # noqa: E402
import line_node  # noqa: E402

ANTENNA = (0.345, 0.165)


def rotate(roll, pitch, yaw, v):
    """REP-103 R = Rz(yaw)·Ry(pitch)·Rx(roll) 乘 v，用矩阵逐项写，与融合里的公式互相独立。"""
    def mul(a, b):
        return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]
    ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
    rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]
    r = mul(rz, mul(ry, rx))
    return [sum(r[i][k] * v[k] for k in range(3)) for i in range(3)]


def drive(height_cfg, T=10.0, roll=lambda t: math.radians(3.0), pitch=lambda t: math.radians(1.5),
          attitude_on=lambda t: True, true_height=0.5, heading_deg=30.0, speed=0.5):
    """直行在斜坡上：天线真值 = 后轴地面点 + R·(antenna_xy, true_height)。融合收轮速、陀螺（带横滚俯仰）、
    ANT1 固定解和双天线测向，50 / 25 Hz。返回 (融合, 最后的位姿, 最后的真值 (x, y))。"""
    f = gf.GnssFusion(gf.FusionConfig(antenna_xy=ANTENNA, antenna_height_m=height_cfg))
    f.set_origin(30.0, 120.0, 0.0)
    yaw = math.radians(heading_deg)
    x = y = 0.0
    dt, t, k = 0.004, 0.0, 0
    while t < T - 1e-9:
        x += speed * math.cos(yaw) * dt
        y += speed * math.sin(yaw) * dt
        t += dt
        k += 1
        if k % 5 == 0:
            f.add_wheel(gf.WheelSample(t, speed, 0.0))
            if attitude_on(t):
                f.add_imu(gf.ImuSample(t, 0.0, None, roll(t), pitch(t)))
            else:
                f.add_imu(gf.ImuSample(t, 0.0))
        if k % 10 == 0:
            ex, ey, _ = rotate(roll(t), pitch(t), yaw, [ANTENNA[0], ANTENNA[1], true_height])
            lat, lon = f.to_geodetic(x + ex, y + ey)
            f.add_fix(gf.GnssFix(t, lat, lon, 0.0, 'NARROW_INT', 0.01, 0.01, 0.02, 1.0, 30))
            f.add_heading(gf.DualHeading(t, (180.0 - math.degrees(yaw)) % 360.0, 0.0, 0.4, 'NARROW_INT', 0.1, 30))
    return f, f.pose(t), (x, y)


class Lever(unittest.TestCase):
    def fusion(self, roll_deg, pitch_deg, **cfg):
        f = gf.GnssFusion(gf.FusionConfig(antenna_xy=ANTENNA, **cfg))
        f.add_wheel(gf.WheelSample(1.0, 0.0, 0.0))
        f.add_imu(gf.ImuSample(1.0, 0.0, None, math.radians(roll_deg), math.radians(pitch_deg)))
        return f

    def test_matches_full_rotation(self):
        rng = random.Random(7)
        for _ in range(50):
            r, p, h = rng.uniform(-14, 14), rng.uniform(-14, 14), rng.uniform(0.05, 1.5)
            f = self.fusion(r, p, antenna_height_m=h)
            ex, ey, _ = rotate(math.radians(r), math.radians(p), 0.0, [ANTENNA[0], ANTENNA[1], h])
            lx, ly = f.antenna_lever()
            self.assertAlmostEqual(lx, ex, places=12)
            self.assertAlmostEqual(ly, ey, places=12)

    def test_signs_follow_rep103(self):
        # 左侧抬起（横滚为正）：天线往右偏；车头压低（俯仰为正）：天线往前偏
        lx, ly = self.fusion(2.0, 0.0, antenna_height_m=0.5).antenna_lever()
        self.assertAlmostEqual(ly - ANTENNA[1], -0.5 * math.sin(math.radians(2.0)), delta=5e-4)
        lx, ly = self.fusion(0.0, 2.0, antenna_height_m=0.5).antenna_lever()
        self.assertAlmostEqual(lx - ANTENNA[0], 0.5 * math.sin(math.radians(2.0)), delta=5e-4)

    def test_level_or_no_height_is_the_old_lever(self):
        self.assertEqual(self.fusion(0.0, 0.0, antenna_height_m=0.5).antenna_lever(), ANTENNA)
        f = self.fusion(3.0, 2.0)                       # 没填高度：和以前完全一样
        self.assertEqual(f.antenna_lever(), ANTENNA)
        st = f.tilt_status()
        self.assertEqual((st['active'], st['reason'], st['height_m']), (False, 'no_height', 0.0))

    def test_too_steep_is_not_compensated(self):
        f = self.fusion(20.0, 0.0, antenna_height_m=0.5)
        self.assertEqual(f.antenna_lever(), ANTENNA)
        self.assertEqual(f.tilt_status()['reason'], 'too_steep')

    def test_status_reports_offset(self):
        st = self.fusion(1.25, -2.4, antenna_height_m=0.4).tilt_status()
        self.assertTrue(st['active'])
        self.assertIsNone(st['reason'])
        self.assertAlmostEqual(st['forward_m'], -0.0172, delta=1e-4)       # 车头抬起：天线往后
        self.assertAlmostEqual(st['left_m'], -0.0088, delta=1e-4)          # 左侧抬起：天线往右
        self.assertAlmostEqual(st['roll_deg'], 1.25)

    def test_config_validation(self):
        self.assertRaises(ValueError, gf.FusionConfig, antenna_height_m=-0.1)
        self.assertRaises(ValueError, gf.FusionConfig, antenna_height_m=40.0)      # 填成厘米了
        self.assertRaises(ValueError, gf.FusionConfig, max_tilt_deg=0.0)
        self.assertRaises(ValueError, gf.FusionConfig, max_tilt_deg=60.0)
        self.assertEqual(gf.FusionConfig().antenna_height_m, 0.0)                   # 默认不补


class OnSlope(unittest.TestCase):
    def test_compensated_position_is_the_ground_point(self):
        _, pose, (x, y) = drive(0.5)
        self.assertLess(math.hypot(pose.x - x, pose.y - y), 0.003)
        self.assertTrue(pose.usable_for_control)
        self.assertNotIn('tilt_uncompensated', pose.reasons)

    def test_without_height_the_slope_shifts_the_car(self):
        # 对照：不补时车的位置偏出 天线高 × sin(横坡)，约 2.6 cm（本来就是这样，只作对照）
        _, pose, (x, y) = drive(0.0)
        err = math.hypot(pose.x - x, pose.y - y)
        self.assertGreater(err, 0.02)
        self.assertNotIn('tilt_uncompensated', pose.reasons)       # 没填高度不算问题

    def test_changing_slope_is_followed(self):
        roll = lambda t: math.radians(0.0 if t < 4.0 else 4.0)    # noqa: E731  开上横坡
        _, pose, (x, y) = drive(0.5, roll=roll, pitch=lambda t: 0.0)
        self.assertLess(math.hypot(pose.x - x, pose.y - y), 0.004)

    def test_attitude_lost_degrades_but_does_not_stop(self):
        # 姿态断了：退回平地杠杆臂，只提示，照常能开（用户：IMU 不稳不能导致停车）
        f, pose, (x, y) = drive(0.5, attitude_on=lambda t: t < 5.0)
        self.assertTrue(pose.usable_for_control)
        self.assertIn('tilt_uncompensated', pose.reasons)
        self.assertEqual(f.tilt_status()['reason'], 'attitude_stale')
        self.assertLess(math.hypot(pose.x - x, pose.y - y), 0.04)


class Wiring(unittest.TestCase):
    class Mission:
        def __init__(self, yaw_input):
            self.fusion = SimpleNamespace(config=gf.FusionConfig(yaw_input=yaw_input))
            self.calls = []

        def on_imu(self, t, gz, yaw=None, roll=None, pitch=None):
            self.calls.append((gz, yaw, roll, pitch))

    @staticmethod
    def imu(roll_deg=None, pitch_deg=0.0, gz=0.3):
        msg = sys.modules['sensor_msgs.msg'].Imu()
        msg.angular_velocity.z = gz
        if roll_deg is None:
            msg.orientation_covariance[0] = -1.0
            return msg
        r, p = math.radians(roll_deg) / 2, math.radians(pitch_deg) / 2
        msg.orientation.x = math.sin(r) * math.cos(p)
        msg.orientation.y = math.cos(r) * math.sin(p)
        msg.orientation.z = -math.sin(r) * math.sin(p)
        msg.orientation.w = math.cos(r) * math.cos(p)
        return msg

    def test_angle_mode_passes_roll_pitch(self):
        m = self.Mission('angle')
        node = line_node.LineNode(m)
        node.on_attitude(self.imu(roll_deg=2.0, pitch_deg=-1.0))
        node.on_attitude(self.imu())                    # 这一帧没带姿态：只给陀螺
        self.assertAlmostEqual(m.calls[0][2], math.radians(2.0), places=6)
        self.assertAlmostEqual(m.calls[0][3], math.radians(-1.0), places=6)
        self.assertEqual(m.calls[1][1:], (None, None, None))

    def test_rate_mode_attaches_fresh_attitude(self):
        now = [10.0]
        m = self.Mission('rate')
        node = line_node.LineNode(m, clock=lambda: now[0])
        node.on_imu(self.imu())                         # 还没有姿态
        node.on_attitude(self.imu(roll_deg=3.0))        # 旧车 /imu/data：只记下
        node.on_imu(self.imu())
        now[0] += 0.5
        node.on_imu(self.imu())                         # 姿态旧了：不带
        self.assertEqual(len(m.calls), 3)
        self.assertEqual(m.calls[0][2:], (None, None))
        self.assertAlmostEqual(m.calls[1][2], math.radians(3.0), places=6)
        self.assertEqual(m.calls[2][2:], (None, None))


class Telemetry(unittest.TestCase):
    def test_tilt_passes_through_and_rejects_bad_input(self):
        ln = fixtures.sample_route_line(now=100.0)
        self.assertEqual(al.line_telemetry(ln, 100.0)['tilt'],
                         {'height_m': 0.4, 'active': True, 'reason': None, 'roll_deg': 1.25, 'pitch_deg': -2.4,
                          'forward_m': -0.0172, 'left_m': -0.0088})
        off = dict(ln, tilt={'height_m': 0.4, 'active': False, 'reason': 'attitude_stale', 'roll_deg': None,
                             'pitch_deg': None, 'forward_m': None, 'left_m': None})
        self.assertEqual(al.line_telemetry(off, 100.0)['tilt']['reason'], 'attitude_stale')
        self.assertIsNone(al.line_telemetry(dict(ln, tilt=None), 100.0)['tilt'])
        self.assertIsNone(al.line_telemetry(dict(ln, tilt={'active': 'yes'}), 100.0)['tilt'])
        self.assertIsNone(al.line_telemetry(fixtures.sample_line(now=100.0), 100.0)['tilt'])     # 旧车端没这项


if __name__ == '__main__':
    unittest.main()
