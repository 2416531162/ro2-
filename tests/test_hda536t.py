#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""瑞芬 HDA536T（Gen5 的 IMU，CAN）：编解码与厂家技术手册 V1.4 的示例报文逐字节一致；NED -> REP-103 的符号
（左转角速度为正、车头抬起俯仰为负、静止水平 z 加速度约 +9.8）；按手册行为模拟的传感器上跑接收逻辑
（出厂设置只有角度帧、自检失败、安装方式不对、断线都不发 /imu，状态写明原因）；ROS 替身上跑一遍节点。"""
import json
import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'rion_protocol'))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, ROOT)

import ros_stubs  # noqa: E402

ros_stubs.install()
sys.modules['sensor_msgs.msg'].Imu = ros_stubs._simple('Imu')

import hda536t_codec as hc  # noqa: E402
import hda536t_imu as hi  # noqa: E402
from line_node import attitude_deg  # noqa: E402

H = bytes.fromhex
NODE = 5
PDO = hc.SDO_REPLY + NODE       # 出厂数据帧 ID 0x585


def cycle(t, roll=0.0, pitch=0.0, yaw=0.0, gyro=(0.0, 0.0, 0.0), accel=(0.0, 0.0, -1.0), self_test=0, mount=0,
          kinds=(hc.ANGLE, hc.ACCEL, hc.GYRO), temp=25):
    """传感器一个输出周期连发的几帧（NED 轴，角度 °、加速度 g、陀螺 °/s）。"""
    frames = {hc.ANGLE: hc.encode_pdo(hc.ANGLE, roll, pitch, yaw, temp, self_test, mount),
              hc.ACCEL: hc.encode_pdo(hc.ACCEL, *accel, self_test=self_test, mount=mount),
              hc.GYRO: hc.encode_pdo(hc.GYRO, *gyro, self_test=self_test, mount=mount)}
    return [(t + 1e-4 * i, PDO, frames[k]) for i, k in enumerate(kinds)]


def run(imu, seconds, period=0.02, t0=0.0, **kw):
    out, t = [], t0
    while t < t0 + seconds - 1e-9:
        for frame in cycle(t, **kw):
            s = imu.feed(*frame)
            if s is not None:
                out.append(s)
        t += period
    return out, t


class ManualExamples(unittest.TestCase):
    def test_data_frames_decode_like_the_manual(self):
        a = hc.decode_pdo(H('2615DAEA28231900'))
        self.assertEqual(a.kind, hc.ANGLE)
        self.assertAlmostEqual(a.x, 54.14); self.assertAlmostEqual(a.y, -54.14); self.assertAlmostEqual(a.z, 90.00)
        self.assertEqual(a.temp_c, 25)
        self.assertTrue(a.self_test_ok); self.assertEqual(a.mount, 0)
        g = hc.decode_pdo(H('2615DAEA70170001'))
        self.assertEqual(g.kind, hc.ACCEL)
        self.assertAlmostEqual(g.x, 5.414); self.assertAlmostEqual(g.y, -5.414); self.assertAlmostEqual(g.z, 6.000)
        w = hc.decode_pdo(H('2615DAEA28230002'))
        self.assertEqual(w.kind, hc.GYRO)
        self.assertAlmostEqual(w.x, 54.14); self.assertAlmostEqual(w.y, -54.14); self.assertAlmostEqual(w.z, 90.00)
        self.assertIsNone(hc.decode_pdo(H('2615DAEA28230003')))     # 帧类型 11 手册里没有
        self.assertIsNone(hc.decode_pdo(H('2615DAEA2823')))

    def test_flag_bits(self):
        f = hc.decode_pdo(hc.encode_pdo(hc.GYRO, 1.0, 2.0, 3.0, self_test=2, mount=2))
        self.assertEqual((f.self_test, f.mount), (2, 2))
        self.assertFalse(f.self_test_ok)
        self.assertEqual(hc.encode_pdo(hc.ANGLE, 54.14, -54.14, 90.0, 25), H('2615DAEA28231900'))
        self.assertEqual(hc.encode_pdo(hc.ACCEL, 5.414, -5.414, 6.0), H('2615DAEA70170001'))

    def test_setting_requests_match_the_manual(self):
        self.assertEqual(hc.set_node(3), H('4010100003000000'))
        self.assertEqual(hc.set_baud(500), H('4020100001000000'))
        self.assertEqual(hc.set_baud(125), H('4020100003000000'))
        self.assertEqual(hc.set_period(10), H('220022000A000000'))
        self.assertEqual(hc.set_period(100), H('2200220064000000'))
        self.assertEqual(hc.set_period(1000), H('22002200E8030000'))
        self.assertEqual(hc.set_mask(), H('4030100007000000'))
        self.assertEqual(hc.set_mask(True, False, True), H('4030100005000000'))
        self.assertEqual(hc.zero_yaw(), H('4010100010101010'))
        self.assertEqual(hc.set_zero(True), H('4031100001000000'))
        self.assertEqual(hc.reset(), H('4010100088888888'))
        for bad in (lambda: hc.set_period(4), lambda: hc.set_baud(800), lambda: hc.set_node(0), lambda: hc.set_mask(0, 0, 0)):
            self.assertRaises(ValueError, bad)

    def test_pdo_ids(self):
        for base in (0x180, 0x280, 0x380, 0x480, 0x580):
            self.assertEqual(hc.pdo_node(base + NODE), NODE)
        self.assertIsNone(hc.pdo_node(0x001))           # 欧艾迪驱动器（节点 1、2）
        self.assertIsNone(hc.pdo_node(0x605))           # 设置请求
        self.assertIsNone(hc.pdo_node(0x585, pdo_id=0x185))
        self.assertEqual(hc.pdo_node(0x185, pdo_id=0x185), NODE)


class RosConventions(unittest.TestCase):
    """NED（x 车头、y 右、z 下）-> REP-103（x 车头、y 左、z 上），并与 line_node 给 app 的姿态解读一致。"""

    def test_left_turn_is_positive_yaw_rate(self):
        # 从上往下看逆时针（左转）绕朝下的 z 轴是负转
        out = hc.to_ros(gyro=hc.Frame(hc.GYRO, 0.0, 0.0, -30.0))
        self.assertAlmostEqual(out['angular_velocity'][2], math.radians(30.0))
        out = hc.to_ros(gyro=hc.Frame(hc.GYRO, 2.0, 3.0, 0.0))
        self.assertAlmostEqual(out['angular_velocity'][0], math.radians(2.0))
        self.assertAlmostEqual(out['angular_velocity'][1], -math.radians(3.0))

    def test_level_and_still_reads_plus_g(self):
        out = hc.to_ros(accel=hc.Frame(hc.ACCEL, 0.0, 0.0, -1.0))
        self.assertAlmostEqual(out['linear_acceleration'][2], hc.G)
        out = hc.to_ros(accel=hc.Frame(hc.ACCEL, 0.0, 0.0, 1.0), invert_accel=True)
        self.assertAlmostEqual(out['linear_acceleration'][2], hc.G)

    def test_attitude_as_the_app_shows_it(self):
        # NED：横滚为正 = 右侧压低（左侧抬起），俯仰为正 = 车头抬起，航向为正 = 右转
        q = hc.to_ros(angle=hc.Frame(hc.ANGLE, 3.0, 5.0, 30.0))['orientation']
        roll, pitch, yaw = attitude_deg(*q)
        self.assertAlmostEqual(roll, 3.0, places=6)       # app：左侧高
        self.assertAlmostEqual(pitch, -5.0, places=6)     # app：车头高
        self.assertAlmostEqual(yaw, -30.0, places=6)      # 左转为正：右转 30°
        q = hc.to_ros(angle=hc.Frame(hc.ANGLE, -20.0, -40.0, 170.0))['orientation']
        self.assertEqual([round(v, 6) for v in attitude_deg(*q)], [-20.0, 40.0, -170.0])


class Mounting(unittest.TestCase):
    """外壳 X 箭头没朝车头（本车 2026-09-29：箭头朝车左，mount_yaw_deg = 90）：换回车体系。"""

    def test_arrow_left_swaps_roll_and_pitch(self):
        # 垫高车头：传感器（箭头朝车左）的右侧抬起 = NED 横滚 −5°；车应读成俯仰 −5°（车头高）、横滚 0
        q = hc.to_ros(angle=hc.Frame(hc.ANGLE, -5.0, 0.0, 0.0), mount_yaw_deg=90)['orientation']
        roll, pitch, yaw = attitude_deg(*q)
        self.assertAlmostEqual(roll, 0.0, places=6); self.assertAlmostEqual(pitch, -5.0, places=6)
        self.assertAlmostEqual(yaw, 0.0, places=6)              # 航向仍是上电为 0 的相对值
        # 垫高车左侧：箭头那头抬起 = NED 俯仰 +4°；车应读成横滚 +4°（左侧高）
        roll, pitch, _ = attitude_deg(*hc.to_ros(angle=hc.Frame(hc.ANGLE, 0.0, 4.0, 0.0), mount_yaw_deg=90)['orientation'])
        self.assertAlmostEqual(roll, 4.0, places=6); self.assertAlmostEqual(pitch, 0.0, places=6)
        # 箭头朝车右：符号反过来
        _, pitch, _ = attitude_deg(*hc.to_ros(angle=hc.Frame(hc.ANGLE, -5.0, 0.0, 0.0), mount_yaw_deg=-90)['orientation'])
        self.assertAlmostEqual(pitch, 5.0, places=6)

    def test_large_tilt_and_yaw_are_exact(self):
        # 大角度也按旋转矩阵精确换算：先按车的姿态造出传感器读数，再换回来
        for mount in (90, -90, 180):
            for roll, pitch, yaw in ((12.0, -7.0, 35.0), (-30.0, 20.0, -150.0)):
                r_car = hc._matrix(math.radians(roll), math.radians(pitch), math.radians(yaw) - math.radians(mount))
                m = math.radians(mount); c, s = math.cos(m), math.sin(m)
                r_s = [[row[0] * c + row[1] * s, -row[0] * s + row[1] * c, row[2]] for row in r_car]   # R_车·Rz(mount)
                sr, sp, sy = math.atan2(r_s[2][1], r_s[2][2]), math.asin(-r_s[2][0]), math.atan2(r_s[1][0], r_s[0][0])
                frame = hc.Frame(hc.ANGLE, math.degrees(sr), -math.degrees(sp), -math.degrees(sy))
                got = [math.degrees(v) for v in hc.to_ros(angle=frame, mount_yaw_deg=mount)['rpy']]
                self.assertEqual([round(v, 6) for v in got], [roll, pitch, yaw], (mount, roll, pitch, yaw))

    def test_rates_and_accel_rotate_with_the_mount(self):
        # 航向角速度不变；绕箭头（车左）的转动是车的俯仰角速度；箭头方向的加速度是车的侧向
        out = hc.to_ros(gyro=hc.Frame(hc.GYRO, 10.0, 0.0, -30.0), accel=hc.Frame(hc.ACCEL, 0.1, 0.0, -1.0), mount_yaw_deg=90)
        wx, wy, wz = out['angular_velocity']
        self.assertAlmostEqual(wz, math.radians(30.0)); self.assertAlmostEqual(wx, 0.0); self.assertAlmostEqual(wy, math.radians(10.0))
        ax, ay, az = out['linear_acceleration']
        self.assertAlmostEqual(ax, 0.0); self.assertAlmostEqual(ay, 0.1 * hc.G); self.assertAlmostEqual(az, hc.G)

    def test_receiver_and_config(self):
        imu = hi.Hda536t(NODE, mount_yaw_deg=90)
        samples, t = run(imu, 1.0, gyro=(0.0, 0.0, -45.8), roll=-5.0, pitch=0.0)
        st = imu.status(t)
        self.assertAlmostEqual(st['pitch_deg'], -5.0); self.assertAlmostEqual(st['roll_deg'], 0.0)
        self.assertAlmostEqual(samples[-1]['angular_velocity'][2], math.radians(45.8), places=4)
        self.assertRaises(ValueError, hi.Hda536t, NODE, mount_yaw_deg=float('nan'))
        with open(os.path.join(ROOT, 'rion_protocol', 'hda536t.yaml'), encoding='utf-8') as fh:
            self.assertIn('mount_yaw_deg: 90.0', fh.read())      # 本车实测：箭头朝车左


class Receiver(unittest.TestCase):
    def test_gen5_setup_gives_50hz_imu(self):
        imu = hi.Hda536t(NODE)
        samples, t = run(imu, 2.0, gyro=(0.0, 0.0, -45.8), roll=1.0, pitch=-2.0)
        self.assertEqual(len(samples), 100)                 # 每个陀螺帧一组
        s = samples[-1]
        self.assertAlmostEqual(s['angular_velocity'][2], math.radians(45.8), places=4)
        self.assertAlmostEqual(s['linear_acceleration'][2], hc.G, places=3)
        self.assertEqual(s['temp_c'], 25)
        st = imu.status(t)
        self.assertIsNone(st['problem'])
        self.assertAlmostEqual(st['rates_hz']['陀螺'], 50.0, delta=0.5)
        self.assertEqual((st['self_test'], st['mount']), ('ok', '水平安装'))
        self.assertAlmostEqual(st['roll_deg'], 1.0); self.assertAlmostEqual(st['pitch_deg'], 2.0)

    def test_factory_setting_has_no_gyro(self):
        # 出厂：100 ms 只发角度帧 -> 没有 /imu，状态提示用设置脚本打开
        imu = hi.Hda536t(NODE)
        samples, t = run(imu, 2.0, period=0.1, kinds=(hc.ANGLE,))
        self.assertEqual(samples, [])
        self.assertIn('没有陀螺帧', imu.status(t)['problem'])

    def test_slow_gyro_is_published_but_flagged(self):
        imu = hi.Hda536t(NODE)
        samples, t = run(imu, 2.0, period=0.1)
        self.assertEqual(len(samples), 20)
        self.assertIn('10 Hz', imu.status(t)['problem'])

    def test_self_test_failure_and_wrong_mount_block_imu(self):
        imu = hi.Hda536t(NODE)
        samples, t = run(imu, 0.5, self_test=2)
        self.assertEqual(samples, [])
        self.assertEqual(imu.status(t)['problem'], '传感器自检失败')
        imu = hi.Hda536t(NODE)
        samples, t = run(imu, 0.5, mount=2)
        self.assertEqual(samples, [])
        self.assertIn('垂直向左', imu.status(t)['problem'])

    def test_missing_accel_frame_and_other_nodes(self):
        imu = hi.Hda536t(NODE)
        samples, _ = run(imu, 0.2, kinds=(hc.ANGLE, hc.GYRO))
        self.assertTrue(samples)
        self.assertNotIn('linear_acceleration', samples[-1])
        self.assertIn('orientation', samples[-1])
        other = hi.Hda536t(6)
        samples, t = run(other, 0.5)
        self.assertEqual(samples, [])
        self.assertIn('没有收到 IMU 数据帧', other.status(t)['problem'])

    def test_silence_is_reported(self):
        imu = hi.Hda536t(NODE)
        _, t = run(imu, 1.0)
        self.assertIsNone(imu.status(t)['problem'])
        self.assertIn('IMU 数据中断', imu.status(t + 1.0)['problem'])


class FakeBus:
    def __init__(self, frames):
        self.frames = list(frames)
        self.sent = []

    def recv(self):
        return self.frames.pop(0)[1:] if self.frames else None

    def send(self, can_id, data):
        self.sent.append((can_id, bytes(data)))

    def close(self):
        pass


class NodeSmoke(unittest.TestCase):
    """ROS 替身上跑一遍节点（不是集成测试）：读三帧发一组 /imu（不带姿态）和 /imu/data（带姿态），从不往总线发。"""

    def test_publishes_raw_and_data(self):
        frames = cycle(0.0, roll=2.0, pitch=4.0, gyro=(0.0, 0.0, -10.0))
        bus = FakeBus(frames)
        ticks = iter(t for t, _, _ in frames)
        node = hi.Hda536tNode(bus=bus, clock=lambda: next(ticks, 1.0))
        while node.poll():
            pass
        raw, data = node.publishers_['/imu'].sent, node.publishers_['/imu/data'].sent
        self.assertEqual((len(raw), len(data)), (1, 1))
        self.assertEqual(raw[0].header.frame_id, 'imu_link')
        self.assertAlmostEqual(raw[0].angular_velocity.z, math.radians(10.0), places=4)
        self.assertEqual(raw[0].orientation_covariance[0], -1.0)       # 融合用的原始 /imu 不带姿态
        self.assertAlmostEqual(raw[0].linear_acceleration.z, hc.G, places=3)
        o = data[0].orientation
        roll, pitch, _ = attitude_deg(o.x, o.y, o.z, o.w)
        self.assertAlmostEqual(roll, 2.0, places=4); self.assertAlmostEqual(pitch, -4.0, places=4)
        self.assertGreater(data[0].orientation_covariance[0], 0)
        self.assertEqual(bus.sent, [])
        node.publish_status()
        st = json.loads(node.publishers_['/imu/status'].sent[-1].data)
        self.assertEqual(st['node_id'], NODE)




class ConcurrentStatus(unittest.TestCase):
    def test_can_feed_and_status_share_a_consistent_snapshot(self):
        import threading
        import time
        imu = hi.Hda536t()
        errors, reads = [], []
        stop = threading.Event()
        def writer():
            try:
                for i in range(6000):
                    for frame in cycle(time.monotonic()):
                        imu.feed(*frame)
            except Exception as exc:
                errors.append(exc)
            finally:
                stop.set()
        old = sys.getswitchinterval()
        sys.setswitchinterval(0.00001)
        t = threading.Thread(target=writer)
        t.start()
        try:
            while not stop.is_set():
                status = imu.status(time.monotonic())
                self.assertTrue(all(math.isfinite(v) for v in status['rates_hz'].values()))
                reads.append(status)
        finally:
            t.join(timeout=10)
            sys.setswitchinterval(old)
        self.assertFalse(t.is_alive())
        self.assertFalse(errors, errors)
        self.assertGreater(len(reads), 10)

if __name__ == '__main__':
    unittest.main()
