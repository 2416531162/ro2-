"""SCH16T 模块 CAN 原生协议：帧解码（按 docs/CAN协议.md 第 3 节逐字节）、REP-103 换算的正负号、接收逻辑。"""
import math
import os
import sys
import unittest
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'rion_protocol'))

import sch16t_codec as sc  # noqa: E402
from hda536t_imu import fill_imu  # noqa: E402
from sch16t_imu import Sch16t  # noqa: E402


def period(imu, t, angle=(0.0, 0.0, 0.0), acc=(0.0, 0.0, 1.0), gyro=(0.0, 0.0, 0.0), test=0, bits=sc.ST_VALID,
           counter=0, can_id=sc.DATA_ID, status=True):
    """一个输出周期：角度 → 加速度 → 陀螺 → 状态，每帧隔 0.2 ms；返回陀螺帧那一次 feed 的结果。"""
    imu.feed(t, can_id, sc.encode(sc.ANGLE, *angle, temp_c=31, test=test))
    imu.feed(t + 2e-4, can_id, sc.encode(sc.ACCEL, *acc, test=test))
    out = imu.feed(t + 4e-4, can_id, sc.encode(sc.GYRO, *gyro, test=test))
    if status:
        imu.feed(t + 6e-4, can_id, sc.encode_status(bits, 0.0, 0.004, counter, test=test))
    return out


class Decode(unittest.TestCase):
    def test_angle_accel_gyro_frames_little_endian_with_flag(self):
        # 横滚 −1.50°（−150 = 0xFF6A）、俯仰 +2.00°、方位角 −179.99°，温度 −5 °C，flag 00 正常、角度帧
        f = sc.decode(bytes([0x6A, 0xFF, 0xC8, 0x00, 0xB1, 0xB9, 0xFB, 0x00]))
        self.assertEqual((f.kind, f.temp_c, f.test, f.mount), (sc.ANGLE, -5, sc.TEST_OK, 0))
        self.assertAlmostEqual(f.x, -1.5)
        self.assertAlmostEqual(f.y, 2.0)
        self.assertAlmostEqual(f.z, -179.99)
        # 加速度帧 Z = 1.000 g，flag 0x05：未就绪（b3b2 = 01）+ 加速度帧（01）
        f = sc.decode(bytes([0, 0, 0, 0, 0xE8, 0x03, 0, 0x05]))
        self.assertEqual((f.kind, f.test, f.temp_c), (sc.ACCEL, sc.TEST_NOT_READY, None))
        self.assertAlmostEqual(f.z, 1.0)
        # 陀螺帧 X = −0.01 °/s，flag 0x0A：故障（10）+ 陀螺帧（10）
        f = sc.decode(bytes([0xFF, 0xFF, 0, 0, 0, 0, 0, 0x0A]))
        self.assertEqual((f.kind, f.test), (sc.GYRO, sc.TEST_FAULT))
        self.assertAlmostEqual(f.x, -0.01)

    def test_extended_attitude_status_and_raw(self):
        # 高精度姿态：横坡 −0.0001°（int24 0xFFFFFF）、俯仰 +83.8860°（838860）
        f = sc.decode(bytes([0xFF, 0xFF, 0xFF, 0xCC, 0xCC, 0x0C, 0x01, 0x03]))
        self.assertEqual(f.kind, sc.ATTITUDE)
        self.assertAlmostEqual(f.cross_deg, -1e-4)
        self.assertAlmostEqual(f.pitch_deg, 83.886)
        # 状态：位 0、1、13（0x2003），方位角 359.99°（35999 = 0x8C9F），横坡 1σ 0.012°，计数 0xFE
        f = sc.decode(bytes([0x03, 0x20, 0x9F, 0x8C, 0x0C, 0xFE, 0x02, 0x03]))
        self.assertEqual((f.kind, f.bits, f.counter), (sc.STATUS, 0x2003, 0xFE))
        self.assertAlmostEqual(f.heading_deg, 359.99)
        self.assertAlmostEqual(f.cross_sigma_deg, 0.012)
        self.assertEqual(f.flags, ['姿态有效', '横坡有效', '样机'])
        self.assertIsNone(sc.decode(bytes([0, 0, 0, 0, 255, 0, 0x02, 0x03])).cross_sigma_deg)   # 255 = ≥ 0.255°
        self.assertEqual(sc.decode(bytes([0] * 6 + [0x05, 0x03])).kind, 'raw')
        self.assertIsNone(sc.decode(bytes([0] * 6 + [0x07, 0x03])))     # 不认识的子类型
        self.assertIsNone(sc.decode(bytes(7)))

    def test_encoders_round_trip(self):
        f = sc.decode(sc.encode(sc.GYRO, 1.23, -4.56, 7.89, test=sc.TEST_NOT_READY))
        for got, want in zip((f.x, f.y, f.z), (1.23, -4.56, 7.89)):
            self.assertAlmostEqual(got, want)
        self.assertEqual(f.test, sc.TEST_NOT_READY)
        a = sc.decode(sc.encode_attitude(-12.3456, 0.0001))
        self.assertAlmostEqual(a.cross_deg, -12.3456)
        self.assertAlmostEqual(a.pitch_deg, 0.0001)
        s = sc.decode(sc.encode_status(0x0405, 12.34, None, 300))
        self.assertEqual((s.bits, s.counter, s.cross_sigma_deg), (0x0405, 300 & 0xFF, None))


class ToRos(unittest.TestCase):
    """模块机体系 X 前、Y 左、Z 上（= REP-103）；角度帧是 HDA536T 那样的欧拉角（俯仰前高为正、方位角顺时针为正）。"""

    def test_rates_and_accel_need_no_axis_flip(self):
        out = sc.to_ros(accel=sc.Frame(sc.ACCEL, 0.1, -0.2, 1.0), gyro=sc.Frame(sc.GYRO, 1.0, -2.0, 10.0))
        wx, wy, wz = out['angular_velocity']
        self.assertAlmostEqual(wz, math.radians(10.0))        # 左转（逆时针）为正
        self.assertAlmostEqual(wx, math.radians(1.0))
        self.assertAlmostEqual(wy, math.radians(-2.0))
        ax, ay, az = out['linear_acceleration']
        self.assertAlmostEqual(az, sc.G)                      # 水平静止 z ≈ +9.8
        self.assertAlmostEqual(ax, 0.1 * sc.G)
        self.assertAlmostEqual(ay, -0.2 * sc.G)

    def test_angles_become_rep103(self):
        roll, pitch, yaw = sc.to_ros(angle=sc.Frame(sc.ANGLE, 1.0, 2.0, 30.0))['rpy']
        self.assertAlmostEqual(math.degrees(roll), 1.0)       # 左侧抬起为正，两边一样
        self.assertAlmostEqual(math.degrees(pitch), -2.0)     # 模块前端抬起 = REP-103 俯仰为负
        self.assertAlmostEqual(math.degrees(yaw), -30.0)      # 顺时针 30° = 右转 = REP-103 偏航 −30°

    def test_mount_yaw_rotates_into_car_frame(self):
        # 模块 +X 朝车左（90°）：模块绕 X 的转动是车绕 Y 的转动
        out = sc.to_ros(gyro=sc.Frame(sc.GYRO, 5.0, 0.0, 0.0), mount_yaw_deg=90.0)
        wx, wy, wz = out['angular_velocity']
        self.assertAlmostEqual(wx, 0.0)
        self.assertAlmostEqual(wy, math.radians(5.0))


class Receiver(unittest.TestCase):
    def test_one_sample_per_period_with_angle_and_accel(self):
        imu = Sch16t()
        out = period(imu, 1.0, angle=(1.0, 2.0, -45.0), gyro=(0.0, 0.0, 10.0))
        self.assertAlmostEqual(out['angular_velocity'][2], math.radians(10.0))
        self.assertAlmostEqual(out['linear_acceleration'][2], sc.G)
        self.assertAlmostEqual(math.degrees(out['rpy'][2]), 45.0)
        self.assertEqual(out['temp_c'], 31)
        msg = fill_imu(NS(header=NS(), angular_velocity=NS(), linear_acceleration=NS(), orientation=NS(),
                          angular_velocity_covariance=[0.0] * 9, linear_acceleration_covariance=[0.0] * 9,
                          orientation_covariance=[0.0] * 9), out, 'stamp', 'imu_link', True)
        self.assertGreater(msg.orientation_covariance[0], 0)
        self.assertAlmostEqual(msg.angular_velocity.z, math.radians(10.0))

    def test_other_ids_ignored_and_hda536t_mode_id_configurable(self):
        imu = Sch16t()
        self.assertIsNone(period(imu, 1.0, can_id=0x585))
        self.assertIsNone(imu.newest)
        self.assertIn('0x400', imu.problem(2.0))
        imu = Sch16t(data_id=0x585)
        self.assertIsNotNone(period(imu, 1.0, can_id=0x585))

    def test_stale_or_not_ready_angle_is_not_attached(self):
        imu = Sch16t(stale_s=0.1)
        imu.feed(1.0, sc.DATA_ID, sc.encode(sc.ANGLE, 0.0, 0.0, 10.0))
        out = imu.feed(1.2, sc.DATA_ID, sc.encode(sc.GYRO, 0.0, 0.0, 1.0))
        self.assertNotIn('orientation', out)
        imu.feed(2.0, sc.DATA_ID, sc.encode(sc.ANGLE, 0.0, 0.0, 10.0, test=sc.TEST_NOT_READY))
        out = imu.feed(2.0005, sc.DATA_ID, sc.encode(sc.GYRO, 0.0, 0.0, 1.0))
        self.assertNotIn('orientation', out)

    def test_not_ready_and_fault_block_imu_with_reason_from_status_bits(self):
        imu = Sch16t()
        period(imu, 1.0, test=sc.TEST_NOT_READY, bits=sc.ST_CALIBRATING)
        self.assertIsNone(period(imu, 1.02, test=sc.TEST_NOT_READY, bits=sc.ST_CALIBRATING, counter=1))
        self.assertIn('上电标定中', imu.problem(1.03))
        self.assertEqual(imu.status(1.03)['self_test'], 'not_ready')
        self.assertIsNone(period(imu, 1.04, test=sc.TEST_FAULT, bits=sc.ST_IMU_FAULT, counter=2))
        self.assertIn('故障', imu.problem(1.05))
        # 不开状态帧时也按 flag 拦，只是说不到哪一位
        imu = Sch16t()
        self.assertIsNone(period(imu, 1.0, test=sc.TEST_NOT_READY, status=False))
        self.assertEqual(imu.problem(1.01), 'IMU 模块未就绪')

    def test_factory_mask_without_gyro_tells_how_to_enable_it(self):
        imu = Sch16t()
        imu.feed(1.0, sc.DATA_ID, sc.encode(sc.ANGLE, 0.0, 0.0, 0.0))
        imu.feed(1.0, sc.DATA_ID, sc.encode_attitude(0.0, 0.0))
        imu.feed(1.0, sc.DATA_ID, sc.encode_status(sc.ST_VALID, 0.0, 0.004, 0))
        p = imu.problem(1.01)
        self.assertIn('没有陀螺帧', p)
        self.assertIn('40 30 10 00 17', p)

    def test_healthy_50hz_status_rates_flags_and_lost_periods(self):
        imu = Sch16t()
        for i in range(60):
            if i == 30:
                continue                              # 丢了一个周期
            period(imu, 1.0 + i * 0.02, angle=(0.5, -0.5, 10.0), counter=i, bits=sc.ST_VALID | sc.ST_PROTOTYPE)
        now = 1.0 + 59 * 0.02 + 0.001
        st = imu.status(now)
        self.assertIsNone(st['problem'])
        self.assertAlmostEqual(st['rates_hz']['陀螺'], 50.0, delta=1.0)
        self.assertEqual(st['sequence_gaps'], 1)
        self.assertEqual(st['flags'], ['姿态有效', '样机'])
        self.assertEqual((st['roll_deg'], st['pitch_deg'], st['yaw_deg']), (0.5, 0.5, -10.0))
        self.assertEqual(st['temp_c'], 31)
        self.assertIn('中断', imu.problem(now + 1.0))

    def test_slow_gyro_is_reported(self):
        imu = Sch16t()
        for i in range(20):
            period(imu, 1.0 + i * 0.1, counter=i)
        self.assertIn('陀螺帧只有 10 Hz', imu.problem(2.95))

    def test_bad_frames_counted(self):
        imu = Sch16t()
        imu.feed(1.0, sc.DATA_ID, bytes(5))
        imu.feed(1.0, sc.DATA_ID, bytes([0] * 6 + [0x09, 0x03]))
        self.assertEqual(imu.status(1.0)['bad_frames'], 2)


if __name__ == '__main__':
    unittest.main()
