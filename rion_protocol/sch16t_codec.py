#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SCH16T 倾角 / 姿态模块（自研板，CAN 原生协议）的数据帧解码。纯计算，不依赖 ROS。

依据模块工程 docs/CAN协议.md（V1.0）和固件 App/can_protocol.c（2.3.0）：
- 数据帧出厂从 0x400 发（不加节点号；HDA536T 模式是 0x180/0x280/0x380/0x480/0x580 + 节点号），8 字节、小端：
  - 角度帧（flag b1b0 = 00）：横滚、俯仰、方位角 int16 0.01°，第 7 字节芯片温度 int8 °C；
  - 加速度帧（01）：X/Y/Z int16 0.001 g，本周期平均，已温补、标定；
  - 陀螺帧（10）：X/Y/Z int16 0.01 °/s，本周期平均，已减估计零偏；
  - 扩展帧（11），第 7 字节是子类型：0x01 高精度姿态（横坡、俯仰 int24 0.0001°）、0x02 状态（状态位 uint16、
    方位角 uint16 0.01°、横坡 1σ uint8 0.001°、帧计数 uint8）、0x03–0x06 原始数据（温度标定用，本驱动不用）。
- 第 8 字节 flag：b1b0 帧类型；b3b2 00 正常、01 未就绪（上电标定 / 中断后重新捕获 / 陀螺零偏清除）、10 故障；
  b7–b4 安装方式，目前只有 0000 水平安装。
- 每个周期按「角度 → 加速度 → 陀螺 → 高精度姿态 → 状态」的顺序发掩码打开的那几帧，出厂掩码 0x19
  （角度 + 高精度姿态 + 状态，**没有陀螺、加速度帧**），出厂周期 10 ms。

轴向（固件 README「轴向」「符号」）：
- 加速度帧、陀螺帧在模块机体系 X 朝前、Y 朝左、Z 朝上（= ROS REP-103），加速度是比力（水平静止 Z ≈ +1 g），
  陀螺绕 Z 逆时针（左转）为正，**不用像 HDA536T 那样把 y、z 取反**。
- 角度帧是和 HDA536T 一样的欧拉角：横滚左侧抬起为正、俯仰前端抬起为正、方位角从上往下看顺时针增大（上电
  标定完成时为 0，相对值）。换成 REP-103：(横滚, −俯仰, −方位角)，与 hda536t_codec.to_ros 的角度部分相同。
"""
import math
import struct
from dataclasses import dataclass

from hda536t_codec import G, _mounted_rpy, quaternion

ANGLE, ACCEL, GYRO, EXT = 0, 1, 2, 3
ATTITUDE, STATUS = 'attitude', 'status'      # 扩展帧 0x01、0x02 在接收端按这两个名字记
EXT_ATTITUDE, EXT_STATUS = 0x01, 0x02
EXT_RAW = (0x03, 0x04, 0x05, 0x06)
KIND_NAMES = {ANGLE: '角度', ACCEL: '加速度', GYRO: '陀螺', ATTITUDE: '高精度姿态', STATUS: '状态'}
SCALES = {ANGLE: 0.01, ACCEL: 0.001, GYRO: 0.01}       # °、g、°/s
DATA_ID = 0x400                                        # 出厂数据帧 ID（0x400 系列，2.3.0 build 7 起）
HDA_BASES = (0x180, 0x280, 0x380, 0x480, 0x580)        # HDA536T 模式：BASE + 节点号
TEST_OK, TEST_NOT_READY, TEST_FAULT = 0, 1, 2
TEST_NAMES = {TEST_OK: 'ok', TEST_NOT_READY: 'not_ready', TEST_FAULT: 'fault'}
MOUNTS = {0: '水平安装'}
# 状态帧状态位（协议 3.6 节）
STATUS_BITS = {
    0: '姿态有效', 1: '横坡有效', 2: '上电标定中', 3: '加速度被拒、仅陀螺推算', 4: '芯片温度变化中',
    5: '数据中断 / 重新捕获中', 6: '传感器故障', 7: '采样率或 CPU 超时故障', 8: '接近竖直',
    9: '横坡不确定度超限', 10: '航向因中断丢失过', 11: '陀螺零偏清除中', 12: '相对零点生效中', 13: '样机',
}
ST_VALID, ST_CALIBRATING, ST_GAP, ST_IMU_FAULT, ST_TIMING_FAULT = 1 << 0, 1 << 2, 1 << 5, 1 << 6, 1 << 7
ST_HEADING_RESET, ST_BIAS_CAPTURE, ST_PROTOTYPE = 1 << 10, 1 << 11, 1 << 13


@dataclass(frozen=True)
class Frame:
    """角度 / 加速度 / 陀螺帧：x, y, z 已换成 °、g、°/s（角度帧是横滚、俯仰、方位角）；temp_c 只有角度帧有。"""
    kind: int
    x: float
    y: float
    z: float
    temp_c: int = None
    test: int = TEST_OK     # flag b3b2
    mount: int = 0          # flag b7–b4


@dataclass(frozen=True)
class Attitude:
    """高精度姿态帧：横坡（刃口相对水平面的真实坡度，左端高为正）、俯仰（前端高为正），°。"""
    cross_deg: float
    pitch_deg: float
    test: int = TEST_OK
    mount: int = 0
    kind: str = ATTITUDE


@dataclass(frozen=True)
class Status:
    """状态帧：bits 状态位，heading_deg 方位角 0–359.99°，cross_sigma_deg 横坡 1σ（None = ≥ 0.255°），counter 帧计数。"""
    bits: int
    heading_deg: float
    cross_sigma_deg: float
    counter: int
    test: int = TEST_OK
    mount: int = 0
    kind: str = STATUS

    def has(self, bit):
        return bool(self.bits & bit)

    @property
    def flags(self):
        """置位的状态位，给人看。"""
        return [name for bit, name in STATUS_BITS.items() if self.bits >> bit & 1]


@dataclass(frozen=True)
class Raw:
    """原始数据帧（温度标定用）：只认出来，不解码。"""
    sub: int
    test: int = TEST_OK
    mount: int = 0
    kind: str = 'raw'


def _i24(data, offset):
    v = data[offset] | data[offset + 1] << 8 | data[offset + 2] << 16
    return v - (1 << 24) if v & 0x800000 else v


def decode(data):
    """8 字节数据帧 -> Frame / Attitude / Status / Raw；长度不对、扩展帧子类型不认识时返回 None。"""
    data = bytes(data)
    if len(data) != 8:
        return None
    flag = data[7]
    kind, test, mount = flag & 0x03, (flag >> 2) & 0x03, flag >> 4
    if kind in SCALES:
        x, y, z = struct.unpack_from('<hhh', data)
        s = SCALES[kind]
        temp = struct.unpack_from('<b', data, 6)[0] if kind == ANGLE else None
        return Frame(kind, x * s, y * s, z * s, temp, test, mount)
    sub = data[6]
    if sub == EXT_ATTITUDE:
        return Attitude(_i24(data, 0) * 1e-4, _i24(data, 3) * 1e-4, test, mount)
    if sub == EXT_STATUS:
        bits, heading = struct.unpack_from('<HH', data)
        return Status(bits, heading * 0.01, None if data[4] == 255 else data[4] * 1e-3, data[5], test, mount)
    if sub in EXT_RAW:
        return Raw(sub, test, mount)
    return None


def is_data_id(can_id, data_id=DATA_ID):
    """数据帧 ID：0x400 系列就是 data_id 本身；HDA536T 模式填 BASE + 节点号（如 0x585）。"""
    return can_id == data_id


# ---- 模拟模块、测试用的编码（decode 的逆运算）----

def _flag(kind, test, mount):
    return (mount << 4) | (test << 2) | kind


def encode(kind, x, y, z, temp_c=0, test=TEST_OK, mount=0):
    s = SCALES[kind]

    def i16(v):
        return max(-32767, min(32767, int(round(v / s))))
    tail = struct.pack('<b', int(temp_c)) if kind == ANGLE else b'\0'
    return struct.pack('<hhh', i16(x), i16(y), i16(z)) + tail + bytes([_flag(kind, test, mount)])


def encode_attitude(cross_deg, pitch_deg, test=TEST_OK, mount=0):
    def i24(v):
        return int(round(v * 1e4)) & 0xFFFFFF
    a, b = i24(cross_deg), i24(pitch_deg)
    return bytes([a & 0xFF, a >> 8 & 0xFF, a >> 16, b & 0xFF, b >> 8 & 0xFF, b >> 16,
                  EXT_ATTITUDE, _flag(EXT, test, mount)])


def encode_status(bits, heading_deg, cross_sigma_deg, counter, test=TEST_OK, mount=0):
    sigma = 255 if cross_sigma_deg is None else min(255, int(round(cross_sigma_deg * 1e3)))
    return struct.pack('<HHBB', bits, int(round(heading_deg * 100)) % 36000, sigma, counter & 0xFF) + \
        bytes([EXT_STATUS, _flag(EXT, test, mount)])


# ---- 换到 ROS REP-103 车体系（x 车头、y 左、z 上）----

def to_ros(angle=None, accel=None, gyro=None, mount_yaw_deg=0.0):
    """模块的帧 -> REP-103：angular_velocity 弧度/秒（左转 z 为正），linear_acceleration 米/秒²（水平静止 z ≈ +9.8），
    orientation 四元数 + rpy（横滚左侧抬起为正、俯仰车头压低为正、偏航左转为正；偏航是上电标定完成时为 0 的相对值）。
    mount_yaw_deg：模块水平装、但 +X 没朝车头时，+X 在车上朝哪（从上往下看逆时针为正）：0 朝车头、90 朝车左、
    −90 朝车右、180 朝车尾；角速度、加速度绕竖轴转回车体系，姿态按 hda536t_codec._mounted_rpy 重算横滚俯仰。"""
    out = {}
    m = math.radians(mount_yaw_deg)
    c, s = math.cos(m), math.sin(m)

    def to_car(x, y, z):
        return (c * x - s * y, s * x + c * y, z)
    if gyro is not None:
        out['angular_velocity'] = to_car(math.radians(gyro.x), math.radians(gyro.y), math.radians(gyro.z))
    if accel is not None:
        out['linear_acceleration'] = to_car(accel.x * G, accel.y * G, accel.z * G)
    if angle is not None:
        rpy = (math.radians(angle.x), -math.radians(angle.y), -math.radians(angle.z))
        if m:
            rpy = _mounted_rpy(rpy, m)
        out['rpy'] = rpy
        out['orientation'] = quaternion(*rpy)
    return out
