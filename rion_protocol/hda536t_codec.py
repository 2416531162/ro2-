#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""瑞芬 HDA536T 动态倾角传感器（CAN 输出，C2 = CAN2.0A 标准帧 / C3 = CAN2.0B 扩展帧）的帧编解码。纯计算，不依赖 ROS。

依据厂家《HDA536T-CAN2.0A/B 动态倾角传感器技术手册》V1.4（2024-9-9）：
- 节点号 NODE_ID 默认 5；设置请求发 0x600+NODE_ID，应答从 0x580+NODE_ID 回，内容与请求相同。
- 数据帧（PDO）从 0x580+NODE_ID 发（出厂也可定成 0x180 / 0x280 / 0x380 / 0x480 + NODE_ID），8 字节：
  三个 int16 小端（X、Y、Z），第 7 字节温度（角度帧，有符号）或保留，第 8 字节标志：
  b1b0 帧类型（00 角度 /100 °、01 加速度 /1000 g、10 陀螺 /100 °/s），b3b2 自检（00 成功、10 失败），
  b7..b4 安装方式（0000 水平、0001 垂直向上、0010 垂直向左、0011 垂直向右、0100 水平向下）。
- 出厂：波特率 125K、输出周期 100 ms、只输出角度帧。改波特率、周期、输出哪几帧都要重新上电才生效。

传感器用 NED 车体系（x 车头、y 右、z 下），角度按 Z-Y-X 顺序（航向、俯仰、横滚），航向上电为 0（没有磁力计，
是相对值）。ROS REP-103 车体系是 x 车头、y 左、z 上：y、z 轴取反（见 to_ros）。
"""
import math
import struct
from dataclasses import dataclass

ANGLE, ACCEL, GYRO = 0, 1, 2
KIND_NAMES = {ANGLE: '角度', ACCEL: '加速度', GYRO: '陀螺'}
SCALES = {ANGLE: 0.01, ACCEL: 0.001, GYRO: 0.01}       # °、g、°/s
MOUNTS = {0: '水平安装', 1: '垂直向上', 2: '垂直向左', 3: '垂直向右', 4: '水平向下'}
PDO_BASES = (0x180, 0x280, 0x380, 0x480, 0x580)
SDO_REQUEST, SDO_REPLY = 0x600, 0x580
BAUDS = {1000: 0x00, 500: 0x01, 250: 0x02, 125: 0x03, 100: 0x04}   # kbps -> 手册里的编码
G = 9.80665


@dataclass(frozen=True)
class Frame:
    """一帧数据：kind 角度 / 加速度 / 陀螺；x, y, z 已换成 °、g、°/s（传感器 NED 轴）；temp_c 只有角度帧有。"""
    kind: int
    x: float
    y: float
    z: float
    temp_c: int = None
    self_test: int = 0      # b3b2：0 成功，2 失败，其余手册里没有
    mount: int = 0          # b7..b4

    @property
    def self_test_ok(self):
        return self.self_test == 0


def decode_pdo(data):
    """8 字节数据帧 -> Frame；长度不对、帧类型是 11（手册里没有）时返回 None。"""
    data = bytes(data)
    if len(data) != 8:
        return None
    flag = data[7]
    kind = flag & 0x03
    if kind not in SCALES:
        return None
    x, y, z = struct.unpack_from('<hhh', data)
    s = SCALES[kind]
    temp = struct.unpack_from('<b', data, 6)[0] if kind == ANGLE else None
    return Frame(kind, x * s, y * s, z * s, temp, (flag >> 2) & 0x03, flag >> 4)


def encode_pdo(kind, x, y, z, temp_c=0, self_test=0, mount=0):
    """Frame 的逆运算（模拟传感器、测试用）。"""
    s = SCALES[kind]

    def i16(v):
        return max(-32768, min(32767, int(round(v / s))))
    tail = struct.pack('<b', int(temp_c)) if kind == ANGLE else b'\0'
    return struct.pack('<hhh', i16(x), i16(y), i16(z)) + tail + bytes([(mount << 4) | (self_test << 2) | kind])


def pdo_node(can_id, pdo_id=0):
    """数据帧 ID -> 节点号；pdo_id 不为 0 时只认这一个 ID。不是数据帧返回 None。"""
    if pdo_id:
        return can_id & 0x7F if can_id == pdo_id else None
    return can_id & 0x7F if (can_id & ~0x7F) in PDO_BASES and can_id & 0x7F else None


# ---- 设置指令（发到 0x600+NODE_ID，应答与请求逐字节相同）----

def set_node(new_id):
    if not 1 <= new_id <= 0x7F:
        raise ValueError('节点号范围 1–127')
    return bytes([0x40, 0x10, 0x10, 0x00, new_id, 0, 0, 0])


def set_baud(kbps):
    if kbps not in BAUDS:
        raise ValueError('波特率只能是 ' + '/'.join('%dK' % k for k in sorted(BAUDS)))
    return bytes([0x40, 0x20, 0x10, 0x00, BAUDS[kbps], 0, 0, 0])


def set_period(ms):
    if not 5 <= ms <= 1000:
        raise ValueError('输出周期范围 5–1000 ms')
    return bytes([0x22, 0x00, 0x22, 0x00, ms & 0xFF, ms >> 8, 0, 0])


def set_mask(angle=True, accel=True, gyro=True):
    mask = (1 if angle else 0) | (2 if accel else 0) | (4 if gyro else 0)
    if not mask:
        raise ValueError('至少要输出一种数据帧')
    return bytes([0x40, 0x30, 0x10, 0x00, mask, 0, 0, 0])


def zero_yaw():
    return bytes([0x40, 0x10, 0x10, 0x00, 0x10, 0x10, 0x10, 0x10])


def set_zero(relative):
    """横滚俯仰零点：False 绝对（相对地理水平面，出厂），True 相对当前安装面。"""
    return bytes([0x40, 0x31, 0x10, 0x00, 1 if relative else 0, 0, 0, 0])


def reset():
    return bytes([0x40, 0x10, 0x10, 0x00, 0x88, 0x88, 0x88, 0x88])


# ---- 换到 ROS REP-103 车体系（x 车头、y 左、z 上）----

def quaternion(roll, pitch, yaw):
    """Z-Y-X 欧拉角（弧度）-> 四元数 (x, y, z, w)。"""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def _matrix(roll, pitch, yaw):
    """Z-Y-X 欧拉角（弧度）-> 旋转矩阵 Rz(航向)·Ry(俯仰)·Rx(横滚)，按行。"""
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def _mounted_rpy(rpy, mount_yaw):
    """传感器自身 REP-103 轴的姿态 -> 车的姿态。传感器 X 箭头在车体系里朝 mount_yaw（逆时针为正）：
    R_车 = R_传感器·Rz(−mount_yaw)，再按 Z-Y-X 取欧拉角；航向加回 mount_yaw，仍是上电为 0 的相对值。"""
    r = _matrix(*rpy)
    c, s = math.cos(mount_yaw), math.sin(mount_yaw)
    m = [[row[0] * c - row[1] * s, row[0] * s + row[1] * c, row[2]] for row in r]     # R · Rz(−mount_yaw)
    pitch = math.asin(max(-1.0, min(1.0, -m[2][0])))
    roll = math.atan2(m[2][1], m[2][2])
    yaw = math.atan2(m[1][0], m[0][0]) + mount_yaw
    return roll, pitch, math.atan2(math.sin(yaw), math.cos(yaw))


def to_ros(angle=None, accel=None, gyro=None, invert_accel=False, mount_yaw_deg=0.0):
    """传感器 NED 车体系的 Frame -> REP-103（x 车头、y 左、z 上）：
    angular_velocity 弧度/秒（左转 z 为正），linear_acceleration 米/秒²（静止水平时 z 约 +9.8），
    orientation 四元数（横滚左侧抬起为正、俯仰车头压低为正、偏航左转为正；偏航是上电时为 0 的相对值）。
    N = diag(1, −1, −1) 同时换车体系和参考系：R' = N R N，Z-Y-X 欧拉角变成 (横滚, −俯仰, −航向)。
    invert_accel：传感器报的是重力方向而不是比力（静止时 z 读成 −9.8）时整个取反，要上车核对。
    mount_yaw_deg：传感器水平装、但外壳 X 箭头没朝车头时，箭头在车上朝哪（从上往下看逆时针为正）：
    0 朝车头、90 朝车左、−90 朝车右、180 朝车尾。角速度、加速度绕竖轴转回车体系，姿态重算横滚俯仰
    （箭头朝车左时：车的俯仰 = 传感器横滚，车的横滚 = −传感器俯仰）。"""
    out = {}
    m = math.radians(mount_yaw_deg)
    c, s = math.cos(m), math.sin(m)

    def to_car(x, y, z):
        return (c * x - s * y, s * x + c * y, z)
    if gyro is not None:
        out['angular_velocity'] = to_car(math.radians(gyro.x), -math.radians(gyro.y), -math.radians(gyro.z))
    if accel is not None:
        k = -G if invert_accel else G
        out['linear_acceleration'] = to_car(accel.x * k, -accel.y * k, -accel.z * k)
    if angle is not None:
        rpy = (math.radians(angle.x), -math.radians(angle.y), -math.radians(angle.z))
        if m:
            rpy = _mounted_rpy(rpy, m)
        out['rpy'] = rpy
        out['orientation'] = quaternion(*rpy)
    return out
