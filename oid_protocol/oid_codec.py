#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""欧艾迪（OIDelec）智能 FOC 无刷驱动器（单路 MINI-CAN）的 CAN 协议编解码，纯计算，不依赖 ROS 或 CAN 库。

依据厂家《智能FOC无刷电机驱动器（CAN 系列）编程手册 V1.2》（单路）；《双驱 CAN 系列》V2.0 的指令、查询与它
逐条相同，只多了双驱的共享 ID。

标准帧 11 位 ID，一台驱动器一个节点 ID，回复用同一个 ID
    控制指令无返回：DATA[0] 为指令，参数大端。0x00 心跳；0x02 转速（int32，erpm）；0x0A / 0x10 速度环
    加速度 / 减速度（int32，erpm/s²）；0x08 刹车电流（int16，10 mA）；0x01 电流（int16，10 mA）。
    查询：DATA = [0x0F, 项目]，回复 [0x0F, 项目, 值…]（值按回复长度：4 字节 int32、2 字节 int16）。
    驱动器也可以在上位机里设成按周期主动回传，格式与查询回复相同。

转速是电角度转速 erpm = 电机转速 rpm × 磁极对数。
"""
import struct

# ---- 故障码 ----
FAULTS = {0: '无错误', 1: '过压', 2: '欠压', 3: '绝对值超过最大电流', 4: 'MOS 管过温', 5: 'MCU 欠压',
          6: '看门狗触发后启动', 7: 'SPI 接口驱动器错误', 8: 'FLASH 损毁', 9: 'U 相电流传感器偏移过大',
          10: 'V 相电流传感器偏移过大', 11: 'W 相电流传感器偏移过大', 12: '三相电流不平衡',
          13: 'FLASH 中电机配置损毁', 14: 'FLASH 中应用配置损毁', 15: 'CANOPEN 心跳错误', 16: '堵转',
          17: '失速', 18: '超差', 19: '编码器未找到 Z 信号', 20: '电机过温'}


def fault_text(code):
    return FAULTS.get(code, '未知故障 %d' % code)


def _i16(value):
    value = int(value)
    if not -32768 <= value <= 32767:
        raise ValueError('数值超出 int16：%d' % value)
    return value


def _i32(value):
    value = int(value)
    if not -2 ** 31 <= value < 2 ** 31:
        raise ValueError('数值超出 int32：%d' % value)
    return value


CAN_HEARTBEAT = 0x00
CAN_CURRENT = 0x01
CAN_SPEED = 0x02
CAN_BRAKE = 0x08
CAN_SPEED_ACCEL = 0x0A
CAN_QUERY = 0x0F
CAN_SPEED_DECEL = 0x10
CAN_MAX_CURRENT = 0x14

Q_FAULT, Q_SPEED, Q_DUTY, Q_POWER, Q_VOLTAGE, Q_MOTOR_CURRENT, Q_BUS_CURRENT, Q_TEMPERATURE, Q_POSITION, \
    Q_ANGLE, Q_Z, Q_ERRORS, Q_IO = range(13)


def can_heartbeat():
    return bytes([CAN_HEARTBEAT])


def can_speed(erpm):
    return bytes([CAN_SPEED]) + struct.pack('>i', _i32(erpm))


def can_speed_accel(erpm_s2):
    return bytes([CAN_SPEED_ACCEL]) + struct.pack('>i', _i32(erpm_s2))


def can_speed_decel(erpm_s2):
    return bytes([CAN_SPEED_DECEL]) + struct.pack('>i', _i32(erpm_s2))


def can_current(centiamps):
    """电流控制，单位 10 mA；0 即放空电机（手册：失能和停止用电流 0）。"""
    return bytes([CAN_CURRENT]) + struct.pack('>h', _i16(centiamps))


def can_brake(centiamps):
    """电子刹车（再生制动），单位 10 mA，仅正值有效。"""
    if centiamps < 0:
        raise ValueError('刹车电流只能是正值')
    return bytes([CAN_BRAKE]) + struct.pack('>h', _i16(centiamps))


def can_query(item):
    if not 0 <= item <= 12:
        raise ValueError('查询项目只能是 0–12')
    return bytes([CAN_QUERY, item])


def parse_can_reply(data):
    """查询回复 / 主动回传 → (项目, 值)；不是回复返回 None。

    值：转速 erpm、位置 0.01° 为 int32；故障、占空比、功率、电压、电流、温度为 int16；单圈角度 uint16；
    Z 信号 0/1；历史错误为 5 个字节的元组；IO 为 (输入, 输出) 位图。
    """
    data = bytes(data)
    if len(data) < 3 or data[0] != CAN_QUERY:
        return None
    item, payload = data[1], data[2:]
    if item == Q_ERRORS:
        return item, tuple(payload[:5])
    if item == Q_IO:
        return (item, (payload[0], payload[1])) if len(payload) >= 2 else None
    if item == Q_Z:
        return item, payload[0]
    if item == Q_ANGLE and len(payload) == 2:
        return item, struct.unpack('>H', payload)[0]
    if len(payload) == 4:
        return item, struct.unpack('>i', payload)[0]
    if len(payload) == 2:
        return item, struct.unpack('>h', payload)[0]
    return None
