#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""欧艾迪（OIDelec）智能 FOC 无刷驱动器 RS485 版的 Modbus RTU 编解码，纯计算，不依赖 ROS 或串口库。

依据厂家《智能FOC无刷电机驱动器（RS485 系列）编程手册 V1.3》（2024-04-10）：
- Modbus RTU：设备地址（站号）+ 功能码 + 数据 + CRC16（低字节在前）。16 位数据大端；32 位数据的字序在上位机里设，
  默认大端（高 16 位在前），手册的示例指令都按大端。
- 输入寄存器（功能码 04，只读）：5000 故障、5001–5002 转速 erpm（int32）、5003 占空比、5004 功率 W、5005 电压 V、
  5006 电机电流 10 mA、5007 总线电流 10 mA、5008 温度 ℃、5009 单圈角度 0.01°（uint16）、5010–5011 位置 0.01°（int32）。
- 保持寄存器（功能码 06 写单个、0x10 写多个）：6000 心跳（要在超时时间内改变它的值）、6001 控制模式
  （0 电流、1 转速……0xFFFF 空模式）、6002 电流 10 mA、6003–6004 转速 erpm（int32）、6016–6017 速度环加速度、
  6025–6026 速度环减速度（erpm/s，int32）。
- 故障码与 CAN 版相同（oid_codec.FAULTS）。没有专门的使能 / 停止指令：停止用电流 0。
"""
import struct

# ---- 寄存器 ----
REG_FAULT = 5000
REG_SPEED = 5001            # 5001–5002，int32
REG_VOLTAGE = 5005
REG_TEMPERATURE = 5008
REG_POSITION = 5010         # 5010–5011，int32
STATUS_COUNT = 12           # 5000–5011 一次读完

REG_HEARTBEAT = 6000
REG_MODE = 6001
REG_CURRENT = 6002
REG_TARGET_SPEED = 6003     # 6003–6004，int32
REG_SPEED_ACCEL = 6016      # 6016–6017，int32
REG_SPEED_DECEL = 6025      # 6025–6026，int32

MODE_CURRENT, MODE_SPEED, MODE_IDLE = 0, 1, 0xFFFF

FC_READ_HOLDING, FC_READ_INPUT, FC_WRITE_SINGLE, FC_WRITE_MULTIPLE = 0x03, 0x04, 0x06, 0x10
EXCEPTIONS = {1: '非法功能码', 2: '非法数据地址', 3: '非法数据值', 4: '从站设备故障'}


class ModbusError(Exception):
    pass


def crc16(data):
    """Modbus CRC16：预置 0xFFFF，多项式 0xA001（反射）。"""
    crc = 0xFFFF
    for b in bytes(data):
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def frame(address, pdu):
    """站号 + PDU + CRC（低字节在前）。"""
    if not 1 <= address <= 247:
        raise ValueError('Modbus 站号只能是 1–247')
    body = bytes([address]) + bytes(pdu)
    return body + struct.pack('<H', crc16(body))


def _u16(value):
    value = int(value)
    if not -32768 <= value <= 0xFFFF:
        raise ValueError('数值超出 16 位：%d' % value)
    return value & 0xFFFF


def i32_words(value, word_order='big'):
    """int32 → 两个 16 位寄存器；big：高 16 位在前（上位机默认），little：低 16 位在前。"""
    value = int(value)
    if not -2 ** 31 <= value < 2 ** 31:
        raise ValueError('数值超出 int32：%d' % value)
    hi, lo = struct.unpack('>HH', struct.pack('>i', value))
    return [hi, lo] if word_order == 'big' else [lo, hi]


def words_i32(words, word_order='big'):
    hi, lo = (words[0], words[1]) if word_order == 'big' else (words[1], words[0])
    return struct.unpack('>i', struct.pack('>HH', hi, lo))[0]


def read_input(address, start, count):
    if not 1 <= count <= 125:
        raise ValueError('一次最多读 125 个寄存器')
    return frame(address, struct.pack('>BHH', FC_READ_INPUT, start, count))


def read_holding(address, start, count):
    if not 1 <= count <= 125:
        raise ValueError('一次最多读 125 个寄存器')
    return frame(address, struct.pack('>BHH', FC_READ_HOLDING, start, count))


def write_single(address, register, value):
    return frame(address, struct.pack('>BHH', FC_WRITE_SINGLE, register, _u16(value)))


def write_multiple(address, register, values):
    values = [_u16(v) for v in values]
    if not 1 <= len(values) <= 123:
        raise ValueError('一次最多写 123 个寄存器')
    return frame(address, struct.pack('>BHHB', FC_WRITE_MULTIPLE, register, len(values), 2 * len(values))
                 + struct.pack('>%dH' % len(values), *values))


def write_i32(address, register, value, word_order='big'):
    return write_multiple(address, register, i32_words(value, word_order))


# ---- 驱动器指令（与手册示例指令逐字节一致）----

def heartbeat(address, counter):
    """心跳：每次写一个与上次不同的值（驱动器看的是「变没变」）。"""
    return write_single(address, REG_HEARTBEAT, counter & 0xFFFF)


def set_mode(address, mode):
    return write_single(address, REG_MODE, mode)


def set_current(address, centiamps):
    """电流 10 mA；电流模式下给 0 即放空电机（手册：失能、停止用电流 0）。"""
    return write_single(address, REG_CURRENT, centiamps)


def set_speed(address, erpm, word_order='big'):
    return write_i32(address, REG_TARGET_SPEED, erpm, word_order)


def set_speed_accel(address, erpm_s, word_order='big'):
    return write_i32(address, REG_SPEED_ACCEL, erpm_s, word_order)


def set_speed_decel(address, erpm_s, word_order='big'):
    return write_i32(address, REG_SPEED_DECEL, erpm_s, word_order)


def read_status(address):
    """5000–5011：故障、转速、占空比、功率、电压、两路电流、温度、角度、位置，一帧读完。"""
    return read_input(address, REG_FAULT, STATUS_COUNT)


# ---- 应答 ----

def reply_length(request):
    """请求对应的正常应答长度（含 CRC）。"""
    fc = request[1]
    if fc in (FC_READ_INPUT, FC_READ_HOLDING):
        return 5 + 2 * struct.unpack('>H', request[4:6])[0]
    return 8


def find_reply(buffer, request):
    """在收到的字节里找这条请求的应答（前面可能有杂字节）：返回完整的一帧（CRC 已核对），还不完整返回 None。
    异常应答（功能码 | 0x80）照样返回，由 parse_reply 报错。"""
    buffer, address, fc = bytes(buffer), request[0], request[1]
    for i in range(len(buffer) - 1):
        if buffer[i] != address or buffer[i + 1] not in (fc, fc | 0x80):
            continue
        if buffer[i + 1] & 0x80:
            n = 5
        elif fc in (FC_READ_INPUT, FC_READ_HOLDING):
            if len(buffer) < i + 3:
                return None
            n = 5 + buffer[i + 2]
        else:
            n = 8
        if len(buffer) < i + n:
            return None
        candidate = buffer[i:i + n]
        if crc16(candidate[:-2]) == struct.unpack('<H', candidate[-2:])[0]:
            return candidate
    return None


def parse_reply(request, reply):
    """核对应答：读寄存器返回寄存器值列表，写返回 True；CRC 错、站号 / 功能码对不上、异常应答抛 ModbusError。"""
    reply = bytes(reply)
    if len(reply) < 5 or crc16(reply[:-2]) != struct.unpack('<H', reply[-2:])[0]:
        raise ModbusError('应答 CRC 错')
    if reply[0] != request[0]:
        raise ModbusError('应答站号 %d 与请求 %d 不同' % (reply[0], request[0]))
    fc = request[1]
    if reply[1] == fc | 0x80:
        raise ModbusError('驱动器拒绝：%s' % EXCEPTIONS.get(reply[2], '异常码 %d' % reply[2]))
    if reply[1] != fc:
        raise ModbusError('应答功能码 0x%02X 与请求 0x%02X 不同' % (reply[1], fc))
    if fc in (FC_READ_INPUT, FC_READ_HOLDING):
        count = struct.unpack('>H', request[4:6])[0]
        if reply[2] != 2 * count or len(reply) != 5 + 2 * count:
            raise ModbusError('应答长度不对')
        return list(struct.unpack('>%dH' % count, reply[3:3 + 2 * count]))
    if reply[:6] != request[:6]:
        raise ModbusError('写应答与请求不一致')
    return True


def _s16(v):
    return v - 0x10000 if v & 0x8000 else v


def decode_status(regs, word_order='big'):
    """5000–5011 的 12 个寄存器 → dict（fault、erpm、duty、power_w、voltage、motor_ca、bus_ca、temp_c、angle_cdeg、
    position_cdeg；电流单位 10 mA）。"""
    if len(regs) != STATUS_COUNT:
        raise ValueError('状态要 %d 个寄存器' % STATUS_COUNT)
    return dict(fault=regs[0], erpm=words_i32(regs[1:3], word_order), duty=_s16(regs[3]), power_w=_s16(regs[4]),
                voltage=_s16(regs[5]), motor_ca=_s16(regs[6]), bus_ca=_s16(regs[7]), temp_c=_s16(regs[8]),
                angle_cdeg=regs[9], position_cdeg=words_i32(regs[10:12], word_order))
