#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""不接硬件模拟 Gen5：同一条 CAN 上两台欧艾迪单路驱动器（按厂家 CAN 编程手册的行为）+ 用它们开的差速车。

SimOidBus   两台驱动器的 CAN 总线替身（send / recv 与 SocketCan 相同）：心跳超时放空、速度环加减速斜坡、
            位置累加（0.01°）、查询回复、故障、断电。测试和演示服务器都用它。
SimModbusPort  485 总线上的驱动器替身（transact 与 SerialPort 相同），按 RS485 编程手册的寄存器：心跳要变值才算、
            先写目标再切转速模式、电流 0 放空、一帧读全状态、非法地址回异常码。
OidSimCar   follow_sim 的底盘接口（x, y, yaw, v, wz, steer, command, step, to_body, to_world）：
            下发的 (车速, 角速度) 经真正的 OidChassis / CanLink 编成 CAN 帧发给 SimOidBus，
            车按两台驱动器实际转出来的轮速走（前面万向轮随动，不影响运动）。
"""
import math
import struct

import oid_codec as oc
import oid_modbus as om
from oid_link import CanLink, OidChassis, WheelConfig


class SimMotor:
    """一台单路驱动器 + 它的电机：目标 erpm 按速度环加减速度爬坡，位置按转速累加（0.01°）；
    心跳超时就放空（目标 0）；powered=False 为断电（收不到指令、也不回复）。"""

    def __init__(self, pole_pairs, timeout_s=1.0):
        self.pole_pairs = pole_pairs
        self.target = self.speed = 0.0
        self.position = 0.0
        self.accel = self.decel = 5000.0
        self.fault = 0
        self.speed_mode = False         # 收到转速指令后进速度闭环；电流 0 为放空
        self.beat = None
        self.timeout_s = timeout_s
        self.powered = True

    def alive(self, t):
        return self.powered and self.beat is not None and t - self.beat <= self.timeout_s

    def step(self, t, dt):
        target = self.target if self.alive(t) and self.speed_mode and not self.fault else 0.0
        limit = (self.accel if abs(target) > abs(self.speed) else self.decel) * dt
        self.speed += max(-limit, min(limit, target - self.speed))
        self.position += self.speed / self.pole_pairs * 360.0 / 60.0 * 100.0 * dt     # erpm → 电机 rpm → 0.01°/s


class SimOidBus:
    """同一条 CAN 上的两台单路驱动器：左轮节点 ID left_id、右轮 right_id，回复用同一个 ID。"""

    def __init__(self, pole_pairs, left_id=1, right_id=2, timeout_s=1.0, voltage=48):
        self.ids = {left_id: 1, right_id: 2}
        self.motors = {1: SimMotor(pole_pairs, timeout_s), 2: SimMotor(pole_pairs, timeout_s)}
        self.t = 0.0
        self.voltage = voltage
        self.inbox, self.sent = [], []

    @property
    def alive(self):
        return all(m.alive(self.t) for m in self.motors.values())

    def advance(self, dt):
        for m in self.motors.values():
            m.step(self.t, dt)
        self.t += dt

    def send(self, can_id, data):
        self.sent.append((can_id, bytes(data)))
        if len(self.sent) > 1000:
            del self.sent[:500]
        motor = self.ids.get(can_id)
        if motor is None or not self.motors[motor].powered:
            return
        m, cmd = self.motors[motor], data[0]
        if cmd == oc.CAN_HEARTBEAT:
            m.beat = self.t
        elif cmd == oc.CAN_SPEED:
            m.target, m.speed_mode = struct.unpack('>i', data[1:5])[0], True
        elif cmd == oc.CAN_CURRENT:
            m.target, m.speed_mode = 0.0, False
        elif cmd == oc.CAN_SPEED_ACCEL:
            m.accel = struct.unpack('>i', data[1:5])[0]
        elif cmd == oc.CAN_SPEED_DECEL:
            m.decel = struct.unpack('>i', data[1:5])[0]
        elif cmd == oc.CAN_QUERY:
            item = data[1]
            value = {oc.Q_SPEED: ('>i', int(m.speed)), oc.Q_POSITION: ('>i', int(m.position)),
                     oc.Q_FAULT: ('>h', m.fault), oc.Q_VOLTAGE: ('>h', self.voltage)}.get(item)
            if value:
                self.inbox.append((can_id, bytes([0x0F, item]) + struct.pack(*value)))

    def recv(self):
        out, self.inbox = self.inbox, []
        return out

    def close(self):
        pass


class SimModbusPort:
    """一条 485 上的驱动器（{站号: 内部编号}），transact 与 SerialPort 相同：收一帧请求、按手册回一帧（断电不回）。
    word_order 模拟上位机设的 32 位字序。"""

    def __init__(self, pole_pairs, addresses=(1,), timeout_s=1.0, voltage=24, word_order='big', baud=115200):
        self.addresses = {a: i + 1 for i, a in enumerate(addresses)}
        self.motors = {a: SimMotor(pole_pairs, timeout_s) for a in addresses}
        self.hold = {a: {om.REG_HEARTBEAT: 0, om.REG_MODE: om.MODE_CURRENT, om.REG_CURRENT: 0} for a in addresses}
        self.t, self.voltage, self.word_order, self.baud = 0.0, voltage, word_order, baud
        self.requests = []

    def advance(self, dt):
        for m in self.motors.values():
            m.step(self.t, dt)
        self.t += dt

    def _i32(self, words):
        return om.words_i32(words, self.word_order)

    def transact(self, request, timeout_s):
        request = bytes(request)
        self.requests.append(request)
        if len(self.requests) > 2000:
            del self.requests[:1000]
        a, fc = request[0], request[1]
        m = self.motors.get(a)
        if m is None or not m.powered or om.crc16(request[:-2]) != struct.unpack('<H', request[-2:])[0]:
            return None
        h = self.hold[a]

        def error(code):
            return om.frame(a, bytes([fc | 0x80, code]))
        if fc == om.FC_READ_INPUT:
            start, count = struct.unpack('>HH', request[2:6])
            if start != om.REG_FAULT or count > om.STATUS_COUNT:
                return error(2)
            regs = ([m.fault] + om.i32_words(int(m.speed), self.word_order) + [0, 0, self.voltage, 0, 0, 30,
                    int(m.position) % 36000] + om.i32_words(int(m.position), self.word_order))[:count]
            return om.frame(a, bytes([fc, 2 * count]) + struct.pack('>%dH' % count, *[v & 0xFFFF for v in regs]))
        if fc == om.FC_READ_HOLDING:
            start, count = struct.unpack('>HH', request[2:6])
            if count != 1 or start not in h:
                return error(2)
            return om.frame(a, bytes([fc, 2]) + struct.pack('>H', h[start]))
        if fc == om.FC_WRITE_SINGLE:
            reg, value = struct.unpack('>HH', request[2:6])
            if reg == om.REG_HEARTBEAT:
                if value != h[reg]:                 # 心跳要变值才算
                    m.beat = self.t
            elif reg == om.REG_MODE:
                m.speed_mode = value == om.MODE_SPEED
                if value == om.MODE_CURRENT and h[om.REG_CURRENT] == 0:
                    m.target = 0.0                  # 电流 0：放空
            elif reg != om.REG_CURRENT:
                return error(2)
            h[reg] = value
            return request                          # 写单个寄存器的应答与请求相同
        if fc == om.FC_WRITE_MULTIPLE:
            reg, count = struct.unpack('>HH', request[2:6])
            words = list(struct.unpack('>%dH' % count, request[7:7 + 2 * count]))
            if count != 2 or reg not in (om.REG_TARGET_SPEED, om.REG_SPEED_ACCEL, om.REG_SPEED_DECEL):
                return error(2)
            value = self._i32(words)
            if reg == om.REG_TARGET_SPEED:
                m.target = value
            elif reg == om.REG_SPEED_ACCEL:
                m.accel = value
            else:
                m.decel = value
            return om.frame(a, request[1:6])
        return error(1)

    def close(self):
        pass


class OidSimCar:
    """用两台模拟驱动器开的 Gen5（follow_sim 底盘接口）。车体原点是后轴中点。

    wheels 默认：Gen5 CAD 轮距 0.675 m、轮半径 0.2046 m，磁极对数按 15 模拟，右轮电机镜像安装。
    control_s：驱动程序下发、查询的周期（与 oid.yaml 的 tx_hz 25 相同）。
    """

    def __init__(self, wheels=None, *, control_s=0.04, accel_mps2=1.0, decel_mps2=2.0):
        self.wheels = wheels or WheelConfig(pole_pairs=15, accel_mps2=accel_mps2, decel_mps2=decel_mps2)
        self.bus = SimOidBus(self.wheels.pole_pairs)
        self.chassis = OidChassis(CanLink(self.bus, (1, 2)), self.wheels)
        self.control_s = control_s
        self.x = self.y = self.yaw = 0.0
        self.v = self.wz = self.steer = 0.0
        self.cmd = (0.0, 0.0)
        self._next = 0.0
        self._opened = False

    def command(self, t, vx, wz):
        self.cmd = (float(vx), float(wz))

    def step(self, t, dt):
        if not self._opened:
            self.chassis.open(t)
            self._opened = True
        if t >= self._next - 1e-9:
            self._next = t + self.control_s
            self.chassis.step(t, *self.cmd)
        self.bus.advance(dt)
        w = self.wheels
        left, right = w.body({m: w.to_mps(s.speed) for m, s in self.bus.motors.items()})
        self.v, self.wz = 0.5 * (left + right), (right - left) / w.track_m
        self.x += self.v * dt * math.cos(self.yaw + self.wz * dt / 2)
        self.y += self.v * dt * math.sin(self.yaw + self.wz * dt / 2)
        self.yaw += self.wz * dt

    def to_body(self, wx, wy):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = wx - self.x, wy - self.y
        return c * dx + s * dy, -s * dx + c * dy

    def to_world(self, bx, by):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + c * bx - s * by, self.y + s * bx + c * by
