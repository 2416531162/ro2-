#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gen5 差速底盘 ↔ 两台欧艾迪单路驱动器（左右轮各一台）：轮速换算、链路、心跳、反馈和轮式里程计。

每台驱动器走 CAN（CanLink，CAN 编程手册）或 RS485（Rs485Link，Modbus RTU，RS485 编程手册）；两台走不同的总线时
用 SplitLink 拼起来（2026-09-29：右轮 CAN、左轮 485），对 OidChassis 来说都一样。

不依赖 ROS；总线对象从外面传进来（真车是 SocketCAN / 串口，测试用 oid_sim 里的模拟驱动器），
所以整套逻辑能在电脑上回归。ROS 节点见 oid_driver.py。

约定：车速 v 前进为正、角速度 w 左转为正；左轮速度 = v − w·轮距/2，右轮 = v + w·轮距/2。
内部编号 1 = 左轮驱动器、2 = 右轮驱动器（哪一台是左由 CAN ID 决定）。电机正转方向取决于装在哪一侧
（左右轮镜像安装，同一个正转一边往前、一边往后），由 left_sign / right_sign 配置。都要上车架空轮子核对。
"""
import math
import socket
import struct
import time
from dataclasses import dataclass, field

import oid_codec as oc
import oid_modbus as om


@dataclass(frozen=True)
class WheelConfig:
    """轮子与电机：wheel_radius_m 轮半径；pole_pairs 电机磁极对数（上位机识别电机参数后可看到）；
    gear_ratio 电机转几圈轮子转一圈（轮毂电机直驱为 1）；max_wheel_mps 单轮最大速度；
    accel_mps2 / decel_mps2 驱动器速度环自带斜坡（写进驱动器），按轮子线速度给。"""
    wheel_radius_m: float = 0.2046          # Gen5 CAD 后轮半径
    track_m: float = 0.675                  # Gen5 CAD 后轮轮距
    pole_pairs: int = 0                     # 0 = 还没填，不许出力
    gear_ratio: float = 1.0
    left_sign: int = 1
    right_sign: int = -1
    max_wheel_mps: float = 1.2
    accel_mps2: float = 1.0
    decel_mps2: float = 2.0

    def __post_init__(self):
        if not (0.02 <= self.wheel_radius_m <= 1.0 and 0.1 <= self.track_m <= 3.0):
            raise ValueError('轮半径或轮距不合理')
        if not (isinstance(self.pole_pairs, int) and 0 <= self.pole_pairs <= 100):
            raise ValueError('磁极对数必须是 0–100 的整数（0 为未填）')
        if not 0.01 <= self.gear_ratio <= 100:
            raise ValueError('减速比不合理')
        if self.left_sign not in (1, -1) or self.right_sign not in (1, -1):
            raise ValueError('方向只能是 ±1')
        if not (0 < self.max_wheel_mps <= 3.0 and 0 < self.accel_mps2 <= 10 and 0 < self.decel_mps2 <= 20):
            raise ValueError('轮速或加减速度不合理')

    @property
    def configured(self):
        return self.pole_pairs > 0

    def erpm_per_mps(self):
        """轮子 1 m/s 对应的电机电角度转速 erpm。"""
        return 60.0 / (2.0 * math.pi * self.wheel_radius_m) * self.gear_ratio * self.pole_pairs

    def to_erpm(self, wheel_mps):
        return int(round(wheel_mps * self.erpm_per_mps()))

    def to_mps(self, erpm):
        return erpm / self.erpm_per_mps()

    def centideg_to_m(self, centideg):
        """电机轴转过的角度（0.01°）→ 轮子走过的距离。"""
        return math.radians(centideg / 100.0) / self.gear_ratio * self.wheel_radius_m

    def wheels(self, v, w):
        """(车速, 角速度) → (左轮, 右轮) 线速度；超过单轮最大速度时两轮按同一比例缩（保持弯的半径）。"""
        half = 0.5 * self.track_m
        left, right = v - w * half, v + w * half
        scale = min(1.0, self.max_wheel_mps / max(abs(left), abs(right), 1e-12))
        return left * scale, right * scale

    def motor_erpm(self, v, w):
        """(车速, 角速度) → {1: 左驱动器 erpm, 2: 右驱动器 erpm}，已按方向换好。"""
        left, right = self.wheels(v, w)
        return {1: self.left_sign * self.to_erpm(left), 2: self.right_sign * self.to_erpm(right)}

    def body(self, motor_values):
        """{1: 左驱动器读数, 2: 右驱动器读数}（转速 erpm 或位置增量）→ (左轮, 右轮)，已换回车体方向。"""
        return self.left_sign * motor_values[1], self.right_sign * motor_values[2]


@dataclass
class MotorState:
    speed_erpm: int = None
    position_cdeg: int = None
    fault: int = None
    voltage: float = None
    t_speed: float = None
    t_position: float = None
    temp_c: int = None              # 485 一帧读全状态时顺带有
    current_a: float = None


def _motor_ids(ids, what):
    """(左, 右) 或 {1: 左, 2: 右} 里的一部分 → {内部编号: 总线地址}；地址不能重复。"""
    ids = dict(ids) if isinstance(ids, dict) else {1: ids[0], 2: ids[1]}
    if not ids or set(ids) - {1, 2}:
        raise ValueError('驱动器编号只能是 1（左）、2（右）')
    if len(set(ids.values())) != len(ids):
        raise ValueError('左右两台驱动器要设成不同的%s' % what)
    return ids


class SocketCan:
    """Linux SocketCAN 原始套接字（python 自带，不用 python-can）。接口要先起来：
    sudo ip link set can0 up type can bitrate 500000"""
    FMT = '=IB3x8s'

    def __init__(self, channel='can0'):
        if not hasattr(socket, 'AF_CAN'):
            raise RuntimeError('本机没有 SocketCAN（只能在 Linux 上用）')
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((channel,))
        self.sock.setblocking(False)
        self.channel = channel

    def send(self, can_id, data):
        data = bytes(data)
        self.sock.send(struct.pack(self.FMT, can_id & 0x7FF, len(data), data.ljust(8, b'\0')))

    def recv(self):
        out = []
        while True:
            try:
                raw = self.sock.recv(16)
            except BlockingIOError:
                return out
            can_id, dlc, data = struct.unpack(self.FMT, raw)
            if can_id & (socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG | socket.CAN_ERR_FLAG):
                continue
            out.append((can_id & 0x7FF, data[:dlc]))

    def close(self):
        self.sock.close()


class CanLink:
    """CAN 上的驱动器，各一个 CAN 节点 ID（上位机「CAN 节点 ID」，两台要设成不同的），单路驱动器用同一个 ID 回复；
    reply_ids 只在回复 ID 不同时给（双驱的「CAN 发送 ID」）。按 (左, 右) 给；只有一台在 CAN 上时给
    {1: 左} 或 {2: 右}。

    每个控制周期：每台发一帧目标转速，查询一次转速和位置；心跳按 heartbeat_s 发；故障、电压每 slow_s 查一次。
    驱动器在上位机设了主动回传时，回传帧格式与查询回复相同，照样收。
    """
    kind = 'can'

    def __init__(self, bus, node_ids, reply_ids=None, *, heartbeat_s=0.1, slow_s=0.5):
        self.bus, self.node_ids = bus, _motor_ids(node_ids, ' CAN 节点 ID')
        reply = _motor_ids(reply_ids, ' CAN 回复 ID') if reply_ids else self.node_ids
        self.reply_to_motor = {can_id: m for m, can_id in reply.items()}
        self.heartbeat_s, self.slow_s = heartbeat_s, slow_s
        self.motors = sorted(self.node_ids)
        self.state = {m: MotorState() for m in self.motors}
        self._beat = self._slow = -math.inf
        self.tx = self.rx = self.unknown = 0
        self.errors = 0

    def _send(self, motor, data):
        self.bus.send(self.node_ids[motor], data)
        self.tx += 1

    def configure(self, accel_erpm_s2, decel_erpm_s2):
        for m in self.motors:
            self._send(m, oc.can_speed_accel(accel_erpm_s2[m]))
            self._send(m, oc.can_speed_decel(decel_erpm_s2[m]))

    def heartbeat(self, now, force=False):
        if force or now - self._beat >= self.heartbeat_s:
            for m in self.motors:
                self._send(m, oc.can_heartbeat())
            self._beat = now

    def command(self, erpm):
        for m in self.motors:
            self._send(m, oc.can_speed(erpm[m]))

    def release(self):
        """放空电机（电流 0），手册里的「失能 / 停止」。"""
        for m in self.motors:
            self._send(m, oc.can_current(0))

    def request(self, now):
        for m in self.motors:
            self._send(m, oc.can_query(oc.Q_SPEED))
            self._send(m, oc.can_query(oc.Q_POSITION))
        if now - self._slow >= self.slow_s:
            for m in self.motors:
                self._send(m, oc.can_query(oc.Q_FAULT))
                self._send(m, oc.can_query(oc.Q_VOLTAGE))
            self._slow = now

    def close(self):
        self.bus.close()

    def poll(self, now):
        for can_id, data in self.bus.recv():
            motor = self.reply_to_motor.get(can_id)
            reply = oc.parse_can_reply(data) if motor else None
            if reply is None:
                self.unknown += 1
                continue
            self.rx += 1
            item, value = reply
            s = self.state[motor]
            if item == oc.Q_SPEED:
                s.speed_erpm, s.t_speed = value, now
            elif item == oc.Q_POSITION:
                s.position_cdeg, s.t_position = value, now
            elif item == oc.Q_FAULT:
                s.fault = value
            elif item == oc.Q_VOLTAGE:
                s.voltage = float(value)


class SerialPort:
    """RS485 串口（pyserial），一问一答。正点原子 RK3588 板载 485 是 UART3 = /dev/ttyS3（JP6 两个跳线帽插到下面），
    SP3485 的收发方向由板上电路自动切换，当普通串口用。echo=True：有些 USB 转 485 会把自己发出去的也收回来，
    先去掉这段回显再找应答。"""

    def __init__(self, port, baud=115200, parity='N', *, echo=False):
        import serial                       # 只有真车用到；测试用 oid_sim.SimModbusPort
        self.ser = serial.Serial(port, baud, bytesize=8, parity=parity, stopbits=1,
                                 timeout=0.002, write_timeout=0.05, exclusive=True)
        self.port, self.baud, self.echo = port, baud, echo

    def transact(self, request, timeout_s):
        """写入、排空和应答共用截止时间；写入失败抛错，让驱动停车并重连。"""
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('transaction timeout must be positive')
        deadline = time.monotonic() + timeout_s
        self.ser.reset_input_buffer()
        self.ser.write_timeout = min(0.05, timeout_s)
        if self.ser.write(request) != len(request):
            raise TimeoutError('RS485 incomplete write')
        # flush() uses unbounded tcdrain(). Poll the output queue instead; the
        # response deadline already includes request/reply wire time.
        while self.ser.out_waiting:
            if time.monotonic() >= deadline:
                raise TimeoutError('RS485 output queue did not drain')
            time.sleep(0.0005)
        if time.monotonic() >= deadline:
            raise TimeoutError('RS485 write deadline exceeded')
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if not chunk:
                continue
            buf += chunk
            body = bytes(buf)
            if self.echo:
                if len(body) < len(request):
                    continue
                if body.startswith(request):
                    body = body[len(request):]
            reply = om.find_reply(body, request)
            if reply is not None:
                return reply
        return None

    def close(self):
        self.ser.close()


class Rs485Link:
    """RS485（Modbus RTU）上的驱动器，一台一个站号（上位机「串口（485）参数」里的地址，同一条 485 上要不同）。
    按 (左, 右) 给；只有一台在 485 上时给 {1: 左} 或 {2: 右}。

    一问一答、同步收发：每个控制周期每台写一次目标转速、读一次全部状态（5000–5011，一帧）；心跳按 heartbeat_s 写
    （每次换个值，驱动器看的是变没变）；控制模式每 slow_s 读一次，不是转速模式时（刚上电、被放空过）先写目标再切
    转速模式（手册的顺序）。一台连续 SILENT_AFTER 次没应答就只每 RETRY_S 试一次，免得拖慢整个周期（另一台可能在 CAN
    上）；反馈过期由上层停车。
    """
    kind = 'rs485'
    SILENT_AFTER, RETRY_S = 3, 0.5

    def __init__(self, port, addresses, *, heartbeat_s=0.1, slow_s=0.5, word_order='big', latency_s=0.03):
        if word_order not in ('big', 'little'):
            raise ValueError('32 位字序只能是 big（上位机默认）或 little')
        self.bus, self.addresses = port, _motor_ids(addresses, ' 485 站号')
        if not all(1 <= a <= 247 for a in self.addresses.values()):
            raise ValueError('485 站号只能是 1–247')
        self.heartbeat_s, self.slow_s, self.word_order, self.latency_s = heartbeat_s, slow_s, word_order, latency_s
        self.motors = sorted(self.addresses)
        self.state = {m: MotorState() for m in self.motors}
        self.mode = {m: None for m in self.motors}
        self.fails = {m: 0 for m in self.motors}
        self._tried = {m: -math.inf for m in self.motors}
        self._counter = 0
        self._beat = self._slow = -math.inf
        self._now = -math.inf
        self.tx = self.rx = self.errors = 0
        self.last_error = ''

    def _timeout(self, request):
        """应答超时：驱动器处理时间 + 请求和应答在线上的时间（按 11 位一个字节）。"""
        return self.latency_s + (len(request) + om.reply_length(request)) * 11.0 / getattr(self.bus, 'baud', 115200)

    def _transact(self, m, request):
        if self.fails[m] >= self.SILENT_AFTER and self._now - self._tried[m] < self.RETRY_S:
            return None
        self._tried[m] = self._now
        self.tx += 1
        try:
            reply = self.bus.transact(request, self._timeout(request))
            if reply is None:
                raise om.ModbusError('没有应答')
            result = om.parse_reply(request, reply)
        except om.ModbusError as exc:
            self.errors += 1
            self.fails[m] += 1
            if self.fails[m] >= self.SILENT_AFTER:
                self.mode[m] = None         # 可能断过电：恢复后重新切模式
            self.last_error = '%s轮（站号 %d）：%s' % (('左', '右')[m - 1], self.addresses[m], exc)
            return None
        self.rx += 1
        self.fails[m] = 0
        return result

    def configure(self, accel_erpm_s2, decel_erpm_s2):
        for m in self.motors:
            a = self.addresses[m]
            self._transact(m, om.set_speed_accel(a, accel_erpm_s2[m], self.word_order))
            self._transact(m, om.set_speed_decel(a, decel_erpm_s2[m], self.word_order))

    def heartbeat(self, now, force=False):
        self._now = now
        if force or now - self._beat >= self.heartbeat_s:
            self._counter = self._counter % 0xFFFF + 1
            for m in self.motors:
                self._transact(m, om.heartbeat(self.addresses[m], self._counter))
            self._beat = now

    def command(self, erpm):
        for m in self.motors:
            a = self.addresses[m]
            if self._transact(m, om.set_speed(a, erpm[m], self.word_order)) and self.mode[m] != om.MODE_SPEED:
                if self._transact(m, om.set_mode(a, om.MODE_SPEED)):
                    self.mode[m] = om.MODE_SPEED

    def release(self):
        """放空电机：电流 0、电流模式（手册：失能和停止用电流 0）。"""
        for m in self.motors:
            a = self.addresses[m]
            self._transact(m, om.set_current(a, 0))
            if self._transact(m, om.set_mode(a, om.MODE_CURRENT)):
                self.mode[m] = om.MODE_CURRENT

    def request(self, now):
        self._now = now
        slow = now - self._slow >= self.slow_s
        for m in self.motors:
            a = self.addresses[m]
            regs = self._transact(m, om.read_status(a))
            if regs:
                st, s = om.decode_status(regs, self.word_order), self.state[m]
                s.speed_erpm, s.t_speed = st['erpm'], now
                s.position_cdeg, s.t_position = st['position_cdeg'], now
                s.fault, s.voltage = st['fault'], float(st['voltage'])
                s.temp_c, s.current_a = st['temp_c'], st['motor_ca'] / 100.0
            if slow:
                mode = self._transact(m, om.read_holding(a, om.REG_MODE, 1))
                if mode:
                    self.mode[m] = mode[0]
        if slow:
            self._slow = now

    def poll(self, now):
        """同步收发：应答在 request / command 里已经收完。"""

    def close(self):
        self.bus.close()


class SplitLink:
    """两台驱动器走不同的总线（例：左轮 485、右轮 CAN）：把两条链路拼成 OidChassis 要的一条（左右各由一条负责）。"""

    def __init__(self, links):
        self.links = list(links)
        owned = sorted(m for link in self.links for m in link.state)
        if owned != [1, 2]:
            raise ValueError('左右轮要各由一条链路负责')
        self.state = {m: s for link in self.links for m, s in link.state.items()}
        self.kind = '+'.join(link.kind for link in self.links)

    tx = property(lambda self: sum(link.tx for link in self.links))
    rx = property(lambda self: sum(link.rx for link in self.links))
    errors = property(lambda self: sum(getattr(link, 'errors', 0) for link in self.links))
    last_error = property(lambda self: '；'.join(e for e in (getattr(link, 'last_error', '') for link in self.links) if e))

    def configure(self, accel_erpm_s2, decel_erpm_s2):
        for link in self.links:
            link.configure(accel_erpm_s2, decel_erpm_s2)

    def heartbeat(self, now, force=False):
        for link in self.links:
            link.heartbeat(now, force)

    def command(self, erpm):
        for link in self.links:
            link.command(erpm)

    def release(self):
        for link in self.links:
            link.release()

    def request(self, now):
        for link in self.links:
            link.request(now)

    def poll(self, now):
        for link in self.links:
            link.poll(now)

    def close(self):
        for link in self.links:
            try:
                link.close()
            except Exception:
                pass


def _wrap_i32(delta):
    return (delta + 2 ** 31) % 2 ** 32 - 2 ** 31


@dataclass
class Odometry:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0
    w: float = 0.0
    left: float = 0.0
    right: float = 0.0
    last_positions: dict = field(default_factory=dict)


class OidChassis:
    """每个控制周期调一次 step(now, v, w)：发心跳、发两轮目标转速（send=False 时只收不发）、取反馈、
    更新里程计。两台驱动器是独立的盒子，可能只有一台断电或掉线：两轮的反馈都在 feedback_timeout_s 内才算
    更新（上层据此判反馈过期、停车）；任一台报故障 telemetry() 会说明，由上层停车。"""

    def __init__(self, link, wheels, *, feedback_timeout_s=0.3):
        self.link, self.wheels, self.feedback_timeout_s = link, wheels, feedback_timeout_s
        self.odom = Odometry()
        self.sent = {1: 0, 2: 0}
        self.updated = None             # 最近一次两路转速都更新过的时刻
        self._seen = (None, None)

    def open(self, now):
        w = self.wheels
        per = w.erpm_per_mps() if w.configured else 0.0
        accel = int(round(w.accel_mps2 * per))
        decel = int(round(w.decel_mps2 * per))
        self.link.configure({1: accel, 2: accel}, {1: decel, 2: decel})
        self.link.heartbeat(now, force=True)

    def step(self, now, v, w, *, send=True):
        if send:
            if not self.wheels.configured and (v or w):
                raise ValueError('pole_pairs_unconfigured')
            self.link.heartbeat(now)
            self.sent = self.wheels.motor_erpm(v, w) if self.wheels.configured else {1: 0, 2: 0}
            self.link.command(self.sent)
        self.link.request(now)
        self.link.poll(now)
        return self._update(now)

    def release(self, now):
        self.link.release()

    def silent(self, now):
        """反馈过期（或从没收到）的驱动器：{1: '左', 2: '右'} 里的那几台。"""
        return [m for m, s in self.link.state.items()
                if s.t_speed is None or now - s.t_speed > self.feedback_timeout_s]

    def _update(self, now):
        s1, s2 = self.link.state[1], self.link.state[2]
        seen = (s1.t_speed, s2.t_speed)
        if seen == self._seen or not self.wheels.configured or self.silent(now):
            return False
        self._seen = seen
        wl = self.wheels
        left, right = wl.body({1: wl.to_mps(s1.speed_erpm), 2: wl.to_mps(s2.speed_erpm)})
        o = self.odom
        o.left, o.right = left, right
        o.v, o.w = 0.5 * (left + right), (right - left) / wl.track_m
        # 位置增量积分（比速度 × 时间准）；第一次只记下起点
        pos = {1: s1.position_cdeg, 2: s2.position_cdeg}
        if None not in pos.values():
            if o.last_positions:
                d = {m: wl.centideg_to_m(_wrap_i32(pos[m] - o.last_positions[m])) for m in (1, 2)}
                dl, dr = wl.body(d)
                ds, dyaw = 0.5 * (dl + dr), (dr - dl) / wl.track_m
                mid = o.yaw + 0.5 * dyaw
                o.x += ds * math.cos(mid)
                o.y += ds * math.sin(mid)
                o.yaw = math.atan2(math.sin(o.yaw + dyaw), math.cos(o.yaw + dyaw))
            o.last_positions = dict(pos)
        self.updated = now
        return True

    def faults(self):
        return {m: s.fault for m, s in self.link.state.items() if s.fault}

    def telemetry(self, now):
        """与 wheeltec 驱动的遥测同一结构（控制策略、状态页用）：velocity=[v, 0, w]，voltage。"""
        volts = [s.voltage for s in self.link.state.values() if s.voltage is not None]
        o = self.odom
        return {'velocity': [o.v, 0.0, o.w], 'wheels_mps': [o.left, o.right], 'voltage': min(volts) if volts else float('nan'),
                'faults': {('左', '右')[m - 1]: oc.fault_text(c) for m, c in self.faults().items()},
                'erpm_sent': [self.sent[1], self.sent[2]],
                'erpm_read': [self.link.state[1].speed_erpm, self.link.state[2].speed_erpm],
                'silent': [('左', '右')[m - 1] for m in self.silent(now)],
                'stale': self.updated is None or now - self.updated > self.feedback_timeout_s}
