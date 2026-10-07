"""通过 HC-08 透传既有单片机指令，水泵压力对应右下通道最大输出百分比。"""
import time

HC08_ADDRESS = 'F8:2E:0C:E3:A0:A8'
SERIAL_UUID = '0000ffe1-0000-1000-8000-00805f9b34fb'
STOP = bytes.fromhex('BB C1 00 00 55')
RIGHT_DOWN = bytes.fromhex('BB C1 04 00 55')
MANUAL = bytes.fromhex('BB C2 00 00 55')
QUERY = bytes.fromhex('BB FF CC FF 55')
DIAGNOSTIC = bytes.fromhex('BB D0 01 00 55')
# 与德国控制器 Communication_BuildStatusFrames 对齐；按长度组帧，负载中的 AA/55 不当分隔符。
FRAME_LENGTHS = {
    **dict.fromkeys((0x01, 0x02, 0x04, 0x0D, 0x10, 0x14, 0x17, 0x18, 0x28), 7),
    **dict.fromkeys((0x05, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x26,
                     0xD0, 0xD2, 0xD3, 0x2D), 8),
    **dict.fromkeys((0x06, 0x07, 0x08, 0x09, 0x0B, 0x0C, 0x0F, 0x12, 0x13,
                     0x16, 0x20, 0x21, 0x22, 0x23, 0x25, 0x27, 0x29, 0x2A,
                     0x2C, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0xD1, 0xC5), 5),
    0x03: 6, 0x0E: 6,
}


class McuReports:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.buffer = bytearray()
        self.reports = {}
        self.sequence = 0

    def receive(self, data):
        self.buffer.extend(data)
        while len(self.buffer) >= 2:
            if self.buffer[0] != 0xAA or self.buffer[1] not in FRAME_LENGTHS:
                del self.buffer[0]
                continue
            length = FRAME_LENGTHS[self.buffer[1]]
            if len(self.buffer) < length:
                break
            if self.buffer[length - 1] != 0x55:
                del self.buffer[0]
                continue
            frame = bytes(self.buffer[:length])
            del self.buffer[:length]
            self.sequence += 1
            self.reports[frame[1]] = (self.sequence, self.clock(), frame)

    def get(self, command, after=0, max_age=2.0):
        report = self.reports.get(command)
        if report and report[0] > after and self.clock() - report[1] <= max_age:
            return report[2]
        return None


class Hc08Pump:
    """控制状态机：参数回读后才开输出；MCU 回报后才向平板确认。"""
    def __init__(self, transport, emit, clock=time.monotonic):
        self.transport, self.emit, self.clock = transport, emit, clock
        self.reports = McuReports(clock)
        self.connected = self.ready = self.wanted = self.active = False
        self.pressure = 50
        self.token = None
        self.deadline = self.confirm_deadline = 0.0
        self.next_action = self.next_query = self.next_diagnostic = 0.0
        self.parameter_after = self.output_after = 0
        self.stage = 'idle'
        self.off_reported = False
        self.retry_at = 0.0
        self.establish_deadline = 0.0

    def write(self, frame):
        self.transport.write(frame)

    def establish(self):
        self.reports = McuReports(self.clock)
        self.transport.open(self.reports.receive)
        self.connected = True
        self.establish_deadline = self.clock() + 5.0
        # 先停旧手动动作，再切手动；重连不会恢复之前的水泵会话。
        self.write(STOP)
        self.active = False
        self.write(MANUAL)
        self.write(QUERY)
        self.write(DIAGNOSTIC)
        self.next_query = self.next_diagnostic = self.clock() + 1.0

    def _off_reports(self, after=0):
        mode = self.reports.get(0x03, after)
        bits = self.reports.get(0xD0, after)
        pwm = self.reports.get(0x04, after)
        return bool(mode and mode[4] <= 3 and not (mode[4] & 1) and bits and bits[2] == 3 and
                    bits[3] == 0 and pwm and pwm[2:6] == b'\0\0\0\0')

    def _limits(self, after=0):
        frame = self.reports.get(0x14, after)
        return frame if frame and all(1 <= value <= 100 for value in frame[2:6]) else None

    def command(self, command):
        now = self.clock()
        action = command.get('action')
        if action == 'keepalive':
            if self.wanted and now < self.deadline:
                self.deadline = now + 2.0
            return
        if action != 'set' or type(command.get('enabled')) is not bool or \
                type(command.get('pressure')) is not int or not 1 <= command['pressure'] <= 100:
            raise ValueError('水泵指令无效')
        if command['enabled'] and not self.ready:
            raise RuntimeError('HC-08 或单片机尚未就绪，请重新打开水泵')
        if command['enabled']:
            mode = self.reports.get(0x03)
            if not mode or mode[4] > 3 or mode[4] & 1:
                raise RuntimeError('单片机不在手动模式，请先停止自动控制')
        self.token, self.pressure = command['token'], command['pressure']
        self.wanted = command['enabled']
        self.deadline = now + 2.0
        self.confirm_deadline = now + 2.0
        self.output_after = self.reports.sequence
        self.off_reported = False
        if not self.connected:
            self.active = False
            self.emit(enabled=False, pressure=self.pressure, token=self.token)
            return
        self.write(STOP)
        self.stage = 'stopping'
        self.active = False
        if self.wanted:
            limits = self._limits()
            if limits is None:
                raise RuntimeError('未收到单片机输出参数，拒绝使用旧压力开启')
            self.parameter_after = self.reports.sequence
            if limits[5] != self.pressure:
                # BB15 的两个值是右上/右下百分比；只改变右下，保留回读的右上值。
                # 参数只在用户改压力时发一次，不在 200 ms 续租中重写 Flash。
                self.write(bytes((0xBB, 0x15, limits[4], self.pressure, 0x55)))
            else:
                self.parameter_after = 0
            self.stage = 'parameters'

    def tick(self):
        try:
            self._tick()
        except Exception as exc:
            self.fail(str(exc))

    def _tick(self):
        now = self.clock()
        if not self.connected:
            if now < self.retry_at:
                return
            self.establish()
        self.transport.poll()
        if not self.transport.connected():
            raise RuntimeError('HC-08 蓝牙已断开，水泵已停止')
        if now >= self.next_query:
            self.write(QUERY)
            self.next_query = now + 1.0
        if now >= self.next_diagnostic:
            self.write(DIAGNOSTIC)
            self.next_diagnostic = now + 1.0
        protection = self.reports.get(0xD1)
        if protection and protection[2] != 0:
            raise RuntimeError('单片机输出保护生效，水泵已停止')
        if not self.ready:
            frequency = self.reports.get(0x08)
            if self._off_reports() and self._limits() and frequency:
                self.ready = True
                self.emit(ready=True, enabled=False, frequency_hz=(frequency[2] * 256 + frequency[3]) / 2)
            elif now >= self.establish_deadline:
                raise RuntimeError('HC-08 已连接，但未收到单片机手动/停止状态和参数回读')
        if self.ready and any(self.reports.get(code) is None for code in (0x03, 0x04, 0xD0)):
            raise RuntimeError('单片机状态超时，水泵已停止')
        if self.wanted and now >= self.deadline:
            self.wanted = False
            self.stage = 'stopping'
            self.output_after = self.reports.sequence
            self.confirm_deadline = now + 2.0
            self.write(STOP)
            self.emit(enabled=False, reason='command_timeout')
        if self.stage == 'parameters':
            if not self.off_reported and self._off_reports(self.output_after):
                # 调压先停后的 MCU 回报同步给平板，等待阶段不能沿用旧的开启状态。
                self.emit(enabled=False, token=self.token)
                self.off_reported = True
            limits = self._limits(self.parameter_after)
            if limits and limits[5] == self.pressure:
                mode = self.reports.get(0x03)
                if not mode or mode[4] > 3 or mode[4] & 1:
                    raise RuntimeError('单片机已切自动模式，水泵已停止')
                self.output_after = self.reports.sequence
                self.write(RIGHT_DOWN)
                self.next_action = now + 0.2
                self.stage = 'running'
        if self.stage == 'running' and self.wanted:
            mode = self.reports.get(0x03)
            if mode and mode[4] & 1:
                raise RuntimeError('单片机已切自动模式，水泵已停止')
            limits = self._limits()
            if limits and limits[5] != self.pressure:
                raise RuntimeError('单片机右下限幅已被修改，请重新打开水泵')
            if now >= self.next_action:
                self.write(RIGHT_DOWN)
                self.next_action = now + 0.2
            bits = self.reports.get(0xD0, self.output_after)
            pwm = self.reports.get(0x04, self.output_after)
            if bits and bits[2] == 3 and bits[3] == 8 and pwm and \
                    pwm[2:4] == b'\0\0' and 0 < int.from_bytes(pwm[4:6], 'big') <= 4095:
                if not self.active:
                    self.active = True
                    self.emit(enabled=True, pressure=self.pressure, token=self.token)
                self.confirm_deadline = now + 2.0
            elif self.active:
                # 收到新鲜的停输出状态，不能沿用之前的开启确认。
                if bits and bits[3] != 8:
                    raise RuntimeError('单片机右下输出已停止或方向不符')
        if self.stage == 'stopping' and self._off_reports(self.output_after):
            self.active = False
            self.stage = 'idle'
            self.emit(enabled=False, pressure=self.pressure, token=self.token)
        if self.stage != 'idle' and now >= self.confirm_deadline:
            raise RuntimeError('单片机未确认压力或右下输出，水泵已停止')

    def fail(self, message):
        try:
            if self.connected:
                self.write(STOP)
        except Exception:
            pass
        try:
            self.transport.close()
        except Exception:
            pass
        self.connected = self.ready = self.wanted = self.active = False
        self.stage = 'idle'
        self.retry_at = self.clock() + 2.0
        self.emit(ready=False, enabled=False, error=message)

    def close(self):
        try:
            if self.connected:
                self.write(STOP)
        finally:
            self.transport.close()
