"""独立进程生成高电平占空比，父进程退出/续租超时立即输出低电平。"""
import json
import math
import os
import selectors
import signal
import sys
import time

from pump_gpio import PumpOutputLine


class Waveform:
    def __init__(self, frequency):
        if not math.isfinite(frequency) or not 1 <= frequency <= 100:
            raise ValueError('频率必须为 1–100 Hz')
        self.period = 1.0 / frequency
        self.pressure = 50
        self.enabled = False
        self.deadline = 0.0
        self.started = 0.0

    def command(self, command, now):
        if command['action'] == 'set':
            self.enabled = command['enabled']
            self.pressure = command['pressure']
            self.started = now
            self.deadline = now + 2.0
        elif command['action'] == 'keepalive' and self.enabled and now < self.deadline:
            self.deadline = now + 2.0

    def sample(self, now):
        if self.enabled and now >= self.deadline:
            self.enabled = False
        if not self.enabled:
            return False, 0.05
        if self.pressure == 100:
            return True, min(0.05, self.deadline - now)
        phase = (now - self.started) % self.period
        high_time = self.period * self.pressure / 100.0
        high = phase < high_time
        remaining = (high_time - phase) if high else (self.period - phase)
        return high, min(remaining, self.deadline - now)


def emit(**message):
    print(json.dumps(message, ensure_ascii=False), flush=True)


def main():
    waveform = Waveform(float(os.environ.get('RK3588_PUMP_PWM_HZ', '50')))
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    line = None
    try:
        line = PumpOutputLine(os.environ.get('RK3588_PUMP_GPIOCHIP') or None)
        emit(ready=True, enabled=False)
        selector = selectors.DefaultSelector()
        selector.register(sys.stdin, selectors.EVENT_READ)
        os.set_blocking(sys.stdin.fileno(), False)
        buffer = b''
        last_high = False
        while not stopped:
            was_enabled = waveform.enabled
            high, wait = waveform.sample(time.monotonic())
            if high != last_high:
                line.high(high)
                last_high = high
            if was_enabled and not waveform.enabled:
                emit(enabled=False, reason='command_timeout')
            events = selector.select(max(0.0, wait))
            if not events:
                continue
            data = os.read(sys.stdin.fileno(), 4096)
            if not data:
                break
            buffer += data
            if len(buffer) > 65536:
                raise ValueError('水泵指令积压')
            while b'\n' in buffer:
                raw, buffer = buffer.split(b'\n', 1)
                cmd = json.loads(raw)
                if cmd['action'] == 'close':
                    stopped = True
                    break
                waveform.command(cmd, time.monotonic())
                if cmd['action'] == 'set':
                    # 先完成电气状态变化，才报告已执行。
                    high, _ = waveform.sample(time.monotonic())
                    line.high(high)
                    last_high = high
                    emit(enabled=waveform.enabled, pressure=waveform.pressure, token=cmd['token'], reason='user')
    except Exception as exc:
        emit(ready=False, enabled=False, error=str(exc))
    finally:
        if line is not None:
            line.close()


if __name__ == '__main__':
    main()
