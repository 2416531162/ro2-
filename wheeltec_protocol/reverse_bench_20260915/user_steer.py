#!/usr/bin/env python3
"""One-shot front wheel steering test: simultaneously populated Y and Z channels (stationary)."""
import json
import os
from pathlib import Path
import signal
import struct
import sys
import time

from driver_reference import FrameParser, STOP_FRAME, DEFAULT_PORT

STEER_VALUE = 500  # Wire value in bytes 5..6 (Y) and bytes 7..8 (Z)
MODE_BYTE = 0
STEER_SECONDS = 5.0
STARTUP_STOP_SECONDS = 2.0
FINAL_STOP_SECONDS = 5.0


def bcc(data):
    res = 0
    for b in data:
        res ^= b
    return res


def build_dual_steer_frame(speed, steer_val, mode=0):
    # Populate BOTH bytes 5..6 (Y) and bytes 7..8 (Z) with steer_val
    frame = bytes([0x7B, mode, 0]) + struct.pack(">hhh", speed, steer_val, steer_val)
    return frame + bytes([bcc(frame), 0x7D])


class SteerBench:
    def __init__(self, serial, clock=time):
        self.ser, self.clock = serial, clock
        self.log = []
        self.parser = FrameParser()
        self.started = clock.monotonic()
        self.last_rx = None
        self.latest = None
        self.stationary = 0
        self.interrupted = False
        self.errors = []
        self.phase = 'startup_stop'

    def receive(self):
        data = self.ser.read(min(self.ser.in_waiting, 4096))
        for telemetry in self.parser.feed(data):
            self.last_rx = self.clock.monotonic()
            self.latest = telemetry
            velocity = telemetry['velocity']
            self.stationary = self.stationary + 1 if max(abs(x) for x in velocity) < 0.015 else 0
            self.log.append(dict(t=round(self.last_rx - self.started, 4), kind='rx',
                                 phase=self.phase, **telemetry))

    def guard(self):
        now = self.clock.monotonic()
        if self.interrupted:
            raise RuntimeError('operator_stop')
        if self.last_rx is None or now - self.last_rx > 0.20:
            raise RuntimeError('feedback_stale')
        if self.latest['stop_flag_raw'] != 0:
            raise RuntimeError('controller_inhibit')
        vx, vy, wz = self.latest['velocity']
        if abs(vx) > 0.035 or abs(vy) > 0.035:
            raise RuntimeError(f'unexpected_wheel_drive: vx={vx}, vy={vy}')
        if self.parser.bad:
            raise RuntimeError('corrupt_telemetry')

    def send(self, frame):
        if self.ser.out_waiting:
            self.ser.reset_output_buffer()
            raise RuntimeError('serial_output_backlog')
        if self.ser.write(frame) != len(frame):
            raise RuntimeError('short_write')
        now = self.clock.monotonic()
        self.log.append(dict(t=round(now - self.started, 4), kind='tx', phase=self.phase, frame=frame.hex()))

    def phase_loop(self, phase, seconds, frame, guarded=False):
        self.phase = phase
        end = self.clock.monotonic() + seconds
        while self.clock.monotonic() < end:
            self.receive()
            if guarded:
                self.guard()
            self.send(frame)
            self.clock.sleep(0.02)

    def run(self):
        steer_frame = build_dual_steer_frame(0, STEER_VALUE, mode=MODE_BYTE)
        center_frame = build_dual_steer_frame(0, 0, mode=MODE_BYTE)
        print(f"STEER_FRAME: {steer_frame.hex()} (mode={MODE_BYTE}, Y={STEER_VALUE}, Z={STEER_VALUE})")
        try:
            # 1. 前置静止检测
            self.phase_loop('startup_stop', STARTUP_STOP_SECONDS, STOP_FRAME)
            self.guard()
            if self.stationary < 10:
                raise RuntimeError('not_stationary_before_steer')
            
            # 2. 持续下发双通道转向 5.0 秒
            print(f'START_DUAL_CHANNEL_STEER val={STEER_VALUE} for {STEER_SECONDS}s', flush=True)
            self.phase_loop('steer_active', STEER_SECONDS, steer_frame, guarded=True)
            
            # 3. 回正并恢复停车
            print('RETURN_TO_CENTER', flush=True)
            self.phase_loop('return_center', 1.0, center_frame, guarded=True)
        except Exception as exc:
            self.errors.append(str(exc))
            print('TEST_ABORTED', str(exc), 'sending_stop', flush=True)
        finally:
            print('FINAL_STOP_PHASE', FINAL_STOP_SECONDS, flush=True)
            try:
                self.phase_loop('final_stop', FINAL_STOP_SECONDS, STOP_FRAME)
                self.receive()
            except Exception as exc:
                self.errors.append('stop_io:' + str(exc))
            finally:
                self.ser.close()
        
        stop_ok = self.stationary >= 10 and not self.errors
        result = dict(
            steer_value=STEER_VALUE,
            mode_byte=MODE_BYTE,
            steer_frame=steer_frame.hex(),
            stop_confirmed=stop_ok,
            errors=self.errors
        )
        print('STEER_RESULT ' + json.dumps(result, sort_keys=True), flush=True)
        return result


def main():
    if sys.argv[1:] != ['--execute-one-shot']:
        raise SystemExit('Explicit --execute-one-shot required')
    import serial
    p = Path(__file__).resolve().parent
    fd = os.open(p / 'attempt-user-steer-dual.started', os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    os.close(fd)
    ser = serial.Serial(DEFAULT_PORT, 115200, timeout=0, write_timeout=0.02, exclusive=True)
    bench = SteerBench(ser)
    signal.signal(signal.SIGTERM, lambda *_: setattr(bench, 'interrupted', True))
    signal.signal(signal.SIGINT, lambda *_: setattr(bench, 'interrupted', True))
    try:
        result = bench.run()
        (p / 'user-steer-result.json').write_text(json.dumps(result, indent=2) + '\n')
        (p / 'user-steer-trace.json').write_text(json.dumps(bench.log, indent=2) + '\n')
        return 0 if result['stop_confirmed'] and not result['errors'] else 1
    finally:
        if ser.is_open:
            ser.close()


if __name__ == '__main__':
    sys.exit(main())
