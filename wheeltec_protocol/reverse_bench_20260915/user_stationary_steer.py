#!/usr/bin/env python3
"""One-shot stationary front wheel steering test using micro-speed trigger (5 mm/s)."""
import json
import os
from pathlib import Path
import signal
import sys
import time

from driver_reference import FrameParser, build_frame, STOP_FRAME, DEFAULT_PORT

# Micro-speed trigger: 5 mm/s forward is far below static friction deadband (drive wheels stay stationary),
# but satisfies lower-level firmware (Vx != 0), computing TurnR = 0.005 / 0.020 = 0.25m -> Left Lock (+0.35 rad).
MICRO_SPEED = 0.005          # 0.005 m/s (5 mm/s)
STEER_Z = 0.020              # 0.020 rad/s (20 mrad/s)
STEER_HOLD_SECONDS = 4.0     # Hold steering for 4.0s for clear observation
STARTUP_STOP_SECONDS = 2.0   # Confirm stationary before starting
RETURN_CENTER_SECONDS = 1.5  # Return wheels to center
FINAL_STOP_SECONDS = 5.0     # 5.0s zero-frame stop tail


class Bench:
    def __init__(self, serial, clock=time, log=None):
        self.ser, self.clock = serial, clock
        self.log = log if log is not None else []
        self.parser = FrameParser()
        self.started = clock.monotonic()
        self.last_rx = None
        self.latest = None
        self.stationary = 0
        self.interrupted = False
        self.nonzero = self.zeros = 0
        self.steer_start = self.steer_end = None
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
            raise RuntimeError('operator_or_watchdog_stop')
        if self.last_rx is None or now - self.last_rx > 0.20:
            raise RuntimeError('feedback_stale')
        if self.latest['stop_flag_raw'] != 0:
            raise RuntimeError('controller_inhibit')
        vx, vy, wz = self.latest['velocity']
        # Guard: In stationary test, neither vx nor vy should ever exceed 0.035 m/s
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
        moving = frame != STOP_FRAME
        self.nonzero += int(moving)
        self.zeros += int(not moving)
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
        steer_frame = build_frame(MICRO_SPEED, STEER_Z)
        center_frame = build_frame(0, 0)
        print(f"STATIONARY_STEER_FRAME: {steer_frame.hex()} (speed={MICRO_SPEED}, steer_z={STEER_Z})", flush=True)
        try:
            # 1. 前置静止检测
            self.phase_loop('startup_stop', STARTUP_STOP_SECONDS, STOP_FRAME)
            self.guard()
            if self.stationary < 10:
                raise RuntimeError('not_stationary_before_test')
            
            # 2. 下发微速触发转向指令（保持 4.0 秒）
            self.steer_start = self.clock.monotonic() - self.started
            print(f'START_STATIONARY_STEER for {STEER_HOLD_SECONDS}s', flush=True)
            self.phase_loop('steer_active', STEER_HOLD_SECONDS, steer_frame, guarded=True)

            # 3. 回正
            print('RETURN_TO_CENTER', flush=True)
            self.phase_loop('return_center', RETURN_CENTER_SECONDS, center_frame, guarded=True)
        except Exception as exc:
            self.errors.append(str(exc))
            print('TEST_ABORTED', str(exc), 'sending_stop', flush=True)
        finally:
            self.steer_end = self.clock.monotonic() - self.started
            print('FINAL_STOP_PHASE', FINAL_STOP_SECONDS, flush=True)
            try:
                self.phase_loop('final_stop', FINAL_STOP_SECONDS, STOP_FRAME)
                self.receive()
            except Exception as exc:
                self.errors.append('stop_io:' + str(exc))
                print('CUT_POWER_NOW', str(exc), flush=True)
            finally:
                self.ser.close()
        
        now = self.clock.monotonic()
        stop_ok = self.last_rx is not None and now - self.last_rx < 0.20 and self.stationary >= 15
        if not stop_ok:
            self.errors.append('final_stop_not_confirmed')
            print('CUT_POWER_NOW final_stop_not_confirmed', flush=True)
        velocities = [r['velocity'][0] for r in self.log if r['kind'] == 'rx' and r['t'] >= STARTUP_STOP_SECONDS]
        result = dict(
            micro_speed=MICRO_SPEED,
            steer_z=STEER_Z,
            steer_frame=steer_frame.hex(),
            hold_window_s=round(self.steer_end - self.steer_start, 4) if self.steer_start is not None else 0,
            feedback_min_vx=min(velocities, default=None),
            feedback_max_vx=max(velocities, default=None),
            stayed_stationary=all(abs(v) < 0.02 for v in velocities),
            final_stop_confirmed=stop_ok,
            errors=self.errors
        )
        print('BENCH_RESULT ' + json.dumps(result, sort_keys=True), flush=True)
        return result


def main():
    if sys.argv[1:] != ['--execute-one-shot']:
        raise SystemExit('Explicit --execute-one-shot required')
    import serial
    p = Path(__file__).resolve().parent
    flag_file = p / 'attempt-user-stationary-steer.started'
    fd = os.open(flag_file, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    os.close(fd)
    ser = serial.Serial(DEFAULT_PORT, 115200, timeout=0, write_timeout=0.02, exclusive=True)
    bench = Bench(ser)
    signal.signal(signal.SIGTERM, lambda *_: setattr(bench, 'interrupted', True))
    signal.signal(signal.SIGINT, lambda *_: setattr(bench, 'interrupted', True))
    try:
        result = bench.run()
        (p / 'user-stationary-steer-result.json').write_text(json.dumps(result, indent=2) + '\n')
        (p / 'user-stationary-steer-trace.json').write_text(json.dumps(bench.log, indent=2) + '\n')
        return 0 if result['final_stop_confirmed'] and not result['errors'] else 1
    finally:
        if ser.is_open:
            ser.close()


if __name__ == '__main__':
    sys.exit(main())
