#!/usr/bin/env python3
"""One-shot user reverse motion pulse with guarded execution and verified stop tail."""
import json
import os
from pathlib import Path
import signal
import sys
import time

from driver_reference import FrameParser, build_frame, STOP_FRAME, DEFAULT_PORT

PULSE_SPEED = -0.20
PULSE_SECONDS = 3.5
STARTUP_STOP_SECONDS = 3.0
FINAL_STOP_SECONDS = 10.0


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
        self.pulse_start = self.pulse_end = None
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
        if vx > 0.03:
            raise RuntimeError('unexpected_forward_feedback')
        if abs(vx) > 0.35 or abs(vy) > 0.08 or abs(wz) > 0.40:
            raise RuntimeError(f'unexpected_velocity: vx={vx}, vy={vy}, wz={wz}')
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
        try:
            self.phase_loop('startup_stop', STARTUP_STOP_SECONDS, STOP_FRAME)
            self.guard()
            if self.stationary < 10:
                raise RuntimeError('not_stationary_before_pulse')
            self.pulse_start = self.clock.monotonic() - self.started
            print('START_REVERSE_PULSE', PULSE_SPEED, PULSE_SECONDS, flush=True)
            self.phase_loop('reverse', PULSE_SECONDS, build_frame(PULSE_SPEED, 0), guarded=True)
        except Exception as exc:
            self.errors.append(str(exc))
            print('TEST_ABORTED', str(exc), 'sending_stop', flush=True)
        finally:
            self.pulse_end = self.clock.monotonic() - self.started
            print('STOP_PHASE', FINAL_STOP_SECONDS, flush=True)
            try:
                self.phase_loop('final_stop', FINAL_STOP_SECONDS, STOP_FRAME)
                self.receive()
            except Exception as exc:
                self.errors.append('stop_io:' + str(exc))
                print('CUT_POWER_NOW', str(exc), flush=True)
            finally:
                self.ser.close()
        now = self.clock.monotonic()
        stop_ok = self.last_rx is not None and now - self.last_rx < 0.20 and self.stationary >= 20
        if not stop_ok:
            self.errors.append('final_stop_not_confirmed')
            print('CUT_POWER_NOW final_stop_not_confirmed', flush=True)
        velocities = [r['velocity'][0] for r in self.log if r['kind'] == 'rx' and r['t'] >= STARTUP_STOP_SECONDS]
        result = dict(
            nonzero_frames=self.nonzero,
            stop_frames=self.zeros,
            pulse_window_s=round(self.pulse_end - self.pulse_start, 4) if self.pulse_start is not None else 0,
            feedback_min_vx=min(velocities, default=None),
            feedback_max_vx=max(velocities, default=None),
            reverse_feedback_observed=any(v < -0.015 for v in velocities),
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
    fd = os.open(p / 'attempt-user-run-v3.started', os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    os.close(fd)
    ser = serial.Serial(DEFAULT_PORT, 115200, timeout=0, write_timeout=0.02, exclusive=True)
    bench = Bench(ser)
    signal.signal(signal.SIGTERM, lambda *_: setattr(bench, 'interrupted', True))
    signal.signal(signal.SIGINT, lambda *_: setattr(bench, 'interrupted', True))
    try:
        result = bench.run()
        (p / 'user-live-result.json').write_text(json.dumps(result, indent=2) + '\n')
        (p / 'user-trace.json').write_text(json.dumps(bench.log, indent=2) + '\n')
        return 0 if result['final_stop_confirmed'] and not result['errors'] else 1
    finally:
        if ser.is_open:
            ser.close()


if __name__ == '__main__':
    sys.exit(main())
