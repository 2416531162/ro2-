"""Motion authority, independent of ROS and serial I/O.

Caller holds the driver's lock. A new epoch revokes all queued commands. Source
age uses ROS time; watchdogs and stationary dwell use injected monotonic time.
"""
from dataclasses import dataclass
import math
import uuid


@dataclass(frozen=True)
class Request:
    vx: float = 0.0
    wz: float = 0.0


class MotionAuthority:
    SOURCES = ('manual', 'follow', 'navigation')

    def __init__(self, timeout_s=.35, settle_s=.30, fault_grace_s=0.0):
        if not all(math.isfinite(v) and v > 0 for v in (timeout_s, settle_s)):
            raise ValueError('invalid authority timeout')
        if not math.isfinite(fault_grace_s) or fault_grace_s < 0:
            raise ValueError('invalid fault grace')
        self.timeout_s, self.settle_s = timeout_s, settle_s
        self.fault_grace_s = fault_grace_s
        self.mode = 'IDLE'
        self.epoch = uuid.uuid4().hex
        self.reason = 'startup'
        self.healthy = False
        self.stationary_since = None
        self.wait_stationary = True
        self.request = Request()
        self.received = None
        self.last_stamp = {}
        self.unhealthy_since = None

    def _transition(self, mode, reason):
        self.mode, self.reason = mode, reason
        self.epoch = uuid.uuid4().hex
        self.request = Request()
        self.received = None
        self.last_stamp.clear()
        self.stationary_since = None
        self.wait_stationary = True

    def health(self, healthy, stationary, now):
        if not healthy:
            # Stop on the first unhealthy sample, but do not permanently latch a
            # routine scheduler/USB hiccup. Clearing the request prevents an old
            # speed from resuming when health returns; the selected source must
            # submit a fresh command.
            if self.unhealthy_since is None:
                self.unhealthy_since = now
                self.request = Request()
                self.received = None
            self.healthy = False
            if (self.mode in ('MANUAL', 'FOLLOW', 'NAVIGATION')
                    and now - self.unhealthy_since >= self.fault_grace_s):
                self._transition('FAULT', 'sensor_or_driver_fault')
            self.stationary_since = None
            return
        self.healthy = True
        self.unhealthy_since = None
        if stationary:
            if self.stationary_since is None:
                self.stationary_since = now
            if now - self.stationary_since >= self.settle_s:
                self.wait_stationary = False
        else:
            self.stationary_since = None

    def select(self, source):
        if source not in ('follow', 'navigation'):
            raise ValueError('invalid autonomous source')
        if not self.healthy or self.mode in ('ESTOP', 'FAULT'):
            return False
        self._transition(source.upper(), 'operator_select')
        return True

    def stop(self, emergency=False):
        self._transition('ESTOP' if emergency else 'IDLE', 'operator_stop')

    def release(self, source):
        if self.mode == source.upper():
            self.stop()

    def reset(self):
        if not self.healthy or self.wait_stationary or self.stationary_since is None:
            return False
        self._transition('IDLE', 'reset')
        return True

    def submit(self, source, data, now, ros_now):
        if source not in self.SOURCES or not isinstance(data, dict):
            return False
        try:
            vx, wz, stamp = (data[k] for k in ('vx', 'wz', 'stamp'))
            if not all(type(v) in (int, float) and math.isfinite(v) for v in (vx, wz, stamp)):
                return False
            if data['epoch'] != self.epoch or stamp <= 0 or not 0 <= ros_now - stamp <= self.timeout_s:
                return False
            if stamp <= self.last_stamp.get(source, -math.inf):
                return False
        except (KeyError, TypeError):
            return False
        if not self.healthy or self.mode in ('ESTOP', 'FAULT'):
            return False
        if source == 'manual':
            if abs(vx) < 1e-6:
                self.stop()
                return True
            if self.mode != 'MANUAL':
                self._transition('MANUAL', 'manual_takeover')
        elif self.mode != source.upper():
            return False
        self.last_stamp[source] = stamp
        self.request = Request(float(vx), float(wz) if abs(vx) >= 1e-6 else 0.0)
        self.received = now - (ros_now - stamp)
        return True

    def output(self, now):
        if self.received is not None and not 0 <= now - self.received <= self.timeout_s:
            self._transition('IDLE', 'command_timeout')
        if not self.healthy or self.wait_stationary or self.received is None:
            return Request()
        return self.request

    def status(self, now):
        command = self.output(now)
        unhealthy_for = None if self.unhealthy_since is None else max(0.0, now - self.unhealthy_since)
        return dict(mode=self.mode, epoch=self.epoch, reason=self.reason,
                    healthy=self.healthy, waiting_stationary=self.wait_stationary,
                    fault_pending=bool(not self.healthy
                                       and self.mode in ('MANUAL', 'FOLLOW', 'NAVIGATION')),
                    unhealthy_for_s=(round(unhealthy_for, 3) if unhealthy_for is not None else None),
                    command=dict(vx=command.vx, wz=command.wz))
