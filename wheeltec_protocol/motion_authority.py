"""Motion authority, independent of ROS and serial I/O.

Caller holds the driver's lock. A new epoch revokes all queued commands. Source
age uses ROS time; watchdogs and stationary dwell use injected monotonic time.
"""
from dataclasses import dataclass
import math
import time
import uuid


@dataclass(frozen=True)
class Request:
    vx: float = 0.0
    wz: float = 0.0


class MotionAuthority:
    SOURCES = ('manual', 'follow', 'navigation')

    def __init__(self, timeout_s=.35, settle_s=.30, fault_grace_s=0.0, allow_pivot=False):
        """allow_pivot：差速车（Gen5）能原地转，手动指令车速 0、角速度不为 0 是原地转，不是停车。"""
        if not all(math.isfinite(v) and v > 0 for v in (timeout_s, settle_s)):
            raise ValueError('invalid authority timeout')
        if not math.isfinite(fault_grace_s) or fault_grace_s < 0:
            raise ValueError('invalid fault grace')
        self.timeout_s, self.settle_s = timeout_s, settle_s
        self.fault_grace_s = fault_grace_s
        self.allow_pivot = bool(allow_pivot)
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
        self.previous_epoch = None
        self.previous_mode = None
        self.epoch_transition_time = 0.0
        self.unhealthy_since_transition = False

    def _transition(self, mode, reason, now=None):
        self.previous_mode = self.mode
        self.mode, self.reason = mode, reason
        self.unhealthy_since_transition = False
        self.previous_epoch = self.epoch
        transition_now = time.monotonic() if now is None else now
        self.epoch_transition_time = transition_now
        self.epoch = uuid.uuid4().hex
        was_moving = abs(self.request.vx) > 1e-6 or (self.allow_pivot and abs(self.request.wz) > 1e-6)
        self.request = Request()
        self.received = None
        self.last_stamp.clear()
        # Preserve measured stationary dwell when taking over a follower that
        # is already stopped; a moving takeover still needs fresh dwell.
        if mode == 'MANUAL' and self.stationary_since is not None and not was_moving:
            self.wait_stationary = transition_now - self.stationary_since < self.settle_s
        else:
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
                self._transition('FAULT', 'sensor_or_driver_fault', now=now)
            self.stationary_since = None
            self.unhealthy_since_transition = True
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
        if self.mode == 'MANUAL':
            return False
        self._transition(source.upper(), 'operator_select')
        return True

    def stop(self, emergency=False):
        self._transition('ESTOP' if emergency else 'IDLE', 'operator_stop')

    def fault(self, reason, now=None):
        if self.mode not in ('FAULT', 'ESTOP'):
            self._transition('FAULT', reason, now=now)

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
            now_epoch = self.epoch
            prev_epoch = getattr(self, 'previous_epoch', None)
            prev_time = getattr(self, 'epoch_transition_time', 0.0)
            epoch_valid = (data['epoch'] == now_epoch) or (
                source == 'manual' and self.mode == 'MANUAL'
                and self.reason == 'manual_takeover'
                and self.previous_mode in ('IDLE', 'FOLLOW', 'NAVIGATION')
                and prev_epoch is not None
                and data['epoch'] == prev_epoch and now - prev_time < 0.60
            ) or (
                # 手动松手刚超时回空闲（换了代次），网页端还没收到新代次就又推了：同一个操作员接着开
                source == 'manual' and self._manual_resume()
                and prev_epoch is not None
                and data['epoch'] == prev_epoch and now - prev_time < 0.60
            )
            if not epoch_valid or stamp <= 0 or not 0 <= ros_now - stamp <= self.timeout_s:
                return False
            if stamp <= self.last_stamp.get(source, -math.inf):
                return False
        except (KeyError, TypeError):
            return False
        if not self.healthy or self.mode in ('ESTOP', 'FAULT'):
            return False
        if source == 'manual':
            if abs(vx) < 1e-6 and (not self.allow_pivot or abs(wz) < 1e-6):
                # 零速（摇杆回中）：手动开着时就地停（留在 MANUAL、不换代次，回中再推马上能走；
                # 松手不再发指令后 timeout_s 超时回 IDLE）；自动驾驶时照旧停车回 IDLE、不会自己恢复；
                # 本来就空闲时什么也不做——app 摇杆回中时每 50 ms 发一次零速，每次都换代次的话
                # 网页端手里的代次总是旧的，再推的指令被丢、还要重新等静止，起步一顿一顿
                if self.mode == 'MANUAL':
                    self.last_stamp[source] = stamp
                    self.request = Request()
                    self.received = now - (ros_now - stamp)
                elif self.mode != 'IDLE':
                    self.stop()
                return True
            if self.mode != 'MANUAL':
                # 手动松手超时回空闲后又推：同一个操作员接着开，不用再等静止 settle_s（车还在滑行时再推，
                # 原来要先停死再等 0.3 s，开起来一顿一顿）；从跟随 / 导航接管、急停或故障复位之后照旧要等
                resume = self._manual_resume()
                self._transition('MANUAL', 'manual_takeover', now=now)
                if resume:
                    self.wait_stationary = False
        elif self.mode != source.upper():
            return False
        self.last_stamp[source] = stamp
        self.request = Request(float(vx), float(wz))
        self.received = now - (ros_now - stamp)
        return True

    def _manual_resume(self):
        # 回空闲之后底盘报过不健康（断链重连、启动保护等）：车的状态不可信，照旧要等停稳
        return (self.mode == 'IDLE' and self.reason == 'command_timeout' and self.previous_mode == 'MANUAL'
                and not self.unhealthy_since_transition)

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
