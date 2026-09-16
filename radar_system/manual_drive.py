#!/usr/bin/env python3
"""线程安全的手动底盘指令门控。

HTTP 线程只写最新指令，ROS 定时器只读取快照。模块不依赖 ROS，
便于在无底盘环境里回归测试心跳、超时和零速只发一次的行为。
"""

import threading
import time


class ManualDriveLatch:
    def __init__(self, timeout_s=0.75):
        if timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        self.timeout_s = float(timeout_s)
        self._lock = threading.Lock()
        self._vx = 0.0
        self._wz = 0.0
        self._last_command = 0.0
        self._zero_pending = False

    def set(self, vx, wz, now=None):
        vx, wz = float(vx), float(wz)
        if abs(vx) < 1e-4:
            vx, wz = 0.0, 0.0
        moving = abs(vx) > 1e-4 or abs(wz) > 1e-4
        stamp = time.monotonic() if now is None else float(now)
        with self._lock:
            self._vx, self._wz = vx, wz
            self._last_command = stamp if moving else 0.0
            self._zero_pending = not moving

    def sample(self, now=None):
        """返回 (vx, wz, active, publish_zero)。

        active 为真时，调用者应当本周期发布该速度；publish_zero
        为真时，调用者应当发布一帧零速，下一次 sample 不再重复。
        """
        stamp = time.monotonic() if now is None else float(now)
        with self._lock:
            vx, wz = self._vx, self._wz
            moving = abs(vx) > 1e-4 or abs(wz) > 1e-4
            active = moving and stamp - self._last_command <= self.timeout_s
            publish_zero = self._zero_pending
            if moving and not active:
                self._vx = self._wz = 0.0
                vx = wz = 0.0
                publish_zero = True
            if publish_zero:
                self._zero_pending = False
            return vx, wz, active, publish_zero
