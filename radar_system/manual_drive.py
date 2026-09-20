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
        self._last_seq = {}

    def accept(self, client_id, seq, vx, wz):
        """按 (页面会话, 序号) 过滤乱序到达的指令,返回是否应当执行。

        - 没带序号的旧客户端一律放行,保持兼容。
        - 刹车(零速)永远放行:宁可多停一下,下一次心跳会重新下发动作。
        - 运动指令的序号不大于该会话已见过的最大序号时丢弃。
        """
        if client_id is None or seq is None:
            return True
        moving = abs(float(vx)) > 1e-4
        with self._lock:
            last = self._last_seq.get(client_id)
            if last is not None and seq <= last:
                return not moving
            if len(self._last_seq) > 64 and client_id not in self._last_seq:
                self._last_seq.clear()
            self._last_seq[client_id] = seq
            return True

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
