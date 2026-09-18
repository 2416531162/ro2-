"""Continuous local pose history; strict mode consumes one external odometry source.

step() is only for explicit synthetic/legacy replay. Production adds timestamped
poses from /odom and refuses observations outside its available history.
"""
import math
from collections import deque

class PoseHistory:
    def __init__(self, horizon_s=3.0, strict=False, extrapolation_s=.12):
        self.strict = strict
        self.extrapolation_s = extrapolation_s
        self.revision = 0
        self.horizon_s = horizon_s
        self.poses = deque()            # (t, x, y, th)
        self.x = self.y = self.th = 0.0
        self.t = None

    def reset(self):
        self.poses.clear()
        self.x = self.y = self.th = 0.0
        self.t = None

    def step(self, t, speed, yaw_rate):
        if self.t is not None:
            dt = t - self.t
            if 0.0 < dt <= 0.5:
                mid = self.th + yaw_rate * dt / 2
                self.x += speed * dt * math.cos(mid)
                self.y += speed * dt * math.sin(mid)
                self.th += yaw_rate * dt
        self.t = t
        self.poses.append((t, self.x, self.y, self.th))
        while self.poses and t - self.poses[0][0] > self.horizon_s:
            self.poses.popleft()

    def pose_at(self, t):
        """t 时刻的位姿(线性插值);超出缓存范围时取最近端点。"""
        if self.strict and (not self.poses or t < self.poses[0][0]
                            or t > self.poses[-1][0] + self.extrapolation_s):
            return None
        if not self.poses:
            return (self.x, self.y, self.th)
        if t >= self.poses[-1][0]:
            return self.poses[-1][1:]
        if t <= self.poses[0][0]:
            return self.poses[0][1:]
        prev = self.poses[0]
        for cur in self.poses:
            if cur[0] >= t:
                span = cur[0] - prev[0]
                k = 0.0 if span <= 0 else (t - prev[0]) / span
                dth = math.atan2(math.sin(cur[3] - prev[3]), math.cos(cur[3] - prev[3]))
                return (prev[1] + k * (cur[1] - prev[1]),
                        prev[2] + k * (cur[2] - prev[2]),
                        prev[3] + k * dth)
            prev = cur
        return self.poses[-1][1:]

    @staticmethod
    def vehicle_to_odom(pose, px, py):
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        return x + c * px - s * py, y + s * px + c * py

    @staticmethod
    def odom_to_vehicle(pose, ox, oy):
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        dx, dy = ox - x, oy - y
        return c * dx + s * dy, -s * dx + c * dy

    def current(self):
        return (self.x, self.y, self.th)


    def add(self, t, x, y, yaw, max_speed=3., max_yaw_rate=3.):
        if not all(math.isfinite(v) for v in (t, x, y, yaw)):
            return False
        if self.t is not None:
            dt = t - self.t
            if dt <= 0:
                return False
            jump = math.hypot(x-self.x, y-self.y)
            angle = abs(math.atan2(math.sin(yaw-self.th), math.cos(yaw-self.th)))
            if dt > .5 or jump > max_speed*dt + .05 or angle > max_yaw_rate*dt + .05:
                self.reset()
                self.revision += 1
        self.x, self.y, self.th, self.t = x, y, yaw, t
        self.poses.append((t, x, y, yaw))
        while self.poses and t - self.poses[0][0] > self.horizon_s:
            self.poses.popleft()
        return True

