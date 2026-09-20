#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘驱动内的独立防撞层(参考 nav2_collision_monitor 的思路)。

为什么放在驱动里
----------------
跟随程序、网页遥控、导航经各自租约入口汇入同一驱动。碰撞检查只写在跟随程序里时:
  - 跟随程序崩溃 / 卡死 / 有 bug,底盘照样执行它最后发的指令(直到命令超时);
  - 导航指令可能缺少独立的防撞兜底。
驱动对自动指令按雷达实测再兜一层底。人工手动接管在驱动入口显式跳过
此软件防撞层；底盘故障、急停和命令超时仍由驱动独立处理。

规则
----
- 沿指令转角和当前转角检查车辆四角的实际扫掠，不把侧面障碍当成前方空地。
- 新扫过的区域需要新鲜、可证实的激光回波；盲区和车身遮挡不可当作空地。
- 允许车速 v 满足:延迟距离 + 刹车距离 + 停车余量 <= 到第一个障碍/未知区域的距离。
- 无效、缺失或过期扫描一律停车；正常租约入口另外锁存传感器故障。
纯 Python,不依赖 ROS。坐标:原点后轴中心,x 前 y 左。
"""

import math
from dataclasses import dataclass
from runtime_config import PROFILE

__all__ = ["GuardConfig", "ScanGuard"]


@dataclass(frozen=True)
class GuardConfig:
    enabled: bool = True
    lidar_x_m: float = PROFILE["sensors"]["lidar_x_m"]
    lidar_y_m: float = PROFILE["sensors"]["lidar_y_m"]
    lidar_yaw_rad: float = PROFILE["sensors"]["lidar_yaw_rad"]
    front_m: float = PROFILE["geometry"]["front_m"]    # 后轴 -> 车头
    rear_m: float = PROFILE["geometry"]["rear_m"]    # 后轴 -> 车尾
    half_width_m: float = PROFILE["geometry"]["half_width_m"]
    # 轮廓外的回波不能按自反射丢掉：紧贴车角的门框可能就在 2cm 内。
    self_skin_m: float = 0.0
    lateral_margin_m: float = 0.05
    turn_extra_m: float = 0.10     # 保留旧参数；扫掠不再用等宽走廊近似
    stop_margin_m: float = 0.04    # 停车后与障碍的最小距离(跟随程序是 0.06~0.12)
    decel_m_s2: float = PROFILE["safety"]["decel_mps2"]
    latency_s: float = PROFILE["safety"]["guard_latency_s"]
    range_min_m: float = 0.15
    scan_timeout_s: float = PROFILE["safety"]["scan_timeout_s"]
    stale_speed_cap: float = 0.15
    min_speed_m_s: float = 0.03    # 允许速度低于此值直接停
    wheelbase_m: float = PROFILE["geometry"]["wheelbase_m"]
    track_m: float = PROFILE["geometry"]["track_m"]
    max_steer_rad: float = PROFILE["geometry"]["max_steer_rad"]

    def __post_init__(self):
        for name in ("front_m", "half_width_m", "decel_m_s2", "wheelbase_m", "max_steer_rad"):
            if not math.isfinite(getattr(self, name)) or not getattr(self, name) > 0:
                raise ValueError(f"{name} 必须为正")
        for name in ("rear_m", "track_m", "self_skin_m", "lateral_margin_m",
                     "latency_s", "stop_margin_m", "scan_timeout_s"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f"{name} 不能为负或非有限值")


class ScanGuard:
    STEP_M = 0.02
    OBS_WINDOW_RAD = math.radians(1.5)

    def __init__(self, config=None):
        self.cfg = config or GuardConfig()
        self.points = []
        self.scan_time = None
        self.last_reason = "guard_scan_unavailable"
        self.last_gap = None
        self.last_block = None
        self.interventions = 0
        self._rays = ()
        self._blocked_rays = ()
        self._lower = self._upper = ()
        self._path_cache = {}
        self._angle_min = self._increment = self._mid = 0.0
        self._full = False
        self._perimeter = self._body_perimeter()

    def invalidate(self):
        self.points = []
        self.scan_time = None
        self._rays = self._blocked_rays = ()
        self._lower = self._upper = ()
        self._path_cache.clear()
        self.last_gap = None
        self.last_block = None
        self.last_reason = "guard_scan_unavailable"

    def _body_perimeter(self):
        c = self.cfg
        half = c.half_width_m + c.lateral_margin_m
        nx = max(1, math.ceil((c.front_m + c.rear_m) / 0.05))
        ny = max(1, math.ceil(2 * half / 0.05))
        sides = [(-c.rear_m + (c.front_m + c.rear_m) * i / nx, y)
                 for i in range(nx + 1) for y in (-half, half)]
        ends = [(x, -half + 2 * half * i / ny)
                for i in range(ny + 1) for x in (-c.rear_m, c.front_m)]
        return sides + ends

    def update_scan(self, ranges, angle_min, angle_increment, range_min, range_max, now):
        c = self.cfg
        if (not ranges or not all(math.isfinite(v) for v in (angle_min, angle_increment,
                                                            range_min, range_max, now))
                or not 0 < abs(angle_increment) <= math.radians(2)
                or not 0 <= range_min < range_max):
            self.invalidate()
            return
        lo = max(range_min, c.range_min_m)
        cy, sy = math.cos(c.lidar_yaw_rad), math.sin(c.lidar_yaw_rad)
        pts = []
        rays = [None] * len(ranges)
        blocked = [False] * len(ranges)
        for i, r in enumerate(ranges):
            if not math.isfinite(r):
                blocked[i] = r == -math.inf
                continue
            if r < lo:
                blocked[i] = True
                continue
            if r > range_max:
                continue
            a = angle_min + i * angle_increment
            lx, ly = r * math.cos(a), r * math.sin(a)
            x = c.lidar_x_m + cy * lx - sy * ly
            y = c.lidar_y_m + sy * lx + cy * ly
            # 自身回波截断该射线，车身后面的区域仍然未知。
            if (-c.rear_m - c.self_skin_m <= x <= c.front_m + c.self_skin_m
                    and abs(y) <= c.half_width_m + c.self_skin_m):
                blocked[i] = True
                continue
            rays[i] = r
            pts.append((x, y))
        self.points = pts
        self.scan_time = now
        self._rays, self._blocked_rays = rays, blocked
        self._angle_min, self._increment = angle_min, angle_increment
        self._mid = angle_min + (len(ranges) - 1) * angle_increment / 2
        self._full = abs(angle_increment) * len(ranges) >= 2 * math.pi - 1.1 * abs(angle_increment)
        self._lower = tuple(self._nearest_ray(i, -1) for i in range(len(ranges)))
        self._upper = tuple(self._nearest_ray(i, 1) for i in range(len(ranges)))
        self._path_cache.clear()

    def _nearest_ray(self, start, step):
        n = len(self._rays)
        reach = max(1, int(self.OBS_WINDOW_RAD / abs(self._increment)))
        for offset in range(reach + 1):
            i = start + step * offset
            if self._full:
                i %= n
            elif not 0 <= i < n:
                return None
            if self._blocked_rays[i]:
                return None
            if self._rays[i] is not None:
                return self._rays[i]
        return None

    def _observed_free(self, x, y):
        c = self.cfg
        dx, dy = x - c.lidar_x_m, y - c.lidar_y_m
        distance = math.hypot(dx, dy)
        if distance <= c.range_min_m:
            return False
        a = math.atan2(dy, dx) - c.lidar_yaw_rad
        a += round((self._mid - a) / (2 * math.pi)) * 2 * math.pi
        index = (a - self._angle_min) / self._increment
        lo, hi = math.floor(index), math.ceil(index)
        n = len(self._rays)
        if self._full:
            lo, hi = lo % n, hi % n
        elif lo < 0 or hi >= n:
            return False
        lower, upper = self._lower[lo], self._upper[hi]
        needed = distance + 0.015 + distance * abs(self._increment) / 2
        return lower is not None and upper is not None and needed < min(lower, upper)

    def _path_gap(self, direction, steer, horizon):
        c = self.cfg
        steer = max(-c.max_steer_rad, min(c.max_steer_rad, steer))
        radius = (c.wheelbase_m / math.tan(abs(steer)) + c.track_m / 2
                  if abs(steer) > 1e-6 else math.inf)
        half = c.half_width_m + c.lateral_margin_m
        body_r = math.hypot(max(c.front_m, c.rear_m), half)
        nearby = [(x, y, max(-c.rear_m - x, x - c.front_m, abs(y) - half))
                  for x, y in self.points if x*x + y*y <= (horizon + body_r)**2]
        count = math.ceil(horizon / self.STEP_M)
        for i in range(1, count + 1):
            travel = min(horizon, i * self.STEP_M)
            yaw = 0.0 if math.isinf(radius) else direction * math.copysign(travel / radius, steer)
            px = direction * travel if math.isinf(radius) else radius * math.sin(yaw) * (1 if steer > 0 else -1)
            py = 0.0 if math.isinf(radius) else radius * (1 - math.cos(yaw)) * (1 if steer > 0 else -1)
            co, si = math.cos(yaw), math.sin(yaw)
            for ox, oy, initial_gap in nearby:
                dx, dy = ox - px, oy - py
                lx, ly = dx * co + dy * si, -dx * si + dy * co
                gap = max(-c.rear_m - lx, lx - c.front_m, abs(ly) - half)
                if gap <= 0 and gap < min(0.0, initial_gap) - 1e-7:
                    return travel, "obstacle", ox, oy, steer
            for bx, by in self._perimeter:
                qx, qy = px + bx * co - by * si, py + bx * si + by * co
                # The translated perimeter may land a few ulps outside the
                # original boundary. That is not newly swept, unobserved space.
                if (-c.rear_m - 1e-9 <= qx <= c.front_m + 1e-9
                        and abs(qy) <= half + 1e-9):
                    continue
                if not self._observed_free(qx, qy):
                    return travel, "unknown", qx, qy, steer
        return math.inf, None, None, None, steer

    def gap(self, direction, turning, horizon=2.0):
        """到首次扫掠障碍或未知空间的路径距离；布尔转弯取两侧最坏情况。"""
        if self.scan_time is None:
            return 0.0
        steers = ((-self.cfg.max_steer_rad, self.cfg.max_steer_rad)
                  if turning is True else (0.0,) if turning is False else (float(turning),))
        return min(self._path_gap(direction, steer, horizon)[0] for steer in steers)

    def allowed_speed(self, gap):
        """v 使 latency·v + v²/(2a) + stop_margin <= gap。"""
        c = self.cfg
        room = gap - c.stop_margin_m
        if room <= 0:
            return 0.0
        a, t = c.decel_m_s2, c.latency_s
        # v²/(2a) + t·v - room = 0
        return -a * t + math.sqrt((a * t) ** 2 + 2 * a * room)

    def limit(self, speed, turning, now, *, current_steer=None):
        """返回 (限制后的速度, 原因)。原因为 None 表示没有干预。"""
        c = self.cfg
        if not c.enabled:
            self.last_reason = "guard_disabled"
            self.last_block = None
            return speed, None
        if abs(speed) < 1e-6:
            self.last_reason = ("guard_scan_unavailable" if self.scan_time is None else
                                "guard_scan_stale" if not 0 <= now - self.scan_time < c.scan_timeout_s
                                else None)
            self.last_block = None
            return speed, None
        if self.scan_time is None:
            self.last_gap = None
            self.last_block = None
            self.last_reason = "guard_scan_unavailable"
            self.interventions += 1
            return 0.0, self.last_reason
        if not 0 <= now - self.scan_time < c.scan_timeout_s:
            self.last_gap = None
            self.last_block = None
            self.last_reason = "guard_scan_stale"
            self.interventions += 1
            return 0.0, self.last_reason
        direction = 1 if speed > 0 else -1
        steers = ([-c.max_steer_rad, c.max_steer_rad] if turning is True
                  else [0.0] if turning is False else [float(turning)])
        if current_steer is not None:
            start = max(-c.max_steer_rad, min(c.max_steer_rad, current_steer))
            end = steers[0]
            steps = max(1, math.ceil(abs(end - start) / .05))
            steers = [start + (end - start) * i / steps for i in range(steps + 1)]
        # Only evaluate the distance needed to brake from this speed, with a
        # short buffer; the 50 Hz serial worker must not scan a fixed 2 m arc
        # for every low-speed control tick.
        horizon = max(.35, c.latency_s * abs(speed) + speed*speed / (2*c.decel_m_s2)
                      + c.stop_margin_m + .20)
        results = []
        for steer in steers:
            # A scan is immutable until update_scan(). Identical paths between
            # 10 Hz scans need not hold the serial/ROS lock for another sweep.
            key = (direction, steer, horizon)
            result = self._path_cache.get(key)
            if result is None:
                result = self._path_gap(direction, steer, horizon)
                if len(self._path_cache) >= 32:
                    self._path_cache.clear()
                self._path_cache[key] = result
            results.append(result)
        g, kind, x, y, blocked_steer = min(results, key=lambda item: item[0])
        self.last_gap = g
        self.last_block = (kind, x, y, blocked_steer) if kind is not None else None
        v_max = self.allowed_speed(g)
        if v_max >= abs(speed):
            self.last_reason = None
            return speed, None
        self.interventions += 1
        if v_max < c.min_speed_m_s:
            self.last_reason = "guard_unknown" if kind == "unknown" else "guard_stop"
            return 0.0, self.last_reason
        self.last_reason = "guard_unknown" if kind == "unknown" else "guard_slow"
        return math.copysign(v_max, speed), self.last_reason
