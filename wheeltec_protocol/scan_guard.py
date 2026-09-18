#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘驱动内的独立防撞层(参考 nav2_collision_monitor 的思路)。

为什么放在驱动里
----------------
跟随程序、网页遥控、导航经各自租约入口汇入同一驱动。碰撞检查只写在跟随程序里时:
  - 跟随程序崩溃 / 卡死 / 有 bug,底盘照样执行它最后发的指令(直到命令超时);
  - 网页手动遥控完全没有防撞。
驱动是所有指令的必经之路,在这里按雷达实测再兜一层底,谁发的指令都管得住。

规则(故意比跟随程序宽松,正常跟随时不会触发)
--------------------------------------------
- 只看**行驶方向**上、车宽(+余量)范围内的雷达点;转弯时走廊再加宽一点。
- 允许车速 v 满足:延迟距离 + 刹车距离 + 停车余量 <= 到最近障碍的距离。
- 类本身保留 legacy 无雷达直通；正常租约入口另由 MotionAuthority 门控禁止。
- 类本身保留 legacy 断流限速；正常租约入口遇到断流会锁存故障停车。
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
    # 车身自反射过滤余量。故意比跟随程序的 0.05 小:用 0.05 时车头 5cm 内的
    # 障碍物也被当成「车自己」,防撞层永远不会停车。实测自反射点都在车身后部
    # (x <= 0.39m),车头附近没有。
    self_skin_m: float = 0.02
    lateral_margin_m: float = 0.05
    turn_extra_m: float = 0.10     # 转弯时走廊额外加宽
    stop_margin_m: float = 0.04    # 停车后与障碍的最小距离(跟随程序是 0.06~0.12)
    decel_m_s2: float = PROFILE["safety"]["decel_mps2"]
    latency_s: float = PROFILE["safety"]["guard_latency_s"]
    range_min_m: float = 0.15
    scan_timeout_s: float = PROFILE["safety"]["scan_timeout_s"]
    stale_speed_cap: float = 0.15
    min_speed_m_s: float = 0.03    # 允许速度低于此值直接停

    def __post_init__(self):
        for name in ("front_m", "rear_m", "half_width_m", "decel_m_s2"):
            if not getattr(self, name) > 0:
                raise ValueError(f"{name} 必须为正")
        if self.latency_s < 0 or self.stop_margin_m < 0:
            raise ValueError("latency_s / stop_margin_m 不能为负")


class ScanGuard:
    def __init__(self, config=None):
        self.cfg = config or GuardConfig()
        self.points = []
        self.scan_time = None
        self.last_reason = "no_scan_passthrough"
        self.last_gap = None
        self.interventions = 0

    def update_scan(self, ranges, angle_min, angle_increment, range_min, range_max, now):
        c = self.cfg
        if not (math.isfinite(angle_min) and math.isfinite(angle_increment)) or angle_increment == 0:
            return
        lo = max(range_min, c.range_min_m)
        cy, sy = math.cos(c.lidar_yaw_rad), math.sin(c.lidar_yaw_rad)
        pts = []
        for i, r in enumerate(ranges):
            if not (math.isfinite(r) and lo <= r <= range_max):
                continue
            a = angle_min + i * angle_increment
            lx, ly = r * math.cos(a), r * math.sin(a)
            x = c.lidar_x_m + cy * lx - sy * ly
            y = c.lidar_y_m + sy * lx + cy * ly
            # 车身自己的回波(轮廓 + skin 以内)不是障碍
            if (-c.rear_m - c.self_skin_m <= x <= c.front_m + c.self_skin_m
                    and abs(y) <= c.half_width_m + c.self_skin_m):
                continue
            pts.append((x, y))
        self.points = pts
        self.scan_time = now

    def gap(self, direction, turning):
        """行驶方向上车宽走廊内到最近障碍的距离(从车头/车尾算)。"""
        c = self.cfg
        half = c.half_width_m + c.lateral_margin_m + (c.turn_extra_m if turning else 0.0)
        best = math.inf
        for x, y in self.points:
            if abs(y) > half:
                continue
            if direction > 0 and x > c.front_m:
                best = min(best, x - c.front_m)
            elif direction < 0 and x < -c.rear_m:
                best = min(best, -c.rear_m - x)
            elif -c.rear_m <= x <= c.front_m:
                # 已经贴在车侧余量里:不阻止离开,只阻止朝它开(由 direction 分支处理)
                continue
        return best

    def allowed_speed(self, gap):
        """v 使 latency·v + v²/(2a) + stop_margin <= gap。"""
        c = self.cfg
        room = gap - c.stop_margin_m
        if room <= 0:
            return 0.0
        a, t = c.decel_m_s2, c.latency_s
        # v²/(2a) + t·v - room = 0
        return -a * t + math.sqrt((a * t) ** 2 + 2 * a * room)

    def limit(self, speed, turning, now):
        """返回 (限制后的速度, 原因)。原因为 None 表示没有干预。"""
        c = self.cfg
        if not c.enabled or abs(speed) < 1e-6:
            self.last_reason = None if c.enabled else "guard_disabled"
            return speed, None
        if self.scan_time is None:
            self.last_reason = "no_scan_passthrough"
            return speed, None
        if now - self.scan_time > c.scan_timeout_s:
            capped = math.copysign(min(abs(speed), c.stale_speed_cap), speed)
            self.last_reason = "guard_scan_stale"
            if capped != speed:
                self.interventions += 1
            return capped, self.last_reason
        direction = 1 if speed > 0 else -1
        g = self.gap(direction, turning)
        self.last_gap = g
        v_max = self.allowed_speed(g)
        if v_max >= abs(speed):
            self.last_reason = None
            return speed, None
        self.interventions += 1
        if v_max < c.min_speed_m_s:
            self.last_reason = "guard_stop"
            return 0.0, self.last_reason
        self.last_reason = "guard_slow"
        return math.copysign(v_max, speed), self.last_reason
