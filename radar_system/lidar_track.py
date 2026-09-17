#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""雷达人体接力跟踪 —— 相机看不到人时,继续用激光雷达跟着「那个人」走。

为什么需要
----------
Astra S 水平视场只有约 60°。人往左一拐就出了画面,旧逻辑立刻判定丢失,
车停在原地观察、再按上一次的方位盲转。但雷达是 360° 的,人的两条腿在扫描里
清清楚楚,一直能看到人往哪边走。

做法(纯 Python,不依赖 ROS)
---------------------------
1. ``cluster_points``: 把车体坐标系下的扫描点按扫描顺序切成点簇,
   两条腿(间距 < 0.45m)合并成一个候选,墙面等大尺寸结构剔除。
2. ``LidarPersonTrack``: 相机确认人时,在人的位置附近「认领」一个点簇
   (seed);之后每帧雷达都在预测位置附近找最近的点簇继续跟,并用底盘实测
   车速/角速度做自车运动补偿。
3. 身份可信度有上限:超过 ``handoff_s`` 没被相机重新确认就判为失效,
   交给原来的丢失搜索逻辑。接力期间跟随节点还会额外限速。

``line_of_sight_gap``: 相机/雷达交叉校验只看「车 -> 人」这条视线附近的点,
而不是整个扇形里最近的任何东西(旧做法会把旁边的椅子、门框当成人)。

坐标约定与 footprint.py 一致:原点在后轴中心,x 向前,y 向左。
"""

import math

__all__ = ["Cluster", "cluster_points", "LidarPersonTrack", "line_of_sight_gap"]


class Cluster:
    __slots__ = ("x", "y", "size", "count")

    def __init__(self, pts):
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        self.x = sum(xs) / len(xs)
        self.y = sum(ys) / len(ys)
        self.size = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        self.count = len(pts)

    def __repr__(self):
        return "Cluster(x=%.2f, y=%.2f, size=%.2f, n=%d)" % (self.x, self.y, self.size, self.count)


def cluster_points(points, break_m=0.15, leg_pair_m=0.45, min_size_m=0.02,
                   max_size_m=0.70, min_points=2, max_range_m=5.0, origin=(0.0, 0.0)):
    """扫描点(按扫描角度顺序) -> 人体尺寸的候选点簇列表。

    >>> legs = [(2.0, 0.10), (2.0, 0.13), (2.02, 0.16), (2.0, -0.10), (2.0, -0.13)]
    >>> [round(c.x, 2) for c in cluster_points(legs)]
    [2.0]
    >>> wall = [(3.0, -1.0 + i * 0.05) for i in range(41)]
    >>> cluster_points(wall)
    []
    """
    segments, current = [], []
    for p in points:
        if current and math.hypot(p[0] - current[-1][0], p[1] - current[-1][1]) > break_m:
            segments.append(current)
            current = []
        current.append(p)
    if current:
        segments.append(current)
    # 360° 扫描首尾相接
    if len(segments) > 1:
        a, b = segments[0][0], segments[-1][-1]
        if math.hypot(a[0] - b[0], a[1] - b[1]) <= break_m:
            segments[0] = segments.pop() + segments[0]

    raw = [s for s in segments if len(s) >= 1]
    # 过大的连续结构(墙、柜子)整体丢弃,不参与腿部配对
    small = []
    for seg in raw:
        c = Cluster(seg)
        if c.size <= max_size_m:
            small.append((seg, c))

    # 两条腿合并:质心相近且合并后仍是人体尺寸
    merged, used = [], [False] * len(small)
    for i, (seg_i, ci) in enumerate(small):
        if used[i]:
            continue
        pts = list(seg_i)
        used[i] = True
        for j in range(i + 1, len(small)):
            if used[j]:
                continue
            seg_j, cj = small[j]
            if math.hypot(ci.x - cj.x, ci.y - cj.y) <= leg_pair_m:
                trial = Cluster(pts + list(seg_j))
                if trial.size <= max_size_m:
                    pts.extend(seg_j)
                    used[j] = True
        merged.append(Cluster(pts))

    ox, oy = origin
    return [c for c in merged
            if c.count >= min_points and c.size >= min_size_m
            and math.hypot(c.x - ox, c.y - oy) <= max_range_m]


def line_of_sight_gap(points, origin, target, front_m, half_width_m=0.25,
                      beyond_m=0.40):
    """「车 -> 目标」视线附近最近的雷达点,返回 (车头间距, 横向偏移 右为正) 或 None。

    只看以视线为中线、半宽 half_width_m 的窄带,且不超过目标后方 beyond_m。
    旧做法取目标方位 ±10°(5° 分桶后实际可达 ±15°)扇形里的最小值,
    2m 外扇形宽 0.7~1m,旁边的椅子、门框都会被当成人。

    >>> pts = [(1.2, 0.6), (2.5, 0.02), (2.52, -0.05)]
    >>> line_of_sight_gap(pts, (0.54, 0.0), (2.6, 0.0), 0.67)
    (1.83, -0.02)
    """
    ox, oy = origin
    tx, ty = target
    dx, dy = tx - ox, ty - oy
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return None
    ux, uy = dx / length, dy / length
    best = None
    for px, py in points:
        rx, ry = px - ox, py - oy
        along = rx * ux + ry * uy
        if along <= 0.0 or along > length + beyond_m:
            continue
        if abs(rx * uy - ry * ux) > half_width_m:
            continue
        if best is None or along < best[0]:
            best = (along, px, py)
    if best is None:
        return None
    _, px, py = best
    return round(px - front_m, 3), round(-py, 3)


class LidarPersonTrack:
    """单目标雷达跟踪,由相机认领、雷达续跟。坐标为车体系 (x 前, y 左)。"""

    def __init__(self, gate_m=0.45, seed_radius_m=0.60, coast_s=0.60,
                 handoff_s=8.0, max_speed_mps=2.0, alpha=0.6, beta=0.25):
        self.gate_m = gate_m
        self.seed_radius_m = seed_radius_m
        self.coast_s = coast_s
        self.handoff_s = handoff_s
        self.max_speed = max_speed_mps
        self.alpha, self.beta = alpha, beta
        self.clusters = []
        self.reset()

    def reset(self):
        self.alive = False
        self.x = self.y = 0.0
        self.vx = self.vy = 0.0
        self.last_time = None
        self.last_match = None
        self.last_seed = None

    # -- 自车运动补偿 ---------------------------------------------------
    def _predict(self, now, speed, yaw_rate):
        if not self.alive or self.last_time is None:
            self.last_time = now
            return
        dt = now - self.last_time
        self.last_time = now
        if not 0.0 < dt <= 0.5:
            return
        # 目标先按自身(相对)速度外推,再扣掉车这一段的平移和转角
        x = self.x + self.vx * dt - speed * dt
        y = self.y + self.vy * dt
        th = -yaw_rate * dt
        c, s = math.cos(th), math.sin(th)
        self.x, self.y = c * x - s * y, s * x + c * y
        self.vx, self.vy = c * self.vx - s * self.vy, s * self.vx + c * self.vy

    def _nearest(self, x, y, radius):
        best, best_d = None, radius
        for cl in self.clusters:
            d = math.hypot(cl.x - x, cl.y - y)
            if d <= best_d:
                best, best_d = cl, d
        return best

    def update(self, clusters, now, speed=0.0, yaw_rate=0.0):
        """每帧雷达调用。返回本帧是否关联上。"""
        self.clusters = list(clusters)
        if not self.alive:
            return False
        prev_t = self.last_time
        self._predict(now, speed, yaw_rate)
        dt = (now - prev_t) if prev_t is not None else 0.0
        gate = self.gate_m + (self.max_speed * dt if 0 < dt <= 0.5 else 0.0) * 0.5
        match = self._nearest(self.x, self.y, gate)
        if match is None:
            if self.last_match is None or now - self.last_match > self.coast_s:
                self.reset()
            return False
        ix, iy = match.x - self.x, match.y - self.y
        self.x += self.alpha * ix
        self.y += self.alpha * iy
        if 0 < dt <= 0.5:
            self.vx += self.beta * ix / dt
            self.vy += self.beta * iy / dt
            v = math.hypot(self.vx, self.vy)
            if v > self.max_speed:
                self.vx *= self.max_speed / v
                self.vy *= self.max_speed / v
        self.last_match = now
        return True

    def seed(self, x, y, now):
        """相机确认人在 (x, y)。认领/确认对应的雷达点簇,返回是否成功。"""
        if self.alive and math.hypot(self.x - x, self.y - y) <= self.seed_radius_m:
            self.last_seed = now
            return True
        match = self._nearest(x, y, self.seed_radius_m)
        if match is None:
            # 相机看得到、雷达找不到(腿被挡住)。已有轨迹若离相机很远,说明跟错了人
            if self.alive and math.hypot(self.x - x, self.y - y) > 2 * self.seed_radius_m:
                self.reset()
            return False
        self.alive = True
        self.x, self.y = match.x, match.y
        self.vx = self.vy = 0.0
        self.last_time = self.last_match = self.last_seed = now
        return True

    def valid(self, now):
        return (self.alive and self.last_match is not None and self.last_seed is not None
                and now - self.last_match <= self.coast_s
                and now - self.last_seed <= self.handoff_s)

    def status(self, now):
        if not self.alive:
            return None
        return {"x": round(self.x, 3), "y": round(self.y, 3),
                "speed": round(math.hypot(self.vx, self.vy), 2),
                "since_camera_s": round(now - self.last_seed, 2) if self.last_seed else None,
                "valid": self.valid(now)}


if __name__ == "__main__":
    import doctest
    doctest.testmod()
