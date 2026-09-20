#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""激光雷达人体候选点簇 + 相机视线校验(纯 Python,不依赖 ROS)。

``cluster_points``: 把车体坐标系下的扫描点按扫描顺序切成点簇,
两条腿(间距 < 0.45m)合并成一个候选,墙面等大尺寸结构剔除。
这些点簇作为观测交给 person_tracker.PersonTracker(统一多人跟踪器),
由它决定属于哪个人;雷达点簇本身不会创造目标。

``line_of_sight_gap``: 相机/雷达交叉校验只看「车 -> 人」这条视线附近的点,
而不是整个扇形里最近的任何东西(旧做法会把旁边的椅子、门框当成人)。

坐标约定与 footprint.py 一致:原点在后轴中心,x 向前,y 向左。
"""

import math

__all__ = ["Cluster", "cluster_points", "line_of_sight_gap"]


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


def cluster_points(points, break_m=0.10, leg_pair_m=0.55, min_size_m=0.02,
                   max_size_m=0.75, min_points=2, max_range_m=5.0, origin=(0.0, 0.0)):
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

    # 过大的连续结构(墙、柜子)整体丢弃,不参与腿部配对
    small = []
    for seg in segments:
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


if __name__ == "__main__":
    import doctest
    doctest.testmod()
