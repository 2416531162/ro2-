#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阿克曼车掉头面向侧后方目标:短程前进/倒车多段搜索(纯算法,无 ROS)。

为什么不再用「一直向前满舵绕圈」
--------------------------------
满舵转弯半径约 1.72m。设人在车体系 (x, y),车沿满舵弧线走 ds,人的方位角 β
的变化率为(y>0 时向人那一侧打舵,前进 d=+1,倒车 d=-1 且反打舵):

    前进: dβ/ds = -1/R + y/r²
    倒车: dβ/ds = -1/R - y/r²

人贴着车侧(y/r² > 1/R)时,前进满舵反而让方位角**变大** —— 车越开人越往后跑,
旧状态机在开阔地里可以绕十几秒都转不过来。倒车反打舵总是更快,但会靠近车后的人、
也可能被墙挡住。什么时候前进、什么时候倒车、每段走多远,取决于人的位置和周围
障碍物,没有一条固定规则,所以这里直接搜:

- 每段都是满舵弧线,前进打向人一侧,倒车打反舵,两者都让车头往人那边转;
- 相邻两段换向,每次换向计入停稳 + 舵机摆动的时间;
- 最多 depth 段,每段长度取若干档;第一段不超过雷达认证的净空;
- 所有段都要求车身与已知障碍点、与人保持余量;
- 代价是到达「人在相机前方 ±goal 内」的预计用时;都到不了就选终点方位最小的。

输出只是**参考量**:方向、满舵转角、本段剩余长度。执行时仍要经过扫掠净空、
刹车包络、AEB 与驱动限幅。每个控制周期从当前位姿重新规划,人走动也能跟上。
"""
import math
from dataclasses import dataclass, field

import numpy as np

__all__ = ['TurnPlan', 'TurnaroundPlanner', 'stroke_time', 'reach_time']


def stroke_time(length, vmax, accel, decel):
    """从静止走 length 米再停下的最短时间(梯形速度)。"""
    if length <= 0:
        return 0.0
    ramp = vmax * vmax * (0.5 / accel + 0.5 / decel)
    if length >= ramp:
        return length / vmax + vmax * (0.5 / accel + 0.5 / decel)
    peak = math.sqrt(2.0 * length * accel * decel / (accel + decel))
    return peak / accel + peak / decel


def reach_time(length, vmax, accel):
    """从静止出发、到达 length 米时不必停下的时间。"""
    if length <= 0:
        return 0.0
    ramp = vmax * vmax / (2.0 * accel)
    if length <= ramp:
        return math.sqrt(2.0 * length / accel)
    return (length - ramp) / vmax + vmax / accel


@dataclass
class TurnPlan:
    direction: int = 0            # +1 前进, -1 倒车, 0 无可行动作
    steer: float = 0.0
    stroke_m: float = 0.0         # 第一段(当前要执行的这段)长度
    eta_s: float = math.inf
    reached: bool = False
    final_bearing: float = math.pi
    turn_dir: int = 1
    segments: tuple = ()
    evaluated: int = 0
    first_clear: dict = field(default_factory=dict)


class TurnaroundPlanner:
    def __init__(self, geometry, footprint, *, goal_rad=math.radians(20.0),
                 lengths=(0.3, 0.6, 0.9, 1.3, 1.8), depth=3, step_m=0.10,
                 person_margin_m=0.45, obstacle_margin_m=0.05, first_buffer_m=0.10,
                 speed_mps=1.0, accel_mps2=1.5, decel_mps2=1.0, switch_s=0.7,
                 min_front_m=0.3, view_x_m=0.0):
        self.geo = geometry
        self.front = footprint.front_m
        self.rear = footprint.rear_m
        self.half = footprint.half_width_m + footprint.margin_m
        self.goal = goal_rad
        self.lengths = tuple(sorted(lengths))
        self.depth = depth
        self.step = step_m
        self.person_margin = person_margin_m
        self.obstacle_margin = obstacle_margin_m
        self.first_buffer = first_buffer_m
        self.speed, self.accel, self.decel = speed_mps, accel_mps2, decel_mps2
        self.switch_s = switch_s
        self.min_front = min_front_m
        # 方位角从相机位置量:近处的人从后轴看已在 ±20° 内,从相机看可能还在视野外
        self.view_x = view_x_m
        self.reach_m = 3.5
        tan = math.tan(geometry.max_steer_rad)
        self.radius = geometry.wheelbase_m / tan + 0.5 * geometry.track_m

    # ------------------------------------------------------------------ 几何
    def _arc(self, pose, direction, turn, length):
        """从 pose 出发沿满舵弧线走 0..length,返回 (n, 3) 位姿数组(规划坐标系)。"""
        n = max(1, int(math.ceil(length / self.step)))
        s = np.linspace(0.0, length, n + 1)
        # 倒车打反舵:转角曲率与前进相反,航向变化方向与前进相同(都朝 turn)
        kappa = direction * turn / self.radius
        phi = direction * kappa * s
        lx = np.sin(phi) / kappa
        ly = (1.0 - np.cos(phi)) / kappa
        x0, y0, t0 = pose
        c, si = math.cos(t0), math.sin(t0)
        return np.stack((x0 + c * lx - si * ly, y0 + si * lx + c * ly, t0 + phi), axis=1), s

    def _rect_gap(self, poses, points):
        """车体矩形到点的距离,(n, m)。负数表示点在车身内。"""
        c = np.cos(poses[:, 2])[:, None]
        s = np.sin(poses[:, 2])[:, None]
        dx = points[None, :, 0] - poses[:, 0, None]
        dy = points[None, :, 1] - poses[:, 1, None]
        lx, ly = dx * c + dy * s, -dx * s + dy * c
        ex = np.maximum(-self.rear - lx, lx - self.front)
        ey = np.abs(ly) - self.half
        outside = np.hypot(np.maximum(ex, 0.0), np.maximum(ey, 0.0))
        return np.where((ex > 0) | (ey > 0), outside, np.maximum(ex, ey))

    def _hits(self, poses, points, limits):
        """(n,) 布尔:该位姿下是否有点落进「车体矩形外扩 limit」的方框。

        方角比精确距离略保守(角上最多多算 0.41*limit),但省掉开方,
        在有墙的场景里比 _rect_gap 快一倍多。
        """
        c = np.cos(poses[:, 2])[:, None]
        s = np.sin(poses[:, 2])[:, None]
        dx = points[None, :, 0] - poses[:, 0, None]
        dy = points[None, :, 1] - poses[:, 1, None]
        lx = dx * c + dy * s
        ly = np.abs(dy * c - dx * s)
        lim = limits[None, :]
        return np.any((lx > -self.rear - lim) & (lx < self.front + lim) & (ly < self.half + lim), axis=1)

    @staticmethod
    def _target_in(poses, target):
        c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
        dx, dy = target[0] - poses[:, 0], target[1] - poses[:, 1]
        return dx * c + dy * s, -dx * s + dy * c

    # ------------------------------------------------------------------ 搜索
    def plan(self, target, obstacles=(), *, first_clearance=None, current_speed=0.0,
             prefer_turn=None, turn_hysteresis_s=1.0):
        """target: 车体系 (x, y);obstacles: 车体系点;first_clearance(d, steer) -> 米。"""
        pts = np.asarray(obstacles, dtype=np.float64).reshape(-1, 2)
        if len(pts):
            # 三段最长行程也走不出 3.5m;10cm 一格去重。规划只用来排序,
            # 精确的逐 2cm 扫掠复核在控制层每周期另做。
            tx, ty = target
            keep = ((pts[:, 0] - tx) ** 2 + (pts[:, 1] - ty) ** 2 > 0.35 ** 2)
            keep &= np.sum(pts * pts, axis=1) <= self.reach_m ** 2
            pts = pts[keep]
            if len(pts):
                cells = np.round(pts / 0.10).astype(np.int64)
                _, idx = np.unique(cells, axis=0, return_index=True)
                pts = pts[np.sort(idx)]
        # 已经贴得比余量还近的点(车停在障碍物旁、人站得很近)不能把所有动作
        # 都判死:允许不再靠近,只禁止比起始位置更近。
        origin = np.zeros((1, 3))
        limits = np.minimum(self.obstacle_margin,
                            self._rect_gap(origin, pts)[0] - 0.005) if len(pts) else pts[:, 0]
        person_limit = min(self.person_margin,
                           float(self._rect_gap(origin, np.asarray([target], dtype=np.float64))[0, 0]) - 0.005)
        side = 1 if target[1] >= 0 else -1
        turns = (side, -side) if abs(target[1]) < 0.6 and target[0] < 0 else (side,)
        plans = [self._plan_turn(target, pts, limits, person_limit, t, first_clearance, current_speed)
                 for t in turns]
        def key(p):
            bias = turn_hysteresis_s if prefer_turn is not None and p.turn_dir != prefer_turn else 0.0
            return (not p.reached, p.eta_s + bias if p.reached else p.final_bearing + 0.05 * p.eta_s)
        best = min(plans, key=key)
        best.evaluated = sum(p.evaluated for p in plans)
        return best

    def _plan_turn(self, target, pts, limits, person_limit, turn, first_clearance, current_speed):
        best = TurnPlan(turn_dir=turn)
        fallback = TurnPlan(turn_dir=turn)
        first_clear = {}
        moving = 1 if current_speed > 0.05 else -1 if current_speed < -0.05 else 0
        steer_of = {1: turn * self.geo.max_steer_rad, -1: -turn * self.geo.max_steer_rad}
        evaluated = 0

        def dfs(pose, depth, t_acc, prev_dir, segs):
            nonlocal best, fallback, evaluated
            dirs = (1, -1) if prev_dir == 0 else (-prev_dir,)
            for d in dirs:
                steer = steer_of[d]
                max_len = self.lengths[-1]
                if depth == 0 and first_clearance is not None:
                    clear = float(first_clearance(d, steer))
                    first_clear[d] = clear
                    max_len = min(max_len, clear - self.first_buffer)
                if max_len < 0.10:
                    continue
                if depth == 0:
                    switch = (0.0 if moving in (0, d)
                              else self.switch_s + abs(current_speed) / self.decel)
                else:
                    switch = self.switch_s
                t0 = t_acc + switch
                if best.reached and t0 >= best.eta_s:
                    continue
                poses, s = self._arc(pose, d, turn, max_len)
                evaluated += 1
                bx, by = self._target_in(poses, target)
                person_gap = self._rect_gap(poses, np.asarray([target], dtype=np.float64))[:, 0]
                ok = person_gap >= person_limit
                if len(pts):
                    ok &= ~self._hits(poses, pts, limits)
                bad = np.flatnonzero(~ok)
                valid = int(bad[0]) if len(bad) else len(s)
                if valid <= 1:
                    continue
                valid_len = float(s[valid - 1])
                ahead = bx - self.view_x
                bearing = np.abs(np.arctan2(by, ahead))
                goal = np.flatnonzero((bearing[:valid] <= self.goal) & (ahead[:valid] >= self.min_front))
                if len(goal):
                    reach = float(s[goal[0]])
                    eta = t0 + reach_time(reach, self.speed, self.accel)
                    if not best.reached or eta < best.eta_s:
                        best = TurnPlan(d if not segs else segs[0][0],
                                        steer if not segs else segs[0][1],
                                        reach if not segs else segs[0][2],
                                        eta, True, float(bearing[goal[0]]), turn,
                                        tuple(segs) + ((d, steer, reach),))
                    continue
                cuts = [L for L in self.lengths if L <= valid_len + 1e-9]
                if valid_len >= 0.15 and (not cuts or valid_len - cuts[-1] > 0.1):
                    cuts.append(valid_len)
                for L in cuts:
                    i = min(int(round(L / self.step)), valid - 1)
                    t_here = t0 + stroke_time(L, self.speed, self.accel, self.decel)
                    here = tuple(segs) + ((d, steer, L),)
                    fb = float(bearing[i]) + 0.05 * t_here
                    if not fallback.segments or fb < fallback.final_bearing + 0.05 * fallback.eta_s:
                        fallback = TurnPlan(here[0][0], here[0][1], here[0][2], t_here, False,
                                            float(bearing[i]), turn, here)
                    if depth + 1 < self.depth and not (best.reached and t_here >= best.eta_s):
                        dfs(tuple(poses[i]), depth + 1, t_here, d, here)

        dfs((0.0, 0.0, 0.0), 0, 0.0, 0, ())
        out = best if best.reached else fallback
        out.evaluated = evaluated
        out.first_clear = first_clear
        return out
