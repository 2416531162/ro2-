#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""局部自主脱困与丢人搜索 (Local Recovery)

`person_follower.py` 只负责「看到人 -> 算速度和舵角」。真正让车在走廊、门口、
桌子腿之间活下来的是这一层:它坐在跟随律和底盘之间,有权把跟随指令改掉。

本模块处理四件事,每一件都对应一个实车上会卡死或撞车的具体场景:

1. **过门收舵 (ALIGNING)**
   阿克曼车转弯时车体扫出的圆环比车身宽 (见 footprint._swept_radii)。
   人在门里往旁边偏一点,车本能地打舵去追,恰好在门框里扫出最宽的轨迹,
   于是刮轮子。对策:净空不够就先把舵收直,摆正过门,出门再修方向。

2. **停稳换向 (RECOVERY_BRAKE)**
   轮趣底盘的换向不是软件层面发个负数就行,电机还在正转时直接给反向指令
   会顶电流。任何前进<->后退的切换都必须先测到车真的停了。

3. **限量后退 (RECOVERY_REVERSE)**
   雷达装在车头 (lidar_offset_x = 0.53),车尾是**物理盲区**。
   盲区不是空地 —— 这是倒车脱困最容易出人命的误解。
   所以倒车只允许沿**刚刚走过的前进路径**原路短退:用同一个舵角配负速度,
   阿克曼车会精确地沿原弧线退回去,而那条路 0.5 秒前刚被雷达确认过是空的。
   没有记忆支撑时退的距离另有更紧的上限 (blind_reverse_distance_m),
   并且倒车的次数、总里程、单次时长全部有硬上限,退不出去就认输停下。

4. **丢人搜索 (SEARCH_SCAN / SEARCH_TURN)**
   人刚丢的一瞬间最可能只是检测漏帧或被柱子挡了半秒,此时**原地停住观察**
   比立刻乱转有效得多 —— 乱转会把人转出视野,本来能接上的也接不上了。
   观察超时后才朝最后已知方位做限量弧线搜索。

坐标约定与 footprint.py 一致:原点在后轴中心,x 向前,y 向左。
"""

import math
import time
from dataclasses import dataclass, field

from footprint import (
    VehicleFootprint, SensorMount, scan_to_vehicle_frame, drop_self_hits,
    swept_path_clearance, limit_steer_for_clearance, widest_passable_steer,
)
from motion_safety import clamp, brake_envelope

EPS = 1e-9


# =============================================================================
# 配置
# =============================================================================

@dataclass
class RecoveryConfig:
    """脱困行为参数。`enabled=False` 只关掉倒车与搜索,过门收舵始终生效 ——
    收舵是避障本身的一部分,不是可选的花活。"""

    enabled: bool = True

    # ---- 过门收舵 ----
    align_clearance_m: float = 0.15     # 低于此净空就开始往中间收舵
    align_hold_s: float = 0.30          # 收舵后至少保持这么久,避免左右横跳

    # ---- 被困判定 ----
    stuck_clearance_m: float = 0.12     # 想走但净空低于此值
    stuck_confirm_s: float = 1.00       # 连续这么久走不动才认定被困
    stationary_mps: float = 0.030       # 测到的车速低于此值算停稳
    brake_timeout_s: float = 2.00       # 等停稳的上限,超时就认输

    # ---- 倒车 ----
    reverse_speed_mps: float = 0.12     # 倒车限速,盲区里不能快
    reverse_leg_distance_m: float = 0.45    # 单次后退里程上限
    reverse_leg_timeout_s: float = 6.0      # 单次后退时长上限
    reverse_total_distance_m: float = 1.50  # 全程累计后退里程上限
    reverse_max_legs: int = 3               # 最多退几次
    blind_reverse_distance_m: float = 0.25  # 没有记忆支撑时的后退上限
    rear_stop_clearance_m: float = 0.20     # 车尾净空低于此值立刻停止后退

    # ---- 丢人搜索 ----
    blink_grace_s: float = 0.60         # 这段时间内只减速不动作,等检测自己接上
    scan_hold_s: float = 1.50           # 原地观察时长
    search_speed_mps: float = 0.16      # 弧线搜索限速
    search_steer_frac: float = 0.85     # 搜索用多大比例的满舵
    search_yaw_limit_rad: float = 2.60  # 累计转过这么多弧度还没找到就放弃 (~150°)
    search_timeout_s: float = 10.0

    # ---- 路径记忆 ----
    memory_horizon_s: float = 6.0       # 记忆点的最长寿命
    memory_range_m: float = 5.0         # 超出这个距离的记忆点丢弃
    memory_cell_m: float = 0.05         # 记忆点去重栅格,控制点数上限
    memory_max_points: int = 1200

    # ---- 复位 ----
    settle_s: float = 2.0               # 正常前进这么久后,把脱困计数清零


@dataclass
class RecoveryResult:
    """交回给 person_follower 的最终动作。"""
    state: str          # 面板显示用的状态名
    reason: str         # limit_reason,出问题时一眼看出是谁限的速
    speed: float        # 期望车速 (负数 = 后退)
    steer: float        # 期望前轮转角


# =============================================================================
# 一帧雷达的证据
# =============================================================================

class ScanEvidence:
    """把一帧 LaserScan 变成车体坐标系下的点集,并自带「这帧能不能用」的判断。

    构造函数刻意接受原始 LaserScan 字段而不是 ROS 消息对象,这样测试里不需要
    装 ROS 也能造出证据来。
    """

    def __init__(self, ranges, angle_min, angle_increment, range_min, range_max,
                 mount=None, footprint=None, blind_sectors_deg=(), skin_m=0.05):
        self.points = []
        self.bearings = []
        self.usable = False
        self.self_hits = 0

        if ranges is None or len(ranges) == 0:
            return
        if not (math.isfinite(angle_min) and math.isfinite(angle_increment)):
            return
        if abs(angle_increment) < EPS:
            return

        mount = mount or SensorMount()
        footprint = footprint or VehicleFootprint()
        lo = max(0.0, float(range_min))
        hi = float(range_max) if math.isfinite(range_max) and range_max > lo else 12.0

        bearings = []
        for i, r in enumerate(ranges):
            try:
                r = float(r)
            except (TypeError, ValueError):
                continue
            if not math.isfinite(r) or not (lo <= r <= hi):
                continue
            ang = angle_min + i * angle_increment
            bearings.append((math.atan2(math.sin(ang), math.cos(ang)), r))

        if not bearings:
            return

        raw = scan_to_vehicle_frame(bearings, mount,
                                    blind_sectors_deg=blind_sectors_deg)
        self.points, self.self_hits = drop_self_hits(raw, footprint, skin_m)
        self.bearings = bearings
        # 一帧里连几个有效回波都没有,说明雷达在报废数据,不能当"前方畅通"用。
        self.usable = len(bearings) >= 8

    @classmethod
    def from_points(cls, points, usable=True):
        """直接用车体坐标系下的点构造证据,给仿真和测试用。

        走原始 ranges 那条路要自己算下标和方位角,算错了测试会在一个根本不
        存在的场景上"通过" —— 那比没有测试更糟。这个入口把几何摆在明面上。
        这些点视为已经过滤好的,不再做自反射剔除。
        """
        ev = cls.__new__(cls)
        ev.points = [(float(x), float(y)) for x, y in points]
        ev.bearings = [(math.atan2(y, x), math.hypot(x, y)) for x, y in ev.points]
        ev.self_hits = 0
        ev.usable = bool(usable)
        return ev


# =============================================================================
# 路径记忆
# =============================================================================

class PathMemory:
    """把最近看到的障碍物点随车一起推算,用来照亮车尾的雷达盲区。

    车往前开的时候,现在身后的东西 0.5 秒前还在雷达视野里。把那时候的点按
    自车运动反向推算到当前车体系,倒车时就不是全瞎的。

    这是**推算**不是观测,所以它只有一个用途:给倒车一个「这段路刚才是空的」
    的依据,不能拿它当正向避障的证据。推算误差会随时间累积,因此有寿命上限。
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self._pts = []      # [(x, y, stamp)]

    def clear(self):
        self._pts = []

    def __len__(self):
        return len(self._pts)

    def advance(self, dt, speed, yaw_rate):
        """自车走了 dt 之后,把记忆点搬到新的车体坐标系里。"""
        if not self._pts or dt <= 0.0:
            return
        if abs(speed) < EPS and abs(yaw_rate) < EPS:
            return
        # 后轴中心沿圆弧移动;直行时退化为直线,分开算避免 0 除
        if abs(yaw_rate) > 1e-6:
            radius = speed / yaw_rate
            dtheta = yaw_rate * dt
            dx = radius * math.sin(dtheta)
            dy = radius * (1.0 - math.cos(dtheta))
        else:
            dtheta = 0.0
            dx, dy = speed * dt, 0.0
        cos_t, sin_t = math.cos(-dtheta), math.sin(-dtheta)
        moved = []
        for x, y, stamp in self._pts:
            rx, ry = x - dx, y - dy
            moved.append((rx * cos_t - ry * sin_t, rx * sin_t + ry * cos_t, stamp))
        self._pts = moved

    def add(self, points, now):
        cfg = self.cfg
        cell = max(cfg.memory_cell_m, 0.01)
        horizon = now - cfg.memory_horizon_s
        keep = {}
        for x, y, stamp in self._pts:
            if stamp < horizon:
                continue
            if math.hypot(x, y) > cfg.memory_range_m:
                continue
            keep[(round(x / cell), round(y / cell))] = (x, y, stamp)
        for x, y in points:
            if math.hypot(x, y) > cfg.memory_range_m:
                continue
            keep[(round(x / cell), round(y / cell))] = (x, y, now)
        merged = list(keep.values())
        if len(merged) > cfg.memory_max_points:
            merged.sort(key=lambda p: p[2], reverse=True)
            merged = merged[:cfg.memory_max_points]
        self._pts = merged

    def points(self):
        return [(x, y) for x, y, _ in self._pts]

    def covers_rear(self, min_points=12):
        """身后到底有没有足够的记忆点。没有就只能算盲退。"""
        n = 0
        for x, _y, _s in self._pts:
            if x < 0.0:
                n += 1
                if n >= min_points:
                    return True
        return False


# =============================================================================
# 倒车净空
# =============================================================================

def reverse_clearance(points, footprint, geometry, steer_rad, max_range=8.0):
    """后退时沿实际弧线到第一个障碍物的距离(从车**尾**最后端算起)。

    footprint.py 里的净空函数只算前进。倒车可以化归成前进:把世界绕后轴中心
    转 180° (x,y -> -x,-y),车头车尾互换,问题就变成了同一个前进问题。

    转角符号要跟着翻:前轮打左舵 (steer>0) 前进是左转,后退时车体绕另一边转,
    在翻转后的坐标系里表现为右转。
    """
    mirrored = [(-x, -y) for x, y in points]
    flipped = VehicleFootprint(front_m=footprint.rear_m,
                               rear_m=footprint.front_m,
                               half_width_m=footprint.half_width_m,
                               margin_m=footprint.margin_m)
    return swept_path_clearance(mirrored, flipped, geometry, -steer_rad, max_range)


# =============================================================================
# 主状态机
# =============================================================================

class LocalRecovery:
    """跟随律与底盘之间的监护层。

    `update()` 每个控制周期调用一次,吃进「跟随律想干什么」,吐出「实际准你干
    什么」。它从不放大跟随律的请求,只会收窄 —— 唯一的例外是倒车和搜索,那是
    跟随律自己给不出的动作。
    """

    IDLE, BRAKE, REVERSE, SCAN, TURN, EXHAUSTED = (
        'IDLE', 'BRAKE', 'REVERSE', 'SCAN', 'TURN', 'EXHAUSTED')

    def __init__(self, footprint, geometry, obstacle_profile, config=None):
        self.footprint = footprint
        self.geometry = geometry
        self.profile = obstacle_profile
        self.cfg = config or RecoveryConfig()
        self.memory = PathMemory(self.cfg)

        self.phase = self.IDLE
        self.legs = 0
        self.total_distance = 0.0
        self.blind_used = False
        self.exhausted = False

        self._last_now = None
        self._phase_since = 0.0
        self._leg_distance = 0.0
        self._leg_blind = False
        self._leg_steer = 0.0
        self._stuck_since = None
        self._clear_since = None
        self._align_until = 0.0
        self._align_steer = 0.0
        self._recent_steer = 0.0
        self._search_yaw = 0.0
        self._search_since = 0.0
        self._next_phase = self.IDLE

    # ---------------- 对外只读状态 ----------------

    @property
    def active(self):
        return self.phase in (self.BRAKE, self.REVERSE, self.SCAN, self.TURN)

    @property
    def blind_leg(self):
        """当前这条腿是不是在往雷达盲区里退 —— 只有它为真才准用路径记忆。"""
        return self.phase == self.REVERSE

    # ---------------- 净空查询 ----------------

    def clearance(self, scan, steer, direction=1, actual_steer=None,
                  allow_history=False, max_range=8.0):
        """沿当前行驶方向的可行距离。

        `steer` 是打算走的弧,`actual_steer` 是舵机实际所在的位置 —— 两者不同
        时取更保守的那个,因为舵还在转的过程中车走的是中间那条路。
        """
        if scan is None or not getattr(scan, 'usable', False):
            return 0.0
        points = list(scan.points)
        if allow_history:
            points.extend(self.memory.points())

        arcs = [steer]
        if actual_steer is not None and abs(actual_steer - steer) > 1e-4:
            arcs.append(actual_steer)

        best = max_range
        for arc in arcs:
            if direction >= 0:
                c = swept_path_clearance(points, self.footprint, self.geometry,
                                         arc, max_range)
            else:
                c = reverse_clearance(points, self.footprint, self.geometry,
                                      arc, max_range)
            best = min(best, c)
        return best

    # ---------------- 主循环 ----------------

    def update(self, now, scan, healthy, speed, yaw_rate, target, gap, bearing,
               requested_speed, requested_steer, current_steer, follow_cap,
               lost_age):
        cfg = self.cfg
        dt = 0.0 if self._last_now is None else max(0.0, min(now - self._last_now, 0.25))
        self._last_now = now

        # 记忆先随车走,再吸收这一帧 —— 顺序反了会把新点也一起搬错位置
        self.memory.advance(dt, speed, yaw_rate)
        usable = scan is not None and getattr(scan, 'usable', False)
        if usable:
            self.memory.add(scan.points, now)

        if self.phase == self.REVERSE:
            self._leg_distance += abs(speed) * dt
            self.total_distance += abs(speed) * dt
        if self.phase == self.TURN:
            self._search_yaw += abs(yaw_rate) * dt

        # 传感器或底盘不健康时,任何自主动作都必须立刻放弃。
        # 计数不清零:刚才没退出去这件事仍然成立。
        if not healthy or not usable:
            if self.active:
                self._enter(self.IDLE, now)
            return RecoveryResult('RECOVERY_WAIT', 'unhealthy', 0.0, current_steer)

        if speed > cfg.stationary_mps:
            self._recent_steer = current_steer      # 只记前进时的舵,倒车要按它原路退

        stationary = abs(speed) <= cfg.stationary_mps

        if self.phase == self.BRAKE:
            return self._do_brake(now, stationary, current_steer)
        if self.phase == self.REVERSE:
            return self._do_reverse(now, scan, current_steer)
        if self.phase == self.SCAN:
            return self._do_scan(now, target, current_steer)
        if self.phase == self.TURN:
            return self._do_turn(now, scan, target, current_steer)

        return self._do_follow(now, scan, target, gap, bearing, requested_speed,
                               requested_steer, current_steer, follow_cap,
                               lost_age, stationary, speed)

    # ---------------- 正常跟随 ----------------

    def _do_follow(self, now, scan, target, gap, bearing, requested_speed,
                   requested_steer, current_steer, follow_cap, lost_age,
                   stationary, speed):
        cfg = self.cfg
        points = scan.points

        # --- 过门收舵 ---
        steer, clear = limit_steer_for_clearance(
            points, self.footprint, self.geometry, requested_steer,
            cfg.align_clearance_m)
        aligning = abs(steer - requested_steer) > 1e-4
        if aligning:
            self._align_until = now + cfg.align_hold_s
            self._align_steer = abs(steer)
        elif now < self._align_until:
            # 收舵后短暂保持上限,否则净空一好转就立刻打回去,在门框里左右横跳。
            # 只压幅值不改方向:人往哪边走仍然跟得上,只是不准再打大舵。
            limit = self._align_steer
            if abs(steer) > limit:
                steer = math.copysign(limit, steer)
                aligning = True

        speed_cap = min(requested_speed, brake_envelope(clear, self.profile))
        speed_out = max(0.0, speed_cap)

        # --- 被困判定 ---
        wants_to_move = requested_speed > cfg.stationary_mps
        blocked = wants_to_move and clear < cfg.stuck_clearance_m
        if blocked:
            if self._stuck_since is None:
                self._stuck_since = now
            self._clear_since = None
        else:
            self._stuck_since = None
            if speed > cfg.stationary_mps:
                if self._clear_since is None:
                    self._clear_since = now
                elif now - self._clear_since > cfg.settle_s:
                    self._reset_budget()        # 顺利走了一段,脱困预算恢复

        if (blocked and cfg.enabled and not self.exhausted
                and now - self._stuck_since >= cfg.stuck_confirm_s):
            if self.legs >= cfg.reverse_max_legs or \
               self.total_distance >= cfg.reverse_total_distance_m:
                self.exhausted = True
            else:
                self._next_phase = self.REVERSE
                self._enter(self.BRAKE, now)
                return RecoveryResult('RECOVERY_BRAKE', 'wait_stationary',
                                      0.0, current_steer)

        # --- 目标状态 ---
        if target:
            if self.exhausted and blocked:
                return RecoveryResult('RECOVERY_EXHAUSTED', 'no_way_out',
                                      0.0, steer)
            if aligning:
                return RecoveryResult('ALIGNING', 'steer_clearance', speed_out, steer)
            if speed_out <= cfg.stationary_mps:
                reason = 'in_deadband' if requested_speed <= cfg.stationary_mps \
                    else 'obstacle_envelope'
                return RecoveryResult('HOLDING', reason, 0.0, steer)
            return RecoveryResult('TRACKING', 'follow_cap', speed_out, steer)

        # --- 目标丢了 ---
        if lost_age <= cfg.blink_grace_s:
            # 只是闪断。停住等,别动 —— 一动就可能把人转出视野
            return RecoveryResult('TARGET_BLINK', 'target_blink', 0.0, current_steer)
        if cfg.enabled and not self.exhausted:
            self._enter(self.SCAN, now)
            return RecoveryResult('SEARCH_SCAN', 'observe', 0.0, current_steer)
        return RecoveryResult('SEARCHING_LOST', 'target_lost', 0.0, current_steer)

    # ---------------- 停稳换向 ----------------

    def _do_brake(self, now, stationary, current_steer):
        if stationary:
            nxt = self._next_phase
            self._next_phase = self.IDLE
            if nxt == self.REVERSE:
                self._begin_reverse(now)
                return RecoveryResult('RECOVERY_REVERSE', 'reverse_leg',
                                      -self.cfg.reverse_speed_mps, self._leg_steer)
            self._enter(self.IDLE, now)
            return RecoveryResult('RECOVERY_WAIT', 'gear_change', 0.0, current_steer)
        if now - self._phase_since > self.cfg.brake_timeout_s:
            # 发了几秒 0 还没停,底盘要么在坡上要么反馈坏了,不能再换向
            self.exhausted = True
            self._enter(self.IDLE, now)
            return RecoveryResult('RECOVERY_EXHAUSTED', 'brake_timeout', 0.0,
                                  current_steer)
        return RecoveryResult('RECOVERY_BRAKE', 'wait_stationary', 0.0, current_steer)

    def _begin_reverse(self, now):
        cfg = self.cfg
        self.legs += 1
        self._leg_distance = 0.0
        # 原路退回:用刚才前进时的舵角,阿克曼车会精确沿原弧线倒回去
        self._leg_steer = clamp(self._recent_steer,
                                -self.geometry.max_steer_rad,
                                self.geometry.max_steer_rad)
        self._leg_blind = not self.memory.covers_rear()
        if self._leg_blind:
            self.blind_used = True
        self._enter(self.REVERSE, now)

    def _do_reverse(self, now, scan, current_steer):
        cfg = self.cfg
        budget = cfg.blind_reverse_distance_m if self._leg_blind \
            else cfg.reverse_leg_distance_m
        budget = min(budget, max(0.0, cfg.reverse_total_distance_m - self.total_distance))

        rear = self.clearance(scan, self._leg_steer, direction=-1,
                              actual_steer=current_steer, allow_history=True)

        if self._leg_distance >= budget:
            reason = 'reverse_budget'
        elif rear < cfg.rear_stop_clearance_m:
            reason = 'rear_blocked'
        elif now - self._phase_since > cfg.reverse_leg_timeout_s:
            reason = 'reverse_timeout'
        else:
            remaining = budget - self._leg_distance
            speed = -min(cfg.reverse_speed_mps,
                         brake_envelope(rear - cfg.rear_stop_clearance_m, self.profile),
                         max(0.0, remaining))
            return RecoveryResult('RECOVERY_REVERSE', 'reverse_leg',
                                  speed, self._leg_steer)

        # 这条腿走完了:先停稳,再回到正常跟随重新评估
        self._next_phase = self.IDLE
        self._enter(self.BRAKE, now)
        if self.legs >= cfg.reverse_max_legs or \
           self.total_distance >= cfg.reverse_total_distance_m:
            self.exhausted = True
        return RecoveryResult('RECOVERY_BRAKE', reason, 0.0, current_steer)

    # ---------------- 丢人搜索 ----------------

    def _do_scan(self, now, target, current_steer):
        if target:
            self._enter(self.IDLE, now)
            return RecoveryResult('TRACKING', 'reacquired', 0.0, current_steer)
        if now - self._phase_since < self.cfg.scan_hold_s:
            return RecoveryResult('SEARCH_SCAN', 'observe', 0.0, current_steer)
        self._search_yaw = 0.0
        self._search_since = now
        self._enter(self.TURN, now)
        return RecoveryResult('SEARCH_TURN', 'search_turn', 0.0, current_steer)

    def _do_turn(self, now, scan, target, current_steer):
        cfg = self.cfg
        if target:
            self._enter(self.IDLE, now)
            return RecoveryResult('TRACKING', 'reacquired', 0.0, current_steer)
        if (self._search_yaw >= cfg.search_yaw_limit_rad
                or now - self._search_since > cfg.search_timeout_s):
            self._enter(self.IDLE, now)
            return RecoveryResult('SEARCHING_LOST', 'search_exhausted', 0.0,
                                  current_steer)

        steer = math.copysign(cfg.search_steer_frac * self.geometry.max_steer_rad,
                              self._search_sign())
        clear = self.clearance(scan, steer, direction=1, actual_steer=current_steer)
        if clear < cfg.align_clearance_m:
            # 想转的方向被挡住了,换一条前方最空的弧继续转,而不是硬顶着墙
            steer, clear = widest_passable_steer(scan.points, self.footprint,
                                                 self.geometry)
        speed = min(cfg.search_speed_mps, brake_envelope(clear, self.profile))
        return RecoveryResult('SEARCH_TURN', 'search_turn', max(0.0, speed), steer)

    def _search_sign(self):
        # 朝人最后出现的那一侧转;没记录就默认左转
        return 1.0 if self._recent_steer >= 0.0 else -1.0

    # ---------------- 内部 ----------------

    def _enter(self, phase, now):
        self.phase = phase
        self._phase_since = now
        if phase == self.IDLE:
            self._stuck_since = None

    def _reset_budget(self):
        self.legs = 0
        self.total_distance = 0.0
        self.blind_used = False
        self.exhausted = False
        self._clear_since = None
