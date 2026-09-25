#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跟随闭环仿真:真实 FollowerEngine + 按实车时序合成的相机/雷达/里程计/底盘。

只用于离线比较算法改动,不代表实车验收。与 tests/test_follower_node.py 的
WorldSim 相比,这里按实车链路补上了会影响"灵敏度"的时序:

- 相机 30 FPS,采集到发布延迟 70 ms,消息带采集时间戳;58° 水平视场;
- N10P 10 Hz、每圈约 450 个采样,扫描结束 15 ms 后到达;
- 人走路时两条腿前后摆动,雷达看到的是两个 6 cm 的圆;
- /odom 50 Hz;控制 20 Hz;指令经仲裁/串口/固件约 60 ms 才生效;
- 底盘按驱动斜坡加速,按实测减速度刹车,舵机有转角速率上限。

    python3 radar_system/tools/follow_sim.py                # 全部场景
    python3 radar_system/tools/follow_sim.py walk_behind --trace
"""
import argparse
import json
import math
import os
import random
import sys
from collections import deque
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

from runtime_config import PROFILE  # noqa: E402
from robot_core.contracts import ScanFrame, LocalPose  # noqa: E402
from robot_core.kinematics import steer_from_yaw, yaw_from_steer, ChassisGeometry  # noqa: E402
from follower_config import build_config  # noqa: E402
from follower_engine import FollowerEngine  # noqa: E402

BINS = 720
DT = 0.01
GEOM = PROFILE['geometry']
SENS = PROFILE['sensors']


# =============================================================================
# 世界:房间墙面 + 圆柱障碍 + 行人
# =============================================================================

def ray_circle(ox, oy, ux, uy, cx, cy, r):
    dx, dy = cx - ox, cy - oy
    along = dx * ux + dy * uy
    if along <= 0:
        return math.inf
    perp2 = dx * dx + dy * dy - along * along
    if perp2 > r * r:
        return math.inf
    return along - math.sqrt(r * r - perp2)


def ray_room(ox, oy, ux, uy, room):
    x0, y0, x1, y1 = room
    best = math.inf
    if ux > 1e-9:
        best = min(best, (x1 - ox) / ux)
    elif ux < -1e-9:
        best = min(best, (x0 - ox) / ux)
    if uy > 1e-9:
        best = min(best, (y1 - oy) / uy)
    elif uy < -1e-9:
        best = min(best, (y0 - oy) / uy)
    return best


class Walker:
    """按路径点匀速行走的人。waypoints: [(t, x, y)],之间线性插值。"""

    def __init__(self, waypoints, leg_gap=0.20, step_hz=1.8, stride=0.30):
        self.wp = waypoints
        self.leg_gap, self.step_hz, self.stride = leg_gap, step_hz, stride

    def state(self, t):
        wp = self.wp
        if t <= wp[0][0]:
            return wp[0][1], wp[0][2], 0.0, 0.0
        for (t0, x0, y0), (t1, x1, y1) in zip(wp, wp[1:]):
            if t <= t1:
                a = (t - t0) / max(t1 - t0, 1e-9)
                vx, vy = (x1 - x0) / max(t1 - t0, 1e-9), (y1 - y0) / max(t1 - t0, 1e-9)
                return x0 + a * (x1 - x0), y0 + a * (y1 - y0), vx, vy
        return wp[-1][1], wp[-1][2], 0.0, 0.0

    def legs(self, t):
        x, y, vx, vy = self.state(t)
        speed = math.hypot(vx, vy)
        hx, hy = (vx / speed, vy / speed) if speed > 0.05 else (1.0, 0.0)
        nx, ny = -hy, hx
        swing = (0.5 * self.stride * math.sin(2 * math.pi * self.step_hz * t)
                 if speed > 0.05 else 0.0)
        g = self.leg_gap / 2
        return [(x + nx * g + hx * swing, y + ny * g + hy * swing),
                (x - nx * g - hx * swing, y - ny * g - hy * swing)]


class World:
    def __init__(self, walkers, room=(-6.0, -5.0, 10.0, 5.0), posts=()):
        self.walkers = walkers
        self.room = room
        self.posts = list(posts)

    def lidar_range(self, t, ox, oy, heading):
        ux, uy = math.cos(heading), math.sin(heading)
        best = ray_room(ox, oy, ux, uy, self.room)
        for w in self.walkers:
            for lx, ly in w.legs(t):
                best = min(best, ray_circle(ox, oy, ux, uy, lx, ly, 0.06))
        for px, py, r in self.posts:
            best = min(best, ray_circle(ox, oy, ux, uy, px, py, r))
        return best


# =============================================================================
# 底盘:指令延迟 + 驱动斜坡 + 物理刹车 + 舵机速率
# =============================================================================

class Chassis:
    def __init__(self, delay_s=0.06, accel=PROFILE['driver']['acceleration_m_s2'],
                 decel=1.5, servo_radps=3.0, tau_s=0.10, trim_rad=0.0, steer_quant_rad=0.0):
        self.geo = ChassisGeometry()
        # 舵机零位偏差（打直时车轮实际偏的角度）和舵机分辨率，走直线仿真用
        self.trim, self.quant = trim_rad, steer_quant_rad
        self.x = self.y = self.yaw = 0.0
        self.v = 0.0
        self.steer = 0.0
        self.delay_s, self.accel, self.decel = delay_s, accel, decel
        self.servo, self.tau = servo_radps, tau_s
        self.queue = deque()
        self.cmd = (0.0, 0.0)
        self.driver_v = 0.0
        self.wz = 0.0

    def command(self, t, vx, wz):
        self.queue.append((t + self.delay_s, vx, wz))

    def step(self, t, dt):
        while self.queue and self.queue[0][0] <= t:
            _, vx, wz = self.queue.popleft()
            self.cmd = (vx, wz)
        vx, wz = self.cmd
        # 驱动:零/换向立即清零,否则按加速度斜坡
        if vx == 0 or self.driver_v * vx < 0:
            self.driver_v = 0.0
        else:
            step = PROFILE['driver']['acceleration_m_s2'] * dt
            self.driver_v += max(-step, min(step, vx - self.driver_v))
        if abs(self.driver_v) > 0.02:
            target_steer = steer_from_yaw(self.driver_v, wz, self.geo)
        else:
            # 速度为 0 时 Twist 协议无法表达转角,固件保持当前舵角
            target_steer = self.steer
        ds = self.servo * dt
        self.steer += max(-ds, min(ds, target_steer - self.steer))
        # 电机:一阶响应,加速/刹车能力分别受限
        want = self.driver_v
        dv = (want - self.v) * min(1.0, dt / self.tau)
        speeding_up = abs(want) > abs(self.v) and want * self.v >= 0
        lim = (self.accel if speeding_up else self.decel) * dt
        self.v += max(-lim, min(lim, dv))
        if abs(self.v) < 1e-4 and want == 0:
            self.v = 0.0
        wheel = self.steer
        if self.quant > 0:
            wheel = round(wheel / self.quant) * self.quant
        self.wz = yaw_from_steer(self.v, wheel + self.trim, self.geo)
        self.x += self.v * dt * math.cos(self.yaw + self.wz * dt / 2)
        self.y += self.v * dt * math.sin(self.yaw + self.wz * dt / 2)
        self.yaw += self.wz * dt

    def to_body(self, wx, wy):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = wx - self.x, wy - self.y
        return c * dx + s * dy, -s * dx + c * dy

    def to_world(self, bx, by):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + c * bx - s * by, self.y + s * bx + c * by


def rect_distance(bx, by):
    """车体系点到车体矩形的距离,负数表示在车身内。"""
    ex = max(-GEOM['rear_m'] - bx, bx - GEOM['front_m'])
    ey = abs(by) - GEOM['half_width_m']
    if ex <= 0 and ey <= 0:
        return max(ex, ey)
    return math.hypot(max(ex, 0.0), max(ey, 0.0))


# =============================================================================
# 传感器合成
# =============================================================================

def make_scan(world, car, t, rng, samples=450):
    lx, ly = car.to_world(SENS['lidar_x_m'], SENS['lidar_y_m'])
    heading0 = car.yaw + SENS['lidar_yaw_rad']
    ranges = [math.inf] * BINS
    sampled = [False] * BINS
    phase = rng.uniform(0, 360.0 / samples)
    for k in range(samples):
        ang = (phase + k * 360.0 / samples) % 360.0
        key = round(((360.0 - ang) % 360.0) * 2) % BINS
        a = key * 2 * math.pi / BINS
        r = world.lidar_range(t, lx, ly, heading0 + a)
        sampled[key] = True
        if 0.15 <= r <= 12.0:
            ranges[key] = r + rng.gauss(0, 0.01)
    return ranges, sampled


def make_detection(world, car, t, rng, walker, torso_above_cam=0.40):
    x, y, _, _ = walker.state(t)
    bx, by = car.to_body(x, y)
    cx, cy, cyaw = SENS['camera_x_m'], SENS['camera_y_m'], SENS['camera_yaw_rad']
    dx, dy = bx - cx, by - cy
    c, s = math.cos(-cyaw), math.sin(-cyaw)
    hx, hy = c * dx - s * dy, s * dx + c * dy
    dist = math.hypot(hx, hy)
    if hx <= 0 or not 0.55 <= dist <= 5.5:
        return None
    if abs(math.atan2(hy, hx)) > math.radians(29.0):
        return None
    # 深度量到的是身体前表面,并带随距离增大的噪声
    surface = max(0.3, dist - 0.08 + rng.gauss(0, 0.015 + 0.01 * dist))
    scale = surface / dist
    hx, hy = hx * scale, hy * scale
    pitch = SENS['camera_pitch_rad']
    cp, sp = math.cos(pitch), math.sin(pitch)
    zv = torso_above_cam
    Z = hx * cp - zv * sp
    Y = -hx * sp - zv * cp
    X = -hy + rng.gauss(0, 0.01)
    return {'label': 'person', 'conf': round(rng.uniform(0.6, 0.92), 2),
            'x': round(X, 3), 'y': round(Y, 3), 'z': round(Z, 3),
            'distance': round(math.sqrt(X * X + Y * Y + Z * Z), 3),
            'bearing_rad': round(math.atan2(-X, max(Z, 0.05)), 5),
            'depth_ratio': 0.8, 'range_valid': True, 'stamp': round(t, 4)}


# =============================================================================
# 仿真主循环
# =============================================================================

def make_config(**overrides):
    args = SimpleNamespace(controller='pure-pursuit', no_recovery=False,
                           no_turnaround=False, pre_steer=False, rear_blind=False,
                           safe_mode=False)
    cfg = build_config(args)
    for k, v in overrides.items():
        if not hasattr(cfg, k):
            raise AttributeError(k)
        setattr(cfg, k, v)
    cfg.__post_init__()
    return cfg


class Sim:
    def __init__(self, world, target, cfg=None, seed=1, cam_latency=0.07,
                 cam_miss=0.03, chassis=None):
        self.world, self.target = world, target
        self.rng = random.Random(seed)
        self.t = 0.0
        self.car = chassis or Chassis()
        self.cfg = cfg or make_config()
        self.engine = FollowerEngine(self.cfg, now=lambda: self.t, ros_time=lambda: self.t,
                                     dry_run=True, simulated_odometry=False)
        self.engine.print_dashboard = lambda *_: None
        self.cam_latency, self.cam_miss = cam_latency, cam_miss
        self.pending = deque()
        self.next = dict(odom=0.0, driver=0.0, scan=0.1, cam=0.0, ctrl=0.05)
        self.samples = []
        self.control_hook = None      # 演示服务器用:替换每个控制周期的 step+下发
        self.last_scan = None
        self.last_cam = []

    def _deliver(self, due, kind, payload):
        self.pending.append((due, kind, payload))

    def run(self, duration):
        e = self.engine
        end = self.t + duration
        while self.t < end - 1e-9:
            self.t = round(self.t + DT, 6)
            t = self.t
            self.car.step(t, DT)
            if t >= self.next['odom']:
                self.next['odom'] += 0.02
                e.observe_pose(LocalPose(t, self.car.x, self.car.y, self.car.yaw))
            if t >= self.next['driver']:
                self.next['driver'] += 0.04
                e.observe_driver({'armed': True, 'ready': 'ready', 'connected': True,
                                  'holding': False, 'age_ms': 5,
                                  'telemetry': {'velocity': [self.car.v, 0.0, self.car.wz]}})
            if t >= self.next['scan']:
                self.next['scan'] += 0.1
                ranges, sampled = make_scan(self.world, self.car, t, self.rng)
                self._deliver(t + 0.015, 'scan', ScanFrame(ranges, 0.0, 2 * math.pi / BINS,
                                                           0.15, 12.0, t, sampled))
            if t >= self.next['cam']:
                self.next['cam'] += 1.0 / 30
                items = []
                for w in self.world.walkers:
                    if self.rng.random() < self.cam_miss:
                        continue
                    det = make_detection(self.world, self.car, t, self.rng, w)
                    if det is not None:
                        items.append(det)
                self._deliver(t + self.cam_latency, 'cam', items)
            while self.pending and self.pending[0][0] <= t:
                _, kind, payload = self.pending.popleft()
                if kind == 'scan':
                    e.observe_scan(payload)
                    self.last_scan = payload
                else:
                    e.observe_targets(payload)
                    self.last_cam = payload
            if t >= self.next['ctrl']:
                self.next['ctrl'] += e.dt
                if self.control_hook is not None:
                    self.control_hook(t)
                else:
                    e.step()
                    self.car.command(t, e.cmd_vx, e.cmd_wz)
                self._sample()
        return self

    def _sample(self):
        e, car = self.engine, self.car
        px, py, pvx, pvy = self.target.state(self.t)
        bx, by = car.to_body(px, py)
        others = [rect_distance(*car.to_body(*w.state(self.t)[:2])) - 0.20
                  for w in self.world.walkers]
        x0, y0, x1, y1 = self.world.room
        wall = min(rect_distance(*car.to_body(cx, cy))
                   for cx, cy in self._wall_probe(car, x0, y0, x1, y1))
        view = e.view
        self.samples.append(dict(
            t=round(self.t, 3), state=e.state, reason=e.limit_reason,
            v=round(car.v, 3), cmd_vx=round(e.cmd_vx, 3), steer=round(car.steer, 3),
            car=(round(car.x, 3), round(car.y, 3), round(car.yaw, 3)),
            person=(round(bx, 3), round(by, 3)),
            person_speed=round(math.hypot(pvx, pvy), 2),
            gap=round(max(bx - GEOM['front_m'], 0.0) if bx > 0 else -1.0, 3),
            bearing=math.degrees(math.atan2(by, bx)),
            clearance=round(min(others), 3), wall=round(wall, 3),
            tracked=view is not None, source=view['source'] if view else None,
            track_id=e.people.target_id,
            est=((round(view['x'], 2), round(view['y'], 2)) if view else None),
            phase=e.turnaround_phase))

    @staticmethod
    def _wall_probe(car, x0, y0, x1, y1):
        # 车身附近墙面上最近的几个点,够判断蹭墙
        out = []
        for bx in (-0.1, 0.3, 0.7):
            for by in (-0.35, 0.35):
                wx, wy = car.to_world(bx, by)
                out += [(x0, wy), (x1, wy), (wx, y0), (wx, y1)]
        return out


# =============================================================================
# 场景
# =============================================================================

def front_start(d=1.6):
    return GEOM['front_m'] + d


def sc_walk_away(speed=1.2):
    """人站在车前 1.0m(期望间距),2s 后以 speed 直行 6.5m 后站住。

    房间各方向都在 N10P 12m 量程以内,避免把"无回波"当成场景特征。
    """
    x0 = front_start(1.0)
    walk = 6.5 / speed
    w = Walker([(0, x0, 0), (2.0, x0, 0), (2.0 + walk, x0 + 6.5, 0), (30, x0 + 6.5, 0)])
    return World([w], room=(-2, -4, 10.5, 4)), w, 2.0 + walk + 4.0, 2.0


def sc_exit_fov():
    """人从车前 1.8m 横着往左走出相机视野,继续斜向左前方走,之后站住。"""
    x0 = front_start(1.8)
    w = Walker([(0, x0, 0), (2.0, x0, 0), (3.6, x0, 1.6), (6.0, x0 + 1.6, 3.2), (12, x0 + 1.6, 3.2)])
    return World([w], room=(-6, -5, 10, 5)), w, 11.0, 2.0


def sc_walk_behind(side=1):
    """人从车前绕到车侧再到车后 2m 站住:车要掉头面对人。"""
    x0 = front_start(1.2)
    pts = [(0, x0, 0), (2.0, x0, 0), (3.2, 0.9, side * 1.3), (4.4, -0.6, side * 1.4),
           (5.6, -2.0, side * 0.3), (16, -2.0, side * 0.3)]
    w = Walker(pts)
    return World([w], room=(-6, -4, 8, 4)), w, 16.0, 2.0


def sc_behind_walk_away():
    """人绕到车后后继续往车后方向走远:掉头后要接着跟。"""
    x0 = front_start(1.2)
    pts = [(0, x0, 0), (2.0, x0, 0), (3.2, 0.9, 1.3), (4.4, -0.6, 1.4),
           (5.6, -2.0, 0.3), (10.5, -6.5, 0.3), (18, -6.5, 0.3)]
    w = Walker(pts)
    return World([w], room=(-9, -4, 6, 4)), w, 18.0, 2.0


def sc_sudden_stop(speed=1.0):
    """人以 speed 走 5s 后突然站住:看刹车是否冲过停车线。"""
    x0 = front_start(1.0)
    w = Walker([(0, x0, 0), (2.0, x0, 0), (7.0, x0 + 5 * speed, 0), (30, x0 + 5 * speed, 0)])
    return World([w], room=(-2, -4, 10.5, 4)), w, 11.0, 7.0


def sc_distractor():
    """目标走出视野改由雷达跟踪时,另一人从旁边经过。"""
    x0 = front_start(1.8)
    target = Walker([(0, x0, 0), (2.0, x0, 0), (3.6, x0, 1.7), (9.0, x0, 1.7), (14, x0, 1.7)])
    other = Walker([(0, 6.0, -3.0), (3.4, 6.0, -3.0), (6.4, x0 + 0.9, 3.5), (14, x0 + 0.9, 3.5)])
    return World([target, other]), target, 12.0, 2.0


def sc_clutter_handoff():
    """人走出相机视野后从一排椅子腿/桌腿旁边经过:雷达轨迹不能跳到椅子腿上。"""
    x0 = front_start(1.6)
    w = Walker([(0, x0, 0), (2.0, x0, 0), (3.4, x0, 1.4), (6.4, x0 + 3.0, 1.4), (14, x0 + 3.0, 1.4)])
    posts = [(x0 + 0.4 + 0.45 * i, 1.4 + dy, 0.025) for i in range(6) for dy in (0.55, 1.0)]
    posts += [(x0 + 1.0, 0.75, 0.03), (x0 + 1.45, 0.75, 0.03)]
    return World([w], room=(-3, -3, 9, 4), posts=posts), w, 11.0, 2.0


def sc_chair_row():
    """人贴着一排椅子腿(0.25m)走,再在椅子旁站住:腿部点簇会和椅子腿配对。"""
    x0 = front_start(1.6)
    w = Walker([(0, x0, 0), (2.0, x0, 0), (3.4, x0, 1.4), (6.4, x0 + 3.0, 1.4),
                (9.0, x0 + 3.0, 1.4), (10.5, x0 + 3.0, 0.2), (30, x0 + 3.0, 0.2)])
    posts = [(x0 + 0.3 + 0.45 * i, 1.65, 0.02) for i in range(8)]
    posts += [(x0 + 0.3 + 0.45 * i, 1.10, 0.02) for i in range(0, 8, 2)]
    return World([w], room=(-3, -3, 9, 4), posts=posts), w, 15.0, 2.0


SCENARIOS = {
    'walk_away_1.0': lambda: sc_walk_away(1.0),
    'walk_away_1.3': lambda: sc_walk_away(1.3),
    'exit_fov': sc_exit_fov,
    'walk_behind_left': lambda: sc_walk_behind(1),
    'walk_behind_right': lambda: sc_walk_behind(-1),
    'behind_walk_away': sc_behind_walk_away,
    'sudden_stop': sc_sudden_stop,
    'distractor': sc_distractor,
    'clutter_handoff': sc_clutter_handoff,
    'chair_row': sc_chair_row,
}


def metrics(sim, start):
    s = [x for x in sim.samples if x['t'] >= start]
    first_id = next((x['track_id'] for x in sim.samples if x['tracked']), None)
    moving = [x for x in s if x['person_speed'] > 0.05]
    react = next((x['t'] - start for x in s if abs(x['v']) > 0.10), None)
    lost = sum(1 for a, b in zip(s, s[1:]) if a['tracked'] and not b['tracked'])
    front_gaps = [x['gap'] for x in s if x['gap'] >= 0 and x['person'][0] > 0]
    behind_since = None
    turn_done = None
    for x in s:
        if behind_since is None and x['person'][0] < 0:
            behind_since = x['t']
        if behind_since is not None and turn_done is None:
            if x['person'][0] > 0.5 and x['source'] == 'camera':
                turn_done = x['t'] - behind_since
    blind, run = 0.0, 0.0
    for a, b in zip(s, s[1:]):
        run = run + (b['t'] - a['t']) if b['source'] != 'camera' else 0.0
        blind = max(blind, run)
    errs = [math.hypot(x['est'][0] - x['person'][0], x['est'][1] - x['person'][1])
            for x in s if x['est'] is not None]
    # 画龙程度:实际前轮转角变化率的均方根 (rad/s)
    rates = [(b['steer'] - a['steer']) / max(b['t'] - a['t'], 1e-6) for a, b in zip(s, s[1:])]
    steer_rms = math.sqrt(sum(r * r for r in rates) / len(rates)) if rates else 0.0
    tail = s[-20:]
    return dict(
        tracked=round(sum(x['tracked'] for x in s) / max(len(s), 1), 3),
        lost_events=lost,
        same_id=all(x['track_id'] in (None, first_id) for x in s),
        lidar_frac=round(sum(x['source'] == 'lidar' for x in s) / max(len(s), 1), 3),
        react_s=None if react is None else round(react, 2),
        max_gap=round(max(front_gaps), 2) if front_gaps else None,
        min_person_clear=round(min(x['clearance'] for x in s), 2),
        min_wall=round(min(x['wall'] for x in s), 2),
        max_v=round(max(x['v'] for x in s), 2),
        max_rev=round(-min(min(x['v'] for x in s), 0.0), 2),
        turnaround_s=None if turn_done is None else round(turn_done, 2),
        max_cam_blind_s=round(blind, 2),
        max_track_err=round(max(errs), 2) if errs else None,
        steer_rate_rms=round(steer_rms, 3),
        end_gap=round(sum(x['gap'] for x in tail) / len(tail), 2),
        end_bearing=round(sum(abs(x['bearing']) for x in tail) / len(tail), 1),
        end_state=s[-1]['state'],
        moving_frac=round(len(moving) / max(len(s), 1), 2),
    )


def run_scenario(name, cfg_overrides=None, seed=1, **sim_kw):
    world, target, duration, start = SCENARIOS[name]()
    sim = Sim(world, target, make_config(**(cfg_overrides or {})), seed=seed, **sim_kw)
    sim.run(duration)
    return sim, metrics(sim, start)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('names', nargs='*', default=list(SCENARIOS))
    p.add_argument('--set', action='append', default=[], metavar='KEY=VALUE',
                   help='覆盖 FollowerConfig 字段,例如 --set max_speed_mps=1.0')
    p.add_argument('--seed', type=int, default=1)
    p.add_argument('--trace', action='store_true', help='逐 0.25s 打印轨迹')
    p.add_argument('--json', action='store_true')
    args = p.parse_args()
    overrides = {}
    for item in args.set:
        key, value = item.split('=', 1)
        overrides[key] = json.loads(value)
    out = {}
    for name in args.names:
        sim, m = run_scenario(name, overrides, seed=args.seed)
        out[name] = m
        if args.trace:
            for x in sim.samples[::5]:
                print(json.dumps(x, ensure_ascii=False))
        if not args.json:
            print(f"{name:20s} " + ' '.join(f"{k}={v}" for k, v in m.items()))
    if args.json:
        print(json.dumps(out, indent=1, ensure_ascii=False))


if __name__ == '__main__':
    main()
