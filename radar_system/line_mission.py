#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RTK 走直线 / 走路线任务：操作流程 + 融合输入转换 + 运行记录。纯逻辑，不依赖 ROS（节点见 line_node.py）。

直线：
    mark A / mark B   把车开到位置上记点（存经纬度，融合重设原点也不受影响）
    start             检查起步条件 → 申请 NAVIGATION → 仲裁切过来后开始走
路线（app 在地图上画的路点折线或圆，几何见 route_plan.py）：
    route_set         保存路线（app 按当前本地坐标发，车上存经纬度，重启换原点后照样对得上）
    route_start       可带 route（先保存再开始）：从车当前位姿规划接入段 → 申请 NAVIGATION → 跑完停在终点
    route_clear       删除路线
stop 停止任何任务；到终点、偏离、定位丢失、控制权被抢走也会停。
每趟记录一份 CSV；直线正常走完的趟次把学到的舵机零位（积分）存下来，下次起步直接用（路线起步也用它）。
"""
import csv
import dataclasses
import json
import math
import os
import time

import route_plan
from line_tracker import LineConfig, LineTracker
from path_tracker import PathConfig, PathTracker

SELECT_TIMEOUT_S = 3.0
ANTENNA_STALE_S = 2.0       # 天线定位这么久没更新就不在状态里报
# 融合给出的原因里会禁止控制的那些，按根因排序（其余如 heading_unavailable、gyro_only 只说明降级来源），
# 见 docs/GNSS_FUSION.md
BLOCKING_REASONS = ('heading_uninitialized', 'imu_stale', 'wheel_stale', 'rtk_fixed_timeout', 'position_reset',
                    'heading_uncertain', 'position_uncertain')


def pose_problem(pose):
    """位姿不能用于控制时返回原因（原因代码用顿号连接，app 负责翻译），能用返回 None。"""
    if pose is None:
        return '还没有定位'
    if pose.usable_for_control:
        return None
    reasons = tuple(getattr(pose, 'reasons', ()))
    blocking = [r for r in BLOCKING_REASONS if r in reasons]
    if 'heading_uninitialized' in blocking:
        # 航向没初始化时航向、位置的不确定度必然超限（杆臂），不重复报
        blocking = [r for r in blocking if r not in ('heading_uncertain', 'position_uncertain')]
    blocking = blocking or list(reasons)
    return 'pose_unusable' + ('：' + '、'.join(blocking) if blocking else '')


class LineMission:
    def __init__(self, fusion, samples, tracker=None, *, state_path=None, log_dir=None, route_path=None,
                 path_config=None):
        """fusion 实现 docs 里约定的 GnssFusion 接口；samples 提供 ImuSample 等输入类型（通常就是 gnss_fusion 模块）。"""
        self.fusion, self.samples = fusion, samples
        self.tracker = tracker or LineTracker(LineConfig())
        self.line_cfg, self.geo = self.tracker.cfg, self.tracker.geo
        self.path_cfg = path_config or PathConfig()
        # 路线曲率最多用到底盘最大曲率的 kappa_margin，剩下的留给纠偏（与 PathTracker 的检查一致）
        self.min_route_radius = self.geo.min_turn_radius_m / self.path_cfg.kappa_margin
        self.state_path, self.log_dir, self.route_path = state_path, log_dir, route_path
        self.task = 'line'              # 当前（或最近一次）任务：line / route
        self.points = {}                # 'A'/'B' -> (lat, lon)
        self.route = None               # route_plan.RouteSpec，路点存经纬度
        self.route_rev = 0
        self._route_cache = None
        self._antenna = None            # 最近一次天线定位 (t, lat, lon, sol, sigma)
        self.phase = 'IDLE'             # IDLE / SELECTING / RUNNING / RELEASING
        self.message = ''
        self.select_since = None
        self.cruise = self.tracker.cfg.cruise_mps
        self.pose = None
        self._last_tick = None
        self.release_pending = False    # 任务结束后要把 NAVIGATION 交还给仲裁
        self._log = self._log_file = None
        self.integral0 = self._load_integral()
        self._load_route()

    # ---- 传感器输入 ----
    def on_gnss(self, m):
        s, kind = self.samples, m.get('kind')
        inf = lambda v: math.inf if v is None else v  # noqa: E731
        if kind == 'fix':
            self._antenna = (m['t'], m['lat'], m['lon'], m['sol'], max(m['sigma_n'], m['sigma_e']))
            self.fusion.add_fix(s.GnssFix(t=m['t'], lat_deg=m['lat'], lon_deg=m['lon'], height_m=m['height'],
                                          sol=m['sol'], sigma_n=m['sigma_n'], sigma_e=m['sigma_e'],
                                          sigma_u=m['sigma_u'], diff_age_s=inf(m['diff_age']), sats=m['sats']))
        elif kind == 'heading':
            # 只用 rtk_node 判为有效的测向（整周固定，且两根天线固定解的平面实时距离 ≥0.10 m）。
            # 融合不再按基线长度判断，所以没带 valid 字段的也不用
            if m.get('valid') is not True:
                return
            self.fusion.add_heading(s.DualHeading(t=m['t'], heading_deg=m['heading'], pitch_deg=m['pitch'],
                                                  length_m=m['length'], sol=m['sol'],
                                                  sigma_heading_deg=m['sigma_heading'], sats=m['sats']))
        elif kind == 'vel':
            self.fusion.add_velocity(s.GnssVelocity(t=m['t'], speed_mps=m['speed'], track_deg=m['track'],
                                                    sol=m['sol']))

    def on_imu(self, t, gyro_z):
        self.fusion.add_imu(self.samples.ImuSample(t=t, gyro_z=gyro_z))

    def on_wheel(self, t, speed, yaw_rate):
        self.fusion.add_wheel(self.samples.WheelSample(t=t, speed=speed, yaw_rate=yaw_rate))

    # ---- 操作 ----
    def command(self, cmd, now):
        """返回 (是否接受, 说明)。start 被接受只表示开始申请控制权，结果看 tick/status。"""
        kind = cmd.get('cmd')
        if kind == 'mark':
            return self._mark(str(cmd.get('point', '')).upper(), now)
        if kind == 'clear':
            if self.phase != 'IDLE':
                return False, '先停止当前任务'
            self.points.clear()
            return True, '已清除 A、B'
        if kind == 'start':
            return self._start(cmd, now)
        if kind == 'route_set':
            return self._route_set(cmd.get('route'))
        if kind == 'route_start':
            return self._route_start(cmd, now)
        if kind == 'route_clear':
            if self.phase != 'IDLE':
                return False, '先停止当前任务'
            self._store_route(None)
            return True, '已删除路线'
        if kind == 'stop':
            self.tracker.stop('operator')
            if self.phase in ('SELECTING', 'RUNNING'):
                self._finish('操作员停止')
            return True, '已停止'
        return False, '未知指令'

    def _mark(self, point, now):
        if point not in ('A', 'B'):
            return False, '只能记 A 或 B'
        if self.phase != 'IDLE':
            return False, '运行中不能改点'
        pose = self.fusion.pose(now)
        # 航向没初始化时融合报 INVALID（位置可能已是固定解），这时说清楚卡在哪，而不是笼统地要固定解
        if pose is not None and pose.quality in ('RTK_FLOAT', 'DEAD_RECKONING'):
            return False, '需要 RTK 固定解才能记点'
        problem = pose_problem(pose)
        if problem:
            return False, problem
        if pose.quality != 'RTK_FIXED':
            return False, '需要 RTK 固定解才能记点'
        px, py = self._track_point(pose)
        self.points[point] = self.fusion.to_geodetic(px, py)
        return True, '已记 %s 点' % point

    def _track_point(self, pose):
        k = self.line_cfg.track_x_m
        return pose.x + k * math.cos(pose.yaw), pose.y + k * math.sin(pose.yaw)

    def _paused(self, task):
        return self.phase == 'RUNNING' and self.task == task and self.tracker.st.state == 'PAUSED'

    def _resume(self, now):
        """定位恢复后操作员再按开始：控制权还在，重新检查起步条件就走。"""
        pose = self.fusion.pose(now)
        problem = pose_problem(pose)
        if problem:
            return False, problem
        ok = self.tracker.start(pose)
        return ok, '继续' if ok else self.tracker.st.reason

    def _start(self, cmd, now):
        if self._paused('line'):
            return self._resume(now)
        if self.phase != 'IDLE':
            return False, '已在运行'
        if not {'A', 'B'} <= set(self.points):
            return False, '先记 A、B 两点'
        cruise = float(cmd.get('cruise', self.cruise))
        if not 0.05 <= cruise <= 1.0:
            return False, '速度应在 0.05–1.0 m/s'
        pose = self.fusion.pose(now)
        problem = pose_problem(pose)
        if problem:
            return False, problem
        tracker = LineTracker(dataclasses.replace(self.line_cfg, cruise_mps=cruise), self.geo)
        try:
            tracker.set_line(self.fusion.to_local(*self.points['A']), self.fusion.to_local(*self.points['B']))
        except ValueError as exc:
            return False, str(exc)
        ok, why = tracker.check_start(pose)
        if not ok:
            return False, why
        self.tracker, self.task, self.cruise = tracker, 'line', cruise
        self.phase, self.select_since, self.message = 'SELECTING', now, '申请控制权'
        return True, '申请控制权'

    # ---- 路线 ----
    def _route_set(self, raw):
        if self.phase != 'IDLE':
            return False, '运行中不能改路线'
        try:
            spec = route_plan.parse_spec(raw)
        except ValueError as exc:
            return False, str(exc)
        if self._origin() is None:
            return False, '还没有定位原点，等 RTK 有解后再画路线'
        try:
            stored = spec.map_points(self.fusion.to_geodetic)
        except ValueError as exc:
            return False, str(exc)
        self._store_route(stored)
        return True, '路线已保存到车上'

    def _route_local(self):
        """存的路线换算到当前本地坐标；还没有原点时抛 ValueError。"""
        return self.route.map_points(self.fusion.to_local)

    def _route_start(self, cmd, now):
        if self._paused('route'):
            return self._resume(now)
        if self.phase != 'IDLE':
            return False, '已在运行'
        if 'route' in cmd:
            ok, why = self._route_set(cmd['route'])
            if not ok:
                return False, why
        if self.route is None:
            return False, '车上还没有路线'
        pose = self.fusion.pose(now)
        problem = pose_problem(pose)
        if problem:
            return False, problem
        try:
            spec = self._route_local()
            k = self.line_cfg.track_x_m     # 与直线相同：控制点（默认后轴中心）压在路线上
            car = (pose.x + k * math.cos(pose.yaw), pose.y + k * math.sin(pose.yaw), pose.yaw)
            path, entry_m = route_plan.plan(spec, car, self.min_route_radius)
            cfg = dataclasses.replace(self.path_cfg, cruise_mps=spec.cruise, track_x_m=k)
            tracker = PathTracker(path, cfg, self.geo)
        except ValueError as exc:
            return False, str(exc)
        ok, why = tracker.check_start(pose)
        if not ok:
            return False, why
        self.tracker, self.task, self.cruise = tracker, 'route', spec.cruise
        self.phase, self.select_since = 'SELECTING', now
        self.message = '申请控制权（接入 %.1f m，全程 %.1f m）' % (entry_m, path.length)
        return True, self.message

    def _store_route(self, spec):
        self.route, self._route_cache = spec, None
        self.route_rev += 1
        if not self.route_path:
            return
        if spec is None:
            try:
                os.remove(self.route_path)
            except FileNotFoundError:
                pass
            return
        os.makedirs(os.path.dirname(self.route_path) or '.', exist_ok=True)
        tmp = self.route_path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as fh:
            json.dump(dict(version=1, rev=self.route_rev, saved=time.strftime('%Y-%m-%d %H:%M:%S'),
                           route=spec.to_dict(digits=9)), fh, ensure_ascii=False)
        os.replace(tmp, self.route_path)

    def _load_route(self):
        try:
            with open(self.route_path, encoding='utf-8') as fh:
                d = json.load(fh)
            self.route = route_plan.parse_spec(d['route'], geodetic=True)
            self.route_rev = int(d.get('rev', 1))
        except (TypeError, OSError, ValueError, KeyError):
            self.route = None

    def _route_status(self):
        """路线换算到当前本地坐标 + 能不能跑（problem）。按 (版本, 原点) 缓存，状态 5 Hz 不用每次重算。"""
        if self.route is None:
            return None
        origin = self._origin()
        key = (self.route_rev, None if origin is None else tuple(origin))
        if self._route_cache and self._route_cache[0] == key:
            return self._route_cache[1]
        try:
            spec = self._route_local()
        except ValueError:
            return None
        out = spec.to_dict()
        out['rev'] = self.route_rev
        length, out['problem'] = route_plan.route_length(spec, self.min_route_radius)
        out['length'] = None if length is None else round(length, 3)
        self._route_cache = (key, out)
        return out

    def _origin(self):
        try:
            lat, lon = self.fusion.to_geodetic(0.0, 0.0)
        except ValueError:
            return None
        return [round(lat, 9), round(lon, 9)]

    def _antenna_status(self):
        a = self._antenna
        if a is None or self._last_tick is None or not 0.0 <= self._last_tick - a[0] <= ANTENNA_STALE_S:
            return None
        try:
            x, y = self.fusion.to_local(a[1], a[2])
        except ValueError:
            return None
        return dict(x=round(x, 3), y=round(y, 3), sol=a[3], sigma=round(a[4], 4))

    # ---- 控制周期 ----
    def tick(self, now, motion_mode):
        """返回 (vx, wz, select)。select=True/False 表示节点要调用 /motion/navigation，None 不动。"""
        dt = 0.05 if self._last_tick is None else max(0.0, min(0.2, now - self._last_tick))
        self._last_tick = now
        self.pose = pose = self.fusion.pose(now)
        if self.phase == 'IDLE' and self.release_pending:
            self.release_pending = False
            return 0.0, 0.0, False
        if self.phase == 'SELECTING':
            if motion_mode == 'NAVIGATION':
                if pose is None or not self.tracker.start(pose, integral=self._start_integral()):
                    self._finish('起步检查未通过：%s' % (self.tracker.st.reason if pose else 'no_pose'))
                    return 0.0, 0.0, None
                self.phase, self.message = 'RUNNING', '行驶中'
                self._open_log(now)
            elif now - self.select_since > SELECT_TIMEOUT_S:
                self._finish('控制权申请失败（当前 %s）' % motion_mode)
                return 0.0, 0.0, None
            else:
                return 0.0, 0.0, True
        if self.phase != 'RUNNING':
            return 0.0, 0.0, None
        if motion_mode != 'NAVIGATION':
            self.tracker.stop('motion_' + str(motion_mode).lower())
            self._finish('控制权被切走（%s）' % motion_mode)
            return 0.0, 0.0, None
        if pose is None:
            pose = _NoPose()
        vx, wz = self.tracker.update(pose, dt)
        self._write_log(now, pose, vx)
        st = self.tracker.st.state
        if st == 'DONE':
            if self.task == 'line':
                self._save_integral()
            self._finish('到达 B 点' if self.task == 'line' else '路线完成')
            return 0.0, 0.0, None
        if st == 'ABORTED':
            self._finish('中止：%s' % self.tracker.st.reason)
            return 0.0, 0.0, None
        return vx, wz, None

    def _start_integral(self):
        """直线学到的舵机零位（积分）。路线控制器积分增益不同，按相同的零位曲率换算过去。"""
        if self.task == 'line':
            return self.integral0
        return self.integral0 * self.line_cfg.ki_per_m2 / self.tracker.cfg.ki_per_m2

    def _finish(self, message):
        self.phase, self.message = 'IDLE', message
        self.release_pending = True
        self._close_log()

    # ---- 记录与持久化 ----
    def _open_log(self, now):
        if not self.log_dir:
            return
        os.makedirs(self.log_dir, exist_ok=True)
        name = time.strftime(self.task + '_%Y%m%d_%H%M%S.csv')
        self._log_file = open(os.path.join(self.log_dir, name), 'w', newline='')
        self._log = csv.writer(self._log_file)
        self._log.writerow(['t', 'x', 'y', 'yaw_deg', 'speed', 'e_m', 'heading_err_deg', 's_m', 'v_cmd',
                            'kappa', 'quality', 'sigma_xy', 'sigma_yaw_deg', 'state'])

    def _write_log(self, now, pose, vx):
        if not self._log:
            return
        st = self.tracker.st
        self._log.writerow(['%.3f' % now, '%.4f' % pose.x, '%.4f' % pose.y, '%.3f' % math.degrees(pose.yaw),
                            '%.3f' % pose.speed, '%.4f' % st.e, '%.3f' % math.degrees(st.heading_err),
                            '%.3f' % st.s, '%.3f' % vx, '%.5f' % st.kappa, getattr(pose, 'quality', ''),
                            '%.4f' % getattr(pose, 'sigma_xy', math.nan),
                            '%.3f' % math.degrees(getattr(pose, 'sigma_yaw', math.nan)), st.state])

    def _close_log(self):
        if self._log_file:
            self._log_file.close()
        self._log = self._log_file = None

    def _load_integral(self):
        try:
            with open(self.state_path, encoding='utf-8') as fh:
                value = float(json.load(fh)['integral'])
            return value if math.isfinite(value) else 0.0
        except (TypeError, OSError, ValueError, KeyError):
            return 0.0

    def _save_integral(self):
        # 只从走满 5 m 的趟次学习，短线积分还没收敛
        if not self.state_path or self.tracker.st.length < 5.0:
            return
        self.integral0 = self.tracker.st.integral
        os.makedirs(os.path.dirname(self.state_path) or '.', exist_ok=True)
        tmp = self.state_path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as fh:
            json.dump(dict(integral=self.integral0, trim_kappa=self.tracker.status()['trim_kappa'],
                           saved=time.strftime('%Y-%m-%d %H:%M:%S')), fh)
        os.replace(tmp, self.state_path)

    def status(self):
        p = self.pose
        pose = None if p is None else dict(
            x=round(p.x, 3), y=round(p.y, 3), yaw_deg=round(math.degrees(p.yaw), 2),
            speed=round(p.speed, 3), quality=p.quality, usable=p.usable_for_control,
            sigma_xy=round(p.sigma_xy, 4), sigma_yaw_deg=round(math.degrees(p.sigma_yaw), 3),
            reasons=list(p.reasons))
        points = {k: self.fusion.to_local(*v) for k, v in self.points.items()} if self.points else {}
        # 融合的在线标定（安装偏差、轮速比例、陀螺零偏）：只报告，经人确认后才写进 ~/rtk/fusion.json
        calibration = getattr(self.fusion, 'calibration', None)
        return _json_safe(dict(phase=self.phase, message=self.message, cruise=self.cruise, pose=pose,
                               points={k: [round(v[0], 3), round(v[1], 3)] for k, v in points.items()},
                               tracker=self.tracker.status(), task=self.task,
                               origin=self._origin(), antenna=self._antenna_status(), route=self._route_status(),
                               route_limits=dict(min_radius=round(self.min_route_radius, 3),
                                                 max_points=route_plan.MAX_POINTS, max_laps=route_plan.MAX_LAPS,
                                                 max_entry_m=route_plan.MAX_ENTRY_M),
                               calibration=calibration() if callable(calibration) else None))


class _NoPose:
    """融合还没有输出时交给控制器的占位：不可用，控制器会立即停车。"""
    x = y = yaw = speed = yaw_rate = 0.0
    usable_for_control = False


def _json_safe(value):
    """状态要用 allow_nan=False 发布：非有限的浮点数换成 None，免得一个 NaN 让状态定时器抛异常。"""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    return value
