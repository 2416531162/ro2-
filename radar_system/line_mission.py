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
差速车（Gen5）的直线改用 line_cascade.ImuLineTracker：IMU 航向保持做内环、RTK 横偏修正做外环（底盘转向晚
0.44 s，LineTracker 0.8 m/s 会来回大摆）；它的积分单位不同，不读也不存上面的舵机零位。
"""
import csv
import dataclasses
import json
import math
import os
import time
from collections import deque

import route_plan
from line_cascade import NOTICES as CASCADE_NOTICES, REASON_TEXT, SHARED_FIELDS, CascadeConfig, ImuLineTracker
from line_tracker import LineConfig, LineTracker
from path_tracker import PathConfig, SectionTracker, differential_config
from route_operations import RouteOperations

SELECT_TIMEOUT_S = 3.0
ANTENNA_STALE_S = 2.0       # 天线定位这么久没更新就不在状态里报
# 定位跳动：RTK 固定解与融合在同一时刻推算的主天线位置差这么多就记一次（app 在地图上标出来，
# 反复出现的地方多半有遮挡或反射）；间隔不到 JUMP_MERGE_S 的算同一次，取最大
JUMP_EVENT_M = 0.03
JUMP_MERGE_S = 1.0
JUMP_EVENTS_MAX = 50
JUMP_EVENTS_REPORTED = 20
RTK_FIXED_SOLS = ('NARROW_INT',)            # 与 gnss_fusion 的固定解一致
INNOVATION_STALE_S = 1.0
# 融合给出的原因里会禁止控制的那些，按根因排序（其余如 heading_unavailable、gyro_only 只说明降级来源），
# 见 docs/GNSS_FUSION.md
BLOCKING_REASONS = ('heading_uninitialized', 'imu_stale', 'imu_mismatch', 'wheel_stale', 'rtk_hold_timeout',
                    'rtk_fixed_timeout', 'position_reset', 'heading_uncertain', 'position_uncertain')
# 测向浮点（无效）时航向只靠陀螺和行驶轨迹维持，融合的航向不确定度一过 0.5° 就判不可控制。
# 主、副天线都是固定解（位置可信）时放宽到这么多照样走，原因里加 gyro_hold（2026-09-27 用户要求：
# 走着走着测向变浮点不能停，要靠 IMU 维持）
GYRO_HOLD_SIGMA_YAW_DEG = 2.0
FIX2_MAX_AGE_S = 1.0        # 副天线固定解这么久内来过就算还固定着
# 因定位问题暂停后，定位连续可用这么久就自动接着走（操作员停止、控制权被切走照样结束，不会自动恢复）
AUTO_RESUME_S = 1.0
# 定点画线 start 没带 cruise 时的车速（用户定的画线速度）；不再沿用上一次任务（跑过别的速度的路线也不影响）
LINE_CRUISE_MPS = 0.8
# 卫星位置不可信、按推算维持（融合 rtk_hold_s，Gen5 5 s）时给操作员的话；恢复后改回「行驶中」
RTK_NOTICE_HOLD = '卫星定位不可信，按 IMU 推算行驶'
RTK_NOTICE_TIMEOUT = '卫星定位不可信超过 5 s，已停车，定位恢复后自动继续'
# 掉信号（固定解没来）原地停（2026-10-01 用户规则），恢复后自动继续
RTK_NOTICE_LOST = '卫星信号中断，已原地停车，恢复后自动继续'


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
                 path_config=None, body=None, nozzle_x=None):
        """fusion 实现 docs 里约定的 GnssFusion 接口；samples 提供 ImuSample 等输入类型（通常就是 gnss_fusion 模块）。

        body = (后轴中心到车头, 到车尾, 半宽) 米，连线路线按车身中心压线；默认取共享底盘配置。
        差速车（tracker.geo.differential，Gen5）的连线路线改按喷头压线（nozzle_x：喷头在后轴前多少米，
        默认取共享配置 drive.nozzle_x_m），急弯停车原地转向。
        """
        self.fusion, self.samples = fusion, samples
        self.tracker = tracker or LineTracker(LineConfig())
        self.line_cfg, self.geo = self.tracker.cfg, self.tracker.geo
        self.path_cfg = path_config or PathConfig()
        # 路线曲率最多用到底盘最大曲率的 kappa_margin，剩下的留给纠偏（与 PathTracker 的检查一致）
        self.min_route_radius = self.geo.min_turn_radius_m / self.path_cfg.kappa_margin
        if body is None:
            from runtime_config import PROFILE
            g = PROFILE['geometry']
            body = (g['front_m'], g['rear_m'], g['half_width_m'])
        self.body = tuple(float(v) for v in body)
        self.differential = bool(getattr(self.geo, 'differential', False))
        if self.differential:
            if path_config is None:
                self.path_cfg = differential_config(self.path_cfg)
            if nozzle_x is None:
                from runtime_config import PROFILE
                nozzle_x = PROFILE['drive']['nozzle_x_m']
        self.nozzle_x = float(nozzle_x) if self.differential else None
        self.state_path, self.log_dir, self.route_path = state_path, log_dir, route_path
        self.task = 'line'              # 当前（或最近一次）任务：line / route
        self.points = {}                # 'A'/'B' -> (lat, lon)
        self.route = None               # route_plan.RouteSpec，路点存经纬度
        self.route_rev = 0
        self._route_cache = None
        self._antenna = None            # 最近一次天线定位 (t, lat, lon, sol, sigma)
        self._imu = None                # 最近一帧 IMU (t, 偏航 rad, 陀螺 z rad/s)，差速车定点画线的内环用
        self._fix2_t = None             # 副天线最近一次固定解的时刻
        self._usable_since = None       # 暂停中定位从什么时候起一直可用（自动继续用）
        self._innovation = None         # 最近一次固定解与融合推算之差 (t, 沿车头, 向左) 米
        self._jumps = deque(maxlen=JUMP_EVENTS_MAX)     # 定位跳动记录，本次开机以来（只留最近 50 次）
        self._jump_count = 0            # 本次开机以来定位跳动总次数（作业记录按首末相减，不能用上面那个的长度）
        self._jump = None               # 正在进行的那次跳动
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
        self.last_motion_mode = None
        self.operations = RouteOperations(self)

    # ---- 传感器输入 ----
    def on_gnss(self, m):
        s, kind = self.samples, m.get('kind')
        inf = lambda v: math.inf if v is None else v  # noqa: E731
        if kind == 'fix':
            self._antenna = (m['t'], m['lat'], m['lon'], m['sol'], max(m['sigma_n'], m['sigma_e']))
            if m['sol'] in RTK_FIXED_SOLS:
                self._check_innovation(m)
            self.fusion.add_fix(s.GnssFix(t=m['t'], lat_deg=m['lat'], lon_deg=m['lon'], height_m=m['height'],
                                          sol=m['sol'], sigma_n=m['sigma_n'], sigma_e=m['sigma_e'],
                                          sigma_u=m['sigma_u'], diff_age_s=inf(m['diff_age']), sats=m['sats']))
        elif kind == 'fix2':
            # 副天线：判断「两根天线都固定」（测向浮点时靠陀螺维持航向的前提）；带位置时交给融合做双天线位置互检
            # （两个固定解位置之差必须等于测向基线，对不上就不信固定解），不直接进融合
            if m.get('sol') in RTK_FIXED_SOLS:
                self._fix2_t = m['t']
            add_fix2 = getattr(self.fusion, 'add_fix2', None)
            if callable(add_fix2) and m.get('lat') is not None and m.get('lon') is not None:
                add_fix2(s.GnssFix(t=m['t'], lat_deg=m['lat'], lon_deg=m['lon'], height_m=m.get('height', 0.0),
                                   sol=m['sol'], sigma_n=m.get('sigma_n', 0.01), sigma_e=m.get('sigma_e', 0.01),
                                   sigma_u=m.get('sigma_u', 0.02), diff_age_s=inf(m.get('diff_age')),
                                   sats=m.get('sats', 0)))
        elif kind == 'heading':
            # 只用 rtk_node 判为有效的测向（整周固定，且基线长度在 0.30–0.50 m，见 um982.heading_usable）。
            # 融合自己不再按基线长度判断，所以没带 valid 字段的也不用
            if m.get('valid') is not True:
                return
            self.fusion.add_heading(s.DualHeading(t=m['t'], heading_deg=m['heading'], pitch_deg=m['pitch'],
                                                  length_m=m['length'], sol=m['sol'],
                                                  sigma_heading_deg=m['sigma_heading'], sats=m['sats']))
        elif kind == 'vel':
            self.fusion.add_velocity(s.GnssVelocity(t=m['t'], speed_mps=m['speed'], track_deg=m['track'],
                                                    sol=m['sol']))

    def _check_innovation(self, m):
        """固定解与融合"先验"（这次观测交给融合之前，按车轮+陀螺推算到同一时刻）的主天线位置之差。

        车体系分解：沿车头、向左。差 JUMP_EVENT_M 以上记一次定位跳动（车当时的位置、经纬度、最大差值）。
        只在融合本来就是固定解、可用于控制时记，开机、失锁恢复时的大差值不算；也只在作业正沿线跑时记
        （2026-10-01 用户：地图上跳动标记太多。原地转、换段、停着、手动开车时融合与 RTK 的差多是转动中的时间差，
        不是定位跳了）。"""
        fusion = self.fusion
        try:
            prior = fusion.pose(m['t'])
            if prior is None:
                return
            fx, fy = fusion.to_local(m['lat'], m['lon'])
            # 与融合用同一个杠杆臂（含倾斜补偿），坡上天线偏出去的那段不算跳动
            lever = getattr(fusion, 'antenna_lever', None)
            ax, ay = lever() if callable(lever) else fusion.config.antenna_xy
        except (AttributeError, ValueError, TypeError):
            return
        c, s = math.cos(prior.yaw), math.sin(prior.yaw)
        dx = fx - (prior.x + c * ax - s * ay)
        dy = fy - (prior.y + s * ax + c * ay)
        along, left = c * dx + s * dy, -s * dx + c * dy
        self._innovation = (m['t'], along, left)
        d = math.hypot(dx, dy)
        if d < JUMP_EVENT_M or prior.quality != 'RTK_FIXED' or not prior.usable_for_control or not self._on_line():
            return
        j = self._jump
        if j is None or m['t'] - j['t1'] > JUMP_MERGE_S:
            j = self._jump = dict(t0=m['t'], t1=m['t'], d=0.0)
            self._jumps.append(j)
            self._jump_count += 1
        j['t1'] = m['t']
        if d > j['d']:
            try:
                lat, lon = fusion.to_geodetic(prior.x, prior.y)
            except (AttributeError, ValueError):
                lat = lon = None
            j.update(d=d, left=left, x=prior.x, y=prior.y, lat=lat, lon=lon)

    def on_imu(self, t, gyro_z, yaw=None, roll=None, pitch=None):
        """yaw：IMU 自己算的航向角（弧度，左转为正）。融合 yaw_input = 'angle' 时用它的增量推算航向。
        roll / pitch：IMU 算好的横滚、俯仰（弧度，REP-103），融合做倾斜补偿用；两个都有才给。"""
        extra = {} if yaw is None else {'yaw': yaw}
        if roll is not None and pitch is not None:
            extra.update(roll=roll, pitch=pitch)
        self.fusion.add_imu(self.samples.ImuSample(t=t, gyro_z=gyro_z, **extra))
        if yaw is not None and math.isfinite(yaw) and math.isfinite(gyro_z):
            self._imu = (t, yaw, gyro_z)

    def on_wheel(self, t, speed, yaw_rate):
        self.fusion.add_wheel(self.samples.WheelSample(t=t, speed=speed, yaw_rate=yaw_rate))

    # ---- 操作 ----
    def command(self, cmd, now):
        """返回 (是否接受, 说明)。start 被接受只表示开始申请控制权，结果看 tick/status。"""
        kind = cmd.get('cmd')
        if kind=='route_field_start':
            try:return True,self.operations.start_field(cmd,now)
            except (ValueError,TypeError,KeyError) as exc:return False,str(exc)
        if kind in ('route_pause', 'route_preview', 'route_resume', 'route_rework'):
            try:
                if kind == 'route_pause':
                    message = self.operations.pause(cmd, now)
                elif kind == 'route_preview':
                    message = self.operations.prepare(cmd, now)
                else:
                    message = self.operations.commit(cmd, now, kind.removeprefix('route_'))
                return True, message
            except (ValueError, TypeError, KeyError) as exc:
                return False, str(exc)
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
        if kind in ('route_start', 'route_start_current'):
            return self._route_start(cmd, now)
        if kind == 'route_clear':
            if self.phase != 'IDLE' or self.operations.held:
                return False, '先停止当前任务'
            self._store_route(None)
            self.operations.clear()
            return True, '已删除路线'
        if kind == 'stop':
            self.tracker.stop('operator')
            if self.phase in ('SELECTING', 'RUNNING', 'PAUSING', 'PAUSED') or self.operations.held:
                self._finish('操作员停止', state='CANCELLED')
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

    def _on_line(self):
        """作业正沿线跑：开着作业、跟踪器在跑，不在换段 / 原地转、不在倒车段。"""
        t = self.tracker
        if self.phase != 'RUNNING' or t.st.state != 'RUNNING' or getattr(t, 'switching', False):
            return False
        sections = getattr(t, 'sections', None)
        return not sections or sections[t.index].direction > 0

    def _paused(self, task):
        return self.phase == 'RUNNING' and self.task == task and self.tracker.st.state == 'PAUSED'

    def _control_pose(self, now):
        """控制用的位姿：融合位姿，只因测向无效、陀螺维持的航向不确定度超了融合门槛而不可控制时，
        若主天线（融合已是 RTK 固定解）和副天线都固定、不确定度不超过 GYRO_HOLD_SIGMA_YAW_DEG，照样可用。"""
        pose = self.fusion.pose(now)
        if pose is None or pose.usable_for_control:
            return pose
        reasons = tuple(getattr(pose, 'reasons', ()))
        blocking = [r for r in BLOCKING_REASONS if r in reasons]
        fix2 = self._fix2_t is not None and now - self._fix2_t <= FIX2_MAX_AGE_S
        if (blocking == ['heading_uncertain'] and 'heading_unavailable' in reasons and fix2
                and pose.quality == 'RTK_FIXED' and pose.sigma_yaw <= math.radians(GYRO_HOLD_SIGMA_YAW_DEG)):
            return _with(pose, usable_for_control=True, reasons=reasons + ('gyro_hold',))
        return pose

    def _resume(self, now):
        """定位恢复后接着走（操作员再按开始，或 tick 里定位连续可用 AUTO_RESUME_S 后自动）：控制权还在，重新检查起步条件。"""
        pose = self._control_pose(now)
        problem = pose_problem(pose)
        if problem:
            return False, problem
        self._sync_imu(self.tracker, now)
        ok = self.tracker.start(pose)
        return ok, '继续' if ok else REASON_TEXT.get(self.tracker.st.reason, self.tracker.st.reason)

    def _start(self, cmd, now):
        if self.operations.held:
            return False, '路线任务断点已保留，请先继续或结束任务'
        if self._paused('line'):
            return self._resume(now)
        if self.phase != 'IDLE':
            return False, '已在运行'
        if not {'A', 'B'} <= set(self.points):
            return False, '先记 A、B 两点'
        cruise = float(cmd['cruise']) if cmd.get('cruise') is not None else LINE_CRUISE_MPS
        if not 0.05 <= cruise <= 1.0:
            return False, '速度应在 0.05–1.0 m/s'
        pose = self._control_pose(now)
        problem = pose_problem(pose)
        if problem:
            return False, problem
        tracker = self._line_tracker(cruise)
        self._sync_imu(tracker, now)
        try:
            tracker.set_line(self.fusion.to_local(*self.points['A']), self.fusion.to_local(*self.points['B']))
        except ValueError as exc:
            return False, str(exc)
        ok, why = tracker.check_start(pose)
        if not ok:
            return False, REASON_TEXT.get(why, why)
        self.tracker, self.task, self.cruise = tracker, 'line', cruise
        self.phase, self.select_since, self.message = 'SELECTING', now, '申请控制权'
        return True, '申请控制权'

    def _line_tracker(self, cruise):
        """定点画线的控制器：差速车用 IMU 航向内环 + RTK 横偏外环（起步、限速、停车规则抄 LineConfig），
        阿克曼旧车照旧 LineTracker。"""
        if self.differential:
            shared = {name: getattr(self.line_cfg, name) for name in SHARED_FIELDS}
            return ImuLineTracker(CascadeConfig(cruise_mps=cruise, nozzle_x_m=self.nozzle_x, **shared), self.geo)
        return LineTracker(dataclasses.replace(self.line_cfg, cruise_mps=cruise), self.geo)

    def _sync_imu(self, tracker, now):
        """IMU 航向内环要此刻的时钟、最近一帧 IMU（与 IMU 回调、融合同一个单调时钟）和融合用的天线杠杆臂
        （航向跳变时按 IMU 航向改算位置）；别的控制器没有 sync。"""
        sync = getattr(tracker, 'sync', None)
        if sync is not None:
            sync(now, self._imu, self._antenna_lever())

    def _antenna_lever(self):
        """融合正在用的主天线水平杠杆臂（车体系前、左，m，含倾斜补偿）；拿不到时 None。"""
        try:
            lever = getattr(self.fusion, 'antenna_lever', None)
            ax, ay = lever() if callable(lever) else self.fusion.config.antenna_xy
            return float(ax), float(ay)
        except (AttributeError, TypeError, ValueError):
            return None

    # ---- 路线 ----
    def _route_set(self, raw):
        if self.phase != 'IDLE' or self.operations.held:
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
        self.operations.clear()
        return True, '路线已保存到车上'

    def _route_local(self):
        """存的路线换算到当前本地坐标；还没有原点时抛 ValueError。"""
        return self.route.map_points(self.fusion.to_local)

    def _route_start(self, cmd, now):
        if self.operations.held:
            return False, '任务断点已保留，请使用继续作业；重新开始须先结束原任务'
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
        pose = self._control_pose(now)
        problem = pose_problem(pose)
        if problem:
            return False, problem
        try:
            spec = self._route_local()
            k = self.line_cfg.track_x_m     # 与直线相同：控制点（默认后轴中心）压在路线上
            car = (pose.x + k * math.cos(pose.yaw), pose.y + k * math.sin(pose.yaw), pose.yaw)
            planner = route_plan.plan_current if cmd.get('cmd') == 'route_start_current' else route_plan.plan
            sections, entry_m = planner(spec, car, self.min_route_radius, body=self.body, nozzle_x=self.nozzle_x)
            cfg = dataclasses.replace(self.path_cfg, cruise_mps=spec.cruise, track_x_m=k)
            tracker = SectionTracker(sections, cfg, self.geo, **self.section_options())
        except ValueError as exc:
            return False, str(exc)
        ok, why = tracker.check_start(pose)
        if not ok:
            return False, why
        try:
            self.operations.begin(sections, entry_m=entry_m)
        except ValueError as exc:
            return False, str(exc)
        self.tracker, self.task, self.cruise = tracker, 'route', spec.cruise
        self.phase, self.select_since = 'SELECTING', now
        turns = '，换向 %d 次' % tracker.cusps if tracker.cusps else ''
        if self.differential:
            # 倒车几段（每段前后各换向一次，tracker.reversals 是换向次数）
            reverses = sum(1 for sec in sections if sec.direction < 0)
            turns = ('，原地转向 %d 次' % tracker.pivots if tracker.pivots else '') + \
                    ('，倒车 %d 次' % reverses if reverses else '')
        self.message = '申请控制权（接入 %.1f m，全程 %.1f m%s）' % (entry_m, tracker.length, turns)
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
        length, out['problem'] = route_plan.route_length(spec, self.min_route_radius, nozzle_x=self.nozzle_x)
        out['length'] = None if length is None else round(length, 3)
        # 连线路线：车身中心压过全部连线要开的总路程（含掉头补角）、换向次数、掉头补角的拐角数
        out['cover'] = route_plan.cover_summary(spec, self.min_route_radius, self.body, nozzle_x=self.nozzle_x) \
            if spec.kind == 'path' else None
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
        self.last_motion_mode = motion_mode
        self.pose = pose = self._control_pose(now)
        self._sync_imu(self.tracker, now)
        if self.phase == 'PAUSING':
            try:
                return self.operations.hold_tick(now, pose, motion_mode)
            except ValueError as exc:
                self.phase, self.message = 'PAUSED', str(exc)
                return 0.0, 0.0, False
        if self.phase == 'PAUSED':
            if self.release_pending:
                self.release_pending = False
                return 0.0, 0.0, False
            return 0.0, 0.0, None
        if self.phase == 'IDLE' and self.release_pending:
            self.release_pending = False
            return 0.0, 0.0, False
        if self.phase == 'SELECTING':
            if motion_mode == 'NAVIGATION':
                if pose is None or not self.tracker.start(pose, integral=self._start_integral()):
                    why = self.tracker.st.reason if pose else 'no_pose'
                    self._finish('起步检查未通过：%s' % REASON_TEXT.get(why, why))
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
        self._auto_resume(now, pose)
        vx, wz = self.tracker.update(pose, dt)
        # 航向跳变按 IMU 维持 / 超过 5 s 停车等航向（定点画线控制器）、卫星位置不可信按推算维持 / 超时停车（融合）：
        # 报给操作员，过去了改回「行驶中」
        notice = getattr(self.tracker, 'notice', None)
        text = notice() if callable(notice) else None
        reasons = getattr(pose, 'reasons', ()) or ()
        if not text and 'rtk_hold_timeout' in reasons:
            text = RTK_NOTICE_TIMEOUT
        elif not text and 'rtk_fixed_timeout' in reasons:
            text = RTK_NOTICE_LOST
        elif not text and 'rtk_hold' in reasons:
            text = RTK_NOTICE_HOLD
        if text:
            self.message = text
        elif self.message in CASCADE_NOTICES + (RTK_NOTICE_HOLD, RTK_NOTICE_TIMEOUT, RTK_NOTICE_LOST):
            self.message = '行驶中'
        self._write_log(now, pose, vx)
        try:
            self.operations.capture(now)
        except ValueError as exc:
            self.operations.record.update(state='PAUSING', reason='storage_error')
            self.tracker.pause('storage_error')
            self.operations.still_since = None
            self.phase, self.message = 'PAUSING', str(exc) + '；正在停车'
            return 0.0, 0.0, None
        st = self.tracker.st.state
        if st == 'DONE':
            if self.task == 'line':
                self._save_integral()
            self._finish('到达 B 点' if self.task == 'line' else '路线完成')
            return 0.0, 0.0, None
        if st == 'ABORTED':
            why = self.tracker.st.reason
            self._finish('中止：%s' % REASON_TEXT.get(why, why))
            return 0.0, 0.0, None
        return vx, wz, None

    def _auto_resume(self, now, pose):
        """定位问题暂停后，定位连续可用 AUTO_RESUME_S 就接着走；起步检查没过（如离线太远）隔 AUTO_RESUME_S 再试。"""
        if self.operations.held or self.tracker.st.state != 'PAUSED' or not pose.usable_for_control:
            self._usable_since = None
            return
        if self._usable_since is None:
            self._usable_since = now
        if now - self._usable_since < AUTO_RESUME_S - 1e-9:
            return
        why = self.tracker.st.reason
        if self.tracker.start(pose):
            self.message = '航向恢复，自动继续' if why == 'heading_jump' else '定位恢复，自动继续'
            self._usable_since = None
        else:
            self._usable_since = now

    def _start_integral(self):
        """直线学到的舵机零位（积分）。路线控制器积分增益不同，按相同的零位曲率换算过去。
        差速车定点画线（ImuLineTracker）的积分是航向偏置、单位不同：不用，从 0 开始。"""
        if self.task == 'line':
            return None if isinstance(self.tracker, ImuLineTracker) else self.integral0
        return self.integral0 * self.line_cfg.ki_per_m2 / self.tracker.cfg.ki_per_m2

    def _finish(self, message, state=None):
        try:
            self.operations.capture(self._last_tick or 0, force=True)
        except ValueError:
            pass
        self.phase, self.message = 'IDLE', message
        self.release_pending = True
        self._close_log()
        try:
            self.operations.finish(state or ('DONE' if self.tracker.st.state == 'DONE' else 'INTERRUPTED'), message)
        except ValueError as exc:
            self.message += '；' + str(exc)

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
        # 只从走满 5 m 的趟次学习，短线积分还没收敛；差速车定点画线的积分不是舵机零位，不存
        if not self.state_path or self.tracker.st.length < 5.0 or isinstance(self.tracker, ImuLineTracker):
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
        calibration = calibration() if callable(calibration) else None
        # 倾斜补偿：天线高度、在不在补、没补的原因、补偿量（app 姿态卡片显示）
        tilt = getattr(self.fusion, 'tilt_status', None)
        tilt = tilt() if callable(tilt) else None
        return _json_safe(dict(phase=self.phase, message=self.message, cruise=self.cruise, pose=pose,
                               points={k: [round(v[0], 3), round(v[1], 3)] for k, v in points.items()},
                               tracker=self.tracker.status(), task=self.task,
                               origin=self._origin(), antenna=self._antenna_status(), route=self._route_status(),
                               route_limits=dict(min_radius=round(self.min_route_radius, 3),
                                                 max_points=route_plan.MAX_POINTS, max_laps=route_plan.MAX_LAPS,
                                                 max_entry_m=route_plan.MAX_ENTRY_M,
                                                 # 连线路线压线用的车身：后轴中心到车头、车尾、半宽（app 画同样的预览）
                                                 body=[round(v, 4) for v in self.body],
                                                 # 差速车（能原地转）按喷头压线：nozzle_x 喷头在后轴前多少米
                                                 drive=self.geo.drive, nozzle_x=self.nozzle_x,
                                                 pivot_rate=self.geo.pivot_rate_rad_s,
                                                 pivot_accel=self.geo.pivot_accel_rad_s2,
                                                 accel=self.path_cfg.accel_mps2,
                                                 lateral_accel=self.path_cfg.lat_accel_mps2),
                               calibration=calibration, diag=self._diag(calibration), tilt=tilt,
                               rtk_events=self._jump_events(), operations=self.operations.status()))

    def section_options(self):
        """SectionTracker 按底盘的额外参数：差速车原地转向、倒车曲率照走（没有转向机构转不够）。"""
        if not self.differential:
            return {}
        return dict(pivot_rate=self.geo.pivot_rate_rad_s, pivot_accel=self.geo.pivot_accel_rad_s2, reverse_gain=1.0)

    def _diag(self, calibration):
        """诊断：下发曲率与车实际走出的曲率（融合角速度 / 车速），RTK 与融合推算之差，离群、重置次数。"""
        p, now = self.pose, self._last_tick
        st = self.tracker.status()
        k_act = None if p is None or abs(p.speed) < 0.05 else getattr(p, 'yaw_rate', 0.0) / p.speed
        inn = self._innovation
        fresh = inn is not None and now is not None and now - inn[0] <= INNOVATION_STALE_S
        rejected = (calibration or {}).get('rejected') or {}
        return dict(kappa_cmd=st.get('kappa'), kappa_act=k_act,
                    yaw_rate=None if p is None else getattr(p, 'yaw_rate', None),
                    rtk_along_m=inn[1] if fresh else None, rtk_left_m=inn[2] if fresh else None,
                    position_outliers=rejected.get('position_outliers'),
                    position_resets=(calibration or {}).get('position_resets'),
                    jumps=self._jump_count)

    def _jump_events(self):
        """最近的定位跳动：车当时的本地坐标与经纬度、最大差值、向左分量、持续时间、距今多久（秒）。"""
        now = self._last_tick
        out = []
        for j in list(self._jumps)[-JUMP_EVENTS_REPORTED:]:
            if 'x' not in j:
                continue
            # id：这次跳动开始的时刻（车上单调时钟），app 用它认出重复收到的同一次跳动
            out.append(dict(id=round(j['t0'], 2), x=round(j['x'], 3), y=round(j['y'], 3),
                            lat=None if j['lat'] is None else round(j['lat'], 9),
                            lon=None if j['lon'] is None else round(j['lon'], 9),
                            d=round(j['d'], 4), left=round(j['left'], 4), dur=round(j['t1'] - j['t0'], 2),
                            age_s=None if now is None else round(max(0.0, now - j['t1']), 1)))
        return out


def _with(pose, **changes):
    """位姿换几个字段的副本（融合给的是 dataclass，测试替身是 SimpleNamespace）。"""
    if dataclasses.is_dataclass(pose):
        return dataclasses.replace(pose, **changes)
    out = type(pose)(**vars(pose))
    for key, value in changes.items():
        setattr(out, key, value)
    return out


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
