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
from line_cascade import REASON_TEXT, SHARED_FIELDS, CascadeConfig, ImuAlignment, ImuLineTracker
from line_tracker import LineConfig, LineTracker
from path_tracker import DIFF_REVERSE_MPS, DIFF_SETTLE_S, PathConfig, SectionTracker, differential_config
from route_operations import RouteOperations
from rtk_hold_budget import DegradedSpeedLimiter
from paint_quality import RTK_LOST_PAINT_STOP_S
from um982 import INS_MAX_PDOP
from gnss_latency import GnssLatencyEstimator
from robot_core.config import CONTROL_DT, PROFILE

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
# 见 docs/GNSS_FUSION.md。pdop_high / pdop_missing 不是融合给的：单天线 PDOP 不合格的帧按没有解交给融合，
# 后面的 rtk_fixed_timeout 等都由它而来，任务层把它插到最前面（_fusion_pose），App 显示真实原因
BLOCKING_REASONS = ('pdop_high', 'pdop_missing', 'heading_uninitialized', 'imu_stale', 'imu_mismatch', 'wheel_stale',
                    'wheel_motion_conflict', 'rtk_fixed_timeout', 'position_reset', 'heading_uncertain',
                    'position_uncertain')
PDOP_REASONS = ('pdop_high', 'pdop_missing')
# PDOP 不合格时由它带出来的原因（融合只看到没有解）：报原因时不再列出，免得把「PDOP 太大」说成「固定解中断」
PDOP_CONSEQUENCES = ('rtk_fixed_timeout', 'rtk_hold', 'rtk_hold_pending', 'fix_unavailable')
# 测向浮点（无效）时航向只靠陀螺和行驶轨迹维持，融合的航向不确定度一过 0.5° 就判不可控制。
# 主、副天线都是固定解（位置可信）时放宽到这么多照样走，原因里加 gyro_hold（2026-09-27 用户要求：
# 走着走着测向变浮点不能停，要靠 IMU 维持）
GYRO_HOLD_SIGMA_YAW_DEG = 2.0
FIX2_MAX_AGE_S = 1.0        # 副天线固定解这么久内来过就算还固定着
# 因定位问题暂停后，定位连续可用这么久就自动接着走（操作员停止、控制权被切走照样结束，不会自动恢复）
AUTO_RESUME_S = 1.0
# 定位问题暂停（推算误差预算不足、定位不能用）时车速按这个减速度降到 0，角速度同比例降（不再按坏定位修方向），
# 原地转按 geo.pivot_accel_rad_s2 降。2026-10-02 遮天线实测：指令一拍 0.8 → 0，驱动器按 3 m/s² 急刹，停住后两轮
# 反转溜回 9–11 cm、约 2 s 才稳，恢复起步又先往后 2–3 cm 再前冲；同一批里路线终点、手动松手（都是 1–2 s 的斜坡）
# 停后最多 0.03 m/s。1.5 m/s² 与手动松手渐停相同：0.8 m/s 约 0.53 s、21 cm 停住；推算误差预算按 0.5 m/s² 预估停车，
# 仍偏保守。
# 2026-10-04 用户：「暂停作业也要缓和刹住，只有急停采用急刹」。现在任务层发出的车速每拍最多降这么多（转弯曲率不变）：
# 操作员暂停 / 停止、偏离中止、存档失败、降级与预算限速、横偏降速都一样，车没停下前不交还控制权（原来任务一结束
# 下一拍就发 0 并交还，驱动器按 3 m/s² 刹停）。急停由仲裁直接切 ESTOP，不经过这里。
# 不快于驱动节点给导航指令的斜坡（车型 driver.acceleration_m_s2，Gen5 OID 节点 1.0 m/s²，加减速同一斜率；给 0 才立即
# 清零、驱动器按 3 m/s² 刹）：按 1.5 降时驱动节点只跟得上 1.0，任务层降到 0 那一拍还剩约 0.3–0.5 m/s 被清零，
# 仿真底盘 2.4 m/s²。Gen5 实际 1.0 m/s²：1.5 m/s 约 1.5 s、1.1 m 停住，0.8 m/s 约 0.8 s、0.32 m
SOFT_STOP_DECEL_MPS2 = min(1.5, PROFILE['driver']['acceleration_m_s2'])
# 蠕行速度以下（到点前 path_tracker min_mps 0.06）给零不算急刹，照旧直接停：渐停 3 拍会让到点多走约 2 mm
SOFT_STOP_CREEP_MPS = 0.07
STOPPING_NOTE = '，正在停车'
# 定点画线 start 没带 cruise 时的车速（用户定的画线速度）；不再沿用上一次任务（跑过别的速度的路线也不影响）
LINE_CRUISE_MPS = 0.8
# 固定解中断（单点、浮点、中转断流）时按推算维持（融合 rtk_hold_s）给操作员的话；恢复后改回「行驶中」。
# Gen5：不降速，满 8 s 先停喷、再走 1 s 停车（paint_quality.RTK_LOST_PAINT_STOP_S，line_node 设 rtk_hold_s = 9 s），恢复后自动继续
# 手动「IMU 维持」测试的时长（也是车端支持该测试要求的最短维持时间）
RTK_HOLD_S = 5.0
RTK_NOTICE_LOST_HOLD = '卫星固定解中断，按 IMU 推算继续行驶'
RTK_NOTICE_LOST_PAINT = '卫星固定解中断超过 %g s，已停喷，即将停车' % RTK_LOST_PAINT_STOP_S
RTK_NOTICE_LOST = '卫星固定解中断，已停喷并原地停车，恢复后自动继续'
# 上面三种是单天线 PDOP 不合格（um982.INS_MAX_PDOP，用户规则）引起的：前半句换成真实原因，不说成「固定解中断」
SINGLE_FIX_FRESH_S = 1.5    # 最近一帧单天线定位这么久内来过，它的 PDOP 判断才算数
PDOP_TEXT_HIGH = '单天线 PDOP %.1f（要小于 %g）'
PDOP_TEXT_MISSING = '单天线没有收到 PDOP'
PDOP_NOTICE_LOST_HOLD = '%s，卫星位置不用，按 IMU 推算继续行驶'
PDOP_NOTICE_LOST_PAINT = '%%s，超过 %g s 已停喷，即将停车' % RTK_LOST_PAINT_STOP_S
PDOP_NOTICE_LOST = '%s，卫星位置不用，已停喷并原地停车，恢复后自动继续'
PDOP_NOTICE_PREFIXES = ('单天线 PDOP', PDOP_TEXT_MISSING)
# 跟踪器因定位暂停（上面几种以外的原因，如定位抖一下）时不能还显示「行驶中」（2026-10-01 实车：暂停 24 s 屏幕
# 一直是「行驶中」）；恢复后起步检查不过、自动继续不了时说清楚原因（原因代码由 app 翻译）
TRACKER_PAUSED_NOTICE = '定位暂时不能用，已原地停车，恢复后自动继续'
RESUME_FAILED_NOTICE = '定位已恢复，但不能自动接着走：%s；请停止后重新开始'
# 定位降级（没有有效测向、传感器接管）时限速。2026-10-05 起「起步前卫星确认」「定位与卫星差太多停车」
# 「推算误差预算停车」都已删掉（用户：「原来的检查必须删除，这种检查是不合理的」）；2026-10-06 晚起没有固定解
# （rtk_hold_pending / rtk_hold）也不降速（用户：「去掉 5 秒停车，也不降速」）
DEGRADED_MOTION_REASONS = ('heading_uninitialized', 'heading_unavailable', 'gyro_only', 'gyro_hold',
                           'wheel_hold', 'imu_hold', 'imu_hold_test')

# 单天线起步定车头（2026-10-07 用户：「起步配合只需要轮子往前动一下就知道车头了」）：单天线没有测向，车头还没定时，
# 开始作业（定点画线、连线路线）或 App 点「定车头」先按这个速度直着往前走，融合按天线行驶轨迹定出车头
# （单天线配置走 0.5 m，按卫星位移对 IMU + 轮程推算位移的夹角定，不要求走直，见 line_node.SINGLE_COURSE_INIT_M）后渐停，
# 再照常规划、起步
ORIENT_SPEED_MPS = 0.25
ORIENT_MAX_M = 1.5          # 走这么远还没定出车头就停下报错
ORIENT_TIMEOUT_S = 15.0
ORIENT_STOP_WAIT_S = 3.0    # 定出车头后最多等这么久停稳（轮速一直报在动也接着往下走）
ORIENT_MESSAGE = '单天线：往前直走一点定车头'
# 定车头时位置要是固定解、传感器都在：除了航向本身没定带出来的几项，有这些原因就不走 / 停下
ORIENT_BLOCKING = ('pdop_high', 'pdop_missing', 'imu_stale', 'imu_mismatch', 'wheel_stale', 'wheel_motion_conflict',
                   'rtk_fixed_timeout', 'position_reset', 'rtk_hold', 'rtk_hold_pending', 'rtk_float', 'fix_unavailable',
                   'position_uninitialized', 'position_outlier')
ANTENNA_TEXT = {'single': '单天线', 'dual': '双天线'}
# 卫星定位的固定延迟（接收机内部、平板转发路上的固定那段）由 gnss_latency 在线估：每次从停稳起步，卫星位移和轮程 + IMU
# 推算的位移对一下。估出来后卫星数据（定位、副天线、测向、地速）的时间都减去它再交给融合；还没估出来时按这个
# （um982.GnssClock 已按接收机出解 20 ms 算过）。只往前挪：时间戳反而偏晚（负的）不补
GNSS_LATENCY_INITIAL_S = 0.0
GNSS_KINDS = ('fix', 'fix2', 'heading', 'vel')


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
    if any(r in blocking for r in PDOP_REASONS):
        blocking = [r for r in blocking if r not in PDOP_CONSEQUENCES]
    blocking = blocking or list(reasons)
    return 'pose_unusable' + ('：' + '、'.join(blocking) if blocking else '')


class LineMission:
    def __init__(self, fusion, samples, tracker=None, *, state_path=None, log_dir=None, route_path=None,
                 path_config=None, body=None, nozzle_x=None, single_config=None):
        """fusion 实现 docs 里约定的 GnssFusion 接口；samples 提供 ImuSample 等输入类型（通常就是 gnss_fusion 模块）。

        body = (后轴中心到车头, 到车尾, 半宽) 米，连线路线按车身中心压线；默认取共享底盘配置。
        差速车（tracker.geo.differential，Gen5）所有作业都按水泵（喷头）压线，急弯停车原地转向。nozzle_x：水泵在
        后轴前多少米，或 (前, 左) 米（水泵可以在侧面）；默认取共享配置 drive.nozzle_x_m、drive.nozzle_right_m
        （2026-10-02 用户：定点位置就是水泵位置，作业都要让水泵走在线上）。app 开始作业带的「对准点」优先。
        single_config：单天线流动站的融合配置（杠杆臂是右天线位置，见 line_node.single_antenna_config）；
        None 表示这台车不用单天线，收到单天线定位不交给融合。
        """
        self.fusion, self.samples = fusion, samples
        # 天线模式跟着 fix 消息的 antenna_mode 走（um982.Um982Stream）；换了就给融合换杠杆臂（switch_config）
        self.antenna_mode = 'dual'
        self._antenna_configs = {'dual': getattr(fusion, 'config', None), 'single': single_config}
        self._orient = None             # 单天线起步定车头：dict(cmd 定完要接着执行的开始指令, since, dist, ...)
        # 卫星定位固定延迟的在线估计；按天线模式存盘（换接收机、转发路径的延迟不同），重启后直接用上次的值
        self._latency_path = os.path.join(os.path.dirname(state_path), 'gnss_latency.json') if state_path else None
        self._latency_saved = self._load_latency()
        self._latency = GnssLatencyEstimator(self._latency_saved.get('dual', GNSS_LATENCY_INITIAL_S))
        self._pdop = None               # 最近一帧单天线定位的 (时刻, PDOP 或 None, 合格否)，App 显示真实原因用
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
                nozzle_x = (PROFILE['drive']['nozzle_x_m'], -PROFILE['drive'].get('nozzle_right_m', 0.0))
        # 水泵在车体系的 (前, 左)；nozzle_x 是它在后轴前多少米（route_limits 照旧报这一项）
        self.nozzle = route_plan.nozzle_point(nozzle_x) if self.differential else None
        self.nozzle_x = self.nozzle[0] if self.differential else None
        self.state_path, self.log_dir, self.route_path = state_path, log_dir, route_path
        self.task = 'line'              # 当前（或最近一次）任务：line / route
        self.points = {}                # 'A'/'B' -> (lat, lon)
        self.route = None               # route_plan.RouteSpec，路点存经纬度
        self.route_rev = 0
        self._route_cache = None
        self._antenna = None            # 最近一次天线定位 (t, lat, lon, sol, sigma)
        self._imu = None                # 最近一帧 IMU (t, 偏航 rad, 陀螺 z rad/s)，差速车定点画线的内环用
        self._heading_baseline = None   # 最近一帧双天线测向 (测量时刻, 基线长 m)：直线 IMU 为主时基线不对就不用卫星修正
        self._rtk_quality = None        # 最近一帧主天线定位 (测量时刻, σ m, 卫星数, 两天线距离误差 m)：直线卫星纠偏只在合格时用
        self._imu_alignment = ImuAlignment()    # 上一条线对好的 IMU → 地面方向，开头基线不对时借（跨作业保留）
        self._fix2_t = None             # 副天线最近一次固定解的时刻
        self._usable_since = None       # 暂停中定位从什么时候起一直可用（自动继续用）
        self._resume_failed = None      # 自动继续时起步检查没过的原因代码（报给操作员）
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
        self._hold_speed_limit = None
        self._speed_hold = DegradedSpeedLimiter()
        self._sent_motion = deque()    # 最近已下发的速度/转速，停车预估覆盖执行延迟里的指令。
        self._soft_prev = None         # 上一拍发出的 (时刻, vx, wz)，渐停用
        self._soft_at = None           # 这一拍已经做过渐停限幅的时刻
        self.braking = False            # 任务已结束 / 已暂停但车还在按渐停减速：节点照常下发、不交还控制权
        self._imu_hold_test = None      # 手动测试只在当前运行任务中生效，不持久化。
        self._imu_hold_gnss_after = -math.inf  # 测试区间的迟到 GNSS 永远不回放。
        self.release_pending = False    # 任务结束后要把 NAVIGATION 交还给仲裁
        self._log = self._log_file = None
        self.integral0 = self._load_integral()
        self._load_route()
        self.last_motion_mode = None
        self.operations = RouteOperations(self)

    # ---- 传感器输入 ----
    def on_gnss(self, m):
        s, kind = self.samples, m.get('kind')
        raw_t = m.get('t')
        if kind in GNSS_KINDS and self._latency.value > 0:
            m = dict(m, t=m['t'] - self._latency.value)
        if kind in GNSS_KINDS and m['t'] < self._imu_hold_gnss_after:
            return
        inf = lambda v: math.inf if v is None else v  # noqa: E731
        if kind == 'fix':
            mode = 'single' if m.get('antenna_mode') == 'single' else 'dual'
            if mode != self.antenna_mode:
                self._switch_antenna(mode)
            if mode == 'single' and self._antenna_configs['single'] is None:
                return                  # 这台车没有单天线的杠杆臂：按主天线算会差 0.41 m，不用
            if mode == 'single':
                self._pdop = (m['t'], m.get('pdop'), bool(m.get('pdop_ok', True)))
            # 单天线 PDOP 不小于 3 一律不用（用户规则，um982.INS_MAX_PDOP）：按没有解交给融合（IMU 推算维持，满 8 s 停喷、9 s 停车）
            sol = m['sol'] if m.get('pdop_ok', True) else 'NONE'
            diff_age = m['diff_age']
            if diff_age is None and m.get('source') == 'INS.NAVI' and sol in RTK_FIXED_SOLS + ('NARROW_FLOAT',):
                # 单天线的差分龄期取自 GGA，有的接收机不填；GGA 报 RTK 固定 / 浮点本身就说明差分在用，按新鲜的算
                diff_age = 0.0
            self._antenna = (m['t'], m['lat'], m['lon'], m['sol'], max(m['sigma_n'], m['sigma_e']))
            pb = m.get('position_baseline') or {}
            pair = pb.get('error_m') if pb.get('epoch_aligned') else None
            self._rtk_quality = (m['t'], math.hypot(m['sigma_n'], m['sigma_e']), m.get('sats'), pair)
            if sol in RTK_FIXED_SOLS:
                self._check_innovation(m)
                self._latency_fix(raw_t, m)
            self.fusion.add_fix(s.GnssFix(t=m['t'], lat_deg=m['lat'], lon_deg=m['lon'], height_m=m['height'],
                                          sol=sol, sigma_n=m['sigma_n'], sigma_e=m['sigma_e'],
                                          sigma_u=m['sigma_u'], diff_age_s=inf(diff_age), sats=m['sats'],
                                          position_variance_scale=m.get('position_variance_scale',1.),
                                          inertial_only=self._constant_arc_speed() and sol not in RTK_FIXED_SOLS))
        elif kind == 'fix2':
            # 副天线固定状态与相对位置进入融合；相对偏差只降权，不否决主天线固定解。
            if m.get('sol') in RTK_FIXED_SOLS:
                self._fix2_t = m['t']
            add_fix2 = getattr(self.fusion, 'add_fix2', None)
            if callable(add_fix2) and m.get('lat') is not None and m.get('lon') is not None:
                add_fix2(s.GnssFix(t=m['t'], lat_deg=m['lat'], lon_deg=m['lon'], height_m=m.get('height', 0.0),
                                   sol=m['sol'], sigma_n=m.get('sigma_n', 0.01), sigma_e=m.get('sigma_e', 0.01),
                                   sigma_u=m.get('sigma_u', 0.02), diff_age_s=inf(m.get('diff_age')),
                                   sats=m.get('sats', 0)))
        elif kind == 'heading':
            self._heading_baseline = (m['t'], m.get('length'))
            # 无效测向也按测量时刻告知融合，撤销旧测向的新鲜许可并中断恢复观察窗口。
            # 用 NONE 标记，绝不把被 rtk_node 拒绝的角度作为观测更新。
            self.fusion.add_heading(s.DualHeading(t=m['t'], heading_deg=m['heading'], pitch_deg=m['pitch'],
                                                  length_m=m['length'], sol=m['sol'] if m.get('valid') is True else 'NONE',
                                                  sigma_heading_deg=m['sigma_heading'], sats=m['sats']))
        elif kind == 'vel':
            self.fusion.add_velocity(s.GnssVelocity(t=m['t'], speed_mps=m['speed'], track_deg=m['track'],
                                                    sol=m['sol']))

    def _switch_antenna(self, mode):
        """单天线 ↔ 双天线：两根天线位置差 0.41 m、单天线又没有测向，融合整个重来（原点留着，见 GnssFusion.switch_config）。
        正在作业就停下（渐停）；路线断点按经纬度存，换完照样能续画。"""
        old = self.antenna_mode
        current = getattr(self.fusion, 'config', None)
        if self._antenna_configs.get(old) is not None and current is not None:
            # 运行中改过的融合配置（App 坡度开关，attitude_controls 直接换 fusion.config）记回这一套，换回来不丢；
            # 这台车没有这一套（单天线配置为 None，融合没换过）时不记，免得把双天线配置记成单天线的
            self._antenna_configs[old] = current
        self.antenna_mode = mode
        config = self._antenna_configs.get(mode)
        if config is None:
            return
        text = '天线换成%s' % ANTENNA_TEXT[mode]
        if self.phase == 'ORIENTING':
            self._end_orient(text + '，已停止定车头')
        elif self.phase != 'IDLE':
            self.tracker.stop('antenna_switch')
            self._finish(text + '，作业已停止')
        switch = getattr(self.fusion, 'switch_config', None)
        if callable(switch):
            switch(config)
        self._latency.reset(self._latency_saved.get(mode, GNSS_LATENCY_INITIAL_S))   # 换了接收机和转发路径：用这一路上次的值重估
        self._imu_alignment = ImuAlignment()    # 借来的 IMU → 地面方向是按旧天线对的
        self._innovation = self._jump = None

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
            self._latency.on_imu(t, yaw, gyro_z)
            self._imu_alignment.feed_imu(t, yaw)   # 累计转角：换算跨线带过去时按转角校准

    def on_wheel(self, t, speed, yaw_rate):
        self.fusion.add_wheel(self.samples.WheelSample(t=t, speed=speed, yaw_rate=yaw_rate))
        before = self._latency.value
        self._latency.on_wheel(t, speed)
        self._save_latency(before)

    def on_wheel_motion(self, sample):
        self.fusion.add_wheel(sample)
        before = self._latency.value
        self._latency.on_wheel(sample.t, sample.speed, getattr(sample, 'end_t', None), getattr(sample, 'end_speed', None))
        self._save_latency(before)

    def _latency_fix(self, raw_t, m):
        """卫星固定解交给延迟估计（按接收机标的原始时间，本地米，现在用的天线杠杆臂）。"""
        try:
            x, y = self.fusion.to_local(m['lat'], m['lon'])
            lever = getattr(self.fusion, 'antenna_lever', None)
            lever = lever() if callable(lever) else self.fusion.config.antenna_xy
        except (AttributeError, ValueError, TypeError):
            return
        before = self._latency.value
        self._latency.on_fix(raw_t, x, y, lever)
        self._save_latency(before)

    def _load_latency(self):
        try:
            with open(self._latency_path, encoding='utf-8') as fh:
                data = json.load(fh)
            return {k: float(v) for k, v in data.items() if k in ANTENNA_TEXT and isinstance(v, (int, float))
                    and math.isfinite(v)}
        except (TypeError, OSError, ValueError, AttributeError):
            return {}

    def _save_latency(self, before):
        """换上的延迟变了 1 ms 以上就存一下（这一路天线模式的）；存不下只是重启后从头估，不影响开车。"""
        value = self._latency.value
        if not self._latency_path or abs(value - self._latency_saved.get(self.antenna_mode, before)) < 0.001:
            return
        self._latency_saved[self.antenna_mode] = value
        try:
            os.makedirs(os.path.dirname(self._latency_path) or '.', exist_ok=True)
            tmp = self._latency_path + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as fh:
                json.dump(dict(self._latency_saved), fh)
            os.replace(tmp, self._latency_path)
        except OSError:
            pass

    # ---- 操作 ----
    def command(self, cmd, now):
        """返回 (是否接受, 说明)。start 被接受只表示开始申请控制权，结果看 tick/status。"""
        kind = cmd.get('cmd')
        if kind == 'route_speed':
            try:
                from live_cruise import change
                return True,change(self.operations,cmd,now)
            except (ValueError,TypeError,KeyError,OSError) as exc:return False,str(exc)
        if kind == 'route_imu_hold':
            return self._start_imu_hold_test(cmd, now)
        if kind == 'route_orient':
            return self._begin_orient(None, now)
        if kind in ('start', 'route_start', 'route_start_current') and self._needs_orient(cmd, now):
            return self._begin_orient(cmd, now)
        if kind=='route_field_start':
            try:return True,self.operations.start_field(cmd,now)
            except (ValueError,TypeError,KeyError) as exc:return False,str(exc)
        if kind in ('route_pause', 'route_preview', 'route_resume', 'route_rework', 'route_end'):
            try:
                if kind == 'route_pause':
                    message = self.operations.pause(cmd, now)
                    self._end_imu_hold_test(now, 'cancelled', '作业已暂停')
                elif kind == 'route_preview':
                    message = self.operations.prepare(cmd, now)
                elif kind == 'route_end':
                    message = self.operations.end(cmd, now)
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
            if self.phase == 'ORIENTING':
                self._end_orient('操作员停止')
                return True, '已停止'
            self._end_imu_hold_test(now, 'cancelled', '作业已停止')
            self.tracker.stop('operator')
            if self.phase in ('SELECTING', 'RUNNING', 'PAUSING', 'PAUSED') or self.operations.held:
                self._finish('操作员停止', state='CANCELLED')
            return True, '已停止'
        return False, '未知指令'

    def _pdop_reason(self, now):
        """单天线最近一帧（SINGLE_FIX_FRESH_S 内）PDOP 不合格时的原因代码：有 PDOP 值是 pdop_high（um982 只报新鲜的值，
        有值又不合格就是不小于 3），没收到是 pdop_missing；其余 None。"""
        p = self._pdop
        if self.antenna_mode != 'single' or p is None or p[2] or not 0.0 <= now - p[0] <= SINGLE_FIX_FRESH_S:
            return None
        return 'pdop_high' if p[1] is not None else 'pdop_missing'

    def _fusion_pose(self, now):
        """融合位姿；单天线 PDOP 不合格时把它插到原因最前面（融合只看到没有解，报的 rtk_hold 等都由它而来），
        能不能用不变。"""
        pose = self.fusion.pose(now)
        why = None if pose is None else self._pdop_reason(now)
        if why is None or why in pose.reasons:
            return pose
        return _with(pose, reasons=(why,) + tuple(pose.reasons))

    def _pdop_text(self, now):
        """单天线 PDOP 不合格时操作员提示的前半句（真实原因），否则 None。"""
        why = self._pdop_reason(now)
        if why == 'pdop_high':
            return PDOP_TEXT_HIGH % (self._pdop[1], INS_MAX_PDOP)
        return PDOP_TEXT_MISSING if why == 'pdop_missing' else None

    # ---- 单天线定车头 ----
    def _needs_orient(self, cmd, now):
        """开始作业前要不要先往前走一点定车头：单天线、闲着、融合有位置但航向还没定。开始指令本身明显不成立
        （没记 A、B，车上没有路线）时不动车，照常交给开始流程报错。"""
        if self.antenna_mode != 'single' or self.phase != 'IDLE' or self.operations.held:
            return False
        if cmd.get('cmd') == 'start' and not {'A', 'B'} <= set(self.points):
            return False
        if cmd.get('cmd') != 'start' and self.route is None and 'route' not in cmd:
            return False
        pose = self._fusion_pose(now)
        return pose is not None and 'heading_uninitialized' in pose.reasons

    @staticmethod
    def _orient_problem(pose):
        """定车头时不能走的原因（原因代码，同 pose_problem 的格式）；能走返回 None。"""
        if pose is None:
            return '还没有定位'
        bad = [r for r in ORIENT_BLOCKING if r in pose.reasons]
        if any(r in bad for r in PDOP_REASONS):
            bad = [r for r in bad if r not in PDOP_CONSEQUENCES]
        return 'pose_unusable：' + '、'.join(bad) if bad else None

    def _begin_orient(self, cmd, now):
        """cmd：定完车头接着执行的开始指令；None 是 App 单点「定车头」，定完停下交还控制权。"""
        if self.antenna_mode != 'single':
            return False, '双天线有测向，不用定车头'
        if self.phase != 'IDLE' or self.operations.held:
            return False, '先停止当前任务'
        pose = self._fusion_pose(now)
        if pose is not None and 'heading_uninitialized' not in pose.reasons:
            return True, '车头已确定'
        problem = self._orient_problem(pose)
        if problem:
            return False, problem
        self._orient = dict(cmd=cmd, since=now, dist=0.0, driving=False, done=None, stop_t=None)
        self.phase, self.select_since, self.message, self.release_pending = 'ORIENTING', now, ORIENT_MESSAGE, False
        return True, ORIENT_MESSAGE

    def _end_orient(self, message):
        """定车头结束（成功后没有要接着开始的、失败、操作员停止）：回到闲着，交还控制权（车没停稳的由渐停接着降）。"""
        self._orient = None
        self.phase, self.message, self.release_pending = 'IDLE', message, True

    def _orient_tick(self, now, motion_mode, dt):
        o = self._orient
        pose = self._fusion_pose(now)
        if motion_mode != 'NAVIGATION':
            if o['driving']:
                self._end_orient('控制权被切走（%s），已停止定车头' % motion_mode)
            elif now - self.select_since > SELECT_TIMEOUT_S:
                self._end_orient('控制权申请失败（当前 %s）' % motion_mode)
            else:
                return 0.0, 0.0, True
            return 0.0, 0.0, None
        o['driving'] = True
        if o['done'] is None:
            problem = self._orient_problem(pose)
            if pose is not None and 'heading_uninitialized' not in pose.reasons:
                o['done'] = 'ok'
            elif problem:
                o['done'] = problem
            elif o['dist'] >= ORIENT_MAX_M or now - o['since'] > ORIENT_TIMEOUT_S:
                o['done'] = 'far'
            else:
                o['dist'] += abs(pose.speed) * dt
                # 不打方向：两轮同速往前走（定车头不要求走直，车身摆由 IMU 记着，见 gnss_fusion._imu_course）
                return ORIENT_SPEED_MPS, 0.0, None
            o['stop_t'] = now
        # 渐停（_limit_decel）到停稳再往下走
        moving = self._soft_prev is not None and abs(self._soft_prev[1]) > 0.0
        rolling = pose is not None and abs(pose.speed) > 0.02
        if (moving or rolling) and now - o['stop_t'] < ORIENT_STOP_WAIT_S:
            return 0.0, 0.0, None
        done, cmd = o['done'], o['cmd']
        if done == 'far':
            self._end_orient('往前走了 %.1f m 还没定出车头（卫星不是固定解或定位跳动），已停车' % o['dist'])
        elif done != 'ok':
            self._end_orient('定车头中止：%s' % done)
        elif cmd is None:
            self._end_orient('车头已确定（单天线，往前走了 %.2f m）' % o['dist'])
        else:
            self._orient, self.phase = None, 'IDLE'
            ok, why = self.command(cmd, now)
            if not ok:
                self._end_orient('车头已确定，但没能开始：%s' % why)
            elif self.phase == 'SELECTING':
                self.select_since = now
        return 0.0, 0.0, None

    def _imu_hold_supported(self):
        cfg = getattr(self.fusion, 'config', None)
        return self.differential and getattr(cfg, 'yaw_input', None) == 'angle' and getattr(cfg, 'rtk_hold_s', 0) >= RTK_HOLD_S

    def _start_imu_hold_test(self, cmd, now):
        if not self._imu_hold_supported():
            return False, '车端尚未支持 IMU 维持 %g 秒测试' % RTK_HOLD_S
        try:
            self.operations.check_identity(cmd)
        except (ValueError, TypeError, KeyError) as exc:
            return False, str(exc)
        request_id = cmd.get('id')
        if not isinstance(request_id, str) or not request_id:
            return False, 'IMU 测试缺少指令标识'
        test = self._imu_hold_test
        if test and test['id'] == request_id:
            return True, '此 IMU 测试指令已处理'
        if self.task != 'route' or self.phase != 'RUNNING' or self.last_motion_mode != 'NAVIGATION' or self.tracker.st.state != 'RUNNING':
            return False, '请先开始路线作业，行驶中才能测试'
        if test and test['state'] == 'running':
            return False, 'IMU 维持测试正在进行，请等待结束'
        pose = self._control_pose(now)
        problem = pose_problem(pose)
        if problem:
            return False, problem
        cfg = self.fusion.config
        if self._imu is None or now - self._imu[0] > cfg.sensor_timeout_s or any(r in pose.reasons for r in ('imu_stale', 'imu_mismatch')):
            return False, 'IMU 航向不可用，不能测试'
        self._imu_hold_test = dict(id=request_id, state='running', started=now, until=now + RTK_HOLD_S, reason='')
        self._imu_hold_gnss_after = now + RTK_HOLD_S
        self._innovation = None
        return True, '已切换 IMU + 轮速维持，%g 秒后自动接回 RTK' % RTK_HOLD_S

    def _end_imu_hold_test(self, now, state, reason):
        test = self._imu_hold_test
        if test and test['state'] == 'running':
            test.update(state=state, ended=now, reason=reason)
            self._imu_hold_gnss_after = min(test['until'], now)

    def _imu_hold_test_status(self):
        if not self._imu_hold_supported():
            return None
        test = self._imu_hold_test
        if test is None:
            return dict(state='idle', duration_s=RTK_HOLD_S, remaining_s=0.0, id=None, reason='')
        now = self._last_tick if test['state'] == 'running' else test['ended']
        remaining = max(0.0, min(RTK_HOLD_S, test['until'] - (now if now is not None else test['started'])))
        return dict(state=test['state'], duration_s=RTK_HOLD_S, remaining_s=round(remaining, 1) if test['state'] == 'running' else 0.0,
                    id=test['id'], reason=test['reason'])

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
        """记 A、B 记的那一点：差速车是水泵（定点画线也让水泵压在 A→B 上），阿克曼车是 track_x_m。"""
        if self.differential:
            return route_plan.nozzle_at(pose.x, pose.y, pose.yaw, *self.nozzle)
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
        pose = self._fusion_pose(now)
        self._hold_speed_limit = None
        if pose is None:
            return pose
        test = self._imu_hold_test
        if test and test['state'] == 'running' and now < test['until']:
            # 标明实际控制来源；传感器中断和航向超限检查照常执行。测试故意不用卫星，不按卫星可信放行
            pose = _with(pose, quality='DEAD_RECKONING', reasons=tuple(pose.reasons) + ('imu_hold_test', 'rtk_hold'),
                         rtk_trusted=False)
        if getattr(pose, 'rtk_trusted', False):
            # 卫星可信（gnss_fusion.TRUSTED_FRESH_S，用户规则）：位置、航向以卫星为准，不限速
            return pose
        if not pose.usable_for_control:
            reasons = tuple(getattr(pose, 'reasons', ()))
            blocking = [r for r in BLOCKING_REASONS if r in reasons]
            fix2 = self._fix2_t is not None and now - self._fix2_t <= FIX2_MAX_AGE_S
            underway = self.phase == 'RUNNING' and self.tracker.st.state == 'RUNNING'
            if (underway and blocking == ['heading_uncertain'] and 'heading_unavailable' in reasons and fix2
                    and pose.quality == 'RTK_FIXED' and pose.sigma_yaw <= math.radians(GYRO_HOLD_SIGMA_YAW_DEG)):
                pose = _with(pose, usable_for_control=True, reasons=reasons + ('gyro_hold',))
            else:
                return pose
        cfg = getattr(self.fusion, 'config', None)
        if getattr(cfg, 'sensor_failover_enabled', False) and any(r in pose.reasons for r in ('wheel_hold', 'imu_hold')):
            # 没有固定解（rtk_hold）不再限速（2026-10-06 晚用户）；轮速 / IMU 断了由另一路接管时照旧限速
            cap = cfg.sensor_hold_speed_mps
            self._hold_speed_limit = min(cap, self._hold_speed_limit) if self._hold_speed_limit is not None else cap
        return pose

    def _constant_straight_speed(self):
        return self.task == 'route' and isinstance(self.tracker, SectionTracker) and self.tracker.constant_speed_here()

    def _constant_arc_speed(self):
        return self.task == 'route' and isinstance(self.tracker, SectionTracker) and self.tracker.constant_arc_speed_here()

    def _stable_speed_limit(self, now, pose, dt):
        """IMU 接管期间保持一致的低速策略，预算更严仍立即收紧，安全停车不被迟滞。"""
        requested = self._hold_speed_limit
        if not self.differential:
            return requested
        if self._constant_straight_speed() or self._constant_arc_speed():
            # 用户指定直线按设定速度、圆弧固定0.5：不叠加信号降级或恢复限速。
            # 不改定位许可；真正不可控仍明确给零速，不能绕过暂停/故障保护。
            self._speed_hold.reset()
            return None if pose.usable_for_control else 0.
        reasons = getattr(pose, 'reasons', ())
        degraded = any(r in reasons for r in DEGRADED_MOTION_REASONS)
        if not pose.usable_for_control:
            requested = 0.
        elif degraded:
            cfg = getattr(self.fusion, 'config', None)
            cap = getattr(cfg, 'sensor_hold_speed_mps', .3) if any(r in reasons for r in
                    ('wheel_hold', 'imu_hold')) else getattr(cfg, 'gnss_hold_speed_mps', .35)   # 没有测向、陀螺维持
            requested = cap if requested is None else min(cap, requested)
        healthy = pose.usable_for_control and not degraded
        previous = (abs(self._soft_prev[1]) if self._soft_prev is not None and now-self._soft_prev[0] <= .2
                    else max(abs(pose.speed), abs(self.tracker.st.v_cmd)))
        return self._speed_hold.update(now, requested, healthy=healthy, cruise=self.cruise,
                                       previous_speed=previous, dt=dt)

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
            # 横偏、沿线距离、起止都按水泵量（A、B 也是按水泵记的，见 _track_point）
            f, l = self.nozzle
            shared.update(track_x_m=f, track_y_m=l)
            return ImuLineTracker(CascadeConfig(cruise_mps=cruise, nozzle_x_m=f, nozzle_y_m=l, **shared), self.geo)
        return LineTracker(dataclasses.replace(self.line_cfg, cruise_mps=cruise), self.geo)

    def _sync_imu(self, tracker, now):
        """IMU 航向内环要此刻的时钟、最近一帧 IMU（与 IMU 回调、融合同一个单调时钟）和融合用的天线杠杆臂
        （航向跳变时按 IMU 航向改算位置）；别的控制器没有 sync。"""
        sync = getattr(tracker, 'sync', None)
        if sync is not None:
            sync(now, self._imu, self._antenna_lever())
            baseline = getattr(tracker, 'set_heading_baseline', None)
            if callable(baseline) and self._heading_baseline is not None:
                baseline(*self._heading_baseline)
            quality = getattr(tracker, 'set_rtk_quality', None)
            if callable(quality) and self._rtk_quality is not None:
                quality(*self._rtk_quality)
            share = getattr(tracker, 'share_imu_alignment', None)
            if callable(share):
                share(self._imu_alignment)
            nozzle = getattr(tracker, 'set_nozzle', None)
            if callable(nozzle):
                forward, left, _ = self._job_nozzle()
                field = (self.operations.record or {}).get('field_context', {})
                if ((self.operations.record or {}).get('field_plan') and isinstance(field, dict)
                        and all(isinstance(field.get(k), (int, float)) and math.isfinite(field[k])
                                for k in ('forward', 'right'))):
                    forward, left = field['forward'], -field['right']
                nozzle(forward, left)

    def _job_nozzle(self):
        """(前, 左, 是否任务里记的) 这次作业的水泵：路线作业记在任务里的（开始时 app 给的对准点，没给是共享配置的
        水泵；续画、补画照用），定点画线和没记的旧任务是共享配置的水泵。场地作业的水泵另看 field_context。"""
        v = (self.operations.record or {}).get('route_nozzle')
        if self.task == 'route' and isinstance(v, dict) and all(
                isinstance(v.get(k), (int, float)) and math.isfinite(v[k]) for k in ('forward', 'right')):
            return v['forward'], -v['right'], True
        f, l = self.nozzle or (0.0, 0.0)
        return f, l, False

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
        nozzle = cmd.get('nozzle')
        if nozzle is not None and not (self.differential and isinstance(nozzle, dict)
                                       and all(isinstance(nozzle.get(key), (int, float)) and math.isfinite(nozzle[key])
                                               for key in ('forward', 'right'))):
            return False, '喷头位置无效' if self.differential else '这台车不按喷头压线'
        # 差速车按水泵压线：app 给了「对准点」就用它，不给用共享配置的水泵；都记进任务（续画、补画照用）
        if nozzle:
            route_nozzle = dict(forward=float(nozzle['forward']), right=float(nozzle['right']))
        elif self.differential:
            route_nozzle = dict(forward=self.nozzle[0], right=-self.nozzle[1])
        else:
            route_nozzle = None
        planner_nozzle = (route_nozzle['forward'], -route_nozzle['right']) if route_nozzle else None
        try:
            spec = self._route_local()
            k = self.line_cfg.track_x_m     # 与直线相同：控制点（默认后轴中心）压在路线上
            car = (pose.x + k * math.cos(pose.yaw), pose.y + k * math.sin(pose.yaw), pose.yaw)
            planner = route_plan.plan_current if cmd.get('cmd') == 'route_start_current' else route_plan.plan
            info = {}
            sections, entry_m = planner(spec, car, self.min_route_radius, body=self.body, nozzle_x=planner_nozzle,
                                        info=info)
            from live_cruise import label_sections
            label_sections(sections,entry_m,spec.cruise)
            cfg = dataclasses.replace(self.path_cfg, cruise_mps=spec.cruise, track_x_m=k)
            tracker = SectionTracker(sections, cfg, self.geo, **self.section_options(route=True))
        except ValueError as exc:
            return False, str(exc)
        ok, why = tracker.check_start(pose)
        if not ok:
            return False, why
        try:
            self.operations.begin(sections, entry_m=entry_m, cruise=spec.cruise, **({'route_nozzle': route_nozzle} if route_nozzle else {}))
        except ValueError as exc:
            return False, str(exc)
        self.tracker, self.task, self.cruise = tracker, 'route', spec.cruise
        self.phase, self.select_since = 'SELECTING', now
        turns = '，换向 %d 次' % tracker.cusps if tracker.cusps else ''
        if self.differential:
            # 倒车几段（每段前后各换向一次，tracker.reversals 是换向次数）
            reverses = sum(1 for sec in sections if sec.direction < 0)
            turns = ('，原地转向 %d 次' % tracker.pivots if tracker.pivots else '') + \
                    ('，圆弧掉头 %d 次' % info['arc_turns'] if info.get('arc_turns') else '') + \
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
        length, out['problem'] = route_plan.route_length(spec, self.min_route_radius, nozzle_x=self.nozzle)
        out['length'] = None if length is None else round(length, 3)
        # 连线路线：车身中心压过全部连线要开的总路程（含掉头补角）、换向次数、掉头补角的拐角数
        out['cover'] = route_plan.cover_summary(spec, self.min_route_radius, self.body, nozzle_x=self.nozzle) \
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
    def _check_paint_output_speed(self, vx):
        # 水泵必须核对最终发给底盘的速度，不能只看渐停限幅之前的跟踪器原始指令。
        t=self.tracker
        if self.task=='route' and isinstance(t,SectionTracker):
            child=t.trackers[t.index]
            if child.constant_paint_speed:
                child.paint_speed_ready=(child.paint_speed_ready and child.paint_target_mps is not None
                                         and abs(vx-child.paint_target_mps)<=1e-6)

    def tick(self, now, motion_mode):
        """返回 (vx, wz, select)。select=True/False 表示节点要调用 /motion/navigation，None 不动。
        除急停外的停车都渐停（SOFT_STOP_DECEL_MPS2）：没停下前不交还控制权，节点看 braking 照常下发。"""
        dt = CONTROL_DT if self._last_tick is None else max(0.0, min(0.2, now - self._last_tick))
        vx, wz, select = self._tick(now, motion_mode)
        vx, wz = self._limit_decel(now, motion_mode, vx, wz, dt)
        self._check_paint_output_speed(vx)
        self.braking = (vx, wz) != (0.0, 0.0) and self.phase not in ('RUNNING', 'PAUSING', 'ORIENTING')
        # 任务已结束但车还在渐停：如实告诉操作员，停住后去掉
        if self.braking and not self.message.endswith(STOPPING_NOTE):
            self.message += STOPPING_NOTE
        elif not self.braking and self.message.endswith(STOPPING_NOTE):
            self.message = self.message[:-len(STOPPING_NOTE)]
        if select is False and (vx, wz) != (0.0, 0.0):
            select, self.release_pending = None, True
        # 开/关喷标记：这一拍之后状态不再变，与节点随后发的 /line/trace paint（水泵按它开关）同值。
        # 只给 App 显示，同 /line/trace 一样出错只丢这一次，不影响开车
        try:
            self.operations.mark_paint(self.task == 'route' and self.phase == 'RUNNING'
                                       and self.operations.paint_status()['painting'])
        except Exception:
            pass
        return vx, wz, select

    def _tick(self, now, motion_mode):
        dt = CONTROL_DT if self._last_tick is None else max(0.0, min(0.2, now - self._last_tick))
        self._last_tick = now
        self.last_motion_mode = motion_mode
        if self.phase != 'RUNNING' or motion_mode != 'NAVIGATION':
            self._speed_hold.reset()
        test = self._imu_hold_test
        if test and test['state'] == 'running':
            if now >= test['until']:
                self._end_imu_hold_test(test['until'], 'done', '%g 秒结束，正在接回 RTK' % RTK_HOLD_S)
            elif self.phase != 'RUNNING' or motion_mode != 'NAVIGATION' or self.tracker.st.state != 'RUNNING':
                self._end_imu_hold_test(now, 'cancelled', '作业已暂停或停止，正在接回 RTK')
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
        if self.phase == 'ORIENTING':
            return self._orient_tick(now, motion_mode, dt)
        if self.phase == 'SELECTING':
            if motion_mode == 'NAVIGATION':
                # 申请控制权期间卫星也可能失效；真正下发起步指令前再确认一次。
                problem = pose_problem(pose)
                if problem or not self.tracker.start(pose, integral=self._start_integral()):
                    why = problem or self.tracker.st.reason
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
        limit = self._stable_speed_limit(now, pose, dt)
        if limit is not None and self._soft_prev is not None and now - self._soft_prev[0] <= 0.2:
            # 降级 / 预算限速一下子变低时按渐停减速度逐拍收紧，跟踪器边减速边跟线（发出去的就是它算的）
            limit = max(limit, abs(self._soft_prev[1]) - SOFT_STOP_DECEL_MPS2 * dt)
        self.tracker.speed_limit_mps = limit
        vx, wz = self.tracker.update(pose, dt)
        vx, wz = self._limit_decel(now, motion_mode, vx, wz, dt)
        self._check_paint_output_speed(vx)
        self.operations.update_paint_tail(now)
        self._sent_motion.append((now, vx, wz))
        if self.tracker.st.state != 'RUNNING':
            self._end_imu_hold_test(now, 'cancelled', '作业已暂停或停止，正在接回 RTK')
        while self._sent_motion and self._sent_motion[0][0] < now - 2.0:
            self._sent_motion.popleft()
        # 航向跳变按 IMU 维持 / 超过 5 s 停车等航向（定点画线控制器）、卫星位置不可信按推算维持 / 超时停车（融合）：
        # 报给操作员，过去了改回「行驶中」
        notice = getattr(self.tracker, 'notice', None)
        text = notice() if callable(notice) else None
        reasons = getattr(pose, 'reasons', ()) or ()
        if not text and 'wheel_hold' in reasons:
            text = '轮速反馈中断，卫星地速＋IMU 接管，' + ('按设定速度直行' if self._constant_straight_speed() else '已限速')
        elif not text and 'imu_hold' in reasons:
            text = 'IMU 中断，轮速＋卫星接管，' + ('按设定速度直行' if self._constant_straight_speed() else '已限速')
        elif not text and 'rtk_fixed_timeout' in reasons:
            pdop = self._pdop_text(now)
            text = PDOP_NOTICE_LOST % pdop if pdop else RTK_NOTICE_LOST
        elif not text and 'rtk_hold' in reasons:
            pdop = self._pdop_text(now)
            if getattr(pose, 'hold_age_s', 0.) >= RTK_LOST_PAINT_STOP_S - 1e-9:
                text = PDOP_NOTICE_LOST_PAINT % pdop if pdop else RTK_NOTICE_LOST_PAINT
            else:
                text = PDOP_NOTICE_LOST_HOLD % pdop if pdop else RTK_NOTICE_LOST_HOLD
        elif not text and self.tracker.st.state == 'PAUSED':
            why = self._resume_failed
            text = RESUME_FAILED_NOTICE % REASON_TEXT.get(why, why) if why else TRACKER_PAUSED_NOTICE
        elif not text and self._speed_hold.limit is not None:
            text = ('定位降级，按允许速度平稳恢复' if self._speed_hold.allowed_since is not None
                    else '定位已稳定，正在平稳恢复速度' if self._speed_hold.healthy_since is not None
                    else '定位降级，保持低速行驶')
        if text:
            self.message = text
        elif self.message in (RTK_NOTICE_LOST_HOLD, RTK_NOTICE_LOST_PAINT, RTK_NOTICE_LOST, TRACKER_PAUSED_NOTICE) \
                or self.message.startswith(RESUME_FAILED_NOTICE.split('%s')[0]) \
                or self.message.startswith(('定位已稳定', '定位降级', '固定解更新稍慢', '轮速反馈中断', 'IMU 中断')
                                           + PDOP_NOTICE_PREFIXES):
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
            if self.task == 'route' and self.operations.waiting_for_paint_tail(now):
                # 运动控制已完成，只等实际喷头到段尾/停稳；不恢复跟踪、不改变减速或拐角流程。
                return 0.0, 0.0, None
            if self.task == 'line':
                self._save_integral()
            self._finish('到达 B 点' if self.task == 'line' else '路线完成')
            return 0.0, 0.0, None
        if st == 'ABORTED':
            why = self.tracker.st.reason
            self._finish('中止：%s' % REASON_TEXT.get(why, why))
            return 0.0, 0.0, None
        return vx, wz, None

    def _limit_decel(self, now, motion_mode, vx, wz, dt):
        """从上一拍发出的指令起，车速每拍最多降 SOFT_STOP_DECEL_MPS2·dt（见常数说明），曲率不变；原地转（车速≈0）的角速度
        按 geo.pivot_accel_rad_s2 降。跟踪器暂停 / 中止 / 结束给的零、操作员暂停、限速突降都在这里变成渐停。
        不在 NAVIGATION（急停、控制权被切走）时不管：仲裁和驱动器自己处理。一拍只做一次。"""
        if self._soft_at == now:
            return vx, wz
        prev, self._soft_prev, self._soft_at = self._soft_prev, None, now
        if motion_mode != 'NAVIGATION':
            return vx, wz
        # 渐停中途来了一帧固定解、跟踪器原因从 pose_unusable 变成 pose_recovered_waiting_operator 也接着降（2026-10-03 实车：
        # 蓝牙 RTK 断 0.15 s，第一拍 0.80 → 0.77，下一拍原因变了直接给 0，驱动器 3 m/s² 急刹后两轮反转往后溜）
        if prev is not None and now - prev[0] <= 0.2:
            _, pv, pw = prev
            floor = abs(pv) - SOFT_STOP_DECEL_MPS2 * dt
            if abs(pv) > SOFT_STOP_CREEP_MPS:
                if floor > 0.0 and (abs(vx) < floor - 1e-9 or vx * pv < 0.0):
                    k = wz / vx if vx * pv > 0.0 and abs(vx) > 1e-6 else pw / pv   # 跟踪器还在给的曲率，否则上一拍的
                    vx = math.copysign(floor, pv)
                    wz = vx * k
            elif vx == 0.0 and abs(pv) < 1e-3:
                floor = abs(pw) - self.geo.pivot_accel_rad_s2 * dt
                if floor > 0.0 and (abs(wz) < floor - 1e-9 or wz * pw < 0.0):
                    wz = math.copysign(floor, pw)
        self._soft_prev = (now, vx, wz)
        return vx, wz

    def _auto_resume(self, now, pose):
        """定位问题暂停后，定位连续可用 AUTO_RESUME_S 就接着走；
        起步检查没过（如离线太远）隔 AUTO_RESUME_S 再试。"""
        if self.operations.held or self.tracker.st.state != 'PAUSED' or not pose.usable_for_control:
            self._usable_since = self._resume_failed = None
            return
        if self._usable_since is None:
            self._usable_since = now
        if now - self._usable_since < AUTO_RESUME_S - 1e-9:
            return
        why = self.tracker.st.reason
        preserve=(self.task=='route' and self.differential and isinstance(self.tracker,SectionTracker))
        options={'preserve_startup':True} if preserve else {}
        if self.tracker.start(pose,**options):
            self.message = '定位恢复，自动继续'
            self._usable_since = self._resume_failed = None
        else:
            self._usable_since, self._resume_failed = now, self.tracker.st.reason

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
                            'kappa', 'quality', 'sigma_xy', 'sigma_yaw_deg', 'state', 'pose_usable', 'pose_reasons'])

    def _write_log(self, now, pose, vx):
        if not self._log:
            return
        st = self.tracker.st
        self._log.writerow(['%.3f' % now, '%.4f' % pose.x, '%.4f' % pose.y, '%.3f' % math.degrees(pose.yaw),
                            '%.3f' % pose.speed, '%.4f' % st.e, '%.3f' % math.degrees(st.heading_err),
                            '%.3f' % st.s, '%.3f' % vx, '%.5f' % st.kappa, getattr(pose, 'quality', ''),
                            '%.4f' % getattr(pose, 'sigma_xy', math.nan),
                            '%.3f' % math.degrees(getattr(pose, 'sigma_yaw', math.nan)), st.state,
                            int(bool(getattr(pose,'usable_for_control',False))),
                            '|'.join(str(v) for v in (getattr(pose,'reasons',()) or ()))])

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
            reasons=list(p.reasons), rtk_trusted=bool(getattr(p, 'rtk_trusted', False)))
        points = {k: self.fusion.to_local(*v) for k, v in self.points.items()} if self.points else {}
        # 融合的在线标定（安装偏差、轮速比例、陀螺零偏）：只报告，经人确认后才写进 ~/rtk/fusion.json
        calibration = getattr(self.fusion, 'calibration', None)
        calibration = calibration() if callable(calibration) else None
        # 倾斜补偿：天线高度、在不在补、没补的原因、补偿量（app 姿态卡片显示）
        tilt = getattr(self.fusion, 'tilt_status', None)
        tilt = tilt() if callable(tilt) else None
        if tilt is not None and self.antenna_mode == 'single':
            # 单天线接收机输出的坐标已按杆长做过倾斜补偿（line_node.single_antenna_config），车端不补：如实报原因
            tilt = dict(tilt, active=False, reason='receiver')
        orient = None if self._orient is None else dict(distance_m=round(self._orient['dist'], 3),
                                                         start_after=self._orient['cmd'] is not None)
        return _json_safe(dict(phase=self.phase, message=self.message, cruise=self.cruise, pose=pose,
                               antenna_mode=self.antenna_mode, orient=orient, gnss_latency=self._latency.status(),
                               points={k: [round(v[0], 3), round(v[1], 3)] for k, v in points.items()},
                               tracker=self.tracker.status(), task=self.task,
                               origin=self._origin(), antenna=self._antenna_status(), route=self._route_status(),
                               route_limits=dict(min_radius=round(self.min_route_radius, 3),
                                                 max_points=route_plan.MAX_POINTS, max_laps=route_plan.MAX_LAPS,
                                                 max_entry_m=route_plan.MAX_ENTRY_M,
                                                 # 连线路线压线用的车身：后轴中心到车头、车尾、半宽（app 画同样的预览）
                                                 body=[round(v, 4) for v in self.body],
                                                 # 差速车（能原地转）按水泵压线：nozzle_x 水泵在后轴前多少米，
                                                 # nozzle_right 在车中线右多少米（共享配置；app 开始作业带的对准点优先）
                                                 drive=self.geo.drive, nozzle_x=self.nozzle_x,
                                                 nozzle_right=-self.nozzle[1] if self.differential else None,
                                                 # 开始作业可以带 app 的「对准点」nozzle {forward, right}（喷头可以在侧面），
                                                 # 连线路线、圆按它压线；旧车端不认，app 看这一项再发
                                                 route_nozzle=True if self.differential else None,
                                                 # 差速车连线路线急弯可选的转法（路线的 turn 字段；圆弧掉头按 turn_radius 绕）
                                                 turns=list(route_plan.TURNS) if self.differential else None,
                                                 pivot_rate=self.geo.pivot_rate_rad_s,
                                                 pivot_accel=self.geo.pivot_accel_rad_s2,
                                                 accel=self.path_cfg.accel_mps2,
                                                 lateral_accel=self.path_cfg.lat_accel_mps2),
                               calibration=calibration, diag=self._diag(calibration), tilt=tilt,
                               imu_hold_test=self._imu_hold_test_status(),
                               rtk_events=self._jump_events(), operations=self.operations.status()))

    def section_options(self, route=False):
        """SectionTracker 按底盘的额外参数：差速车原地转向、倒车曲率照走（没有转向机构转不够），倒车段限速、
        段尾停稳时间按差速车（拐角提速，见 path_tracker.differential_config）。route：普通路线（开始作业、续画、补画，
        不是场地作业），倒车接入后的助跑起步锁车头更长（path_tracker.REVERSE_HOLD_M）。"""
        if not self.differential:
            return {}
        from paint_quality import offset_limit
        return dict(pivot_rate=self.geo.pivot_rate_rad_s, pivot_accel=self.geo.pivot_accel_rad_s2, reverse_gain=1.0,
                    reverse_mps=DIFF_REVERSE_MPS, settle_s=DIFF_SETTLE_S, paint_offset_limit_m=offset_limit(),
                    constant_straight_speed=True, constant_paint_speed=True, **({'reverse_runup': True} if route else {}))

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
