#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成安卓 app 单元测试用的协议样例(车端编码器的真实输出)。

样例基准放在 tests/fixtures/app_protocol/,tests/test_app_link.py 检查它与当前
编码器一致。安卓工程单独存放(本机 ~/Desktop/app/follow-cockpit),改了协议字段后:

    python3 radar_system/tools/export_app_fixtures.py \\
        --out ~/Desktop/app/follow-cockpit/app/src/test/resources
"""
import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, REPO)

import app_link  # noqa: E402
import route_plan  # noqa: E402
from robot_core.kinematics import ChassisGeometry  # noqa: E402
from path_tracker import PathConfig  # noqa: E402
from runtime_config import PROFILE  # noqa: E402

FIXTURE_DIR = os.path.join(REPO, 'tests', 'fixtures', 'app_protocol')
BODY = (PROFILE['geometry']['front_m'], PROFILE['geometry']['rear_m'], PROFILE['geometry']['half_width_m'])
# 差速车（Gen5）的缓弯半径：共享配置 drive.min_radius_m（0.5 m）/ kappa_margin，与 line_mission 相同
PIVOT_RADIUS = PROFILE['drive']['min_radius_m'] / PathConfig().kappa_margin


def sample_snapshot():
    """radar_web_server 状态快照的一个典型样本：底盘在线、手动控制中、车速 0.42 m/s。"""
    return {
        'voltage': 24.1, 'battery_pct': 82,
        'wheeltec': {'connected': True, 'telemetry': {'velocity': [0.42, 0.0, 0.1]}},
        'wheeltec_status_age_s': 0.05,
        'motion': {'mode': 'MANUAL', 'healthy': True},
        'manual_override': {'active': False},
    }


def sample_gnss(now=100.0):
    """/gnss/status 的典型样本：RTK 固定解、双天线航向有效，接收机在平板串口上（差分由平板连 CORS，车端 CORS 暂停）。"""
    return {'_rx_monotonic': now - 0.3, 'receiver_ok': True, 'rtk': 'FIXED', 'sol': 'NARROW_INT',
            'sats': 32, 'diff_age': 1.0, 'sigma_h': 0.0092, 'heading_ok': True, 'heading_sol': 'L1_INT',
            'baseline_m': 0.3312, 'rates': {'fix': 20.0, 'heading': 20.0, 'vel': 20.0}, 'crc_errors': 0,
            'port_open': True, 'port_error': None,
            # 接收机接在平板串口上（平板 app 经 TCP 转发），车上 USB 没接
            'source': 'tablet', 'usb': {'open': False, 'error': 'FileNotFoundError: no such device'},
            'tablet': {'listening': True, 'error': None, 'port': 48622, 'connected': True, 'peer': '192.168.2.50:40112',
                       'connections': 1, 'bytes_in': 512000},
            'antennas': {
                'ant1': dict(available=True, source='BESTPOS', sol='NARROW_INT', fix='FIXED',
                             sats_used=32, sats_tracked=40, age_s=0.1),
                'ant2': dict(available=True, source='BESTNAVH', sol='NARROW_INT', fix='FIXED',
                             sats_used=29, sats_tracked=37, age_s=0.4, offset_m=0.331, consistent=True)},
            'heading': dict(sol='L1_INT', fix='FIXED', sats_used=28, sats_tracked=36, length_m=0.331,
                            pitch_deg=-0.8, sigma_deg=0.28, ref_m=0.331, valid=True,
                            yaw_deg=90.18),     # 双天线测出的车头方向（ENU，东 0 逆时针）：车头朝北
            # 车端 CORS 让给平板：账号已配置，但数据来自平板时不连（同一账号两处登录会互相顶掉）
            'ntrip': {'configured': True, 'connected': False, 'enabled': True, 'error': None,
                      'config_revision': 'fixture-1', 'config_error': None, 'paused': 'tablet'}}


def sample_line(now=100.0):
    """/line/status 的典型样本：A(0,0)→B(20,0) 走到 8.2 m，左偏 6 mm。"""
    return {'_rx_monotonic': now - 0.1, 'phase': 'RUNNING', 'message': '行驶中', 'cruise': 0.5,
            'motion_mode': 'NAVIGATION', 'points': {'A': [0.0, 0.0], 'B': [20.0, 0.0]},
            'pose': {'x': 8.2, 'y': 0.006, 'yaw_deg': 0.12, 'speed': 0.5, 'quality': 'RTK_FIXED',
                     'usable': True, 'sigma_xy': 0.009, 'sigma_yaw_deg': 0.08, 'reasons': []},
            'tracker': {'state': 'RUNNING', 'reason': '', 'e': 0.006, 'heading_err_deg': 0.12, 's': 8.2,
                        'length': 20.0, 'v_cmd': 0.5, 'kappa': -0.004, 'integral': 0.01, 'trim_kappa': -0.006,
                        'stats': {'rms_m': 0.004, 'max_m': 0.011, 'samples': 170}},
            'last_result': {'ok': True, 'message': '申请控制权', 'id': '7', 'cmd': 'start'}}


ROUTE = {'kind': 'points', 'points': [[0.0, 0.0], [10.0, 0.0], [10.0, 8.0], [0.0, 8.0]], 'closed': True,
         'center': None, 'radius': 3.0, 'clockwise': True, 'laps': 2, 'turn_radius': 2.5, 'cruise': 0.5}


def min_route_radius():
    return ChassisGeometry().min_turn_radius_m / PathConfig().kappa_margin


def sample_bad_heading_gnss(now=100.0):
    """外场实测的样子：两根天线位置都报 NARROW_INT、相距 2.72 m，测向浮点 2.10 m（测向没固定，没法核对）。"""
    g = sample_gnss(now)
    g.update(heading_ok=False, heading_sol='NARROW_FLOAT', baseline_m=2.097,
             heading=dict(sol='NARROW_FLOAT', fix='FLOAT', sats_used=30, sats_tracked=38, length_m=2.097,
                          pitch_deg=72.8, sigma_deg=18.8, ref_m=None, valid=False, yaw_deg=None))
    g['antennas'] = dict(g['antennas'], ant2=dict(g['antennas']['ant2'], sats_used=31, sats_tracked=38,
                                                  offset_m=2.716, consistent=None))
    return g


def sample_route_line(now=100.0):
    """/line/status：走路线中（第 1 圈过了第 2 个拐角），带原点、天线定位和车上存的路线。"""
    spec = route_plan.parse_spec(ROUTE)
    _, pieces = route_plan.loop_pieces(spec, min_route_radius())
    route = dict(spec.to_dict(), rev=3, length=round(sum(p[0] for p in pieces), 3), problem=None)
    return {'_rx_monotonic': now - 0.1, 'phase': 'RUNNING', 'message': '行驶中', 'cruise': 0.5,
            'motion_mode': 'NAVIGATION', 'points': {}, 'task': 'route',
            'origin': [30.123456789, 120.987654321],
            'antenna': {'x': 10.12, 'y': 5.03, 'sol': 'NARROW_INT', 'sigma': 0.011},
            'route': route,
            'route_limits': {'min_radius': round(min_route_radius(), 3), 'max_points': route_plan.MAX_POINTS,
                             'max_laps': route_plan.MAX_LAPS, 'max_entry_m': route_plan.MAX_ENTRY_M,
                             'body': list(BODY), 'drive': 'ackermann', 'nozzle_x': None},
            'pose': {'x': 10.004, 'y': 5.0, 'yaw_deg': 90.3, 'speed': 0.5, 'quality': 'RTK_FIXED',
                     'usable': True, 'sigma_xy': 0.009, 'sigma_yaw_deg': 0.08, 'reasons': []},
            'tracker': {'state': 'RUNNING', 'reason': '', 'e': -0.004, 'heading_err_deg': 0.3, 's': 17.1,
                        'length': 66.4, 'v_cmd': 0.35, 'kappa': 0.0, 'integral': 0.0, 'trim_kappa': 0.0,
                        'stats': {'rms_m': 0.006, 'max_m': 0.018, 'samples': 700},
                        'gear': 'forward', 'section': 1, 'sections': 1, 'switching': False},
            'last_result': {'ok': True, 'message': '申请控制权（接入 3.5 m，全程 66.4 m）', 'id': 'r4',
                            'cmd': 'route_start'},
            'diag': {'kappa_cmd': 0.0123, 'kappa_act': 0.0111, 'yaw_rate': 0.0056, 'rtk_along_m': 0.0042,
                     'rtk_left_m': -0.0061, 'position_outliers': 3, 'position_resets': 0, 'jumps': 1},
            'rtk_events': [{'id': 87.35, 'x': 8.2, 'y': 4.9, 'lat': 30.123500123, 'lon': 120.987750456, 'd': 0.0821,
                            'left': 0.078, 'dur': 1.05, 'age_s': 12.5}],
            # 车身姿态（IMU）：右侧略低 1.25°、车头抬起 2.4°（REP-103：横滚左侧抬起为正，俯仰车头压低为正），
            # IMU 偏航比它启动时左转了 12.6°（没有磁力计，只是相对值）
            'attitude': {'roll_deg': 1.25, 'pitch_deg': -2.4, 'yaw_deg': 12.6, 'source': 'imu_madgwick'},
            # 倾斜补偿：天线高 0.40 m，按上面的姿态天线比平地时往后 1.72 cm、往右 0.88 cm（车体系前、左为正）
            'tilt': {'height_m': 0.4, 'active': True, 'reason': None, 'roll_deg': 1.25, 'pitch_deg': -2.4,
                     'forward_m': -0.0172, 'left_m': -0.0088}}


def route_geometry_cases():
    """app 预览（RouteGeometry.kt）与车端几何的对照样例：同样的路线和车位姿，分段必须一致。"""
    rect = dict(ROUTE, laps=1)
    cases = [
        ('矩形闭合 1 圈', rect, None),
        ('矩形闭合 3 圈', dict(rect, laps=3), None),
        ('五边形 2 圈', dict(rect, points=[[0, 0], [12, 0], [15, 9], [4, 11], [-3, 5]], laps=2), None),
        ('开放 3 点', dict(rect, points=[[0, 0], [8, 0], [8, 6]], closed=False), None),
        ('共线', dict(rect, points=[[0, 0], [5, 0], [10, 0]], closed=False), None),
        ('圆 逆时针', dict(rect, kind='circle', center=[3.0, 4.0], radius=2.5, clockwise=False, laps=2), None),
        ('圆 顺时针', dict(rect, kind='circle', center=[-2.0, 1.0], radius=4.0, clockwise=True), None),
        ('接入 车在左下朝北', rect, [-2.0, -3.0, math.pi / 2]),
        ('接入 车在起点正后方', rect, [-3.0, 0.0, 0.0]),
        ('接入 车在起点前方反向', rect, [2.0, 0.5, math.pi]),
        ('接入 圆', dict(rect, kind='circle', center=[3.0, 4.0], radius=2.5), [6.0, 6.0, -2.0]),
        # 现场定点画圆：车（喷头）停在圆心；差速车喷头画圆、原地转向接入
        ('圆 车在圆心 顺时针 2 圈', dict(rect, kind='circle', center=[1.0, 2.0], radius=2.0, clockwise=True, laps=2),
         [0.7, 2.0, 0.0]),
        ('错误 圆 差速车半径太小', dict(rect, kind='circle', center=[0.0, 0.0], radius=0.6, clockwise=False), [0.0, 0.0, 0.0]),
        ('接入 已在起点', rect, [0.0, 0.0, 0.0]),
        ('错误 路点太少', dict(rect, points=[[0, 0], [5, 0]]), None),
        ('错误 拐角不够长', dict(rect, points=[[0, 0], [3, 0], [3, 2]], closed=False), None),
        ('错误 折返', dict(rect, points=[[0, 0], [10, 0], [0, 0.01]], closed=False), None),
        ('错误 半径太小', dict(rect, turn_radius=1.5), None),
        ('错误 太远', rect, [80.0, 0.0, 0.0]),
    ]
    path = dict(kind='path', points=[[3, 0], [9, 0], [12, 5], [16, 1]], closed=False, laps=1, cruise=0.5)
    cases += [
        ('连线 折线', path, [0.0, 0.0, 0.0]),
        ('连线 起点在左后方', dict(path, points=[[-2, 4], [-2, 9], [3, 11]]), [0.0, 0.0, 0.0]),
        ('连线 10cm 密点', dict(path, points=[[3 + 0.1 * i, round(0.4 * math.sin(0.15 * i), 4)] for i in range(60)]),
         [0.5, -0.2, 0.1]),
        ('连线 直角急弯', dict(path, points=[[2, 0], [6, 0], [6, 8]]), [0.0, 0.0, 0.0]),
        ('连线 缓弯', dict(path, points=[[0, 0], [6, 0], [12, 2], [18, 2.5]]), [-2.0, 0.03, 0.05]),
        ('连线 闭环 2 圈', dict(path, points=[[3, 0], [11, 0], [11, 7], [3, 7]], closed=True, laps=2), [0.0, 0.0, 0.0]),
        ('连线 车在起点前方', dict(path, points=[[0, 0], [6, 0], [6, 5]]), [0.3, 0.1, 0.0]),
        ('连线 起点在正后方', dict(path, points=[[-3, 0], [-6, 0]]), [0.0, 0.0, 0.0]),
        ('连线 起点贴车侧', dict(path, points=[[0.2, 0.8], [3, 0.8]]), [0.0, 0.0, 0.0]),
        ('连线 行距 0.8 m 往返', dict(path, points=[[0, 0], [6, 0], [6, 0.8], [0, 0.8]], closed=False), [-2.0, 0.0, 0.0]),
        ('连线 折返 176°', dict(path, points=[[0, 0], [8, 0], [0, 0.5]]), [-2.0, 0.0, 0.0]),
        ('连线 闭环 右转 3 圈', dict(path, points=[[13.5, 0], [0, 0], [0, 8], [13.5, 8]], closed=True, laps=3), [16.0, 0.0, math.pi]),
        ('连线 急弯各角度', dict(path, points=[[0, 0], [6, 0], [6 + 5 * math.cos(math.radians(40)), 5 * math.sin(math.radians(40))],
                                           [12, 9], [4, 9.5], [0, 3]]), [-2.0, 0.0, 0.0]),
        ('错误 连线 太远', path, [60.0, 0.0, 0.0]),
    ]

    def bend(deg, leg=6.0):
        a = math.radians(deg)
        return [[0, 0], [leg, 0], [round(leg + leg * math.cos(a), 4), round(leg * math.sin(a), 4)]]

    def arc(radius, deg, sag):
        n = math.ceil(math.radians(deg) / (2 * math.acos(1 - sag / radius)))
        return [[round(radius * math.sin(math.radians(deg) * i / n), 4),
                 round(radius - radius * math.cos(math.radians(deg) * i / n), 4)] for i in range(n + 1)]
    # 差速车喷头在后轴前：缓弯时后轴被喷头拖着走（第 4 项是喷头位置，不按单双号轮换）
    cases += [
        ('连线 缓弯 10° 拖过去', dict(path, points=bend(10.0)), [-2.0, 0.0, 0.0], 0.30),
        ('连线 缓弯 右转 25° 拖过去', dict(path, points=bend(-25.0)), [-2.0, 0.0, 0.0], 0.30),
        ('连线 35° 超过拖弯极限', dict(path, points=bend(35.0)), [-2.0, 0.0, 0.0], 0.30),
        ('连线 缓弯 喷头 0.154', dict(path, points=bend(12.0)), [-2.0, 0.0, 0.0], 0.154),
        ('连线 罚球弧折线', dict(path, points=arc(9.15, 106.0, 0.005)), [-2.0, 0.0, 0.0], 0.30),
        ('连线 短边连续缓弯', dict(path, points=[[0, 0], [3, 0], [3.5, 0.1], [4, 0], [4.5, 0.1], [8, 0.1]]),
         [-2.0, 0.0, 0.0], 0.30),
    ]
    out = []
    for name, d, car, *fixed in cases:
        spec = route_plan.parse_spec(d)
        case = {'name': name, 'route': spec.to_dict(digits=6), 'car': car, 'min_radius': min_route_radius()}
        if spec.kind == 'path':
            # 路线本身（不含接入段，app 画图纸、报问题用）和从车位姿起的整趟（app 仿真用）
            case['body'] = list(BODY)
            try:
                route, _ = route_plan._route_sections(spec, min_route_radius(), BODY)
                case.update(route_sections=[[d_, list(s), [list(p) for p in ps]] for d_, s, ps in route],
                            cover=route_plan.cover_summary(spec, min_route_radius(), BODY))
                sections, entry = route_plan.plan(spec, tuple(car), min_route_radius(), body=BODY)
                case.update(sections=[[s.direction, list(s.path.pieces[0].pose(0.0)),
                                       [[p.length, p.kappa] for p in s.path.pieces]] for s in sections],
                            entry_m=entry, error=None)
            except ValueError as exc:
                case.update(sections=None, entry_m=None, error=str(exc))
                case.setdefault('route_sections', None)
                case.setdefault('cover', None)
            # 差速车（Gen5）同一条路线：喷头压线、急弯原地转向（app PivotRoute.kt 对照）；喷头在后轴前或两轮中间
            nozzle = fixed[0] if fixed else 0.30 if len(out) % 2 == 0 else 0.0
            try:
                pivot, corners = route_plan._pivot_route_sections(spec, PIVOT_RADIUS, nozzle)
                case['pivot'] = dict(radius=PIVOT_RADIUS, nozzle_x=nozzle, corners=corners, pivots=route_plan.pivot_count(pivot),
                                     sections=[[d_, list(st), [list(p) for p in ps]] for d_, st, ps in pivot])
                if car is not None:
                    # 从车位姿起的整趟（原地转向接入 + 路线），接入太远时记原因
                    try:
                        plan, entry_m = route_plan.plan(spec, tuple(car), PIVOT_RADIUS, nozzle_x=nozzle)
                        case['pivot'].update(plan=[[s.direction, list(s.path.pieces[0].pose(0.0)),
                                                    [[p.length, p.kappa] for p in s.path.pieces]] for s in plan],
                                             entry_m=entry_m, plan_error=None)
                    except ValueError as exc:
                        case['pivot'].update(plan=None, entry_m=None, plan_error=str(exc))
            except ValueError as exc:
                case['pivot'] = dict(radius=PIVOT_RADIUS, nozzle_x=nozzle, error=str(exc))
            out.append(case)
            continue
        try:
            start, pieces = route_plan.loop_pieces(spec, min_route_radius())
            entry = [] if car is None else route_plan.dubins(tuple(car), start, spec.turn_radius)
            if sum(p[0] for p in entry) > route_plan.MAX_ENTRY_M:
                raise ValueError('太远')
            case.update(start=list(start), entry=[list(p) for p in entry], pieces=[list(p) for p in pieces],
                        error=None)
        except ValueError as exc:
            case.update(start=None, entry=None, pieces=None, error=str(exc))
        if spec.kind == 'circle':
            # 差速车（Gen5）同一个圆：喷头画半径 R 的圆（后轴走 √(R²−d²)），原地转向接入（app 对照）
            try:
                pstart, ppieces = route_plan.loop_pieces(spec, PIVOT_RADIUS, 0.30)
                case['pivot'] = dict(radius=PIVOT_RADIUS, nozzle_x=0.30, start=list(pstart),
                                     pieces=[list(p) for p in ppieces])
                if car is not None:
                    try:
                        plan, entry_m = route_plan.plan(spec, tuple(car), PIVOT_RADIUS, nozzle_x=0.30)
                        case['pivot'].update(plan=[[s.direction, list(s.path.pieces[0].pose(0.0)),
                                                    [[p.length, p.kappa] for p in s.path.pieces]] for s in plan],
                                             entry_m=entry_m, plan_error=None)
                    except ValueError as exc:
                        case['pivot'].update(plan=None, entry_m=None, plan_error=str(exc))
            except ValueError as exc:
                case['pivot'] = dict(radius=PIVOT_RADIUS, nozzle_x=0.30, error=str(exc))
        out.append(case)
    return {'max_entry_m': route_plan.MAX_ENTRY_M, 'cover_tol_m': route_plan.COVER_TOL_M,
            'vertex_pass_m': route_plan.VERTEX_PASS_M, 'corner_clear_m': route_plan.CORNER_CLEAR_M,
            'corner_extra_m': list(route_plan.CORNER_EXTRA_M), 'corner_back_m': list(route_plan.CORNER_BACK_M),
            'min_run_m': route_plan.MIN_RUN_M, 'cusp_tie_m': route_plan.CUSP_TIE_M,
            'entry_lead_m': route_plan.ENTRY_LEAD_M, 'on_line_m': route_plan.ON_LINE_M,
            'on_line_deg': route_plan.ON_LINE_DEG, 'body': list(BODY), 'cases': out}


def route_start_cases():
    """开始作业（plan_current：从离车最近处开始、跑完整条路线）与 app RouteStart.kt 的对照样例。"""
    line = dict(kind='path', points=[[0, 0], [10, 0]], closed=False, cruise=0.8)
    legs = dict(kind='path', points=[[0, 0], [20, 0], [20, 10]], closed=False, cruise=0.5)
    rect = dict(kind='path', points=[[0, 0], [10, 0], [10, 5], [0, 5]], closed=True, cruise=0.8)
    circle = dict(kind='circle', center=[0, 0], radius=2.5, clockwise=True, laps=1, cruise=0.5)
    d, ack = 0.30, min_route_radius()
    cases = [
        ('直线 喷头在起点', line, [-0.3, 0.0, 0.0], PIVOT_RADIUS, d),
        ('直线 喷头在终点 从起点开过来', line, [9.7, 0.0, 0.0], PIVOT_RADIUS, d),
        ('直线 标点偏 0.24 m', line, [9.7, 0.24, 0.0], PIVOT_RADIUS, d),
        ('直线 中间 去近的一头', line, [5.5, 0.0, 0.0], PIVOT_RADIUS, d),
        ('直线 助跑以外直接开进', line, [-1.5, 0.0, 0.0], PIVOT_RADIUS, d),
        ('直线 起点后方偏一点 往前开到助跑点', line, [-1.0, 0.2, 0.0], PIVOT_RADIUS, d),
        ('折线 在最后一点', legs, [20.0, 9.7, math.pi / 2], PIVOT_RADIUS, d),
        ('矩形 边中间', rect, [4.7, -0.1, 0.0], PIVOT_RADIUS, d),
        ('矩形 靠近拐角', rect, [9.6, 0.1, 0.0], PIVOT_RADIUS, d),
        ('矩形 两圈', dict(rect, laps=2), [5.3, 5.1, math.pi], PIVOT_RADIUS, d),
        ('矩形 车在中间', rect, [5.0, 2.4, math.pi / 2], PIVOT_RADIUS, d),
        ('圆 喷头在圆边', circle, [2.5, -0.3, -math.pi / 2], PIVOT_RADIUS, d),
        ('圆 喷头在圆心 朝东', circle, [-0.3, 0.0, 0.0], PIVOT_RADIUS, d),
        ('圆 喷头在圆心 朝西北', circle, [-0.3 * math.cos(2.0), -0.3 * math.sin(2.0), 2.0], PIVOT_RADIUS, d),
        ('圆 逆时针两圈', dict(circle, radius=3.0, clockwise=False, laps=2), [3.0, 0.0, math.pi / 2], PIVOT_RADIUS, d),
        ('阿克曼 直线 靠近终点', line, [9.0, 0.0, math.pi], ack, None),
        ('阿克曼 矩形', rect, [5.0, 0.3, 0.0], ack, None),
        ('阿克曼 圆', dict(circle, radius=3.0), [3.0, 0.0, -math.pi / 2], ack, None),
        ('阿克曼 路点折线 闭合', dict(rect, kind='points', turn_radius=2.0), [9.0, 0.0, 0.0], ack, None),
        ('错误 太远', line, [0.0, -100.0, 0.0], PIVOT_RADIUS, d),
    ]
    out = []
    for name, raw, car, radius, nozzle in cases:
        spec = route_plan.parse_spec(raw)
        case = dict(name=name, route=spec.to_dict(digits=6), car=car, radius=radius, nozzle=nozzle, body=list(BODY))
        try:
            sections, entry = route_plan.plan_current(spec, tuple(car), radius, body=BODY, nozzle_x=nozzle)
            case.update(entry_m=entry, error=None, sections=[
                dict(direction=s.direction, start=list(s.path.pieces[0].pose(0.0)),
                     pieces=[[p.length, p.kappa] for p in s.path.pieces]) for s in sections])
        except ValueError as exc:
            case.update(entry_m=None, error=str(exc), sections=None)
        out.append(case)
    return out


def fixture_messages():
    """文件名 -> 消息。tel_sample 是 IMU 定向直行中，tel_line_sample 是 RTK 走直线中，
    tel_route_sample 是走路线中，route_geometry_sample 是路线几何对照样例，route_start_current_sample 是开始作业
    （从最近处开始跑完整条）对照样例。"""
    snap = sample_snapshot()
    snap['gnss'] = sample_gnss()
    # IMU 定向直行正在跑（heading_hold.HeadingHold.status 的样子，作业键 = (会话, app 作业编号)）
    snap['straight'] = {'imu_ok': True, 'yaw_deg': 30.42, 'run': ('app-1', 3), 'state': 'running', 'reason': None,
                        'target_deg': 30.0, 'error_deg': 0.42, 'max_error_deg': 0.83, 'within_pct': 98.5,
                        'elapsed_s': 12.3, 'speed': 0.4, 'wz': -0.021}
    snap['network'] = {'mode': 'hotspot', 'ssid': 'RTKCAR-A2-10', 'hotspot_ssid': 'RTKCAR-A2-10', 'hotspot_ip': '192.168.42.1',
                       'busy': False, 'error': None, 'revert_to': 'client', 'revert_in_s': 87}
    tel = app_link.build_telemetry(snap, now=100.0, seq=42)
    line = sample_snapshot()
    line.update(motion={'mode': 'NAVIGATION', 'healthy': True}, gnss=sample_gnss(), line=sample_line())
    tel_line = app_link.build_telemetry(line, now=100.0, seq=43)
    # 车热点（car_network.py）：名字、密码、地址；遥测里是现在用哪种方式、等平板连上来的倒计时
    network_info = {'hotspot_ssid': 'RTKCAR-A2-10', 'hotspot_psk': 'rtkcar2026', 'hotspot_ip': '192.168.42.1'}
    hello = app_link.build_hello(PROFILE, name='测试车', session='app-1', network=network_info)
    route = sample_snapshot()
    route.update(motion={'mode': 'NAVIGATION', 'healthy': True}, gnss=sample_bad_heading_gnss(),
                 line=sample_route_line())
    route['wheeltec'] = dict(route['wheeltec'], guard={'navigation_uses_lidar': False, 'bypassed': True})
    tel_route = app_link.build_telemetry(route, now=100.0, seq=44)
    return {'tel_sample.json': tel, 'tel_line_sample.json': tel_line, 'tel_route_sample.json': tel_route,
            'route_geometry_sample.json': route_geometry_cases(), 'hello_sample.json': hello,
            'route_start_current_sample.json': route_start_cases()}


def write(directory):
    os.makedirs(directory, exist_ok=True)
    for name, msg in fixture_messages().items():
        with open(os.path.join(directory, name), 'w', encoding='utf-8') as f:
            json.dump(msg, f, ensure_ascii=False, allow_nan=False)
    return sorted(fixture_messages())


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--out', action='append', default=[],
                   help='额外写到这个目录(可重复),例如安卓工程的 app/src/test/resources')
    args = p.parse_args()
    for directory in [FIXTURE_DIR] + [os.path.expanduser(d) for d in args.out]:
        print(directory, write(directory))


if __name__ == '__main__':
    main()
