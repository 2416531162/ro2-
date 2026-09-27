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


def sample_snapshot(stale=False):
    """radar_web_server 状态快照的一个典型样本:正前方 1.8m 有一个被相机和雷达同时看到的人。"""
    s = PROFILE['sensors']
    return {
        'hz': 10.0, 'voltage': 24.1, 'battery_pct': 82, 'follower_running': True,
        'ranges': [1.0] * 720, 'angle_min': 0.0, 'angle_increment': 2 * math.pi / 720,
        'ai_status': {'fps': 29.7},
        'ai_targets': [{'label': 'person', 'conf': 0.9, 'x1': 200, 'y1': 80, 'x2': 300,
                        'y2': 420, 'distance': 1.8, 'bearing_rad': 0.0},
                       {'label': 'chair', 'x1': 1}],
        'wheeltec': {'connected': True, 'telemetry': {'velocity': [0.42, 0.0, 0.1]}},
        'wheeltec_status_age_s': 0.05,
        'motion': {'mode': 'FOLLOW', 'healthy': True},
        'manual_override': {'active': False},
        'follower': {'stale': stale, 'state': 'TRACKING', 'limit_reason': 'follow',
                     'turnaround_phase': 'IDLE', 'turnaround_plan': None,
                     'aim_point': [2.1, 0.0], 'speed_cap_mps': 0.8,
                     'path_clearance_m': 1.6, 'aeb_active': False,
                     'tracks': [{'id': 3, 'x': 2.3, 'y': 0.0, 'confirmed': True, 'target': True}],
                     'target': {'track_id': 3, 'veh_x': s['camera_x_m'] + 1.8, 'veh_y': 0.0,
                                'gap_used': 1.2, 'bearing_deg': 0.0, 'ground_speed': 0.9,
                                'in_camera_view': True, 'coasting': False}},
    }


def sample_gnss(now=100.0):
    """/gnss/status 的典型样本：RTK 固定解、双天线航向有效、CORS 已连上。"""
    return {'_rx_monotonic': now - 0.3, 'receiver_ok': True, 'rtk': 'FIXED', 'sol': 'NARROW_INT',
            'sats': 32, 'diff_age': 1.0, 'sigma_h': 0.0092, 'heading_ok': True, 'heading_sol': 'L1_INT',
            'baseline_m': 0.3312, 'rates': {'fix': 20.0, 'heading': 20.0, 'vel': 20.0}, 'crc_errors': 0,
            'port_open': True, 'port_error': None,
            'antennas': {
                'ant1': dict(available=True, source='BESTPOS', sol='NARROW_INT', fix='FIXED',
                             sats_used=32, sats_tracked=40, age_s=0.1),
                'ant2': dict(available=True, source='BESTNAVH', sol='NARROW_INT', fix='FIXED',
                             sats_used=29, sats_tracked=37, age_s=0.4, offset_m=0.331, consistent=True)},
            'heading': dict(sol='L1_INT', fix='FIXED', sats_used=28, sats_tracked=36, length_m=0.331,
                            pitch_deg=-0.8, sigma_deg=0.28, ref_m=0.331, valid=True,
                            yaw_deg=90.18),     # 双天线测出的车头方向（ENU，东 0 逆时针）：车头朝北
            'ntrip': {'configured': True, 'connected': True, 'mount': 'AUTO', 'bytes': 81234,
                      'rtcm_types': [1005, 1074, 1084, 1124], 'error': None, 'age_s': 0.4,
                      'config_revision': 'fixture-1', 'config_error': None}}


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
            'attitude': {'roll_deg': 1.25, 'pitch_deg': -2.4, 'yaw_deg': 12.6, 'source': 'imu_madgwick'}}


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
    out = []
    for name, d, car in cases:
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
            nozzle = 0.30 if len(out) % 2 == 0 else 0.0
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
        out.append(case)
    return {'max_entry_m': route_plan.MAX_ENTRY_M, 'cover_tol_m': route_plan.COVER_TOL_M,
            'vertex_pass_m': route_plan.VERTEX_PASS_M, 'corner_clear_m': route_plan.CORNER_CLEAR_M,
            'corner_extra_m': list(route_plan.CORNER_EXTRA_M), 'corner_back_m': list(route_plan.CORNER_BACK_M),
            'min_run_m': route_plan.MIN_RUN_M, 'cusp_tie_m': route_plan.CUSP_TIE_M,
            'entry_lead_m': route_plan.ENTRY_LEAD_M, 'on_line_m': route_plan.ON_LINE_M,
            'on_line_deg': route_plan.ON_LINE_DEG, 'body': list(BODY), 'cases': out}


def fixture_messages():
    """文件名 -> 消息。tel_sample 是跟随掉头中（覆盖 plan/segments），tel_line_sample 是 RTK 走直线中，
    tel_route_sample 是走路线中，route_geometry_sample 是路线几何对照样例。"""
    snap = sample_snapshot()
    snap['follower'].update(state='TURNAROUND', turnaround_phase='REVERSE', turnaround_plan={
        'direction': -1, 'turn_dir': 1, 'stroke_m': 1.2, 'eta_s': 2.4, 'reached': True,
        'final_bearing_deg': 18.0, 'segments': [[-1, 1.2], [1, 0.3]]})
    snap['ranges'][0] = 2.5
    snap['gnss'] = sample_gnss()
    tel = app_link.build_telemetry(snap, now=100.0, seq=42, sensors=PROFILE['sensors'])
    line = sample_snapshot(stale=True)
    line.update(follower_running=False, motion={'mode': 'NAVIGATION', 'healthy': True},
                gnss=sample_gnss(), line=sample_line())
    tel_line = app_link.build_telemetry(line, now=100.0, seq=43, sensors=PROFILE['sensors'],
                                        include_scan=False)
    hello = app_link.build_hello(PROFILE, name='测试车', camera={'width': 640, 'height': 480,
                                                              'fx': 570.3, 'cx': 319.5},
                                 session='app-1')
    route = sample_snapshot(stale=True)
    route.update(follower_running=False, motion={'mode': 'NAVIGATION', 'healthy': True},
                 gnss=sample_bad_heading_gnss(), line=sample_route_line())
    route['wheeltec'] = dict(route['wheeltec'], guard={'navigation_uses_lidar': False, 'bypassed': True})
    tel_route = app_link.build_telemetry(route, now=100.0, seq=44, sensors=PROFILE['sensors'],
                                         include_scan=False)
    return {'tel_sample.json': tel, 'tel_line_sample.json': tel_line, 'tel_route_sample.json': tel_route,
            'route_geometry_sample.json': route_geometry_cases(), 'hello_sample.json': hello}


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
