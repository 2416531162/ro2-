#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RTK 路径跟踪闭环仿真：小圆、跑道形、8 字、app 画的路点路线（带接入段）。控制器是 path_tracker.PathTracker。

定位来源（source）：
    fused  原始传感器 → 真实融合（默认）：与 line_sim.run_fused 同一套模型——合成 UM982 日志（20 Hz，
           位置慢漂 6 mm + 白噪声 3 mm，航向慢漂 0.1° + 白噪声 0.25°，天线安装偏差 1.5°）、IMU（零偏、噪声、
           量化、尖峰）、轮速（2% 比例误差），经 Um982Stream 和 LineMission 的输入转换送进 gnss_fusion。
           ideal=True 时把传感器噪声、延迟都关掉，只剩控制器和底盘本身的误差。
    truth  直接用底盘真值当位姿（无噪声、无延迟），看控制律和底盘动态的下限。
底盘与 line_sim 相同：指令延迟 60 ms、驱动加速斜坡、舵机 3 rad/s、舵角量化 0.2°，可加舵机零位偏差。
横偏统计用真值：真实后轴中心到路径的距离。只用于离线调参和回归，不代表实车（没有轮胎侧滑）。

    python3 radar_system/tools/path_sim.py                      # 全部场景
    python3 radar_system/tools/path_sim.py circle_r2 figure8_r2 --seeds 8
    python3 radar_system/tools/path_sim.py circle_r2 --source truth
"""
import argparse
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

from line_sim import CONTROL_DT, DT, NO_POSE, Chassis, FusedSource, GaussMarkov  # noqa: E402
from path_tracker import PathConfig, PathTracker, circle, figure8, stadium, wrap  # noqa: E402
import route_plan  # noqa: E402

TRANSIENT_M = 0.6          # 曲率变化后这么长算过渡段，其余算稳态


class TruthSource:
    """底盘真值直接当位姿：没有噪声和延迟。"""

    def __init__(self):
        self.car = None

    def step(self, t, car):
        self.car = car

    def pose(self, t):
        c = self.car
        return _Pose(c.x, c.y, c.yaw, c.v, c.wz + getattr(c, 'extra_wz', 0.0))


class _Pose:
    quality, sigma_xy, sigma_yaw, reasons, usable_for_control = 'TRUTH', 0.0, 0.0, (), True

    def __init__(self, x, y, yaw, speed, yaw_rate):
        self.x, self.y, self.yaw, self.speed, self.yaw_rate = x, y, yaw, speed, yaw_rate


def _fused_source(seed, ideal, mount_deg, mount_err_deg, sensor_kw):
    import gnss_fusion
    rx = dict(pos_drift=0.006, pos_white=0.003, hdg_drift_deg=0.1, hdg_white_deg=0.25, vel_noise=0.02,
              mount_deg=mount_deg, outage=None, heading_outage=None)
    mo = dict(gyro_bias_dps=0.1, gyro_noise_dps=0.034, spikes=True, wheel_error=0.02, imu_delay=0.02,
              wheel_delay=0.03)
    if ideal:
        rx.update(pos_drift=0.0, pos_white=0.0, hdg_drift_deg=0.0, hdg_white_deg=0.0, vel_noise=0.0)
        mo.update(gyro_bias_dps=0.0, gyro_noise_dps=0.0, spikes=False, wheel_error=0.0, imu_delay=0.0,
                  wheel_delay=0.0)
    for k, v in sensor_kw.items():
        (rx if k in rx else mo)[k] = v
    return FusedSource(seed, gnss_fusion.FusionConfig(mount_offset_deg=mount_deg - mount_err_deg), rx, mo)


def run(name='circle_r2.5', *, path=None, cruise=0.5, start=(0.0, 0.0, 0.0), source='fused', ideal=False,
        trim_deg=0.0, quant_deg=0.2, terrain=0.0, mount_deg=1.5, mount_err_deg=0.0, config=None, seed=1,
        trace=False, warmup_s=3.0, max_time=None, integral0=None, **sensor_kw):
    """start=(沿路径, 横偏, 车头误差°)，相对路径起点。返回指标字典。"""
    path = path or SCENARIOS[name]['path']()
    rng = random.Random(seed)
    cfg = config or PathConfig(cruise_mps=cruise)
    tracker = PathTracker(path, cfg)
    x0, y0, yaw0 = path.pose(0.0)
    car = Chassis(trim_rad=math.radians(trim_deg), steer_quant_rad=math.radians(quant_deg))
    along, lat, hd = start
    car.x = x0 + along * math.cos(yaw0) - lat * math.sin(yaw0)
    car.y = y0 + along * math.sin(yaw0) + lat * math.cos(yaw0)
    car.yaw = yaw0 + math.radians(hd)
    if source == 'truth':
        src, warmup_s = TruthSource(), 0.0
    else:
        src = _fused_source(seed, ideal, mount_deg, mount_err_deg, sensor_kw)
    bumps = GaussMarkov(terrain, 0.5, rng)

    t, next_ctrl, started = 0.0, 0.0, False
    s_true = None
    rows = []
    max_time = max_time or (warmup_s + path.length / cruise * 1.5 + 20.0)
    while t < max_time:
        src.step(t, car)
        if t >= next_ctrl - 1e-9:
            next_ctrl += CONTROL_DT
            pose = src.pose(t) or NO_POSE
            if not started and t >= warmup_s:
                started = tracker.start(pose, integral=integral0)
                if not started and t > warmup_s + 5.0:
                    break
            if started:
                vx, wz = tracker.update(pose, CONTROL_DT)
                car.command(t, vx, wz)
                s_true, e_true, _, k_true = path.project(car.x, car.y, s_true, extend_start=cfg.start_before_m)
                st = tracker.st
                rows.append(dict(
                    t=t, x=car.x, y=car.y, s=s_true, e_true=e_true, e_ctrl=st.e, kappa_ref=k_true,
                    kappa_cmd=st.kappa, steer=car.steer, v=car.v, state=st.state,
                    pos_err=None if pose is NO_POSE else math.hypot(pose.x - car.x, pose.y - car.y),
                    yaw_err=None if pose is NO_POSE else wrap(pose.yaw - car.yaw)))
                if st.state in ('DONE', 'ABORTED') and abs(car.v) < 1e-3:
                    break
        car.step(t, DT)
        if terrain > 0:
            car.extra_wz = bumps.step(DT) * car.v
            car.yaw += car.extra_wz * DT
        t += DT
    return summarize(name, path, tracker, rows, t, trace)


def summarize(name, path, tracker, rows, t, trace=False):
    """横偏（真值）统计：从入口直线结束一直算到终点（含出口直线）；过渡 = 每次曲率变化后 TRANSIENT_M 米内。"""
    loop_a = path.pieces[0].length
    changes = [p.s0 for i, p in enumerate(path.pieces) if i and p.kappa != path.pieces[i - 1].kappa]
    run = [r for r in rows if r['state'] == 'RUNNING']
    loop = [r for r in run if loop_a <= r['s']]
    trans = [r for r in run if any(0.0 <= r['s'] - c <= TRANSIENT_M for c in changes)]
    steady_arc = [r for r in loop if abs(r['kappa_ref']) > 1e-9 and r not in trans]

    def rms(sel):
        return round(math.sqrt(sum(r['e_true'] ** 2 for r in sel) / len(sel)) * 100, 2) if sel else None

    def peak(sel, key='e_true', scale=100.0):
        vals = [abs(r[key]) for r in sel if r[key] is not None]
        return round(max(vals) * scale, 2) if vals else None

    kmax = tracker.kappa_max
    last = rows[-1] if rows else None
    usable = [r for r in run if r['pos_err'] is not None]
    result = dict(
        name=name, path=path.name, state=tracker.st.state, reason=tracker.st.reason,
        radius_m=round(1.0 / path.kappa_abs_max, 2) if path.kappa_abs_max else None,
        rms_cm=rms(loop), max_cm=peak(loop), arc_rms_cm=rms(steady_arc), arc_max_cm=peak(steady_arc),
        trans_max_cm=peak(trans),
        steer_max_deg=round(math.degrees(max((abs(r['steer']) for r in loop), default=0.0)), 1),
        saturated_pct=round(100.0 * sum(abs(r['kappa_cmd']) >= 0.999 * kmax for r in run) / max(1, len(run)), 1),
        end_err_cm=None if last is None else round((last['s'] - path.length) * 100, 1),
        fusion_pos_max_cm=peak(usable, 'pos_err'), fusion_yaw_max_deg=peak(usable, 'yaw_err', math.degrees(1.0)),
        time_s=round(t, 2), integral=tracker.st.integral)
    if trace:
        result['rows'] = rows
    return result


def app_route(points, *, laps=2, turn_radius=2.5, car=(-2.0, -3.0, math.pi / 2), closed=True):
    """app 画的路点路线：从 car 位姿经 Dubins 接入，拐角圆弧过渡（与 line_mission 走路线同一套几何）。"""
    from robot_core.kinematics import ChassisGeometry
    spec = route_plan.parse_spec(dict(kind='points', points=points, closed=closed, laps=laps,
                                      turn_radius=turn_radius))
    return route_plan.plan(spec, car, ChassisGeometry().min_turn_radius_m / PathConfig().kappa_margin)[0]


def app_path(points, *, closed=False, laps=1, car=(0.0, 0.0, 0.0)):
    """连线路线：点按顺序连直线，从 car 位姿先开到第 1 个点（与 line_mission 同一套几何）。"""
    from robot_core.kinematics import ChassisGeometry
    spec = route_plan.parse_spec(dict(kind='path', points=points, closed=closed, laps=laps))
    return route_plan.plan(spec, car, ChassisGeometry().min_turn_radius_m / PathConfig().kappa_margin)[0]


SCENARIOS = {
    'circle_r3': dict(path=lambda: circle(3.0)),
    'circle_r2.5': dict(path=lambda: circle(2.5)),
    'circle_r2': dict(path=lambda: circle(2.0)),
    'circle_r2_cw': dict(path=lambda: circle(2.0, left=False)),
    'circle_r2.5_2laps': dict(path=lambda: circle(2.5, laps=2)),
    'circle_r2.5_fast': dict(path=lambda: circle(2.5), cruise=0.8),
    'circle_r2.5_trim2': dict(path=lambda: circle(2.5), trim_deg=2.0),
    'circle_r2.5_noisy': dict(path=lambda: circle(2.5), pos_drift=0.012, pos_white=0.006, hdg_drift_deg=0.2,
                              hdg_white_deg=0.5),
    'circle_r2.5_offset': dict(path=lambda: circle(2.5), start=(-1.0, 0.30, 10.0)),
    'circle_r2.5_mount1': dict(path=lambda: circle(2.5), mount_err_deg=1.0),
    'stadium_r2.5': dict(path=lambda: stadium(2.5, 4.0)),
    'figure8_r2.5': dict(path=lambda: figure8(2.5)),
    'figure8_r2': dict(path=lambda: figure8(2.0)),
    # app 画的路线：矩形 2 圈、五边形（锐角拐弯）拐弯半径贴近底盘极限
    'route_rect': dict(path=lambda: app_route([[0, 0], [10, 0], [10, 8], [0, 8]])),
    'route_penta_r2': dict(path=lambda: app_route([[0, 0], [12, 0], [15, 9], [4, 11], [-3, 5]], turn_radius=2.0)),
    'route_open_zigzag': dict(path=lambda: app_route([[0, 0], [8, 0], [10, 7], [16, 1], [22, 6]], closed=False,
                                                     laps=1, car=(-3.0, 1.0, -0.3))),
    # 连线路线（app 默认）：操作员点的点按顺序连直线，参考线是车能开的那条
    'path_zigzag': dict(path=lambda: app_path([[3, 0], [9, 0], [12, 5], [16, 1], [22, 4]])),
    'path_close_points': dict(path=lambda: app_path([[3 + 0.1 * i, 0.4 * math.sin(0.15 * i)] for i in range(120)])),
    'path_square_loop': dict(path=lambda: app_path([[3, 0], [11, 0], [11, 7], [3, 7]], closed=True, laps=2)),
}


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('names', nargs='*')
    p.add_argument('--source', choices=('fused', 'truth'), default='fused')
    p.add_argument('--ideal', action='store_true', help='融合链路但传感器无噪声、无延迟')
    p.add_argument('--seeds', type=int, default=1, help='跑几个随机种子，取最差')
    args = p.parse_args()
    keys = ('state', 'rms_cm', 'max_cm', 'arc_rms_cm', 'arc_max_cm', 'trans_max_cm', 'steer_max_deg',
            'saturated_pct', 'end_err_cm', 'fusion_pos_max_cm', 'fusion_yaw_max_deg')
    print('%-20s' % 'scenario' + ''.join('%15s' % k for k in keys))
    for n in args.names or list(SCENARIOS):
        kw = {k: v for k, v in SCENARIOS[n].items() if k != 'path'}
        rs = [run(n, seed=s, source=args.source, ideal=args.ideal, **kw) for s in range(1, args.seeds + 1)]

        def worst(k):
            vals = [r[k] for r in rs if isinstance(r[k], (int, float))]
            if k == 'state':
                return '/'.join(sorted({r['state'] for r in rs}))
            return max(vals, key=abs) if vals else None
        print('%-20s' % n + ''.join('%15s' % (worst(k),) for k in keys))


if __name__ == '__main__':
    main()
