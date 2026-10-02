#!/usr/bin/env python3
"""Gen5 原地转向和圆轨迹离线评估；只用数学底盘，不加载 ROS，不向车辆发指令。"""
import argparse
from dataclasses import replace
import importlib.util
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'tests'), str(ROOT/'radar_system'), str(ROOT)]
from gen5_line_sim import Gen5Chassis, GEN5, make_pose, percentile, Disturbance, PoseNoise
from robot_core.config import CONTROL_DT
import path_tracker


def load_tracker(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def pivot(module=path_tracker, angle_deg=180., delay=0.39, gain=1.03, rate=0.8, accel=1.0,
          trace=None, pause_at=None):
    cfg = module.differential_config(module.PathConfig())
    target = math.radians(angle_deg)
    sections = [module.Section(1, module.Path((0., 0., target), [(3., 0.)]))]
    tracker = module.SectionTracker(sections, cfg, GEN5, pivot_rate=rate, pivot_accel=accel, reverse_gain=1.)
    car = Gen5Chassis(0., 0., 0., delay_s=delay, gain=gain)
    tracker.start(make_pose(car.x, car.y, car.yaw, car.v, car.wz))
    starts = reversals = 0
    was_turning = False
    sign = 0
    commands = []
    settled = False
    paused = False
    next_ctrl = 0.0
    for i in range(4500):
        t = i*.01
        if t >= next_ctrl - 1e-9:
            next_ctrl += CONTROL_DT
            pose = make_pose(car.x, car.y, car.yaw, car.v, car.wz)
            if pause_at is not None and not paused and t >= pause_at:
                tracker.pause('operator')
                paused = True
            if paused and tracker.st.state == 'PAUSED' and t >= pause_at + 2:
                tracker.start(pose)
            v, w = tracker.update(pose, CONTROL_DT)
            turning = tracker._pivot is not None
            starts += bool(turning and not was_turning)
            was_turning = turning
            next_sign = 1 if w > 1e-4 else -1 if w < -1e-4 else 0
            if next_sign:
                reversals += bool(sign and sign != next_sign)
                sign = next_sign
            commands.append((t, w))
            car.command(t, v, w)
            if trace is not None:
                trace.append({'t':t, 'yaw_deg':math.degrees(car.yaw), 'target_deg':angle_deg,
                              'w_cmd':w, 'w_actual':car.wz, 'pivoting':turning})
            if not tracker.switching:
                settled = True
                break
            if tracker.st.state == 'ABORTED':
                break
        car.step(t, .01)
    ended = t
    # 状态已判停稳后，再等一个完整延迟窗口，核对没有藏在队列里的过冲。
    for j in range(100):
        t = ended + j*.01
        car.command(t, 0., 0.)
        car.step(t, .01)
    error = math.degrees(math.atan2(math.sin(target-car.yaw), math.cos(target-car.yaw)))
    return {'angle_deg':angle_deg, 'delay_s':delay, 'gain':gain, 'rate':rate, 'accel':accel,
            'seconds':round(ended,3), 'error_deg':round(error,4), 'turns':starts,
            'reversals':reversals, 'settled':settled, 'state':tracker.st.state, 'reason':tracker.st.reason}


def circle(radius, lateral_accel, seed, clockwise=True):
    """从已对正的圆起点静止起步；统计真实喷头，另列按含噪融合位姿计算的误差。

    不模拟转向后中心平移、驱动器几十 erpm 时的失稳；因此不能据此承诺实车小圆精度。
    """
    import route_plan
    spec = route_plan.parse_spec(dict(kind='circle',center=[0.,0.],radius=radius,clockwise=clockwise,laps=1,cruise=.8))
    start,pieces = route_plan.loop_pieces(spec,GEN5.min_turn_radius_m/.9,nozzle_x=.30)
    path = path_tracker.Path(start,pieces)
    cfg = replace(path_tracker.differential_config(path_tracker.PathConfig(cruise_mps=.8)),lat_accel_mps2=lateral_accel)
    tr = path_tracker.PathTracker(path,cfg,GEN5)
    car = Gen5Chassis(*start,dist=Disturbance(seed))
    noise = PoseNoise(seed+100)
    tr.start(make_pose(car.x,car.y,car.yaw,car.v,car.wz))
    errors,reported,first,speeds,inner = [],[],[],[],[]
    travelled = 0.
    next_ctrl = 0.0
    for i in range(40000):
        t=i*.01
        if t >= next_ctrl - 1e-9:
            next_ctrl += CONTROL_DT
            dx,dy,dyaw = noise.step(CONTROL_DT)
            pose = make_pose(car.x+dx,car.y+dy,car.yaw+dyaw,car.v,car.wz)
            v,w=tr.update(pose,CONTROL_DT)
            car.command(t,v,w)
            nx,ny=car.nozzle()
            e=abs(math.hypot(nx,ny)-radius)*100
            if travelled<=1:
                first.append(e)
            if travelled>1 and travelled<path.length-.5 and car.v>.05:
                errors.append(e)
                reported.append(abs(math.hypot(pose.x+.30*math.cos(pose.yaw),pose.y+.30*math.sin(pose.yaw))-radius)*100)
                speeds.append(car.v)
                # 实车磁极对数 20、轮半径 0.2046；只报告转速，不把某一阈值当成已验证的稳定边界。
                inner.append(min(abs(car.v-.3375*car.wz),abs(car.v+.3375*car.wz))*60*20/(2*math.pi*.2046))
            if tr.st.state in ('DONE','ABORTED','PAUSED'):
                break
        car.step(t,.01)
        travelled+=abs(car.v)*.01
    if not errors:
        raise ValueError('没有足够的圆轨迹样本')
    return {'radius_m':radius,'lat_accel':lateral_accel,'seed':seed,'clockwise':clockwise,
            'state':tr.st.state,'reason':tr.st.reason,'seconds':round(t,2),'v_mean':sum(speeds)/len(speeds),
            'p95_cm':percentile(errors,.95),'max_cm':max(errors),'first_m_max_cm':max(first),
            'reported_p95_cm':percentile(reported,.95),'inner_erpm_p05':percentile(inner,.05)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--circles',action='store_true')
    args = parser.parse_args()
    if args.circles:
        rows=[]
        for radius in (1.,1.2,3.,9.15):
            for acc in (.08,.15,.2,.3):
                group=[circle(radius,acc,seed,direction) for seed in (1,2,3,4) for direction in (True,False)]
                rows.extend(group)
                print('圆',radius,'横向加速度',acc,'平均车速',round(sum(r['v_mean'] for r in group)/len(group),3),
                      '最差种子95%',round(max(r['p95_cm'] for r in group),2),'厘米',flush=True)
        args.out.write_text(json.dumps(rows,ensure_ascii=False,indent=2))
        return
    before = load_tracker(args.baseline, 'path_tracker_baseline') if args.baseline else path_tracker
    result = {'baseline':[pivot(before, a) for a in (90.,180.)],
              'fixed':[pivot(path_tracker,a) for a in (90.,180.)],
              'grid':[pivot(path_tracker,a,d,g) for a in (90.,180.) for d in (.3,.39,.45,.5,.6) for g in (.9,1.,1.03,1.2)],
              'speed_comparison':[pivot(path_tracker,a,rate=r,accel=k) for a in (90.,180.) for r in (.8,1.2,1.5) for k in (1.,2.)]}
    args.out.write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print('原版',result['baseline'])
    print('修复',result['fixed'])
    print('网格最慢 / 最大误差 / 最多反向 / 最多转向次数',
          max(r['seconds'] for r in result['grid']),max(abs(r['error_deg']) for r in result['grid']),
          max(r['reversals'] for r in result['grid']),max(r['turns'] for r in result['grid']))
    print('网格未完成',[r for r in result['grid'] if not r['settled']])


if __name__ == '__main__':
    main()
