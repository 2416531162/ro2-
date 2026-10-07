#!/usr/bin/env python3
"""车端代码仿真公共入口：网页与 CLI 调用同一执行器，不连接 ROS、网络或实车。"""
import argparse
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import spray_sim_chassis as gen5
from spray_simulation import SpraySettings, SimulatedSpray, stroke_length

MODELS = ('gen5', 'legacy')
TURNS = ('pivot', 'arc', 'lock')   # 转场掉头：原地转 / 圆弧（turn_radius）/ 打满方向（车端最小半径）；旧车端不认时按原地转
HEAD_M = 1.0          # 线头统计：每条线开始喷后的这段路程


def number(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{name}须在 {low}–{high} 之间')
    return value


def normalize(request):
    if not isinstance(request, dict):
        raise ValueError('仿真请求须为对象')
    cfg = request.get('settings', {})
    if not isinstance(cfg, dict):
        raise ValueError('设置须为对象')
    defaults = dict(seed=1, seeds=1, max_seconds=3600, source='fused', nozzle_forward=.24,
                    nozzle_right=.4575, paint_speed=.8, transit_speed=1.5, start_x=0, start_y=0, start_heading=0,
                    model='gen5', calibration=dict(gen5.CALIBRATION_DEFAULT), plan_only=False,
                    turn='pivot', turn_radius=1.0)
    limits = dict(seed=(1, 999999), seeds=(1, 10), max_seconds=(10, 21600), nozzle_forward=(-3, 3),
                  nozzle_right=(.001, 2), paint_speed=(.05, 1.2), transit_speed=(.05, 1.5),
                  start_x=(-1000, 1000), start_y=(-1000, 1000), start_heading=(-360, 360), turn_radius=(.3, 5))
    for key, value in defaults.items():
        defaults[key] = cfg.get(key, value)
        if key in limits:
            number(defaults[key], key, *limits[key])
    for key in ('seed', 'seeds'):
        if int(defaults[key]) != defaults[key]:
            raise ValueError(f'{key}须为整数')
        defaults[key] = int(defaults[key])
    if defaults['turn'] not in TURNS:
        raise ValueError('掉头方式无效')
    if not isinstance(defaults['plan_only'], bool):
        raise ValueError('plan_only 须为 true / false')
    if defaults['plan_only']:
        defaults['seeds'] = 1          # 只规划：同一起点规划结果与种子无关
    if defaults['source'] not in ('fused', 'truth'):
        raise ValueError('定位来源无效')
    if defaults['model'] not in MODELS:
        raise ValueError('仿真模型无效')
    if not isinstance(defaults['calibration'], dict):
        raise ValueError('定位标定须为对象')
    defaults['calibration'] = gen5.check_calibration(defaults['calibration'])
    defaults['spray'] = vars(SpraySettings.parse(cfg.get('spray')))
    job = request.get('job')
    if not isinstance(job, dict):
        raise ValueError('请先载入场地')
    if 'boundary' in job and 'elements' in job:
        field = job
    elif job.get('format') == 'follow-cockpit-field-job':
        template = job.get('template', {})
        length = number(template.get('length'), '场地长度', 1, 500)
        width = number(template.get('width'), '场地宽度', 1, 500)
        entries = job.get('sequence', []) + job.get('manual', [])
        field = dict(boundary=[[0, 0], [length, 0], [length, width], [0, width]], elements=[
            dict(e['field'], id=e['id'], name=e.get('name', e['id']), kind=e['kind']) for e in entries])
    else:
        raise ValueError('需要场地设计器的整场 JSON，或 boundary / elements 格式')
    return dict(job=field, settings=defaults, name=str(request.get('name') or job.get('name') or '车端代码仿真')[:100])


def distance(point, stroke):
    """独立按原始图纸计算喷头距离，不使用控制器 e，也不用显示采样线。"""
    x, y = point
    if stroke['kind'] == 'line':
        a, b = stroke['a'], stroke['b']
        dx, dy = b[0]-a[0], b[1]-a[1]
        u = max(0, min(1, ((x-a[0])*dx+(y-a[1])*dy)/(dx*dx+dy*dy)))
        return math.hypot(x-a[0]-u*dx, y-a[1]-u*dy)
    cx, cy = stroke['c']; r = stroke['r']
    if stroke['kind'] == 'circle':
        return abs(math.hypot(x-cx, y-cy)-r)
    angle = math.atan2(y-cy, x-cx); start = stroke['start']; sweep = stroke['sweep']
    rel = ((angle-start) * (1 if sweep >= 0 else -1)) % (2*math.pi)
    if rel <= abs(sweep):
        return abs(math.hypot(x-cx, y-cy)-r)
    return min(math.hypot(x-cx-r*math.cos(a), y-cy-r*math.sin(a)) for a in (start, start+sweep))


def nozzle(pose, forward, right):
    x, y, h = pose
    return (x + forward*math.cos(h) + right*math.sin(h), y + forward*math.sin(h) - right*math.cos(h))


def metrics(values):
    vals = sorted(values)
    return dict(samples=len(vals), p95_m=vals[int(.95*(len(vals)-1))] if vals else None,
                max_m=vals[-1] if vals else None, rms_m=math.sqrt(sum(v*v for v in vals)/len(vals)) if vals else None)


LIVE_MAX_FRAMES = 2400
PLAN_KEYS = ('boundary','target','preview','order','manual','distance_m','duration_s','max_outside_m','start',
             'turn','turn_radius','pivot_count','reverse_count')


def live_sample(frames, limit=LIVE_MAX_FRAMES):
    """实时画面有界抽样，优先保留划线区间首尾；完整回放与统计不变。"""
    if limit < 2:
        raise ValueError('实时抽样至少保留两个点')
    if len(frames) <= limit:
        return frames[:]
    protected = {0, len(frames)-1}
    def key(frame):
        return tuple(frame[9:12]) + (frame[15] if len(frame)>15 else None,)
    for i in range(1,len(frames)):
        if key(frames[i]) != key(frames[i-1]): protected.update((i-1,i))
    if len(protected) >= limit:
        indices=sorted(protected)
        return [frames[indices[i*(len(indices)-1)//(limit-1)]] for i in range(limit)]
    others=[i for i in range(len(frames)) if i not in protected]
    remaining=limit-len(protected)
    protected.update(others[i*len(others)//remaining] for i in range(remaining))
    return [frames[i] for i in sorted(protected)]


def painting_now(mission, plan, index, tracker):
    """喷不喷用车端自己的 job.painting（含刷线段前的助跑、换段、原地转、暂停的判断）；旧车端没有这一项时按刷线段、
    不在换段、跟踪 RUNNING。"""
    ops = getattr(mission, 'operations', None)
    if ops is not None:
        job = (ops.status() or {}).get('job') or {}
        if 'painting' in job:
            return bool(job['painting'])
    return plan['motions'][index]['role'] == 'paint' and not tracker.switching and tracker.st.state == 'RUNNING'


def run_one(request, seed, record, emit, publish=None, source=None):
    # 导入语句在子进程内执行；服务器进程从不缓存车端模块。
    import gnss_fusion
    import path_sim as ps
    from line_mission import LineMission
    from field_plan_service import FieldPlanService
    from field_planner import parse_strokes
    from robot_core.config import CONTROL_DT, PROFILE, profile_hash
    cfg = request['settings']; job = request['job']
    legacy = cfg['model'] == 'legacy'
    if legacy:
        # 旧模型：path_sim 理想差速底盘 + line_sim 传感器 + FusionConfig 默认值（旧天线位置，不经过车上 line_node 配置）
        car = gen5.IdealChassis(ps)
        src = ps._fused_source(seed, cfg['source'] == 'truth', 1.5, 0.0, {}, differential=True)
        fusion = src.fusion
        fusion_note = 'FusionConfig 默认值（path_sim）'
        dt = ps.DT
    else:
        car = gen5.Gen5Chassis(seed)
        fusion_cfg = gen5.load_fusion_config(str(source), gnss_fusion, cfg['calibration'])
        fusion = gnss_fusion.GnssFusion(fusion_cfg)
        fusion.set_origin(*gen5.ORIGIN)
        fusion_note = '车上 line_node.load_fusion_config + 给定 fusion.json 标定'
        dt = 0.005
    car.x, car.y, car.yaw = cfg['start_x'], cfg['start_y'], math.radians(cfg['start_heading'])
    if cfg['source'] == 'truth':
        class TruthFusion:
            def __getattr__(self, key): return getattr(fusion, key)
            def pose(self, t):
                return SimpleNamespace(x=car.x,y=car.y,yaw=car.yaw,speed=car.v,yaw_rate=car.wz,
                    quality='RTK_FIXED',sigma_xy=0.0,sigma_yaw=0.0,reasons=(),usable_for_control=True)
        control_fusion = TruthFusion()
    else:
        control_fusion = fusion
    mission = LineMission(control_fusion, gnss_fusion)
    if not mission.differential:
        raise ValueError('车端配置不是 Gen5 差速底盘')
    if legacy:
        src.inputs = mission
        step_sensors = lambda t: src.step(t, car)
        control_sensors = lambda t: None
        sensor_model = dict(model='legacy', sensors='path_sim._fused_source / line_sim.FusedSource')
    else:
        sensors = gen5.Gen5Sensors(seed, fusion, mission, car, cfg['calibration'])
        step_sensors = sensors.step
        control_sensors = sensors.control_tick
        sensor_model = None
    # 同一个传感器仿真源驱动真实任务与融合，静止预热（至少 3 s、等到定位可用），不跳过启动检查。
    next_ctrl = 0.0
    for step in range(int(round(30.0 / dt)) + 1):
        t = step * dt; step_sensors(t)
        if t >= next_ctrl - 1e-9:
            next_ctrl += CONTROL_DT
            control_sensors(t)
            mission.tick(t, 'IDLE')
            if t >= 3.0 and (mission.status().get('pose') or {}).get('usable'):
                break
    t0 = t
    line = mission.status(); line.update(_rx_monotonic=t,motion_mode='IDLE')
    emit(dict(phase='planning', seed=seed, message='调用车端整场规划'))
    live_frames = []
    live_plan = None
    def send_live(phase, state, seconds):
        if publish:
            publish(dict(schema=1,partial=True,phase=phase,frame_count=len(live_frames),
                request=request,replay_seed=seed,runs=[dict(seed=seed,state=state,seconds=seconds,
                    frames=live_sample(live_frames),plan=live_plan,model=dict(body=list(mission.body)))]))
    send_live('planning', 'PLANNING', 0.0)
    planner = FieldPlanService()
    preview = planner.prepare(dict(origin=mission._origin(), job=job,
        nozzle=[cfg['nozzle_forward'],cfg['nozzle_right']],paint_speed=cfg['paint_speed'],transit_speed=cfg['transit_speed'],
        **({} if cfg['turn']=='pivot' else dict(turn=cfg['turn'],turn_radius=cfg['turn_radius']))), line, t)
    plan = planner.get(preview['token'], t)
    live_plan = {k:plan[k] for k in PLAN_KEYS if k in plan}     # 旧车端没有 start
    if cfg['plan_only']:
        # 只规划：车端从这个起点算出的路线，不行驶、不统计精度
        mission._close_log()
        send_live('seed_done', 'PLANNED', 0.0)
        empty = metrics([])
        return dict(seed=seed,state='PLANNED',reason='只规划，未行驶',seconds=0.0,sections=len(plan['motions']),
            truth=empty,estimated=empty,controller=empty,head=empty,elements=[],frames=[],
            manual=parse_strokes(job['elements'])[1],plan=live_plan,
            model=dict(name=cfg['model'],profile_hash=profile_hash(PROFILE),body=list(mission.body),fusion=fusion_note,
                       path_config=asdict(mission.path_cfg),calibration=None if legacy else cfg['calibration']))
    ok, why = mission.command(dict(cmd='route_field_start',field_plan=plan,allow_outside=True), t)
    if not ok: raise ValueError('仿真任务未启动：'+str(why))
    strokes, manual = parse_strokes(job['elements'])
    strokes = {s['id']:s for s in strokes}
    styles={e['id']:e.get('dash') for e in job['elements']}
    for stroke in strokes.values():stroke['dash']=styles.get(stroke['key'][0])
    used={m['element_id'] for m in plan['motions'] if m['role']=='paint'}
    spray=SimulatedSpray([s for key,s in strokes.items() if key in used],SpraySettings.parse(cfg['spray']),cfg['paint_speed'])
    truth_errors, estimated_errors, controls, frames, head_errors = [], [], [], [], []
    per_element, per_head = {}, {}
    head_run, head_from, last_q = None, 0.0, None
    sample_period = max(.1, cfg['max_seconds']/24000)
    next_record = t; next_ctrl = t; last_emit = 0.0; done_at = None
    paint_run = 0
    previous_paint = None
    previous_spraying = None
    total_steps = int(math.ceil(cfg['max_seconds']/dt))
    state, reason = 'TIMEOUT', '达到仿真时间上限'
    for step in range(1, total_steps+1):
        t = t0 + step*dt
        step_sensors(t)
        if t >= next_ctrl - 1e-9:
            next_ctrl += CONTROL_DT
            control_sensors(t)
            vx, wz, _ = mission.tick(t, 'NAVIGATION')
            car.command(t, vx, wz)
            tracker = mission.tracker
            index = tracker.index
            motion = plan['motions'][index]
            painting = painting_now(mission, plan, index, tracker)
            paint_key = (index, painting)
            if paint_key != previous_paint:
                paint_run += 1
                previous_paint = paint_key
            q = nozzle((car.x,car.y,car.yaw),cfg['nozzle_forward'],cfg['nozzle_right'])
            p = mission.pose
            qp = nozzle((p.x,p.y,p.yaw),cfg['nozzle_forward'],cfg['nozzle_right']) if p and p.usable_for_control else None
            stroke = strokes.get(motion['element_id'])
            et = distance(q,stroke) if painting and stroke else None
            ee = distance(qp,stroke) if painting and stroke and qp else None
            ec = tracker.st.e if painting else None
            sec=tracker.sections[index]
            scale=stroke_length(stroke)/max(sec.path.length-getattr(sec,'paint_from_m',0.),1e-6) if stroke else 1.
            sample=spray.update(t-t0,CONTROL_DT,point=q,speed=p.speed if p and p.usable_for_control else 0.,
                painting=painting,eligible=motion['role']=='paint' and not tracker.switching and
                    tracker.st.state=='RUNNING' and mission.phase=='RUNNING' and p is not None and p.usable_for_control,
                progress=tracker.trackers[index].st.s,paint_from=getattr(sec,'paint_from_m',0.),
                length=sec.path.length,scale=scale,stroke=stroke,estimate=qp,heading=p.yaw if p else 0.)
            if et is not None:
                truth_errors.append(et); per_element.setdefault(motion['element_id'], []).append(et)
                # 线头：这条线这次开始喷之后的 HEAD_M 内（续喷 / 换段重新开始也算新的线头）
                if head_run != paint_run:
                    head_run, head_from = paint_run, 0.0
                elif last_q is not None:
                    head_from += math.dist(q, last_q)
                if head_from < HEAD_M:
                    head_errors.append(et); per_head[motion['element_id']] = max(per_head.get(motion['element_id'], 0.0), et)
            last_q = q if et is not None else None
            if ee is not None: estimated_errors.append(ee)
            if ec is not None and math.isfinite(ec): controls.append(abs(ec))
            if (record or publish) and (t >= next_record - 1e-9 or sample['spraying'] != previous_spraying):
                if t >= next_record - 1e-9:next_record += sample_period
                previous_spraying = sample['spraying']
                frame = [round(t-t0,4),round(car.x,6),round(car.y,6),round(car.yaw,6),
                    *[round(v,6) for v in q], *([round(v,6) for v in qp] if qp else [None,None]),
                    round(car.v,5), index, 'paint' if painting else 'transit', tracker.st.state,
                    et,ee,ec,paint_run,sample['flow_pct'],sample['command_pct']]
                if record:
                    if len(frames)>=30000:raise ValueError('模拟喷线变化过密，请缩小图纸范围')
                    frames.append(frame)
                if publish: live_frames.append(frame)
            if time.monotonic()-last_emit > .4:
                send_live('running', tracker.st.state, round(t-t0,4))
                emit(dict(phase='running',seed=seed,sim_seconds=round(t-t0,1),section=index+1,sections=len(plan['motions']),
                    fraction=max(0,min(1,tracker.st.s/max(tracker.length,1e-9))),state=tracker.st.state))
                last_emit = time.monotonic()
            if tracker.st.state in ('DONE','ABORTED'):
                done_at = done_at if done_at is not None else t
                if abs(car.v)<.001 and sample['flow_pct']<.05 or t-done_at>max(3.,cfg['spray']['close_delay_s']+cfg['spray']['response_s']*8):
                    state, reason = tracker.st.state, tracker.st.reason
                    break
        car.step(t,dt)
    mission._close_log()
    if sensor_model is None:
        sensor_model = sensors.describe()
    send_live('seed_done', state, round(t-t0,4))
    return dict(seed=seed,state=state,reason=reason,seconds=round(t-t0,4),sections=len(plan['motions']),
        truth=metrics(truth_errors),estimated=metrics(estimated_errors),controller=metrics(controls),head=metrics(head_errors),
        elements=[dict(id=k,name=strokes[k]['name'],head_max_m=per_head.get(k),**metrics(v)) for k,v in per_element.items()],
        frames=frames, manual=manual, plan=live_plan,spray=spray.result(),
        model=dict(name=cfg['model'],profile_hash=profile_hash(PROFILE),control_dt=CONTROL_DT,physics_dt=dt,
            chassis=car.describe(),sensors=sensor_model,fusion=fusion_note,calibration=None if legacy else cfg['calibration'],
            controller='LineMission / SectionTracker',fusion_config={k:(list(v) if isinstance(v,tuple) else v) for k,v in vars(fusion.config).items()},
            path_config=asdict(mission.path_cfg),body=list(mission.body),painting='车端 job.painting' if 'painting' in ((mission.operations.status() or {}).get('job') or {}) else '刷线段 · 不在换段 · 跟踪 RUNNING'))


def run(request, source, emit=lambda _: None, publish=None):
    request = normalize(request)
    source = Path(source).resolve()
    os.environ['RK3588_ROBOT_CONFIG'] = str(source/'robot_core/robot_gen5.json')
    sys.path[:0] = [str(source/'radar_system/tools'),str(source/'radar_system'),str(source/'oid_protocol'),str(source)]
    results = []
    cfg = request['settings']
    for i in range(cfg['seeds']):
        result = run_one(request,cfg['seed']+i,i==0,emit,publish,source)
        results.append(result)
        emit(dict(phase='seed_done',seed=result['seed'],completed_seeds=i+1,seeds=cfg['seeds'],state=result['state']))
    return dict(schema=1,simulation_only=True,hardware_output=False,name='模拟喷水',request=request,replay_seed=cfg['seed'],runs=results,
        limitations=[*(['只规划：显示车端从所给起点算出的整场路线与预计用时，没有行驶仿真。'] if cfg['plan_only'] else []),
            '同一代码、参数和随机种子可复现；不等同实车验收。',
            *(['旧模型：path_sim 理想差速底盘（纯延迟 0.28 s、无万向轮与扰动）+ line_sim 传感器 + FusionConfig 默认值'
               '（旧天线位置，车上 line_node 的融合配置、测向偏差学习、详细轮程、长直线 IMU 内环都不生效），只用于对照。']
              if cfg['model']=='legacy' else
              ['Gen5 实车标定模型（'+gen5.MODEL_INFO['source']+'）：'+gen5.MODEL_INFO['chassis']+'；'+gen5.MODEL_INFO['sensors']+'。',
               '融合按车上 line_node.load_fusion_config 与所给 fusion.json 标定；闭环核对时巡航横偏比实车偏悲观约 0.3 cm。']),
            '落料为虚拟延迟和喷幅模型，参数需现场标定；结果不等于真实喷涂保证，人工点位单独列出。'])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--request',required=True);p.add_argument('--source',required=True);p.add_argument('--out',required=True)
    p.add_argument('--progress');p.add_argument('--live');args=p.parse_args()
    def emit(value):
        if args.progress:
            path=Path(args.progress);temp=path.with_suffix('.tmp');temp.write_text(json.dumps(value,ensure_ascii=False));temp.replace(path)
    def publish(value):
        path=Path(args.live);temp=path.with_suffix('.tmp')
        temp.write_text(json.dumps(value,ensure_ascii=False,allow_nan=False));temp.replace(path)
    result=run(json.loads(Path(args.request).read_text()),args.source,emit,publish if args.live else None)
    path=Path(args.out);temp=path.with_suffix('.tmp');temp.write_text(json.dumps(result,ensure_ascii=False,allow_nan=False));temp.replace(path)

if __name__=='__main__':main()
