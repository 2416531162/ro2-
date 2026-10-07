"""圆作业禁喷区间记账；追加圈只覆盖欠喷位置，不操作任何泵接口。"""
import math
from path_tracker import Path
import route_plan as rp


def initialise(record,mission,sections,paint_start):
    if 'circle_spray' in record or record.get('field_plan') or record.get('kind')!='route':return
    spec=mission._route_local()
    if spec.kind!='circle' or paint_start is None or sections[-1].direction!=1:return
    k=sections[-1].path.pieces[-1].kappa
    if abs(k)<1e-6:return
    lap=2*math.pi/abs(k);bins=max(16,min(20000,math.ceil(lap/.02)))
    offset=0.;start=None
    for sec in sections:
        if offset+sec.path.length>=paint_start-1e-8:
            start=sec.car_pose(max(0.,paint_start-offset));break
        offset+=sec.path.length
    nozzle=record.get('route_nozzle') or {}
    f,l=nozzle.get('forward',mission.nozzle[0]),-nozzle.get('right',-mission.nozzle[1])
    q=rp.nozzle_at(*start,f,l);sign=-1 if spec.clockwise else 1
    record['circle_spray']=dict(lap_m=lap,bins=bins,paint_start_m=paint_start,
        original_end_m=record['total_m'],wanted=spec.laps,good=[0.]*bins,last_s=record['cursor_m'],
        last_allowed=False,last_phase=False,missing_seen=False,extra_added=False,
        start_fraction=(sign*math.atan2(q[1]-spec.center[1],q[0]-spec.center[0])/(2*math.pi))%1,
        stage='original',error=None)


def add_good(data,a,b):
    if b<=a:return
    lap=data['lap_m'];step=lap/data['bins']
    first,last=math.floor(a/step),math.floor((b-1e-9)/step)
    if last-first>data['bins']*12:raise ValueError('圆作业进度跳变过大，不能标记补喷完成')
    for i in range(first,last+1):
        overlap=max(0.,min(b,(i+1)*step)-max(a,i*step))
        data['good'][i%data['bins']]+=overlap/step


def step(record,cursor,phase,allowed):
    data=record.get('circle_spray')
    if not data:return
    a=max(0.,data['last_s']-data['paint_start_m']);b=max(a,cursor-data['paint_start_m'])
    if phase:
        if allowed and (data['last_allowed'] or not data['last_phase']):add_good(data,a,b)
        elif b>a and cursor<=data['original_end_m']+1e-6:data['missing_seen']=True
    elif data['last_phase'] and data['last_allowed']:
        # DONE 正常收尾、或者进入已覆盖的盖头段：当前关喷不等于刚经过的区间欠喷。
        # 只结清上一拍已允许喷线的区间，不能因正常离开喷线阶段把 missing_seen 置为真。
        add_good(data,a,b)
    data['last_s']=cursor;data['last_allowed']=bool(allowed);data['last_phase']=bool(phase)
    data['stage']='repair' if data['extra_added'] and cursor>=data['original_end_m']-1e-6 else 'original'


def allowed(record,cursor):
    data=record.get('circle_spray')
    if not data:return True
    distance=max(0.,cursor-data['paint_start_m']);index=min(data['bins']-1,int((distance%data['lap_m'])/data['lap_m']*data['bins']))
    if cursor<data['original_end_m']-1e-6:
        # 原计划尾部的盖头段不重复喷已经足量的位置。
        return distance<data['wanted']*data['lap_m'] or data['good'][index]<data['wanted']-.15
    return data['extra_added'] and data['good'][index]<data['wanted']-.15


def windows(data,cursor):
    # finish 会把任务进度设到计划终点，但那不是新的喷线观测；未观察到的收尾不能凭空算成欠喷。
    observed=min(cursor,data['last_s'])
    distance=max(0.,observed-data['paint_start_m']);lap=data['lap_m'];step_m=lap/data['bins']
    complete=observed>=data['original_end_m']-1e-6
    output=[];begin=None
    for i,good in enumerate(data['good']):
        expected=data['wanted'] if complete else min(data['wanted'],max(0,math.floor(distance/lap))+min(1.,max(0.,distance%lap-i*step_m)/step_m))
        missing=good<expected-.15
        if missing and begin is None:begin=i
        if not missing and begin is not None:output.append((begin/data['bins'],i/data['bins']));begin=None
    if begin is not None:output.append((begin/data['bins'],1.))
    result=[]
    for a,b in output:
        start=(a+data['start_fraction'])%1;end=start+(b-a)
        if end<=1:result.append([start,end])
        else:result.extend([[start,1.],[0.,end-1.]])
    return result


def summary(record,cursor):
    data=record.get('circle_spray')
    if not data:return None
    ranges=windows(data,cursor)
    return dict(stage=data['stage'],extra_added=data['extra_added'],max_extra_laps=1,
        pending_windows=ranges,lap_remaining_m=data['lap_m']-max(0.,cursor-data['paint_start_m'])%data['lap_m'],
        pending_fraction=sum(b-a for a,b in ranges),error=data['error'])


def append_path(path,lap):
    k=path.pieces[-1].kappa
    if abs(k)<1e-6:raise ValueError('补喷圈缺少连续圆弧，不能自动追加')
    return Path(path.pose(0),[(p.length,p.kappa) for p in path.pieces]+[(lap,k)],name=path.name)
