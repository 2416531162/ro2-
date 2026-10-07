"""运行中只改划线巡航目标，不重建路径或控制器。"""
import copy
import math
import time
from motion_speeds import DEFAULT_TRANSIT_SPEED, MAX_SECTION_SPEED
from path_tracker import SectionTracker, DIFF_REVERSE_MPS
import route_plan as rp


def entry_speed(cruise):
    """普通路线接入（不喷）的速度：同场地转场 DEFAULT_TRANSIT_SPEED（2026-10-07 用户：「离点很远也要提速到 1.5」）；
    离正式路线 3 m 内由 SectionTracker 按画线速度收（PathConfig.approach_zone_m）。"""
    return max(cruise, DEFAULT_TRANSIT_SPEED)


def label_sections(sections, entry_m, cruise):
    """为普通路线保留接入边界；前进的接入段按 entry_speed，倒车段由 SectionTracker 按倒车速度封顶。"""
    remaining=entry_m
    for sec in sections:
        mark=min(sec.path.length,max(0.,remaining))
        remaining-=sec.path.length
        if getattr(sec,'field_role',None) is not None:
            if getattr(sec,'cruise_mps',None) is None:
                sec.cruise_mps=entry_speed(cruise) if sec.field_role=='transit' and sec.direction>0 else cruise
            continue
        sec.field_role='transit' if mark>=sec.path.length-1e-8 else 'paint'
        sec.paint_from_m=mark
        if getattr(sec,'cruise_mps',None) is None:
            sec.cruise_mps=entry_speed(cruise) if sec.field_role=='transit' and sec.direction>0 else cruise
        if sec.field_role=='paint' and mark>0 and sec.direction>0:
            sec.entry_speed_mps=entry_speed(cruise)


def speed_for(sec,record,cruise,reverse_limit=DIFF_REVERSE_MPS,yaw_rate=None,lateral_accel=None):
    """场地刷线段与 field_planner.speed_for(paint=True) 同一公式：喷头按 cruise 走，再按转向角速度、弯道加速度封顶
    （角球弧 0.8 m/s 时规划 0.52，只换算不封顶会写成 1.16）。刷线部分只有一种曲率，助跑和绕圈不算。"""
    role=getattr(sec,'field_role',None)
    if role=='transit':return None
    if role!='paint':raise ValueError('当前作业缺少速度分段信息，请重新开始作业')
    value=cruise
    if record.get('field_plan'):
        context=record.get('field_context') or {}
        f,l=context.get('forward'),context.get('right')
        if any(type(x) not in (int,float) or not math.isfinite(x) for x in (f,l)):
            raise ValueError('任务缺少喷头参数，不能在线调速')
        if any(type(x) not in (int,float) or not math.isfinite(x) or x<=0 for x in (yaw_rate,lateral_accel)):
            raise ValueError('缺少底盘弯道限速参数，不能在线调速')
        l=-l
        mark=min(sec.path.length,max(0.,getattr(sec,'paint_from_m',0.)))
        k=sec.path.kappa(mark+1e-6)
        value=cruise/max(1e-9,math.hypot(1-sec.direction*k*l,sec.direction*k*f))
        k=sec.path.kappa_ahead(mark,sec.path.length-mark)
        if k>1e-9:value=max(.02,min(value,yaw_rate/k,math.sqrt(lateral_accel/k)))
    value=min(MAX_SECTION_SPEED,value)
    return min(value,reverse_limit) if sec.direction<0 else value


def change(operations,cmd,now):
    m=operations.m;operations.check_identity(cmd)
    cruise=rp._number(cmd.get('cruise'),'划线巡航速度',.1,1.)
    if (m.task!='route' or m.phase!='RUNNING' or m.last_motion_mode!='NAVIGATION'
            or not isinstance(m.tracker,SectionTracker) or m.tracker.st.state!='RUNNING'
            or not operations.record or operations.record.get('state')!='RUNNING'):
        raise ValueError('只可在正在执行的作业中切换速度')
    if m.pose is None or not m.pose.usable_for_control:
        raise ValueError('定位不可用，不能切换巡航速度')
    if operations.record.get('field_plan') and not operations.field_source_usable(operations.record):
        raise ValueError('场地参数已改变，不能在线调速')
    # 先校验并保存目标；磁盘错误时控制器和旧目标完全不动。
    old=operations.record;updated=copy.deepcopy(old)
    sections=operations.decode(updated)
    # 与开始作业、续画（route_operations.prepare）的规划参数同源
    limits=dict(reverse_limit=m.section_options().get('reverse_mps',.3),
                yaw_rate=m.geo.pivot_rate_rad_s,lateral_accel=m.path_cfg.lat_accel_mps2)
    for raw,sec in zip(updated['plan'],sections):
        value=speed_for(sec,updated,cruise,**limits)
        if value is not None:raw['speed']=value
    changes=[(sec,tr,speed_for(sec,updated,cruise,**limits)) for sec,tr in zip(m.tracker.sections,m.tracker.trackers)]
    updated.update(cruise=cruise,updated_at=time.time())
    if updated.get('field_plan'):updated['field_profile']=operations.field_profile(updated)
    operations.record=updated
    try:operations.save()
    except (ValueError,OSError):
        operations.record=old
        raise
    for sec,tr,value in changes:
        if value is not None:
            sec.cruise_mps=value
            tr.set_cruise(value)
    m.cruise=cruise
    operations.preview=None
    return '划线巡航目标已设为 %.1f m/s，正在平滑调整' % cruise
