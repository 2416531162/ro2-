"""喷线资格：检查实际喷头和目标几何的距离，不改变车辆导航判定。"""
from robot_core.standstill import moving
import math
import os
import route_plan as rp

# Gen5 没有固定解（浮点、单点、平板中转断流）时按 IMU + 轮速推算接着走、接着喷；满 RTK_LOST_PAINT_STOP_S 先停喷，
# 再走 RTK_LOST_STOP_DELAY_S 才停车（融合 rtk_hold_s = 两者之和，line_node 设），恢复后自动继续
# （2026-10-07 用户：「8 秒内没有一个可用的卫星坐标就停止，停喷也有讲究，先停喷 1 秒，再停止」）
RTK_LOST_PAINT_STOP_S = 8.0
RTK_LOST_STOP_DELAY_S = 1.0

DEFAULT_OFFSET_M=.03


def close_delay():
    """实测关泵到断料的补偿秒数；默认不补偿，非法配置沿用已有配置保护。"""
    try:
        value=float(os.environ.get('RK3588_SPRAY_CLOSE_DELAY_S', '0'))
        return value if math.isfinite(value) and 0<=value<=.5 else None
    except (TypeError,ValueError):return None


def nozzle(m, record):
    """作业快照优先，路线与场地使用各自保存的喷头杆臂。"""
    cfg=(record.get('field_context') if record.get('field_plan') else record.get('route_nozzle')) or {}
    f,l=cfg.get('forward'),-(cfg.get('right') or 0.)
    # 旧阿克曼任务没有喷头配置，交给原有 paint_nozzle_invalid 保护，不让状态查询抛异常。
    return (m.nozzle or (None,None)) if f is None else (f,l)


def offset_limit():
    try:
        value=float(os.environ.get('RK3588_SPRAY_MAX_OFFSET_M',DEFAULT_OFFSET_M))
        return value if math.isfinite(value) and .002<=value<=.10 else None
    except (TypeError,ValueError):return None


def segment_distance(point,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1];length2=dx*dx+dy*dy
    if length2<=1e-12:return math.dist(point,a)
    u=max(0.,min(1.,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/length2))
    return math.hypot(point[0]-a[0]-u*dx,point[1]-a[1]-u*dy)


def continuous_arc(m, section, tracker):
    """仅 Gen5 前进圆弧作业使用持续喷涂，转场是否喷仍由 phase 决定。"""
    return bool(getattr(m,'differential',False) and section.direction>0
                and callable(getattr(tracker,'constant_arc_speed_here',None))
                and tracker.constant_arc_speed_here())


def imu_arc_hold(m, tracker, pose):
    """使用已同步到控制周期的 IMU，接续时长沿用融合的主固定解中断上限。"""
    cfg=getattr(getattr(m,'fusion',None),'config',None)
    maximum=getattr(cfg,'rtk_hold_s',5.)
    age=getattr(pose,'hold_age_s',math.inf)
    reasons=getattr(pose,'reasons',())
    fresh=getattr(tracker,'_fresh_straight_imu',lambda:None)()
    return (fresh is not None and math.isfinite(age) and 0<=age<=maximum+1e-9
            and not any(r in reasons for r in ('imu_stale','imu_mismatch','imu_hold_timeout','rtk_fixed_timeout')))


def imu_straight(tracker):
    """直线 IMU 为主、内环已接管时的喷头横偏估计（卫星跳了也按 IMU 推算）；不是这种直线时 None。"""
    estimate=getattr(tracker,'straight_guide_estimate',None)
    value=estimate() if callable(estimate) else None
    return value if type(value) in (int,float) and math.isfinite(value) else None


def status(m,record,phase,section,tracker):
    limit=offset_limit()
    out=dict(paint_phase=bool(phase),painting=False,paint_blocked=None,
             nozzle_deviation_m=None,paint_offset_limit_m=limit)
    if not phase:out['paint_blocked']='not_painting';return out
    if not getattr(tracker,'paint_start_allowed',lambda:True)():
        out['paint_blocked']='paint_start_unready';return out
    if getattr(tracker,'constant_paint_speed',False) and not tracker.paint_speed_ready:
        # 尚未达到当前阶段的计划速度，或保护正在减速：停喷。正式线头后经稳定确认的平滑提速可以连续喷涂。
        out['paint_blocked']='paint_speed_not_ready';return out
    pose=m.pose
    arc=continuous_arc(m,section,tracker)
    # 2026-10-06 用户：直线 IMU 为主，「只要和 IMU 推算对不上，直线上也听 IMU 的」：卫星跳了、暂时不是固定解时
    # 按 IMU 推算的喷头横偏判，照常喷；卫星断的时长受融合 rtk_hold_s 限制（同圆弧），Gen5 满 8 s 先停喷
    guided=None if arc else imu_straight(tracker)
    if limit is None and not arc:out['paint_blocked']='paint_configuration_invalid';return out
    if pose is None or not pose.usable_for_control:
        out['paint_blocked']='paint_position_untrusted';return out
    if not getattr(pose,'rtk_trusted',False) and getattr(pose,'hold_age_s',0.)>=RTK_LOST_PAINT_STOP_S-1e-9:
        # 固定解断满 8 s：先停喷，车再走 1 s 才停（见 RTK_LOST_PAINT_STOP_S）
        out['paint_blocked']='paint_position_untrusted';return out
    holding=(arc or guided is not None) and not getattr(pose,'rtk_trusted',False)
    if holding and not imu_arc_hold(m,tracker,pose):
        out['paint_blocked']='paint_position_untrusted';return out
    if not arc and guided is None and pose.quality!='RTK_FIXED':
        out['paint_blocked']='paint_position_untrusted';return out
    if not arc and guided is None and any(r in pose.reasons for r in ('rtk_hold_pending','rtk_hold','rtk_suspect','gnss_displacement_conflict','position_reset','paint_budget_exceeded','transit_budget_slow')):
        out['paint_blocked']='paint_position_unconfirmed';return out
    if not all(math.isfinite(v) for v in (pose.x,pose.y,pose.yaw,pose.speed)):
        out['paint_blocked']='paint_position_invalid';return out
    if not moving(pose):out['paint_blocked']='paint_stationary';return out
    f,l=nozzle(m,record)
    if not all(type(v) in (int,float) and math.isfinite(v) for v in (f,l)):
        out['paint_blocked']='paint_nozzle_invalid';return out
    if guided is not None:
        out['nozzle_deviation_m']=abs(guided)
        out['paint_control_source']='imu_line_hold' if holding else 'imu_line'
        if abs(guided)>limit:out['paint_blocked']='paint_off_line';return out
        out['painting']=True
        return out
    point=rp.nozzle_at(pose.x,pose.y,pose.yaw,f,l)
    try:
        spec=m._route_local() if not record.get('field_plan') else None
        if spec is not None and spec.kind=='circle':
            deviation=abs(math.dist(point,spec.center)-spec.radius)
        else:
            # 当前实际执行的刷线段：按车身参考路径变换到喷头轨迹，禁止错配另一条交叉线。
            from path_tracker import paint_window
            _,start,end=paint_window(section,tracker.st.s,tail_extra=.05 if getattr(m,'differential',False) and section.paint_to_m is not None else 0.)
            hint=min(end,max(start,tracker.st.s))
            reach=math.hypot(f,l)+1.0
            lo,hi=max(start,hint-reach),min(end,hint+reach)
            n=max(1,math.ceil((hi-lo)/.02))
            points=[rp.nozzle_at(*section.car_pose(lo+(hi-lo)*i/n),f,l) for i in range(n+1)]
            deviation=min(segment_distance(point,a,b) for a,b in zip(points,points[1:]))
    except (ValueError,TypeError,AttributeError):
        out['paint_blocked']='paint_reference_unavailable';return out
    if not math.isfinite(deviation):out['paint_blocked']='paint_reference_invalid';return out
    out['nozzle_deviation_m']=deviation
    if arc:out['paint_control_source']='imu_hold' if holding else 'rtk_imu'
    if not arc and deviation>limit:out['paint_blocked']='paint_off_line';return out
    out['painting']=True
    return out
