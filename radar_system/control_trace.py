#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""控制记录 /line/trace：作业时每个控制周期（50 Hz）一条，作业记录（job_recorder）原样存进 bag，事后分析用。

/line/status 只有 5 Hz、而且是给 app 看的；分析「为什么走不直」要的是每一拍：控制用的是哪个位置（融合位姿、
不确定度、推算维持了多久），跟踪器怎么看（横偏 e、航向差、曲率、积分），最后下发了什么（vx、wz）。
和 /gnss/raw、/imu、/odom、/oid/status、/navigation/command 放在一起，就能一拍一拍对上「看到的 → 算的 → 发的 → 车做的」。

只读 mission 上已经算好的东西（tick 里刚设的 mission.pose、跟踪器 status()），不改任何状态；出错只丢这一条，不影响开车。
字段（单位：米、弧度、秒、米/秒；t 是本机单调时钟，与 /gnss/raw 的 t / t_rx 同一时钟）：
    t, phase, mode（底盘控制权）, vx, wz（这一拍下发的），select（申请 / 放弃控制权时才有）
    pose:    t（位姿时刻）, x, y（米，作业原点 ENU）, yaw（弧度，东 0 逆时针）, speed, yaw_rate, sigma_xy, sigma_yaw,
             gyro_bias, quality, usable（能不能用来控制）, hold_age_s / hold_distance_m / hold_turn_rad（RTK 断了靠推算的时长 / 路程 / 转角）,
             reasons（不能用的原因）
    tracker: 跟踪器 status() 里的标量（e 横偏、heading_err_deg、s、kappa_ref、kappa、v_cmd、integral、trim_kappa、
             section、gear、pivoting……，随跟踪器种类不同）；tracker_type 是跟踪器类名
"""
import math

TRACE_PHASES = ('SELECTING', 'RUNNING', 'PAUSING')
POSE_FIELDS = ('t', 'x', 'y', 'yaw', 'speed', 'yaw_rate', 'sigma_xy', 'sigma_yaw', 'gyro_bias', 'quality',
               'hold_age_s', 'hold_distance_m', 'hold_turn_rad', 'rtk_trusted')
MAX_TRACKER_KEYS = 40


def _value(v, digits=5):
    """JSON 里放得下的标量：浮点保留 digits 位，非有限数为 None，其它类型（dict、list……）返回 _SKIP。"""
    if isinstance(v, bool) or v is None or isinstance(v, str):
        return v
    if isinstance(v, int):
        return v
    if isinstance(v, float):
        return round(v, digits) if math.isfinite(v) else None
    return _SKIP


_SKIP = object()


def record(mission, now, mode, vx, wz, select=None):
    """这一拍的控制记录（dict）；任务没在跑（IDLE、PAUSED 等）返回 None，不发。

    >>> class P: t, x, y, yaw, speed, quality, usable_for_control, reasons = 9.98, 1.234567891, -0.5, 0.1, 0.4, 'RTK_FIXED', True, ()
    >>> class T:
    ...     def status(self): return dict(state='RUNNING', e=0.0123456789, stats=dict(rms_m=0.01), section=2)
    >>> class M: phase, pose, tracker = 'RUNNING', P(), T()
    >>> r = record(M(), 10.0, 'NAVIGATION', 0.4, 0.05)
    >>> r['pose']['x'], r['pose']['usable'], r['tracker'], r['tracker_type']
    (1.23457, True, {'state': 'RUNNING', 'e': 0.01235, 'section': 2}, 'T')
    >>> M.phase = 'IDLE'; record(M(), 10.0, 'IDLE', 0.0, 0.0) is None
    True
    """
    phase = getattr(mission, 'phase', None)
    if phase not in TRACE_PHASES:
        return None
    out = {'t': round(now, 4), 'phase': phase, 'mode': mode, 'vx': _value(float(vx), 4), 'wz': _value(float(wz), 4)}
    if select is not None:
        out['select'] = select
    pose = getattr(mission, 'pose', None)
    if pose is not None:
        p = {}
        for k in POSE_FIELDS:
            v = _value(getattr(pose, k, None))
            if v is not _SKIP and v is not None:
                p[k] = v
        p['usable'] = bool(getattr(pose, 'usable_for_control', False))
        if getattr(pose, 'motion_evidence', False):
            p.update(motion_evidence=True, hall_still=getattr(pose, 'hall_still', None),
                     gnss_still=getattr(pose, 'gnss_still', None))
        reasons = getattr(pose, 'reasons', ())
        if reasons:
            p['reasons'] = [str(r) for r in list(reasons)[:5]]
        out['pose'] = p
    out['task']=getattr(mission,'task',None)
    if out['task']=='route' and getattr(mission,'operations',None) is not None:
        out['paint']=mission.operations.paint_status()
    out['speed_limit_mps'] = _value(getattr(getattr(mission, 'tracker', None), 'speed_limit_mps', None))
    tracker = getattr(mission, 'tracker', None)
    status = getattr(tracker, 'status', None)
    if callable(status):
        st = status()
        if isinstance(st, dict):
            out['tracker'] = {k: v for k, v in ((k, _value(v)) for k, v in list(st.items())[:MAX_TRACKER_KEYS])
                              if v is not _SKIP}
            for name in ('straight_hold', 'arc_hold'):
                hold = st.get(name)
                if isinstance(hold, dict):
                    out[name] = {k: v for k, v in ((k, _value(v)) for k, v in list(hold.items())[:96])
                                 if v is not _SKIP}
        out['tracker_type'] = type(tracker).__name__
    return out
