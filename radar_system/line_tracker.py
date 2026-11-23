#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""定位质量的共用小判断（路线跟踪 path_tracker、直线内外环 line_cascade 用）。纯计算，不依赖 ROS。

2026-11-23 删了阿克曼车：原来这里的 RTK 走直线控制器 LineTracker / LineConfig（阿克曼车 A→B 定点画线、
开机空闲占位）一起删了，空闲占位见 line_mission.IdleTracker。
"""
import math


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


_POSITION_INTEGRAL_BLOCKERS = frozenset({
    'rtk_hold', 'rtk_hold_timeout', 'rtk_suspect', 'rtk_rejoin',
    'rtk_weak', 'rtk_weighted', 'rtk_float', 'rtk_fixed_timeout', 'position_reset',
    'position_outlier', 'position_uncertain', 'position_uninitialized', 'fix_unavailable',
})
# 融合平滑输出还没追上的位置校正超过这么多才冻结积分。原来 1 mm：2026-10-02 实车 5 趟巡航时校正量 p95 约 1.1 mm、
# 最大 3 mm（正常的平滑跟随），积分 42–72% 的时间被冻住，测向偏差引起的 0.2–0.8 cm 平行偏移拉不回来。
# 5 mm 仍挡住 RTK 接回、重置这类厘米级的校正（它们另有 rtk_rejoin / position_reset 等原因）。
ALIGN_PENDING_MAX_M = 0.005


def position_observation_allowed(pose):
    """定位观测可用；融合的平滑校正余量单独约束积分，不作为观测失效。"""
    return (pose.usable_for_control
            and not _POSITION_INTEGRAL_BLOCKERS.intersection(getattr(pose, 'reasons', ()) or ())
            and getattr(pose, 'quality', None) not in ('INVALID', 'DEAD_RECKONING', 'RTK_FLOAT'))


def lateral_integral_allowed(pose):
    """位置可信、绝对位置校正已接回时才学习横偏积分；测向质量单独管理。

    轻量位姿没有质量字段时保留原控制器行为。航向不确定或测向与 IMU 不一致，仍可用可信
    RTK 横偏消除慢航向偏差；位置靠推算、降权或正在接回时保留原积分，不再积入推算漂移。
    """
    if not position_observation_allowed(pose):
        return False
    try:
        dx, dy = getattr(pose, 'correction_xy', (0.0, 0.0))
        pending = math.hypot(dx, dy)
    except (TypeError, ValueError, OverflowError):
        return False
    return math.isfinite(pending) and pending <= ALIGN_PENDING_MAX_M
