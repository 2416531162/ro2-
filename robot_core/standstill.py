"""任务层共享的停稳判断；Gen5 必须同时具备新鲜霍尔静止和卫星地速证据（单天线没有卫星地速：只看霍尔，
融合位姿的 gnss_still_required 为 False，2026-10-07 用户「只要轮子不动就行」）。"""
import math

HALL_STILL_MAX_MPS = .06


def stationary(pose, speed_limit=.02, yaw_limit=.05):
    if pose is None or not all(math.isfinite(v) for v in (pose.speed,pose.yaw_rate)):
        return False
    if 'wheel_stale' in getattr(pose,'reasons',()):
        return False
    if getattr(pose,'motion_evidence',False):
        return (pose.hall_still is True
                and (pose.gnss_still is True or getattr(pose, 'gnss_still_required', True) is False)
                and abs(pose.yaw_rate) <= yaw_limit)
    # 旧阿克曼与纯运动学仿真保留原速度接口；Gen5 的新轮程明确标记证据能力。
    return abs(pose.speed) <= speed_limit and abs(pose.yaw_rate) <= yaw_limit


def moving(pose, threshold=.02):
    """喷线起步资格：任一路确认还在动即可，证据缺失时不能凭一个零速判停。"""
    if getattr(pose,'motion_evidence',False):
        return pose.hall_still is False or pose.gnss_still is False
    return abs(pose.speed) >= threshold
