# -*- coding: utf-8 -*-
"""底盘运动服务（/motion/*、/wheeltec/*）回给 App 和网页的提示文字：一律中文。

状态里的 mode / reason / health_faults 仍是代码（程序按它判断），只在拼成给人看的回复时换成这里的中文；
不认识的代码不原样显示英文，用通用说明。纯文字表，不依赖 ROS。
"""

MODE_TEXT = {
    'IDLE': '待命', 'MANUAL': '手动', 'NAVIGATION': '自动作业', 'FOLLOW': '跟随',
    'FAULT': '故障锁存', 'ESTOP': '急停锁存',
}

SOURCE_TEXT = {'manual': '手动', 'navigation': '自动作业', 'follow': '跟随'}

REASON_TEXT = {
    'startup': '底盘启动中',
    'command_timeout': '指令超时，已退出运动模式',
    'sensor_or_driver_fault': '传感器或底盘故障，已锁定停车',
    'operator_stop': '已停车',
    'operator_select': '已选择运动模式',
    'manual_takeover': '手动接管',
    'reset': '已复位，等待重新选择任务',
    'reset_failed': '复位时底盘没就绪，继续锁定停车',
    'zero_hold_oscillation': '停车锁轮时轮子一直来回动，已松开驱动器、中断作业',
}

# ControlPolicy.ready / arm 的原因
READY_TEXT = {
    'ready': '底盘就绪',
    'armed_waiting_command': '已解锁，等待指令',
    'receive_only': '底盘只读模式（没有核对过，不发指令）',
    'firmware_profile_unconfirmed': '底盘协议没有确认',
    'disconnected': '底盘连接断开',
    'startup_stop': '底盘刚启动，正在等待静止',
    'feedback_stale': '底盘反馈超时',
    'feedback_missing': '还没收到底盘反馈',
    'waiting_for_stationary_feedback': '等待车辆停稳',
}

# refresh_motion_health 的 health_faults
FAULT_TEXT = {
    'driver_disconnected': '底盘连接断开',
    'feedback_missing': '还没收到底盘反馈',
    'feedback_stale': '底盘反馈超时',
    'scan_unavailable': '没有雷达数据',
    'scan_stale': '雷达数据超时',
    'battery_invalid': '电池电压读不到',
    'battery_low': '电池电量低',
    'driver_receive_only': '底盘只读模式（没有核对过，不发指令）',
    'protocol_unconfirmed': '底盘协议没有确认',
    'startup_stop': '底盘刚启动，正在等待静止',
}

# 带冒号的故障：前缀 → 中文；冒号后面是轮子（左轮）或驱动器故障的中文说明，driver_hold 后面是 READY_TEXT 的原因
FAULT_PREFIX_TEXT = {
    'driver_silent': '驱动器没有回应',
    'driver_fault': '驱动器故障',
    'driver_hold': '底盘暂停',
}

UNKNOWN_TEXT = '底盘状态异常'


def _has_cjk(text):
    return any('\u4e00' <= ch <= '\u9fff' for ch in text)


def mode_text(mode):
    return MODE_TEXT.get(mode, UNKNOWN_TEXT)


def reason_text(reason):
    return REASON_TEXT.get(reason) or READY_TEXT.get(reason) or FAULT_TEXT.get(reason) or UNKNOWN_TEXT


def ready_text(reason):
    return READY_TEXT.get(reason) or FAULT_TEXT.get(reason) or REASON_TEXT.get(reason) or UNKNOWN_TEXT


def fault_text(code):
    """一条 health_faults 的中文：driver_silent:右轮 → 驱动器没有回应（右轮）。"""
    code = str(code or '')
    if code in FAULT_TEXT:
        return FAULT_TEXT[code]
    prefix, sep, detail = code.partition(':')
    if sep and prefix in FAULT_PREFIX_TEXT:
        if prefix == 'driver_hold':
            detail = ready_text(detail) if detail in READY_TEXT or detail in FAULT_TEXT else ''
        elif not _has_cjk(detail):
            detail = ''         # 轮子名、驱动器故障说明都是中文（可带 MOS、FLASH 这类器件名）；不是中文的不显示
        return FAULT_PREFIX_TEXT[prefix] + ('（%s）' % detail if detail else '')
    return UNKNOWN_TEXT


def faults_text(codes):
    """多条故障去重后用顿号连起来；没有时返回「待确认」。"""
    out = []
    for code in codes or ():
        text = fault_text(code)
        if text not in out:
            out.append(text)
    return '、'.join(out) if out else '待确认'


def status_text(mode, reason):
    """select_motion 回复：「待命：已复位，等待重新选择任务」。"""
    return '%s：%s' % (mode_text(mode), reason_text(reason))
