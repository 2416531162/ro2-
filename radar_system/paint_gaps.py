"""普通路线的停喷区间记账；依据喷线许可，不把行驶完成当作实际漆线完整。"""
import math


def validate(record):
    data=record.get('paint_gaps')
    if data is None:return
    if not isinstance(data,dict) or not isinstance(data.get('ranges'),list) or len(data['ranges'])>1000:
        raise ValueError('停喷区间记录无效')
    end=total=0.
    for pair in data['ranges']:
        if (not isinstance(pair,list) or len(pair)!=2
                or not all(type(v) in (int,float) and math.isfinite(v) for v in pair)
                or not 0<=end<=pair[0]<pair[1]<=record['total_m']+1e-6):
            raise ValueError('停喷区间超出任务范围')
        total+=pair[1]-pair[0];end=pair[1]
    if (type(data.get('total_m')) not in (int,float) or not math.isfinite(data['total_m'])
            or abs(data['total_m']-total)>1e-6 or type(data.get('overflow')) is not bool):
        raise ValueError('停喷长度记录无效')


def observe(record, cursor, phase, allowed, previous=None):
    """只记录已观察到的应喷行程；previous 只在当前运行进程中保留。

    一次控制周期内状态改变的区间保守记为待检查。正常段尾关喷不产生欠喷，
    起步/换段/恢复时不跨未知区间补造观测。返回下一拍的运行时锚点。
    """
    if (not isinstance(record.get('id'),str) or not record['id']
            or not math.isfinite(cursor)):
        return None
    cursor=max(0.,min(record['total_m'],cursor))
    current=(record['id'],cursor,bool(phase),bool(allowed))
    if previous is None or previous[0]!=record['id'] or cursor<previous[1]:
        return current
    _,start,was_phase,was_allowed=previous
    # 上一拍在喷线区且该区间至少一端不允许喷线。正常离开喷线区时，
    # 只按上一拍的许可结清；当前的正常关喷不能把整段尾部记成漏喷。
    missing=was_phase and (not was_allowed or (phase and not allowed))
    if missing and cursor>start+1e-7:
        data=record.setdefault('paint_gaps',dict(ranges=[],total_m=0.,estimated=True,overflow=False))
        ranges=data['ranges']
        if ranges and start<=ranges[-1][1]+1e-6:
            ranges[-1][1]=max(ranges[-1][1],cursor)
        elif len(ranges)<1000:
            ranges.append([start,cursor])
        else:
            # 保留已有区间并明确记录容量不足，不能静默丢掉欠喷状态。
            data['overflow']=True
        data['total_m']=sum(b-a for a,b in ranges)
    return current


def has_gaps(record):
    data=(record or {}).get('paint_gaps') or {}
    return data.get('total_m',0.)>1e-7 or bool(data.get('overflow'))


def completion_message(record):
    if not has_gaps(record):return None
    return '行驶结束，有约 %.2f 米停喷区段，请检查并选段补画' % record['paint_gaps']['total_m']
