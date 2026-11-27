#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RTK 完好性监测（gnss_integrity.RtkIntegrity）离线回放：作业记录里的 /gnss/usb_bytes（接收机原始字节，平板 USB 线
接着时才有）和 /gnss/raw（RTK 定位、测向）按录的先后喂进去，和车上 rtk_node 一样，看哪里判 RTK 跳了。

    python3 rtk_integrity_replay.py <作业目录/segNN | xxx.mcap> [...] [--events] [--eph 另一个作业目录 ...]

星历接收机有变化才发，一段作业里不一定齐：--eph 给同一时段的别的作业，先把它们的星历喂进去（只取星历）。
读 bag 要装：pip install mcap mcap-ros2-support（分析电脑上装，车上不用）。
"""
import argparse
import glob
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

import gnss_integrity as gi  # noqa: E402
from um982 import RTK_FIXED  # noqa: E402


def _mcap(path):
    if path.endswith('.mcap'):
        return path
    files = sorted(glob.glob(os.path.join(path, '*.mcap')))
    if not files:
        raise SystemExit('没有 mcap：' + path)
    return files[0]


def messages(path, topics):
    from mcap_ros2.reader import read_ros2_messages
    for m in read_ros2_messages(_mcap(path), topics=topics):
        yield m.log_time_ns, m.channel.topic, m.ros_msg


def preload_eph(mon, paths):
    for p in paths:
        for _, _, msg in messages(p, ['/gnss/usb_bytes']):
            for mid, week, tow, body in mon.split.feed(bytes(msg.data)):
                if mid in (gi.ID_GPSEPH, gi.ID_BDSEPH, gi.ID_GALEPH):
                    got = gi.decode_eph(mid, body)
                    if got is not None and got[1].get('health', 0) == 0:
                        mon.eph[got[0]] = got[1]
    mon.split = gi.FrameSplitter()


def replay(path, mon=None, eph_paths=()):
    """返回 (监测器, 每次检查后的 [(tow, 状态, 水平差 m, 星数, 原因)], [(tow, 喷线里走了多少 m 或 None, 在不在喷)])。"""
    mon = mon or gi.RtkIntegrity()
    if eph_paths:
        preload_eph(mon, eph_paths)
    timeline, trace = [], []
    checks = mon.checks
    tow_to_t = None
    paint0 = None
    for _, topic, msg in messages(path, ['/gnss/usb_bytes', '/gnss/raw', '/line/trace']):
        if topic == '/gnss/usb_bytes':
            mon.feed_bytes(bytes(msg.data))
        elif topic == '/gnss/raw':
            m = json.loads(msg.data)
            if m.get('kind') == 'fix' and m.get('gps_tow_s') is not None:
                mon.add_fix(m['gps_tow_s'], m['lat'], m['lon'], m['height'], m.get('sol') in RTK_FIXED)
                if tow_to_t is None:
                    tow_to_t = m['t'] - m['gps_tow_s']
            elif m.get('kind') == 'heading':
                mon.add_heading(m.get('gps_tow_s'), m.get('heading'), m.get('valid') is True)
        else:
            m = json.loads(msg.data)
            painting = bool((m.get('paint') or {}).get('painting')) and m.get('vx', 0) > 0.1
            if painting and paint0 is None:
                paint0 = m['tracker']['s']
            if tow_to_t is not None:
                trace.append((m['t'] - tow_to_t, None if paint0 is None else m['tracker']['s'] - paint0, painting))
        if mon.checks != checks:
            checks = mon.checks
            st = mon.status()
            timeline.append((mon.last_tow, st['state'], st['delta_m'], st['sats'], st['reason']))
    return mon, timeline, trace


def summarize(name, timeline, trace, events=False):
    import bisect
    T = [t for t, _, _ in trace]

    def at(tow):
        i = bisect.bisect_left(T, tow)
        return trace[min(i, len(trace) - 1)] if trace else (tow, None, False)
    paint = [(tow, s, d, n, r) for tow, s, d, n, r in timeline if at(tow)[2]]
    count = lambda rows, k: sum(1 for r in rows if r[1] == k)  # noqa: E731
    print(f"== {name}: 检查 {len(timeline)} 次（喷线中 {len(paint)}）| 喷线中 ok {count(paint, 'ok')} / jump {count(paint, 'jump')} "
          f"/ unknown {count(paint, 'unknown')}")
    prev = None
    for tow, s, d, n, r in timeline:
        if s != prev and (events or s == 'jump' or prev == 'jump'):
            u = at(tow)[1]
            print(f"   tow {tow:.1f}  {prev} → {s}  ({r})  水平差 {'' if d is None else '%.1f cm' % (100 * d)}  星 {n}  "
                  f"{'喷线 %.1f m' % u if u is not None and at(tow)[2] else ('喷线前' if u is None else '喷线外')}")
        prev = s


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('paths', nargs='+')
    ap.add_argument('--eph', nargs='*', default=[], help='先从这些作业取星历')
    ap.add_argument('--events', action='store_true', help='打印所有状态变化（默认只打印 jump 前后）')
    args = ap.parse_args()
    for p in args.paths:
        mon, timeline, trace = replay(p, eph_paths=args.eph)
        summarize(os.path.basename(os.path.dirname(p.rstrip('/'))) + '/' + os.path.basename(p.rstrip('/')), timeline, trace, args.events)


if __name__ == '__main__':
    main()
