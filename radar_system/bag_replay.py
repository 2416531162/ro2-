#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线回放:把现场录下的 rosbag 喂给跟随节点,复现问题、对比参数。

为什么需要
----------
现场问题(「时有时无」「出画就停」「前方无路」)靠截图和口述很难复现,
改了参数也不知道是不是真的变好了。专业项目的做法是录包 -> 离线回放 -> 回归。

录包(板子上,跟随程序照常运行):
    bash radar_system/record_follow_bag.sh            # Ctrl+C 结束

回放(不需要底盘、相机、雷达在线;节点以演练模式运行,不发任何指令):
    python3 radar_system/bag_replay.py ~/bags/follow_20260917_1530
    python3 radar_system/bag_replay.py BAG --set follow_breadcrumbs=False   # A/B 对比
    python3 radar_system/bag_replay.py BAG --out timeline.jsonl              # 逐周期状态

回放时的时钟是包里的时间:相机/雷达时间戳、控制周期、超时判断都和现场一致,
但不受板子当时 CPU 负载影响(想复现「控制周期超时」要看录下来的 /follower/status)。
"""

import argparse
import json
import sys
from collections import Counter
from runtime_config import PROFILE

TOPICS = {
    '/scan': 'on_scan',
    '/camera/ai_detection/targets': 'on_targets',
    '/wheeltec/status': 'on_driver_status',
    '/voltage': 'on_voltage',
    PROFILE['localization']['topic']: 'on_odom',
}


class ReplayClock:
    """替换 person_follower 模块里的 time,以及节点的 ROS 时钟。"""

    def __init__(self, t0):
        self.t = float(t0)

    def monotonic(self):
        return self.t

    def time(self):
        return self.t

    def sleep(self, dt):
        self.t += max(0.0, dt)

    # rclpy Clock 接口的最小子集
    def now(self):
        clock = self

        class _Now:
            nanoseconds = int(clock.t * 1e9)
        return _Now()


def replay(node, messages, clock, control_hz=20.0, on_status=None):
    """messages: 按时间排序的 (t 秒, topic, msg)。返回状态时间线摘要。

    节点必须以 dry_run=True 创建。调用前应已把 person_follower.time 换成 clock。
    """
    captured = []
    original_publish = node.pub_status.publish

    def capture(msg):
        captured.append(msg.data)
        return None
    node.pub_status.publish = capture
    node.print_dashboard = lambda payload: None
    node.get_clock = lambda: clock

    period = 1.0 / control_hz
    next_tick = None
    timeline = []
    counts = Counter()
    last_state = None
    last_target = None
    lost_events = 0
    max_compute = 0.0

    def tick():
        nonlocal last_state, last_target, lost_events, max_compute
        node.control_loop()
        if not captured:
            return
        payload = json.loads(captured[-1])
        captured.clear()
        state = payload.get('state')
        counts[state] += 1
        target = payload.get('target')
        if last_target is not None and target is None:
            lost_events += 1
        last_target = target
        diag = payload.get('diag') or {}
        if isinstance(diag.get('loop_compute_ms'), (int, float)):
            max_compute = max(max_compute, diag['loop_compute_ms'])
        if state != last_state:
            timeline.append((round(clock.t, 2), state, payload.get('limit_reason')))
            last_state = state
        if on_status is not None:
            on_status(clock.t, payload)

    for t, topic, msg in messages:
        if next_tick is None:
            next_tick = t
        while next_tick <= t:
            clock.t = next_tick
            tick()
            next_tick += period
        clock.t = t
        handler = TOPICS.get(topic)
        if handler is not None:
            getattr(node, handler)(msg)
    node.pub_status.publish = original_publish

    total = sum(counts.values()) or 1
    return {
        'cycles': total,
        'duration_s': round(total * period, 1),
        'state_share': {k: round(v / total, 3) for k, v in counts.most_common()},
        'lost_events': lost_events,
        'target_switches': node.people.switches,
        'dropped_not_person': node.people.dropped_unseen,
        'stamp_warnings': node.stamp_warnings,
        'max_loop_compute_ms': max_compute,
        'transitions': timeline,
    }


def read_bag(path, topics=TOPICS):
    """rosbag2 -> (t 秒, topic, msg)。时间用录制时的接收时间。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in topics if t not in types]
    if missing:
        print(f"注意:包里没有 {missing}", file=sys.stderr)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    classes = {t: get_message(types[t]) for t in topics if t in types}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        yield t_ns / 1e9, topic, deserialize_message(data, classes[topic])


def parse_value(text):
    for cast in (int, float):
        try:
            return cast(text)
        except ValueError:
            pass
    if text in ('True', 'true'):
        return True
    if text in ('False', 'false'):
        return False
    return text


def main():
    p = argparse.ArgumentParser(description="跟随节点离线回放")
    p.add_argument('bag')
    p.add_argument('--set', action='append', default=[], metavar='KEY=VALUE',
                   help='覆盖 FollowerConfig 参数,可多次使用')
    p.add_argument('--out', help='逐周期状态写成 JSON Lines')
    p.add_argument('--target', default='person')
    p.add_argument('--legacy-velocity-odometry', action='store_true', help='显式为旧包启用速度积分，不用于实车')
    args = p.parse_args()

    import rclpy
    import person_follower as pf

    cfg = pf.FollowerConfig()
    for item in args.set:
        key, _, value = item.partition('=')
        if not hasattr(cfg, key):
            p.error(f"FollowerConfig 没有参数 {key}")
        setattr(cfg, key, parse_value(value))
    cfg.__post_init__()

    messages = read_bag(args.bag)
    first = next(messages, None)
    if first is None:
        print("包里没有可回放的消息")
        return 1

    def chained():
        yield first
        yield from messages

    clock = ReplayClock(first[0])
    real_time = pf.time
    pf.time = clock
    rclpy.init()
    out = open(args.out, 'w', encoding='utf-8') if args.out else None
    try:
        node = pf.PersonFollowerNode(cfg, dry_run=True, target_class=args.target,
                                     simulated_odometry=args.legacy_velocity_odometry)
        node.timer.cancel()
        writer = (lambda t, payload: out.write(json.dumps(payload, ensure_ascii=False) + '\n')) \
            if out else None
        summary = replay(node, chained(), clock, cfg.control_hz, writer)
        node.destroy_node()
    finally:
        pf.time = real_time
        if out:
            out.close()
        if rclpy.ok():
            rclpy.shutdown()

    print(json.dumps({k: v for k, v in summary.items() if k != 'transitions'},
                     ensure_ascii=False, indent=1))
    print("\n状态切换(时间, 状态, 原因):")
    for t, state, reason in summary['transitions'][:200]:
        print(f"  {t:12.2f}  {state:<20} {reason}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
