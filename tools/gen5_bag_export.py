#!/usr/bin/env python3
"""只读录包，导出离线融合回放所需的数据；不初始化节点、不发布话题、不打开硬件。"""
import argparse
import gzip
import json
import math
from pathlib import Path
import statistics


PREFIXES = ('20260930_185010_', '20260930_185104_', '20260930_192512_', '20260930_192712_',
            '20260930_192909_', '20260930_193159_', '20260930_193427_', '20260930_193711_')


def export_segment(job, segment):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    types = {'/gnss/raw': String, '/imu/data': Imu, '/odom': Odometry, '/line/status': String}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(segment), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    messages, offsets = [], []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in types:
            continue
        msg = deserialize_message(data, types[topic])
        received = stamp / 1e9
        if topic == '/gnss/raw':
            value = json.loads(msg.data)
            if value.get('t_rx') is not None:
                offsets.append(received - value['t_rx'])
            messages.append([received, 'gnss', value])
        elif topic == '/line/status':
            value = json.loads(msg.data)
            messages.append([received, 'status', {k: value.get(k) for k in
                             ('phase', 'tracker', 'pose', 'calibration', 'message')}])
        else:
            head = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            measured = head if 0 <= received - head <= 0.5 else received
            if topic == '/odom':
                value = [measured, msg.twist.twist.linear.x, msg.twist.twist.angular.z]
                messages.append([received, 'wheel', value])
            else:
                q = msg.orientation
                n = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
                if n < 1e-6 or not math.isfinite(n):
                    continue
                x, y, z, w = q.x/n, q.y/n, q.z/n, q.w/n
                roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
                pitch = math.asin(max(-1., min(1., 2*(w*y-z*x))))
                yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
                messages.append([received, 'imu', [measured, msg.angular_velocity.z, yaw, roll, pitch]])
    if not offsets:
        raise ValueError('录包没有用于时间对齐的 GNSS 接收时刻：' + str(segment))
    offset = statistics.median(offsets)
    for item in messages:
        item[0] -= offset
        if item[1] in ('imu', 'wheel'):
            item[2][0] -= offset
    context = job/'context'/segment.name
    return {'job': job.name, 'segment': segment.name, 'offset_s': offset,
            'fusion_config': json.loads((context/'fusion.json').read_text()),
            'release': (context/'release.txt').read_text().strip(), 'events': messages}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jobs', type=Path, default=Path('/root/rtk/jobs'))
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    result = []
    for prefix in PREFIXES:
        jobs = sorted(args.jobs.glob(prefix+'*'))
        if len(jobs) != 1:
            raise ValueError('作业匹配不唯一：' + prefix)
        for segment in sorted(jobs[0].glob('seg[0-9][0-9]')):
            result.append(export_segment(jobs[0], segment))
            print('已读取', jobs[0].name, segment.name, len(result[-1]['events']), '条', flush=True)
    with gzip.open(args.out, 'wt', encoding='utf-8') as fh:
        json.dump(result, fh, ensure_ascii=False, separators=(',', ':'))


if __name__ == '__main__':
    main()
