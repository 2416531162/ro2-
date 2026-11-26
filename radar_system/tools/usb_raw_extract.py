#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把作业记录里平板 USB 线的原始字节（/gnss/usb_bytes）拆成事后精算用的两个文件。

    python3 usb_raw_extract.py <作业目录/segNN> [输出目录]

写出：
    um982.bin    接收机 UM982 COM2 的原始字节流（原始观测 OBSVMB / OBSVHB、星历、车用日志、NMEA），已去掉平板插进来的
                 `$RTCM3,...` 行；RTKLIB demo5 用 `-r unicore` 读
    base.rtcm3   平板连 CORS 收到的 RTCM3 改正数（虚拟基站观测和坐标），二进制，CRC24Q 校验过
    summary.json 字节数、消息条数、RTCM3 帧数和消息号统计、时间范围

读 bag 要装：pip install mcap mcap-ros2-support（分析电脑上装，车上不用）。
"""
import json
import os
import re
import sys

RTCM_LINE = re.compile(rb'\$RTCM3,([^\r\n]*)\r?\n')


def crc24q(data):
    crc = 0
    for b in data:
        crc ^= b << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def rtcm_frame_ok(frame):
    """一帧 RTCM3（D3 + 10 位长度 + 数据 + 3 字节 CRC24Q）是否完整、校验对。"""
    if len(frame) < 6 or frame[0] != 0xD3:
        return False
    length = ((frame[1] & 0x03) << 8) | frame[2]
    if len(frame) != 3 + length + 3:
        return False
    return crc24q(frame[:-3]) == int.from_bytes(frame[-3:], 'big')


def rtcm_type(frame):
    return (frame[3] << 4) | (frame[4] >> 4) if len(frame) > 4 else None


def split_stream(stream):
    """原始字节流 → (接收机字节, [RTCM3 帧], 坏行数)。"""
    frames, bad = [], 0

    def take(m):
        nonlocal bad
        try:
            frame = bytes.fromhex(m.group(1).decode('ascii'))
        except ValueError:
            frame = b''
        if rtcm_frame_ok(frame):
            frames.append(frame)
        else:
            bad += 1
        return b''

    receiver = RTCM_LINE.sub(take, stream)
    return receiver, frames, bad


def read_bag(seg_dir):
    from mcap_ros2.reader import read_ros2_messages
    paths = sorted(os.path.join(seg_dir, n) for n in os.listdir(seg_dir) if n.endswith('.mcap'))
    chunks = []
    for path in paths:
        for m in read_ros2_messages(path, topics=['/gnss/usb_bytes']):
            chunks.append((m.log_time_ns, bytes(m.ros_msg.data)))
    chunks.sort(key=lambda c: c[0])
    return chunks


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    seg = argv[1]
    out = argv[2] if len(argv) > 2 else seg
    os.makedirs(out, exist_ok=True)
    chunks = read_bag(seg)
    stream = b''.join(c for _, c in chunks)
    receiver, frames, bad = split_stream(stream)
    with open(os.path.join(out, 'um982.bin'), 'wb') as fh:
        fh.write(receiver)
    with open(os.path.join(out, 'base.rtcm3'), 'wb') as fh:
        fh.write(b''.join(frames))
    types = {}
    for f in frames:
        types[str(rtcm_type(f))] = types.get(str(rtcm_type(f)), 0) + 1
    summary = dict(messages=len(chunks), stream_bytes=len(stream), receiver_bytes=len(receiver),
                   rtcm_frames=len(frames), rtcm_bad_lines=bad, rtcm_types=types,
                   first_ns=chunks[0][0] if chunks else None, last_ns=chunks[-1][0] if chunks else None)
    with open(os.path.join(out, 'summary.json'), 'w', encoding='utf-8') as fh:
        json.dump(summary, fh, ensure_ascii=False, indent=1)
    print(json.dumps(summary, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
