#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""平板 USB 线原始字节记录（2026-11-26 用户：先加上记录，基站数据由平板转给车）。

- 平板把 CORS 改正数编成 `$RTCM3,<十六进制>` 行，只插在接收机一条消息的末尾：车端解析器只计数，车用日志、GGA 不受影响。
- rtk_node 把 USB 线原始字节攒 0.1 s 发到 /gnss/usb_bytes；作业记录点开始前多留 120 s。
- tools/usb_raw_extract.py 把字节流拆回接收机字节和 RTCM3 二进制。
"""
import os
import struct
import sys
import tempfile
import threading
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, os.path.join(ROOT, 'radar_system', 'tools'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, ROOT)

import ros_stubs  # noqa: E402

ros_stubs.install()
for _name in ('Imu', 'NavSatFix', 'NavSatStatus'):
    setattr(sys.modules['sensor_msgs.msg'], _name, ros_stubs._simple(_name))

import job_recorder as jr  # noqa: E402
import rtk_node  # noqa: E402
import um982  # noqa: E402
import usb_raw_extract as ux  # noqa: E402
from um982 import Um982Stream  # noqa: E402

SAMPLE = os.path.join(ROOT, 'tests', 'fixtures', 'um982_com1_ascii_binary_20261006.bin')


def car_frames():
    raw = open(SAMPLE, 'rb').read()
    out, i = [], 0
    while True:
        i = raw.find(b'\xaa\x44', i)
        if i < 0:
            return out
        kind, end = um982.binary_frame(raw, i)
        if kind == 'frame':
            out.append(raw[i:end])
            i = end
        else:
            i += 1


def n4_frame(msg_id, payload):
    """Unicore N4 原生帧（AA 44 B5，帧头 24 字节）：原始观测、星历都是这种。"""
    head = bytearray(24)
    head[0:3] = b'\xaa\x44\xb5'
    struct.pack_into('<HH', head, 4, msg_id, len(payload))
    body = bytes(head) + payload
    return body + struct.pack('<I', um982.crc32(body))


def rtcm_frame(msg_type, payload_len=19):
    payload = bytearray(payload_len)
    payload[0] = msg_type >> 4
    payload[1] = (msg_type & 0x0F) << 4
    for k in range(2, payload_len):
        payload[k] = (k * 37 + msg_type) & 0xFF
    head = bytes([0xD3, (payload_len >> 8) & 0x03, payload_len & 0xFF])
    frame = head + bytes(payload)
    return frame + ux.crc24q(frame).to_bytes(3, 'big')


def rtcm_line(frame):
    return b'$RTCM3,' + frame.hex().upper().encode() + b'\r\n'


def gga():
    body = 'GNGGA,024019.00,3000.00000741,N,12000.00004073,E,4,32,0.6,12.3456,M,8.5000,M,1.0,2001'
    return ('$%s*%02X\r\n' % (body, um982.nmea_checksum(body))).encode()


class Stream(unittest.TestCase):
    def test_rtcm_lines_and_raw_obs_do_not_disturb_car_logs(self):
        fs = car_frames()[:12]
        raw_obs = n4_frame(12, bytes((k * 7) & 0xFF for k in range(1404)))   # 原始观测帧，数据里有 0x0A
        rt = [rtcm_line(rtcm_frame(t)) for t in (1005, 1074, 1124)]
        data = b''
        for k, f in enumerate(fs):
            data += f
            if k % 3 == 0:
                data += raw_obs
            if k % 4 == 1:
                data += rt[k % 3]                       # 插在接收机一条消息末尾
            if k == 5:
                data += rt[0] + gga()                   # 改正数行后面紧跟 GGA
        s = Um982Stream()
        out = []
        for i in range(0, len(data), 97):               # 随便切块喂
            out += s.feed(data[i:i + 97], 1.0 + i * 1e-5)
        whole = Um982Stream().feed(b''.join(fs), 1.0)
        self.assertEqual([m['kind'] for m in out], [m['kind'] for m in whole])
        self.assertEqual(s.crc_errors, 0)
        self.assertEqual(s.rtcm_lines, 4)
        self.assertIsNotNone(s.gga)


class Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


def bare_node():
    node = rtk_node.RtkNode.__new__(rtk_node.RtkNode)
    node.stream = Um982Stream()
    node.stream_lock = threading.Lock()
    node.source, node.source_rx, node.usb_run_since = None, {}, None
    node.pub_usb_bytes = Pub()
    node.usb_bytes, node.usb_bytes_since, node.usb_bytes_error = bytearray(), None, None
    return node


class RecordUsb(unittest.TestCase):
    def test_bytes_are_batched_and_flushed_in_order(self):
        node = bare_node()
        node.record_usb(b'abc', 0.00)
        node.record_usb(b'def', 0.05)
        self.assertEqual(node.pub_usb_bytes.msgs, [])               # 还没攒够 0.1 s
        node.record_usb(b'ghi', 0.11)
        self.assertEqual([bytes(m.data) for m in node.pub_usb_bytes.msgs], [b'abcdefghi'])
        node.record_usb(b'jk', 0.12)
        node.record_usb(b'', 0.20)                                  # 读超时（线上没数据）：攒着的发掉
        self.assertEqual(bytes(node.pub_usb_bytes.msgs[-1].data), b'jk')
        node.record_usb(b'', 0.30)                                  # 没有攒着的：不发空消息
        self.assertEqual(len(node.pub_usb_bytes.msgs), 2)
        node.record_usb(b'x' * rtk_node.USB_BYTES_MAX, 0.31)         # 攒满马上发
        self.assertEqual(len(node.pub_usb_bytes.msgs), 3)

    def test_publish_error_never_reaches_the_reader(self):
        node = bare_node()

        class Broken:
            def publish(self, _msg):
                raise RuntimeError('boom')
        node.pub_usb_bytes = Broken()
        node.record_usb(b'abc', 0.0)
        node.record_usb(b'', 0.2)
        self.assertIn('boom', node.usb_bytes_error)
        self.assertEqual(node.usb_bytes, bytearray())


class FakeWriter:
    opened = []

    def __init__(self, uri):
        self.uri, self.messages, self.closed = uri, [], False
        FakeWriter.opened.append(self)

    def write(self, topic, type_name, data, t_ns):
        self.messages.append((t_ns, topic))

    def close(self):
        self.closed = True


class Recorder(unittest.TestCase):
    def test_usb_bytes_keep_120_s_before_start_others_10_s(self):
        FakeWriter.opened = []
        with tempfile.TemporaryDirectory() as tmp:
            t = [1000.0]
            rec = jr.JobRecorder(root=tmp, writer=FakeWriter, files=lambda: [], clock=lambda: t[0],
                                 wall=lambda: 1.8e9 + t[0], log=lambda *_: None)
            while t[0] < 1300.0:                                    # 停着 300 s
                ns = int(t[0] * 1e9)
                rec.on_message('/motion/status', 'std_msgs/msg/String', b'm', ns, {'mode': 'IDLE'})
                rec.on_message('/gnss/raw', 'std_msgs/msg/String', b'g', ns)
                rec.on_message('/gnss/usb_bytes', 'std_msgs/msg/UInt8MultiArray', b'u', ns)
                t[0] += 0.5
            rec.on_message('/motion/status', 'std_msgs/msg/String', b'm', int(t[0] * 1e9), {'mode': 'MANUAL'})
            seg, = FakeWriter.opened
            start = t[0]
            first = {}
            for t_ns, topic in seg.messages:
                first.setdefault(topic, t_ns / 1e9)
            self.assertAlmostEqual(start - first['/gnss/usb_bytes'], 120.0, delta=0.6)
            self.assertAlmostEqual(start - first['/gnss/raw'], jr.PRE_S, delta=0.6)
            stamps = [t_ns for t_ns, _ in seg.messages]
            self.assertEqual(stamps, sorted(stamps))                # 开始前那一段按时间先后写
            self.assertIn('/gnss/usb_bytes', jr.TOPICS)
            self.assertIn('/gnss/usb_bytes', jr.README)


class Extract(unittest.TestCase):
    def test_split_restores_receiver_bytes_and_rtcm_frames(self):
        fs = car_frames()[:6]
        r1, r2 = rtcm_frame(1005), rtcm_frame(1074, 200)
        stream = fs[0] + rtcm_line(r1) + fs[1] + fs[2] + rtcm_line(r2) + gga() + b'$RTCM3,D300ZZ\r\n' + fs[3]
        receiver, frames, bad = ux.split_stream(stream)
        self.assertEqual(receiver, fs[0] + fs[1] + fs[2] + gga() + fs[3])
        self.assertEqual(frames, [r1, r2])
        self.assertEqual(bad, 1)
        self.assertEqual([ux.rtcm_type(f) for f in frames], [1005, 1074])


if __name__ == '__main__':
    unittest.main()
