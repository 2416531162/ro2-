#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""作业记录：不在作业时车在运行（手动摇杆、自动）也记（2026-11-19 用户：不管手动和自动，只要在运行就把数据记录下来）。

不连 ROS、不写 bag：用假的写入器，只看什么时候开段、关段、目录叫什么。
"""
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'radar_system'))
import job_recorder as jr  # noqa: E402


class FakeWriter:
    opened = []

    def __init__(self, uri):
        self.uri, self.messages, self.closed = uri, [], False
        FakeWriter.opened.append(self)

    def write(self, topic, type_name, data, t_ns):
        self.messages.append(topic)

    def close(self):
        self.closed = True


class TestDriveRecording(unittest.TestCase):
    def setUp(self):
        FakeWriter.opened = []
        self.tmp = tempfile.TemporaryDirectory()
        self.t = [1000.0]
        self.rec = jr.JobRecorder(root=self.tmp.name, writer=FakeWriter, files=lambda: [],
                                  clock=lambda: self.t[0], wall=lambda: 1.8e9 + self.t[0], log=lambda *_: None)

    def tearDown(self):
        self.tmp.cleanup()

    def feed(self, seconds, mode, line=None, straight=None, dt=0.2):
        end = self.t[0] + seconds
        while self.t[0] < end - 1e-9:
            ns = int(self.t[0] * 1e9)
            self.rec.on_message('/motion/status', 'std_msgs/msg/String', b'm', ns, {'mode': mode})
            self.rec.on_message('/line/status', 'std_msgs/msg/String', b'l', ns, line or {'phase': 'IDLE'})
            self.rec.on_message('/straight/status', 'std_msgs/msg/String', b's', ns, straight or {'state': 'idle'})
            self.rec.on_message('/gnss/raw', 'std_msgs/msg/String', b'g', ns)
            self.rec.on_message('/imu/data', 'sensor_msgs/msg/Imu', b'i', ns)
            self.rec.tick()
            self.t[0] += dt

    def jobs(self):
        out = []
        for name in sorted(os.listdir(self.tmp.name)):
            with open(os.path.join(self.tmp.name, name, 'job.json'), encoding='utf-8') as fh:
                out.append((name, json.load(fh)))
        return out

    def test_manual_driving_is_recorded_with_gnss_and_imu(self):
        self.feed(5.0, 'IDLE')
        self.assertFalse(FakeWriter.opened)                                   # 停着不记
        self.feed(10.0, 'MANUAL')
        self.assertEqual(len(FakeWriter.opened), 1)
        seg = FakeWriter.opened[0]
        self.assertIn('/gnss/raw', seg.messages)
        self.assertIn('/imu/data', seg.messages)
        (name, info), = self.jobs()
        self.assertIn('手动驾驶', name)
        self.assertEqual(info['label'], '手动驾驶')
        self.feed(jr.DRIVE_POST_S + 1.0, 'IDLE')                             # 停下 30 s 后结束
        self.assertTrue(seg.closed)
        self.assertEqual(self.jobs()[0][1]['segments'][0]['closed_by'], 'idle')

    def test_short_stops_stay_in_one_recording(self):
        for _ in range(3):                                                    # 开开停停（每次停 10 s）
            self.feed(5.0, 'MANUAL')
            self.feed(10.0, 'IDLE')
        self.assertEqual(len(FakeWriter.opened), 1)
        self.assertFalse(FakeWriter.opened[0].closed)
        self.assertEqual(len(self.jobs()), 1)

    def test_automatic_motion_without_job_is_recorded(self):
        self.feed(3.0, 'NAVIGATION')
        (name, info), = self.jobs()
        self.assertEqual(info['label'], '自动运行')

    def test_route_job_still_gets_its_own_recording(self):
        self.feed(5.0, 'MANUAL')
        job = {'id': 'abcdef1234', 'label': '足球场', 'state': 'RUNNING'}
        line = {'phase': 'RUNNING', 'task': 'route', 'operations': {'job': job}}
        self.feed(5.0, 'NAVIGATION', line=line)
        self.assertEqual(len(FakeWriter.opened), 2)
        self.assertTrue(FakeWriter.opened[0].closed)                          # 手动那段先关
        labels = sorted(info['label'] for _, info in self.jobs())
        self.assertEqual(labels, ['手动驾驶', '足球场'])
        self.feed(jr.POST_S + 1.0, 'IDLE', line={'phase': 'IDLE', 'operations': {'job': dict(job, state='DONE')}})
        self.assertTrue(FakeWriter.opened[1].closed)                          # 作业照旧停下 5 s 就结束

    def test_imu_straight_keeps_its_own_name(self):
        self.feed(3.0, 'MANUAL')
        self.feed(3.0, 'NAVIGATION', straight={'state': 'running'})
        labels = [info['label'] for _, info in self.jobs()]
        self.assertEqual(labels.count(jr.STRAIGHT_TASK), 1)
        self.assertEqual(labels.count('手动驾驶'), 1)


if __name__ == '__main__':
    unittest.main()
