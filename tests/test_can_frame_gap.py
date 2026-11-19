"""CAN 帧间隔（oid_link.CanLink frame_gap_s）：每帧至少隔 frame_gap_s，等的时候收回复，接收时刻按收到那一刻记。"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', 'oid_protocol'))

import oid_codec as oc  # noqa: E402
from oid_link import CanLink  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


class Bus:
    def __init__(self, clock):
        self.clock, self.sent, self.pending = clock, [], []

    def send(self, can_id, data):
        self.sent.append((self.clock(), can_id, bytes(data)))

    def recv(self):
        due = [f for f in self.pending if f[0] <= self.clock()]
        self.pending = [f for f in self.pending if f[0] > self.clock()]
        return [(cid, data) for _, cid, data in due]


def reply(item, value):
    return bytes([0x0F, item]) + value.to_bytes(4, 'big', signed=True)


class FrameGapTest(unittest.TestCase):
    def make(self, gap):
        clock = Clock()
        bus = Bus(clock)
        link = CanLink(bus, {1: 126, 2: 116}, clock=clock, frame_gap_s=gap, sleep=clock.sleep)
        return clock, bus, link

    def test_frames_spaced(self):
        clock, bus, link = self.make(0.003)
        worst = 0.0
        for k in range(80):                                   # 40 Hz 跑 2 s：每拍都在 25 ms 内发完
            t0 = 100.0 + 0.025 * k
            clock.t = max(clock.t, t0)
            n = len(bus.sent)
            link.heartbeat(clock())
            link.command({1: 100, 2: -100})
            link.request(clock())
            worst = max(worst, bus.sent[-1][0] - t0)
            self.assertLessEqual(len(bus.sent) - n, 8)
        times = [t for t, _, _ in bus.sent]
        gaps = [b - a for a, b in zip(times, times[1:])]
        self.assertGreaterEqual(min(gaps), 0.003 - 1e-9)
        self.assertLess(worst, 0.025)
        beats = [t for t, cid, d in bus.sent if cid == 126 and d == bytes(oc.can_heartbeat())]
        self.assertLessEqual(max(b - a for a, b in zip(beats, beats[1:])), 0.13)   # 心跳不被挤掉（0.1 s 按 25 ms 一拍取整）
        kinds = {d[1] for _, _, d in bus.sent if d[0] == 0x0F}
        self.assertTrue({oc.Q_FAULT, oc.Q_VOLTAGE, oc.Q_MOTOR_CURRENT} <= kinds)   # 慢查询还在

    def test_no_gap_is_old_behaviour(self):
        clock, bus, link = self.make(0.0)
        link.command({1: 1, 2: 2})
        link.request(clock())
        self.assertEqual(len({t for t, _, _ in bus.sent}), 1)  # 同一时刻全部发出，不等

    def test_replies_received_while_waiting_keep_their_time(self):
        clock, bus, link = self.make(0.003)
        bus.pending.append((clock() + 0.0015, 126, reply(oc.Q_POSITION, 1234)))  # 3 ms 间隔内到
        link.request(clock())                                 # 发后面几帧时就把这条回复收了
        s = link.state[1]
        self.assertEqual(s.position_cdeg, 1234)
        self.assertAlmostEqual(s.t_position, 100.0015, delta=0.001)


if __name__ == '__main__':
    unittest.main()
