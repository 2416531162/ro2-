#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跟随节点端到端逻辑测试(ROS 替身 + 模拟 N10P 扫描,不需要实车)。

对应的现场问题:
- 一点启动就报「雷达 AEB 硬急停」,状态 RECOVERY_EXHAUSTED / no_observed_path,
  前方 1.58m 明明空着。根因:N10P 每圈约 450 个点却分进 720 个格子,
  三分之一格子是 inf,旧版路径检查要求紧挨着的两个格子都有回波,几乎每帧都判定无路。
- 相机深度无效、改用雷达兜底测距时,人在左边车往右打舵。

    python3 tests/test_follower_node.py
"""

import json
import math
import os
import sys
import types
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tests"))
sys.path.insert(0, os.path.join(ROOT, "radar_system"))

import ros_stubs  # noqa: E402

ros_stubs.install()
sys.modules['std_msgs.msg'].Float32 = ros_stubs._simple('Float32')
if 'std_srvs.srv' not in sys.modules:
    _srv = types.ModuleType('std_srvs.srv')
    _srv.SetBool = _srv.Trigger = type('Srv', (), {'Request': object})
    sys.modules['std_srvs'] = types.ModuleType('std_srvs')
    sys.modules['std_srvs.srv'] = _srv
ros_stubs.Node.create_client = lambda self, *a, **k: types.SimpleNamespace(
    service_is_ready=lambda: False)

import person_follower as pf  # noqa: E402
from follower_recovery import ScanEvidence  # noqa: E402

String = sys.modules['std_msgs.msg'].String
BINS = 720


def room_range(angle, half_size=2.0):
    """以雷达为中心、边长 4m 的方形房间。"""
    c, s = abs(math.cos(angle)), abs(math.sin(angle))
    return min(half_size / c if c > 1e-6 else 1e9, half_size / s if s > 1e-6 else 1e9)


def n10p_scan(samples_per_rev=450, phase_deg=0.13, extra=None):
    """复现 real_lidar_node 的分桶方式:720 格,只有 samples_per_rev 个格子有回波。"""
    ranges = [math.inf] * BINS
    for k in range(samples_per_rev):
        ang = (phase_deg + k * 360.0 / samples_per_rev) % 360.0
        key = round(((360.0 - ang) % 360.0) * 2) % BINS
        a = key * 2 * math.pi / BINS
        r = room_range(a)
        if extra:
            r = min(r, extra(a))
        ranges[key] = r
    return types.SimpleNamespace(
        header=None, ranges=ranges, angle_min=0.0,
        angle_increment=2 * math.pi / BINS, range_min=0.15, range_max=12.0)


class FollowerHarness:
    def __init__(self):
        self.node = pf.PersonFollowerNode(pf.FollowerConfig(), dry_run=True)
        self.node.print_dashboard = lambda s: None

    def driver_ok(self, speed=0.0):
        self.node.on_driver_status(String(data=json.dumps({
            'armed': True, 'ready': 'ready', 'connected': True, 'holding': False,
            'age_ms': 5, 'telemetry': {'velocity': [speed, 0.0, 0.0]}})))

    def tick(self, scan, targets=None):
        self.driver_ok(self.node.cmd_vx)        # 模拟底盘跟上指令,否则会被判为堵转
        self.node.on_scan(scan)
        if targets is not None:
            self.node.on_targets(String(data=json.dumps(targets)))
        self.node.last_control_time = None      # 测试里不模拟控制周期抖动
        self.node.control_loop()

    def status(self):
        return json.loads(self.node.pub_status.sent[-1].data)


class TestScanEvidenceWithSparseBins(unittest.TestCase):

    def test_real_n10p_density_still_certifies_open_space(self):
        mount = pf.FollowerConfig().lidar_mount
        fp = pf.FollowerConfig().footprint
        for phase in (0.0, 0.13, 0.41, 0.7):
            msg = n10p_scan(450, phase)
            ev = ScanEvidence(msg.ranges, 0.0, msg.angle_increment, 0.15, 12.0, mount, fp)
            self.assertTrue(ev.free(1.2, 0.0), f"phase={phase}")
            self.assertTrue(ev.free(1.0, 0.3), f"phase={phase}")

    def test_obstacles_and_wide_holes_stay_blocking(self):
        mount = pf.FollowerConfig().lidar_mount
        fp = pf.FollowerConfig().footprint
        msg = n10p_scan(450)
        ev = ScanEvidence(msg.ranges, 0.0, msg.angle_increment, 0.15, 12.0, mount, fp)
        self.assertFalse(ev.free(2.8, 0.0), "墙后面不能算空地")
        ranges = list(msg.ranges)
        for i in list(range(0, 12)) + list(range(BINS - 12, BINS)):
            ranges[i] = math.inf                # 正前方 ±6° 整片没有回波
        ev = ScanEvidence(ranges, 0.0, msg.angle_increment, 0.15, 12.0, mount, fp)
        self.assertFalse(ev.covered(1.2, 0.0), "大片缺失仍然是未知")


class TestTurningWithRearBlindSector(unittest.TestCase):
    """默认屏蔽扇区 (155°, -130°) 下,静止起步也必须能左右打舵。"""

    def setUp(self):
        from follower_recovery import LocalRecovery
        self.cfg = pf.FollowerConfig()
        self.rec = LocalRecovery(self.cfg.footprint, self.cfg.geometry,
                                 self.cfg.obstacle_profile)

    def evidence(self, ranges=None):
        msg = n10p_scan(450)
        return ScanEvidence(ranges or msg.ranges, 0.0, msg.angle_increment, 0.15, 12.0,
                            self.cfg.lidar_mount, self.cfg.footprint,
                            self.cfg.scan_blind_sectors_deg)

    def test_forward_turns_allowed_from_standstill(self):
        ev = self.evidence()
        for deg in (-20, -10, -5, 5, 10, 20):
            self.assertGreaterEqual(self.rec.clearance(ev, math.radians(deg)), 0.5, deg)

    def test_reverse_into_blind_sector_stays_blocked(self):
        ev = self.evidence()
        for deg in (-20, 0, 20):
            self.assertLess(self.rec.clearance(ev, math.radians(deg), -1), 0.05, deg)

    def test_unmasked_missing_returns_ahead_still_block(self):
        ranges = list(n10p_scan(450).ranges)
        for i in list(range(0, 16)) + list(range(BINS - 16, BINS)):
            ranges[i] = math.inf                # 正前方 ±8° 没有回波(可能是贴脸的物体)
        self.assertLess(self.rec.clearance(self.evidence(ranges), 0.0), 0.1)

    def test_obstacle_beside_turn_still_blocks(self):
        def post(a):   # 右前方约 0.55m 处的立柱,落在右转扫掠范围内
            d = math.atan2(math.sin(a + math.radians(40)), math.cos(a + math.radians(40)))
            return 0.55 if abs(d) < math.radians(3) else math.inf
        msg = n10p_scan(450, extra=post)
        ev = ScanEvidence(msg.ranges, 0.0, msg.angle_increment, 0.15, 12.0,
                          self.cfg.lidar_mount, self.cfg.footprint,
                          self.cfg.scan_blind_sectors_deg)
        self.assertLess(self.rec.clearance(ev, math.radians(-20)),
                        self.rec.clearance(ev, math.radians(20)))


class TestFollowerStartup(unittest.TestCase):

    def test_start_without_person_does_not_latch_aeb(self):
        h = FollowerHarness()
        for _ in range(20):
            h.tick(n10p_scan())
        s = h.status()
        self.assertFalse(s['aeb_active'], s)
        self.assertGreater(s['path_clearance_m'], 0.5, s)
        self.assertNotEqual(s['limit_reason'], 'no_observed_path', s)

    def test_person_ahead_is_followed(self):
        h = FollowerHarness()
        person = [{'label': 'person', 'conf': 0.9, 'x': 0.0, 'y': 0.0,
                   'z': 2.4, 'depth_ratio': 0.9}]
        for _ in range(30):
            h.tick(n10p_scan(), person)
        s = h.status()
        self.assertEqual(s['state'], 'TRACKING', s)
        self.assertGreater(s['cmd_vx'], 0.05, s)
        self.assertFalse(s['aeb_active'], s)

    def test_lidar_fallback_steers_toward_person_on_left(self):
        """相机深度无效、只剩雷达测距时,人在左前方,车必须往左打舵(ROS: 正为左)。"""
        h = FollowerHarness()
        bearing = math.radians(20)

        def legs(a):
            d = math.atan2(math.sin(a - bearing), math.cos(a - bearing))
            return 2.0 if abs(d) < math.radians(1.5) else math.inf

        person = [{'label': 'person', 'conf': 0.9, 'x': None, 'z': None,
                   'range_valid': False, 'bearing_rad': bearing}]
        for _ in range(30):
            h.tick(n10p_scan(extra=legs), person)
        self.assertGreater(h.node.lidar_fallback_matches, 0)
        self.assertLess(h.node.tracker_x.position, 0.0, "横向偏移右为正,人在左应为负")
        self.assertGreater(h.node.cmd_steer, 0.0, h.status())


if __name__ == '__main__':
    unittest.main()
