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


def n10p_scan(samples_per_rev=450, phase_deg=0.13, extra=None, half_size=2.0):
    """复现 real_lidar_node 的分桶方式:720 格,只有 samples_per_rev 个格子有回波。"""
    ranges = [math.inf] * BINS
    for k in range(samples_per_rev):
        ang = (phase_deg + k * 360.0 / samples_per_rev) % 360.0
        key = round(((360.0 - ang) % 360.0) * 2) % BINS
        a = key * 2 * math.pi / BINS
        r = room_range(a, half_size)
        if extra:
            r = min(r, extra(a))
        ranges[key] = r
    return types.SimpleNamespace(
        header=None, ranges=ranges, angle_min=0.0,
        angle_increment=2 * math.pi / BINS, range_min=0.15, range_max=12.0)


LIDAR_X = 0.53


def disc(cx, cy, radius):
    """车体系 (cx, cy) 处半径 radius 的圆柱(腿/椅子腿),返回 extra(angle) 函数。"""
    def ray(a):
        dx, dy = cx - LIDAR_X, cy
        along = dx * math.cos(a) + dy * math.sin(a)
        perp = abs(dx * math.sin(a) - dy * math.cos(a))
        if along <= 0 or perp > radius:
            return math.inf
        return along - math.sqrt(radius * radius - perp * perp)
    return ray


def person_legs(x, y):
    """人站在车体系 (x, y):两条腿,间距 0.2m。"""
    left, right = disc(x, y + 0.1, 0.06), disc(x, y - 0.1, 0.06)
    return lambda a: min(left(a), right(a))


def combine(*fns):
    return lambda a: min(f(a) for f in fns)


def camera_person(x, y, conf=0.9):
    """车体系 (x, y) 的人 -> 检测消息(相机俯角 15°,取躯干高度与光轴同高)。"""
    cfg = pf.FollowerConfig()
    horiz = x - cfg.camera_offset_x_m
    # 与相机同高的点:光轴俯 θ,则 z = h·cosθ,y(向下为正) = -h·sinθ
    z = horiz * math.cos(cfg.camera_pitch_rad)
    y_opt = -horiz * math.sin(cfg.camera_pitch_rad)
    return [{'label': 'person', 'conf': conf, 'x': -y, 'y': y_opt, 'z': z,
             'depth_ratio': 0.9}]


class FollowerHarness:
    def __init__(self, **overrides):
        cfg = pf.FollowerConfig()
        for k, v in overrides.items():
            setattr(cfg, k, v)
        self.node = pf.PersonFollowerNode(cfg, dry_run=True)
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


class TestCrossCheckAndHandoff(unittest.TestCase):

    def test_camera_person_geometry_helper(self):
        from footprint import optical_to_vehicle
        cfg = pf.FollowerConfig()
        det = camera_person(2.5, 0.3)[0]
        vx, vy, _ = optical_to_vehicle(det['x'], det['y'], det['z'],
                                       cfg.camera_mount, cfg.camera_pitch_rad)
        self.assertAlmostEqual(vx, 2.5, places=6)
        self.assertAlmostEqual(vy, 0.3, places=6)

    def test_chair_beside_line_of_sight_is_not_the_person(self):
        """人在正前方 2.9m(车头 2.2m),左前方约 12° 处 1.3m 有椅子腿。

        旧版 ±10° 扇形(分桶后到 15°)会把椅子当成人:车以为人在 0.7m 处,不走。
        椅子离视线 0.28m,在新的 ±0.25m 窄带之外。
        """
        h = FollowerHarness()
        world = combine(person_legs(2.9, 0.0), disc(1.9, 0.33, 0.05))
        for _ in range(30):
            h.tick(n10p_scan(extra=world, half_size=5.0), camera_person(2.9, 0.0))
        s = h.status()
        self.assertEqual(s['state'], 'TRACKING', s)
        self.assertAlmostEqual(s['target']['gap_used'], 2.23, delta=0.1)
        self.assertGreater(s['cmd_vx'], 0.2, s)
        self.assertEqual(s['range_conflicts'], 0)

    def test_object_on_line_of_sight_is_respected(self):
        """人和车之间正好有东西挡着:按更近的距离算,不往前冲,但不丢目标。"""
        h = FollowerHarness()
        world = combine(person_legs(3.2, 0.0), disc(1.60, 0.0, 0.05))
        for _ in range(30):
            h.tick(n10p_scan(extra=world, half_size=5.0), camera_person(3.2, 0.0))
        s = h.status()
        self.assertIsNotNone(s['target'], s)
        self.assertLess(s['target']['gap_used'], 1.0, s)
        self.assertLessEqual(s['cmd_vx'], 0.05, s)

    def test_person_walks_out_of_camera_view_to_the_left(self):
        """相机锁定后人向左走出画面,雷达接力:车继续跟并往左打舵。"""
        h = FollowerHarness()
        x, y = 2.6, 0.0
        for _ in range(25):
            h.tick(n10p_scan(extra=person_legs(x, y), half_size=5.0), camera_person(x, y))
        self.assertTrue(h.status()['lidar_track'], h.status())
        for _ in range(40):
            y += 0.03                                  # 每帧左移 3cm
            h.tick(n10p_scan(extra=person_legs(x, y), half_size=5.0), [])   # 相机已看不到
        s = h.status()
        self.assertTrue(s['lidar_handoff'], s)
        self.assertEqual(s['target']['range_source'], 'lidar_track')
        self.assertLess(s['target']['x'], -0.8, "人在左边,横向偏移为负")
        self.assertGreater(s['cmd_steer_deg'], 5.0, s)
        self.assertGreater(s['cmd_vx'], 0.0, s)
        self.assertLessEqual(s['speed_cap_mps'], 0.35 + 1e-9, s)

    def test_handoff_expires_without_camera_confirmation(self):
        h = FollowerHarness(lidar_handoff_max_s=0.3)
        for _ in range(25):
            h.tick(n10p_scan(extra=person_legs(2.6, 0.0), half_size=5.0), camera_person(2.6, 0.0))
        import time
        time.sleep(0.4)
        for _ in range(15):
            h.tick(n10p_scan(extra=person_legs(2.6, 0.0), half_size=5.0), [])
        s = h.status()
        self.assertFalse(s['lidar_handoff'], s)
        self.assertIsNone(s['target'], s)
        self.assertEqual(s['cmd_vx'], 0.0, s)

    def test_wall_is_not_adopted_as_person(self):
        """相机报的人位置附近只有墙:雷达不认领,也就不会接力跟墙。"""
        h = FollowerHarness()

        def wall(a):
            c = math.cos(a)
            return (2.2 - LIDAR_X + 0.53) / c if c > 0.2 else math.inf
        for _ in range(20):
            h.tick(n10p_scan(extra=wall), camera_person(2.2, 0.0))
        self.assertIsNone(h.status()['lidar_track'])


class TestStuckWithPersonVisible(unittest.TestCase):
    """现场:人在正前方 2.3m、置信度 91%,状态 RECOVERY_EXHAUSTED,车不动。"""

    def test_self_reflection_just_outside_body_does_not_block(self):
        """车侧外 1.5cm 的一个回波(轮子凸出/线缆):跟随节点当作自身,脱困模块也必须如此。"""
        h = FollowerHarness()
        world = combine(person_legs(3.0, 0.0), disc(0.45, 0.36, 0.005))
        for _ in range(30):
            h.tick(n10p_scan(extra=world, half_size=5.0), camera_person(3.0, 0.0))
        s = h.status()
        self.assertEqual(s['state'], 'TRACKING', s)
        self.assertGreater(s['cmd_vx'], 0.2, s)

    def test_brief_camera_dropout_does_not_exhaust_recovery(self):
        h = FollowerHarness(lidar_handoff=False)
        scan = n10p_scan(extra=person_legs(3.0, 0.0), half_size=5.0)
        for _ in range(20):
            h.tick(scan, camera_person(3.0, 0.0))
        import time
        t_end = time.monotonic() + 0.8           # 相机漏检 0.8s,触发丢失搜索
        while time.monotonic() < t_end:
            h.tick(scan, [])
            time.sleep(0.03)
        for _ in range(30):                      # 人重新出现
            h.tick(scan, camera_person(3.0, 0.0))
        s = h.status()
        self.assertFalse(s['recovery_exhausted'], s)
        self.assertEqual(s['state'], 'TRACKING', s)

    def test_blocked_path_is_reported_with_its_cause(self):
        """人可见但车头左前方紧贴一根柱子:明确报「前方无路」和挡路的位置。"""
        h = FollowerHarness()
        world = combine(person_legs(3.0, 0.0), disc(0.76, 0.32, 0.03))
        for _ in range(30):
            h.tick(n10p_scan(extra=world, half_size=5.0), camera_person(3.0, 0.0))
        s = h.status()
        self.assertEqual(s['cmd_vx'], 0.0, s)
        self.assertEqual(s['state'], 'PATH_BLOCKED', s)
        self.assertEqual(s['blocked_by']['kind'], 'obstacle', s)
        self.assertGreater(s['blocked_by']['lidar_bearing_deg'], 0.0, s)


class TestStructuralShadowAhead(unittest.TestCase):
    """现场:挡路 = 雷达看不到的区域,方位 16.7°、距离 0.20m(车上结构件遮挡)。"""

    def scan_with_sliver(self, centre_deg, width_deg, value):
        msg = n10p_scan(450, half_size=5.0)
        ranges = list(msg.ranges)
        for i in range(BINS):
            deg = math.degrees(i * msg.angle_increment)
            d = (deg - centre_deg + 180.0) % 360.0 - 180.0
            if abs(d) <= width_deg / 2:
                ranges[i] = value
        return ranges, msg.angle_increment

    def clearance(self, ranges, inc):
        from follower_recovery import LocalRecovery
        cfg = pf.FollowerConfig()
        ev = ScanEvidence(ranges, 0.0, inc, 0.15, 12.0, cfg.lidar_mount, cfg.footprint,
                          cfg.scan_blind_sectors_deg, self_hit_skin_m=cfg.self_hit_skin_m)
        rec = LocalRecovery(cfg.footprint, cfg.geometry, cfg.obstacle_profile)
        return rec.clearance(ev, 0.0), rec, ev

    def test_narrow_car_structure_shadow_is_bridged(self):
        clear, _, _ = self.clearance(*self.scan_with_sliver(-15.0, 3.0, -math.inf))
        self.assertGreater(clear, 0.5)

    def test_same_width_without_echo_still_blocks(self):
        """较宽但「完全没回波」(可能是黑色物体):仍然保守判为未知。"""
        clear, rec, ev = self.clearance(*self.scan_with_sliver(-15.0, 6.0, math.inf))
        self.assertLess(clear, 0.1)
        kind, x, y, _ = rec.last_block
        self.assertEqual(kind, "unknown")
        causes = {c for _, _, c in ev.explain(x, y)}
        self.assertIn("none", causes)

    def test_wide_structure_shadow_still_blocks(self):
        clear, _, _ = self.clearance(*self.scan_with_sliver(-15.0, 10.0, -math.inf))
        self.assertLess(clear, 0.1)

    def test_configured_narrow_front_blind_sector_is_bridged(self):
        """用户把车头的结构遮挡(测出来是 +inf)配进屏蔽扇区后,可以正常前进。"""
        from follower_recovery import LocalRecovery
        cfg = pf.FollowerConfig()
        ranges, inc = self.scan_with_sliver(-15.0, 3.0, math.inf)
        blind = cfg.scan_blind_sectors_deg + ((-17.0, -13.0),)
        ev = ScanEvidence(ranges, 0.0, inc, 0.15, 12.0, cfg.lidar_mount, cfg.footprint,
                          blind, self_hit_skin_m=cfg.self_hit_skin_m)
        rec = LocalRecovery(cfg.footprint, cfg.geometry, cfg.obstacle_profile)
        self.assertGreater(rec.clearance(ev, 0.0), 0.5)
        self.assertLess(rec.clearance(ev, 0.0, -1), 0.05, "车尾宽屏蔽不受影响")

    def test_status_explains_unknown_block(self):
        h = FollowerHarness()
        ranges, inc = self.scan_with_sliver(-15.0, 6.0, math.inf)
        msg = n10p_scan(450, half_size=5.0)
        msg.ranges = ranges
        for _ in range(20):
            h.tick(msg, camera_person(2.5, 0.0))
        b = h.status()['blocked_by']
        self.assertIsNotNone(b, h.status())
        self.assertEqual(b['kind'], 'unknown')
        self.assertGreater(b['ray_causes'].get('none', 0), 0)
        self.assertAlmostEqual(b['lidar_bearing_deg'], -15.0, delta=3.0)


class TestN10PNearEcho(unittest.TestCase):
    """驱动区分「太近」(-inf) 和「没回波」(+inf),且真实回波优先。"""

    def test_rank_order(self):
        from n10p_pipeline import _rank
        vals = [math.inf, -math.inf, 2.0, 0.5]
        self.assertEqual(sorted(vals, key=_rank)[:2], [0.5, 2.0])
        self.assertEqual(sorted(vals, key=_rank)[2], -math.inf)

    def test_decoder_marks_near_echo(self):
        from n10p_pipeline import N10PDecoder, FRAME
        pkt = bytearray(FRAME)
        pkt[0:2] = b'\xa5\x5a'
        pkt[2], pkt[3] = FRAME, 16
        pkt[5:7] = (1000).to_bytes(2, 'big')        # 起始角 10.00°
        pkt[105:107] = (2000).to_bytes(2, 'big')    # 结束角 20.00°
        for i in range(16):
            off = 7 + i * 6
            first = 80 if i == 0 else (0 if i == 1 else 1500)   # 8cm / 无回波 / 1.5m
            pkt[off:off + 2] = first.to_bytes(2, 'big')
        pkt[-1] = sum(pkt[:-1]) & 255
        points = [p for batch in N10PDecoder().feed(bytes(pkt)) for p in batch]
        self.assertEqual(points[0][1], -math.inf)
        self.assertEqual(points[1][1], math.inf)
        self.assertAlmostEqual(points[2][1], 1.5)


class TestStuckAgainstLowObstacle(unittest.TestCase):
    """现场照片:左前轮顶在推车上(底板低于雷达扫描面),识别到人也要会倒车。"""

    def drive_then_stall(self, steer, drive_s=4.0, stall_s=8.0, odom_ok=True):
        from follower_recovery import LocalRecovery
        cfg = pf.FollowerConfig()
        msg = n10p_scan(450, half_size=5.0)
        ev = ScanEvidence(msg.ranges, 0.0, msg.angle_increment, 0.15, 12.0, cfg.lidar_mount,
                          cfg.footprint, cfg.scan_blind_sectors_deg,
                          self_hit_skin_m=cfg.self_hit_skin_m)
        rec = LocalRecovery(cfg.footprint, cfg.geometry, cfg.obstacle_profile)
        t, dt, speed = 0.0, 0.05, 0.0
        reversed_m, log = 0.0, []
        for k in range(int((drive_s + stall_s) / dt)):
            t += dt
            stalled = k * dt >= drive_s
            wz = speed * math.tan(steer) / cfg.geometry.wheelbase_m
            c = rec.update(now=t, scan=ev, healthy=True, odom_ok=odom_ok, speed=speed,
                           yaw_rate=wz, target=True, gap=2.0, bearing=0.3,
                           requested_speed=0.3, requested_steer=steer, current_steer=steer,
                           follow_cap=0.55, lost_age=0.0)
            speed = 0.0 if (stalled and c.speed > 0) else c.speed
            if c.speed < 0:
                reversed_m += -c.speed * dt
            log.append((c.state, c.speed, c.steer))
        return reversed_m, log, rec

    def test_reverses_along_trail_while_steering(self):
        reversed_m, log, rec = self.drive_then_stall(steer=0.2)
        self.assertIn('RECOVERY_REVERSE', [s for s, _, _ in log])
        self.assertGreater(reversed_m, 0.15)
        self.assertLessEqual(reversed_m, rec.cfg.blind_reverse_m + 1e-6, "倒车不超过额度")

    def test_no_trail_no_reverse(self):
        """没有来路(刚启动就卡住):车尾看不见,绝不盲倒。"""
        reversed_m, _, _ = self.drive_then_stall(steer=0.2, drive_s=0.0)
        self.assertEqual(reversed_m, 0.0)

    def test_broken_odometry_forgets_trail(self):
        reversed_m, _, rec = self.drive_then_stall(steer=0.0, odom_ok=False)
        self.assertEqual(reversed_m, 0.0)
        self.assertEqual(rec.trail_length, 0.0)

    def test_escape_leg_steers_away_from_stall_side(self):
        _, log, rec = self.drive_then_stall(steer=0.25)
        first_rev = next(i for i, (s, v, _) in enumerate(log) if v < 0)
        after = [st for s, v, st in log[first_rev:] if v > 0 and s != 'TRACKING']
        self.assertTrue(after, "倒车后应有一段脱困前进")
        self.assertLess(after[0], 0.0, "卡住时左打舵,脱困前进应往右")

    def test_trail_survives_cancel(self):
        _, _, rec = self.drive_then_stall(steer=0.0, stall_s=0.1)
        before = rec.trail_length
        self.assertGreater(before, 0.5)
        rec.active = True
        rec.cancel()
        self.assertEqual(rec.trail_length, before)


if __name__ == '__main__':
    unittest.main()
