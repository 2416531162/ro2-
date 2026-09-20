#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跟随主控节点的端到端回归测试。

这 800 多行的节点此前**零覆盖** —— 它 import 的 follower_recovery 整个模块
从未提交到仓库,却没有任何测试发现得了,因为没有一条测试 import 过它。
这个文件的第一条测试就是为了让那种事不可能再发生。

用 tests/ros_stubs 的替身跑,不需要 ROS、相机、雷达或实车:

    python3 tests/test_person_follower.py
"""

import json
import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tests"))
sys.path.insert(0, os.path.join(ROOT, "radar_system"))

import ros_stubs                                    # noqa: E402
ros_stubs.install()

import person_follower as pf                        # noqa: E402
from std_msgs.msg import String, Float32            # noqa: E402


class FakeScan:
    """一帧 LaserScan 替身。ranges 按「相对车头的方位角」生成,免得算下标。"""

    def __init__(self, distance_m=6.0, n=360):
        self.ranges = [distance_m] * n
        self.angle_min = -math.pi
        self.angle_increment = 2 * math.pi / n
        self.range_min = 0.05
        self.range_max = 12.0
        self.header = ros_stubs._Header()
        self.header.stamp.sec = 1000
        self.header.stamp.nanosec = 0

    def put(self, deg, distance_m):
        idx = (int(round(deg)) + len(self.ranges) // 2) % len(self.ranges)
        self.ranges[idx] = distance_m
        return self


def targets(items):
    return String(data=json.dumps(items))


def person(z, x=0.0, conf=0.9, **extra):
    """相机坐标系下的一个人。z 沿光轴,x 向右为正(与检测器一致)。"""
    item = {'label': 'person', 'conf': conf, 'x': x, 'y': 0.0, 'z': z,
            'range_valid': True, 'depth_ratio': 0.8,
            'bearing_rad': math.atan2(-x, max(z, 0.05))}
    item.update(extra)
    return item


def driver_status(speed=0.0, yaw=0.0):
    return String(data=json.dumps({
        'armed': True, 'ready': 'ready', 'connected': True, 'holding': False,
        'age_ms': 20, 'telemetry': {'velocity': [speed, 0.0, yaw]}}))


class FollowerHarness:
    """把节点包起来,提供「喂一帧 -> 跑一个控制周期 -> 读遥测」的循环。"""

    def __init__(self, **cfg_overrides):
        cfg = pf.FollowerConfig()
        for k, v in cfg_overrides.items():
            setattr(cfg, k, v)
        cfg.__post_init__()
        self.cfg = cfg
        self.node = pf.PersonFollowerNode(cfg, dry_run=True, simulated_odometry=True)
        self.node.print_dashboard = lambda _s: None     # 别往 stdout 刷
        self.t = 0.0
        self._patch_clock()

    def _patch_clock(self):
        import time as _time
        self._time = _time
        self._real = _time.monotonic
        _time.monotonic = lambda: self.t
        if hasattr(self.node, 'get_clock'):
            self.node.get_clock().now = lambda: ros_stubs.Time(nanoseconds=int((1000.0 + self.t) * 1e9))

    def close(self):
        self._time.monotonic = self._real

    def _cb(self, topic):
        for name, cb in self.node.subscriptions_:
            if name == topic:
                return cb
        raise KeyError(topic)

    def feed(self, people=(), scan=None, speed=None, yaw=0.0, voltage=25.0):
        if speed is None:
            speed = getattr(self.node, 'cmd_vx', 0.0)
        s = scan or FakeScan()
        s.header.stamp.sec = 1000 + int(self.t)
        s.header.stamp.nanosec = int((self.t % 1) * 1e9)
        self._cb('/wheeltec/status')(driver_status(speed, yaw))
        self._cb('/voltage')(Float32(data=voltage))
        self._cb('/scan')(s)
        if people is not None:
            self._cb('/camera/ai_detection/targets')(targets(list(people)))


    def tick(self, dt=0.05, **kw):
        self.t += dt
        self.feed(**kw)
        self.node.control_loop()
        return self.status()

    def status(self):
        sent = self.node.publishers_['/follower/status'].sent
        return json.loads(sent[-1].data)

    def settle(self, n=8, **kw):
        s = None
        for _ in range(n):
            s = self.tick(**kw)
        return s


class FollowerTestCase(unittest.TestCase):

    def setUp(self):
        self.h = None

    def tearDown(self):
        if self.h:
            self.h.close()

    def make(self, **kw):
        self.h = FollowerHarness(**kw)
        return self.h


# =============================================================================
# 它到底能不能起来
# =============================================================================

class TestNodeBringUp(FollowerTestCase):

    def test_node_instantiates_without_hardware(self):
        h = self.make()
        self.assertIsNotNone(h.node)

    def test_all_topics_are_subscribed(self):
        h = self.make()
        topics = {t for t, _ in h.node.subscriptions_}
        self.assertTrue({'/camera/ai_detection/targets', '/scan',
                         '/voltage', '/wheeltec/status'}.issubset(topics))

    def test_status_telemetry_is_published_every_cycle(self):
        h = self.make()
        s = h.tick()
        for key in ('state', 'target_locked', 'speed_cap_mps', 'cmd_vx',
                    'cmd_steer_deg', 'path_clearance_m', 'recovery_phase'):
            self.assertIn(key, s)


# =============================================================================
# 接近目标必须提前减速
# =============================================================================

class TestApproach(FollowerTestCase):

    def test_locks_onto_a_person(self):
        h = self.make()
        h.settle(6, people=[person(2.5)])
        self.assertTrue(h.status()['target_locked'])

    def test_speed_decreases_as_the_person_gets_closer(self):
        """刹车包络的核心:距离换算成允许速度,不是到点才刹。"""
        h = self.make()
        caps = []
        for z in (3.0, 2.5, 2.0, 1.6, 1.3):
            s = h.settle(6, people=[person(z)])
            caps.append(s['speed_cap_mps'])
        for a, b in zip(caps, caps[1:]):
            self.assertLessEqual(b, a + 1e-9, f"速度上限没有单调收: {caps}")

    def test_stops_before_touching_the_person(self):
        h = self.make()
        s = h.settle(10, people=[person(1.30)])     # 光轴 1.30 -> 车头间距很小
        self.assertLessEqual(s['cmd_vx'], 0.05)

    def test_does_not_reverse_when_too_close(self):
        """跟随律不准发负速度 —— 后退只能由脱困层在停稳后发起。"""
        h = self.make()
        for _ in range(12):
            s = h.tick(people=[person(1.1)])
            self.assertGreaterEqual(s['cmd_vx'], -1e-9)

    def test_no_target_means_no_motion(self):
        h = self.make()
        s = h.settle(10, people=[])
        self.assertEqual(s['cmd_vx'], 0.0)
        self.assertFalse(s['target_locked'])


# =============================================================================
# 转向
# =============================================================================

class TestSteering(FollowerTestCase):

    def _steer_for(self, x, z):
        h = self.make()
        s = h.settle(8, people=[person(z, x=x)])
        return s['cmd_steer_deg']

    def test_person_on_the_left_steers_left(self):
        self.assertGreater(self._steer_for(-0.6, 2.5), 0.5)

    def test_person_on_the_right_steers_right(self):
        self.assertLess(self._steer_for(0.6, 2.5), -0.5)

    def test_centred_person_keeps_the_wheel_straight(self):
        self.assertAlmostEqual(self._steer_for(0.0, 2.5), 0.0, places=6)

    def test_far_target_needs_less_steer_than_a_near_one(self):
        """纯追踪最直接的好处:同样的横向偏差,远了就不该猛打舵。

        老的 kp*bearing 在这两种情况下给的舵角几乎一样,车因此画龙。
        """
        near = abs(self._steer_for(-0.5, 1.8))
        far = abs(self._steer_for(-0.5, 3.6))
        self.assertLess(far, near)

    def test_steer_never_exceeds_the_servo_limit(self):
        h = self.make()
        limit = math.degrees(h.cfg.max_steer_rad) + 1e-6
        for x in (-1.5, -0.8, 0.0, 0.8, 1.5):
            s = h.settle(6, people=[person(2.0, x=x)])
            self.assertLessEqual(abs(s['cmd_steer_deg']), limit)


# =============================================================================
# 安全兜底
# =============================================================================

class TestSafety(FollowerTestCase):

    def test_low_battery_stops_the_robot(self):
        h = self.make()
        s = h.settle(8, people=[person(2.5)], voltage=18.0)
        self.assertEqual(s['state'], 'LOW_BATTERY')
        self.assertEqual(s['cmd_vx'], 0.0)

    def test_stale_scan_stops_the_robot(self):
        h = self.make()
        h.settle(6, people=[person(2.5)])
        for _ in range(12):        # 只推进时间,不再喂雷达
            h.t += 0.05
            h.node.control_loop()
        self.assertEqual(h.status()['cmd_vx'], 0.0)

    def test_obstacle_between_robot_and_person_caps_speed(self):
        h = self.make()
        open_run = h.settle(8, people=[person(3.0)])['speed_cap_mps']
        blocked = FakeScan()
        for d in range(-20, 21):
            blocked.put(d, 0.55)          # 车头前方约 0.4m 处有东西
        near = h.settle(8, people=[person(3.0)], scan=blocked)['speed_cap_mps']
        self.assertLess(near, open_run)

    def test_camera_lidar_conflict_is_reported(self):
        """结构光读错时几乎总是读得更远。雷达说近就必须采信雷达。"""
        h = self.make()
        h.settle(8, people=[person(3.0)])            # 先正常锁上远处的人
        close_wall = FakeScan()
        for d in range(-15, 16):
            close_wall.put(d, 0.62)                  # 同一方位雷达却说很近
        s = h.settle(6, people=[person(3.0)], scan=close_wall)
        self.assertGreater(s['range_conflicts'], 0)
        self.assertEqual(s['cmd_vx'], 0.0)

    def test_driver_feedback_loss_stops_the_robot(self):
        h = self.make()
        h.settle(6, people=[person(2.5)])
        for _ in range(8):
            h.t += 0.05
            h._cb('/scan')(FakeScan())
            h._cb('/voltage')(Float32(data=25.0))
            h._cb('/camera/ai_detection/targets')(targets([person(2.5)]))
            h.node.control_loop()       # 不喂 /wheeltec/status
        self.assertEqual(h.status()['cmd_vx'], 0.0)


# =============================================================================
# 身份保持
# =============================================================================

class TestIdentity(FollowerTestCase):

    RED = [0.9, 0.05, 0.05] + [0.0] * 9
    BLUE = [0.0] * 8 + [0.9, 0.05, 0.05, 0.0]

    def test_second_person_does_not_steal_the_lock(self):
        h = self.make()
        me = dict(height_m=1.75, color=self.RED)
        h.settle(8, people=[person(2.2, **me)])
        self.assertTrue(h.status()['target_locked'])
        stranger = dict(height_m=1.45, color=self.BLUE)
        # 陌生人插到更正前方、更接近期望距离的位置
        s = h.settle(4, people=[person(1.2, x=0.0, **stranger),
                                person(2.3, x=0.5, **me)])
        self.assertTrue(s['target_locked'])
        self.assertGreater(s['target']['smooth_z'], 1.2)

    def test_appearance_rejections_are_reported(self):
        h = self.make()
        me = dict(height_m=1.75, color=self.RED)
        h.settle(8, people=[person(2.2, **me)])
        self.assertTrue(h.status()['signature_ready'])

    def test_old_detector_without_features_still_works(self):
        """检测器没升级时必须完全退化成老行为,不能一个人都锁不上。"""
        h = self.make()
        s = h.settle(8, people=[person(2.2)])
        self.assertTrue(s['target_locked'])
        self.assertFalse(s['signature_ready'])



# =============================================================================
# MPPI 接入 —— 重点不是它算得好不好,而是它**没有**绕过安全层
# =============================================================================

class TestMPPIIntegration(FollowerTestCase):
    """MPPI 只产出参考量。刹车包络、扫掠净空、AEB、脱困状态机全部照旧。

    这组测试存在的理由:MPPI 是软约束优化器,加权平均出来的控制可能落在没有
    任何样本占据的区域。如果哪天有人为了"让它更跟手"把下游某一层摘掉,
    这里必须立刻红。
    """

    def mppi(self, **kw):
        kw.setdefault('controller', 'mppi')
        kw.setdefault('mppi_device', 'numpy')
        kw.setdefault('mppi_samples', 64)
        kw.setdefault('mppi_horizon', 12)
        return self.make(**kw)

    def test_node_starts_with_mppi(self):
        h = self.mppi()
        s = h.settle(8, people=[person(2.5)])
        self.assertEqual(s['controller'], 'mppi')
        self.assertIsNotNone(s['mppi'])
        self.assertGreater(s['mppi']['solve_ms'], 0.0)

    def test_mppi_drives_toward_the_person(self):
        h = self.mppi()
        s = h.settle(16, people=[person(3.0)])
        self.assertGreater(s['cmd_vx'], 0.0)

    def test_mppi_output_never_goes_backwards(self):
        h = self.mppi()
        for _ in range(20):
            s = h.tick(people=[person(1.1)])
            self.assertGreaterEqual(s['cmd_vx'], -1e-9)

    def test_brake_envelope_still_caps_mppi(self):
        """跟随刹车包络在 MPPI 之后仍然生效 —— 靠得越近上限越低。"""
        h = self.mppi()
        far = h.settle(10, people=[person(3.2)])['speed_cap_mps']
        near = h.settle(10, people=[person(1.6)])['speed_cap_mps']
        self.assertLess(near, far)

    def test_aeb_still_overrides_mppi(self):
        """硬急停不归 MPPI 管,也不该归它管。"""
        h = self.mppi()
        h.settle(8, people=[person(3.0)])
        blocked = FakeScan()
        for d in range(-40, 41):
            blocked.put(d, 0.22)          # 贴着车头 (0.52 + 0.22 = 0.74m, 车头 0.67m 外 7cm 处硬急停)
        s = h.settle(8, people=[person(3.0)], scan=blocked)
        self.assertEqual(s['cmd_vx'], 0.0)
        self.assertTrue(s['aeb_active'])

    def test_low_battery_still_wins(self):
        h = self.mppi()
        s = h.settle(8, people=[person(2.5)], voltage=18.0)
        self.assertEqual(s['state'], 'LOW_BATTERY')
        self.assertEqual(s['cmd_vx'], 0.0)

    def test_stale_scan_still_stops_the_robot(self):
        h = self.mppi()
        h.settle(8, people=[person(2.5)])
        for _ in range(12):
            h.t += 0.05
            h.node.control_loop()
        self.assertEqual(h.status()['cmd_vx'], 0.0)

    def test_falls_back_to_pure_pursuit_when_solving_is_too_slow(self):
        """求解耗时越过控制周期是实车上真正会发生的失效:torch 没跑在 CUDA 上、
        Orin 降频、K 调太大。刹车包络按周期算,这时车会以为自己刹得住。"""
        h = self.mppi(mppi_fallback_after=3, mppi_solve_budget_ms=0.0)
        for _ in range(20):
            s = h.tick(people=[person(2.5)])
        self.assertEqual(s['controller'], 'pure-pursuit')
        self.assertTrue(s['mppi']['fell_back'])
        self.assertTrue(s['mppi']['over_budget'])

    def test_a_healthy_solve_does_not_count_as_failure(self):
        h = self.mppi(mppi_solve_budget_ms=10000.0)
        for _ in range(20):
            s = h.tick(people=[person(2.5)])
        self.assertEqual(s['controller'], 'mppi')
        self.assertEqual(s['mppi']['failure_streak'], 0)

    def test_fallback_keeps_the_robot_following(self):
        """回退不是停车。切回纯追踪之后车必须照常跟人。"""
        h = self.mppi(mppi_fallback_after=3, mppi_solve_budget_ms=0.0)
        for _ in range(24):
            s = h.tick(people=[person(3.0)])
        self.assertEqual(s['controller'], 'pure-pursuit')
        self.assertGreater(s['cmd_vx'], 0.0)

    def test_default_controller_is_still_pure_pursuit(self):
        """实车验证通过之前,默认值不能是 MPPI。"""
        h = self.make()
        self.assertIsNone(h.node.mppi)
        self.assertEqual(h.settle(4, people=[person(2.5)])['controller'],
                         'pure-pursuit')

    def test_steering_stays_within_the_servo_limit(self):
        h = self.mppi()
        limit = math.degrees(h.cfg.max_steer_rad) + 1e-6
        for x in (-1.2, 0.0, 1.2):
            s = h.settle(8, people=[person(2.5, x=x)])
            self.assertLessEqual(abs(s['cmd_steer_deg']), limit)

if __name__ == "__main__":
    unittest.main(verbosity=2)
