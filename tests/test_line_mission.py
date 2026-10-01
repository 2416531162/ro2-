#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""走直线任务流程(radar_system/line_mission.py)测试：融合用替身，不需要 ROS。

输入类型按与融合模块约定的字段定义；字段名写错会直接 TypeError，顺带检查接口约定。
"""
import csv
import json
import math
import os
import sys
import tempfile
import unittest
from dataclasses import dataclass
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, ROOT)

from line_mission import LineMission  # noqa: E402


@dataclass(frozen=True)
class ImuSample:
    t: float
    gyro_z: float


@dataclass(frozen=True)
class WheelSample:
    t: float
    speed: float
    yaw_rate: float


@dataclass(frozen=True)
class GnssFix:
    t: float
    lat_deg: float
    lon_deg: float
    height_m: float
    sol: str
    sigma_n: float
    sigma_e: float
    sigma_u: float
    diff_age_s: float
    sats: int


@dataclass(frozen=True)
class DualHeading:
    t: float
    heading_deg: float
    pitch_deg: float
    length_m: float
    sol: str
    sigma_heading_deg: float
    sats: int


@dataclass(frozen=True)
class GnssVelocity:
    t: float
    speed_mps: float
    track_deg: float
    sol: str


SAMPLES = SimpleNamespace(ImuSample=ImuSample, WheelSample=WheelSample, GnssFix=GnssFix,
                          DualHeading=DualHeading, GnssVelocity=GnssVelocity)


class FakeFusion:
    """位姿由测试直接给；经纬度用 1e-5°/m 的线性映射。"""

    def __init__(self):
        self.inputs = []
        self.current = self.make(0.0, 0.0, 0.0)

    @staticmethod
    def make(x, y, yaw_deg, speed=0.0, quality='RTK_FIXED', usable=True):
        return SimpleNamespace(x=x, y=y, yaw=math.radians(yaw_deg), speed=speed, yaw_rate=0.0,
                               usable_for_control=usable, quality=quality, sigma_xy=0.01,
                               sigma_yaw=math.radians(0.1), reasons=())

    def pose(self, t):
        return self.current

    def to_geodetic(self, x, y):
        return 30.0 + y * 1e-5, 120.0 + x * 1e-5

    def to_local(self, lat, lon):
        return (lon - 120.0) / 1e-5, (lat - 30.0) / 1e-5

    def __getattr__(self, name):
        if name.startswith('add_'):
            return lambda s: self.inputs.append((name, s))
        raise AttributeError(name)


class Base(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.state = os.path.join(self.tmp.name, 'state', 'line.json')
        self.logs = os.path.join(self.tmp.name, 'runs')
        self.fusion = FakeFusion()
        self.m = LineMission(self.fusion, SAMPLES, state_path=self.state, log_dir=self.logs)

    def tearDown(self):
        self.m._close_log()
        self.tmp.cleanup()

    def mark_line(self, length=10.0):
        self.fusion.current = FakeFusion.make(0.0, 0.0, 0.0)
        self.assertEqual(self.m.command({'cmd': 'mark', 'point': 'a'}, 0.0)[0], True)
        self.fusion.current = FakeFusion.make(length, 0.0, 0.0)
        self.assertEqual(self.m.command({'cmd': 'mark', 'point': 'B'}, 0.0)[0], True)
        self.fusion.current = FakeFusion.make(-0.5, 0.0, 0.0)

    def drive(self, t0, until, mode='NAVIGATION', dt=0.05):
        """简单运动学：按指令移动替身位姿，返回最后一次 tick 结果和时间。"""
        t, out = t0, None
        while t < until:
            out = self.m.tick(t, mode)
            p = self.fusion.current
            vx, wz = out[0], out[1]
            yaw = p.yaw + wz * dt
            self.fusion.current = FakeFusion.make(p.x + vx * dt * math.cos(yaw), p.y + vx * dt * math.sin(yaw),
                                                  math.degrees(yaw), speed=vx)
            t += dt
            if self.m.phase == 'IDLE' and out[2] is False:
                break
        return out, t


class TestInputs(Base):

    def test_gnss_imu_wheel_are_converted_to_fusion_samples(self):
        self.m.on_gnss(dict(kind='fix', t=1.0, lat=30.0, lon=120.0, height=20.0, sol='NARROW_INT',
                            sigma_n=0.01, sigma_e=0.01, sigma_u=0.02, diff_age=None, sats=30))
        self.m.on_gnss(dict(kind='heading', t=1.0, heading=91.0, pitch=0.2, length=0.331, sol='L1_INT',
                            sigma_heading=0.3, sats=30, valid=True))
        self.m.on_gnss(dict(kind='vel', t=1.0, speed=0.5, track=1.0, sol='DOPPLER_VELOCITY'))
        self.m.on_gnss(dict(kind='other'))
        self.m.on_imu(1.01, 0.002)
        self.m.on_wheel(1.02, 0.5, 0.001)
        # rtk_node 判为无效（两根天线没都固定）或没带 valid 的测向都不交给融合：融合不再按基线长度判断
        self.m.on_gnss(dict(kind='heading', t=1.005, heading=91.0, pitch=0.2, length=0.9, sol='L1_INT',
                            sigma_heading=0.3, sats=30, valid=False))
        self.m.on_gnss(dict(kind='heading', t=1.006, heading=91.0, pitch=0.2, length=0.331, sol='L1_INT',
                            sigma_heading=0.3, sats=30))
        names = [n for n, _ in self.fusion.inputs]
        self.assertEqual(names, ['add_fix', 'add_heading', 'add_velocity', 'add_imu', 'add_wheel'])
        fix = self.fusion.inputs[0][1]
        self.assertEqual(fix.diff_age_s, math.inf)
        self.assertEqual(self.fusion.inputs[1][1].length_m, 0.331)


class TestOperatorFlow(Base):

    def test_mark_needs_rtk_fixed(self):
        self.fusion.current = FakeFusion.make(0, 0, 0, quality='RTK_FLOAT')
        ok, msg = self.m.command({'cmd': 'mark', 'point': 'A'}, 0.0)
        self.assertFalse(ok)
        self.assertIn('固定解', msg)
        self.assertFalse(self.m.command({'cmd': 'mark', 'point': 'C'}, 0.0)[0])

    def test_start_requires_points_and_valid_speed(self):
        self.assertEqual(self.m.command({'cmd': 'start'}, 0.0), (False, '先记 A、B 两点'))
        self.mark_line()
        self.assertFalse(self.m.command({'cmd': 'start', 'cruise': 2.0}, 0.0)[0])
        self.fusion.current = FakeFusion.make(0.0, 0.0, 50.0)
        self.assertEqual(self.m.command({'cmd': 'start'}, 0.0), (False, 'heading_not_aligned'))

    def test_full_run_saves_trim_and_log(self):
        self.mark_line(10.0)
        self.assertTrue(self.m.command({'cmd': 'start', 'cruise': 0.5}, 0.0)[0])
        self.assertEqual(self.m.tick(0.05, 'IDLE'), (0.0, 0.0, True))     # 仲裁还没切过来：继续申请
        out, t = self.drive(0.1, 60.0)
        self.assertEqual(out, (0.0, 0.0, False))                          # 结束后交还控制权
        self.assertEqual(self.m.message, '到达 B 点')
        self.assertAlmostEqual(self.fusion.current.x, 10.0, delta=0.1)
        with open(self.state) as fh:
            self.assertIn('integral', json.load(fh))
        logs = os.listdir(self.logs)
        self.assertEqual(len(logs), 1)
        with open(os.path.join(self.logs, logs[0])) as fh:
            rows = list(csv.DictReader(fh))
        self.assertGreater(len(rows), 100)
        self.assertLess(max(abs(float(r['e_m'])) for r in rows), 0.01)
        # 下一次任务读回学到的零位
        again = LineMission(self.fusion, SAMPLES, state_path=self.state)
        self.assertEqual(again.integral0, self.m.integral0)

    def test_selection_timeout(self):
        self.mark_line()
        self.m.command({'cmd': 'start'}, 0.0)
        self.assertEqual(self.m.tick(1.0, 'MANUAL')[2], True)
        self.assertEqual(self.m.tick(3.5, 'MANUAL'), (0.0, 0.0, None))
        self.assertEqual(self.m.phase, 'IDLE')
        self.assertIn('控制权申请失败', self.m.message)
        self.assertEqual(self.m.tick(3.55, 'MANUAL'), (0.0, 0.0, False))

    def test_manual_takeover_ends_run(self):
        self.mark_line()
        self.m.command({'cmd': 'start'}, 0.0)
        self.drive(0.05, 2.0)
        self.assertEqual(self.m.phase, 'RUNNING')
        self.assertEqual(self.m.tick(2.05, 'MANUAL'), (0.0, 0.0, None))
        self.assertEqual(self.m.phase, 'IDLE')
        self.assertIn('MANUAL', self.m.message)

    def test_operator_stop_releases_navigation(self):
        self.mark_line()
        self.m.command({'cmd': 'start'}, 0.0)
        self.drive(0.05, 1.0)
        self.assertEqual(self.m.command({'cmd': 'stop'}, 1.0), (True, '已停止'))
        self.assertEqual(self.m.tick(1.05, 'NAVIGATION'), (0.0, 0.0, False))
        self.assertEqual(self.m.tick(1.10, 'NAVIGATION'), (0.0, 0.0, None))
        self.assertFalse(os.path.exists(self.state))                        # 没走完不学零位

    def test_pose_loss_pauses_and_operator_resumes(self):
        self.mark_line()
        self.m.command({'cmd': 'start'}, 0.0)
        self.drive(0.05, 2.0)
        p = self.fusion.current
        self.fusion.current = FakeFusion.make(p.x, p.y, math.degrees(p.yaw), quality='DEAD_RECKONING',
                                              usable=False)
        self.assertEqual(self.m.tick(2.05, 'NAVIGATION'), (0.0, 0.0, None))
        self.assertEqual((self.m.phase, self.m.tracker.st.state), ('RUNNING', 'PAUSED'))
        self.fusion.current = FakeFusion.make(p.x, p.y, math.degrees(p.yaw))
        self.assertEqual(self.m.tick(2.10, 'NAVIGATION')[:2], (0.0, 0.0))  # 恢复也不自己走
        self.assertEqual(self.m.command({'cmd': 'start'}, 2.2), (True, '继续'))
        out, _ = self.drive(2.25, 60.0)
        self.assertEqual(self.m.message, '到达 B 点')

    def test_status_is_json_ready(self):
        self.mark_line()
        self.m.tick(0.0, 'IDLE')
        st = self.m.status()
        json.dumps(st, allow_nan=False)
        self.assertEqual(st['points']['B'], [10.0, 0.0])
        self.assertEqual(st['pose']['quality'], 'RTK_FIXED')


RECT = dict(kind='points', points=[[0, 0], [10, 0], [10, 8], [0, 8]], closed=True, laps=1, turn_radius=2.5,
            cruise=0.5)


class NoOriginFusion(FakeFusion):
    """刚开机、还没有有效定位：融合没有原点。"""

    def to_geodetic(self, x, y):
        raise ValueError('尚未设置原点')

    def to_local(self, lat, lon):
        raise ValueError('尚未设置原点')


class TestRoute(Base):

    def setUp(self):
        super().setUp()
        self.route_file = os.path.join(self.tmp.name, 'rtk', 'route.json')
        self.m = LineMission(self.fusion, SAMPLES, state_path=self.state, log_dir=self.logs,
                             route_path=self.route_file)

    def test_set_stores_geodetic_and_echoes_local(self):
        self.assertEqual(self.m.command({'cmd': 'route_set', 'route': RECT}, 0.0), (True, '路线已保存到车上'))
        with open(self.route_file) as fh:
            saved = json.load(fh)
        self.assertEqual(saved['route']['points'][1], [30.0, 120.0001])     # 存经纬度
        st = self.m.status()
        json.dumps(st, allow_nan=False)
        self.assertEqual(st['route']['points'], RECT['points'])
        self.assertIsNone(st['route']['problem'])
        self.assertAlmostEqual(st['route']['length'], 36.0 - 3 * (5 - math.pi * 2.5 / 2), places=3)
        self.assertEqual(st['origin'], [30.0, 120.0])
        self.assertAlmostEqual(st['route_limits']['min_radius'], self.m.geo.min_turn_radius_m / 0.9, places=3)
        # 重启后从文件读回；原点变了照样对得上（替身融合的原点是固定的，这里只看读回）
        again = LineMission(self.fusion, SAMPLES, route_path=self.route_file)
        self.assertEqual(again.status()['route']['points'], RECT['points'])
        self.assertEqual(again.route_rev, self.m.route_rev)

    def test_incomplete_draft_is_saved_with_problem(self):
        draft = dict(RECT, points=[[0, 0], [5, 0]])
        self.assertTrue(self.m.command({'cmd': 'route_set', 'route': draft}, 0.0)[0])
        self.assertIn('至少要 3 个路点', self.m.status()['route']['problem'])
        ok, msg = self.m.command({'cmd': 'route_start'}, 0.0)
        self.assertFalse(ok)
        self.assertIn('至少要 3 个路点', msg)

    def test_bad_route_and_clear(self):
        self.assertFalse(self.m.command({'cmd': 'route_set', 'route': {'kind': 'star'}}, 0.0)[0])
        self.assertEqual(self.m.command({'cmd': 'route_start'}, 0.0), (False, '车上还没有路线'))
        self.m.command({'cmd': 'route_set', 'route': RECT}, 0.0)
        self.assertEqual(self.m.command({'cmd': 'route_clear'}, 0.0), (True, '已删除路线'))
        self.assertIsNone(self.m.status()['route'])
        self.assertFalse(os.path.exists(self.route_file))

    def test_needs_origin(self):
        m = LineMission(NoOriginFusion(), SAMPLES)
        ok, msg = m.command({'cmd': 'route_set', 'route': RECT}, 0.0)
        self.assertFalse(ok)
        self.assertIn('还没有定位原点', msg)
        st = m.status()
        self.assertIsNone(st['origin'])
        json.dumps(st, allow_nan=False)

    def test_full_run_from_car_pose(self):
        # 车在路线外、车头朝北：先接入，再跑两圈，停在第 1 个路点
        self.fusion.current = FakeFusion.make(-2.0, -3.0, 90.0)
        ok, msg = self.m.command({'cmd': 'route_start', 'route': dict(RECT, laps=2)}, 0.0)
        self.assertTrue(ok, msg)
        self.assertIn('接入', msg)
        self.assertEqual(self.m.task, 'route')
        out, t = self.drive(0.05, 300.0)
        self.assertEqual(out, (0.0, 0.0, False))
        self.assertEqual(self.m.message, '路线完成')
        p = self.fusion.current
        self.assertLess(math.hypot(p.x, p.y), 0.1)
        st = self.m.tracker.status()
        self.assertLess(st['stats']['max_m'], 0.05)
        self.assertTrue(os.listdir(self.logs)[0].startswith('route_'))
        self.assertFalse(os.path.exists(self.state))                        # 路线不改直线学到的零位

    def test_refusals(self):
        self.m.command({'cmd': 'route_set', 'route': RECT}, 0.0)
        self.fusion.current = FakeFusion.make(0, 0, 0, quality='DEAD_RECKONING', usable=False)
        self.assertIn('pose_unusable', self.m.command({'cmd': 'route_start'}, 0.0)[1])
        self.fusion.current = FakeFusion.make(200.0, 0, 0)
        self.assertIn('太远', self.m.command({'cmd': 'route_start'}, 0.0)[1])
        self.fusion.current = FakeFusion.make(-1.0, 0, 0)
        self.assertTrue(self.m.command({'cmd': 'route_start'}, 0.0)[0])
        self.assertEqual(self.m.command({'cmd': 'start'}, 0.1), (False, '已在运行'))
        self.assertEqual(self.m.command({'cmd': 'route_set', 'route': RECT}, 0.1), (False, '运行中不能改路线'))
        self.assertEqual(self.m.command({'cmd': 'route_clear'}, 0.1), (False, '先停止当前任务'))

    def test_pause_in_second_lap_resumes_there(self):
        self.fusion.current = FakeFusion.make(-1.0, 0.0, 0.0)
        self.m.command({'cmd': 'route_start', 'route': dict(RECT, laps=3)}, 0.0)
        self.drive(0.05, 90.0)                                              # 约 1.3 圈
        s_before = self.m.tracker.st.s
        lap = self.m.tracker.length / 3
        self.assertGreater(s_before, lap)
        p = self.fusion.current
        self.fusion.current = FakeFusion.make(p.x, p.y, math.degrees(p.yaw), usable=False)
        self.m.tick(90.05, 'NAVIGATION')
        self.assertEqual(self.m.tracker.st.state, 'PAUSED')
        self.fusion.current = FakeFusion.make(p.x, p.y, math.degrees(p.yaw))
        self.assertEqual(self.m.command({'cmd': 'start'}, 90.1)[0], False)  # 直线的开始不接管暂停的路线
        self.assertEqual(self.m.command({'cmd': 'route_start'}, 90.1), (True, '继续'))
        self.m.tick(90.15, 'NAVIGATION')
        self.assertAlmostEqual(self.m.tracker.st.s, s_before, delta=0.3)

    def gyro_only(self, p, sigma_deg=1.0, extra=()):
        """测向无效、陀螺维持的航向不确定度过了融合门槛：融合判不可控制（主天线仍是固定解）。"""
        q = FakeFusion.make(p.x, p.y, math.degrees(p.yaw), speed=p.speed, usable=False)
        q.sigma_yaw = math.radians(sigma_deg)
        q.reasons = ('heading_unavailable', 'gyro_only', 'heading_uncertain') + tuple(extra)
        return q

    def drive_gyro(self, t0, until, sigma_deg=1.0, fix2='NARROW_INT', extra=(), dt=0.05):
        """同 drive，但每步都是测向无效的位姿；fix2 是副天线每步的解（None 为不来）。"""
        t = t0
        while t < until:
            if fix2:
                self.m.on_gnss({'kind': 'fix2', 't': t, 'sol': fix2})
            self.fusion.current = self.gyro_only(self.fusion.current, sigma_deg, extra)
            vx, wz, _ = self.m.tick(t, 'NAVIGATION')
            p = self.fusion.current
            yaw = p.yaw + wz * dt
            self.fusion.current = FakeFusion.make(p.x + vx * dt * math.cos(yaw), p.y + vx * dt * math.sin(yaw),
                                                  math.degrees(yaw), speed=vx)
            t += dt
        return t

    def running_rect(self):
        """重新建一个任务，开始跑矩形路线 5 s。"""
        self.m.command({'cmd': 'stop'}, 0.0)  # Each subtest starts a new job; restart now preserves a checkpoint.
        self.m._close_log()
        self.fusion = FakeFusion()
        self.m = LineMission(self.fusion, SAMPLES, state_path=self.state, log_dir=self.logs, route_path=self.route_file)
        self.fusion.current = FakeFusion.make(-1.0, 0.0, 0.0)
        self.m.command({'cmd': 'route_start', 'route': RECT}, 0.0)
        self.drive(0.05, 5.0)
        self.assertEqual(self.m.tracker.st.state, 'RUNNING')

    def test_heading_float_keeps_going_on_gyro_when_both_antennas_fixed(self):
        self.running_rect()
        s0 = self.m.tracker.st.s
        self.drive_gyro(5.0, 15.0)
        self.assertEqual((self.m.phase, self.m.tracker.st.state), ('RUNNING', 'RUNNING'))
        self.assertGreater(self.m.tracker.st.s, s0 + 3.0)
        self.assertIn('gyro_hold', self.m.status()['pose']['reasons'])

    def test_gyro_hold_needs_both_antennas_fixed_and_small_uncertainty(self):
        # 副天线没来 / 不是固定解、航向不确定度超过 2°、还有别的禁止原因：照旧暂停
        for kw in (dict(fix2=None), dict(fix2='NARROW_FLOAT'), dict(sigma_deg=2.5), dict(extra=('position_reset',))):
            with self.subTest(case=str(kw)):
                self.running_rect()
                self.drive_gyro(5.0, 6.0, **kw)
                self.assertEqual(self.m.tracker.st.state, 'PAUSED')

    def test_pause_resumes_by_itself_once_pose_is_usable_again(self):
        self.running_rect()
        p = self.fusion.current
        self.fusion.current = FakeFusion.make(p.x, p.y, math.degrees(p.yaw), usable=False)
        self.m.tick(5.0, 'NAVIGATION')
        self.assertEqual(self.m.tracker.st.state, 'PAUSED')
        self.fusion.current = FakeFusion.make(p.x, p.y, math.degrees(p.yaw))
        for t in (5.05, 5.5, 5.95):                              # 恢复不到 1 s：还停着
            self.assertEqual(self.m.tick(t, 'NAVIGATION')[:2], (0.0, 0.0))
        self.assertEqual(self.m.tracker.st.state, 'PAUSED')
        s0 = self.m.tracker.st.s
        self.drive(6.1, 10.0)
        self.assertEqual(self.m.tracker.st.state, 'RUNNING')
        self.assertEqual(self.m.message, '定位恢复，自动继续')
        self.assertGreater(self.m.tracker.st.s, s0 + 1.0)

    def test_signal_loss_stops_in_place_with_notice_then_resumes(self):
        # 2026-10-01 用户规则：掉信号（融合报 rtk_fixed_timeout）不要瞎动，原地停；恢复后自动继续
        from line_mission import RTK_NOTICE_LOST
        self.running_rect()
        p = self.fusion.current
        lost = FakeFusion.make(p.x, p.y, math.degrees(p.yaw), quality='DEAD_RECKONING', usable=False)
        lost.reasons = ('heading_unavailable', 'gyro_only', 'rtk_fixed_timeout')
        self.fusion.current = lost
        self.assertEqual(self.m.tick(5.0, 'NAVIGATION')[:2], (0.0, 0.0))
        self.assertEqual(self.m.tracker.st.state, 'PAUSED')
        self.assertEqual(self.m.message, RTK_NOTICE_LOST)
        self.assertEqual(self.m.phase, 'RUNNING')                            # 作业没结束，只是停着等信号
        self.fusion.current = FakeFusion.make(p.x, p.y, math.degrees(p.yaw))
        self.drive(5.05, 10.0)
        self.assertEqual(self.m.tracker.st.state, 'RUNNING')
        self.assertEqual(self.m.message, '定位恢复，自动继续')

    def test_route_start_uses_learned_trim(self):
        self.m.integral0 = 0.05
        self.fusion.current = FakeFusion.make(-1.0, 0.0, 0.0)
        self.m.command({'cmd': 'route_start', 'route': RECT}, 0.0)
        self.m.tick(0.05, 'NAVIGATION')
        trim_line = -self.m.line_cfg.ki_per_m2 * 0.05
        self.assertAlmostEqual(self.m.tracker.status()['trim_kappa'], trim_line, places=4)

    def centre(self):
        """车身中心（后轴前 (车头 − 车尾)/2）。"""
        p, (front, rear, _) = self.fusion.current, self.m.body
        return p.x + 0.5 * (front - rear) * math.cos(p.yaw), p.y + 0.5 * (front - rear) * math.sin(p.yaw)

    def test_connected_points_route_runs_once_and_stops_on_last_point(self):
        # 操作员点的点按连线顺序走：先开到第 1 个点，再到第 2、3 个点，没连回起点就不绕圈
        path = dict(kind='path', points=[[3, 0], [10, 0], [10, 6]], closed=False, laps=3, cruise=0.5)
        self.fusion.current = FakeFusion.make(0.0, 0.0, 0.0)
        ok, msg = self.m.command({'cmd': 'route_start', 'route': path}, 0.0)
        self.assertTrue(ok, msg)
        st = self.m.status()['route']
        self.assertEqual((st['kind'], st['laps'], st['closed'], st['length']), ('path', 1, False, 13.0))
        self.assertEqual(self.m.status()['route_limits']['body'], [0.67, 0.08, 0.335])
        out, _ = self.drive(0.05, 200.0)
        self.assertEqual(self.m.message, '路线完成')
        x, y = self.centre()                                               # 车身中心停在终点后 5 cm
        self.assertLess(math.hypot(x - 10, y - 6.05), 0.05)
        self.assertLess(self.m.tracker.status()['stats']['max_m'], 0.05)    # 跟参考线的误差

    def test_connected_points_right_angle_is_fully_driven_over(self):
        # 直角：车身中心开过拐角 → 整车开出去 → 倒车一次 → 往前开上下一条边；车身中心压过全部连线，拐角点也压到
        path = dict(kind='path', points=[[0, 0], [6, 0], [6, 5]], closed=False, cruise=0.5)
        self.fusion.current = FakeFusion.make(-2.0, 0.0, 0.0)
        ok, msg = self.m.command({'cmd': 'route_start', 'route': path}, 0.0)
        self.assertTrue(ok, msg)
        self.assertIn('换向 2 次', msg)
        cover = self.m.status()['route']['cover']
        self.assertEqual((cover['cusps'], cover['corners']), (2, 1))
        trace, gears, t = [], [], 0.0
        while t < 200.0:
            out = self.m.tick(t, 'NAVIGATION')
            p = self.fusion.current
            if self.m.phase == 'RUNNING':
                gear = self.m.tracker.status()['gear']
                if not gears or gears[-1] != gear:
                    gears.append(gear)
            yaw = p.yaw + out[1] * 0.05
            self.fusion.current = FakeFusion.make(p.x + out[0] * 0.05 * math.cos(yaw),
                                                  p.y + out[0] * 0.05 * math.sin(yaw), math.degrees(yaw), speed=out[0])
            trace.append(self.centre())
            t += 0.05
            if self.m.phase == 'IDLE' and out[2] is False:
                break
        self.assertEqual(self.m.message, '路线完成')
        self.assertEqual(gears, ['forward', 'reverse', 'forward'])
        for i in range(0, 1101):                                             # 连线上每 1 cm 一点
            q = (i / 100.0, 0.0) if i <= 600 else (6.0, (i - 600) / 100.0)
            self.assertLess(min(math.hypot(q[0] - x, q[1] - y) for x, y in trace), 0.03, q)

    def test_connected_points_start_behind_turns_around(self):
        # 第一个点在车正后方：倒车掉头开过去
        self.fusion.current = FakeFusion.make(0.0, 0.0, 0.0)
        ok, msg = self.m.command({'cmd': 'route_start', 'route': dict(kind='path', points=[[-3, 0], [-9, 0]])}, 0.0)
        self.assertTrue(ok, msg)
        self.drive(0.05, 200.0)
        self.assertEqual(self.m.message, '路线完成')
        x, y = self.centre()
        self.assertLess(math.hypot(x + 9.05, y), 0.05)

    def test_connected_points_short_rows_need_no_long_sides(self):
        # 行距 0.8 m 的往返：掉头在拐角外面完成，不要求两边有长直线
        route = dict(kind='path', points=[[0, 0], [6, 0], [6, 0.8], [0, 0.8]], closed=False)
        self.m.command({'cmd': 'route_set', 'route': route}, 0.0)
        st = self.m.status()['route']
        self.assertIsNone(st['problem'])
        self.assertEqual((st['cover']['corners'], st['cover']['cusps']), (2, 4))
        self.fusion.current = FakeFusion.make(-2.0, 0.0, 0.0)
        self.assertTrue(self.m.command({'cmd': 'route_start'}, 0.0)[0])

    def test_antenna_position_is_reported_while_fresh(self):
        self.m.on_gnss(dict(kind='fix', t=1.0, lat=30.00002, lon=120.00001, height=20.0, sol='NARROW_FLOAT',
                            sigma_n=0.02, sigma_e=0.03, sigma_u=0.05, diff_age=1.0, sats=30))
        self.m.tick(1.5, 'IDLE')
        self.assertEqual(self.m.status()['antenna'], dict(x=1.0, y=2.0, sol='NARROW_FLOAT', sigma=0.03))
        self.m.tick(3.6, 'IDLE')
        self.assertIsNone(self.m.status()['antenna'])



class TestRtkJumps(Base):
    """定位跳动：固定解与融合先验（交给融合之前）差 3 cm 以上记一次，1 s 内连续的合并；诊断带实际曲率。"""

    def setUp(self):
        super().setUp()
        self.fusion.config = SimpleNamespace(antenna_xy=(0.345, 0.165))

    def fix(self, t, x, y, yaw_deg, left=0.0, sol='NARROW_INT'):
        a = math.radians(yaw_deg)
        ax, ay = self.fusion.config.antenna_xy
        lat, lon = self.fusion.to_geodetic(x + math.cos(a) * ax - math.sin(a) * (ay + left),
                                           y + math.sin(a) * ax + math.cos(a) * (ay + left))
        self.m.on_gnss(dict(kind='fix', t=t, lat=lat, lon=lon, height=20.0, sol=sol, sigma_n=0.01,
                            sigma_e=0.01, sigma_u=0.02, diff_age=0.5, sats=30))

    def test_sideways_jump_is_recorded_once_with_where_and_how_long(self):
        self.fusion.current = FakeFusion.make(5.0, 2.0, 90.0)
        for i in range(10):
            self.fix(1.0 + 0.1 * i, 5.0, 2.0, 90.0)                        # 与推算一致：不记
        self.m.tick(2.0, 'IDLE')
        self.assertEqual(self.m.status()['rtk_events'], [])
        for i, left in enumerate((0.02, 0.04, 0.06, 0.08, 0.05)):           # 1 s 内向左漂到 8 cm
            self.fix(2.1 + 0.1 * i, 5.0, 2.0, 90.0, left=left)
        self.fix(4.0, 5.0, 2.0, 90.0, left=0.05)                           # 隔 1.5 s 又偏：第二次
        self.m.tick(4.1, 'IDLE')
        st = self.m.status()
        first, second = st['rtk_events']
        self.assertAlmostEqual(first['d'], 0.08, places=6)
        self.assertAlmostEqual(first['left'], 0.08, places=6)
        self.assertEqual((first['x'], first['y']), (5.0, 2.0))
        self.assertAlmostEqual(first['lat'], 30.00002, places=9)
        self.assertAlmostEqual(first['lon'], 120.00005, places=9)
        self.assertAlmostEqual(first['dur'], 0.3, places=6)
        self.assertAlmostEqual(first['age_s'], 1.6, places=6)
        self.assertEqual((first['id'], second['id']), (2.2, 4.0))
        self.assertAlmostEqual(second['d'], 0.05, places=6)
        self.assertAlmostEqual(st['diag']['rtk_left_m'], 0.05, places=6)
        self.assertAlmostEqual(st['diag']['rtk_along_m'], 0.0, places=6)
        self.assertEqual(st['diag']['jumps'], 2)

    def test_jump_count_keeps_counting_past_the_kept_records(self):
        """记录只留最近 50 次，但次数一直往上数：作业记录按首末相减（2026-09-30 满 50 后每次作业都算成 0）。"""
        self.fusion.current = FakeFusion.make(5.0, 2.0, 90.0)
        for i in range(60):
            self.fix(10.0 + 2.0 * i, 5.0, 2.0, 90.0, left=0.05)             # 隔 2 s 偏一次：每次单独算
        self.m.tick(200.0, 'IDLE')
        self.assertEqual(self.m.status()['diag']['jumps'], 60)

    def test_not_recorded_while_fusion_is_not_fixed(self):
        self.fusion.current = FakeFusion.make(5.0, 2.0, 0.0, quality='RTK_FLOAT', usable=False)
        self.fix(1.0, 5.0, 2.0, 0.0, left=0.3)
        self.m.tick(1.1, 'IDLE')
        self.assertEqual(self.m.status()['rtk_events'], [])
        self.assertAlmostEqual(self.m.status()['diag']['rtk_left_m'], 0.3, places=6)

    def test_float_fixes_are_not_compared(self):
        self.fusion.current = FakeFusion.make(5.0, 2.0, 0.0)
        self.fix(1.0, 5.0, 2.0, 0.0, left=0.3, sol='NARROW_FLOAT')
        self.m.tick(1.1, 'IDLE')
        self.assertIsNone(self.m.status()['diag']['rtk_left_m'])
        self.assertEqual(self.m.status()['rtk_events'], [])

    def test_diag_reports_actual_curvature_from_fused_yaw_rate(self):
        p = FakeFusion.make(1.0, 1.0, 0.0, speed=0.5)
        p.yaw_rate = 0.1
        self.fusion.current = p
        self.m.tick(1.0, 'IDLE')
        d = self.m.status()['diag']
        self.assertAlmostEqual(d['kappa_act'], 0.2, places=9)
        self.assertAlmostEqual(d['yaw_rate'], 0.1, places=9)
        self.fusion.current = FakeFusion.make(1.0, 1.0, 0.0, speed=0.01)       # 几乎不动：不算曲率
        self.m.tick(1.05, 'IDLE')
        self.assertIsNone(self.m.status()['diag']['kappa_act'])


if __name__ == '__main__':
    unittest.main()
