#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""走直线 + 真实融合模块（gnss_fusion）的闭环测试，不需要 ROS 和硬件。

- 仿真层：radar_system/tools/line_sim.py 的 run_fused（原始传感器 → Um982Stream → 融合 → 控制器）。
- 节点层：合成的 UM982 日志经 Um982Stream 变成 /gnss/raw 消息，和 IMU、里程计一起喂给 line_node；
  line_node 里是真实的 LineMission + GnssFusion，经 /line/command 记点、开始，发出的指令驱动底盘模型。
数字来自离线仿真，不代表实车；实车以 ~/rtk/runs/*.csv 为准。
"""
import csv
import dataclasses
import json
import math
import os
import random
import sys
import tempfile
import unittest
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system', 'tools'))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, ROOT)

import ros_stubs  # noqa: E402

ros_stubs.install()
if not hasattr(sys.modules['sensor_msgs.msg'], 'Imu'):
    sys.modules['sensor_msgs.msg'].Imu = ros_stubs._simple('Imu')

import gnss_fusion  # noqa: E402
import line_node  # noqa: E402
import line_sim  # noqa: E402
import um982  # noqa: E402
from line_mission import LineMission  # noqa: E402

SEEDS = (1, 2, 3, 4)


def worst(name, key, **extra):
    kw = dict(line_sim.FUSED_SCENARIOS[name], **extra)
    return max(line_sim.run_fused(name, seed=s, **kw)[key] for s in SEEDS)


class TestFusedSimulation(unittest.TestCase):
    """原始传感器 + 真实融合的场景回归（阈值留了余量，8 个种子的实测见 docs）。"""

    def test_nominal_and_fast(self):
        self.assertLess(worst('nominal', 'max_cm'), 1.5)
        self.assertLess(worst('nominal', 'rms_cm'), 1.0)
        self.assertLess(worst('fast', 'max_cm'), 2.0)
        for s in SEEDS:
            r = line_sim.run_fused('nominal', seed=s)
            self.assertEqual(r['state'], 'DONE')
            self.assertLess(abs(r['end_err_cm']), 4.0)
            self.assertLess(r['fusion_yaw_max_deg'], 0.5)

    def test_disturbances_stay_within_two_centimetres(self):
        for name in ('offset_start', 'trim_2deg', 'noisy', 'rough', 'mount_1deg', 'heading_loss'):
            with self.subTest(name):
                self.assertLess(worst(name, 'max_cm'), 2.5)

    def test_trim_learned_through_fusion(self):
        r = line_sim.run_fused('trim_2deg', **line_sim.FUSED_SCENARIOS['trim_2deg'])
        self.assertAlmostEqual(r['trim_learned_deg'], -2.0, delta=0.4)
        again = line_sim.run_fused('trim_2deg', seed=2, integral0=r['integral'], **line_sim.FUSED_SCENARIOS['trim_2deg'])
        self.assertLess(again['max_cm'], 1.5)

    def test_rtk_loss_pauses_within_half_a_metre(self):
        # 融合在丢固定解后还允许推算 0.5 s（fixed_timeout_s），之后不可用 → 控制器暂停、停车
        for s in SEEDS:
            r = line_sim.run_fused('outage', seed=s, **line_sim.FUSED_SCENARIOS['outage'])
            self.assertEqual(r['state'], 'PAUSED')
            self.assertIsNotNone(r['paused_stop_s'])
            self.assertLess(r['stop_x'] - r['outage_x'], 0.5)
            self.assertLess(r['max_cm'], 1.5)


# =============================================================================
# 节点层：line_node + 真实融合
# =============================================================================

class FakeMotion:
    """运动仲裁替身：申请 NAVIGATION 立刻批准，取消后回到 IDLE；记下发出的指令。"""

    def __init__(self):
        self.mode, self.selects, self.published = 'IDLE', [], []

    def fresh(self):
        return True

    def active(self):
        return self.mode == 'NAVIGATION'

    @property
    def state(self):
        return {'mode': self.mode}

    def select(self, enabled=True):
        self.selects.append(enabled)
        self.mode = 'NAVIGATION' if enabled else 'IDLE'
        return object()

    def publish(self, vx, wz):
        self.published.append((vx, wz))
        return True


class Rig:
    """一台模拟车：底盘模型 + 接收机/IMU/轮速；数据按车上的路径进 line_node。

    UM982 日志 → Um982Stream（GPS 时间换算）→ JSON → line_node.on_gnss；IMU、里程计按收到时刻进
    on_imu / on_odom；每 50 ms 调一次 line_node.control()，NAVIGATION 期间把节点发的指令交给底盘。
    """

    def __init__(self, tmp, *, x=0.0, seed=3, mount_deg=1.5, receiver_kw=None):
        self.t = 0.0
        fusion = gnss_fusion.GnssFusion(gnss_fusion.FusionConfig(mount_offset_deg=mount_deg))
        self.mission = LineMission(fusion, gnss_fusion, state_path=os.path.join(tmp, 'line_state.json'),
                                   log_dir=os.path.join(tmp, 'runs'))
        self.node = line_node.LineNode(self.mission, clock=lambda: self.t)
        self.motion = self.node.motion = FakeMotion()
        self.car = line_sim.Chassis()
        self.car.x = x
        rng = random.Random(seed)
        self.receiver = line_sim.Receiver(rng, mount_deg=mount_deg, **(receiver_kw or {}))
        self.sensors = line_sim.MotionSensors(rng)
        self.stream = um982.Um982Stream()
        self.pending = []
        self.next_sample = 0.0
        self.manual = (0.0, 0.0)
        self.rows = []

    def cmd(self, **kw):
        self.node.on_command(SimpleNamespace(data=json.dumps(kw)))
        return self.node.last_result

    def step(self):
        t, car = self.t, self.car
        self.sensors.record(t, car)
        if t >= self.next_sample - 1e-9:
            self.next_sample += line_sim.SAMPLE_DT
            self.node.on_imu(SimpleNamespace(angular_velocity=SimpleNamespace(z=self.sensors.imu(t))))
            v, wz = self.sensors.wheel(t)
            self.node.on_odom(SimpleNamespace(twist=SimpleNamespace(twist=SimpleNamespace(
                linear=SimpleNamespace(x=v), angular=SimpleNamespace(z=wz)))))
            self.pending += self.receiver.measure(t, car)
            self.pending.sort()
            sent = len(self.motion.published)
            self.node.control()
            if self.motion.active():
                if len(self.motion.published) > sent:
                    car.command(t, *self.motion.published[-1])
            elif len(self.motion.published) > sent:
                car.command(t, 0.0, 0.0)            # 交还控制权时的停车指令
            else:
                car.command(t, *self.manual)
            self.rows.append(dict(t=t, x=car.x, y=car.y, phase=self.mission.phase,
                                  state=self.mission.tracker.st.state))
        while self.pending and self.pending[0][0] <= t:
            t_rx, data = self.pending.pop(0)
            for m in self.stream.feed(data, t_rx):
                self.node.on_gnss(SimpleNamespace(data=json.dumps(m, allow_nan=False)))
        car.step(t, line_sim.DT)
        self.t = round(t + line_sim.DT, 9)

    def wait(self, seconds, until=None):
        end = self.t + seconds
        while self.t < end:
            self.step()
            if until is not None and until():
                return True
        return until is None

    def drive_to(self, x_target, speed=0.4):
        """操作员手动开车：沿直线开到 x_target 停稳。"""
        while True:
            dx = x_target - self.car.x
            if abs(dx) < 0.01:
                break
            v = math.copysign(max(0.05, min(speed, 1.2 * abs(dx))), dx)
            self.manual = (v, 0.0)
            self.wait(0.05)
        self.manual = (0.0, 0.0)
        self.wait(1.5)


class TestLineNodeWithRealFusion(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.tmp.cleanup()

    def mark_and_back_up(self, rig, length=7.0):
        rig.wait(4.0)                              # 静止：融合初始化航向、估陀螺零偏
        res = rig.cmd(cmd='mark', point='A', id=1)
        self.assertEqual((res['ok'], res['message'], res['id']), (True, '已记 A 点', 1))
        rig.drive_to(length)
        self.assertTrue(rig.cmd(cmd='mark', point='B')['ok'])
        rig.drive_to(-2.0)                         # 倒回 A 点后方 2 m 助跑

    def test_mark_drive_and_reach_b(self):
        rig = Rig(self.tmp.name)
        self.mark_and_back_up(rig)
        res = rig.cmd(cmd='start', cruise=0.5, id=9)
        self.assertEqual((res['ok'], res['id']), (True, 9))
        self.assertTrue(rig.wait(40.0, until=lambda: rig.mission.phase == 'IDLE' and rig.motion.selects[-1:] == [False]))
        self.assertEqual(rig.mission.message, '到达 B 点')
        self.assertEqual(rig.motion.selects, [True, False])        # 申请一次，结束后交还
        rig.wait(1.0)
        self.assertLess(abs(rig.car.x - 7.0), 0.05)
        run = [r for r in rig.rows if r['state'] == 'RUNNING' and 1.0 <= r['x'] <= 6.5]
        self.assertGreater(len(run), 80)
        self.assertLess(max(abs(r['y']) for r in run), 0.02)       # 真值横偏（含记点误差）
        # 运行记录和学到的零位
        runs = os.listdir(os.path.join(self.tmp.name, 'runs'))
        self.assertEqual(len(runs), 1)
        with open(os.path.join(self.tmp.name, 'runs', runs[0])) as fh:
            rows = list(csv.DictReader(fh))
        self.assertTrue(all(r['quality'] == 'RTK_FIXED' for r in rows))
        with open(os.path.join(self.tmp.name, 'line_state.json')) as fh:
            self.assertIn('integral', json.load(fh))
        # /line/status 能直接 JSON 化，位姿来自真实融合
        rig.node.publish_status()
        st = json.loads(rig.node.pub_status.sent[-1].data)
        self.assertEqual((st['phase'], st['pose']['quality'], st['pose']['usable']), ('IDLE', 'RTK_FIXED', True))
        self.assertAlmostEqual(st['points']['B'][0] - st['points']['A'][0], 7.0, delta=0.03)

    def test_rtk_loss_stops_then_resumes_by_itself(self):
        # 起步后约 6 s 进入 1 s 的失锁区（记点、倒车这一段约 36 s）
        rig = Rig(self.tmp.name, receiver_kw=dict(outage=(1e9, 1e9)))
        self.mark_and_back_up(rig)
        # 0.5 m/s 标定的停车距离（不带 cruise 时默认 0.8，失锁后要多滑一段）
        self.assertTrue(rig.cmd(cmd='start', cruise=0.5)['ok'])
        rig.wait(6.0)
        self.assertEqual(rig.mission.phase, 'RUNNING')
        rig.receiver.outage = (rig.t, rig.t + 1.0)
        x_lost = rig.car.x
        self.assertTrue(rig.wait(3.0, until=lambda: rig.mission.tracker.st.state == 'PAUSED' and abs(rig.car.v) < 1e-3))
        self.assertLess(rig.car.x - x_lost, 0.5)
        # 固定解恢复、定位连续可用 1 s 后自己接着走（2026-09-27 起不再等操作员再按开始）
        self.assertTrue(rig.wait(5.0, until=lambda: rig.mission.tracker.st.state == 'RUNNING'))
        self.assertEqual(rig.mission.message, '定位恢复，自动继续')
        self.assertTrue(rig.wait(30.0, until=lambda: rig.mission.phase == 'IDLE'))
        self.assertEqual(rig.mission.message, '到达 B 点')

    def test_heading_float_all_the_way_holds_on_gyro_and_reaches_b(self):
        # 起步前测向就没解（副天线仍是固定解），全程靠陀螺 / 行驶轨迹维持航向。融合的航向门槛调到 0.06°（仿真里约 0.004°/s 地涨），
        # 让「航向不确定」很快出现：两根天线都固定就不停车，一直开到 B
        rig = Rig(self.tmp.name)
        self.mark_and_back_up(rig)
        f = rig.mission.fusion
        f.config = dataclasses.replace(f.config, control_sigma_yaw_deg=0.06)
        rig.receiver.heading_outage = (rig.t, 1e9)
        rig.wait(0.5)
        self.assertTrue(rig.cmd(cmd='start')['ok'])
        start, held = len(rig.rows), 0
        while rig.mission.phase != 'IDLE' and rig.t < 200:
            rig.wait(0.05)
            held += 'gyro_hold' in tuple(getattr(rig.mission.pose, 'reasons', ()) or ())
        self.assertEqual(rig.mission.message, '到达 B 点')
        self.assertGreater(held, 20)                                 # 确实用上了陀螺维持（> 1 s）
        self.assertNotIn('PAUSED', {r['state'] for r in rig.rows[start:]})

    def test_indoor_repeater_heading_is_rejected(self):
        # 室内转发器：两根天线收到同一个信号，基线只有几毫米 → 双天线航向无效，不能记点
        rig = Rig(self.tmp.name, receiver_kw=dict(baseline_m=0.004))
        rig.wait(4.0)
        self.assertFalse(rig.stream.status(rig.t)['heading_ok'])
        res = rig.cmd(cmd='mark', point='A')
        self.assertEqual((res['ok'], res['message']), (False, 'pose_unusable：heading_uninitialized'))
        rig.node.publish_status()
        st = json.loads(rig.node.pub_status.sent[-1].data)
        self.assertEqual(st['pose']['quality'], 'INVALID')
        self.assertIn('heading_uninitialized', st['pose']['reasons'])


if __name__ == '__main__':
    unittest.main()
