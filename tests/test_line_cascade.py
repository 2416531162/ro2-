#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""差速车定点画线 radar_system/line_cascade.py（IMU 航向内环 + RTK 横偏外环）与 LineMission 接入：
纠偏方向、外环限幅、积分条件、暂停 / 中止 / 到 B、画线默认车速；实测底盘模型上的闭环仿真（tests/gen5_line_sim.py）。"""
import math
import os
import sys
import tempfile
import unittest
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, ROOT)

import gen5_line_sim as sim  # noqa: E402
from line_cascade import DEG, NOTICE_HOLD, NOTICE_PAUSED, CascadeConfig, ImuLineTracker  # noqa: E402
from line_mission import LINE_CRUISE_MPS, LineMission  # noqa: E402
from line_tracker import LineConfig, LineTracker  # noqa: E402
from test_line_mission import FakeFusion  # noqa: E402

IMU_ZERO = 1.3          # IMU 偏航与 ENU 偏航差一个任意角（HDA536T 上电为 0）


def pose(x, y, yaw_deg, speed=0.0, usable=True):
    return SimpleNamespace(x=x, y=y, yaw=yaw_deg * DEG, speed=speed, yaw_rate=0.0, usable_for_control=usable)


class Rig:
    """直接驱动 ImuLineTracker：线 (0,0)→(length,0)，位姿、IMU 由测试给（IMU 偏航 = 真偏航 + IMU_ZERO）。"""

    def __init__(self, length=20.0, **cfg):
        self.tr = ImuLineTracker(CascadeConfig(**cfg), sim.GEN5)
        self.tr.set_line((0.0, 0.0), (length, 0.0))
        self.t = 0.0

    def tick(self, p, imu=True, dt=0.05, imu_yaw=None):
        """imu_yaw 给了就用它当真偏航（融合航向跳了、IMU 没变）。"""
        self.t = round(self.t + dt, 6)
        yaw = p.yaw if imu_yaw is None else imu_yaw
        imu_sample = (self.t, yaw + IMU_ZERO, 0.0) if imu else None
        self.tr.sync(self.t, imu_sample if imu else self.tr._imu)
        return self.tr.update(p, dt)

    def start(self, p):
        self.tr.sync(self.t, (self.t, p.yaw + IMU_ZERO, 0.0))
        return self.tr.start(p)

    def settle(self, p, lever=None, imu_yaw=None):
        """开到起步阶段结束（对准、加速完再过 2 s），之后才判航向跳变。"""
        for _ in range(400):
            self.tick(p, imu_yaw=imu_yaw)
            if lever is not None:
                self.tr._lever = lever
            if not self.tr.status()['hold']['start_phase']:
                return
        raise AssertionError('起步阶段一直没结束')


class TestOuterLoop(unittest.TestCase):

    def test_turns_back_toward_the_line(self):
        """喷头在线左（e > 0）→ 目标航向偏右（δ < 0）→ 往右转（wz < 0）；在线右反过来。"""
        for y, sign in ((0.05, -1.0), (-0.05, 1.0)):
            with self.subTest(y=y):
                rig = Rig()
                p = pose(1.0, y, 0.0, speed=0.5)
                self.assertTrue(rig.start(p))
                for _ in range(20):
                    vx, wz = rig.tick(p)
                self.assertGreater(vx, 0.0)
                self.assertEqual(math.copysign(1.0, rig.tr._bias), sign)
                self.assertEqual(math.copysign(1.0, wz), sign)
                st = rig.tr.status()
                self.assertAlmostEqual(st['e'], y, places=6)          # 横偏按喷头：车头朝线方向时与后轴相同
                self.assertEqual(st['hold']['state'], 'running')

    def test_nozzle_point_sets_the_offset(self):
        """横偏量的是喷头（后轴前 0.30 m）：车头偏 10° 时比后轴多偏 0.30·sin 10°。"""
        rig = Rig()
        e, he, s = rig.tr.geometry_of(1.0, 0.0, 10 * DEG)
        self.assertAlmostEqual(e, 0.30 * math.sin(10 * DEG))
        self.assertAlmostEqual(s, 1.0)                                  # 沿线距离按记点的点（后轴中心）
        self.assertAlmostEqual(he, 10 * DEG)

    def test_bias_is_limited_to_three_degrees(self):
        rig = Rig()
        p = pose(1.0, 0.3, 0.0, speed=0.5)
        rig.start(p)
        for _ in range(100):
            rig.tick(p)
        self.assertAlmostEqual(rig.tr._bias, -3.0 * DEG)
        # 目标（IMU 系）= 线方向 − 偏移 + δ：转到位后离 IMU 偏航正好 3°
        self.assertAlmostEqual(math.degrees(rig.tr._target - IMU_ZERO), -3.0, places=6)

    def test_integral_only_near_the_line_and_moving_forward(self):
        cfg = CascadeConfig()
        for y, step_x, grows in ((0.05, 0.04, True), (0.15, 0.04, False), (0.05, 0.0, False), (0.05, -0.04, False)):
            with self.subTest(y=y, step_x=step_x):
                rig = Rig()
                x = 1.0
                rig.start(pose(x, y, 0.0, speed=0.5))
                for _ in range(40):
                    x += step_x
                    rig.tick(pose(x, y, 0.0, speed=0.5))
                self.assertEqual(rig.tr.st.integral != 0.0, grows)
                if grows:
                    self.assertAlmostEqual(rig.tr.st.integral, y * 0.04 * 39, places=6)     # ∫e ds（第一帧没有 ds）
        rig = Rig()                                                     # 贴线走很久也有上限
        x = 0.0
        rig.start(pose(x, 0.09, 0.0, speed=0.5))
        for _ in range(4000):
            x += 0.004
            rig.tick(pose(x, 0.09, 0.0, speed=0.5))
        self.assertAlmostEqual(cfg.k_i * rig.tr.st.integral, cfg.integ_max_deg * DEG)

    def test_imu_offset_learns_fused_minus_imu(self):
        """偏移 = 融合航向 − IMU 偏航：IMU 零点差多少都行，目标换算回 IMU 系。"""
        rig = Rig()
        p = pose(1.0, 0.0, 2.0, speed=0.5)
        rig.start(p)
        rig.tick(p)
        self.assertAlmostEqual(rig.tr._offset, -IMU_ZERO, places=6)


class TestStopsAndPauses(unittest.TestCase):

    def running(self):
        rig = Rig()
        p = pose(1.0, 0.0, 0.0, speed=0.5)
        self.assertTrue(rig.start(p))
        for _ in range(10):
            rig.tick(p)
        return rig, p

    def test_pose_loss_pauses_and_needs_a_new_start(self):
        rig, p = self.running()
        self.assertEqual(rig.tick(pose(1.0, 0.0, 0.0, usable=False)), (0.0, 0.0))
        self.assertEqual((rig.tr.st.state, rig.tr.st.reason), ('PAUSED', 'pose_unusable'))
        self.assertEqual(rig.tick(p), (0.0, 0.0))                       # 定位恢复也不自己走
        self.assertEqual(rig.tr.st.reason, 'pose_recovered_waiting_operator')
        run = rig.tr._run
        self.assertTrue(rig.start(p))                                   # 再按开始：内环重新开始
        self.assertEqual(rig.tr._run, run + 1)
        vx, _ = rig.tick(p)
        self.assertGreater(vx, 0.0)
        self.assertEqual(rig.tr.st.state, 'RUNNING')

    def test_far_off_line_aborts(self):
        for p in (pose(1.0, 0.55, 0.0), pose(1.0, 0.0, 50.0)):
            with self.subTest(y=p.y, yaw=p.yaw):
                rig, _ = self.running()
                self.assertEqual(rig.tick(p), (0.0, 0.0))
                self.assertEqual((rig.tr.st.state, rig.tr.st.reason), ('ABORTED', 'off_line'))

    def test_imu_dropout_aborts(self):
        rig, p = self.running()
        for _ in range(4):                                              # 0.2 s 内还在开
            self.assertGreater(rig.tick(p, imu=False)[0], 0.0)
        self.assertEqual(rig.tick(p, imu=False), (0.0, 0.0))
        self.assertEqual((rig.tr.st.state, rig.tr.st.reason), ('ABORTED', 'imu_stale'))

    def test_imu_jump_aborts_as_deviation(self):
        """IMU 偏航突然跳了 10°（重启归零、被推歪）：内环偏离目标超过 5° → 中止 imu_deviation。"""
        rig, p = self.running()
        rig.t += 0.05
        rig.tr.sync(rig.t, (rig.t, p.yaw + IMU_ZERO + 10 * DEG, 0.0))
        self.assertEqual(rig.tr.update(p, 0.05), (0.0, 0.0))
        self.assertEqual((rig.tr.st.state, rig.tr.st.reason), ('ABORTED', 'imu_deviation'))

    def test_start_needs_fresh_imu_and_same_limits_as_line_tracker(self):
        rig = Rig()
        rig.tr.sync(0.0, None)
        self.assertEqual(rig.tr.check_start(pose(0.0, 0.0, 0.0)), (False, 'no_imu'))
        rig.tr.sync(1.0, (0.5, IMU_ZERO, 0.0))                          # 0.5 s 前的也不算
        self.assertEqual(rig.tr.check_start(pose(0.0, 0.0, 0.0)), (False, 'no_imu'))
        rig.tr.sync(1.0, (0.99, IMU_ZERO, 0.0))
        for p, why in ((pose(0.0, 0.6, 0.0), 'too_far_from_line'), (pose(0.0, 0.0, 35.0), 'heading_not_aligned'),
                       (pose(-3.5, 0.0, 0.0), 'too_far_behind_a'), (pose(19.6, 0.0, 0.0), 'already_past_b'),
                       (pose(0.0, 0.0, 0.0, usable=False), 'pose_unusable')):
            self.assertEqual(rig.tr.check_start(p), (False, why))
        self.assertEqual(rig.tr.check_start(pose(-2.9, 0.1, 25.0)), (True, ''))

    def test_operator_stop(self):
        rig, p = self.running()
        self.assertEqual(rig.tr.stop(), (0.0, 0.0))
        self.assertEqual((rig.tr.st.state, rig.tr.st.reason), ('ABORTED', 'operator'))
        self.assertEqual(rig.tr.hold.state, 'stopped')
        self.assertEqual(rig.tick(p), (0.0, 0.0))

    def test_status_keys_match_line_tracker(self):
        rig, _ = self.running()
        old = LineTracker(LineConfig(), sim.GEN5)
        old.set_line((0.0, 0.0), (20.0, 0.0))
        self.assertLessEqual(set(old.status()), set(rig.tr.status()))


class TestHeadingJump(unittest.TestCase):
    """2026-09-30 用户规则：IMU 航向 5 s 内肯定没问题。融合（双天线）航向突然跳了、IMU 没变：不跟着动，按 IMU 维持；
    5 s 还没恢复就停车，航向恢复后自动继续；恢复后重新计 5 s。"""

    def test_hysteresis_pause_after_five_seconds_and_resume_only_when_recovered(self):
        rig = Rig(length=40.0)
        p = pose(1.0, 0.0, 0.0, speed=0.5)
        rig.start(p)
        rig.settle(p)
        offset = rig.tr._offset
        jumped = pose(1.0, 0.0, 0.6, speed=0.5)                        # 融合航向跳 0.6°，IMU 没变
        for _ in range(100):                                            # 5 s 内照常开、偏移不学
            self.assertGreater(rig.tick(jumped, imu_yaw=0.0)[0], 0.0)
            self.assertEqual(rig.tr.notice(), NOTICE_HOLD)
        self.assertEqual(rig.tr._offset, offset)
        self.assertAlmostEqual(rig.tr.status()['hold']['heading_diff_deg'], 0.6, places=3)
        rig.tick(jumped, imu_yaw=0.0)
        self.assertEqual(rig.tick(jumped, imu_yaw=0.0), (0.0, 0.0))  # 超过 5 s：停车
        self.assertEqual((rig.tr.st.state, rig.tr.st.reason), ('PAUSED', 'heading_jump'))
        self.assertEqual(rig.tr.notice(), NOTICE_PAUSED)
        self.assertEqual(rig.tr.hold.state, 'stopped')
        for yaw_deg in (0.6, 0.4):                                      # 回到 0.3° 以内才算恢复
            p2 = pose(1.0, 0.0, yaw_deg, speed=0.0)
            rig.tick(p2, imu_yaw=0.0)
            self.assertFalse(rig.tr.start(p2))
            self.assertEqual((rig.tr.st.state, rig.tr.st.reason), ('PAUSED', 'heading_jump'))
        back = pose(1.0, 0.0, 0.2, speed=0.0)
        rig.tick(back, imu_yaw=0.0)
        self.assertTrue(rig.tr.start(back))
        self.assertIsNone(rig.tr.notice())

    def test_running_hysteresis(self):
        """差超过 0.5° 进跳变，回到 0.3° 以内才出来：0.4° 还算跳变（偏移不学），0.2° 恢复、偏移接着学。"""
        rig = Rig(length=40.0)
        p = pose(1.0, 0.0, 0.0, speed=0.5)
        rig.start(p)
        rig.settle(p)
        rig.tick(pose(1.0, 0.0, 0.4, speed=0.5), imu_yaw=0.0)             # 0.4° 还没到 0.5°：不算跳变
        self.assertIsNone(rig.tr.notice())
        rig.tick(pose(1.0, 0.0, 0.6, speed=0.5), imu_yaw=0.0)
        self.assertEqual(rig.tr.notice(), NOTICE_HOLD)
        offset = rig.tr._offset
        rig.tick(pose(1.0, 0.0, 0.4, speed=0.5), imu_yaw=0.0)             # 跳变中 0.4° 还没恢复
        self.assertEqual((rig.tr.notice(), rig.tr._offset), (NOTICE_HOLD, offset))
        rig.tick(pose(1.0, 0.0, 0.2, speed=0.5), imu_yaw=0.0)
        self.assertIsNone(rig.tr.notice())
        self.assertNotEqual(rig.tr._offset, offset)

    def test_lateral_error_ignores_the_heading_jump(self):
        """融合航向跳 3°（融合位置也跟着按杠杆臂偏）、IMU 没变：量出的喷头横偏还是真值。"""
        rig = Rig(length=40.0)
        ax, ay = sim.ANTENNA_XY
        p = pose(1.0, 0.02, 0.0, speed=0.5)
        rig.start(p)
        rig.tr._lever = sim.ANTENNA_XY
        rig.settle(p, lever=sim.ANTENNA_XY)
        bad = 3 * DEG                                                   # 后轴 = 天线 − R(bad)·杠杆臂
        jumped = pose(1.0 + ax - (ax * math.cos(bad) - ay * math.sin(bad)),
                      0.02 + ay - (ax * math.sin(bad) + ay * math.cos(bad)), 3.0, speed=0.5)
        rig.t = round(rig.t + 0.05, 6)
        rig.tr.sync(rig.t, (rig.t, IMU_ZERO, 0.0), sim.ANTENNA_XY)
        rig.tr.update(jumped, 0.05)
        self.assertEqual(rig.tr.notice(), NOTICE_HOLD)
        self.assertAlmostEqual(rig.tr.st.e, 0.02, delta=0.0005)
        self.assertAlmostEqual(rig.tr.st.heading_err, 0.0, delta=0.05 * DEG)

    def test_recovery_restarts_the_five_second_count(self):
        rig = Rig(length=60.0)
        p = pose(1.0, 0.0, 0.0, speed=0.5)
        rig.start(p)
        rig.settle(p)
        for _ in range(3):                                              # 跳 4 s、恢复一下，再跳 4 s……都不停车
            for _ in range(80):
                rig.tick(pose(1.0, 0.0, 1.0, speed=0.5), imu_yaw=0.0)
            rig.tick(pose(1.0, 0.0, 0.1, speed=0.5), imu_yaw=0.0)
            self.assertIsNone(rig.tr.notice())
        self.assertEqual(rig.tr.st.state, 'RUNNING')

    def test_start_phase_ignores_heading_differences(self):
        """起步（对准、加速、再 2 s）不判跳变（用户：起步是万向轮，不管 IMU 的变化）；偏移照学。
        起步完了从头重新平均偏移，之后才按 5 s 规则判。"""
        rig = Rig(length=40.0)
        p = pose(1.0, 0.0, 0.0, speed=0.0)
        rig.start(p)
        offsets = []
        for i in range(30):                                             # 起步 1.5 s：融合与 IMU 差 1°
            rig.tick(pose(1.0, 0.0, 1.0, speed=0.3), imu_yaw=0.0)
            self.assertIsNone(rig.tr.notice())
            self.assertTrue(rig.tr.status()['hold']['start_phase'])
            offsets.append(rig.tr._offset)
        self.assertGreater(offsets[-1], offsets[0])                     # 偏移照学
        rig.settle(pose(1.0, 0.0, 1.0, speed=0.5), imu_yaw=0.0)
        self.assertAlmostEqual(rig.tr._offset, 1 * DEG - IMU_ZERO, places=4)   # 起步完重新平均：学到的就是 1°
        self.assertEqual(rig.tr.st.state, 'RUNNING')
        rig.tick(pose(1.0, 0.0, 1.8, speed=0.5), imu_yaw=0.0)             # 之后再差 0.8°：算跳变
        self.assertEqual(rig.tr.notice(), NOTICE_HOLD)

    def test_resume_after_pause_is_a_new_start_phase(self):
        """定位丢了暂停、恢复后接着走：又是一次起步（万向轮），起步阶段照样不判跳变。"""
        rig = Rig(length=40.0)
        p = pose(1.0, 0.0, 0.0, speed=0.5)
        rig.start(p)
        rig.settle(p)
        rig.tick(pose(1.0, 0.0, 0.0, usable=False))
        self.assertEqual(rig.tr.st.state, 'PAUSED')
        still = pose(1.0, 0.0, 0.0, speed=0.0)
        rig.tick(still)
        self.assertTrue(rig.start(still))
        rig.tick(pose(1.0, 0.0, 1.0, speed=0.1), imu_yaw=0.0)
        self.assertTrue(rig.tr.status()['hold']['start_phase'])
        self.assertIsNone(rig.tr.notice())

    def test_big_start_turn_with_imu_scale_error_does_not_stop_the_car(self):
        """起步车头差 20–29°、IMU 刻度误差 ±4%（转 29° 差 1.2°）：起步转完不会被当成航向跳变。"""
        for start in ((-2.0, 0.3, 20.0), (-2.0, 0.0, 29.0), (-2.0, -0.3, -25.0)):
            for scale in (0.04, -0.04):
                trace = []
                r = sim.run('cascade', v=0.8, seed=1, length=30.0, start=start, imu_scale=scale, trace=trace)
                with self.subTest(start=start, scale=scale):
                    self.assertEqual((r['state'], r['message']), ('DONE', '到达 B 点'))
                    self.assertNotIn(NOTICE_HOLD, {row[4] for row in trace})
                    self.assertNotIn('PAUSED', {row[2] for row in trace})

    def test_position_is_recomputed_with_the_imu_reference_heading(self):
        """融合航向错了 3°：融合按错的航向从天线倒推的后轴偏了（杠杆臂），按参考航向改算回真值。"""
        rig = Rig()
        rig.tr._lever = sim.ANTENNA_XY
        ax, ay = sim.ANTENNA_XY
        true_yaw, bad = 10 * DEG, 13 * DEG
        antenna = (2.0 + ax * math.cos(true_yaw) - ay * math.sin(true_yaw),
                   1.0 + ax * math.sin(true_yaw) + ay * math.cos(true_yaw))
        fused = pose(antenna[0] - ax * math.cos(bad) + ay * math.sin(bad),
                     antenna[1] - ax * math.sin(bad) - ay * math.cos(bad), 13.0)
        self.assertGreater(math.hypot(fused.x - 2.0, fused.y - 1.0), 0.015)
        x, y = rig.tr._position(fused, true_yaw)
        self.assertAlmostEqual(x, 2.0, places=9)
        self.assertAlmostEqual(y, 1.0, places=9)

    def test_short_jump_does_not_move_the_car(self):
        """跳 3° 持续 2 s：app 显示「按 IMU 维持」，车照常开、不停；那一段喷头横偏与没跳时差不到 3 mm
        （偏移照学的话 2 s 就把约 0.6° 学进目标航向，车会真的拐过去）。"""
        def window_max(trace):
            return max(abs(row[6]) for row in trace if 19.0 <= row[0] <= 30.0)
        for seed in (1, 2):
            base, jump = [], []
            sim.run('cascade', v=0.8, seed=seed, trace=base)
            r = sim.run('cascade', v=0.8, seed=seed, heading_jump=(20.0, 22.0, 3.0), trace=jump)
            with self.subTest(seed=seed):
                self.assertEqual((r['state'], r['message']), ('DONE', '到达 B 点'))
                messages = {row[4] for row in jump if 20.1 <= row[0] < 21.9}
                self.assertEqual(messages, {NOTICE_HOLD})
                self.assertEqual({row[4] for row in jump if 22.1 <= row[0] < 30.0}, {'行驶中'})
                self.assertNotIn('PAUSED', {row[2] for row in jump})
                self.assertLess(window_max(jump), window_max(base) + 0.003)

    def test_long_jump_stops_after_five_seconds_and_goes_on_when_heading_recovers(self):
        trace = []
        r = sim.run('cascade', v=0.8, seed=1, heading_jump=(20.0, 40.0, 3.0), trace=trace)
        self.assertEqual((r['state'], r['message']), ('DONE', '到达 B 点'))
        paused = [row for row in trace if row[2] == 'PAUSED']
        self.assertAlmostEqual(paused[0][0], 25.0, delta=0.15)             # 按 IMU 维持 5 s 后停车
        self.assertEqual({row[3] for row in paused}, {'heading_jump'})
        self.assertEqual({row[4] for row in paused}, {NOTICE_PAUSED})
        self.assertLess(max(row[5] for row in paused if row[0] > 27.0), 0.01)  # 真的停住了
        resumed = next(row for row in trace if row[0] > 25.0 and row[2] == 'RUNNING')
        self.assertTrue(40.0 <= resumed[0] <= 41.1)                        # 航向恢复后 1 s 内自动继续
        self.assertEqual(resumed[4], '航向恢复，自动继续')
        self.assertLess(r['max_cm'], 4.0)


class TestMission(unittest.TestCase):
    """LineMission：差速车定点画线换成 ImuLineTracker，阿克曼旧车照旧；start 不带 cruise 用 0.8。"""

    def mission(self, differential=True, state_path=None):
        fusion = FakeFusion()
        tracker = LineTracker(LineConfig(), sim.GEN5) if differential else None
        m = LineMission(fusion, sim.SAMPLES, tracker=tracker, nozzle_x=0.30 if differential else None,
                        state_path=state_path)
        m.points = {'A': fusion.to_geodetic(0.0, 0.0), 'B': fusion.to_geodetic(10.0, 0.0)}
        fusion.current = FakeFusion.make(-0.5, 0.0, 0.0)
        m.on_imu(0.0, 0.0, IMU_ZERO)
        return m

    def test_differential_line_uses_imu_cascade_and_ackermann_keeps_line_tracker(self):
        m = self.mission()
        self.assertEqual(m.command({'cmd': 'start'}, 0.01), (True, '申请控制权'))
        self.assertIsInstance(m.tracker, ImuLineTracker)
        self.assertEqual(m.tracker.cfg.nozzle_x_m, 0.30)
        self.assertEqual(m.tracker.cfg.stop_tol_m, m.line_cfg.stop_tol_m)     # 起步、刹停规则抄 LineConfig
        old = self.mission(differential=False)
        self.assertTrue(old.command({'cmd': 'start'}, 0.01)[0])
        self.assertIs(type(old.tracker), LineTracker)

    def test_cruise_defaults_to_line_speed_not_the_last_task(self):
        for differential in (True, False):
            with self.subTest(differential=differential):
                m = self.mission(differential)
                m.cruise = 0.3                                          # 上一次任务（例如路线）的速度
                self.assertTrue(m.command({'cmd': 'start'}, 0.01)[0])
                self.assertEqual(m.tracker.cfg.cruise_mps, LINE_CRUISE_MPS)
                self.assertEqual(LINE_CRUISE_MPS, 0.8)
                m.command({'cmd': 'stop'}, 0.02)
                m.tick(0.03, 'NAVIGATION')
                self.assertTrue(m.command({'cmd': 'start', 'cruise': 0.5}, 0.04)[0])
                self.assertEqual(m.tracker.cfg.cruise_mps, 0.5)

    def test_start_without_imu_is_refused_with_reason(self):
        m = self.mission()
        m._imu = None
        ok, msg = m.command({'cmd': 'start'}, 0.01)
        self.assertFalse(ok)
        self.assertIn('IMU', msg)

    def test_full_line_stops_at_b_and_does_not_store_steering_trim(self):
        with tempfile.TemporaryDirectory() as tmp:
            state = os.path.join(tmp, 'line.json')
            r = sim.run('cascade', v=0.8, seed=1, length=10.0)
            self.assertEqual((r['state'], r['message']), ('DONE', '到达 B 点'))
            # 记点的点（后轴中心）停在 B 前后 3 cm 内：喷头在它前面 0.30 m
            self.assertLess(abs(r['stop_m'] - sim.NOZZLE_X), 0.03)
            m = self.mission(state_path=state)
            m.integral0 = 0.07
            self.assertTrue(m.command({'cmd': 'start'}, 0.01)[0])
            m.tick(0.05, 'NAVIGATION')
            self.assertEqual(m.tracker.st.integral, 0.0)                 # 舵机零位是另一个量，不拿来用
            m.tracker.st.state = 'DONE'
            m.tick(0.1, 'NAVIGATION')
            self.assertEqual(m.message, '到达 B 点')
            self.assertFalse(os.path.exists(state))

    def test_abort_messages_explain_imu_reasons(self):
        r = sim.run('cascade', v=0.8, seed=1, length=20.0, imu_gap=(8.0, 60.0))
        self.assertEqual((r['state'], r['reason']), ('ABORTED', 'imu_stale'))
        self.assertEqual(r['message'], '中止：IMU 数据中断')
        r = sim.run('cascade', v=0.8, seed=1, length=20.0, push=(8.0, 8.5, 0.4))
        self.assertEqual((r['state'], r['reason']), ('ABORTED', 'imu_deviation'))
        self.assertIn('IMU 航向偏离目标超过 5°', r['message'])

    def test_pose_loss_pauses_then_mission_resumes(self):
        """定位丢了暂停（与 LineTracker 相同）；LineMission 的自动继续（定位连续可用 1 s）对它同样生效，照样走到 B。"""
        r = sim.run('cascade', v=0.8, seed=1, length=20.0, pose_gap=(8.0, 9.0))
        m = r['mission']
        self.assertEqual((r['state'], r['message']), ('DONE', '到达 B 点'))
        self.assertEqual(m.tracker._run, 2)                             # 暂停后重新开始了一次内环
        self.assertLess(r['max_cm'], 4.0)


def pooled_p95_cm(results):
    return sim.percentile([e for r in results for e in r['errs']], 0.95) * 100


class TestClosedLoop(unittest.TestCase):
    """实测底盘（指令延迟 0.39 s + 滞后 0.05 s、增益 1.03、加速 1.0 / 减速 2.4 m/s²）+ 随车速变大的万向轮扰动
    + 定位噪声（位置白噪声 3 mm + 慢漂 6 mm，双天线航向约 0.25°）。控制 20 Hz（line_node 的控制周期）。

    喷头横偏有两种量法：真实喷头（仿真才有）和按融合位姿算（实车作业记录里能算的，看不到 RTK 自己的慢漂）。
    RTK 慢漂 6 mm（一阶马尔可夫 20 s）在 40 m / 50 s 里偶尔漂到 1.8 cm（种子 2：约 3σ），控制器看不见它，
    真实喷头就跟着偏；所以 0.8 m/s 的 95% 按 4 个种子合在一起算，每个种子单独的看按融合位姿量的。"""

    @classmethod
    def setUpClass(cls):
        cls.fast = [sim.run('cascade', v=0.8, seed=s) for s in range(1, 5)]

    def test_disturbance_model_matches_measurements(self):
        """0.15–2 Hz：0.4 m/s 0.5–0.6°/s、0.8 m/s 1.5–1.9°/s；0.3–0.8 Hz 段 0.8 m/s 约为 0.4 m/s 的 2 倍。"""
        d = sim.Disturbance(1)
        self.assertTrue(0.5 <= d.band_rms(0.4, 0.15, 2.0) <= 0.6)
        self.assertTrue(1.5 <= d.band_rms(0.8, 0.15, 2.0) <= 1.9)
        self.assertAlmostEqual(d.band_rms(0.8, 0.3, 0.8) / d.band_rms(0.4, 0.3, 0.8), 2.0, places=6)

    def test_0p8_mps_40m_four_seeds(self):
        for s, r in enumerate(self.fast, 1):
            with self.subTest(seed=s):
                self.assertEqual((r['state'], r['message']), ('DONE', '到达 B 点'))
                self.assertLessEqual(r['max_cm'], 4.0)
                self.assertLessEqual(r['rtk_p95_cm'], 2.5)
        self.assertLessEqual(pooled_p95_cm(self.fast), 2.5)

    def test_inner_loop_at_20hz_is_as_good_as_50hz(self):
        """HeadingHold 的 Smith 预估按 50 Hz 调、实车验证过；定点画线在 20 Hz 控制周期里跑，两者差别在种子间的散布以内。"""
        at50 = [sim.run('cascade', v=0.8, seed=s, rate_hz=50.0) for s in range(1, 5)]
        self.assertLessEqual(pooled_p95_cm(self.fast), pooled_p95_cm(at50) + 0.3)
        self.assertLessEqual(max(r['max_cm'] for r in self.fast), max(r['max_cm'] for r in at50) + 0.8)

    def test_0p5_mps_40m(self):
        for s in range(1, 5):
            r = sim.run('cascade', v=0.5, seed=s)
            with self.subTest(seed=s):
                self.assertEqual(r['state'], 'DONE')
                self.assertLessEqual(r['p95_cm'], 2.0)
                self.assertLessEqual(r['max_cm'], 3.0)

    def test_imu_drift_and_alignment_error_100m(self):
        """IMU 每分钟漂 1°、IMU / 双天线与真北对齐误差 0.5°、100 m 线、0.8 m/s。"""
        for s in range(1, 5):
            trace = []
            r = sim.run('cascade', v=0.8, seed=s, length=100.0, imu_drift_deg_min=1.0, align_deg=0.5, trace=trace)
            with self.subTest(seed=s):
                self.assertEqual(r['state'], 'DONE')
                self.assertNotIn('PAUSED', {row[2] for row in trace})       # 漂移、对齐误差不会被当成航向跳变
                self.assertLessEqual(r['p95_cm'], 3.0)
                self.assertLessEqual(r['max_cm'], 4.5)

    def test_chassis_grid_never_aborts(self):
        for delay in (0.3, 0.4, 0.5, 0.6):
            for gain in (0.9, 1.05, 1.2):
                r = sim.run('cascade', v=0.8, seed=1, length=30.0, delay_s=delay, gain=gain)
                with self.subTest(delay=delay, gain=gain):
                    self.assertEqual((r['state'], r['message']), ('DONE', '到达 B 点'))

    def test_large_start_heading_error_turns_in_slowly(self):
        """起步条件允许车头差 30°、离线 0.5 m：目标按 3°/s 转过去（一步跳过去内环会因偏离 5° 停车），边慢走边转。"""
        for start in ((-2.0, 0.3, 20.0), (-2.0, -0.3, 20.0), (-2.0, 0.4, -25.0), (-2.0, 0.0, 29.0)):
            for delay, gain in ((0.39, 1.03), (0.6, 0.9), (0.3, 1.2)):
                r = sim.run('cascade', v=0.8, seed=1, length=20.0, start=start, delay_s=delay, gain=gain)
                with self.subTest(start=start, delay=delay, gain=gain):
                    self.assertEqual(r['state'], 'DONE', r['message'])

    def test_old_line_tracker_is_clearly_worse_at_0p8(self):
        """对照：原 LineTracker（只外推 0.1 s）在同一模型上 0.8 m/s 来回大摆（约 25 cm），证明仿真能区分。"""
        old = [sim.run('old', v=0.8, seed=s) for s in range(1, 3)]
        self.assertGreater(min(r['p95_cm'] for r in old), 10.0)
        self.assertLess(pooled_p95_cm(self.fast), 0.2 * min(r['p95_cm'] for r in old))


if __name__ == '__main__':
    unittest.main()
