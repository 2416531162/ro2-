#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IMU 定向直行 radar_system/heading_hold.py：定目标、纠偏方向、停车条件、延迟补偿、闭环仿真。"""
import math
import os
import random
import sys
import unittest
from collections import deque

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))

from heading_hold import DEG, HeadingHold, HoldConfig, wrap, yaw_from_quaternion  # noqa: E402


def quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class TestHeadingHold(unittest.TestCase):

    def test_new_run_locks_current_imu_yaw_and_ramps_to_fixed_speed(self):
        h = HeadingHold()
        h.on_imu(30 * DEG, 0.0, now=10.0)
        vx, wz = h.step(1, now=10.0)
        self.assertEqual(h.state, 'running')
        self.assertAlmostEqual(h.target, 30 * DEG)
        self.assertAlmostEqual(vx, HoldConfig().min_speed)      # 起步第一帧不是 0（0 会被底盘当成停车）
        self.assertAlmostEqual(wz, 0.0)
        for i in range(1, 90):                                  # 0.5 m/s² 斜坡 1.5 s 升到 0.8
            h.on_imu(30 * DEG, 0.0, now=10.0 + i * 0.02)
            vx, wz = h.step(1, now=10.0 + i * 0.02)
        self.assertAlmostEqual(vx, 0.8)                         # 用户定的固定车速
        self.assertAlmostEqual(wz, 0.0)
        st = h.status(now=11.78)
        self.assertEqual((st['state'], st['run'], st['target_deg'], st['error_deg']), ('running', 1, 30.0, 0.0))
        self.assertEqual(st['within_pct'], 100.0)

    def test_turns_back_toward_target(self):
        """车头偏左（偏航比目标大）→ 往右打（wz 为负）；偏右 → 往左。"""
        h = HeadingHold()
        h.on_imu(0.0, 0.0, now=0.0)
        h.step(1, now=0.0)
        h.on_imu(1 * DEG, 0.0, now=0.02)
        self.assertLess(h.step(1, now=0.02)[1], 0.0)
        h.on_imu(-1 * DEG, 0.0, now=0.04)
        self.assertGreater(h.step(1, now=0.04)[1], 0.0)

    def test_error_wraps_across_180(self):
        h = HeadingHold()
        h.on_imu(179 * DEG, 0.0, now=0.0)
        h.step(1, now=0.0)
        h.on_imu(-179 * DEG, 0.0, now=0.02)     # 实际左转了 2°，不是右转 358°
        vx, wz = h.step(1, now=0.02)
        self.assertEqual(h.state, 'running')
        self.assertLess(wz, 0.0)
        self.assertAlmostEqual(h.status(now=0.02)['error_deg'], 2.0, places=3)

    def test_no_imu_refuses_to_start(self):
        h = HeadingHold()
        self.assertEqual(h.step(1, now=5.0), (0.0, 0.0))
        self.assertEqual((h.state, h.reason), ('stopped', 'no_imu'))
        h.on_imu(0.0, 0.0, now=1.0)                         # 太旧的也不算
        self.assertEqual(h.step(2, now=5.0), (0.0, 0.0))
        self.assertEqual(h.reason, 'no_imu')

    def test_imu_dropout_stops_and_same_run_stays_stopped(self):
        h = HeadingHold()
        h.on_imu(0.0, 0.0, now=0.0)
        h.step(1, now=0.0)
        for i in range(1, 11):                              # 50 Hz 照常算，IMU 0 s 以后没了
            out = h.step(1, now=i * 0.02)
        self.assertGreater(out[0], 0.0)                     # 0.2 s 还在开
        self.assertEqual(h.step(1, now=0.22), (0.0, 0.0))  # 超过 0.2 s 没有新 IMU 就停
        self.assertEqual((h.state, h.reason), ('stopped', 'imu_stale'))
        h.on_imu(0.0, 0.0, now=0.3)
        self.assertEqual(h.step(1, now=0.3), (0.0, 0.0))   # 同一次作业不自己恢复
        vx, _ = h.step(2, now=0.3)                          # 再点开始才重新开
        self.assertGreater(vx, 0.0)
        self.assertEqual(h.state, 'running')

    def test_own_stall_waits_for_imu_callback_to_catch_up(self):
        """web 进程自己卡了 0.4 s（IMU 其实一直在发，回调压在队列里）：卡完第一次计算先按卡之前的输出开，
        IMU 回调追上后照常；不能当成 IMU 中断停车（2026-09-30 13:23、13:26 两次误停）。"""
        h, t = cruising()
        before = h.step(1, now=t)
        t += 0.4                                            # 卡住：这 0.4 s 既没算也没收 IMU
        self.assertEqual(h.step(1, now=t), before)
        self.assertEqual(h.state, 'running')
        h.on_imu(0.0, 0.0, now=t + 0.005)                   # 队列里的 IMU 处理了
        vx, _ = h.step(1, now=t + 0.02)
        self.assertEqual((h.state, vx), ('running', HoldConfig().speed))

    def test_imu_lost_during_own_stall_still_stops(self):
        """卡完 IMU 也追不上（真断了）：宽限 imu_grace_s 一过就停；卡之前 IMU 已经不正常（超过 stall_s 没来）
        则不给宽限。"""
        c = HoldConfig()
        h, t = cruising()
        t0 = t + 0.4
        self.assertGreater(h.step(1, now=t0)[0], 0.0)
        for i in range(1, 8):                               # 卡完 50 Hz 照常算，0.14 s 内还在开
            self.assertGreater(h.step(1, now=t0 + i * 0.02)[0], 0.0)
        self.assertEqual(h.step(1, now=t0 + c.imu_grace_s + 0.01), (0.0, 0.0))
        self.assertEqual(h.reason, 'imu_stale')
        h, t = cruising()
        for i in range(1, 10):                              # IMU 断了 0.18 s，然后才卡
            h.step(1, now=t + i * 0.02)
        self.assertEqual(h.step(1, now=t + 0.18 + 0.3), (0.0, 0.0))
        self.assertEqual(h.reason, 'imu_stale')

    def test_large_deviation_stops(self):
        h = HeadingHold()
        h.on_imu(0.0, 0.0, now=0.0)
        h.step(1, now=0.0)
        h.on_imu(5.5 * DEG, 0.0, now=0.02)
        self.assertEqual(h.step(1, now=0.02), (0.0, 0.0))
        self.assertEqual((h.state, h.reason), ('stopped', 'deviation'))

    def test_end_reasons_only_while_running(self):
        h = HeadingHold()
        self.assertFalse(h.end('user'))
        self.assertEqual(h.status(now=0.0), {'imu_ok': False, 'yaw_deg': None, 'state': 'idle'})
        h.on_imu(0.0, 0.0, now=0.0)
        h.step(('app-1', 3), now=0.0)
        self.assertTrue(h.end('command_timeout'))
        self.assertFalse(h.end('user'))                     # 第一个原因留着
        st = h.status(now=0.01)
        self.assertEqual((st['state'], st['reason'], st['run'], st['speed']),
                         ('stopped', 'command_timeout', ('app-1', 3), 0.0))

    def test_integral_is_bounded(self):
        c = HoldConfig()
        h = HeadingHold()
        h.on_imu(0.0, 0.0, now=0.0)
        h.step(1, now=0.0)
        for i in range(1, 2000):                             # 一直偏 4°（车被卡住）也不会攒出更大的转向
            h.on_imu(4 * DEG, 0.0, now=i * 0.02)
            _, wz = h.step(1, now=i * 0.02)
        self.assertLessEqual(abs(h._integ), c.i_max + 1e-12)
        self.assertLessEqual(abs(wz), c.max_wz + 1e-12)

    def test_quaternion_yaw_and_wrap(self):
        for deg in (-179.0, -45.0, 0.0, 30.0, 179.9):
            self.assertAlmostEqual(math.degrees(yaw_from_quaternion(*quat(deg * DEG))), deg, places=6)
        self.assertIsNone(yaw_from_quaternion(0, 0, 0, 0))
        self.assertAlmostEqual(wrap(3 * math.pi / 2), -math.pi / 2)


def cruising(config=None, seconds=2.0, odom=None):
    """偏差 0 开 seconds 秒（起步斜坡 1.5 s 走完），返回 (hold, 当前时刻)。odom 给了就每帧喂这个车速。"""
    h = HeadingHold(config)
    t = 0.0
    h.on_imu(0.0, 0.0, now=t)
    h.step(1, now=t)
    while t < seconds - 1e-9:
        t = round(t + 0.02, 6)
        h.on_imu(0.0, 0.0, now=t)
        if odom is not None:
            h.on_odom(odom, now=t)
        h.step(1, now=t)
    return h, t


class TestCallerTarget(unittest.TestCase):
    """step(run, now, target=, vx=)：定点画线的外环每步给目标偏航和车速；不给时与原来一样（上面的测试）。"""

    def test_target_is_taken_every_step(self):
        h = HeadingHold()
        h.on_imu(0.3, 0.0, now=0.0)
        h.step(1, now=0.0, target=0.3)
        self.assertAlmostEqual(h.target, 0.3)
        h.on_imu(0.3, 0.0, now=0.02)
        _, wz = h.step(1, now=0.02, target=0.3 + 2 * DEG)      # 目标在左 → 往左转
        self.assertGreater(wz, 0.0)
        self.assertAlmostEqual(h.target, 0.3 + 2 * DEG)
        self.assertAlmostEqual(h.status(now=0.02)['error_deg'], -2.0, places=3)
        h.on_imu(0.3, 0.0, now=0.04)
        self.assertEqual(h.step(1, now=0.04, target=0.3 - 6 * DEG), (0.0, 0.0))   # 离目标超过 5° 照样停
        self.assertEqual(h.reason, 'deviation')

    def test_new_run_without_imu_still_refuses_even_with_target(self):
        h = HeadingHold()
        self.assertEqual(h.step(1, now=1.0, target=0.0), (0.0, 0.0))
        self.assertEqual((h.state, h.reason, h.target), ('stopped', 'no_imu', None))

    def test_caller_speed_is_output_and_gates_the_integral(self):
        """vx 原样输出；不到 speed 的一半算「没走起来」不积分（与自己的起步斜坡同一规则）。"""
        c = HoldConfig()
        for vx, grows in ((0.3 * c.speed, False), (0.9 * c.speed, True)):
            h = HeadingHold()
            h.on_imu(0.0, 0.0, now=0.0)
            h.step(1, now=0.0, vx=vx)
            for i in range(1, 20):
                h.on_imu(1 * DEG, 0.0, now=i * 0.02)
                out, _ = h.step(1, now=i * 0.02, vx=vx)
                self.assertEqual(out, vx)
            self.assertEqual(h._integ != 0.0, grows, vx)


class TestDelayCompensation(unittest.TestCase):
    """底盘转向延迟 0.44 s：发出去的纠偏要 0.44 s 后车头才转，这段时间不能按原偏差接着加码。"""

    def test_prediction_counts_turn_already_sent(self):
        """车头偏左 1°、底盘还没反应（陀螺 0）：新控制器把已发出的右转算进预估，0.4 s 内纠偏量收回来；
        旧控制器（不补偿）一直按 1° 打、积分还往上加，等车头转过来就过头了——这就是 0.56 Hz 摆动的来源。"""
        def turns(config):
            h, t = cruising(config)
            out = []
            for _ in range(20):
                t = round(t + 0.02, 6)
                h.on_imu(1 * DEG, 0.0, now=t)
                out.append(h.step(1, now=t)[1])
            return out
        new = turns(HoldConfig())
        self.assertLess(new[0], 0.0)
        self.assertLessEqual(max(new), 0.0)
        self.assertLess(abs(new[-1]), 0.7 * abs(new[0]))           # 按 1/kp ≈ 0.45 s 收回来
        old = turns(HoldConfig(kp=3.0, ki=1.2, kd=0.2, delay_s=0.0))
        self.assertGreaterEqual(abs(old[-1]), abs(old[0]))

    def test_no_integral_during_start_ramp(self):
        """起步斜坡前半段（指令车速还没到 0.4）偏差再大也不积分，过了一半才开始。"""
        c = HoldConfig()
        h = HeadingHold()
        h.on_imu(0.0, 0.0, now=0.0)
        h.step(1, now=0.0)
        for i in range(1, 90):
            t = i * 0.02
            h.on_imu(1 * DEG, 0.0, now=t)
            vx, _ = h.step(1, now=t)
            if vx < c.moving_ratio * c.speed:
                self.assertEqual(h._integ, 0.0, t)
        self.assertEqual(vx, c.speed)
        self.assertNotEqual(h._integ, 0.0)

    def test_no_integral_while_output_saturated(self):
        """被撞得猛转（陀螺 1 rad/s）：纠偏打满 max_wz，这时积分不动。"""
        c = HoldConfig()
        h, t = cruising()
        h.on_imu(1 * DEG, 0.0, now=t + 0.02)
        h.step(1, now=t + 0.02)
        before = h._integ
        h.on_imu(1 * DEG, 1.0, now=t + 0.04)
        _, wz = h.step(1, now=t + 0.04)
        self.assertEqual(wz, -c.max_wz)
        self.assertEqual(h._integ, before)

    def test_no_integral_while_odom_says_wheels_stuck(self):
        """里程计车速不到指令的一半（轮子卡住）不积分；车速正常、或里程计断了（只看起步斜坡）照常积分。"""
        speed = HoldConfig().speed
        for odom, grows in ((0.0, False), (0.2 * speed, False), (0.95 * speed, True)):
            h, t = cruising(odom=odom)
            for _ in range(20):
                t = round(t + 0.02, 6)
                h.on_imu(1 * DEG, 0.0, now=t)
                h.on_odom(odom, now=t)
                h.step(1, now=t)
            self.assertEqual(h._integ != 0.0, grows, odom)
        h, t = cruising(odom=0.0)
        h.on_imu(1 * DEG, 0.0, now=t + 0.5)                 # 里程计 0.5 s 没更新：不看它
        h.step(1, now=t + 0.5)
        self.assertNotEqual(h._integ, 0.0)


class LowFreq:
    """低频偏转（万向轮、地面）：几个正弦叠成的角速度，各分量角速度等幅，积成角度来回 pp_deg。
    MEASURED：2026-09-30 11:50、11:51 两次实车直行，陀螺减去「0.39 s 前的指令 × 1.04、0.05 s 滞后」剩下的转动，
    0.05–0.35 Hz 的角速度谱基本是平的（每档 0.25–0.4°/s，合计 rms 约 0.3–0.4°/s，积成角度来回 1.6–1.9°）。
    SLOW：只有 0.1 Hz 以下的慢摆（一个来回 10 s 以上）。"""
    MEASURED = (0.06, 0.11, 0.17, 0.24, 0.32)
    SLOW = (0.04, 0.07, 0.1)

    def __init__(self, freqs=MEASURED, pp_deg=2.0, seed=1, seconds=25.0):
        rng = random.Random(seed)
        self.freqs = freqs
        self.phases = [rng.uniform(0, 2 * math.pi) for _ in freqs]
        self.k = 1.0
        angles = [self.angle(i * 0.05) for i in range(int(seconds / 0.05))]
        self.k = pp_deg / (max(angles) - min(angles))

    def angle(self, t):
        """积成的角度，度。"""
        return self.k * sum(-math.cos(2 * math.pi * f * t + p) / (2 * math.pi * f)
                            for f, p in zip(self.freqs, self.phases))

    def rate(self, t):
        """角速度，rad/s。"""
        return self.k * DEG * sum(math.sin(2 * math.pi * f * t + p) for f, p in zip(self.freqs, self.phases))


class Plant:
    """差速车偏航：指令角速度经死区 tau、0.05 s 一阶滞后，乘上实际/指令比 gain，再加两轮不一致的固定偏转 bias、
    kick_t 时段的甩尾 kick、2.3 Hz 转速波动 ripple、低频偏转 lf。IMU 50 Hz、0.01° 一格。
    sign=−1 是 IMU 装反（偏航读数与真实转向相反）。
    true_gyro：陀螺给真实转动（含偏转、甩尾、波动，实车 IMU 就是这样）；默认只给底盘对指令的响应（最早的写法）。
    app_steps：除了 50 Hz 发布循环，再按 app 的 go 指令节奏（约 20 Hz、±20 ms 抖动）调 step，同 radar_web_server。
    stuck=(t0, t1)：这段时间轮子卡住，车不动也不转；odom=True 时按实际车速喂里程计（卡住时 0）。"""

    def __init__(self, tau=0.2, gain=1.0, bias=0.0, kick=0.0, ripple=0.03, sign=1.0, seed=1,
                 kick_t=(0.0, 0.8), lf=None, true_gyro=False, app_steps=False, stuck=None):
        self.tau, self.gain, self.bias, self.kick, self.ripple, self.sign = tau, gain, bias, kick, ripple, sign
        self.kick_t, self.lf, self.true_gyro, self.app_steps, self.stuck = kick_t, lf, true_gyro, app_steps, stuck
        self.rng = random.Random(seed)

    def run(self, hold, seconds=15.0, yaw0=0.3, odom=False):
        dt, t, yaw, wz_act, v_act, rate = 0.002, 0.0, yaw0, 0.0, 0.0, 0.0
        cmds = deque()      # (时刻, wz 指令, vx 指令)
        err, next_ctl, next_app = [], 0.0, 0.013
        while t < seconds:
            stuck = self.stuck is not None and self.stuck[0] <= t < self.stuck[1]
            ctl = t >= next_ctl - 1e-9
            app = self.app_steps and t >= next_app - 1e-9
            if ctl or app:
                if ctl:
                    next_ctl += 0.02
                    meas = round(math.degrees(self.sign * yaw) / 0.01) * 0.01 + self.rng.gauss(0, 0.005)
                    hold.on_imu(math.radians(meas), self.sign * (rate if self.true_gyro else wz_act), now=t)
                    if odom:
                        hold.on_odom(0.0 if stuck else v_act, now=t)
                if app:
                    next_app += 0.05 + self.rng.uniform(-0.02, 0.02)
                vx, wz = hold.step(7, now=t)
                cmds.append((t, wz if vx > 0 else 0.0, vx))
                if hold.state != 'running':
                    return err
            while len(cmds) > 1 and cmds[1][0] <= t - self.tau:
                cmds.popleft()
            w_d, v_d = cmds[0][1:] if cmds and cmds[0][0] <= t - self.tau else (0.0, 0.0)
            wz_act += (self.gain * w_d - wz_act) * dt / 0.05
            v_act += (v_d - v_act) * dt / 0.1
            rate = wz_act + self.bias + (self.kick if self.kick_t[0] <= t < self.kick_t[1] else 0.0) \
                + self.ripple * math.sin(2 * math.pi * 2.3 * t) + (self.lf.rate(t) if self.lf else 0.0)
            if stuck:
                rate = 0.0
            yaw += rate * dt
            err.append((t, math.degrees(wrap(yaw - yaw0))))
            t += dt
        return err


def swing(err, after=5.0):
    """after 秒以后的 (来回幅度 = 最大 − 最小, 最大 |偏差|, 平均偏差)，度。"""
    x = [e for t, e in err if t > after]
    return max(x) - min(x), max(abs(v) for v in x), sum(x) / len(x)


def swing_hz(err, after=5.0):
    """after 秒以后偏差穿过均值的次数折成频率（一个来回穿两次），Hz。"""
    x = [e for t, e in err if t > after]
    m = sum(x) / len(x)
    crossings = sum(1 for a, b in zip(x, x[1:]) if (a - m) * (b - m) < 0)
    return crossings / 2.0 / (err[-1][0] - after)


OLD = HoldConfig(kp=3.0, ki=1.2, kd=0.2, delay_s=0.0)       # 2026-09-30 11:50 实车跑的：没有延迟补偿
RETUNE = HoldConfig(kp=1.5, ki=0.3, kd=0.3, delay_s=0.0)    # 对照：不补偿、只把参数调软


def measured_chassis(lf=None, tau=0.44, gain=1.03, bias=0.0, kick=0.2, seed=1):
    """2026-09-30 实测底盘：转向延迟 0.44 s（另有 0.05 s 滞后，比实测略保守）、转出指令的 1.03 倍，
    起步万向轮甩尾（0.55–0.8 s 0.2 rad/s ≈ 2.9°，实车起步 ±3.7–3.8°），转速波动 0.01 rad/s（实测 1.5 Hz 以上
    rms 约 0.35–0.41°/s，原来的 0.03 大了 5 倍），陀螺给真实转动，app 指令和 50 Hz 循环都调 step。"""
    return Plant(tau, gain, bias, kick, ripple=0.01, seed=seed, kick_t=(0.55, 0.8), lf=lf, true_gyro=True,
                 app_steps=True)


class TestClosedLoop(unittest.TestCase):
    """底盘死区 0.1–0.3 s（比实测 0.44 s 快的底盘也要稳）、两轮不一致 ±0.04 rad/s、起步甩尾（一侧晚 50 ms）。"""

    def test_holds_within_one_degree(self):
        worst_settled, worst_max = 0.0, 0.0
        for tau in (0.1, 0.2, 0.3):
            for gain in (0.9, 1.0, 1.2):
                for bias in (-0.04, 0.0, 0.04):
                    for kick in (0.0, 0.037):
                        hold = HeadingHold()
                        err = Plant(tau, gain, bias, kick).run(hold)
                        with self.subTest(tau=tau, gain=gain, bias=bias, kick=kick):
                            self.assertEqual(hold.state, 'running', hold.reason)
                            self.assertLess(max(abs(e) for t, e in err if t > 4.0), 1.0)
                            worst_settled = max(worst_settled, max(abs(e) for t, e in err if t > 6.0))
                            worst_max = max(worst_max, max(abs(e) for _, e in err))
        # 最差是两轮不一致 0.04 rad/s（实测不到 0.002）：4 s 后约 0.9°，6 s 后约 0.35°；
        # 起步甩尾和两轮不一致同向叠加时起步最大约 2.9°，离停车线 5° 还远
        self.assertLess(worst_settled, 0.4)
        self.assertLess(worst_max, 3.5)

    def test_half_steering_response_still_converges(self):
        """底盘只转出指令的一半（实测 1.02–1.06，这是给极端情况留的）：不停车、8 s 后回到 ±1° 内。
        为了在 0.7 s 延迟下也不振，纠偏比 2026-09-30 之前软，这种情况收得慢一些（原来 4 s，现在 8 s）。"""
        for tau in (0.1, 0.2, 0.3):
            for bias in (-0.04, 0.0, 0.04):
                for kick in (0.0, 0.037):
                    hold = HeadingHold()
                    err = Plant(tau, 0.5, bias, kick).run(hold)
                    with self.subTest(tau=tau, bias=bias, kick=kick):
                        self.assertEqual(hold.state, 'running', hold.reason)
                        self.assertLess(max(abs(e) for t, e in err if t > 8.0), 1.0)
                        self.assertLess(max(abs(e) for _, e in err), 4.0)

    def test_nominal_chassis_never_leaves_one_degree(self):
        """死区 0.2 s、转向如实、两轮一致：全程（含起步）都在 ±1° 内。"""
        hold = HeadingHold()
        err = Plant(0.2, 1.0, 0.0, 0.015).run(hold, seconds=20.0)
        self.assertEqual(hold.state, 'running')
        self.assertLess(max(abs(e) for _, e in err), 1.0)
        self.assertGreater(hold.status(now=20.0)['within_pct'], 99.0)

    def test_imu_mounted_backwards_stops_quickly(self):
        """IMU 方向反了：纠偏变成越拐越偏，偏差过 5° 必须停，不能一直开下去。"""
        hold = HeadingHold()
        err = Plant(0.2, 1.0, 0.02, 0.0, sign=-1.0).run(hold, seconds=10.0)
        self.assertEqual((hold.state, hold.reason), ('stopped', 'deviation'))
        self.assertLess(err[-1][0], 3.0)
        self.assertLess(max(abs(e) for _, e in err), 6.0)


GRID = [(tau, gain, bias, kick) for tau in (0.3, 0.44, 0.55, 0.7) for gain in (0.9, 1.03, 1.2)
        for bias in (-0.02, 0.0, 0.02) for kick in (0.2, -0.2)]


class TestMeasuredChassis(unittest.TestCase):
    """按 2026-09-30 实车数据：转向延迟 0.44 s（网格 0.3–0.7 s）、增益 1.03（网格 0.9–1.2）、两轮不一致 ±0.02 rad/s、
    万向轮低频偏转积成角度来回 2°。都看 5 s 以后（起步甩尾不算）。"""

    def worst(self, config, freqs):
        """网格上 5 s 后最大 |偏差|；有停车直接判失败。"""
        worst = 0.0
        for tau, gain, bias, kick in GRID:
            hold = HeadingHold(config)
            err = measured_chassis(LowFreq(freqs), tau, gain, bias, kick).run(hold, seconds=25.0)
            with self.subTest(tau=tau, gain=gain, bias=bias, kick=kick):
                self.assertEqual(hold.state, 'running', hold.reason)
            worst = max(worst, swing(err)[1])
        return worst

    def nominal(self, config, freqs):
        """标称底盘、6 种扰动相位 × 起步左右甩：5 s 后来回幅度的 (平均, 最大)。"""
        pps = []
        for seed in range(1, 7):
            for kick in (0.2, -0.2):
                hold = HeadingHold(config)
                err = measured_chassis(LowFreq(freqs, seed=seed), kick=kick, seed=seed).run(hold, seconds=25.0)
                self.assertEqual(hold.state, 'running', hold.reason)
                pps.append(swing(err)[0])
        return sum(pps) / len(pps), max(pps)

    def test_old_gains_oscillate_at_measured_delay_new_does_not(self):
        """复现实车：旧参数在 0.44 s 延迟下持续摆（实车 0.54–0.62 Hz、来回 1.8–2.6°），新控制器不摆。"""
        for kick in (0.2, -0.2):
            err = measured_chassis(kick=kick).run(HeadingHold(OLD), seconds=25.0)
            pp, _, _ = swing(err)
            self.assertGreater(pp, 2.0)
            self.assertTrue(0.45 <= swing_hz(err) <= 0.65, swing_hz(err))
            hold = HeadingHold()
            err = measured_chassis(kick=kick).run(hold, seconds=25.0)
            self.assertEqual(hold.state, 'running')
            self.assertLess(swing(err)[0], 0.3)                  # 只剩 2.3 Hz 转速波动

    def test_slow_drift_nominal_and_grid(self):
        """0.1 Hz 以下的慢摆（来回 2°）：标称平均来回 ≤0.5°（新 0.44°、最差 0.56°；只调参 0.52°、0.73°）；
        整个网格不停车、都在 ±1° 内（新最差 0.95°，只调参 1.25° 出界）。"""
        avg, mx = self.nominal(HoldConfig(), LowFreq.SLOW)
        self.assertLessEqual(avg, 0.5)
        self.assertLess(mx, 0.7)
        self.assertLess(avg, self.nominal(RETUNE, LowFreq.SLOW)[0])
        worst = self.worst(HoldConfig(), LowFreq.SLOW)
        self.assertLess(worst, 1.0)
        self.assertGreater(self.worst(RETUNE, LowFreq.SLOW), worst)

    def test_measured_drift_spectrum(self):
        """按实测谱（0.06–0.32 Hz 等幅，来回 2°）：标称平均来回约 1.18°，网格最差约 1.25°，不停车。
        0.5° 做不到：0.17–0.32 Hz 那几份扰动，纠偏最早 0.44 s 后才生效，还要在 0.7 s 延迟下不振，
        任何参数都压不下去（2026-09-30 扫了约 900 组：网格不出 ±1.1° 的，标称都在 1.1° 以上）；
        只调参（kp 1.5/ki 0.3/kd 0.3）在这里和新控制器差不多（1.15°、1.18°）。"""
        avg, _ = self.nominal(HoldConfig(), LowFreq.MEASURED)
        self.assertLess(avg, 1.3)
        self.assertLess(self.worst(HoldConfig(), LowFreq.MEASURED), 1.4)

    def test_constant_drift_leaves_no_offset(self):
        """两轮不一致带来的固定偏转：积分项同时当偏转估计并进预估，稳态不留「偏转 × 延迟」
        （0.02 rad/s × 0.44 s ≈ 0.5°，只做 Smith 预估 + PI 会一直偏这么多）。"""
        for bias in (-0.02, 0.02):
            hold = HeadingHold()
            err = measured_chassis(bias=bias).run(hold, seconds=25.0)
            self.assertLess(abs(swing(err, after=12.0)[2]), 0.05)

    def test_stuck_wheels_then_release_does_not_swing(self):
        """被推歪 3° 后轮子卡住 3 s：里程计说没走，不积分；松开后回正不甩过头（不接里程计会甩到约 −2.5°）。"""
        plant = Plant(0.44, 1.03, kick=0.26, kick_t=(7.8, 8.0), ripple=0.01, true_gyro=True, app_steps=True,
                      stuck=(8.0, 11.0))
        hold = HeadingHold()
        err = plant.run(hold, seconds=20.0, odom=True)
        self.assertEqual(hold.state, 'running', hold.reason)
        self.assertGreater(min(e for t, e in err if 8.0 <= t < 11.0), 2.5)
        self.assertGreater(min(e for t, e in err if t >= 11.0), -1.0)
        self.assertLess(max(abs(e) for t, e in err if t > 14.0), 0.5)


if __name__ == '__main__':
    unittest.main()
