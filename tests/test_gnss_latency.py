#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""卫星定位固定延迟的在线估计（gnss_latency，2026-10-07 用户「好的修复」）：定位比标的时间晚 τ，开喷、停喷点沿线挪
「车速 × τ」；每次从停着起步，卫星位置和轮程 + IMU 推算的位移对一下估出 τ，任务层把卫星数据的时间减去它。"""
import json
import math
import os
import random
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, ROOT)

import gnss_latency as gl  # noqa: E402

LEVER = (0.234, -0.212)


def drive(est, *, lag, heading_deg=30.0, imu_offset=1.1, scale=1.01, wobble=0.0, profile=None, seed=1, t=100.0,
          gnss_hz=5, still_s=1.0, start=None):
    """车停 still_s 秒，再按 profile（[(时长 s, 目标车速 m/s)]，1 m/s² 加减速）走，最后停 1 s。卫星位置是 lag 秒前的、
    2 mm 噪声；轮速按 scale 报（轮径差）；IMU 航向角 = 真车头 + imu_offset；车身左右摆 wobble °/s（1 Hz）。"""
    rng = random.Random(seed)
    profile = profile or [(6.0, 0.8)]
    x, y, yaw = start if start else (0.0, 0.0, math.radians(heading_deg))
    v = 0.0
    dt = 0.02
    hist = [(t, x, y, yaw)]
    plan = [(still_s, 0.0)] + list(profile) + [(1.5, 0.0)]

    def at(tt):                                     # 真位置按时间插值
        if tt <= hist[0][0]:
            return hist[0][1:]
        for (ta, xa, ya, wa), (tb, xb, yb, wb) in zip(reversed(hist[:-1]), reversed(hist[1:])):
            if ta <= tt <= tb:
                f = (tt - ta) / (tb - ta)
                return xa + (xb - xa) * f, ya + (yb - ya) * f, wa + (wb - wa) * f
        return hist[-1][1:]

    for duration, target in plan:
        for _ in range(int(round(duration / dt))):
            v += max(-dt, min(dt, target - v))
            est.on_wheel(t, v / scale)               # 这一拍的车速，管到下一拍
            wz = math.radians(wobble) * math.sin(2 * math.pi * t) if abs(v) > 0.05 else 0.0
            mid = yaw + wz * dt / 2
            yaw += wz * dt
            x += v * dt * math.cos(mid)
            y += v * dt * math.sin(mid)
            t = round(t + dt, 4)
            hist.append((t, x, y, yaw))
            del hist[:-60]
            est.on_imu(t, yaw + imu_offset, wz)
            if round(t * 1000) % int(1000 / gnss_hz) == 0:
                hx, hy, hyaw = at(t - lag)
                c, s = math.cos(hyaw), math.sin(hyaw)
                ax, ay = hx + c * LEVER[0] - s * LEVER[1], hy + s * LEVER[0] + c * LEVER[1]
                est.on_fix(t, ax + rng.gauss(0, 0.002), ay + rng.gauss(0, 0.002), LEVER)
    drive.end = (x, y, yaw)
    return t


class Estimator(unittest.TestCase):
    def test_constant_lag_is_found_from_one_start(self):
        for lag in (0.0, 0.05, 0.15):
            with self.subTest(lag=lag):
                est = gl.GnssLatencyEstimator()
                drive(est, lag=lag)
                self.assertEqual(est.status()['windows'], 1, est.last)
                self.assertAlmostEqual(est.estimate, lag, delta=0.012)  # 0.8 m/s 时 1 cm 以内
                self.assertAlmostEqual(est.value, min(lag, gl.LATENCY_APPLY_MAX_S), delta=0.012)
                self.assertEqual(est.status()['capped'], lag > gl.LATENCY_APPLY_MAX_S + 0.012)
                self.assertAlmostEqual(est.last['scale'], 1.01, delta=0.01)

    def test_wobble_reverse_and_20hz(self):
        est = gl.GnssLatencyEstimator()
        t = drive(est, lag=0.08, wobble=3.0, profile=[(2.0, -0.5)], gnss_hz=20)       # 倒车助跑
        drive(est, lag=0.08, wobble=3.0, profile=[(5.0, 0.8)], t=t, seed=2, gnss_hz=20, start=drive.end)
        self.assertEqual(est.status()['windows'], 2, est.last)
        self.assertAlmostEqual(est.value, 0.08, delta=0.012)

    def test_short_moves_and_pivots_do_not_count(self):
        est = gl.GnssLatencyEstimator()
        drive(est, lag=0.1, profile=[(0.6, 0.25)])                         # 只挪了十几厘米
        self.assertEqual(est.status()['windows'], 0)
        self.assertEqual(est.last['reason'], 'too_short')
        self.assertEqual(est.value, 0.0)

    def test_median_of_recent_starts_and_reset(self):
        est = gl.GnssLatencyEstimator()
        t = 100.0
        end = None
        for seed in range(1, 5):
            t = drive(est, lag=0.06, t=t, seed=seed, start=end)
            end = drive.end
        self.assertEqual(est.status()['windows'], 4)
        self.assertAlmostEqual(est.value, 0.06, delta=0.008)
        est.reset()
        self.assertEqual((est.value, est.status()['windows']), (0.0, 0))


class MissionAppliesLatency(unittest.TestCase):
    """任务层：卫星数据的时间减去估出来的延迟再交给融合；换天线清零。"""

    @staticmethod
    def starts(lag):
        import test_single_antenna as T
        m, fusion, _, _ = T.mission()
        car = T.Car(m, yaw_deg=40.0)
        car.gnss_lag = lag
        car.run(1.0)
        assert T.orient(m, car).startswith('车头已确定')
        for _ in range(3):                                   # 停 1.5 s、往前 4 s、再停：几次起步
            car.free_v = 0.0
            car.run(1.5)
            car.free_v = 0.6
            car.run(4.0)
        car.free_v = 0.0
        car.run(1.5)
        return m, fusion, car

    def test_late_satellite_is_shifted_back(self):
        # 测试小车自己的时间口径（轮速、位置差一拍）有固定偏差：比「晚 80 ms」和「不晚」两次估出来的差
        m0, _, _ = self.starts(0.0)
        m, fusion, car = self.starts(0.08)
        st0, st = m0.status()['gnss_latency'], m.status()['gnss_latency']
        self.assertGreaterEqual(st['windows'], 2, st)
        self.assertAlmostEqual(st['value_s'] - st0['value_s'], 0.08, delta=0.015)
        seen = []
        add_fix = fusion.add_fix
        fusion.add_fix = lambda sample: (seen.append(sample.t), add_fix(sample))[1]
        car.run(0.4)
        self.assertTrue(seen)
        stamps = [round(v + m._latency.value, 3) for v in seen]
        self.assertTrue(all(abs(s * 5 - round(s * 5)) < 1e-6 for s in stamps), stamps)   # 原始时间是 5 Hz 整拍
        car.mode = 'dual'
        car.run(0.2)
        self.assertEqual(m.status()['gnss_latency']['value_s'], 0.0)          # 换天线：重新估

    def test_pending_while_moving_and_saved_per_antenna(self):
        import tempfile
        import test_single_antenna as T
        with tempfile.TemporaryDirectory() as tmp:
            dual, single = T.gen5_configs()
            m = T.LineMission(T.gf.GnssFusion(dual), T.gf, single_config=single, state_path=os.path.join(tmp, 'line_state.json'))
            car = T.Car(m, yaw_deg=40.0)
            car.gnss_lag = 0.1
            car.run(1.0)
            assert T.orient(m, car).startswith('车头已确定')
            car.free_v = 0.0
            car.run(1.5)
            car.free_v = 0.8
            car.run(7.0)                                      # 走满 4 m 这段结束，车还在走：先挂着
            st = m.status()['gnss_latency']
            self.assertIsNotNone(st['pending_s'], st)
            moving_value = st['value_s']
            car.free_v = 0.0
            car.run(2.0)                                      # 停稳：换上、存盘
            st = m.status()['gnss_latency']
            self.assertIsNone(st['pending_s'])
            self.assertNotEqual(st['value_s'], moving_value)
            with open(os.path.join(tmp, 'gnss_latency.json'), encoding='utf-8') as fh:
                saved = json.load(fh)
            self.assertAlmostEqual(saved['single'], st['value_s'], places=6)
            m2 = T.LineMission(T.gf.GnssFusion(dual), T.gf, single_config=single, state_path=os.path.join(tmp, 'line_state.json'))
            car2 = T.Car(m2, yaw_deg=40.0)
            car2.run(0.5)                                     # 收到单天线定位：换成单天线，用存的值
            self.assertAlmostEqual(m2.status()['gnss_latency']['value_s'], saved['single'], places=6)


if __name__ == '__main__':
    unittest.main()
