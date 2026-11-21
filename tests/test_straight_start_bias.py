"""IMU 直行起步带上学到的偏转 + 起步斜坡 0.5 → 0.25 m/s²（2026-11-21：两趟实车起步都先往右偏 2–3°，匀速时积分
稳在 +0.02–0.03 rad/s）。

车模型按那两趟拟合（/oid/trace 的 v_cmd/w_cmd → IMU 陀螺，起步 0–3 s，rms 0.028 rad/s）：两轮各自纯延迟
0.165 s + 二阶，左 0.95 Hz ζ0.34、右 0.72 Hz ζ0.31，右轮稳态只跑出指令的 97.2%。"""
import dataclasses
import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT)]
import heading_hold as hh

TRACK = 0.675
DT = 0.005
FITTED = dict(dL=.165, dR=.165, wL=5.98, wR=4.51, zL=.34, zR=.31, sR=.9724)
SYMMETRIC = dict(FITTED, wR=FITTED['wL'], zR=FITTED['zL'], sR=1.0)


class Wheel:
    def __init__(self, delay, wn, zeta, scale):
        self.d, self.wn, self.z, self.k = delay, wn, zeta, scale
        self.y = self.yd = 0.0
        self.buf = []

    def step(self, t, u):
        self.buf.append((t, u))
        while len(self.buf) > 1 and self.buf[1][0] <= t - self.d:
            self.buf.pop(0)
        u = self.buf[0][1] * self.k if self.buf[0][0] <= t - self.d else 0.0
        self.yd += (self.wn * self.wn * (u - self.y) - 2 * self.z * self.wn * self.yd) * DT
        self.y += self.yd * DT
        return self.y


class Car:
    """一台车一直用同一个 HeadingHold（像 web 进程那样），每趟从静止、车头 0 开始。"""

    def __init__(self, plant=FITTED, learn=True, path=None, accel=None):
        self.plant = plant
        self.t = 0.0
        cfg = hh.HoldConfig(learn_bias=learn)
        if accel is not None:
            cfg = dataclasses.replace(cfg, accel=accel)
        self.hold = hh.HeadingHold(cfg, clock=lambda: self.t, bias_path=path)
        self.n = 0

    def drive(self, seconds=8.0, stop='user', push_at=None):
        """开一趟 seconds 秒再按 stop 停；返回 [(起步后秒数, 偏差°, 指令车速)]。push_at：那一刻车头被推歪 8°。"""
        p = self.plant
        left, right = Wheel(p['dL'], p['wL'], p['zL'], 1.0), Wheel(p['dR'], p['wR'], p['zR'], p['sR'])
        self.n += 1
        yaw = rate = 0.0
        t0 = self.t
        self.hold.on_imu(0.0, 0.0, self.t)
        vx = wz = 0.0
        out = []
        k = 0
        while self.t - t0 < seconds:
            if k % 4 == 0:              # IMU、控制 50 Hz
                if push_at is not None and abs(self.t - t0 - push_at) < 1e-6:
                    yaw -= math.radians(8.0)
                self.hold.on_imu(yaw, rate, self.t)
                vx, wz = self.hold.step(self.n, self.t)
                out.append((self.t - t0, math.degrees(yaw), vx))
            vl, vr = left.step(self.t, vx - wz * TRACK / 2), right.step(self.t, vx + wz * TRACK / 2)
            rate = (vr - vl) / TRACK
            yaw += rate * DT
            self.t = round(self.t + DT, 6)
            k += 1
        if self.hold.running:
            self.hold.end(stop)
        self.t = round(self.t + 5.0, 6)     # 两趟之间停 5 s
        return out


def start_max(trace, upto=4.0):
    return max(abs(e) for t, e, v in trace if t <= upto)


def start_mean(trace, upto=4.0):
    errs = [e for t, e, v in trace if t <= upto]
    return sum(errs) / len(errs)


def steady_max(trace, after=5.0):
    return max(abs(e) for t, e, v in trace if t >= after)


class StraightStartBias(unittest.TestCase):
    def test_second_run_starts_straighter(self):
        car = Car()
        first = car.drive()
        self.assertGreater(start_max(first), 1.0)          # 还没学到：斜坡放慢了，仍往右偏 ~1.2°
        self.assertLess(start_mean(first), -0.5)
        self.assertEqual(car.hold.bias_runs, 1)
        self.assertGreater(-car.hold.bias, 0.015)          # 学到车要往左修 ~0.02 rad/s
        second = car.drive()
        self.assertLess(start_max(second), 0.7)            # ~0.5°
        self.assertLess(abs(start_mean(second)), 0.3)
        self.assertLess(steady_max(second), 0.5)           # 匀速段照样稳

    def test_old_ramp_for_reference(self):
        """原来 0.5 m/s²：第一趟 ~1.95°（实车 3.31° / 2.12°），只带修正 ~1.1°；放慢斜坡再减一半。"""
        old = Car(accel=0.5)
        first, second = old.drive(), old.drive()
        self.assertGreater(start_max(first), 1.7)
        self.assertLess(start_max(second), 1.3)
        new = Car()
        new.drive()
        self.assertLess(start_max(new.drive()), start_max(second) - 0.4)

    def test_ramp_reaches_speed_in_about_3_s(self):
        self.assertEqual(hh.HoldConfig().accel, 0.25)
        trace = Car(SYMMETRIC).drive()
        reached = next(t for t, e, v in trace if v >= hh.HoldConfig().speed)
        self.assertAlmostEqual(reached, 3.0, delta=0.05)

    def test_symmetric_car_learns_nothing(self):
        car = Car(SYMMETRIC)
        car.drive()
        self.assertLess(abs(car.hold.bias), 0.002)
        self.assertLess(start_max(car.drive()), 0.05)

    def test_survives_restart(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'sub', 'straight_bias.json')
            car = Car(path=path)
            car.drive()
            saved = json.load(open(path, encoding='utf-8'))
            self.assertEqual(saved['runs'], 1)
            self.assertAlmostEqual(saved['integral'], car.hold.bias, places=4)
            again = Car(path=path)                          # web 进程重启
            self.assertAlmostEqual(again.hold.bias, car.hold.bias, places=4)
            self.assertGreater(again.hold.status(again.t)['start_wz'], 0.015)
            self.assertLess(start_max(again.drive()), 0.7)

    def test_bad_file_starts_from_zero(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'straight_bias.json')
            Path(path).write_text('{"integral": "nan", "runs": 3}')
            self.assertEqual(Car(path=path).hold.bias, 0.0)
            Path(path).write_text('not json')
            self.assertEqual(Car(path=path).hold.bias, 0.0)
            Path(path).write_text('{"integral": 9, "runs": 2}')
            self.assertEqual(Car(path=path).hold.bias, hh.HoldConfig().i_max)

    def test_short_run_not_learned(self):
        car = Car()
        car.drive(seconds=5.5)          # 3 s 起步 + 1 s 稳住，全速平均只有 1.5 s
        self.assertEqual(car.hold.bias_runs, 0)
        self.assertEqual(car.hold.bias, 0.0)

    def test_pushed_off_not_learned(self):
        car = Car()
        car.drive(push_at=7.5)          # 全速已平均 3.5 s，够学，但被推歪那趟不学
        self.assertEqual(car.hold.reason, 'deviation')
        self.assertEqual(car.hold.bias_runs, 0)

    def test_new_run_weighs_half(self):
        car = Car()
        car.drive()
        first = car.hold.bias
        car.hold.bias = first + 0.04                        # 假装上一趟在坡上学偏了
        car.drive()
        self.assertAlmostEqual(car.hold.bias, first + 0.02, delta=0.006)

    def test_off_by_default(self):
        """定点画线内环（line_cascade）用默认 HoldConfig：不学、积分照旧从 0 起、不写文件、状态不多字段。"""
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'straight_bias.json')
            car = Car(learn=False, path=path)
            first = car.drive()
            second = car.drive()
            self.assertEqual(car.hold.bias_runs, 0)
            self.assertFalse(os.path.exists(path))
            self.assertNotIn('start_wz', car.hold.status(car.t))
            self.assertAlmostEqual(start_max(first), start_max(second), delta=0.02)   # 只差浮点


if __name__ == '__main__':
    unittest.main()
