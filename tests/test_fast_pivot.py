"""作业原地转提速（2026-11-21 用户「原地转不顺……那就不要低速，速度再提高点」）。

Gen5 作业里原地转 0.8 → 1.5 rad/s（每个轮子 0.27 → 约 0.5 m/s，避开驱动器低速跑快区），收尾减速允许到 2.0 rad/s²
（减速曲线仍按 1.0 规划），任务层渐停原地转同一个上限。规划估时、场地规划、上报 app 的 pivot_rate 仍是 0.8。
仅运行数学模型，不访问车辆。"""
from collections import deque
from dataclasses import replace
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT)]
import path_tracker as pt
from robot_core import config
from robot_core.kinematics import ChassisGeometry

DT = .02
DEG = math.pi / 180.
GEO = ChassisGeometry(drive='differential', track_m=.675, diff_min_radius_m=.5, pivot_rate_rad_s=.8,
                      pivot_accel_rad_s2=1., job_pivot_rate_rad_s=1.5, job_pivot_brake_rad_s2=2.)


def pose(yaw=0., rate=0.):
    return NS(x=0., y=0., yaw=yaw, speed=0., yaw_rate=rate, usable_for_control=True)


def turn(angle, rate, brake, delay=.35, lag=.08):
    """原地转 angle 度：延迟 + 一阶响应的车。返回 (秒, 停后差°, 冲过°, 反向次数, 最大降速 rad/s², 最大升速)。"""
    cfg = replace(pt.differential_config(pt.PathConfig()), straight_imu_enabled=False)
    tr = pt.SectionTracker([pt.Section(1, pt.Path((0., 0., angle * DEG), [(3., 0.)]))], cfg, GEO,
                           pivot_rate=rate, pivot_accel=1., pivot_brake=brake, settle_s=.2)
    assert tr.start(pose())
    sent, yaw, r, clock = deque([(0., 0.)]), 0., 0., 0.
    p, top, flips, last, prev, up, down = pose(), 0., 0, 0., 0., 0., 0.
    while clock < 30:
        _, w = tr.update(p, DT)
        if abs(w) > 1e-5:
            if last and math.copysign(1, w) != last:
                flips += 1
            last = math.copysign(1, w)
        step = (abs(w) - abs(prev)) / DT
        up, down, prev = max(up, step), max(down, -step), w
        if tr.st.state != 'RUNNING' or not tr.switching:
            break
        sent.append((clock, w))
        while len(sent) > 1 and sent[1][0] <= clock - delay + 1e-9:
            sent.popleft()
        r += (sent[0][1] - r) * DT / (lag + DT)
        yaw += r * DT
        clock += DT
        top = max(top, yaw / DEG)
        p = pose(yaw, r)
    return clock, angle - yaw / DEG, top - angle, flips, down, up


class Profile(unittest.TestCase):
    def test_gen5_profile(self):
        p = config.load_profile(ROOT / 'robot_core' / 'robot_gen5.json')
        self.assertEqual((p['drive']['pivot_rate_rad_s'], p['drive']['job_pivot_rate_rad_s']), (.8, 1.5))
        self.assertEqual((p['drive']['pivot_accel_rad_s2'], p['drive']['job_pivot_brake_rad_s2']), (1., 2.))
        self.assertGreaterEqual(p['driver']['max_yaw_rate_rad_s'], 1.5)     # 驱动层不再把 1.5 卡回 1.0

    def test_old_car_and_old_site_config_unchanged(self):
        p = config.load_profile(ROOT / 'robot_core' / 'robot.json')
        self.assertEqual((p['drive']['job_pivot_rate_rad_s'], p['drive']['job_pivot_brake_rad_s2']), (.8, 1.))
        raw = json.loads((ROOT / 'robot_core' / 'robot_gen5.json').read_text())
        for key in ('job_pivot_rate_rad_s', 'job_pivot_brake_rad_s2'):
            del raw['drive'][key]
        path = Path(self.id().replace('.', '_') + '.json')
        try:
            path.write_text(json.dumps(raw))
            old = config.load_profile(path)
        finally:
            path.unlink()
        self.assertEqual((old['drive']['job_pivot_rate_rad_s'], old['drive']['job_pivot_brake_rad_s2']), (.8, 1.))

    def test_brake_must_not_be_below_accel(self):
        with self.assertRaises(ValueError):
            pt.SectionTracker([pt.Section(1, pt.Path((0., 0., 0.), [(3., 0.)]))], pt.PathConfig(), GEO,
                              pivot_rate=1.5, pivot_accel=1., pivot_brake=.5)


class FastPivot(unittest.TestCase):
    def test_faster_and_still_stops_on_target(self):
        for delay, lag in ((.325, .05), (.35, .08), (.40, .10)):
            for angle in (45., 90., 170.):
                old = turn(angle, .8, None, delay, lag)
                new = turn(angle, 1.5, 2., delay, lag)
                key = (delay, lag, angle)
                self.assertEqual(new[3], 0, key)                     # 不反向补转
                self.assertLess(abs(new[1]), 1.5, key)               # 停后差 1.5° 以内
                self.assertLess(new[2], .5, key)                     # 冲过不到 0.5°
                self.assertLessEqual(new[0], old[0] + .05, key)      # 不比原来慢
                if angle >= 170:
                    self.assertLess(new[0], old[0] - .5, key)        # 掉头快 0.5 s 以上（90° 加速 1.0 rad/s² 到不了 1.5）

    def test_speeds_up_at_1_rad_s2_and_brakes_at_most_2(self):
        _, _, _, _, down, up = turn(170., 1.5, 2.)
        self.assertLessEqual(up, 1. + 1e-6)
        self.assertGreater(down, 1. + 1e-6)
        self.assertLessEqual(down, 2. + 1e-6)

    def test_without_extra_brake_fast_pivot_overshoots(self):
        # 只提速、刹车仍按 1.0：90° / 170° 冲过头还要反向补转（所以收尾要允许 2.0）
        self.assertTrue(any(turn(a, 1.5, None)[3] > 0 for a in (90., 170.)))


if __name__ == '__main__':
    unittest.main()
