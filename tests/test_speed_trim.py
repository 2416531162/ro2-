"""低速转速修正（oid_protocol/speed_trim.py）：模拟 11-19 架空实测的驱动器（低速跑快 1.83×），看修正后实际轮速。"""
import json
import math
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', 'oid_protocol'))

from oid_link import MotorState, WheelConfig  # noqa: E402
from speed_trim import DEFAULT_TABLE, GAIN_MAX, GAIN_MIN, SpeedTrim  # noqa: E402

W = WheelConfig(wheel_radius_m=0.194, pole_pairs=20, right_sign=1)
PER = W.erpm_per_mps()


def bad_driver(cmd):
    """实际 erpm：300 以下跑快 1.83×，以上正确（11-19 架空）。"""
    return cmd * (1.83 if abs(cmd) < 300 else 1.0)


class Plant:
    """两轮：实际转速按 bad_driver，位置 20 Hz 回读。"""

    def __init__(self, factor=bad_driver):
        self.factor = factor
        self.pos = {1: 0.0, 2: 0.0}
        self.state = {1: MotorState(), 2: MotorState()}

    def run(self, trim, target_mps, secs, t0, dt=1 / 30):
        t, k, true = t0, 0, {}
        e = int(round(target_mps * PER))
        while t < t0 + secs:
            cmd = trim.apply(t, {1: e, 2: e}, self.state)
            for m in (1, 2):
                true[m] = self.factor(cmd[m])
                self.pos[m] += true[m] / 20 / 60 * 36000 * dt
            if k % 2 == 0:
                for m in (1, 2):
                    self.state[m].position_cdeg, self.state[m].t_position = int(self.pos[m]), t
            t += dt
            k += 1
        return t, true[1] / PER if true else 0.0, cmd


class SpeedTrimTest(unittest.TestCase):
    def test_default_table_close_at_low_speed(self):
        trim = SpeedTrim(W, enabled=True, tau_s=1e9)        # 不学，只看默认表
        _, actual, _ = Plant().run(trim, 0.1, 2.0, 0.0)
        self.assertAlmostEqual(actual, 0.1, delta=0.01)      # 0.55 × 1.83 ≈ 1.0

    def test_learns_to_target(self):
        trim = SpeedTrim(W)
        plant = Plant(lambda c: c * 1.6)                     # 和默认表不一样的毛病
        t, actual, _ = plant.run(trim, 0.15, 25.0, 0.0)
        self.assertAlmostEqual(actual, 0.15, delta=0.15 * 0.03)
        self.assertGreater(trim.learned, 0)

    def test_zero_is_zero_and_no_learning_while_changing(self):
        trim = SpeedTrim(W)
        plant = Plant()
        t = 0.0
        for i in range(60):                                  # 每 0.1 s 换一次目标：一直在变，不学
            t, _, _ = plant.run(trim, 0.1 + 0.002 * i, 0.1, t)
        self.assertEqual(trim.learned, 0)
        out = trim.apply(t, {1: 0, 2: 0}, plant.state)
        self.assertEqual(out, {1: 0, 2: 0})

    def test_gain_clamped(self):
        trim = SpeedTrim(W, tau_s=0.05)
        Plant(lambda c: c * 5.0).run(trim, 0.1, 20.0, 0.0)  # 驱动器犯病：增益不会无限往下
        self.assertGreaterEqual(min(trim.gains[1]), GAIN_MIN)
        self.assertLessEqual(max(trim.gains[1]), GAIN_MAX)

    def test_reverse_and_pivot_sign(self):
        trim = SpeedTrim(W, tau_s=1e9)
        out = trim.apply(0.0, {1: -98, 2: 98}, {1: MotorState(), 2: MotorState()})
        self.assertLess(out[1], 0)
        self.assertGreater(out[2], 0)
        self.assertEqual(out[1], -out[2])

    def test_save_and_load(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'trim.json')
            trim = SpeedTrim(W, path=path, save_every_s=0)
            plant = Plant(lambda c: c * 1.6)
            t, _, _ = plant.run(trim, 0.15, 10.0, 0.0)
            trim.apply(t, {1: 0, 2: 0}, plant.state)         # 停下才存
            self.assertTrue(os.path.exists(path))
            again = SpeedTrim(W, path=path)
            self.assertEqual([round(g, 4) for g in trim.gains[1]], again.gains[1])
            json.dump({'nodes': [1, 2], 'left': [1], 'right': [1]}, open(path, 'w'))
            bad = SpeedTrim(W, path=path)                    # 表坏了：用默认值并报出来
            self.assertEqual(bad.gains[1], [g for _, g in DEFAULT_TABLE])
            self.assertIsNotNone(bad.load_error)

    def test_chassis_sends_trimmed(self):
        from oid_link import OidChassis

        class Link:
            kind = 'test'
            def __init__(self):
                self.state = {1: MotorState(), 2: MotorState()}
                self.cmds = []
            def heartbeat(self, now, force=False): pass
            def command(self, erpm): self.cmds.append(dict(erpm))
            def request(self, now): pass
            def poll(self, now): pass
            def configure(self, *a, **k): pass

        link = Link()
        ch = OidChassis(link, W, clock=lambda: 0.0, speed_trim=SpeedTrim(W, tau_s=1e9))
        ch.step(0.0, 0.1, 0.0)
        target = W.motor_erpm(0.1, 0.0)
        self.assertEqual(ch.target, target)
        self.assertAlmostEqual(link.cmds[-1][1], target[1] * 0.55, delta=1)
        tele = ch.telemetry(0.0)
        self.assertEqual(tele['erpm_target'], [target[1], target[2]])
        self.assertTrue(tele['speed_trim']['enabled'])
        plain = OidChassis(Link(), W, clock=lambda: 0.0)
        plain.step(0.0, 0.1, 0.0)
        self.assertEqual(plain.link.cmds[-1], target)

    def test_disabled_still_measures(self):
        """修正关着：下发 = 目标、不学，但照样按位置算真实轮速（/oid/trace 的 erpm_true）。"""
        trim = SpeedTrim(W, enabled=False)
        plant = Plant(lambda c: c * 1.6)
        _, actual, cmd = plant.run(trim, 0.15, 5.0, 0.0)
        e = int(round(0.15 * PER))
        self.assertEqual(cmd, {1: e, 2: e})
        self.assertAlmostEqual(actual, 0.15 * 1.6, delta=0.01)
        self.assertEqual(trim.learned, 0)
        self.assertEqual(trim.gains[1], [g for _, g in DEFAULT_TABLE])
        st = trim.status()
        self.assertFalse(st['enabled'])
        for m in (0, 1):
            self.assertAlmostEqual(st['measured_erpm'][m], e * 1.6, delta=e * 1.6 * 0.03)


class TableMigrationTest(unittest.TestCase):
    def test_old_table_resampled_onto_new_nodes(self):
        old = {'nodes': [0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.8, 1.2],
               'left': [0.55, 0.584, 0.527, 0.702, 1.0, 1.0, 1.0, 1.0],
               'right': [0.55, 0.579, 0.504, 0.489, 1.0, 1.0, 1.0, 1.0]}
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'trim.json')
            json.dump(old, open(path, 'w'))
            trim = SpeedTrim(W, path=path, save_every_s=0)
            self.assertIsNone(trim.load_error)
            self.assertEqual(trim.nodes, [n for n, _ in DEFAULT_TABLE])
            self.assertAlmostEqual(trim.gain(2, 0.3), 0.489, places=3)       # 原节点上的值不变
            self.assertAlmostEqual(trim.gain(2, 0.35), 0.489 + 0.5 * (1 - 0.489), places=3)
            trim.apply(0.0, {1: 0, 2: 0}, {})                                # 停着：按新节点存回
            self.assertEqual(json.load(open(path))['nodes'], trim.nodes)

    def test_step_near_switch_learned(self):
        """驱动器 0.32 m/s 以下跑快 1.83×、以上正确：0.33 m/s 学到接近 100%，0.3 m/s 也不被带偏。"""
        plant = Plant(lambda c: c * (1.83 if abs(c) < 0.32 * PER else 1.0))
        trim = SpeedTrim(W)
        t, a33, _ = plant.run(trim, 0.331, 20.0, 0.0)
        t, a30, _ = plant.run(trim, 0.30, 20.0, t)
        t, b33, _ = plant.run(trim, 0.331, 10.0, t)
        self.assertAlmostEqual(a30, 0.30, delta=0.30 * 0.04)
        self.assertAlmostEqual(b33, 0.331, delta=0.331 * 0.04)


if __name__ == '__main__':
    unittest.main()


class CommonLookupTest(unittest.TestCase):
    """2026-11-19 用户「改成同一个增益」：两轮按平均速度查表，跟线的小转向不被台阶放大（04:28 倒车 0.35 m/s 左右摆）。"""

    def send(self, left_mps, right_mps):
        trim = SpeedTrim(W, tau_s=1e9)
        st = {1: MotorState(), 2: MotorState()}
        e = {1: int(round(left_mps * PER)), 2: int(round(right_mps * PER))}
        return e, trim.apply(0.0, e, st)

    def test_small_steer_on_the_step_not_amplified(self):
        # 04:28 那一下：倒车 0.35 m/s，两轮目标约 0.33 / 0.37
        e, out = self.send(-0.37, -0.33)
        want = e[2] - e[1]
        got = out[2] - out[1]
        self.assertAlmostEqual(got / want, 1.0, delta=0.05)      # 改之前约 3 倍
        self.assertAlmostEqual(out[1] / e[1], out[2] / e[2], delta=0.01)   # 同一个增益

    def test_equal_speeds_same_as_table(self):
        for v in (0.1, 0.3, 0.33, 0.5):
            with self.subTest(v=v):
                trim = SpeedTrim(W, tau_s=1e9)
                e, out = self.send(v, v)
                self.assertAlmostEqual(out[1], e[1] * trim.gain(1, v), delta=1)

    def test_arc_wheels_far_apart_keep_own_speed(self):
        # 圆弧内轮 0.1、外轮 0.5：内轮仍是低速增益（约 0.55），外轮仍约 1.0，不被平均成 0.3 的增益
        e, out = self.send(0.1, 0.5)
        self.assertAlmostEqual(out[1] / e[1], 0.55, delta=0.02)
        self.assertAlmostEqual(out[2] / e[2], 1.0, delta=0.02)

    def test_lookup_continuous(self):
        prev = None
        for k in range(0, 41):
            d = k * 0.005
            look = SpeedTrim.lookup_mps({1: 0.35 - d, 2: 0.35 + d})
            if prev:
                self.assertLess(abs(look[1] - prev[1]), 0.006)
                self.assertLess(abs(look[2] - prev[2]), 0.006)
            prev = look

    def test_pivot_both_same(self):
        e, out = self.send(-0.3, 0.3)
        self.assertEqual(out[1], -out[2])
