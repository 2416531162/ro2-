"""原地转回弹补偿（11-27 用户「做」，path_tracker.PivotRebound，user_rules.PIVOT_REBOUND_*）。

c30c07fd 实测 7 趟：原地转停下后车头往回弹 1.4–3.3°，加上刹得早一点，停稳后没转够 1.5–4.9°（中位 2.8°）；
差在 PIVOT_MIN_DEG 以内不补转，全留给 2 m 助跑，每趟起步都有点歪。现在按左 / 右转各学一个补偿量、多转这么多。
底盘用 test_pivot_slope_recovery.PivotPlant（延迟 + 一阶响应 + 有限回弹），Gen5 原地转 1.5 rad/s、刹车 2.0 rad/s²。"""
from dataclasses import replace
import math
from pathlib import Path
import random
import statistics as st
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT), str(ROOT / 'tests')]
import path_tracker as pt  # noqa: E402
import user_rules as rules  # noqa: E402
from test_pivot_slope_recovery import DT, GEO, PivotPlant, pose  # noqa: E402
import test_launch_settle as launch  # noqa: E402

DEG = math.pi / 180.


def tracker(angle, rebound=None):
    cfg = replace(pt.differential_config(pt.PathConfig()), straight_imu_enabled=False)
    tr = pt.SectionTracker([pt.Section(1, pt.Path((0., 0., angle * DEG), [(3., 0.)]))], cfg, GEO,
                           pivot_rate=1.5, pivot_accel=1., pivot_brake=2., settle_s=.2)
    if rebound is not None:
        tr.share_pivot_rebound(rebound)
    assert tr.start(pose())
    return tr


def pivot(angle, rebound=None, **plant):
    """原地转一次：返回 (停稳后没转够多少 °（没转够为正）, 用了几次, 状态, tracker)。"""
    tr, car = tracker(angle, rebound), PivotPlant(**plant)
    p = pose()
    for _ in range(int(40 / DT)):
        tr.update(p, DT)
        if tr.st.state != 'RUNNING' or not tr.switching:
            break
        p = car.step(tr._w)
    return math.degrees(pt.wrap(angle * DEG - p.yaw)) * math.copysign(1., angle), tr._turns, tr.st.state, tr


class Learner(unittest.TestCase):
    def test_rules_table_and_checks(self):
        rb = pt.PivotRebound()
        self.assertAlmostEqual(rb.status()['left_deg'], rules.PIVOT_REBOUND_PRIOR_DEG)
        self.assertAlmostEqual(rb.status()['right_deg'], rules.PIVOT_REBOUND_PRIOR_DEG)
        self.assertEqual((rb.gain, round(math.degrees(rb.max), 9), round(math.degrees(rb.outlier), 9)),
                         (rules.PIVOT_REBOUND_GAIN, rules.PIVOT_REBOUND_MAX_DEG, rules.PIVOT_REBOUND_OUTLIER_DEG))
        for kw in (dict(gain=0.), dict(gain=1.5), dict(prior_deg=-1.), dict(prior_deg=5., max_deg=4.), dict(outlier_deg=math.nan)):
            with self.assertRaises(ValueError):
                pt.PivotRebound(**kw)

    def test_learns_each_direction_separately_and_clamps(self):
        rb = pt.PivotRebound(prior_deg=2.)
        rb.learn(1, 2. * DEG, 1. * DEG)              # 左转多转 2° 还差 1°：补偿 2.5°
        self.assertAlmostEqual(math.degrees(rb.compensation(1)), 2.5)
        self.assertAlmostEqual(math.degrees(rb.compensation(-1)), 2.)
        rb.learn(-1, 2. * DEG, -6. * DEG)            # 右转转过了 6°：降到 0（不出负）
        self.assertEqual(rb.compensation(-1), 0.)
        rb.learn(1, 4. * DEG, 3. * DEG)              # 上限 4°
        self.assertAlmostEqual(math.degrees(rb.compensation(1)), rules.PIVOT_REBOUND_MAX_DEG)
        self.assertEqual(rb.status()['left_n'], 2)

    def test_outlier_is_not_learned(self):
        rb = pt.PivotRebound(prior_deg=2.)
        rb.learn(1, 2. * DEG, 9. * DEG)              # 被推、打滑
        self.assertAlmostEqual(math.degrees(rb.compensation(1)), 2.)
        self.assertTrue(rb.status()['last']['rejected'])
        self.assertEqual(rb.status()['left_n'], 0)


class Pivot(unittest.TestCase):
    def test_without_learner_turns_exactly_as_before(self):
        # 任务层没给（单元测试、纯几何调用）：不补偿
        short, turns, state, tr = pivot(25., spring_deg=2.5)
        self.assertEqual(tr._pivot_comp, 0.)
        self.assertIsNone(tr.status()['pivot_rebound'])
        self.assertGreater(short, 2.)                # 现场那样没转够

    def test_rebound_is_learned_within_a_few_pivots(self):
        # 回弹 2.5°（现场中位）：不补偿每次都差约 2.7°；补偿第一次就小一截，第 3 次起 ≤ 0.3°
        rb = pt.PivotRebound()
        old = [pivot(a, spring_deg=2.5)[0] for a in (25., 23., 12., 25., 128.)]
        new = [pivot(a, rb, spring_deg=2.5)[0] for a in (25., 23., 12., 25., 128.)]
        self.assertTrue(all(x > 2.3 for x in old), old)
        self.assertLess(new[0], .5 * old[0])
        self.assertTrue(all(abs(x) < .3 for x in new[2:]), new)
        self.assertAlmostEqual(rb.status()['left_deg'], 2.6, delta=.2)
        self.assertEqual(rb.status()['right_deg'], rules.PIVOT_REBOUND_PRIOR_DEG)   # 没右转过：照旧

    def test_varying_rebound_on_both_directions(self):
        # 回弹每次在 1.4–3.3° 里变（现场范围），左右转都有：学几次后平均差 < 0.5°、最差 < 1.5°（原来平均 2.5°）
        rng = random.Random(1)
        rb, old, new = pt.PivotRebound(), [], []
        for _ in range(20):
            a = rng.choice((25., 23., 12., 128.)) * rng.choice((1., -1.))
            spring = rng.uniform(1.4, 3.3)
            old.append(pivot(a, spring_deg=spring)[0])
            new.append(pivot(a, rb, spring_deg=spring)[0])
        self.assertGreater(st.mean(old), 2.)
        self.assertLess(abs(st.mean(new[6:])), .5)
        self.assertLess(max(abs(x) for x in new[6:]), 1.5)

    def test_no_rebound_car_unlearns_without_extra_turns(self):
        # 没有回弹的车：开机按 2° 先多转，一次转完（不补转、不中止），几次后补偿退下来
        rb = pt.PivotRebound()
        rows = [pivot(a, rb)[:3] for a in (25., 25., 25., 25., 25.)]
        self.assertTrue(all(turns == 1 and state == 'RUNNING' for _, turns, state in rows), rows)
        self.assertGreater(-rows[0][0], 1.)          # 第一次转过了
        self.assertLess(abs(rows[-1][0]), .3)
        self.assertLess(rb.status()['left_deg'], .5)

    def test_slope_load_still_finishes_in_one_attempt(self):
        # 坡上起动阻力 + 大回弹（同 test_pivot_slope_recovery）：照样一次转完、不超时
        rb = pt.PivotRebound()
        for angle in (-90., -10., 10., 90.):
            with self.subTest(angle=angle):
                short, turns, state, _ = pivot(angle, rb, breakaway=.4, spring_deg=6.)
                self.assertEqual((turns, state), (1, 'RUNNING'))
                self.assertLessEqual(abs(short), 5.)

    def test_status_reports_compensation(self):
        rb = pt.PivotRebound()
        tr = tracker(25., rb)
        tr.update(pose(), DT)
        tr.update(pose(), DT)
        for _ in range(int(1. / DT)):
            tr.update(pose(), DT)
            if tr.status()['pivoting']:
                break
        self.assertAlmostEqual(tr.status()['pivot_comp_deg'], rules.PIVOT_REBOUND_PRIOR_DEG)
        self.assertEqual(tr.status()['pivot_rebound']['left_deg'], rules.PIVOT_REBOUND_PRIOR_DEG)


class LineHead(unittest.TestCase):
    def test_line_head_is_not_worse(self):
        # 线头规则（用户）：线头精度不能变差。助跑起点车头误差：原来现场 1.5–4.9°（都没转够），补偿后按上面仿真 ±0.5° 内；
        # 同一助跑模型（test_launch_settle，轮子欠阻尼、左右轮起动先后差）量喷线起点后 2 m 喷头最大横偏
        cases = [(seed, e0) for seed in range(6) for e0 in (-.02, .02)]
        old = [launch.run(seed, e0, -h0, settle=True)['head'] for seed, e0 in cases for h0 in (1.5, 2.4, 2.8, 4.0, 4.9)]
        new = [launch.run(seed, e0, h0, settle=True)['head'] for seed, e0 in cases for h0 in (-.5, 0., .5)]
        self.assertLessEqual(st.median(new), st.median(old))
        self.assertLessEqual(max(new), max(old) + .2)


if __name__ == '__main__':
    unittest.main()


class Mission(unittest.TestCase):
    def test_mission_shares_one_learner_across_jobs(self):
        # 任务层持有一个学习器，每拍给当前跟踪器；换一个作业（新的 SectionTracker）接着用同一个
        import tempfile
        from types import SimpleNamespace
        from line_mission import IdleTracker, LineMission
        car = SimpleNamespace(to_geodetic=lambda x, y: (30. + y * 1e-5, 120. + x * 1e-5),
                              to_local=lambda lat, lon: ((lon - 120.) / 1e-5, (lat - 30.) / 1e-5))
        with tempfile.TemporaryDirectory() as folder:
            m = LineMission(car, SimpleNamespace(), IdleTracker(GEO), route_path=str(Path(folder) / 'route.json'),
                            path_config=pt.differential_config(pt.PathConfig(cruise_mps=.8)), nozzle_x=(.24, -.4575))
            self.assertIsInstance(m._pivot_rebound, pt.PivotRebound)
            first, second = tracker(25.), tracker(-25.)
            m._sync_imu(first, 1.)
            m._sync_imu(second, 2.)
            self.assertIs(first._rebound, m._pivot_rebound)
            self.assertIs(second._rebound, m._pivot_rebound)
