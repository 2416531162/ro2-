"""开机后作业前的行驶里顺带学车头换算 c（line_cascade.DriveLearner / ImuAlignment.adopt_drive）
（2026-11-25 用户「开机，自动还有多跑一会学习，这个能不能省去」→「改，先仿真验证」）：

- 往前、倒车（按轮速分）、转弯（减主天线杆臂随转弯的修正）都学得准；原地转、太快的转弯、非固定解不用；
- 学够 DRIVE_LEARN_MIN_M 才算；只算最近 DRIVE_LEARN_KEEP_S 内的；
- 车停着 IMU 偏航自己跳（固件静止修正）学到的跟着挪；IMU 断过清空；
- 任务层：没有记忆但行驶里学够了 → 当记忆用，开机第一条不空跑、不按没有记忆走；学不够照旧空跑；有记忆不动。
"""
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT), str(ROOT / 'tests')]
import route_plan as rp
import user_rules as rules
from line_cascade import DriveLearner, ImuAlignment, wrap
from line_mission import IdleTracker, LineMission
from path_tracker import PathConfig, differential_config
from robot_core.kinematics import ChassisGeometry

LEVER = (.235, .205)                 # 主天线（前、左）
C_TRUE = math.radians(-158.2)        # 真实车头 − IMU 偏航


def drive(learner, t0=0., x=0., y=0., heading=math.radians(30.), speed=.5, rate_deg_s=0., seconds=10.,
          dt=.05, imu_dt=.01, wheel_dt=.02, fixed=True, c=C_TRUE, lever=LEVER, align=None):
    """车体（后轴中心）按 speed（倒车为负）、rate 开；喂 IMU 偏航（= 车头 − c）、轮速、主天线位置。返回终点 (t, x, y, heading)。"""
    t, w = t0, math.radians(rate_deg_s)
    nxt_fix = nxt_wheel = t0
    while t < t0 + seconds - 1e-9:
        t += imu_dt
        mid = heading + .5 * w * imu_dt                         # 中点积分：车体轨迹本身不带半步的角度偏
        x += speed * math.cos(mid) * imu_dt
        y += speed * math.sin(mid) * imu_dt
        heading += w * imu_dt
        yaw = wrap(heading - c)
        if align is not None:
            align.feed_imu(t, yaw)
        else:
            raw = getattr(learner, '_test_raw', None)
            learner._test_raw = yaw if raw is None else raw + wrap(yaw - wrap(raw))
            learner.on_imu(t, learner._test_raw)
        if t >= nxt_wheel - 1e-9:
            nxt_wheel += wheel_dt
            learner.on_wheel(t, speed)
        if t >= nxt_fix - 1e-9:
            nxt_fix += dt
            ax = x + lever[0] * math.cos(heading) - lever[1] * math.sin(heading)
            ay = y + lever[0] * math.sin(heading) + lever[1] * math.cos(heading)
            learner.on_fix(t, ax, ay, fixed, lever)
    return t, x, y, heading


class Learner(unittest.TestCase):
    def check(self, est, metres):
        self.assertIsNotNone(est)
        c, sigma, dist = est
        self.assertLess(abs(math.degrees(wrap(c - C_TRUE))), .02)
        self.assertAlmostEqual(dist, metres, delta=.6)
        return sigma

    def test_forward_straight(self):
        d = DriveLearner()
        t, *_ = drive(d, seconds=10.)
        self.check(d.estimate(t), 5.)

    def test_reverse(self):
        d = DriveLearner()
        t, *_ = drive(d, speed=-.4, seconds=10.)
        self.check(d.estimate(t), 4.)

    def test_turning_with_lever_arm(self):
        d = DriveLearner()
        t, *_ = drive(d, rate_deg_s=8., seconds=12.)          # 半径 3.6 m 的弯，转 96°
        self.check(d.estimate(t), 6.)
        # 不减杆臂修正会错 ~4°：证明修正真在起作用
        d3 = DriveLearner()
        d3.on_fix = (lambda f: lambda t_, x_, y_, ok, lever: f(t_, x_, y_, ok, (0., 0.)))(d3.on_fix)
        drive(d3, rate_deg_s=8., seconds=12.)
        self.assertGreater(abs(math.degrees(wrap(d3.estimate(t)[0] - C_TRUE))), 2.)

    def test_position_jump_breaks_the_chain(self):
        d = DriveLearner()
        t, x, y, h = drive(d, seconds=10.)
        n = len(d.samples)
        d.on_fix(t + .05, x + 1., y, True, LEVER)                  # 跳 1 m
        self.assertEqual(len(d.fixes), 1)
        self.assertEqual(len(d.samples), n)

    def test_pivot_and_slow_and_float_are_not_used(self):
        d = DriveLearner()
        t, *_ = drive(d, speed=.02, rate_deg_s=30., seconds=10.)    # 原地转
        t, *_ = drive(d, t0=t, speed=.1, seconds=10.)              # 太慢
        t, *_ = drive(d, t0=t, seconds=10., fixed=False)           # 不是固定解
        self.assertEqual(len(d.samples), 0)
        self.assertIsNone(d.estimate(t))

    def test_too_short_is_none(self):
        d = DriveLearner()
        t, x, y, h = drive(d, seconds=5.)                          # 2.5 m（头 0.4 m 凑不齐前后两帧，算 2.1 m）
        self.assertIsNone(d.estimate(t))
        t, *_ = drive(d, t0=t, x=x, y=y, heading=h, seconds=3.)    # 再 1.5 m：够 3 m
        self.check(d.estimate(t), 3.6)

    def test_only_recent_counts(self):
        d = DriveLearner()
        t, *_ = drive(d, seconds=10.)
        self.assertIsNotNone(d.estimate(t))
        self.assertIsNone(d.estimate(t + rules.DRIVE_LEARN_KEEP_S + 11.))

    def test_noise_gives_sigma_within_gate(self):
        import random
        rnd = random.Random(3)
        d = DriveLearner()
        f = d.on_fix
        d.on_fix = lambda t_, x_, y_, ok, lever: f(t_, x_ + rnd.gauss(0, .004), y_ + rnd.gauss(0, .004), ok, lever)
        t, *_ = drive(d, seconds=10.)                              # 5 m、位置噪声 4 mm
        c, sigma, _ = d.estimate(t)
        self.assertLess(abs(math.degrees(wrap(c - C_TRUE))), .3)
        self.assertLessEqual(math.degrees(sigma), rules.DRIVE_LEARN_MAX_SIGMA_DEG)


class Alignment(unittest.TestCase):
    def test_still_jump_moves_learned_c(self):
        a = ImuAlignment()
        t, x, y, h = drive(a.drive, align=a, seconds=10.)
        # 停着：轮速 0，IMU 偏航自己跳 +4°（固件静止修正）
        yaw = wrap(h - C_TRUE)
        for k in range(100):
            t += .01
            a.drive.on_wheel(t, 0.)
            a.feed_imu(t, wrap(yaw + (math.radians(4.) if k > 50 else 0.)), still=True)
        c, _, _ = a.drive.estimate(t)
        self.assertAlmostEqual(math.degrees(wrap(c - (C_TRUE - math.radians(4.)))), 0., delta=.03)

    def test_imu_gap_clears(self):
        a = ImuAlignment()
        t, x, y, h = drive(a.drive, align=a, seconds=10.)
        a.feed_imu(t + 5., 0.)                                     # IMU 断了 5 s
        self.assertIsNone(a.drive.estimate(t + 5.))

    def test_adopt_only_without_memory(self):
        a = ImuAlignment()
        self.assertIsNone(a.adopt_drive())                         # 没学
        t, x, y, h = drive(a.drive, align=a, seconds=10.)
        self.assertFalse(a.remembered())
        est = a.adopt_drive()
        self.assertIsNotNone(est)
        self.assertTrue(a.remembered())
        c, sigma = a.predict(t)
        self.assertLess(abs(math.degrees(wrap(c - C_TRUE))), .02)
        self.assertEqual(a.status()['align_adopted_m'], round(est[2], 2))
        # 已经有记忆（上一条线拟合的）：不换
        a2 = ImuAlignment()
        a2.feed_imu(1., 0.)
        a2.update_line(a2.begin_line(), 1., .5, math.radians(.05))
        t2, *_ = drive(a2.drive, align=a2, t0=1., seconds=10.)
        self.assertIsNone(a2.adopt_drive())
        self.assertAlmostEqual(a2.pending[2], .5)


GEO = ChassisGeometry(drive='differential', track_m=.675, diff_min_radius_m=.5)
NOZZLE = (.24, -.4575)
LINE = rp.RouteSpec(kind='path', points=((0., 0.), (12., 0.)), closed=False, cruise=.8, entry_space_m=3.)   # 后面有 3 m 空地才绕得回来空跑


class Mission(unittest.TestCase):
    def mission(self, folder):
        car = SimpleNamespace(to_geodetic=lambda x, y: (30. + y * 1e-5, 120. + x * 1e-5),
                              to_local=lambda lat, lon: ((lon - 120.) / 1e-5, (lat - 30.) / 1e-5),
                              antenna_lever=lambda: LEVER)
        return LineMission(car, SimpleNamespace(), IdleTracker(GEO), route_path=str(Path(folder) / 'route.json'),
                           path_config=differential_config(PathConfig(cruise_mps=.8)), nozzle_x=NOZZLE)

    def plan(self, m, pose):
        sections, _, tracker, info = m._plan_route(LINE, pose, rp.plan_current, dict(forward=NOZZLE[0], right=-NOZZLE[1]), .8)
        return sections, info

    def test_learned_while_driving_skips_the_dry_run(self):
        pose = SimpleNamespace(x=-6., y=3., yaw=0.)
        with tempfile.TemporaryDirectory() as folder:
            m = self.mission(folder)
            sections, info = self.plan(m, pose)
            self.assertTrue(any(getattr(s, 'learn', False) for s in sections))     # 没学：照旧空跑
            self.assertIn('learn_m', info)
            m = self.mission(folder)
            a = m._imu_alignment
            # 开机后手动开了 5 m（喂任务层：卫星走 on_gnss 的同一条路）
            drive(a.drive, align=a, seconds=10.)
            self.assertTrue(m.first_line_no_memory())
            sections, info = self.plan(m, pose)
            self.assertFalse(any(getattr(s, 'learn', False) for s in sections))    # 不空跑
            self.assertEqual(info['learn_m'], 0.)
            self.assertAlmostEqual(info['drive_learned_m'], 5., delta=.6)
            self.assertFalse(any(getattr(s, 'first_line', False) for s in sections))  # 有记忆：不按没有记忆拟合
            self.assertFalse(m.first_line_no_memory())

    def test_short_drive_keeps_the_dry_run(self):
        pose = SimpleNamespace(x=-6., y=3., yaw=0.)
        with tempfile.TemporaryDirectory() as folder:
            m = self.mission(folder)
            a = m._imu_alignment
            drive(a.drive, align=a, seconds=4.)                    # 2 m 不够
            sections, info = self.plan(m, pose)
            self.assertTrue(any(getattr(s, 'learn', False) for s in sections))
            self.assertNotIn('drive_learned_m', info)

    def test_gnss_fix_feeds_the_learner(self):
        with tempfile.TemporaryDirectory() as folder:
            m = self.mission(folder)
            a = m._imu_alignment
            feed = []
            a.drive.on_fix = lambda *args: feed.append(args)
            m._drive_fix(dict(t=1., lat=30. + 2e-5, lon=120. + 1e-5, sigma_n=.01, sigma_e=.01), True)
            m._drive_fix(dict(t=1.05, lat=30., lon=120., sigma_n=.02, sigma_e=.02), True)     # σ 2.8 cm：断开
            m._drive_fix(dict(t=1.1, lat=30., lon=120., sigma_n=.01, sigma_e=.01), False)     # 不是固定解：断开
            self.assertEqual([(f[0], round(f[1], 6), round(f[2], 6), f[3], f[4]) for f in feed],
                             [(1., 1., 2., True, LEVER), (1.05, 0., 0., False, LEVER), (1.1, 0., 0., False, LEVER)])


if __name__ == '__main__':
    unittest.main()
