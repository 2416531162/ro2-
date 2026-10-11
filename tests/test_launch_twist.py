"""起步扭头（2026-10-11 用户「解决起步问题」）。

实录 3 天 37 次直线起步：起步前最后一下是原地转（一边轮子往前、一边往后），起步时要换向的那个轮子晚动 0.1–0.5 s，
车头往它那边扭 4–5°（18:34 那趟扭 5.6°，到线头喷头偏 6 cm，被起喷门槛挡掉 0.72 m 线头）；两轮一起换向（直线倒车后）
或都不换向时在 2° 以内。

1. 规划（现在关着，RUNUP_BACK_M = 0）：助跑起点要原地转正时，改在前方 RUNUP_BACK_M 处转正、再直线倒回助跑起点，
   起步时两轮一起换向。测试按打开时（1.0 m）检查。
2. 作业第一个线头没过起喷门槛：渐停、退回去重新助跑（与接入时偏离同一套），不再跳过线头往前喷。"""
from dataclasses import replace
from pathlib import Path
import math
import sys
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT), str(ROOT / 'tests')]
import line_mission as lm
import paint_quality
import route_plan as rp
from line_mission import LineMission
from line_tracker import LineConfig, LineTracker
from path_tracker import PathConfig, differential_config
from test_offline_reentry import DT, GEO, NOZZLE, Car, mission, run
from types import SimpleNamespace

LINE = rp.RouteSpec(kind='path', points=((0., 0.), (12., 0.)), closed=False, cruise=.8)
TRACK = .675
DEAD_S = .35         # 实录：换向的轮子比另一个晚动 0.1–0.5 s（编码器不动），18:34 那次 0.36 s


def line_car(q, side=0., yaw=0.):
    """喷头在 (q, side)、车头 yaw 时的后轴位姿。"""
    return rp.axle_for((q, side), yaw, *NOZZLE) + (yaw,)


def headings(sections, car_yaw):
    """[(方向, 长度, 这一段前要原地转的角度°)]"""
    out, yaw = [], car_yaw
    for s in sections:
        h = s.path.pose(0.)[2]
        head = h if s.direction > 0 else rp.wrap(h + math.pi)
        out.append((s.direction, s.path.length, math.degrees(rp.wrap(head - yaw))))
        end = s.path.pose(s.path.length)[2]
        yaw = end if s.direction > 0 else rp.wrap(end + math.pi)
    return out


# 刚喷完旁边一条反方向的线、停在新线头附近（18:34 那趟）：转过去倒到助跑起点，到了还要转约 10°
BESIDE = line_car(.3, .25, math.pi)


BACK_ON = 1.0        # 打开时的回倒长度


class RunupPlan(unittest.TestCase):
    def test_off_by_default_plan_unchanged(self):
        # 2026-10-11 用户：这次先不带
        self.assertEqual(rp.RUNUP_BACK_M, 0.)
        secs, _ = rp.plan_current(LINE, BESIDE, .556, nozzle_x=NOZZLE)
        *_, (d2, _, t2), (d3, _, _) = headings(secs, BESIDE[2])
        self.assertEqual((d2, d3), (-1, 1))
        self.assertGreater(abs(t2), 150.)          # 转过去倒到助跑起点，到了再转正就起步（原来的走法）

    @mock.patch.object(rp, 'RUNUP_BACK_M', BACK_ON)
    def test_pivot_at_runup_start_then_straight_reverse_into_it(self):
        for car in (BESIDE, line_car(-6., 3., math.pi / 2)):
            secs, entry = rp.plan_current(LINE, car, .556, nozzle_x=NOZZLE)
            moves = headings(secs, car[2])
            with self.subTest(car=car, moves=moves):
                *_, (d2, l2, t2), (d3, _, t3) = moves
                self.assertGreater(abs(t2), 5.)                     # 先原地转正（真要转）
                self.assertEqual((d2, d3), (-1, 1))
                self.assertAlmostEqual(l2, rp.RUNUP_BACK_M)          # 再直线倒回助跑起点
                self.assertLess(abs(t3), 1e-6)                       # 倒完不再转，直接往前起步
                paint = secs[-1]
                self.assertAlmostEqual(paint.paint_from_m, rp.RUN_UP_M)
                gx, gy, gh = paint.path.pose(0.)
                bx, by, _ = secs[-2].path.pose(secs[-2].path.length)
                self.assertAlmostEqual(math.dist((gx, gy), (bx, by)), 0., places=6)
                self.assertAlmostEqual(entry, sum(s.path.length for s in secs[:-1]) + rp.RUN_UP_M, places=6)

    @mock.patch.object(rp, 'RUNUP_BACK_M', BACK_ON)
    def test_no_extra_reverse_when_runup_start_needs_no_pivot(self):
        # 车头已经顺着线、在线头附近：直接倒到助跑起点、往前，最后一下本来就是直线倒车
        for car in (line_car(.1), line_car(.1, 0., math.radians(2.)), line_car(-.8)):
            secs, _ = rp.plan_current(LINE, car, .556, nozzle_x=NOZZLE)
            self.assertEqual([s.direction for s in secs], [-1, 1], headings(secs, car[2]))

    def test_run_up_is_three_point_five(self):
        self.assertEqual(rp.RUN_UP_M, 3.5)


class WheelDeadCar(Car):
    """每个轮子要换方向时（指令与它上次动的方向相反）先不动 DEAD_S 秒，另一个照常（实录的起步扭头）。"""
    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.last = [0, 0]
        self.dead = [0., 0.]

    def step(self, vx, wz):
        wheels = [vx - wz * TRACK / 2, vx + wz * TRACK / 2]
        for i, w in enumerate(wheels):
            sign = (w > 1e-4) - (w < -1e-4)
            if sign and self.last[i] and sign != self.last[i] and self.dead[i] <= 0.:
                self.dead[i] = DEAD_S
                self.last[i] = sign
            elif sign and not self.last[i]:
                self.last[i] = sign
            if self.dead[i] > 0.:
                self.dead[i] -= DT
                wheels[i] = 0.
        super().step((wheels[0] + wheels[1]) / 2, (wheels[1] - wheels[0]) / TRACK)


def launch(back_m):
    """BESIDE 起步、助跑段头 1.5 m 车头差最大值（°）和到线头时的横偏（cm）。"""
    with tempfile.TemporaryDirectory() as folder, mock.patch.object(rp, 'RUNUP_BACK_M', back_m):
        x, y, h = BESIDE
        car = WheelDeadCar(x, y, h)
        m = mission(car, folder)
        paint = len(m.tracker.sections) - 1
        worst, head = 0., None
        for i in range(int(90. / DT)):
            vx, wz, _ = m.tick(i * DT, 'NAVIGATION')
            car.step(vx, wz)
            t = m.tracker
            if m.phase == 'RUNNING' and t.index == paint and not t.switching:
                st = t.trackers[paint].st
                if .05 < st.s < 1.5:
                    worst = max(worst, abs(math.degrees(st.heading_err or 0.)))
                if head is None and st.s >= rp.RUN_UP_M:
                    head = abs(st.e or 0.) * 100
                    break
        return worst, head


class LaunchTwist(unittest.TestCase):
    def test_reverse_after_pivot_removes_launch_twist(self):
        old_twist, old_head = launch(0.)
        new_twist, new_head = launch(BACK_ON)
        self.assertGreater(old_twist, 4.)            # 模型复现实录：转完直接起步扭 4° 以上
        self.assertLess(new_twist, 1.5)
        self.assertLess(new_head, old_head)
        self.assertLess(new_head, 1.)


class RedoRunup(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.folder.cleanup()

    def test_first_head_off_line_backs_up_and_runs_up_again(self):
        car = Car(*line_car(-6., 2., 0.))
        m = mission(car, self.folder.name)
        job = m.operations.record['id']
        # 第一次到线头：门槛设成 0，一定挡住
        with mock.patch.object(paint_quality, 'PAINT_START_MAX_OFFSET_M', 0.):
            messages = run(m, car, 60., stop=lambda m: m._reentry is not None)
        self.assertTrue(m.message.startswith('到线头时喷头离线'), messages)
        self.assertEqual(m.tracker.st.reason, 'paint_start_off_line')
        self.assertFalse(any(i['on'] for i in m.operations._paint_marks))
        messages += run(m, car, 120.)
        self.assertEqual(m.message, '路线完成', messages)
        self.assertFalse(any(t.startswith('中止') for t in messages), messages)
        self.assertTrue(any(t.startswith('已退回去重新助跑') for t in messages), messages)
        r = m.operations.record
        self.assertEqual((r['id'], r['state'], r['reentries']), (job, 'DONE', 1))
        # 喷线从线头开始（不是过了线头才开）
        on = [i for i in m.operations._paint_marks if i['on']]
        self.assertEqual(len(on), 1)
        self.assertLess(abs(on[0]['x']), .05)

    def test_after_max_redos_keeps_driving_without_paint_as_before(self):
        car = Car(*line_car(-6., 2., 0.))
        m = mission(car, self.folder.name)
        m.operations.record['reentries'] = lm.REENTRY_MAX
        with mock.patch.object(paint_quality, 'PAINT_START_MAX_OFFSET_M', 0.):
            messages = run(m, car, 40., stop=lambda m: m.message.startswith(lm.PAINT_START_OFF_PREFIX))
        self.assertEqual(m.phase, 'RUNNING', messages)
        self.assertEqual(m.tracker.st.state, 'RUNNING')
        self.assertIsNone(m._reentry)

    def test_later_head_is_not_redone(self):
        # 拐角后的第二条线：第一条已经喷过，重新接入要从头跑，照旧车照走、先不喷
        spec = rp.RouteSpec(kind='path', points=((0., 0.), (6., 0.), (6., 5.)), closed=False, cruise=.8)
        car = Car(*line_car(-6., 2., 0.))
        cfg = replace(differential_config(PathConfig(cruise_mps=.8)), straight_imu_enabled=False)
        m = LineMission(car, SimpleNamespace(), LineTracker(LineConfig(), GEO),
                        route_path=str(Path(self.folder.name) / 'route.json'), path_config=cfg, nozzle_x=NOZZLE)
        ok, message = m.command(dict(cmd='route_start_current', route=spec.to_dict(digits=12),
                                     nozzle=dict(forward=NOZZLE[0], right=-NOZZLE[1])), 0.)
        self.assertTrue(ok, message)
        run(m, car, 120., stop=lambda m: m.operations.paint_status().get('painting')
            and m.operations.record['cursor_m'] > 8.)
        self.assertEqual(m.phase, 'RUNNING')
        with mock.patch.object(paint_quality, 'PAINT_START_MAX_OFFSET_M', 0.):
            messages = run(m, car, 60., stop=lambda m: m.message.startswith(lm.PAINT_START_OFF_PREFIX))
        self.assertTrue(m.message.startswith(lm.PAINT_START_OFF_PREFIX), messages)
        self.assertEqual(m.tracker.st.state, 'RUNNING')
        self.assertIsNone(m._reentry)


if __name__ == '__main__':
    unittest.main()
