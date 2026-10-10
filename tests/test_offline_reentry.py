"""接入时偏离路线：还没开喷就不结束作业，停车等定位对齐后从最近处重新接入（2026-10-10 用户）。

复现 10-10 实录 16:05:00：开始作业时给控制的航向比融合差 58°（连续输出每秒只追 1°），原地转向转错方向，
开出 0.4 m 就偏 40 cm，原来直接「中止：偏离路线太远」。"""
from dataclasses import replace
from pathlib import Path
import math
import sys
import tempfile
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT)]
import line_mission as lm
import route_plan as rp
from line_mission import LineMission
from line_tracker import LineConfig, LineTracker
from path_tracker import PathConfig, differential_config
from robot_core.kinematics import ChassisGeometry

DT = .02
GEO = ChassisGeometry(drive='differential', track_m=.675, diff_min_radius_m=.5)
NOZZLE = (.24, -.4575)
ROUTE = rp.RouteSpec(kind='path', points=((0., 0.), (12., 0.)), closed=False, cruise=.8)


class Car:
    """真车（后轴中心、车头）+ 给控制的融合输出：输出航向 = 真车头 + err，err 每秒最多追 1°（连续输出）。"""
    def __init__(self, x, y, h, err_deg=0.):
        self.x, self.y, self.h, self.v, self.w = x, y, h, 0., 0.
        self.err = math.radians(err_deg)

    def step(self, vx, wz):
        self.v, self.w = vx, wz
        self.h += wz * DT
        self.x += vx * math.cos(self.h) * DT
        self.y += vx * math.sin(self.h) * DT
        self.err -= math.copysign(min(abs(self.err), math.radians(1.) * DT), self.err)

    def pose(self, now):
        return SimpleNamespace(x=self.x, y=self.y, yaw=self.h + self.err, speed=self.v, yaw_rate=self.w,
                               usable_for_control=True, quality='RTK_FIXED', reasons=(), rtk_trusted=True,
                               sigma_xy=.005, sigma_yaw=.005, correction_xy=(0., 0.), correction_yaw=-self.err,
                               hold_age_s=0., hold_distance_m=0., hold_turn_rad=0.)

    to_geodetic = staticmethod(lambda x, y: (30. + y * 1e-5, 120. + x * 1e-5))
    to_local = staticmethod(lambda lat, lon: ((lon - 120.) / 1e-5, (lat - 30.) / 1e-5))


def mission(car, folder):
    cfg = replace(differential_config(PathConfig(cruise_mps=.8)), straight_imu_enabled=False)
    m = LineMission(car, SimpleNamespace(), LineTracker(LineConfig(), GEO), route_path=str(Path(folder) / 'route.json'),
                    path_config=cfg, nozzle_x=NOZZLE)
    ok, message = m.command(dict(cmd='route_start_current', route=ROUTE.to_dict(digits=12),
                                 nozzle=dict(forward=NOZZLE[0], right=-NOZZLE[1])), 0.)
    assert ok, message
    return m


def run(m, car, seconds, stop=lambda m: False):
    messages = []
    for i in range(int(seconds / DT)):
        vx, wz, _ = m.tick(i * DT, 'NAVIGATION')
        car.step(vx, wz)
        if not messages or messages[-1] != m.message:
            messages.append(m.message)
        if m.phase == 'IDLE' or stop(m):
            break
    return messages


class OfflineReentry(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.folder.cleanup()

    def test_wrong_heading_at_start_waits_then_reenters_and_finishes(self):
        # 车在线左后方 1.7 m、车头朝北偏西；给控制的航向错 58°
        car = Car(-3., 1.7, math.radians(100.), err_deg=58.)
        m = mission(car, self.folder.name)
        job = m.operations.record['id']
        messages = run(m, car, 240.)
        self.assertEqual(m.phase, 'IDLE', messages)
        self.assertEqual(m.message, '路线完成', messages)
        self.assertFalse(any(t.startswith('中止') for t in messages), messages)
        self.assertIn(lm.REENTRY_WAIT_NOTICE, messages)
        r = m.operations.record
        self.assertEqual(r['id'], job)
        self.assertEqual(r['state'], 'DONE')
        self.assertGreaterEqual(r['reentries'], 1)
        self.assertLessEqual(r['reentries'], lm.REENTRY_MAX)

    def test_correct_heading_never_reenters(self):
        car = Car(-3., 1.7, math.radians(100.))
        m = mission(car, self.folder.name)
        messages = run(m, car, 120.)
        self.assertEqual(m.message, '路线完成', messages)
        self.assertNotIn('reentries', m.operations.record)

    def test_off_line_after_paint_started_still_aborts(self):
        car = Car(-3., 1.7, math.radians(100.))
        m = mission(car, self.folder.name)
        # 开到已经开喷后（进度过了第一次划线）再判偏离
        run(m, car, 120., stop=lambda m: m.operations.paint_status().get('painting')
            and m.operations.record['cursor_m'] > 1.)
        self.assertEqual(m.phase, 'RUNNING')
        m.tracker.st.state, m.tracker.st.reason = 'ABORTED', 'off_line'
        messages = run(m, car, 5.)
        self.assertEqual(m.phase, 'IDLE')
        self.assertEqual(messages[-1].split('，')[0], '中止：偏离路线太远')

    def test_gives_up_after_max_reentries(self):
        car = Car(-3., 1.7, math.radians(100.))
        m = mission(car, self.folder.name)
        m.operations.record['reentries'] = lm.REENTRY_MAX
        run(m, car, 3., stop=lambda m: m.phase == 'RUNNING' and m.tracker.st.state == 'RUNNING')
        m.tracker.st.state, m.tracker.st.reason = 'ABORTED', 'off_line'
        run(m, car, 5.)
        self.assertEqual(m.phase, 'IDLE')
        self.assertTrue(m.message.startswith('中止：偏离路线太远'), m.message)

    def test_gives_up_when_position_never_aligns(self):
        car = Car(-3., 1.7, math.radians(100.))
        m = mission(car, self.folder.name)
        run(m, car, 3., stop=lambda m: m.phase == 'RUNNING' and m.tracker.st.state == 'RUNNING')
        m.tracker.st.state, m.tracker.st.reason = 'ABORTED', 'off_line'
        car.err = math.radians(1e6)      # 一直不对齐
        messages = run(m, car, lm.REENTRY_WAIT_S + 5.)
        self.assertEqual(m.phase, 'IDLE')
        self.assertIn(lm.REENTRY_WAIT_NOTICE, messages)
        self.assertTrue(m.message.startswith('中止：偏离路线太远'), m.message)


if __name__ == '__main__':
    unittest.main()
