"""开/关喷标记（operations.job.paint_marks）：与发给水泵的 /line/trace paint 同一拍、同一个值，记那一拍的实际喷头位置，
App 的喷涂线按它起止（2026-10-07：地图上线头白一截、线尾早断一截，原因是 App 按 5 Hz 状态 + 网络延迟才开始画）。"""
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT)]
import control_trace
import route_plan as rp
from path_tracker import Path as RoutePath, PathConfig, Section, SectionTracker
from robot_core.kinematics import ChassisGeometry
from line_tracker import LineConfig, LineTracker
from line_mission import LineMission
from route_operations import PAINT_MARKS_KEEP

DT = .02
CFG = PathConfig(cruise_mps=.8, delay_s=0., lead_s=0., steer_gain=1., start_min_left_m=0.)
GEO = ChassisGeometry(drive='differential')
NOZZLE = dict(forward=.24, right=.4525)


def pose(x=0., speed=.8, usable=True):
    return NS(x=x, y=0., yaw=0., speed=speed, yaw_rate=0., usable_for_control=usable, quality='RTK_FIXED',
              sigma_xy=.006, sigma_yaw=math.radians(.1), reasons=(), rtk_trusted=True)


class Fusion:
    def __init__(self): self.current = pose()
    def pose(self, now): return self.current
    def to_geodetic(self, x, y): return 30. + y * 1e-5, 120. + x * 1e-5
    def to_local(self, lat, lon): return (lon - 120.) / 1e-5, (lat - 30.) / 1e-5


def mission():
    """10.4 m 直线，1.0–10.0 m 是刷线段（段内路程按后轴算）。"""
    fusion = Fusion()
    m = LineMission(fusion, NS(), tracker=LineTracker(LineConfig(), GEO), path_config=CFG)
    sec = Section(1, RoutePath((0., 0., 0.), [(10.4, 0.)]))
    sec.field_role, sec.paint_from_m, sec.paint_to_m, sec.cruise_mps = 'paint', 1., 10., .8
    m.tracker = SectionTracker([sec], CFG, GEO, constant_straight_speed=True)
    m.tracker.set_nozzle(NOZZLE['forward'], -NOZZLE['right'])
    assert m.tracker.start(fusion.current)
    m.task, m.phase, m.cruise = 'route', 'RUNNING', .8
    m._route_local = lambda: NS(kind='path')
    m.operations.begin([sec], cruise=.8, route_nozzle=dict(NOZZLE))
    return m, fusion


def step(m, fusion, i, act=None):
    """跑一拍：返回 (这一拍 trace 发给水泵的 painting, 这一拍控制用的喷头位置)。"""
    now = i * DT
    if act:
        act(m, now)
    vx, wz, _ = m.tick(now, 'NAVIGATION')
    rec = control_trace.record(m, now, 'NAVIGATION', vx, wz)
    painting = bool(rec and (rec.get('paint') or {}).get('painting'))
    p = m.pose
    tip = rp.nozzle_at(p.x, p.y, p.yaw, NOZZLE['forward'], -NOZZLE['right'])
    actual = max(.16, vx) if vx > 0. else 0.
    fusion.current = pose(fusion.current.x + actual * DT, actual)
    return painting, tip


def marks(m):
    return m.operations.status()['job']['paint_marks']


class PaintMarks(unittest.TestCase):
    def low_speed_mission(self):
        m, fusion = mission()
        m.tracker.trackers[0].constant_paint_speed = True
        m._stable_speed_limit = lambda now, pose, dt: .35
        return m, fusion

    def test_low_speed_starts_pump_at_line_head_and_keeps_one_paint_interval(self):
        m, fusion = self.low_speed_mission()
        try:
            edges, prev = [], False
            for i in range(1700):
                painting, tip = step(m, fusion, i)
                if painting != prev:
                    edges.append((painting, tip))
                    prev = painting
                if painting:
                    child = m.tracker.trackers[m.tracker.index]
                    self.assertFalse(child.paint_speed_ready)
                    self.assertLessEqual(child.st.v_cmd, .35)
                if m.phase == 'IDLE':
                    break
            self.assertEqual(m.operations.record['state'], 'DONE')
            self.assertEqual([e[0] for e in edges], [True, False])
            self.assertLess(abs(edges[0][1][0] - NOZZLE['forward'] - 1.), .35 * DT + 1e-6)
            self.assertEqual([k['on'] for k in marks(m)['items']], [True, False])
        finally:
            m._close_log()

    def test_low_speed_spray_stops_on_control_loss_or_unusable_pose(self):
        for cause in ('control_loss', 'position_lost'):
            with self.subTest(cause=cause):
                m, fusion = self.low_speed_mission()
                try:
                    for i in range(400):
                        painting, _ = step(m, fusion, i)
                        if painting:
                            break
                    self.assertTrue(painting)
                    mode = 'MANUAL' if cause == 'control_loss' else 'NAVIGATION'
                    if cause == 'position_lost':
                        fusion.current.usable_for_control = False
                    m.tick((i + 1) * DT, mode)
                    self.assertFalse(m.operations.paint_status()['painting'])
                    self.assertEqual([k['on'] for k in marks(m)['items']], [True, False])
                finally:
                    m._close_log()

    def test_marks_are_the_pump_edges_at_the_actual_nozzle(self):
        m, fusion = mission()
        edges, prev = [], False
        for i in range(1500):
            painting, tip = step(m, fusion, i)
            if painting != prev:
                edges.append((painting, tip))
                prev = painting
            if m.phase == 'IDLE':
                break
        self.assertEqual(m.operations.record['state'], 'DONE')
        self.assertEqual([e[0] for e in edges], [True, False])
        out = marks(m)
        self.assertIsInstance(out['run'], str)
        self.assertEqual([k['on'] for k in out['items']], [True, False])
        self.assertLess(out['items'][0]['seq'], out['items'][1]['seq'])
        for (on, tip), k in zip(edges, out['items']):
            self.assertAlmostEqual(k['x'], tip[0], places=4)
            self.assertAlmostEqual(k['y'], tip[1], places=4)
        # 开喷点就在刷线段起点（后轴 1.0 m）附近一拍以内
        self.assertLess(abs(out['items'][0]['x'] - NOZZLE['forward'] - 1.), .8 * DT + 1e-6)
        m._close_log()

    def test_operator_pause_marks_off_on_the_tick_the_pump_stops(self):
        m, fusion = mission()
        on_at = None
        for i in range(400):
            painting, tip = step(m, fusion, i)
            if painting and on_at is None:
                on_at = i
            if on_at is not None and i == on_at + 25:
                break
        job = m.operations.record['id']
        pause = lambda mm, now: mm.operations.pause(dict(route_rev=mm.route_rev, job_id=job), now)
        painting, tip = step(m, fusion, i + 1, pause)
        self.assertFalse(painting)
        items = marks(m)['items']
        self.assertEqual([k['on'] for k in items], [True, False])
        self.assertAlmostEqual(items[-1]['x'], tip[0], places=4)
        m._close_log()

    def test_new_job_starts_an_empty_list_and_run_stays(self):
        m, fusion = mission()
        for i in range(200):
            step(m, fusion, i)
        first = marks(m)
        self.assertEqual([k['on'] for k in first['items']], [True])
        m.operations.begin(m.tracker.sections, cruise=.8, route_nozzle=dict(NOZZLE))
        self.assertEqual(marks(m), dict(run=first['run'], items=[]))
        m.operations.mark_paint(True)
        self.assertGreater(marks(m)['items'][0]['seq'], first['items'][0]['seq'])
        m._close_log()

    def test_without_pose_still_marks_off_without_a_point(self):
        m, fusion = mission()
        m.operations.mark_paint(True)
        m.pose = None
        m.operations.mark_paint(False)
        self.assertEqual([(k['on'], k['x'], k['y']) for k in marks(m)['items']][-1], (False, None, None))
        m._close_log()

    def test_keeps_only_the_latest(self):
        m, fusion = mission()
        for k in range(PAINT_MARKS_KEEP + 5):
            m.operations.mark_paint(k % 2 == 0)
        items = marks(m)['items']
        self.assertEqual(len(items), PAINT_MARKS_KEEP)
        self.assertEqual([b['seq'] - a['seq'] for a, b in zip(items, items[1:])], [1] * (PAINT_MARKS_KEEP - 1))
        m._close_log()


if __name__ == '__main__':
    unittest.main()
