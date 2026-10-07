"""复现线尾仍给正速、实测低速达不到阈值时不能结束的问题。只做纯逻辑测试。"""
import json
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

ROOT = Path(os.environ.get('ROUTE_END_ROOT', Path(__file__).resolve().parents[1])).resolve()
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT)]
from path_tracker import Path as RoutePath, PathConfig, PathTracker, Section, SectionTracker
from robot_core.kinematics import ChassisGeometry
from line_tracker import LineConfig, LineTracker
from line_mission import LineMission

DT = .02
CFG = PathConfig(cruise_mps=.8, delay_s=0., lead_s=0., steer_gain=1., start_min_left_m=0.)
GEO = ChassisGeometry(drive='differential')

def pose(x=0., speed=.8, usable=True):
    return SimpleNamespace(x=x, y=0., yaw=0., speed=speed, yaw_rate=0.,
        usable_for_control=usable, quality='RTK_FIXED', sigma_xy=.006,
        sigma_yaw=math.radians(.1), reasons=())

def tracker(length=10.4):
    t = PathTracker(RoutePath((0., 0., 0.), [(length, 0.)]), CFG, GEO)
    t.set_nozzle(.24, -.4525)
    t.paint_to_m = length-.4
    t.run_out_m = .4
    t.constant_straight_speed = True
    return t

class Fusion:
    def __init__(self): self.current = pose()
    def pose(self, now): return self.current
    def to_geodetic(self, x, y): return 30.+y*1e-5, 120.+x*1e-5
    def to_local(self, lat, lon): return (lon-120.)/1e-5, (lat-30.)/1e-5

class RouteEndFeedback(unittest.TestCase):
    def test_low_speed_floor_must_not_keep_positive_command_forever(self):
        t = tracker(); p = pose(); self.assertTrue(t.start(p))
        commands = []
        for _ in range(1500):
            v, _ = t.update(p, DT)
            commands.append((p.x, v))
            if t.st.state == 'DONE': break
            # 复现记录：0.06 的正速命令仍对应 0.10–0.20 的实测反馈。
            actual = max(.16, v) if v > 0. else 0.
            p = pose(p.x+actual*DT, actual)
        self.assertEqual(t.st.state, 'DONE')
        self.assertEqual(commands[-1][1], 0.)
        self.assertLess(p.x, 10.55)
        self.assertTrue(all(v > 0. for x,v in commands if x < 10.))
        self.assertTrue(all(abs(v-.8) < 1e-9 for x,v in commands if 1. < x < 10.))

    def test_no_high_speed_zero_at_run_out_boundary(self):
        t = tracker(); p = pose(10.39)
        self.assertTrue(t.start(p))
        v, _ = t.update(p, DT)
        self.assertEqual(t.st.state, 'RUNNING')
        self.assertGreaterEqual(v, .8-CFG.run_out_decel_max_mps2*DT-1e-9)

    def test_unusable_pose_still_pauses(self):
        t = tracker(); self.assertTrue(t.start(pose()))
        self.assertEqual(t.update(pose(5., usable=False), DT), (0.,0.))
        self.assertEqual(t.st.state, 'PAUSED')

    def test_last_positive_command_reaches_creep_floor_before_zero(self):
        t=tracker(); p=pose(10.41,.2)
        self.assertTrue(t.start(pose(10.2,.2)))
        t._run_out_started=True; t._v_prev=.08
        v,_=t.update(p,DT)
        self.assertEqual(t.st.state,'RUNNING')
        self.assertAlmostEqual(v,CFG.min_mps)
        self.assertEqual(t.update(p,DT),(0.,0.))
        self.assertEqual(t.st.state,'DONE')

    def test_pause_resume_after_paint_end_still_finishes(self):
        t = tracker(); p = pose()
        self.assertTrue(t.start(p))
        while p.x < 10.1:
            v,_=t.update(p,DT); actual=max(.16,v)
            p=pose(p.x+actual*DT,actual)
        t.pause('operator'); self.assertTrue(t.start(p))
        for _ in range(150):
            v,_=t.update(p,DT)
            if t.st.state=='DONE': break
            actual=max(.16,v) if v>0. else 0.
            p=pose(p.x+actual*DT,actual)
        self.assertEqual(t.st.state,'DONE')

    def test_recorded_tail_reaches_zero_despite_feedback_over_threshold(self):
        file = Path(__file__).resolve().parent / 'fixtures/route_end_feedback_tail.json'
        rows=json.loads(file.read_text())['rows']
        length=rows[0]['data']['tracker']['length']
        t=tracker(length); first=rows[0]['data']
        self.assertTrue(t.start(pose(first['tracker']['s'],first['pose']['speed'])))
        for r in rows:
            d=r['data']; p=pose(d['tracker']['s'],d['pose']['speed'])
            v,_=t.update(p,.2)
            if t.st.state=='DONE': break
        self.assertEqual(t.st.state,'DONE')
        self.assertEqual(v,0.)
        self.assertGreater(p.speed,CFG.min_mps*1.5)

    def test_mission_marks_done_and_releases_navigation(self):
        fusion=Fusion()
        m=LineMission(fusion,SimpleNamespace(),tracker=LineTracker(LineConfig(),GEO),path_config=CFG)
        sec=Section(1,RoutePath((0.,0.,0.),[(10.4,0.)]))
        sec.field_role='paint'; sec.paint_to_m=10.; sec.cruise_mps=.8
        m.tracker=SectionTracker([sec],CFG,GEO,constant_straight_speed=True)
        m.tracker.set_nozzle(.24,-.4525)
        self.assertTrue(m.tracker.start(fusion.current))
        m.task,m.phase,m.cruise='route','RUNNING',.8
        m.operations.begin([sec],cruise=.8,route_nozzle={'forward':.24,'right':.4525})
        out=None
        for i in range(1500):
            out=m.tick(i*DT,'NAVIGATION')
            p=fusion.current; actual=max(.16,out[0]) if out[0]>0. else 0.
            fusion.current=pose(p.x+actual*DT,actual)
            if out[2] is False: break
        self.assertEqual(m.phase,'IDLE')
        self.assertEqual(m.operations.record['state'],'DONE')
        self.assertEqual(out,(0.,0.,False))
        self.assertFalse(m.operations.record.get('painting',False))
        m._close_log()

if __name__ == '__main__': unittest.main()
