"""Current-position route start: geometry, command parsing and real mission state."""
import json
import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, 'radar_system'), ROOT, os.path.join(ROOT, 'tests')]
import route_plan as rp
import app_link
from test_line_mission import Base, FakeFusion, SAMPLES
from line_mission import LineMission

BODY = (.67, .08, .335)

def cases():
    path = dict(kind='path', points=[[0,0],[20,0],[20,10]], closed=False, cruise=.5)
    circle = dict(kind='circle', center=[0,0], radius=3, clockwise=False, laps=2)
    return [
        ('middle straight',path,(10,0,0),.556,.3),
        ('later leg',path,(20,5,math.pi/2),.556,.3),
        ('off line',path,(10,2,0),.556,.3),
        ('wrong heading',path,(10,0,math.pi),.556,.3),
        ('ackermann middle',path,(10,0,0),1.916,None),
        ('ackermann off line',path,(10,1,0),1.916,None),
        ('closed two laps',dict(path,points=[[0,0],[20,0],[20,10],[0,10]],closed=True,laps=2),(10,0,0),.556,.3),
        ('circle',circle,(3,0,math.pi/2),1.916,None),
        ('nozzle circle',circle,(3,0,math.pi/2),.556,.3),
        ('legacy points',dict(path,kind='points'),(10,0,0),1.916,None),
    ]

class Geometry(unittest.TestCase):
    def test_mid_line_starts_here_and_keeps_original(self):
        spec=rp.parse_spec(cases()[0][1])
        sections,entry=rp.plan_current(spec,(10,0,0),.556,body=BODY,nozzle_x=.3)
        self.assertEqual(0,entry)
        self.assertAlmostEqual(10,sections[0].car_pose(0)[0])
        self.assertEqual(((0.,0.),(20.,0.),(20.,10.)),spec.points)
        self.assertAlmostEqual(20,sections[-1].car_pose(sections[-1].path.length)[0])
        self.assertTrue(any(s.direction<0 for s in sections))
        self.assertTrue(all(p.x>8 for s in sections for p in s.path.pieces))

    def test_later_leg_skips_prior_points(self):
        spec=rp.parse_spec(cases()[0][1])
        sections,entry=rp.plan_current(spec,(20,5,math.pi/2),.556,body=BODY,nozzle_x=.3)
        self.assertEqual(0,entry)
        self.assertEqual(1,len(sections))
        self.assertAlmostEqual(4.75,sections[0].path.length)

    def test_at_end_does_not_go_back_to_first(self):
        spec=rp.parse_spec(cases()[0][1])
        with self.assertRaisesRegex(ValueError,'终点'):
            rp.plan_current(spec,(20,9.7,math.pi/2),.556,body=BODY,nozzle_x=.3)

    def test_off_line_connects_to_nearby_segment(self):
        spec=rp.parse_spec(cases()[0][1])
        sections,entry=rp.plan_current(spec,(10,2,0),.556,body=BODY,nozzle_x=.3)
        self.assertAlmostEqual(2,entry)
        self.assertAlmostEqual(10,sections[0].car_pose(0)[0])
        self.assertAlmostEqual(10,sections[0].car_pose(sections[0].path.length)[0])
        with self.assertRaisesRegex(ValueError,'最近路线段'):
            rp.plan_current(spec,(10,-100,0),.556,body=BODY,nozzle_x=.3)

    def test_all_kinds_have_finite_connected_remaining_paths(self):
        for name,raw,car,radius,nozzle in cases():
            with self.subTest(name):
                sections,entry=rp.plan_current(rp.parse_spec(raw),car,radius,body=BODY,nozzle_x=nozzle)
                self.assertTrue(0<=entry<=rp.MAX_ENTRY_M)
                for a,b in zip(sections,sections[1:]):
                    self.assertLess(math.dist(a.car_pose(a.path.length)[:2],b.car_pose(0)[:2]),1e-6)
                self.assertTrue(all(math.isfinite(s.path.length) and s.path.length>0 for s in sections))

    def test_new_action_is_explicit_and_preserves_route(self):
        raw=dict(type='route',action='start_current',id='current',route=cases()[0][1])
        command=app_link.parse_command(json.dumps(raw))
        self.assertEqual('start_current',command['action'])
        self.assertEqual([[0.,0.],[20.,0.],[20.,10.]],command['route']['points'])

class Mission(Base):
    def test_current_start_saves_original_and_persists_remaining_job(self):
        self.m=LineMission(self.fusion,SAMPLES,route_path=os.path.join(self.tmp.name,'route.json'))
        self.fusion.current=FakeFusion.make(10,0,0)
        ok,why=self.m.command(dict(cmd='route_start_current',route=cases()[0][1]),0)
        self.assertTrue(ok,why)
        self.assertEqual('SELECTING',self.m.phase)
        self.assertAlmostEqual(10,self.m.tracker.sections[0].car_pose(0)[0])
        self.assertEqual(3,len(self.m._route_local().points))
        self.assertAlmostEqual(0,self.m._route_local().points[0][0])
        self.assertEqual(0,self.m.operations.execution['entry_m'])
        self.drive(.05,12)
        self.assertGreater(self.fusion.current.x,10)

if __name__=='__main__': unittest.main()
