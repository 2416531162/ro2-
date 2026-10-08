"""所有正常前进路径提前平滑减速入弯，喷线定速不能覆盖弯道规划。"""
from collections import deque
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
import math,sys,unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT)]
from path_tracker import Path as Route,PathConfig,PathTracker,Section,SectionTracker,differential_config
from robot_core.kinematics import ChassisGeometry
from motion_speeds import TURN_SPEED_MPS, PAINT_ARC_SPEED_MPS, transit_bend_speed
GEO=ChassisGeometry(drive='differential',track_m=.675,diff_min_radius_m=.5);DT=.02
# R 2 m 转场弯：内侧轮不低于 0.3 m/s（10-08 用户）→ 0.361 m/s
BEND=transit_bend_speed(-.5,GEO.track_m,differential_config(PathConfig()).lat_accel_mps2)

def pose(path,s,v):
 x,y,h=path.pose(s)
 return NS(x=x,y=y,yaw=h,speed=v,yaw_rate=v*path.kappa(s),usable_for_control=True,
           quality='RTK_FIXED',rtk_trusted=True,reasons=(),correction_xy=(0.,0.),correction_yaw=0.)

def drive(cruise=1.,straight=12.,painting=True,start_speed=None):
 cfg=replace(differential_config(PathConfig(cruise_mps=cruise)),straight_imu_enabled=False,accel_mps2=.2,accel_jerk_mps3=.5)
 path=Route((0.,0.,0.),[(straight,0.),(6.,-.5),(3.,0.)]);tr=PathTracker(path,cfg,GEO)
 tr.constant_straight_speed=True;tr.constant_paint_speed=painting;tr.paint_from_m=0.;tr.paint_to_m=straight-.4
 tr.paint_spans=[(0.,straight-.4)] if painting else None
 speed=cruise if start_speed is None else start_speed;s=0.;queue=deque([speed]*15);rows=[]
 assert tr.start(pose(path,s,speed))
 for i in range(10000):
  v,w=tr.update(pose(path,s,speed),DT)
  rows.append(dict(t=i*DT,s=s,v=v,actual=speed,painting=0<=s<tr.paint_to_m,ready=tr.paint_speed_ready))
  if s>straight+1. or tr.st.state!='RUNNING':break
  queue.append(v);speed=queue.popleft();s+=speed*DT
 return tr,rows

class GlobalBendSpeed(unittest.TestCase):
 def test_all_straight_speeds_slow_before_curve_with_bounded_accel_and_jerk(self):
  for speed in (.5,.8,1.,1.5):
   for painting in (False,True):
    with self.subTest(speed=speed,painting=painting):
     tr,rows=drive(speed,painting=painting)
     self.assertEqual(tr.st.state,'RUNNING')
     curve=[r for r in rows if r['s']>=12.]
     self.assertTrue(curve)
     self.assertLessEqual(max(r['v'] for r in curve),BEND+1e-5)
     self.assertLessEqual(max(r['actual'] for r in curve),BEND+1e-5)
     accel=[(b['v']-a['v'])/DT for a,b in zip(rows,rows[1:])]
     jerk=[(b-a)/DT for a,b in zip(accel,accel[1:])]
     self.assertGreaterEqual(min(accel),-.25001)
     self.assertLessEqual(max(abs(j) for j in jerk),.5001)
     if painting:self.assertTrue(all(r['ready'] for r in rows if r['painting']))

 def test_short_straight_does_not_sprint_then_brake(self):
  tr,rows=drive(1.,straight=2.,painting=False,start_speed=0.)
  self.assertEqual(tr.st.state,'RUNNING')
  self.assertLess(max(r['v'] for r in rows),.65)
  self.assertLessEqual(max(r['actual'] for r in rows if r['s']>=2.),BEND+1e-5)
  accel=[(b['v']-a['v'])/DT for a,b in zip(rows,rows[1:])]
  self.assertLessEqual(max(abs(b-a)/DT for a,b in zip(accel,accel[1:])),.5001)

 def test_curve_preview_crosses_continuous_section_boundary(self):
  a=Section(1,Route((0.,0.,0.),[(8.,0.)]));a.field_role='paint';a.paint_from_m=0.;a.paint_to_m=8.;a.continuous_after=True
  b=Section(1,Route((8.,0.,0.),[(8.,-.5)]));b.field_role='paint';b.paint_from_m=0.;b.paint_to_m=7.5
  route=SectionTracker([a,b],replace(differential_config(PathConfig(cruise_mps=1.)),straight_imu_enabled=False),GEO,constant_paint_speed=True)
  tr=route.trackers[0];tr.start(pose(tr.path,4.,1.));tr._hint=4.;tr._v_prev=1.
  self.assertLess(tr._curve_speed(5.,1.),1.)

 def test_external_stop_and_unusable_pose_bypass_comfort_braking(self):
  tr,rows=drive(.8)
  tr.speed_limit_mps=0.;self.assertEqual(tr.update(pose(tr.path,5.,.8),DT),(0.,0.))
  tr,_=drive(.8);p=pose(tr.path,5.,.8);p.usable_for_control=False
  self.assertEqual(tr.update(p,DT),(0.,0.))

 def test_planner_also_caps_painted_and_transit_curves(self):
  from field_planner import Settings,speed_for
  from motion_speeds import TURN_SPEED_MPS
  cfg=Settings(.24,.4575,(.911,.604,.39),.557,paint_speed=1.,transit_speed=1.5,lateral_accel=.2)
  for paint in (False,True):
   for k in (-1.,-.2,.2,1.):
    sec=Section(1,Route((0.,0.,0.),[(8.,k)]))
    self.assertEqual(speed_for(sec,cfg,paint=paint),PAINT_ARC_SPEED_MPS if paint else TURN_SPEED_MPS)
  sec=Section(1,Route((0.,0.,0.),[(8.,0.)]))
  self.assertEqual(speed_for(sec,cfg,paint=True),1.)

if __name__=='__main__':unittest.main()
