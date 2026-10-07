"""直线稳定后平滑提速、弯前减速；喷涂不等待速度达标，仍检查位置和停喷状态。"""
from pathlib import Path
from dataclasses import replace
from types import SimpleNamespace as NS
import json,math,sys,unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT),str(ROOT/'tests')]
from test_continuous_field_arcs import GEO,DT,CFG,corner,sections,pose,drive
from path_tracker import Path as Route,Section,SectionTracker,PathConfig,differential_config
from field_planner import plan
from route_operations import RouteOperations,rework_sections
from live_cruise import label_sections
import route_plan as rp
import paint_quality
from motion_speeds import TURN_SPEED_MPS, PAINT_ARC_SPEED_MPS

def strict(secs):
 # 纯速度/几何模型不输入 IMU；起喷检查另由真实传感器链路测试覆盖。
 tr=SectionTracker(secs,replace(differential_config(PathConfig(cruise_mps=.8)),straight_imu_enabled=False),GEO,constant_straight_speed=True,constant_paint_speed=True)
 tr.set_nozzle(.24,-.4575)
 return tr

def section(pieces,mark,end,cruise=.8):
 s=Section(1,Route((0.,0.,0.),pieces));s.field_role='paint';s.paint_from_m=mark;s.paint_to_m=end;s.cruise_mps=cruise
 return s

def at(tr,s,v=.8,offset=0.):
 x,y,h=tr.path.pose(s)
 p=pose(x-offset*math.sin(h),y+offset*math.cos(h),h,v,v*tr.path.kappa(s))
 tr._hint=s;return p

class PaintSpeedInvariant(unittest.TestCase):
 def test_straight_brakes_early_for_future_turn_and_keeps_painting(self):
  tr=strict([section([(10.,0.),(8.,-.7)],2.,10.)]).trackers[0]
  self.assertTrue(tr.start(at(tr,5.)))
  s=5.;v=.8;values=[]
  while s<10.2:
   v,_=tr.update(at(tr,s,v),DT)
   if s<10.:
    values.append(v);self.assertTrue(tr.paint_speed_ready)
   s+=v*DT
  self.assertLess(min(values),TURN_SPEED_MPS+.01)
  self.assertLessEqual(v,TURN_SPEED_MPS+1e-6);self.assertFalse(tr.paint_speed_ready)

 def test_arc_and_circle_do_not_slow_down_before_paint_end(self):
  for k,length in ((-.7,2.3),(.7,2.3),(-.4,16.),(.6,11.)):
   with self.subTest(k=k,length=length):
    tr=strict([section([(length+.5,k)],.5,length)]).trackers[0]
    self.assertTrue(tr.start(at(tr,1.,PAINT_ARC_SPEED_MPS)))
    for s in (length-.4,length-.15,length-.01):
     v,_=tr.update(at(tr,s,PAINT_ARC_SPEED_MPS),DT)
     self.assertAlmostEqual(v,PAINT_ARC_SPEED_MPS);self.assertTrue(tr.paint_speed_ready)
    v,_=tr.update(at(tr,length+.3,PAINT_ARC_SPEED_MPS),DT)
    self.assertLessEqual(v,PAINT_ARC_SPEED_MPS);self.assertFalse(tr.paint_speed_ready)

 def test_mixed_straight_arc_stroke_chooses_one_speed_before_paint(self):
  tr=strict([section([(4.,0.),(2.,-.5),(4.5,0.)],1.,10.)]).trackers[0]
  self.assertTrue(tr.start(at(tr,2.,PAINT_ARC_SPEED_MPS)))
  for s in (2.,3.9,4.1,5.9,6.1,9.8):
   v,_=tr.update(at(tr,s,PAINT_ARC_SPEED_MPS),DT)
   self.assertAlmostEqual(v,PAINT_ARC_SPEED_MPS);self.assertTrue(tr.paint_speed_ready)

 def test_online_speed_change_waits_until_paint_is_off(self):
  tr=strict([section([(12.,0.)],1.,10.)]).trackers[0]
  self.assertTrue(tr.start(at(tr,4.)))
  self.assertAlmostEqual(tr.update(at(tr,4.),DT)[0],.8)
  tr.set_cruise(.4)
  for s in (4.2,8.,9.99):
   self.assertAlmostEqual(tr.update(at(tr,s),DT)[0],.8)
   self.assertTrue(tr.paint_speed_ready)
  tr.update(at(tr,10.2),DT)
  self.assertFalse(tr.paint_speed_ready)
  v,_=tr.update(at(tr,10.3),DT)
  self.assertLess(v,.8)

 def test_offset_does_not_reduce_paint_speed(self):
  for k in (0.,-.5):
   tr=strict([section([(12.,k)],1.,11.)]).trackers[0]
   speed=.8 if k==0. else PAINT_ARC_SPEED_MPS
   self.assertTrue(tr.start(at(tr,4.,speed)))
   v,_=tr.update(at(tr,4.,speed,offset=.12),DT)
   self.assertAlmostEqual(v,speed)

 def test_startup_and_speed_cap_allow_spray_without_bypassing_stop(self):
  sec=section([(12.,0.)],1.,11.);tr=strict([sec]);child=tr.trackers[0]
  self.assertTrue(tr.start(at(child,2.,0.)))
  m=NS(pose=at(child,2.,.2),differential=True,nozzle=(.24,-.4575),fusion=NS(config=NS(rtk_hold_s=9.)),_route_local=lambda:None)
  tr.update(m.pose,DT)
  self.assertFalse(child.paint_speed_ready)
  self.assertTrue(paint_quality.status(m,{'field_plan':True},True,sec,child)['painting'])
  child._v_prev=.8;tr.speed_limit_mps=.3
  v,_=tr.update(at(child,3.,.8),DT)
  self.assertEqual(v,.3);self.assertFalse(child.paint_speed_ready)
  m.pose=at(child,3.,.3)
  self.assertTrue(paint_quality.status(m,{'field_plan':True},True,sec,child)['painting'])
  tr.speed_limit_mps=0.;self.assertEqual(tr.update(at(child,3.01,.3),DT),(0.,0.))
  m.pose=at(child,3.01,0.)
  self.assertEqual(paint_quality.status(m,{'field_plan':True},True,sec,child)['paint_blocked'],'paint_stationary')

 def test_final_motion_smoother_speed_difference_is_diagnostic_only(self):
  from line_mission import LineMission
  sec=section([(12.,0.)],1.,11.);tr=strict([sec]);child=tr.trackers[0]
  self.assertTrue(tr.start(at(child,3.,.8)));tr.update(at(child,3.,.8),DT)
  self.assertTrue(child.paint_speed_ready)
  m=LineMission.__new__(LineMission);m.tracker=tr;m.task='route'
  m.pose=at(child,3.,.95);m.differential=True;m.nozzle=(.24,-.4575)
  m._check_paint_output_speed(.95)
  self.assertFalse(child.paint_speed_ready)
  self.assertTrue(paint_quality.status(m,{'field_plan':True},True,sec,child)['painting'])
  tr.update(at(child,3.02,.8),DT);m._check_paint_output_speed(.8)
  self.assertTrue(child.paint_speed_ready)

 def test_low_speed_spray_does_not_depend_on_selected_cruise(self):
  for cruise in (.5,.8,1.):
   for speed in (.2,.35,.5,.6792,.7366,.8):
    with self.subTest(cruise=cruise,speed=speed):
     sec=section([(12.,0.)],1.,11.,cruise);tr=strict([sec]);child=tr.trackers[0]
     self.assertTrue(tr.start(at(child,1.,speed)))
     m=NS(pose=at(child,1.,speed),differential=True,nozzle=(.24,-.4575))
     tr.update(m.pose,DT)
     # 模拟限速或任务层最终指令与计划速度不同，不能再触发线头停喷。
     child.paint_speed_ready=False
     out=paint_quality.status(m,{'field_plan':True},True,sec,child)
     self.assertTrue(out['painting'],out);self.assertIsNone(out['paint_blocked'])

 def test_low_speed_still_checks_phase_position_and_start_alignment(self):
  sec=section([(12.,0.)],1.,11.);tr=strict([sec]);child=tr.trackers[0]
  self.assertTrue(tr.start(at(child,2.,.35)))
  m=NS(pose=at(child,2.,.35),differential=True,nozzle=(.24,-.4575))
  tr.update(m.pose,DT);child.paint_speed_ready=False
  check=lambda phase=True:paint_quality.status(m,{'field_plan':True},phase,sec,child)
  self.assertTrue(check()['painting'])
  self.assertEqual(check(False)['paint_blocked'],'not_painting')
  m.pose.usable_for_control=False
  self.assertEqual(check()['paint_blocked'],'paint_position_untrusted')
  m.pose=at(child,2.,.35,offset=.04)
  self.assertEqual(check()['paint_blocked'],'paint_off_line')
  m.pose=at(child,2.,.35);child.paint_start_guard=NS(started=False)
  self.assertEqual(check()['paint_blocked'],'paint_start_unready')

 def test_partial_line_and_circle_rework_have_a_dry_tail(self):
  specs=[rp.RouteSpec(kind='path',points=((0.,0.),(10.,0.)),closed=False),rp.RouteSpec(kind='circle',center=(0.,0.),radius=2.)]
  for spec in specs:
   secs,_=rework_sections(spec,0,1.,3.,.557,(.911,.604,.39),nozzle=(.24,-.4575))
   label_sections(secs,0.,.8);tr=strict(secs)
   self.assertIsNotNone(secs[-1].paint_to_m)
   self.assertGreater(secs[-1].path.length-secs[-1].paint_to_m,.39)

 def test_legacy_geometry_without_paint_endpoint_cannot_start_under_new_policy(self):
  sec=section([(3.,0.)],0.,None)
  with self.assertRaisesRegex(ValueError,'重新规划'):strict([sec])

 def test_corner_start_stabilizes_then_remaining_lines_and_arc_keep_speed(self):
  out=plan(corner(),(-.3,.4,0.),CFG);tr=strict(sections(out));rows=drive(tr)
  self.assertEqual(tr.st.state,'DONE')
  for index,expected in ((0,.8),(1,PAINT_ARC_SPEED_MPS),(2,.8)):
   values=[r['v'] for r in rows if r['index']==index and r['paint']]
   self.assertTrue(values)
   if index in (0,2):
    first=[r['v'] for r in rows if r['index']==index and r['paint'] and r['s']<tr.sections[index].paint_from_m+1.]
    self.assertTrue(first);self.assertTrue(all(abs(v-.5)<1e-6 for v in first))
    self.assertGreaterEqual(min(values),TURN_SPEED_MPS-1e-6)
    self.assertLessEqual(max(values),expected+1e-6)
   else:self.assertAlmostEqual(min(values),expected,delta=1e-6)
   if index==1:self.assertAlmostEqual(max(values),expected,delta=1e-6)
  self.assertTrue(all(r['v']>0 for r in rows[:-1]))

if __name__=='__main__':unittest.main()
