"""正常喷线从线头到线尾保持一笔速度；转场可减速，保护先停喷。"""
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
 def test_straight_does_not_brake_for_a_turn_after_its_paint_end(self):
  tr=strict([section([(10.,0.),(8.,-.7)],2.,10.)]).trackers[0]
  self.assertTrue(tr.start(at(tr,7.)))
  for s in (7.,8.,9.,9.8,9.98):
   v,_=tr.update(at(tr,s),DT)
   self.assertAlmostEqual(v,.8);self.assertTrue(tr.paint_speed_ready)
  # 已停喷后，允许为转场弯道降速。
  v,_=tr.update(at(tr,10.2),DT)
  self.assertLess(v,.8);self.assertFalse(tr.paint_speed_ready)

 def test_arc_and_circle_do_not_slow_down_before_paint_end(self):
  for k,length in ((-.7,2.3),(.7,2.3),(-.4,16.),(.6,11.)):
   with self.subTest(k=k,length=length):
    tr=strict([section([(length+.5,k)],.5,length)]).trackers[0]
    self.assertTrue(tr.start(at(tr,1.,.5)))
    for s in (length-.4,length-.15,length-.01):
     v,_=tr.update(at(tr,s,.5),DT)
     self.assertAlmostEqual(v,.5);self.assertTrue(tr.paint_speed_ready)
    v,_=tr.update(at(tr,length+.3,.5),DT)
    self.assertLess(v,.5);self.assertFalse(tr.paint_speed_ready)

 def test_mixed_straight_arc_stroke_chooses_one_speed_before_paint(self):
  tr=strict([section([(4.,0.),(2.,-.5),(4.5,0.)],1.,10.)]).trackers[0]
  self.assertTrue(tr.start(at(tr,2.,.5)))
  for s in (2.,3.9,4.1,5.9,6.1,9.8):
   v,_=tr.update(at(tr,s,.5),DT)
   self.assertAlmostEqual(v,.5);self.assertTrue(tr.paint_speed_ready)

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
   speed=.8 if k==0. else .5
   self.assertTrue(tr.start(at(tr,4.,speed)))
   v,_=tr.update(at(tr,4.,speed,offset=.12),DT)
   self.assertAlmostEqual(v,speed)

 def test_startup_and_protective_cap_close_spray_without_bypassing_stop(self):
  sec=section([(12.,0.)],1.,11.);tr=strict([sec]);child=tr.trackers[0]
  self.assertTrue(tr.start(at(child,2.,0.)))
  m=NS(pose=at(child,2.,.2),differential=True,nozzle=(.24,-.4575),fusion=NS(config=NS(rtk_hold_s=9.)),_route_local=lambda:None)
  tr.update(m.pose,DT)
  self.assertFalse(paint_quality.status(m,{'field_plan':True},True,sec,child)['painting'])
  self.assertEqual(paint_quality.status(m,{'field_plan':True},True,sec,child)['paint_blocked'],'paint_speed_not_ready')
  child._v_prev=.8;tr.speed_limit_mps=.3
  v,_=tr.update(at(child,3.,.8),DT)
  self.assertEqual(v,.3);self.assertFalse(child.paint_speed_ready)
  tr.speed_limit_mps=0.;self.assertEqual(tr.update(at(child,3.01,.3),DT),(0.,0.))

 def test_final_motion_smoother_cannot_allow_spray_while_still_slowing(self):
  from line_mission import LineMission
  sec=section([(12.,0.)],1.,11.);tr=strict([sec]);child=tr.trackers[0]
  self.assertTrue(tr.start(at(child,3.,.8)));tr.update(at(child,3.,.8),DT)
  self.assertTrue(child.paint_speed_ready)
  m=LineMission.__new__(LineMission);m.tracker=tr;m.task='route'
  m._check_paint_output_speed(.95)
  self.assertFalse(child.paint_speed_ready)
  tr.update(at(child,3.02,.8),DT);m._check_paint_output_speed(.8)
  self.assertTrue(child.paint_speed_ready)

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

 def test_corner_lines_and_arc_are_constant_through_the_tail(self):
  out=plan(corner(),(-.3,.4,0.),CFG);tr=strict(sections(out));rows=drive(tr)
  self.assertEqual(tr.st.state,'DONE')
  for index,expected in ((0,.8),(1,.5),(2,.8)):
   values=[r['v'] for r in rows if r['index']==index and r['paint']]
   self.assertTrue(values);self.assertAlmostEqual(min(values),expected);self.assertAlmostEqual(max(values),expected)
  self.assertTrue(all(r['v']>0 for r in rows[:-1]))

if __name__=='__main__':unittest.main()
