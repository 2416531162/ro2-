"""圆弧走法：整场不停顿交接，线尾停喷、最终缓停，起步提速圆滑。只使用软件底盘。"""
from collections import deque
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
import copy,json,math,sys,unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT)]
import route_plan as rp
from field_planner import Settings,plan,transit,polygon
from path_tracker import Path as Route,Section,SectionTracker,PathConfig,differential_config
from route_operations import RouteOperations,tail
from robot_core.kinematics import ChassisGeometry

GEO=ChassisGeometry(drive='differential',track_m=.675,diff_min_radius_m=.5)
CFG=Settings(.24,.4575,(.911,.604,.39),.557,turn='arc',turn_radius=1.,lateral_accel=.2)
DT=.02

def corner():
 return dict(execution_order='corner',boundary=[[0,0],[6,0],[6,6],[0,6]],elements=[
  dict(id='corner-leg-1',kind='polyline',points=[[0,0],[6,0]]),
  dict(id='corner-arc',kind='arc',center=[6,0],radius=1,start_deg=90,sweep_deg=90),
  dict(id='corner-leg-2',kind='polyline',points=[[6,0],[6,6]])])

def sections(out):
 result=[]
 for m in out['motions']:
  s=Section(m['direction'],Route(m['start'],m['pieces']))
  s.field_role=m['role'];s.cruise_mps=m['speed'];s.entry_speed_mps=m.get('entry_speed')
  s.paint_from_m=m.get('paint_from',0.);s.paint_to_m=m.get('paint_to');s.continuous_after=m.get('continuous_after',False)
  result.append(s)
 return result

def tracker(secs):
 # 本组是无 IMU 的纯运动学模型；起喷传感器检查由 test_paint_start 和完整任务仿真覆盖。
 tr=SectionTracker(secs,replace(differential_config(PathConfig(cruise_mps=.8)),straight_imu_enabled=False),GEO,pivot_rate=.8,constant_straight_speed=True)
 tr.set_nozzle(.24,-.4575)
 return tr

def pose(x,y,h,v=0.,w=0.):
 return NS(x=x,y=y,yaw=h,speed=v,yaw_rate=w,usable_for_control=True,quality='RTK_FIXED',reasons=(),rtk_trusted=True,
           sigma_xy=.005,sigma_yaw=.005,correction_xy=(0.,0.),correction_yaw=0.)

def drive(tr,limit=250.):
 x,y,h=tr.sections[0].car_pose(0.);v=w=0.;q=deque([(0.,0.)]*15)
 assert tr.start(pose(x,y,h))
 rows=[]
 # 使用生产喷线资格，不能用后轴路程代替带喷头偏置的真实停喷判定。
 mission=NS(route_path=None,tracker=tr,route=None,differential=True,phase='RUNNING',nozzle=(.24,-.4575))
 ops=RouteOperations(mission)
 ops.record=dict(id='kinematic',field_plan=True,field_context=dict(forward=.24,right=.4575))
 for i in range(int(limit/DT)):
  p=pose(x,y,h,v,w);vc,wc=tr.update(p,DT)
  child=tr.trackers[tr.index];sec=tr.sections[tr.index]
  mission.pose=p;ops.update_paint_tail(i*DT)
  spray=ops.paint_status()
  rows.append(dict(t=i*DT,index=tr.index,s=child.st.s,v=vc,w=wc,switching=tr.switching,e=child.st.e,
      paint=spray['painting']))
  if tr.st.state!='RUNNING':break
  q.append((vc,wc));v,w=q.popleft();x+=v*DT*math.cos(h+w*DT/2);y+=v*DT*math.sin(h+w*DT/2);h+=w*DT
 return rows

class ContinuousFieldArcs(unittest.TestCase):
 def test_corner_from_both_ends_finishes_without_an_intermediate_stop(self):
  for start in ((-.3,.4,0.),(5.5,6.3,-math.pi/2)):
   with self.subTest(start=start):
    out=plan(corner(),start,CFG);tr=tracker(sections(out));rows=drive(tr)
    self.assertEqual(tr.st.state,'DONE',tr.status())
    self.assertEqual(out['reverse_count'],0);self.assertEqual(out['pivot_count'],0)
    self.assertEqual(set(r['index'] for r in rows),set(range(len(tr.sections))))
    self.assertFalse(any(r['switching'] for r in rows))
    self.assertTrue(all(r['v']>0 for r in rows[:-1]))
    for a,b in zip(rows,rows[1:]):
     self.assertLessEqual((b['v']-a['v'])/DT,.250001)
     if a['index']!=b['index']:
      self.assertGreater(b['v'],.25)
      self.assertLess(abs(b['v']-a['v']),.015)
    self.assertTrue(any(r['paint'] for r in rows))
    last_paint=max(i for i,r in enumerate(rows) if r['paint'])
    self.assertGreater(rows[last_paint]['v'],.6)
    self.assertGreater(len(rows)-last_paint,10)

 def test_imported_full_football_field_has_no_stop_or_heading_jump_at_seams(self):
  job=json.loads((ROOT/'tests/fixtures/football_continuous.json').read_text())
  out=plan(job,(1.,66.,0.),CFG);tr=tracker(sections(out));rows=drive(tr,5000.)
  self.assertEqual(tr.st.state,'DONE',tr.status())
  self.assertEqual(len(out['order']),24)
  self.assertEqual((out['pivot_count'],out['reverse_count']),(0,0))
  self.assertFalse(any(r['switching'] for r in rows))
  self.assertTrue(all(r['v']>0 for r in rows[:-1]))
  seams=[(a,b) for a,b in zip(rows,rows[1:]) if a['index']!=b['index']]
  self.assertEqual(len(seams),23)
  for a,b in seams:
   self.assertGreater(b['v'],.25)
   self.assertLess(abs(b['v']-a['v']),.01)
   self.assertLess(abs(b['w']-a['w']),.015)

 def test_real_spray_gate_closes_in_transit_while_vehicle_keeps_moving(self):
  out=plan(corner(),(-.3,.4,0.),CFG);tr=tracker(sections(out))
  m=NS(route_path=None,tracker=tr,route=None,differential=True,phase='RUNNING',nozzle=(.24,-.4575))
  ops=RouteOperations(m);ops.record=dict(id='test',field_plan=True,field_context=dict(forward=.24,right=.4575))
  tr.st.state='RUNNING'
  for i,sec in enumerate(tr.sections):
   tr.index=i;child=tr.trackers[i];child.st.state='RUNNING'
   # 转场/起喷前、正式线中、尾端停喷后都读生产喷涂判据。
   for s,want in ((sec.paint_from_m-.15,False),((sec.paint_from_m+sec.paint_to_m)/2,True),(sec.paint_to_m+.1,False)):
    child.st.s=s;child.st.kappa_ref=sec.path.kappa(s)
    m.pose=pose(*sec.car_pose(s),.5,.5*sec.path.kappa(s))
    ops.update_paint_tail(s+i*100)
    self.assertEqual(ops.paint_status()['painting'],want,(i,s,ops.paint_status()))
    self.assertEqual(m.pose.speed,.5)

 def test_arc_paint_end_is_original_endpoint_and_tail_is_dry(self):
  job=corner();job['selected_strokes']=[dict(element_id='corner-arc',edge_index=None)]
  out=plan(job,(-.3,.4,0.),CFG);tr=tracker(sections(out));sec=tr.sections[-1]
  self.assertAlmostEqual(sec.path.length-sec.paint_to_m,rp.CIRCLE_RUN_OUT_M)
  q=rp.nozzle_at(*sec.car_pose(sec.paint_to_m),.24,-.4575)
  self.assertLess(min(math.dist(q,p) for p in ((5.,0.),(6.,1.))),1e-6)
  rows=drive(tr);self.assertEqual(tr.st.state,'DONE',tr.status())
  last=max(i for i,r in enumerate(rows) if r['paint']);self.assertAlmostEqual(rows[last]['v'],.5,places=5)
  self.assertTrue(any(not r['paint'] and r['v']>.1 for r in rows[last+1:]))

 def test_forward_only_even_when_outside_space_is_large(self):
  _,planes=polygon([[0,0],[1,0],[1,1],[0,1]])
  secs,cost,turns=transit((.5,.5,0.),(.5,.5,math.pi),planes,CFG)
  self.assertTrue(secs);self.assertTrue(all(s.direction==1 for s in secs));self.assertFalse(turns)
  self.assertGreater(cost[1],1.)

 def test_discontinuous_or_reverse_flags_are_rejected(self):
  for direction,start in ((-1,(1.,0.,math.pi)),(1,(1.1,0.,0.)),(1,(1.,0.,.1))):
   a=Section(1,Route((0.,0.,0.),[(1.,0.)]));a.continuous_after=True
   b=Section(direction,Route(start,[(2.,0.)]))
   with self.assertRaises(ValueError):tracker([a,b])

 def test_pause_and_bad_pose_still_stop_immediately(self):
  for reason in ('pause','pose','limit'):
   tr=tracker(sections(plan(corner(),(-.3,.4,0.),CFG)))
   x,y,h=tr.sections[0].car_pose(0.);p=pose(x,y,h,.5,0.)
   self.assertTrue(tr.start(p));tr.trackers[0]._v_prev=.5
   if reason=='pause':tr.pause()
   if reason=='pose':p.usable_for_control=False
   if reason=='limit':tr.speed_limit_mps=0.
   self.assertEqual(tr.update(p,DT),(0.,0.))

 def test_failed_rolling_start_remains_stopped_on_following_ticks(self):
  a=Section(1,Route((0.,0.,0.),[(1.,0.)]));a.continuous_after=True
  b=Section(1,Route((1.,0.,0.),[(5.,0.)]))
  tr=tracker([a,b]);self.assertTrue(tr.start(pose(0.,0.,0.)))
  tr.trackers[0]._hint=1.;tr.trackers[0]._v_prev=.5
  p=pose(1.01,0.,math.radians(35),.5,0.)
  for _ in range(5):self.assertEqual(tr.update(p,DT),(0.,0.))
  self.assertEqual(tr.st.state,'PAUSED')
  self.assertEqual(tr.trackers[0].st.state,'PAUSED')

 def test_saved_and_cut_routes_keep_continuations(self):
  secs=sections(plan(corner(),(-.3,.4,0.),CFG))
  ops=RouteOperations.__new__(RouteOperations);ops.m=NS(fusion=NS(to_geodetic=lambda x,y:(x,y),to_local=lambda x,y:(x,y)))
  raw=ops.encode(secs);restored=ops.decode({'plan':json.loads(json.dumps(raw))})
  self.assertEqual([s.continuous_after for s in secs],[s.continuous_after for s in restored])
  cut=tail(restored,.2);self.assertTrue(cut[0].continuous_after);tracker(cut)
  # 旧作业没有此字段时仍按原停车流程，不能擅自补出连续运行。
  for s in raw:s.pop('continuous_after')
  self.assertFalse(any(s.continuous_after for s in ops.decode({'plan':raw})))

 def test_restart_ramp_has_bounded_acceleration_change(self):
  sec=Section(1,Route((0.,0.,0.),[(5.,0.),(8.,-.5)]));sec.field_role='paint';sec.paint_from_m=3.
  tr=tracker([sec]).trackers[0]
  speeds=[]
  for target in [.5]*180+[.8]*220:
   v=tr._accelerate(target,DT);speeds.append(v);tr._v_prev=v
  a=[(b-a)/DT for a,b in zip([0.]+speeds,speeds)]
  self.assertLessEqual(max(a),.250001)
  self.assertLessEqual(max(abs(b-a)/DT for a,b in zip([0.]+a,a)),.500001)
  self.assertAlmostEqual(speeds[179],.5,places=5);self.assertAlmostEqual(speeds[-1],.8,places=5)
  tr.pause();self.assertEqual(tr._accel_prev,0.)

if __name__=='__main__':unittest.main()
