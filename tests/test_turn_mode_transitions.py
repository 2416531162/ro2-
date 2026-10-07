"""圆弧掉头的多条直边和闭合路线不能丢失稳行阶段或提前触发收尾。"""
from pathlib import Path
from dataclasses import replace
import math,sys,unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT),str(ROOT/'tests')]
import route_plan as rp
from live_cruise import label_sections
from path_tracker import PathConfig,PathTracker,SectionTracker,differential_config,Path as Route
from test_continuous_field_arcs import GEO,pose,DT

def route(closed=False):
 points=((0.,0.),(8.,0.),(8.,5.),(0.,5.)) if closed else ((0.,0.),(8.,0.),(8.,6.))
 spec=rp.RouteSpec(kind='path',points=points,closed=closed,cruise=.8,turn='arc',turn_radius=1.)
 car=rp.axle_for((-3.,0.),0.,.24,-.4575)+(0.,)
 secs,entry=rp.plan_current(spec,car,.556,nozzle_x=(.24,-.4575));label_sections(secs,entry,.8)
 tr=SectionTracker(secs,replace(differential_config(PathConfig(cruise_mps=.8)),straight_imu_enabled=False),GEO,
                   constant_straight_speed=True,constant_paint_speed=True)
 tr.set_nozzle(.24,-.4575)
 return tr

def at(tr,s,v):
 tr._hint=s
 return pose(*tr.path.pose(s),v,v*tr.path.kappa(s))

class TurnModeTransitions(unittest.TestCase):
 def test_next_straight_in_same_tracker_gets_its_own_stabilization(self):
  tr=route().trackers[0];self.assertTrue(tr.start(at(tr,0.,0.)));tr.update(at(tr,0.,0.),DT)
  first,second=tr.paint_spans[:2]
  tr._entry_speed_active=False;tr._entry_ramp_released=True;tr._v_prev=.8
  tr.update(at(tr,first[1]+.1,.8),DT)
  self.assertTrue(tr._entry_speed_active);self.assertFalse(tr._entry_ramp_released)
  self.assertEqual(tr._entry_hold_from,second[0])
  tr.update(at(tr,second[0]+.25,.5),DT)
  self.assertEqual(tr.paint_target_mps,.5)

 def test_dashed_gaps_on_same_straight_do_not_restart_entry(self):
  tr=PathTracker(Route((0.,0.,0.),[(12.,0.)]),differential_config(PathConfig(cruise_mps=.8)),GEO)
  tr.constant_paint_speed=True;tr.entry_stabilize_mps=.5;tr.paint_spans=[(1.,3.),(4.,8.)];tr.paint_from_m=1.;tr.paint_to_m=8.
  self.assertTrue(tr.start(at(tr,0.,0.)));tr.update(at(tr,0.,0.),DT)
  tr._entry_speed_active=False;tr._entry_ramp_released=True;tr._v_prev=.8
  tr.update(at(tr,3.5,.8),DT);tr.update(at(tr,4.2,.8),DT)
  self.assertFalse(tr._entry_speed_active);self.assertEqual(tr.paint_target_mps,.8)

 def test_closed_route_start_near_end_plane_does_not_trigger_runout(self):
  tr=route(closed=True).trackers[0];p=at(tr,0.,0.);p.y-=.005
  self.assertTrue(tr.start(p));remaining,_=tr.nozzle_end(p,*tr._straight_nozzle,at=tr.paint_to_m)
  self.assertLess(remaining,0.)
  v,_=tr.update(p,DT)
  self.assertFalse(tr._run_out_started);self.assertGreaterEqual(v,.35)

 def test_curve_profile_keeps_spray_ready_across_preview_jitter(self):
  tr=PathTracker(Route((0.,0.,0.),[(12.,0.)]),differential_config(PathConfig(cruise_mps=.8)),GEO)
  tr.constant_paint_speed=True;tr.paint_from_m=1.;tr.paint_to_m=11.
  self.assertTrue(tr.start(at(tr,5.,.8)))
  tr._v_prev=.7999;tr._accel_prev=-.009;tr._curve_speed_active=True
  v,_=tr.update(at(tr,5.,.8),DT)
  self.assertLess(v,.8);self.assertTrue(tr.paint_speed_ready)
  tr.speed_limit_mps=.1
  tr.update(at(tr,5.01,.8),DT)
  self.assertFalse(tr.paint_speed_ready)

if __name__=='__main__':unittest.main()
