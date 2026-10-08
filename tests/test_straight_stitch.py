"""毫米级接入拼接弧保持直线控制连续，真实弯道和画出来的圆弧不受此例外影响。"""
from pathlib import Path
from types import SimpleNamespace as NS
import math,sys,unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT)]
from path_tracker import Path as Route,PathTracker,PathConfig,differential_config
from paint_start import PaintStartGuard
from robot_core.kinematics import ChassisGeometry
from motion_speeds import transit_bend_speed
GEO=ChassisGeometry(drive='differential',track_m=.675,diff_min_radius_m=.5);DT=.02

def tracker(bridge=.001160461653204159,k=-1.):
 path=Route((0.,0.,0.),[(2.1595074830350915,0.),(bridge,k),(9.233333333333333,0.)])
 tr=PathTracker(path,differential_config(PathConfig(cruise_mps=.8)),GEO)
 tr.paint_from_m=3.;tr.paint_to_m=11.;tr.runup_speed_mps=.3;tr.entry_stabilize_mps=.5
 tr.constant_straight_speed=True;tr.constant_paint_speed=True;tr.paint_start_guard=PaintStartGuard();tr.set_nozzle(0.,0.)
 return tr

def pose(tr,s,v=.5):
 x,y,h=tr.path.pose(s)
 return NS(x=x,y=y,yaw=h,speed=v,yaw_rate=v*tr.path.kappa(s),usable_for_control=True,quality='RTK_FIXED',
           rtk_trusted=True,reasons=(),correction_xy=(0.,0.),correction_yaw=0.)

class StraightStitch(unittest.TestCase):
 def test_imu_reference_and_speed_survive_the_recorded_millimetre_arc(self):
  tr=tracker();self.assertTrue(tr.start(pose(tr,0.,0.)));guide=None;values=[]
  for i in range(340):
   t=(i+1)*DT;s=i*.01;p=pose(tr,s)
   tr.sync(t,(t,p.yaw,p.yaw_rate));tr.set_heading_baseline(t,.412);tr.set_rtk_quality(t,.008,32,.005)
   v,_=tr.update(p,DT)
   if 1.5<s<3.2:
    guide=tr._straight_guide if guide is None else guide
    self.assertIs(tr._straight_guide,guide)
    self.assertGreaterEqual(tr._straight_weight,.99)
    values.append(v)
  self.assertTrue(tr.paint_start_allowed());self.assertTrue(tr.paint_start_guard.ready)
  self.assertTrue(all(abs(v-.5)<1e-6 for v in values))

 def test_geometry_and_paint_start_are_not_moved(self):
  tr=tracker();original=Route((0.,0.,0.),[(2.1595074830350915,0.),(.001160461653204159,-1.),(9.233333333333333,0.)])
  mark=tr.paint_from_m
  self.assertTrue(tr._straight_stitch(tr.path.pieces,1))
  self.assertEqual(mark,3.)
  for s in (0.,1.,2.1595,2.160,3.,5.,11.):self.assertEqual(original.pose(s),tr.path.pose(s))
  self.assertEqual(tr.path.kappa(2.160),-1.)
  tr._v_prev=.5
  self.assertGreaterEqual(tr._curve_speed(1.5,.5),.5)

 def test_real_turn_and_painted_small_arc_still_have_curve_limit(self):
  for bridge,k in ((.1,-1.),(.009,-1.),(.009,-1.7)):
   tr=tracker(bridge,k)
   self.assertFalse(tr._straight_stitch(tr.path.pieces,1))
   tr._v_prev=.5
   self.assertLessEqual(tr._curve_speed(2.1596,.5),transit_bend_speed(k,GEO.track_m,tr.cfg.lat_accel_mps2)+1e-9)
   self.assertLess(tr._curve_speed(2.1596,.5),.5)
  tr=tracker();tr.paint_from_m=0.;tr.paint_spans=[(0.,11.)]
  self.assertFalse(tr._straight_stitch(tr.path.pieces,1))

 def test_no_stitch_exception_at_start_or_end(self):
  tr=tracker();self.assertFalse(tr._straight_stitch(tr.path.pieces,0));self.assertFalse(tr._straight_stitch(tr.path.pieces,2))

 def test_receiver_noise_variants_are_the_same_straight_connection(self):
  for bridge,k in ((.00116046,-1.),(.00207655,-1.),(.00208052,1.)):
   with self.subTest(bridge=bridge,k=k):
    tr=tracker(bridge,k)
    self.assertTrue(tr._straight_stitch(tr.path.pieces,1))
    self.assertTrue(tr._same_straight_run(tr.path.pieces[0],tr.path.pieces[2]))

if __name__=='__main__':unittest.main()
