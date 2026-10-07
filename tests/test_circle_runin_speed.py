"""圆上预走与直线助跑分开：已进圆后不得重新降到 0.3 m/s。"""
from pathlib import Path
from types import SimpleNamespace as NS
import math
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT)]
from path_tracker import Path as Route, PathConfig, PathTracker, differential_config
from robot_core.kinematics import ChassisGeometry

GEO=ChassisGeometry(drive='differential',track_m=.675,diff_min_radius_m=.5)

def pose(path,s,v=.5):
    x,y,h=path.pose(s)
    return NS(x=x,y=y,yaw=h,speed=v,yaw_rate=v*path.kappa(s),usable_for_control=True,
              quality='RTK_FIXED',reasons=(),correction_xy=(0.,0.),correction_yaw=0.)

def tracker(curvature):
    path=Route((0.,0.,0.),[(1.5,0.),(20.,curvature)])
    tr=PathTracker(path,differential_config(PathConfig(cruise_mps=.8)),GEO)
    tr.paint_from_m=1.5+2./(abs(curvature)*2.) if curvature else 4.
    tr.runup_speed_mps=.3;tr.runup_distance_m=1.5
    tr.constant_straight_speed=True
    tr.start(pose(path,0.,0.))
    return tr

class CircleRunInSpeed(unittest.TestCase):
    def test_both_directions_keep_half_mps_through_the_unpainted_circle(self):
        for k in (-.40932478145,.654244):
            with self.subTest(curvature=k):
                tr=tracker(k);tr._v_prev=.5;tr._creep_from=0.
                for i in range(120):
                    s=1.5+(tr.paint_from_m-1.5)*i/120
                    tr.st.s=s
                    v,_=tr.update(pose(tr.path,s),.02)
                    self.assertAlmostEqual(v,.5,places=10)

    def test_straight_approach_is_slow_before_reaching_the_circle(self):
        tr=tracker(-.40932478145);tr._v_prev=.3;tr._creep_from=0.
        tr.st.s=.4
        v,_=tr.update(pose(tr.path,.4,v=.3),.02)
        self.assertAlmostEqual(v,.3,places=10)

    def test_plain_straight_runup_no_longer_waits_at_point_three(self):
        tr=tracker(0.);tr._v_prev=.3;tr._creep_from=0.;tr.st.s=2.8;tr._hint=2.8
        v,_=tr.update(pose(tr.path,2.8,v=.3),.02)
        self.assertGreater(v,.3)
        self.assertLessEqual(v,.3+tr.cfg.accel_mps2*.02)
        tr._v_prev=.8;tr.st.s=3.8;tr._hint=3.8
        v,_=tr.update(pose(tr.path,3.8,v=.8),.02)
        self.assertAlmostEqual(v,.8,places=10)

    def test_hard_stop_overrides_arc_runin_speed(self):
        tr=tracker(-.40932478145);tr._v_prev=.5;tr.st.s=2.
        tr.speed_limit_mps=0.
        self.assertEqual(tr.update(pose(tr.path,2.),.02),(0.,0.))

    def test_distant_straight_approach_keeps_its_cruise_speed(self):
        path=Route((0.,0.,0.),[(10.,0.),(20.,-.40932478145)])
        tr=PathTracker(path,differential_config(PathConfig(cruise_mps=.8)),GEO)
        tr.paint_from_m=12.44;tr.runup_speed_mps=.3;tr.runup_distance_m=1.5
        tr.constant_straight_speed=True;tr.start(pose(path,0.,0.))
        tr._v_prev=.8;tr._creep_from=0.;tr.st.s=1.;tr._hint=1.
        self.assertAlmostEqual(tr.update(pose(path,1.,v=.8),.02)[0],.8,places=10)

if __name__=='__main__':unittest.main()
