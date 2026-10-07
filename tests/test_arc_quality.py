"""画圆位置不可信时，沿圆进度、轮速比例和航向换算也必须停止学习。"""
from pathlib import Path
from types import SimpleNamespace as NS
import math
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'radar_system'), str(ROOT)]
from path_tracker import PathConfig, PathTracker, circle, differential_config
from robot_core.kinematics import ChassisGeometry

DT=.02
GEO=ChassisGeometry(drive='differential', track_m=.675, diff_min_radius_m=.5)

def tracker():
    t=PathTracker(circle(2.4,left=False,lead_in=2.,lead_out=1.),differential_config(PathConfig(cruise_mps=.5)),GEO)
    t.set_nozzle(.24,-.4575)
    x,y,h=t.path.pose(0.)
    t.start(NS(x=x,y=y,yaw=h,speed=0.,yaw_rate=0.,usable_for_control=True,quality='RTK_FIXED',reasons=(),correction_xy=(0.,0.)))
    return t

def feed(tr, i, *, shift=0., yaw_error=0., quality='RTK_FIXED', receiver_bad=False):
    now=i*DT;s=2.5+.5*now
    _,_,truth_yaw=tr.path.pose(s)
    x,y,_=tr.path.pose(s+shift)
    tr.st.s=s
    tr.sync(now,(now,truth_yaw+1.7,-.5/2.4))
    tr.set_heading_baseline(now,.412)
    tr.set_rtk_quality(now,.04 if receiver_bad else .008,15 if receiver_bad else 30,.08 if receiver_bad else .005)
    # 故意保留上一帧未冻结状态；_arc_first 在 _control 更新它之前运行。
    tr.st.integral_frozen=False
    pose=NS(x=x,y=y,yaw=truth_yaw+yaw_error,speed=.5,yaw_rate=-.5/2.4,
            usable_for_control=True,quality=quality,reasons=(),correction_xy=(0.,0.))
    return tr._arc_first(pose,DT)

class ArcQuality(unittest.TestCase):
    def warmed_pair(self):
        a,b=tracker(),tracker()
        for i in range(1,151):
            feed(a,i);feed(b,i)
        self.assertEqual(a._arc_weight,1.)
        return a,b

    def test_quality_is_checked_on_the_current_frame(self):
        a,_=self.warmed_pair()
        feed(a,151,quality='RTK_FLOAT')
        self.assertFalse(a._arc_status['rtk_ok'])

    def test_untrusted_position_and_heading_do_not_distort_the_inertial_circle(self):
        for quality,receiver_bad in [('RTK_FLOAT',False),('RTK_FIXED',True)]:
            with self.subTest(quality=quality,receiver_bad=receiver_bad):
                a,b=self.warmed_pair()
                # 接收机连续两帧退化后，注入沿圆错位 0.4 m、融合航向错 2°。
                for i in range(151,154):
                    feed(a,i,quality=quality,receiver_bad=receiver_bad)
                    feed(b,i,quality=quality,receiver_bad=receiver_bad)
                for i in range(154,254):
                    va=feed(a,i,quality=quality,receiver_bad=receiver_bad)
                    vb=feed(b,i,shift=.4,yaw_error=math.radians(2.),quality=quality,receiver_bad=receiver_bad)
                    self.assertFalse(b._arc_status['rtk_ok'])
                    self.assertAlmostEqual(a._arc_s,b._arc_s,places=10)
                    self.assertAlmostEqual(a._arc_scale,b._arc_scale,places=10)
                    self.assertAlmostEqual(a._arc_off,b._arc_off,places=10)
                    self.assertAlmostEqual(va.x,vb.x,places=10)
                    self.assertAlmostEqual(va.y,vb.y,places=10)

    def test_trusted_progress_still_corrects_wheel_scale(self):
        a,b=self.warmed_pair()
        for i in range(151,251):
            feed(a,i);feed(b,i,shift=.04)
        self.assertTrue(b._arc_status['rtk_ok'])
        self.assertGreater(b._arc_s,a._arc_s+.01)
        self.assertGreater(b._arc_scale,a._arc_scale)

if __name__=='__main__':unittest.main()
