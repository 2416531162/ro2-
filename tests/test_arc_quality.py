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
import paint_quality
from path_tracker import Section

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
            feed(a,i);feed(b,i,shift=.005)
        self.assertTrue(b._arc_status['rtk_ok'])
        self.assertGreater(b._arc_s,a._arc_s+.001)
        self.assertGreater(b._arc_scale,a._arc_scale)

    def test_fixed_solution_cannot_relearn_a_disagreement_after_quality_gap(self):
        a,b=self.warmed_pair()
        for i in range(151,166):
            feed(a,i,quality='RTK_FLOAT');feed(b,i,quality='RTK_FLOAT')
        scale,offset=b._arc_scale,b._arc_off
        for i in range(166,366):
            reference=feed(a,i,quality='RTK_FLOAT')
            shifted=feed(b,i,shift=.05,yaw_error=math.radians(1.2))
            self.assertTrue(b._arc_status['holding'])
            self.assertFalse(b._arc_status['rtk_ok'])
            self.assertEqual(b._arc_scale,scale)
            self.assertEqual(b._arc_off,offset)
            self.assertAlmostEqual(reference.x,shifted.x,places=9)
            self.assertAlmostEqual(reference.y,shifted.y,places=9)
        self.assertGreater(b._arc_status['hold_age_s'],3.)
        for i in range(366,415):
            feed(b,i)
            self.assertTrue(b._arc_status['holding'])
        feed(b,415)
        self.assertFalse(b._arc_status['holding'])
        self.assertEqual(b._arc_status['correction_weight'],0.)
        feed(b,416)
        self.assertGreater(b._arc_status['correction_weight'],0.)
        self.assertLess(b._arc_status['correction_weight'],.1)

    def test_mismatch_uses_existing_eight_second_paint_and_nine_second_motion_limits(self):
        tr,_=self.warmed_pair();tr.set_nozzle(0.,0.)
        for i in range(151,550):feed(tr,i,shift=.05)
        self.assertTrue(tr._arc_status['holding'])
        def sample():
            x,y,h=tr.path.pose(tr.st.s)
            return NS(x=x,y=y,yaw=h,speed=.5,yaw_rate=-.5/2.4,usable_for_control=True,
                      quality='RTK_FIXED',rtk_trusted=True,reasons=(),hold_age_s=.05)
        m=NS(pose=sample(),nozzle=(0.,0.),differential=True,fusion=NS(config=NS(rtk_hold_s=9.)),
             _route_local=lambda:NS(kind='circle',center=(2.,-2.4),radius=2.4))
        q=lambda:paint_quality.status(m,{},True,Section(1,tr.path),tr)
        self.assertTrue(q()['painting']);self.assertEqual(q()['paint_control_source'],'imu_hold')
        feed(tr,550,shift=.05)
        self.assertEqual(q()['paint_blocked'],'paint_position_untrusted')
        for i in range(551,601):feed(tr,i,shift=.05)
        tr._hint=tr.st.s
        self.assertEqual(tr._control(sample(),DT),(0.,0.,0.))
        self.assertEqual(tr.st.state,'PAUSED')
        self.assertEqual(tr.st.reason,'arc_position_disagreement')
        self.assertFalse(tr.position_reference_ready())
        self.assertFalse(tr.start(sample()))

    def test_wheel_or_imu_failure_during_hold_does_not_fall_back_to_rejected_position(self):
        for failure in ('wheel_hold','imu_hold'):
            tr,_=self.warmed_pair();feed(tr,151,shift=.05)
            self.assertTrue(tr._arc_status['holding'])
            x,y,h=tr.path.pose(tr.st.s)
            p=NS(x=x,y=y,yaw=h,speed=.5,yaw_rate=-.5/2.4,usable_for_control=True,
                 quality='RTK_FIXED',rtk_trusted=True,reasons=(failure,),correction_xy=(0.,0.))
            self.assertEqual(tr.update(p,DT),(0.,0.))
            self.assertEqual(tr.st.state,'PAUSED')
            self.assertFalse(tr.position_reference_ready())

if __name__=='__main__':unittest.main()
