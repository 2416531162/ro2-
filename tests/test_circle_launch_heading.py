"""实录圆前助跑：低前进速度配大转向曾造成一轮前进、一轮后退。只验证软件。"""
import math
from unittest import TestCase

from test_smooth_runup import GEO, DT, pose
from path_tracker import Path as Route, Section, SectionTracker, PathConfig, differential_config


def tracker(k=-.40932478145,lead=2.5):
    sec=Section(1,Route((0.,0.,0.),[(lead,0.),(12.,k),(1.,0.)]))
    sec.field_role='paint';sec.paint_from_m=lead+2.;sec.paint_to_m=lead+12.
    route=SectionTracker([sec],differential_config(PathConfig(cruise_mps=.8)),GEO,
                         constant_straight_speed=True,constant_paint_speed=True)
    route.set_nozzle(0.,0.)
    return route.trackers[0]


def tick(tr,t,s=0.,v=0.,e=0.,heading=0.,imu=True):
    x,y,h=tr.path.pose(s)
    p=pose(x,v);p.y=y+e;p.yaw=h+heading;p.yaw_rate=v*tr.path.kappa(s)
    tr.sync(t,(t,p.yaw,p.yaw_rate) if imu else (t-.3,p.yaw,p.yaw_rate))
    tr.set_heading_baseline(t,.412);tr.set_rtk_quality(t,.008,32,.005)
    return tr.update(p,DT)


class CircleLaunch(TestCase):
    def test_low_speed_correction_keeps_both_wheel_targets_forward(self):
        for k in (-.40932478145,.654244):
            for sign in (-1.,1.):
                for imu in (True,False):
                    with self.subTest(k=k,sign=sign,imu=imu):
                        tr=tracker(k);p=pose();p.y=sign*.03;p.yaw=sign*math.radians(2.)
                        self.assertTrue(tr.start(p))
                        self.assertIsNone(tr.entry_stabilize_mps)
                        self.assertEqual(tr.runup_start_mps,0.)
                        x=v=0.
                        for i in range(1,41):
                            v,w=tick(tr,i*DT,x,v,e=sign*.03,heading=sign*math.radians(2.),imu=imu)
                            self.assertGreaterEqual(v-GEO.track_m*w/2.,-1e-12)
                            self.assertGreaterEqual(v+GEO.track_m*w/2.,-1e-12)
                            self.assertLessEqual(v,.35)
                            x+=v*DT

    def test_arc_still_turns_in_both_directions_at_original_speed(self):
        for k in (-.40932478145,.654244):
            with self.subTest(k=k):
                tr=tracker(k);tr.start(pose());s=v=0.;saw_arc=False
                for i in range(1,850):
                    v,w=tick(tr,i*DT,s,v)
                    if .9<s<1.3:
                        self.assertAlmostEqual(v,.35,places=7)
                    if s>2.8:
                        saw_arc=True
                        self.assertGreater(w*k,0.)
                        self.assertAlmostEqual(v,.5,places=7)
                    s+=v*DT
                    if s>4.5:break
                self.assertTrue(saw_arc)

    def test_actual_sent_history_uses_the_bounded_output(self):
        tr=tracker();tr.start(pose())
        v,w=tick(tr,.02,e=.03,heading=math.radians(2.))
        self.assertEqual(tr._sent[-1][1:],(v,w))
        self.assertLessEqual(abs(w),2.*v/GEO.track_m)

    def test_zero_speed_protection_cannot_leave_a_turn_command(self):
        tr=tracker();tr.start(pose());tick(tr,.02)
        tr.speed_limit_mps=0.
        self.assertEqual(tick(tr,.04,e=.03,heading=.04),(0.,0.))
