"""Reliable paired GNSS corrects circle radial drift; IMU holds across bad observations."""
from dataclasses import replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'radar_system'), str(ROOT)]
from path_tracker import PathConfig, PathTracker, circle, differential_config
from robot_core.kinematics import ChassisGeometry

DT = .02
GEO = ChassisGeometry(drive='differential', track_m=.675, diff_min_radius_m=.5)


def tracker(enabled=True):
    cfg = replace(differential_config(PathConfig(cruise_mps=.5)), arc_rtk_radial_enabled=enabled)
    tr = PathTracker(circle(2.4, left=False, lead_in=2., lead_out=1.), cfg, GEO)
    tr.set_nozzle(.24, -.4575)
    x,y,h = tr.path.pose(0.)
    tr.start(NS(x=x, y=y, yaw=h, speed=0., yaw_rate=0., usable_for_control=True,
                quality='RTK_FIXED', reasons=(), correction_xy=(0.,0.)))
    return tr


def feed(tr, i, *, offset=0., pair=.005, sigma=.01, fixed=True, secondary_fixed=True,
         fresh=True, observe=True, quality='RTK_FIXED'):
    now = i*DT
    s = 2.5+.5*now
    x,y,h = tr.path.pose(s)
    x -= math.sin(h)*offset
    y += math.cos(h)*offset
    tr.st.s = s
    tr.sync(now, (now,h+1.7,-.5/2.4))
    tr.set_heading_baseline(now,.412)
    if observe:
        stamp = now if fresh else now-1.
        tr.set_rtk_quality(stamp,sigma,24,pair,fixed)
        tr.set_secondary_fix(stamp,x,y,secondary_fixed,(.24,-.2025))
    pose = NS(x=x,y=y,yaw=h,speed=.5,yaw_rate=-.5/2.4,usable_for_control=True,
              quality=quality,reasons=(),correction_xy=(0.,0.),rtk_trusted=quality=='RTK_FIXED')
    tr._arc_first(pose,DT)
    return dict(tr._arc_status)


class CircleRadialRtk(unittest.TestCase):
    def warmed(self, enabled=True):
        tr=tracker(enabled)
        for i in range(1,201): feed(tr,i)
        self.assertTrue(tr._arc_status['active'])
        return tr

    def test_consistent_position_sees_lateral_drift_that_imu_cannot_observe(self):
        old,new=self.warmed(False),self.warmed()
        # 侧向滑移持续两秒，车头角不变：可靠双天线都看到偏移。
        for i in range(201,351):
            offset=.05*min(1.,(i-200)/100)
            before=feed(old,i,offset=offset)
            after=feed(new,i,offset=offset)
        self.assertEqual(after['radial_source'],'rtk_paired')
        self.assertLess(abs(after['e_rtk']-after['e_est']),.012)
        self.assertLess(abs(after['e_rtk']-after['e_est']),
                        .5*abs(before['e_rtk']-before['e_est']))

    def test_pair_missing_disagreeing_or_stale_never_enables_fast_correction(self):
        for args in (dict(pair=None),dict(pair=.04),dict(sigma=.04),dict(fixed=False),
                     dict(secondary_fixed=False),dict(fresh=False)):
            with self.subTest(args=args):
                tr=self.warmed()
                for i in range(201,251): state=feed(tr,i,offset=.02,**args)
                self.assertEqual(state['radial_rtk_weight'],0.)
                self.assertNotEqual(state['radial_source'],'rtk_paired')

    def test_one_fresh_frame_cannot_count_as_continuous_confirmation(self):
        tr=self.warmed(False)
        tr.cfg=replace(tr.cfg,arc_rtk_radial_enabled=True)
        feed(tr,201)
        for i in range(202,241):
            state=feed(tr,i,observe=False)
            self.assertEqual(state['radial_rtk_weight'],0.)

    def test_bad_fix_holds_prediction_without_absorbing_position_jump(self):
        a,b=self.warmed(),self.warmed()
        for i in range(201,251):
            sa=feed(a,i,quality='RTK_FLOAT',fixed=False)
            sb=feed(b,i,offset=.15,quality='RTK_FLOAT',fixed=False)
            self.assertEqual(sb['radial_source'],'imu_hold')
            self.assertAlmostEqual(sa['e_est'],sb['e_est'],places=10)

    def test_paired_step_is_still_rejected_and_corrections_remain_bounded(self):
        tr=self.warmed()
        for i in range(201,251):
            state=feed(tr,i,offset=.04)
            self.assertTrue(state['holding'])
            self.assertEqual(state['radial_rtk_weight'],0.)
            self.assertEqual(state['radial_correction_m'],0.)
        self.assertLess(abs(state['e_est']),.005)
        for i in range(251,351):
            state=feed(tr,i,offset=.04)
            self.assertLessEqual(abs(state['radial_correction_m']),.12*.5*DT+1e-10)

    def test_radial_weight_does_not_change_imu_heading_or_progress(self):
        a,b=self.warmed(False),self.warmed()
        for i in range(201,281):
            sa=feed(a,i,offset=.02*(i-200)/80)
            sb=feed(b,i,offset=.02*(i-200)/80)
            for key in ('offset_deg','heading_deg','arc_s','scale'):
                self.assertAlmostEqual(sa[key],sb[key],places=10)

    def test_millimetre_position_noise_keeps_the_original_smooth_response(self):
        old,new=self.warmed(False),self.warmed()
        for i in range(201,351):
            before=feed(old,i,offset=.001*math.sin(i))
            after=feed(new,i,offset=.001*math.sin(i))
            self.assertEqual(after['radial_rtk_weight'],0.)
            self.assertAlmostEqual(before['e_est'],after['e_est'],places=10)

    def test_fast_correction_cannot_hide_a_smoothed_common_position_jump(self):
        old,new=self.warmed(False),self.warmed()
        first_hold={}
        for i in range(201,291):
            # 融合把共同跳点平滑展开；跳点判断仍须和原来的慢修估计一致。
            offset=.04*min(1.,(i-200)/30)
            for name,tr in (('old',old),('new',new)):
                state=feed(tr,i,offset=offset)
                if state['holding']: first_hold.setdefault(name,i)
        self.assertIn('old',first_hold)
        self.assertEqual(first_hold['new'],first_hold['old'])


if __name__=='__main__': unittest.main()
