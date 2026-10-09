"""Raw paired antenna positions must be timed and transformed to the nozzle correctly."""
import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace as NS

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT),str(ROOT/'tests')]
from path_tracker import PathTracker,PathConfig,circle,differential_config
from robot_core.kinematics import ChassisGeometry
from test_circle_radial_rtk import tracker

GEO=ChassisGeometry(drive='differential',track_m=.675,diff_min_radius_m=.5)
NOZZLE=(.24,-.4575)
ANTENNAS=((.24,.2075),(.24,-.2075))


def setup(left=False,offset=0.):
    tr=PathTracker(circle(3.,left=left,lead_in=2.,lead_out=1.),differential_config(PathConfig()),GEO)
    tr.set_nozzle(*NOZZLE)
    piece=tr.path.pieces[1]
    tr._arc_off=-1.7
    for i in range(101):
        t=i*.02
        _,_,h=tr.path.pose(2.5+.5*t)
        tr.sync(t,(t,h+1.7,.5*piece.kappa))
    def fix(t,lever,shift=offset):
        x,y,h=tr.path.pose(2.5+.5*t)
        x-=math.sin(h)*shift;y+=math.cos(h)*shift
        ax,ay=lever
        return t,x+ax*math.cos(h)-ay*math.sin(h),y+ax*math.sin(h)+ay*math.cos(h),True,lever
    tr.set_primary_fix(*fix(1.92,ANTENNAS[0]))
    tr.set_secondary_fix(*fix(1.84,ANTENNAS[1]))
    return tr,piece,fix


def feed_paired(tr,i,*,fused_bias=0.,gnss_shift=0.,secondary_shift=0.):
    now=i*.02;s=2.5+.5*now
    x,y,h=tr.path.pose(s)
    tr.st.s=s
    tr.sync(now,(now,h+1.7,-.5/2.4))
    tr.set_heading_baseline(now,.412)
    tr.set_rtk_quality(now-.08,.01,24,.005,True)
    for age,lever,shift,setter in ((.08,ANTENNAS[0],gnss_shift,tr.set_primary_fix),
                                  (.16,ANTENNAS[1],gnss_shift+secondary_shift,tr.set_secondary_fix)):
        stamp=now-age
        px,py,ph=tr.path.pose(2.5+.5*stamp)
        px-=math.sin(ph)*shift;py+=math.cos(ph)*shift
        ax,ay=lever
        setter(stamp,px+ax*math.cos(ph)-ay*math.sin(ph),py+ax*math.sin(ph)+ay*math.cos(ph),True,lever)
    pose=NS(x=x-math.sin(h)*fused_bias,y=y+math.cos(h)*fused_bias,yaw=h,
            speed=.5,yaw_rate=-.5/2.4,usable_for_control=True,quality='RTK_FIXED',
            reasons=(),correction_xy=(0.,0.),rtk_trusted=True)
    tr._arc_first(pose,.02)
    return tr._arc_status


class CircleRawRadial(unittest.TestCase):
    def test_delayed_antenna_samples_match_nozzle_radius_in_both_directions(self):
        for left in (False,True):
            for shift in (-.03,0.,.03):
                with self.subTest(left=left,shift=shift):
                    tr,p,fix=setup(left,shift)
                    k=p.kappa;nx,ny=NOZZLE
                    target=math.hypot(nx,1./k-ny)
                    observed=math.hypot(nx,1./k-ny-shift)
                    expected=-math.copysign(1.,k)*(observed-target)
                    self.assertAlmostEqual(tr._arc_raw_radial(p),expected,places=8)

    def test_missing_stale_float_or_disagreeing_antenna_is_not_used(self):
        for mode in ('missing','stale','float','disagree'):
            tr,p,fix=setup()
            if mode=='missing':tr._primary=None
            if mode=='stale':tr.set_secondary_fix(*fix(1.,ANTENNAS[1]))
            if mode=='float':
                f=list(fix(1.84,ANTENNAS[1]));f[3]=False;tr.set_secondary_fix(*f)
            if mode=='disagree':tr.set_secondary_fix(*fix(1.84,ANTENNAS[1],.04))
            self.assertIsNone(tr._arc_raw_radial(p),mode)

    def test_yaw_interpolation_crosses_wrap_without_rotating_the_wrong_way(self):
        tr,_,_=setup()
        tr._imu_history.clear()
        tr._imu_history.extend([(1.,math.radians(179),.2),(1.1,math.radians(-179),.2)])
        self.assertAlmostEqual(abs(tr._imu_yaw_at(1.05)),math.pi,places=10)
        self.assertIsNone(tr._imu_yaw_at(.9))
        self.assertIsNone(tr._imu_yaw_at(1.3))

    def test_paired_positions_correct_bias_left_in_the_fused_position(self):
        tr=tracker()
        for i in range(1,501):state=feed_paired(tr,i,fused_bias=.02)
        self.assertLess(abs(state['e_rtk_raw']),1e-7)
        self.assertLess(abs(state['e_est']),.004)
        self.assertGreater(state['e_rtk'],.019)

    def test_common_raw_jump_is_held_before_fusion_has_followed_it(self):
        tr=tracker()
        for i in range(1,301):feed_paired(tr,i)
        for i in range(301,341):
            state=feed_paired(tr,i,gnss_shift=.04)
            self.assertTrue(state['holding'])
            self.assertEqual(state['radial_source'],'imu_hold')
            self.assertEqual(state['radial_correction_m'],0.)
        self.assertLess(abs(state['e_est']),.003)
        state=feed_paired(tr,341)
        self.assertFalse(state['holding'])

    def test_raw_radial_disagreement_blocks_fast_correction_even_if_length_metadata_passes(self):
        tr=tracker()
        for i in range(1,301):feed_paired(tr,i)
        for i in range(301,351):state=feed_paired(tr,i,secondary_shift=.04)
        self.assertIsNone(state['e_rtk_raw'])
        self.assertEqual(state['radial_rtk_weight'],0.)


if __name__=='__main__':unittest.main()
