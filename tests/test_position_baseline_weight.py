"""GPS-epoch baseline comparison adjusts observation weight without vetoing RTK."""
import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT)]
import um982
import gnss_fusion as gf


class Clock:
    def __init__(self,offset=0.):self.offset=offset
    def measurement_time(self,week,tow,rx):return tow+self.offset


def antenna(t,which,length=.41,sigma=.01):
    lat=31.;phi=math.radians(lat);w=1-um982._WGS84_E2*math.sin(phi)**2
    lon=120. if which==1 else 120.+math.degrees(length/(um982._WGS84_A/math.sqrt(w)*math.cos(phi)))
    return um982.BestPos(2439,t,'SOL_COMPUTED','NARROW_INT',lat,lon,10.,sigma,sigma,.02,1.,40,35,which)


def stream():
    s=um982.Um982Stream(Clock());s.secondary_clock=Clock(.075)
    return s


def pair(s,t,length=.41,sigma=.01,aux_first=False):
    output=[]
    a,b=antenna(t,1,length,sigma),antenna(t,2,length,sigma)
    for m,rx in ((b,t+.025),(a,t+.030)) if aux_first else ((a,t+.020),(b,t+.025)):
        s._emit(m,rx,output)
    return output


class BaselineWeight(unittest.TestCase):
    def test_same_gps_epoch_matches_despite_independent_clock_offsets(self):
        s=stream();pair(s,1.)
        self.assertAlmostEqual(s._antenna_offset(),.41,places=7)
        self.assertTrue(s.position_baseline['epoch_aligned'])
        self.assertFalse(s.position_baseline['suspect'])

    def test_near_mono_times_do_not_pair_different_gps_epochs(self):
        s=um982.Um982Stream(Clock());s.secondary_clock=Clock()
        out=[];s._emit(antenna(1.,1),1.02,out);s._emit(antenna(1.02,2),1.03,out)
        self.assertIsNone(s._antenna_offset());self.assertIsNone(s.position_baseline)

    def test_sustained_disagreement_increases_only_variance(self):
        s=stream()
        for t in (1.,1.2,1.4):pair(s,t,.46)
        self.assertTrue(s.position_baseline['suspect'])
        out=[];s._emit(antenna(1.45,1),1.47,out)
        self.assertEqual(out[0]['sol'],'NARROW_INT')
        self.assertGreater(out[0]['position_variance_scale'],1.)
        self.assertLessEqual(out[0]['position_variance_scale'],25.)

    def test_single_spike_does_not_change_weight(self):
        s=stream();pair(s,1.,.46);pair(s,1.2,.41)
        self.assertFalse(s.position_baseline['suspect'])
        self.assertEqual(s.position_baseline['variance_scale'],1.)

    def test_threshold_tracks_reported_precision(self):
        s=stream()
        for t in (1.,1.2,1.4):pair(s,t,.46,sigma=.05)
        self.assertGreater(s.position_baseline['threshold_m'],.05)
        self.assertFalse(s.position_baseline['suspect'])

    def test_waiting_for_main_frame_preserves_continuity(self):
        s=stream()
        for t in (1.,1.2,1.4):pair(s,t,.46,aux_first=True)
        self.assertTrue(s.position_baseline['suspect'])

    def test_old_three_second_median_is_not_the_trigger(self):
        s=stream();s.baseline_pairs.extend([(0.,.3),(0.,.3),(0.,.3),(0.,.3),(0.,.3)])
        pair(s,1.,.41)
        self.assertFalse(s.position_baseline['suspect'])

    def test_stale_pair_is_not_reused_for_downweighting(self):
        s=stream()
        for t in (1.,1.2,1.4):pair(s,t,.46)
        out=[];s._emit(antenna(2.,1),2.02,out)
        self.assertIsNone(s.position_baseline)
        self.assertEqual(out[0].get('position_variance_scale',1.),1.)

    def test_fixed_pose_remains_trusted_with_larger_observation_variance(self):
        def run(scale):
            f=gf.GnssFusion(gf.FusionConfig(drive_differential=True,yaw_input='angle',rtk_hold_s=5.,
                                          antenna_xy=(0.,0.),baseline_yaw_body_deg=-90.))
            f.set_origin(31.,120.)
            def fix(t,x,y,variance=1.):
                lat,lon=f.to_geodetic(x,y)
                return gf.GnssFix(t,lat,lon,0.,'NARROW_INT',.01,.01,.02,1.,35,variance)
            f.add_wheel(gf.WheelSample(0.,0.,0.));f.add_imu(gf.ImuSample(0.,0.,0.))
            f.add_heading(gf.DualHeading(0.,180.,0.,.41,'NARROW_INT',.1,35))
            f.add_fix(fix(0.,0.,0.));f.add_fix2(fix(.1,0.,-.41))
            f.add_wheel(gf.WheelSample(.1,0.,0.));f.add_imu(gf.ImuSample(.1,0.,0.))
            f.add_heading(gf.DualHeading(.1,180.,0.,.41,'NARROW_INT',.1,35))
            f.add_fix(fix(.1,.015,0.,scale))
            return f,f.pose(.1)
        a,pa=run(1.);b,pb=run(9.)
        self.assertTrue(pb.rtk_trusted);self.assertTrue(pb.usable_for_control)
        self.assertEqual(pb.quality,'RTK_FIXED')
        self.assertEqual(b.calibration()['position_variance_scale'],9.)
        self.assertLess(abs(pb.x),abs(pa.x))

    def test_unfixed_arc_position_does_not_move_the_inertial_estimate(self):
        f=gf.GnssFusion(gf.FusionConfig(drive_differential=True,yaw_input='angle',rtk_hold_s=5.,antenna_xy=(0.,0.)))
        f.set_origin(31.,120.)
        f.add_fix(gf.GnssFix(0.,31.,120.,0.,'NARROW_INT',.01,.01,.02,1.,35))
        before=(f._s.x,f._s.y)
        lat,lon=f.to_geodetic(20.,0.)
        f.add_fix(gf.GnssFix(.1,lat,lon,0.,'NARROW_FLOAT',.01,.01,.02,1.,35,inertial_only=True))
        self.assertEqual((f._s.x,f._s.y),before)
        self.assertEqual(f._s.fixed_t,0.)
        self.assertEqual(f._s.position_quality,'DEAD_RECKONING')


if __name__=='__main__':unittest.main()
