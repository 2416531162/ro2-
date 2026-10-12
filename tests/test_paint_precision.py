"""A 15 mm requirement is checked against nozzle geometry and preserves missing paint."""
from pathlib import Path as FilePath
from types import SimpleNamespace as NS
from unittest.mock import patch
import math,sys,unittest

ROOT=FilePath(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT)]
import paint_quality as pq
from paint_start import PaintStartGuard
from path_tracker import Path,PathConfig,PathTracker,Section,differential_config
from robot_core.kinematics import ChassisGeometry
from route_operations import RouteOperations

GEO=ChassisGeometry(drive='differential',track_m=.675,diff_min_radius_m=.5)

def scene(error,internal=.003,trusted=True,speed=.5):
    path=Path((0.,0.,0.),[(10.,0.)]);section=Section(1,path);section.field_role='paint'
    tracker=PathTracker(path,differential_config(PathConfig(cruise_mps=speed)),GEO)
    tracker.st.s=5.;tracker.st.state='RUNNING'
    tracker.straight_guide_estimate=lambda:internal
    pose=NS(x=5.,y=error,yaw=0.,speed=.5,yaw_rate=0.,usable_for_control=True,
            rtk_trusted=trusted,hold_age_s=.01,sigma_xy=.0085)
    mission=NS(differential=True,pose=pose,_route_local=lambda:None)
    record=dict(route_nozzle=dict(forward=.24,right=.4575))
    return mission,record,section,tracker

class PrecisionGeometry(unittest.TestCase):
    def test_internal_small_error_cannot_hide_actual_geometric_offset(self):
        m,r,s,t=scene(.016,internal=.003)
        out=pq.status(m,r,True,s,t)
        self.assertAlmostEqual(out['nozzle_deviation_m'],.016)
        self.assertAlmostEqual(out['control_nozzle_deviation_m'],.003)
        self.assertFalse(out['painting'])
        self.assertEqual(out['paint_blocked'],'paint_precision_exceeded')
        self.assertEqual(out['paint_offset_limit_m'],.015)

    def test_reserve_boundary_is_reported_without_clipping_error(self):
        for error,on in ((.009,True),(.0119,True),(.0121,False),(.03,False)):
            with self.subTest(error=error):
                m,r,s,t=scene(error);out=pq.status(m,r,True,s,t)
                self.assertAlmostEqual(out['nozzle_deviation_m'],error)
                self.assertEqual(out['painting'],on)
                self.assertEqual(out['paint_start_limit_m'],.010)

    def test_a_tighter_existing_limit_is_not_relaxed(self):
        with patch.dict('os.environ',{'RK3588_SPRAY_MAX_OFFSET_M':'.005'}):
            m,r,s,t=scene(-.006);out=pq.status(m,r,True,s,t)
        self.assertFalse(out['painting'])
        self.assertAlmostEqual(out['nozzle_signed_deviation_m'],-.006)
        self.assertEqual(out['paint_stop_limit_m'],.005)
        self.assertEqual(out['paint_start_limit_m'],.005)

    def test_invalid_precision_limit_cannot_open_the_pump(self):
        with patch.dict('os.environ',{'RK3588_SPRAY_MAX_OFFSET_M':'invalid'}):
            m,r,s,t=scene(0.);out=pq.status(m,r,True,s,t)
        self.assertFalse(out['painting'])
        self.assertEqual(out['paint_blocked'],'paint_configuration_invalid')

    def test_strict_mode_does_not_use_inertial_hold_as_precision_confirmation(self):
        m,r,s,t=scene(0.,trusted=False)
        out=pq.status(m,r,True,s,t)
        self.assertFalse(out['painting'])
        self.assertEqual(out['paint_blocked'],'paint_precision_untrusted')

    def test_high_speed_legacy_scope_is_not_silently_changed(self):
        m,r,s,t=scene(.03,internal=.004,speed=.8)
        out=pq.status(m,r,True,s,t)
        self.assertNotIn('precision_mode',out)
        self.assertAlmostEqual(out['nozzle_deviation_m'],.004)

    def test_tighter_head_qualification_is_instance_local(self):
        old=PaintStartGuard();new=PaintStartGuard(precision=True)
        self.assertEqual(old.OFFSET_M,.02);self.assertEqual(new.OFFSET_M,.008)
        for i in range(10):
            self.assertFalse(new.update(.05,trusted=True,frame=i,weight=1.,rtk_error=.011,
                guide_error=.011,heading_error=0.))
        for i in range(10,20):new.update(.05,trusted=True,frame=i,weight=1.,rtk_error=.007,
                guide_error=.007,heading_error=math.radians(.2))
        self.assertTrue(new.ready)

class PrecisionProgress(unittest.TestCase):
    def test_braking_after_precision_stop_cannot_complete_unpainted_distance(self):
        ops=RouteOperations.__new__(RouteOperations)
        ops.record=dict(id='job',state='RUNNING',cursor_m=1.2,total_m=10.,last_painted_cursor_m=1.1)
        ops.parent=None;ops.runtime_base=0.;ops.runtime_entry=0.;ops.last_saved=0.
        ops.save=lambda:None
        tracker=NS(st=NS(s=1.2),pause=lambda reason:None)
        ops.m=NS(task='route',phase='RUNNING',route_rev=1,tracker=tracker,message='',_close_log=lambda:None)
        ops.pause_precision(.1,dict(paint_blocked='paint_precision_exceeded',nozzle_deviation_m=.013,
                                  paint_offset_limit_m=.015))
        self.assertEqual(ops.record['state'],'PAUSING')
        self.assertEqual(ops.record['reason'],'precision_hold')
        self.assertAlmostEqual(ops.record['cursor_m'],1.1)
        tracker.st.s=1.5;ops.capture(.2,force=True)
        self.assertAlmostEqual(ops.record['cursor_m'],1.1)
        self.assertAlmostEqual(ops.record['precision_pause']['observed_cursor_m'],1.2)
        self.assertNotEqual(ops.record['state'],'DONE')

if __name__=='__main__':unittest.main()
