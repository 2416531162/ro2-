"""开喷前完成稳定交接；未收稳不越过线头开始喷，恢复接入保留未画线头。"""
from pathlib import Path
from types import SimpleNamespace as NS
import math,sys,unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT)]
from paint_start import PaintStartGuard
import route_plan as rp
import paint_quality
from path_tracker import Path as Route,PathConfig,PathTracker,Section,SectionTracker,differential_config
from robot_core.kinematics import ChassisGeometry
from route_operations import tail,pivot_entry_to

GEO=ChassisGeometry(drive='differential',track_m=.675,diff_min_radius_m=.5)

def observe(guard,t,**changes):
    values=dict(trusted=True,frame=t,weight=1.,rtk_error=.002,guide_error=.002,heading_error=0.)
    values.update(changes);return guard.update(.02,**values)

def tracker():
    tr=PathTracker(Route((0.,0.,0.),[(13.,0.)]),differential_config(PathConfig(cruise_mps=.8)),GEO)
    tr.set_nozzle(0.,0.);tr.paint_from_m=2.5;tr.paint_start_guard=PaintStartGuard()
    tr.st.state='RUNNING';tr._straight_weight=1.;tr._straight_guided=(.002,0.)
    tr._straight_guide.locked=True;tr._straight_guide.rtk_accepted=True
    return tr

def sample(tr,t,s,e=.002):
    tr.st.s=s;tr.sync(t,(t,0.,0.));tr.set_heading_baseline(t,.412);tr.set_rtk_quality(t,.008,32,.005)
    return NS(x=s,y=e,yaw=0.,speed=.3,yaw_rate=0.,usable_for_control=True,
              quality='RTK_FIXED',rtk_trusted=True,reasons=(),correction_xy=(0.,0.))

class Evidence(unittest.TestCase):
    def test_requires_continuous_stability_and_multiple_receiver_frames(self):
        g=PaintStartGuard()
        for i in range(30):observe(g,1.)
        self.assertFalse(g.ready)
        for i in range(30):observe(g,1.+i*.05)
        self.assertTrue(g.ready)
        observe(g,3.,trusted=False);self.assertFalse(g.ready)
        for i in range(10):observe(g,3.+i*.02)
        self.assertFalse(g.ready)

    def test_recorded_bad_start_and_estimate_disagreement_do_not_pass(self):
        for values in [dict(weight=0.,rtk_error=.0215),dict(weight=1.,rtk_error=-.03856,guide_error=-.00957),
                       dict(heading_error=math.radians(.8))]:
            g=PaintStartGuard()
            for i in range(60):observe(g,i*.02,**values)
            self.assertFalse(g.ready)

class Integration(unittest.TestCase):
    def test_stable_entry_opens_at_original_start_not_early(self):
        tr=tracker()
        for i in range(30):
            self.assertFalse(tr._check_paint_start(sample(tr,i*.02,1.8+i*.006),.02,.3))
        self.assertTrue(tr.paint_start_guard.ready)
        self.assertFalse(tr.paint_start_allowed())
        self.assertFalse(tr._check_paint_start(sample(tr,.7,2.501),.02,.3))
        self.assertTrue(tr.paint_start_allowed())
        # 已经开始的喷线不被起步门反复打断；后续保护仍交回原控制。
        self.assertFalse(tr._check_paint_start(sample(tr,.72,2.6,e=.04),.02,.8))

    def test_unready_brakes_with_margin_before_paint_start(self):
        tr=tracker();tr._straight_weight=0.;tr._straight_guided=None
        self.assertFalse(tr._check_paint_start(sample(tr,0.,1.5,e=.02),.02,.3))
        self.assertTrue(tr._check_paint_start(sample(tr,.02,2.2,e=.02),.02,.3))
        self.assertFalse(tr.paint_start_allowed())
        m=NS(pose=sample(tr,.04,2.6),differential=True,nozzle=(0.,0.))
        status=paint_quality.status(m,{},True,Section(1,tr.path),tr)
        self.assertFalse(status['painting']);self.assertEqual(status['paint_blocked'],'paint_start_unready')

    def test_ready_state_does_not_survive_bad_observation_at_the_start(self):
        tr=tracker()
        for i in range(30):tr._check_paint_start(sample(tr,i*.02,1.5),.02,.3)
        self.assertTrue(tr.paint_start_guard.ready)
        self.assertTrue(tr._check_paint_start(sample(tr,.7,2.51,e=.025),.02,.3))
        self.assertFalse(tr.paint_start_allowed())

    def test_pause_after_paint_preserves_start_authorization(self):
        tr=tracker();tr.paint_start_guard.started=True;tr.st.state='PAUSED';tr._hint=4.
        self.assertTrue(tr.start(sample(tr,1.,4.)))
        self.assertTrue(tr.paint_start_allowed())

    def test_short_confirmation_uses_a_stoppable_speed_instead_of_rushing_the_start(self):
        tr=tracker()
        for i in range(18):tr._check_paint_start(sample(tr,i*.02,1.5),.02,.3)
        self.assertFalse(tr.paint_start_guard.ready)
        self.assertFalse(tr._check_paint_start(sample(tr,.38,2.0),.02,.6))
        self.assertIsNotNone(tr._paint_start_speed_limit)
        self.assertLess(tr._paint_start_speed_limit,.6)

    def test_unpainted_start_is_not_lost_if_position_crosses_during_stopping(self):
        from route_operations import RouteOperations
        tr=tracker()
        sec=Section(1,tr.path);sec.field_role='paint';sec.paint_from_m=2.5
        sections=SectionTracker([sec],tr.cfg,GEO,constant_straight_speed=True)
        sections.trackers[0].paint_start_guard=PaintStartGuard()
        sections.st.s=2.65;sections.st.state='ABORTED'
        ops=RouteOperations.__new__(RouteOperations)
        ops.record=dict(cursor_m=2.3,total_m=13.)
        ops.runtime_base=ops.runtime_entry=0.;ops.last_saved=10.
        ops.m=NS(task='route',phase='PAUSING',tracker=sections)
        ops.capture(10.1)
        self.assertEqual(ops.record['cursor_m'],2.5)

    def test_reentry_adds_runup_without_skipping_the_unpainted_start(self):
        sec=Section(1,Route((0.,0.,0.),[(13.,0.)]));sec.field_role='paint';sec.paint_from_m=2.5;sec.paint_to_m=12.5;sec.cruise_mps=.8
        rest=tail([sec],2.1)
        self.assertAlmostEqual(rest[0].path.pose(rest[0].paint_from_m)[0],2.5)
        plan,_,_=pivot_entry_to(rest,(2.1,0.,0.),.556,(0.,0.))
        paint=next(s for s in plan if getattr(s,'field_role',None)=='paint')
        self.assertAlmostEqual(paint.path.pose(paint.paint_from_m)[0],2.5,places=6)
        self.assertGreaterEqual(paint.paint_from_m,rp.RUN_UP_M)

    def test_field_and_normal_route_share_requested_runup(self):
        import field_planner
        self.assertEqual(rp.RUN_UP_M,2.5);self.assertEqual(field_planner.RUN_IN_M,2.5)
        sec=Section(1,Route((0.,0.,0.),[(13.,0.)]));sec.field_role='paint';sec.paint_from_m=2.5
        tr=SectionTracker([sec],differential_config(PathConfig()),GEO,constant_straight_speed=True)
        self.assertEqual(tr.trackers[0].runup_distance_m,2.5)
        self.assertIsNotNone(tr.trackers[0].paint_start_guard)

if __name__=='__main__':unittest.main()
