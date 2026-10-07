"""画圆/画弧线 0.5，普通转场弯道 0.35；身份随续画、分段预瞄保留。"""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
import math,sys,unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT),str(ROOT/'tests')]
from path_tracker import Path as Route,Section,SectionTracker,PathConfig,differential_config
from route_operations import RouteOperations,tail,pivot_entry_to,rework_sections
from live_cruise import label_sections,speed_for
from test_continuous_field_arcs import GEO,drive
import route_plan as rp


def tracker(sections,cruise=.8):
    cfg=replace(differential_config(PathConfig(cruise_mps=cruise)),straight_imu_enabled=False)
    return SectionTracker(sections,cfg,GEO,constant_straight_speed=True,constant_paint_speed=True)


def stroke(sweep):
    sec=Section(1,Route((0.,0.,0.),[(3.,0.),(1.,.5),(2*sweep,.5),(.5,.5)]))
    sec.field_role='paint';sec.paint_from_m=4.;sec.paint_to_m=4.+2*sweep
    return sec


class WorkArcSpeed(unittest.TestCase):
    def test_unpainted_turn_does_not_cap_the_whole_straight_route_at_half_speed(self):
        sec=Section(1,Route((0.,0.,0.),[(8.,0.),(3.,.5),(8.4,0.)]))
        sec.field_role='paint';sec.paint_from_m=0.;sec.paint_to_m=19.
        sec.paint_spans=[(0.,8.),(11.,19.)]
        tr=tracker([sec]).trackers[0]
        self.assertEqual(tr.cfg.cruise_mps,.8)
        self.assertEqual(tr._planned_paint_speed(0.,8.),.8)
        self.assertEqual(tr._planned_paint_speed(11.,19.),.8)
        self.assertEqual(tr._curve_target_at(9.),.35)

    def test_drawn_arc_and_circle_hold_half_speed_for_every_selected_cruise(self):
        for sweep in (math.pi/2,2*math.pi):
            for cruise in (.5,.8,1.):
                with self.subTest(sweep=sweep,cruise=cruise):
                    sec=stroke(sweep);sec.cruise_mps=cruise
                    tr=tracker([sec],cruise);rows=drive(tr,100.)
                    self.assertEqual(tr.st.state,'DONE',tr.status())
                    speeds=[r['v'] for r in rows if r['paint']]
                    self.assertTrue(speeds)
                    self.assertAlmostEqual(min(speeds),.5,places=6)
                    self.assertAlmostEqual(max(speeds),.5,places=6)

    def test_transit_bend_is_distinct_from_paint_arc_within_the_same_section(self):
        sec=Section(1,Route((0.,0.,0.),[(1.,-.5),(3.,0.),(6.,.5),(.5,.5)]))
        sec.field_role='paint';sec.paint_from_m=4.;sec.paint_to_m=10.
        tr=tracker([sec]).trackers[0]
        self.assertEqual(tr._curve_target_at(.5),.35)
        self.assertEqual(tr._curve_target_at(4.5),.5)
        self.assertEqual(tr._curve_target_at(10.1),.5)

    def test_continuation_preview_retains_the_next_sections_role(self):
        for role,want in (('paint',.5),('transit',.35)):
            a=Section(1,Route((0.,0.,0.),[(8.,0.)]));a.field_role='transit';a.continuous_after=True
            b=Section(1,Route(a.car_pose(8.),[(6.,.5),(.5,.5)]))
            b.field_role=role;b.paint_from_m=0.;b.paint_to_m=6.
            tr=tracker([a,b])
            self.assertEqual(tr.trackers[0]._curve_target_at(8.1),want)

    def test_resume_and_saved_plan_keep_work_arc_speed(self):
        sec=stroke(math.pi/2)
        ops=RouteOperations.__new__(RouteOperations)
        ops.m=NS(fusion=NS(to_geodetic=lambda x,y:(x,y),to_local=lambda x,y:(x,y)))
        restored=ops.decode({'plan':ops.encode([sec])})
        remaining=tail(restored,4.5)
        joined,_,_=pivot_entry_to(remaining,remaining[0].car_pose(0.),.557,(0.,0.))
        for plan in (restored,remaining,joined):
            child=tracker(plan).trackers[-1]
            self.assertEqual(child._planned_paint_speed(child.paint_from_m,child.paint_to_m),.5)

    def test_partial_circle_rework_and_online_cruise_preserve_half_speed(self):
        spec=rp.RouteSpec(kind='circle',center=(0.,0.),radius=2.)
        plan,_=rework_sections(spec,0,.5,2.,.557,(.911,.604,.39),nozzle=(.24,-.4575))
        label_sections(plan,0.,1.)
        child=tracker(plan).trackers[0]
        self.assertEqual(child._curve_target_at(.1),.5)
        self.assertEqual(speed_for(plan[0],{},.8),.5)
        self.assertEqual(speed_for(plan[0],{},1.),.5)


if __name__=='__main__':unittest.main()
