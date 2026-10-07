"""Offline arc speed, spray qualification and bounded IMU continuation checks."""
import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'radar_system'), str(ROOT)]
import paint_quality
import circle_spray_repair
from path_tracker import Path as Route, PathConfig, PathTracker, Section, SectionTracker, differential_config
from line_mission import LineMission
from route_operations import RouteOperations
from robot_core.kinematics import ChassisGeometry
from motion_speeds import PAINT_ARC_SPEED_MPS

GEO = ChassisGeometry(drive='differential', track_m=.675, diff_min_radius_m=.5)


def pose(x=0., y=0., yaw=0., *, trusted=True, age=.1, usable=True):
    return NS(x=x, y=y, yaw=yaw, speed=PAINT_ARC_SPEED_MPS, yaw_rate=-.25, usable_for_control=usable,
              quality='RTK_FIXED' if trusted else 'DEAD_RECKONING', rtk_trusted=trusted,
              hold_age_s=age, reasons=() if trusted else ('rtk_hold',), correction_xy=(0.,0.))


def tracker(fixed=True):
    cfg=differential_config(PathConfig(cruise_mps=.8))
    if not fixed:cfg=replace(cfg,arc_speed_mps=None)
    tr=PathTracker(Route((0.,0.,0.),[(20.,-.5)]),cfg,GEO)
    tr.sync(7.,(7.,0.,-.25));tr.start(pose())
    return tr


def mission(p, tr):
    return NS(pose=p,differential=True,nozzle=(0.,0.),
              fusion=NS(config=NS(rtk_hold_s=5.)),
              _route_local=lambda:NS(kind='circle',center=(0.,-2.),radius=2.))


class ConstantArcPolicy(unittest.TestCase):
    def test_arc_stays_at_paint_speed_despite_offset_and_soft_hold_limit(self):
        for curvature in (-.5,.5,-1.2):
            with self.subTest(curvature=curvature):
                cfg=differential_config(PathConfig(cruise_mps=.8))
                path=Route((0.,0.,0.),[(20.,curvature)])
                tr=PathTracker(path,cfg,GEO)
                tr.start(pose());tr._v_prev=PAINT_ARC_SPEED_MPS
                tr.paint_offset_limit_m=.03;tr.speed_limit_mps=.1
                for i in range(1,51):
                    x,y,h=path.pose(2.+i*.01)
                    # 12 cm lateral error previously caused a speed reduction.
                    p=pose(x-.12*math.sin(h),y+.12*math.cos(h),h,trusted=False,age=2.)
                    v,_=tr.update(p,.02)
                    self.assertAlmostEqual(v,PAINT_ARC_SPEED_MPS,places=12)
                    self.assertEqual(tr.st.state,'RUNNING')

    def test_zero_limit_and_unusable_pose_still_stop(self):
        tr=tracker();tr.speed_limit_mps=0.
        self.assertEqual(tr.update(pose(),.02),(0.,0.))
        tr=tracker()
        self.assertEqual(tr.update(pose(usable=False,trusted=False,age=5.1),.02),(0.,0.))
        self.assertEqual(tr.st.state,'PAUSED')

    def test_mission_ignores_soft_degraded_arc_speed_cap(self):
        tr=SectionTracker([Section(1,Route((0.,0.,0.),[(20.,-.5)]))],differential_config(PathConfig()),GEO)
        m=LineMission.__new__(LineMission)
        m.differential=True;m.task='route';m.tracker=tr;m._hold_speed_limit=.1
        from rtk_hold_budget import DegradedSpeedLimiter
        m._speed_hold=DegradedSpeedLimiter()
        self.assertIsNone(m._stable_speed_limit(7.,pose(trusted=False,age=2.),.02))
        self.assertEqual(m._stable_speed_limit(7.02,pose(usable=False,trusted=False,age=5.1),.02),0.)

    def test_arc_continues_spray_at_six_cm_error(self):
        tr=tracker();p=pose(y=.06)
        sec=Section(1,tr.path)
        q=paint_quality.status(mission(p,tr),{},True,sec,tr)
        self.assertTrue(q['painting']);self.assertIsNone(q['paint_blocked'])
        self.assertAlmostEqual(q['nozzle_deviation_m'],.06)
        old=tracker(fixed=False)
        q=paint_quality.status(mission(p,old),{},True,Section(1,old.path),old)
        self.assertTrue(q['painting']);self.assertEqual(q['paint_warning'],'paint_off_line')

    def test_imu_keeps_arc_spraying_when_rtk_is_not_trusted(self):
        tr=tracker();p=pose(y=.06,trusted=False,age=4.9)
        q=paint_quality.status(mission(p,tr),{},True,Section(1,tr.path),tr)
        self.assertTrue(q['painting']);self.assertEqual(q['paint_control_source'],'imu_hold')
        p.hold_age_s=5.01
        self.assertFalse(paint_quality.status(mission(p,tr),{},True,Section(1,tr.path),tr)['painting'])
        p.hold_age_s=2.;tr.sync(7.21,None)
        self.assertFalse(paint_quality.status(mission(p,tr),{},True,Section(1,tr.path),tr)['painting'])

    def test_transit_and_stationary_are_not_automatic_spray(self):
        tr=tracker();p=pose()
        self.assertFalse(paint_quality.status(mission(p,tr),{},False,Section(1,tr.path),tr)['painting'])
        p.speed=0.
        self.assertFalse(paint_quality.status(mission(p,tr),{},True,Section(1,tr.path),tr)['painting'])

    def test_covered_repair_gate_does_not_close_arc_spray(self):
        tr=SectionTracker([Section(1,Route((0.,0.,0.),[(20.,-.5)]))],differential_config(PathConfig()),GEO)
        tr.sections[0].field_role='paint';tr.st.state=tr.trackers[0].st.state='RUNNING'
        m=NS(tracker=tr,phase='RUNNING',differential=True)
        ops=RouteOperations.__new__(RouteOperations);ops.m=m;ops.record={'id':'arc'}
        ops.runtime_base=ops.runtime_entry=0.
        ops._paint_window=lambda:(0,0.,20.)
        ops._paint_key=lambda:'key';ops._paint_tail_key=None;ops._paint_tail_closed=False
        ops._paint_tail_section=lambda:None
        qualification=dict(paint_phase=True,painting=True,paint_blocked=None,nozzle_deviation_m=.04,
                           paint_offset_limit_m=.03)
        with patch.object(paint_quality,'status',return_value=qualification.copy()), patch.object(circle_spray_repair,'allowed',return_value=False) as allowed:
            self.assertTrue(ops.paint_status()['painting']);allowed.assert_not_called()

    def test_continuous_circle_does_not_append_repair_lap(self):
        tr=SectionTracker([Section(1,Route((0.,0.,0.),[(20.,-.5)]))],differential_config(PathConfig()),GEO)
        tr.st.state='RUNNING'
        m=NS(tracker=tr,phase='RUNNING',task='route',differential=True,
             path_cfg=tr.cfg,route=NS(kind='circle'))
        ops=RouteOperations.__new__(RouteOperations);ops.m=m
        ops.record={'cursor_m':0.,'total_m':20.};ops.runtime_base=ops.runtime_entry=0.
        ops._gap_previous=None;ops.last_saved=100.
        ops.paint_status=lambda:dict(paint_phase=True,painting=True,paint_blocked=None)
        ops._append_spray_repair=Mock()
        with patch.object(circle_spray_repair,'initialise') as init, patch.object(circle_spray_repair,'step') as step, patch('paint_gaps.observe',return_value=None):
            ops.capture(7.)
            init.assert_not_called();step.assert_not_called();ops._append_spray_repair.assert_not_called()

    def test_reverse_arc_keeps_previous_speed_policy(self):
        tr=SectionTracker([Section(-1,Route((0.,0.,0.),[(20.,-.5)]))],differential_config(PathConfig()),GEO)
        self.assertIsNone(tr.trackers[0].cfg.arc_speed_mps)
        self.assertFalse(tr.constant_arc_speed_here())


if __name__=='__main__':unittest.main()
