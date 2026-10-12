"""Low-speed straight-line position tracking and bounded convergence on grass."""
import copy
from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT)]
from line_cascade import ImuFirstConfig,ImuFirstGuide,imu_first_target,wrap
from path_tracker import Path,PathConfig,PathTracker,differential_config
from robot_core.kinematics import ChassisGeometry

CFG=ImuFirstConfig(rtk_trim=False,rtk_fit=True,grass_lateral=True)
GEO=ChassisGeometry(drive='differential',track_m=.675,diff_min_radius_m=.5)

def warm():
    g=ImuFirstGuide(CFG);g.offset=0.;g.locked=True
    for _ in range(350):g.update(.015,0.,.01,.02,(0.,0.),True)
    assert g._position_e is not None
    return g

class GrassPosition(unittest.TestCase):
    def test_trusted_small_crossing_is_followed_without_wrong_side_tail(self):
        a=ImuFirstGuide(replace(CFG,grass_lateral=False));b=ImuFirstGuide(CFG)
        wrong=[0.,0.];errors=[[],[]]
        for i in range(4000):
            s=i*.01;measured=.009*math.sin(2*math.pi*s/18.)
            for k,g in enumerate((a,b)):
                g.update(measured,0.,.01,.02,(0.,0.),True)
                if s>10:
                    errors[k].append(abs(g.e-measured))
                    if abs(measured)>.003 and measured*g.e<0:wrong[k]+=.02
        self.assertLess(sum(errors[1]),.55*sum(errors[0]))
        self.assertLess(wrong[1],wrong[0])

    def test_reference_angle_model_is_identical_on_fixed_inputs(self):
        a=ImuFirstGuide(replace(CFG,grass_lateral=False));b=ImuFirstGuide(CFG)
        for i in range(2000):
            measured=.01*math.sin(i*.01/4.)
            for g in (a,b):g.update(measured,.001,.01,.02,(.24,-.4575),True,roll=.01)
            self.assertAlmostEqual(a.beta,b.beta,places=14)
            self.assertAlmostEqual(a.crab,b.crab,places=14)

    def test_invalid_position_cannot_steer_or_accumulate_confirmation(self):
        a=warm();b=copy.deepcopy(a)
        for _ in range(150):
            a.update(.5,0.,.01,.02,(0.,0.),False)
            b.update(-.5,0.,.01,.02,(0.,0.),False)
            self.assertAlmostEqual(a.e,b.e,places=12)
            self.assertEqual(a.convergence_error(),a.e)
            self.assertEqual(a._grass_trusted_m,0.)

    def test_short_fixed_solution_jump_is_still_rejected(self):
        g=warm();worst=0.
        for i in range(700):
            measured=.065 if 100<i<150 else .015
            g.update(measured,0.,.01,.02,(0.,0.),True)
            worst=max(worst,abs(g.e-.015))
        self.assertGreater(g.jumps,0)
        self.assertLess(worst,.003)

    def test_source_switch_waits_for_a_valid_anchor_and_does_not_integrate_bad_position(self):
        g=warm();g.precision_position_target()
        g.source_changed()
        self.assertTrue(g._grass_pi_anchor_pending)
        before=g._grass_pi_integral
        for _ in range(20):
            g.update(.5,0.,.01,.02,(0.,0.),False)
            g.precision_position_target()
        self.assertEqual(g._grass_pi_integral,before)

    def test_standstill_has_no_position_integral_windup(self):
        g=ImuFirstGuide(CFG)
        for _ in range(1000):
            g.update(.015,0.,0.,.02,(0.,0.),True)
            g.precision_position_target()
        self.assertEqual(g._grass_pi_integral,0.)
        self.assertEqual(g.convergence_error(),g.e)

    def test_position_integral_is_bounded_and_freezes_on_quality_loss(self):
        g=warm()
        for i in range(1500):
            g.update(.015+.001*i*.01,0.,.01,.02,(0.,0.),True)
            g.precision_position_target()
            self.assertLessEqual(abs(g._grass_pi_integral),math.tan(math.radians(CFG.grass_pi_max_deg))/CFG.grass_pi_k_i+1e-12)
        integral=g._grass_pi_integral
        g.update(.3,0.,.01,.02,(0.,0.),False)
        g.precision_position_target()
        self.assertAlmostEqual(g._grass_pi_integral,integral,places=12)

    def test_transit_and_reverse_are_not_enabled_as_paint_policy(self):
        from path_tracker import Section,SectionTracker
        cfg=differential_config(PathConfig(cruise_mps=.5))
        for direction,role in ((-1,'paint'),(1,'transit')):
            section=Section(direction,Path((0.,0.,0.),[(20.,0.)]))
            section.field_role=role
            tr=SectionTracker([section],cfg,GEO)
            self.assertFalse(tr.trackers[0]._straight_guide.cfg.grass_lateral)
            self.assertFalse(tr.trackers[0]._straight_core.cfg.adaptive_response)

    def test_target_is_rate_bounded_and_each_measurement_is_integrated_once(self):
        g=warm();g.fast=False
        target=0.;prev_rate=0.
        for i in range(600):
            measured=.015-.00005*i
            g.update(measured,math.asin(-.005),.01,.02,(0.,0.),True)
            before=target
            target,_,_,_=imu_first_target(CFG,g,target,0.,.02,3.,1.,True)
            rate=wrap(target-before)/.02
            self.assertLessEqual(abs(rate),math.radians(CFG.grass_pi_target_rate_deg_s)+1e-9)
            integral=g._grass_pi_integral
            g.precision_position_target()
            self.assertEqual(g._grass_pi_integral,integral)
            prev_rate=rate

    def test_only_low_speed_straight_guide_uses_new_policy(self):
        for speed,enabled in ((.5,True),(.8,False)):
            tr=PathTracker(Path((0.,0.,0.),[(20.,0.)]),differential_config(PathConfig(cruise_mps=speed)),GEO)
            self.assertEqual(tr._straight_guide.cfg.grass_lateral,enabled)
            self.assertEqual(tr._straight_core.cfg.direct_rate_damping,enabled)
            self.assertEqual(tr._straight_core.cfg.adaptive_response,enabled)
            self.assertEqual(tr._straight_core.cfg.ki,0. if enabled else tr.cfg.straight_ki)
            self.assertFalse(tr._arc_guide.cfg.grass_lateral)
        tr.set_cruise(.5);tr._reset_straight()
        self.assertTrue(tr._straight_guide.cfg.grass_lateral)
        self.assertTrue(tr._straight_core.cfg.direct_rate_damping)
        tr.set_cruise(.8);tr._reset_straight()
        self.assertFalse(tr._straight_guide.cfg.grass_lateral)
        self.assertFalse(tr._straight_core.cfg.direct_rate_damping)

    def test_slewed_reference_cannot_coast_past_a_reversed_correction(self):
        g=ImuFirstGuide(CFG);g.offset=0.;g.locked=True;g.fast=False
        target=0.
        for measured in [.02]*150+[-.005]*150:
            g.update(measured,0.,.01,.02,(0.,0.),True)
            previous=target
            target,bias,_,_=imu_first_target(CFG,g,target,0.,.02,3.,1.,True)
            gap=wrap(bias-previous);step=wrap(target-previous)
            self.assertGreaterEqual(step*gap,-1e-14)
            self.assertLessEqual(abs(step),abs(gap)+1e-12)

    def test_invalid_new_configuration_is_rejected(self):
        for field in ('grass_position_confirm_m','grass_position_tau_m','grass_trend_tau_m',
                      'grass_convergence_m','grass_convergence_limit_m','grass_pull_k_e','grass_max_bias_deg',
                      'grass_motion_tau_m','grass_motion_limit_deg','grass_motion_min_error_m','grass_motion_confirm_m'):
            with self.subTest(field=field),self.assertRaises(ValueError):replace(CFG,**{field:0.})
        with self.assertRaises(ValueError):replace(CFG,grass_lateral=1)

if __name__=='__main__':unittest.main()
