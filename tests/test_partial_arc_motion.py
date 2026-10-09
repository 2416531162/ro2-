"""Pending-response prediction starts from measured wheel/IMU motion, not command values."""
from collections import deque
from dataclasses import replace
import math
from types import SimpleNamespace as NS
import unittest
from test_partial_arc_observer import tracker


class PartialArcMotion(unittest.TestCase):
    def make(self, v=.5, w=0.):
        tr=tracker();tr._clock=1.
        tr._sent=deque([(0.,v,w)])
        return tr

    def test_constant_measured_motion_matches_exact_circle_geometry(self):
        tr=self.make(w=.2)
        source=NS(x=1.,y=2.,yaw=.4)
        x,y,h=tr._predict_partial_motion(source,NS(speed=.5,yaw_rate=.2))
        horizon=tr.cfg.arc_partial_response_delay_s+tr.cfg.arc_partial_response_tau_s
        self.assertAlmostEqual(h,.4+.2*horizon,places=12)
        self.assertAlmostEqual(x,1.+2.5*(math.sin(h)-math.sin(.4)),delta=1e-6)
        self.assertAlmostEqual(y,2.-2.5*(math.cos(h)-math.cos(.4)),delta=1e-6)

    def test_measured_gyro_motion_is_not_replaced_with_the_last_command(self):
        tr=self.make();source=NS(x=0.,y=0.,yaw=0.)
        calm=tr._predict_partial_motion(source,NS(speed=.5,yaw_rate=0.))
        turning=tr._predict_partial_motion(source,NS(speed=.5,yaw_rate=.2))
        self.assertEqual(calm[2],0.)
        tau=tr.cfg.arc_partial_response_tau_s
        horizon=tr.cfg.arc_partial_response_delay_s+tau
        expected=.2*tau*(1.-math.exp(-horizon/tau))
        self.assertAlmostEqual(turning[2],expected,delta=2e-5)

    def test_measured_wheel_velocity_sets_the_pending_travel(self):
        tr=self.make();source=NS(x=0.,y=0.,yaw=0.)
        fast=tr._predict_partial_motion(source,NS(speed=.5,yaw_rate=0.))
        slow=tr._predict_partial_motion(source,NS(speed=.3,yaw_rate=0.))
        self.assertGreater(fast[0]-slow[0],.015)
        self.assertLess(fast[0]-slow[0],.021)
        self.assertEqual(fast[1:],slow[1:])

    def test_feedback_profile_uses_nozzle_radius_in_both_turn_directions(self):
        for radius,want in ((1.5,3.5),(3.,4.5)):
            for sign in (-1.,1.):
                rear=math.sqrt(radius*radius-.24**2)-sign*.4575
                tr=tracker(k=sign/rear);tr._arc_piece=tr.path.pieces[1]
                self.assertAlmostEqual(tr._partial_arc_feedback_scale(),want,places=10)

    def test_disabled_prediction_and_standstill_stay_at_the_current_pose(self):
        tr=self.make(0.,0.);source=NS(x=1.,y=2.,yaw=.3)
        self.assertEqual(tr._predict_partial_motion(source,NS(speed=0.,yaw_rate=0.)),(1.,2.,.3))
        tr.cfg=replace(tr.cfg,delay_s=0.)
        self.assertEqual(tr._predict_partial_motion(source,NS(speed=.5,yaw_rate=.2)),(1.,2.,.3))

if __name__=='__main__':unittest.main()
