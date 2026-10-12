"""Passive response identification must earn its correction and remain bounded."""
from pathlib import Path
from collections import deque
import math,sys,unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT)]
from turn_response import TurnResponse
from heading_control import HeadingControl,HeadingControlConfig

def exercise(gain=.8,after=None,noise=0.,eligible=True,flat=False,duration=80.):
    model=TurnResponse();queue=deque([(-2.,0.)]);rate=0.;trace=[]
    for i in range(int(duration/.02)):
        t=i*.02
        command=.025 if flat else .035*math.sin(t*.95)+.023*math.sin(t*2.1+.2)
        queue.append((t,command))
        while len(queue)>1 and queue[1][0]<=t-.3:queue.popleft()
        g=after if after is not None and t>=40 else gain
        rate+=(1-math.exp(-.02/.12))*(g*queue[0][1]-rate)
        measured=rate+noise*math.sin(t*7.3)+.001
        model.observe(t,t,measured,eligible=eligible)
        model.advance(t,.02)
        model.record_sent(t,command)
        trace.append(model.scale)
    return model,trace

class AdaptiveResponse(unittest.TestCase):
    def test_confident_response_scales_in_the_correct_direction(self):
        for gain in (.8,1.2):
            with self.subTest(gain=gain):
                m,trace=exercise(gain)
                self.assertGreater(m.accepted_windows,0,m.status())
                self.assertLess(abs(m.fit_gain-gain),.08)
                self.assertGreater(m.scale,1.1) if gain<1 else self.assertLess(m.scale,.9)
                self.assertTrue(all(.85<=v<=1.15 for v in trace))
                self.assertLessEqual(max(abs(b-a) for a,b in zip(trace,trace[1:])),.01*.02+1e-12)

    def test_normal_motion_can_adapt_after_a_surface_change(self):
        m,trace=exercise(.8,after=1.2,duration=100.)
        self.assertGreater(trace[1900],1.05)
        self.assertLess(m.scale,.95,m.status())
        self.assertGreater(m.response_changes,0)
        self.assertLess(abs(trace[int(47/.02)]-1.),.01)
        self.assertLessEqual(max(abs(b-a) for a,b in zip(trace,trace[1:])),.03*.02+1e-12)

    def test_unobservable_straight_motion_and_bad_quality_do_not_learn(self):
        for kw in (dict(flat=True),dict(eligible=False),dict(noise=.10),dict(gain=0.),dict(gain=-1.)):
            with self.subTest(kw=kw):
                m,trace=exercise(**kw)
                self.assertEqual(m.accepted_windows,0,m.status())
                self.assertEqual(trace,[1.]*len(trace))

    def test_repeated_imu_frames_never_create_evidence(self):
        m=TurnResponse()
        for i in range(500):
            t=i*.02;m.record_sent(t,.03)
            m.observe(t,0.,.024,eligible=True);m.advance(t,.02)
        self.assertEqual(m.scale,1.)
        self.assertEqual(m.accepted_windows,0)

    def test_old_confidence_decays_smoothly_after_sensor_loss(self):
        m,_=exercise()
        self.assertGreater(m.scale,1.)
        previous=m.scale
        for i in range(4000):
            t=80.+i*.02;m.observe(t,t,0.,eligible=False);m.advance(t,.02)
            self.assertLessEqual(m.scale,previous+1e-12)
            self.assertLessEqual(abs(m.scale-previous),.0002+1e-12)
            previous=m.scale
        self.assertAlmostEqual(m.scale,1.)

    def test_history_contains_applied_not_provisional_commands(self):
        core=HeadingControl(HeadingControlConfig(adaptive_response=True))
        core.step(.02,.01,0.,dt=.02,record=False,adaptation_ok=True)
        self.assertFalse(core.response.sent)
        core.record_sent(.02,-.004)
        self.assertEqual(list(core.response.sent),[(.02,-.004)])
        core.record_sent(.02,-.003)
        self.assertEqual(list(core.response.sent),[(.02,-.003)])
        with self.assertRaises(ValueError):core.record_sent(.01,0.)

    def test_reset_removes_learned_surface_and_disable_preserves_old_output(self):
        a=HeadingControl();b=HeadingControl(HeadingControlConfig(adaptive_response=False))
        for i in range(100):
            args=(i*.02,.01*math.sin(i*.1),.005)
            self.assertEqual(a.step(*args,dt=.02),b.step(*args,dt=.02,adaptation_ok=True))
        a=HeadingControl(HeadingControlConfig(adaptive_response=True))
        a.response.scale=1.1;a.reset()
        self.assertEqual(a.response.scale,1.)
        with self.assertRaises(ValueError):HeadingControlConfig(adaptive_response=1)

    def test_scaled_output_saturation_cannot_wind_up_the_integral(self):
        core=HeadingControl(HeadingControlConfig(adaptive_response=True))
        core.response.scale=core.response.target_scale=1.15
        core.response.last_accepted=0.
        command=core.step(.02,.15,0.,dt=.02,moving=True,adaptation_ok=False)
        self.assertEqual(command,-core.cfg.max_wz)
        self.assertEqual(core.integral,0.)

    def test_zero_prediction_horizon_still_records_applied_commands(self):
        core=HeadingControl(HeadingControlConfig(delay_s=0.,adaptive_response=True))
        command=core.step(.02,.01,0.,dt=.02,adaptation_ok=True)
        self.assertEqual(list(core.response.sent),[(.02,command)])

if __name__=='__main__':unittest.main()
