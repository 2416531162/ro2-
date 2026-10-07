"""固定解不等于与惯导一致：不一致期间隔离，连续恢复和渐入均须有证据。"""
from pathlib import Path
import sys,math,unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT)]
from position_agreement import PositionAgreement
from line_mission import LineMission


def step(g,**kw):
    args=dict(quality_ok=True,position_error=.002,heading_error=0.)
    args.update(kw);return g.update(.02,**args)


class Agreement(unittest.TestCase):
    def test_disagreement_blocks_before_any_correction_and_time_alone_cannot_rejoin(self):
        g=PositionAgreement();self.assertEqual(step(g),1.)
        self.assertEqual(step(g,position_error=.03),0.)
        for _ in range(300):self.assertEqual(step(g,position_error=.03),0.)
        self.assertTrue(g.holding);self.assertGreater(g.age_s,6.)
        self.assertEqual(g.disagreements,1)

    def test_quality_gap_does_not_erase_disagreement_or_resume_on_fixed_label(self):
        g=PositionAgreement()
        for _ in range(15):step(g,quality_ok=False)
        self.assertEqual(step(g,position_error=.05),0.)
        self.assertTrue(g.holding);self.assertEqual(g.disagreements,1)

    def test_recovery_requires_continuous_agreement_then_smoothly_blends(self):
        g=PositionAgreement();step(g,position_error=.04)
        for _ in range(40):step(g)
        step(g,quality_ok=False)
        for _ in range(49):self.assertEqual(step(g),0.);self.assertTrue(g.holding)
        self.assertEqual(step(g),0.);self.assertFalse(g.holding)
        weights=[step(g) for _ in range(50)]
        self.assertAlmostEqual(weights[0],.02);self.assertEqual(weights[-1],1.)
        self.assertTrue(all(0<=b-a<=.020001 for a,b in zip(weights,weights[1:])))

    def test_heading_disagreement_and_nonfinite_observation_also_block(self):
        for values in (dict(heading_error=math.radians(1.1)),dict(position_error=math.nan)):
            g=PositionAgreement();self.assertEqual(step(g,**values),0.);self.assertTrue(g.holding)

    def test_new_disagreement_during_rejoin_cuts_correction_immediately(self):
        g=PositionAgreement();step(g,position_error=.04)
        for _ in range(60):step(g)
        self.assertGreater(g.weight,0.);self.assertLess(g.weight,1.)
        self.assertEqual(step(g,position_error=.04),0.)
        self.assertEqual(g.disagreements,2)

    def test_fixed_pose_alone_cannot_auto_resume_a_quarantined_reference(self):
        m=LineMission.__new__(LineMission)
        m.operations=NS(held=False);m._usable_since=0.;m._resume_failed=None
        m.tracker=NS(st=NS(state='PAUSED',reason='arc_position_disagreement'),
                     position_reference_ready=lambda:False,start=Mock())
        m._auto_resume(10.,NS(usable_for_control=True))
        m.tracker.start.assert_not_called()
        self.assertIsNone(m._usable_since)


if __name__=='__main__':unittest.main()
