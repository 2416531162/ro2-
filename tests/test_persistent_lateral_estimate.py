"""2a018383 继续作业段：可信卫星横偏 3.25 cm，控制估计只有 1.64 cm。

固定实录观测、IMU 相对角和轮程，回放观察器而非预测实车轨迹。
fixture 来源及量化误差见 docs/PERSISTENT_LATERAL_ESTIMATE.md。
"""
import csv
import copy
import dataclasses
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'radar_system'), str(ROOT)]
from line_cascade import ImuFirstConfig, ImuFirstGuide, PullBoost


def recorded(crab=True, sign=1.):
    with (ROOT/'tests/fixtures/job_2a018383_observer.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    rows = [{k: float(v) for k,v in row.items()} for row in rows]
    cfg = ImuFirstConfig(rtk_trim=False, rtk_fit=True, crab_enabled=crab)
    guide, boost = ImuFirstGuide(cfg), PullBoost(cfg)
    guide.offset, guide.offset_n, guide.locked, guide.source = 0., 10, True, 'partial'
    guide.start = {'weight':.929, 'rate':(sign*math.radians(rows[0]['recorded_rate']), math.radians(cfg.fit_prior_rate_deg))}
    result=[]
    for row in rows:
        measured=sign*row['measured']
        guide.update(measured, sign*row['rel'], row['distance'], row['dt'], (.24,sign*-.4575),
                     bool(row['rtk_ok']), roll=sign*row['roll'], fast=row['s']<0.)
        if row['entry']>0 and guide.rtk_accepted:
            guide.pull_to(measured, 1.-math.exp(-row['distance']*row['entry']/cfg.runup_rtk_tau_m))
        on=boost.update(guide.e,guide.rtk_accepted and guide.mode=='rtk',row['distance'])
        if row['painting']:
            result.append((row['s'],measured,guide.e,on))
    return result


class RecordedTail(unittest.TestCase):
    def test_tail_estimate_follows_trusted_observation_with_and_without_slope_compensation(self):
        for crab in (False,True):
            with self.subTest(crab=crab):
                out=recorded(crab)
                self.assertLess(abs(out[-1][1]-out[-1][2]),.005)
                errors=sorted(abs(r[1]-r[2]) for r in out if r[0]>=20.)
                self.assertLess(errors[int(.95*(len(errors)-1))],.006)
                self.assertLess(next(r[0] for r in out if r[0]>=20. and r[3]),23.)

    def test_opposite_side_has_the_same_recovery(self):
        positive,negative=recorded(sign=1.),recorded(sign=-1.)
        self.assertAlmostEqual(positive[-1][2],-negative[-1][2],places=7)


class RejectedObservations(unittest.TestCase):
    def test_recovery_does_not_relearn_heading_from_position_error(self):
        cfg=ImuFirstConfig(rtk_trim=False,rtk_fit=True,crab_enabled=False)
        following=ImuFirstGuide(cfg)
        model_only=ImuFirstGuide(dataclasses.replace(cfg,position_follow_min_e_m=1.))
        for i in range(2500):
            s=i*.01
            measured=-.02-.001*max(0.,s-10.)
            for g in (following,model_only):
                g.update(measured,0.,.01,.02,(.24,-.4575),True)
            self.assertAlmostEqual(following.beta,model_only.beta,places=12)
            self.assertLessEqual(abs(following.status()['first_position_correction_m']),cfg.fit_rate_m*.01+1e-12)
        self.assertTrue(following.status()['first_position_follow'])
        self.assertFalse(model_only.status()['first_position_follow'])

    def test_after_confirmed_offset_returns_to_line_noise_is_still_smoothed(self):
        for period in (3.,6.):
            g=ImuFirstGuide(ImuFirstConfig(rtk_trim=False,rtk_fit=True,crab_enabled=False))
            previous_y=.02
            quiet=[]
            for i in range(4000):
                s=i*.01
                y=.02*max(0.,1.-max(0.,s-6.)/5.)
                rel=math.asin((y-previous_y)/.01)
                measured=y+(.008*math.sin(2*math.pi*s/period) if s>20. else 0.)
                g.update(measured,rel,.01,.02,(0.,0.),True)
                previous_y=y
                if s>25.:quiet.append(g.e)
            self.assertTrue(g.status()['first_position_follow'])
            self.assertLess(max(map(abs,quiet)),.005)

    def test_bad_satellite_after_activation_does_not_erase_confirmed_position(self):
        g=ImuFirstGuide(ImuFirstConfig(rtk_trim=False,rtk_fit=True))
        for _ in range(500):g.update(-.02,0.,.01,.02,(.24,-.4575),True)
        self.assertTrue(g.status()['first_position_follow'])
        other=copy.deepcopy(g)
        for _ in range(100):
            g.update(.50,0.,.01,.02,(.24,-.4575),False)
            other.update(-.50,0.,.01,.02,(.24,-.4575),False)
            self.assertAlmostEqual(g.e,other.e,places=12)
            self.assertEqual(g.status()['first_position_correction_m'],0.)
        self.assertAlmostEqual(g.e,-.02,places=8)

    def test_standstill_does_not_confirm_deviation(self):
        g=ImuFirstGuide(ImuFirstConfig(rtk_trim=False,rtk_fit=True))
        for _ in range(500):g.update(-.03,0.,0.,.02,(.24,-.4575),True)
        self.assertFalse(g.status()['first_position_follow'])

    def test_oscillation_around_line_does_not_trigger_persistent_recovery(self):
        for period in (3.,6.):
            g=ImuFirstGuide(ImuFirstConfig(rtk_trim=False,rtk_fit=True))
            values=[]
            for i in range(3001):
                s=i*.01
                g.update(.008*math.sin(2*math.pi*s/period),0.,.01,.02,(.24,-.4575),True)
                if s>8.:values.append(g.e)
            self.assertEqual(g.persists,0)
            self.assertLess(max(map(abs,values)),.005)

    def test_untrusted_tail_cannot_trigger_recovery(self):
        g=ImuFirstGuide(ImuFirstConfig(rtk_trim=False,rtk_fit=True))
        for i in range(1200):
            trusted=i<1000
            g.update(0. if trusted else -.03,0.,.01,.02,(.24,-.4575),trusted)
        self.assertEqual(g.persists,0)
        self.assertFalse(g.rtk_accepted)
        self.assertLess(abs(g.e),1e-9)

    def test_short_jump_is_not_persistent_deviation(self):
        g=ImuFirstGuide(ImuFirstConfig(rtk_trim=False,rtk_fit=True))
        worst=0.
        for i in range(1500):
            measured=-.04 if 1000<=i<1050 else 0.
            g.update(measured,0.,.01,.02,(.24,-.4575),True)
            worst=max(worst,abs(g.e))
        self.assertEqual(g.persists,0)
        self.assertGreater(g.jumps,0)
        self.assertLess(worst,.003)


if __name__=='__main__':
    unittest.main()
