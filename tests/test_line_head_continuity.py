"""d1502469 线头：1.667 m 路径不变，可信参考提前渐入，开喷不撤掉纠偏。"""
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT),str(ROOT/'tests')]
from line_cascade import ImuAlignment
from path_tracker import PathConfig,SectionTracker,differential_config
from test_continuous_field_arcs import sections,GEO

DT=.02

def tracker(history=True,sigma=.1,gap=0.):
    plan=json.loads((ROOT/'tests/fixtures/line_head_d1502469.json').read_text())
    route=SectionTracker(sections(plan),differential_config(PathConfig(cruise_mps=.8)),GEO,
                         constant_straight_speed=True,constant_paint_speed=True)
    tr=route.trackers[-1];tr.set_nozzle(0.,0.);tr.st.state='RUNNING'
    a=ImuAlignment();yaw=tr.path.pose(tr.paint_from_m)[2]
    a.feed_imu(0.,yaw)
    if history:a.obs=(0.,a.theta,math.radians(gap),math.radians(sigma))
    tr.share_imu_alignment(a)
    return tr,route,a

def tick(tr,a,t,s,e=.015,trusted=True,sigma=.01):
    x,y,yaw=tr.path.pose(s)
    p=NS(x=x-e*math.sin(yaw),y=y+e*math.cos(yaw),yaw=yaw,speed=.5,yaw_rate=0.,
         usable_for_control=True,quality='RTK_FIXED' if trusted else 'RTK_FLOAT',
         rtk_trusted=trusted,reasons=(),correction_xy=(0.,0.),correction_yaw=0.)
    a.feed_imu(t,yaw);tr.sync(t,(t,yaw,0.))
    tr.set_heading_baseline(t,.41);tr.set_rtk_quality(t,sigma,28,.005,True)
    tr.st.s=s;tr.st.e=e;tr.st.heading_err=0.
    tr._straight_first(p,DT,.5,0.,tr.path._piece(s),yaw,0.)
    return tr._straight_status

class LineHeadContinuity(unittest.TestCase):
    def test_trusted_reference_uses_existing_blend_without_waiting_full_second(self):
        tr,route,a=tracker()
        paint_from=tr.paint_from_m
        runup=paint_from-tr.path._piece(paint_from-.01).s0
        self.assertAlmostEqual(runup,5/3)
        for i in range(1,11):tick(tr,a,i*DT,paint_from-1.2+.01*i)
        self.assertTrue(tr._straight_guide.locked)
        self.assertEqual(tr._straight_guide.source,'partial')
        self.assertEqual(tr._straight_guide.offset_n,10)
        self.assertGreater(tr._straight_weight,0.)
        self.assertLessEqual(tr._straight_weight,DT/.8+1e-9)
        self.assertAlmostEqual(tr.cfg.straight_blend_s,.8)
        self.assertEqual(tr.paint_from_m,paint_from)
        self.assertFalse(tr.paint_start_allowed())

    def test_unconfirmed_or_uncertain_history_cannot_shortcut_learning(self):
        cases=[dict(history=False),dict(sigma=.5),dict(gap=2.),dict(trusted=False),dict(measure_sigma=.05)]
        for case in cases:
            with self.subTest(case=case):
                args=dict(case);trusted=args.pop('trusted',True)
                measure_sigma=args.pop('measure_sigma',.01)
                tr,_,a=tracker(**args)
                for i in range(1,11):tick(tr,a,i*DT,tr.paint_from_m-1.+.01*i,trusted=trusted,sigma=measure_sigma)
                self.assertFalse(tr._straight_guide.locked)
                self.assertEqual(tr._straight_weight,0.)

    def test_no_jump_in_correction_when_crossing_line_head(self):
        tr,_,a=tracker(history=False)
        for i in range(1,101):tick(tr,a,i*DT,tr.paint_from_m-.1)
        before=tick(tr,a,2.02,tr.paint_from_m-.005)
        after=tick(tr,a,2.04,tr.paint_from_m+.005)
        self.assertLess(abs(after['bias_deg']-before['bias_deg']),.05)
        self.assertLess(abs(after['pull_k_e']-before['pull_k_e']),.01)
        self.assertLess(abs(after['outer_e']-before['outer_e']),.001)
        # 过完短交接后恢复既有线头策略，不把强纠偏带到整条直线。
        later=tick(tr,a,2.06,tr.paint_from_m+.55)
        self.assertAlmostEqual(later['pull_k_e'],tr.cfg.straight_imu_first.head_k_e)
        self.assertEqual(later['entry_blend_weight'],0.)

    def test_imu_discontinuity_invalidates_history(self):
        tr,_,a=tracker()
        a.feed_imu(a.cfg.imu_gap_s+1.,tr.path.pose(tr.paint_from_m)[2])
        for i in range(1,11):tick(tr,a,a.cfg.imu_gap_s+1.+i*DT,tr.paint_from_m-1.)
        self.assertFalse(tr._straight_guide.locked)


if __name__=='__main__':unittest.main()
