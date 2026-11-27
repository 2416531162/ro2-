"""d1502469 线头：1.667 m 路径不变，可信参考渐入，开喷不撤掉纠偏。"""
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT),str(ROOT/'tests')]
from path_tracker import PathConfig,SectionTracker,differential_config
from test_continuous_field_arcs import sections,GEO

DT=.02

def tracker():
    plan=json.loads((ROOT/'tests/fixtures/line_head_d1502469.json').read_text())
    route=SectionTracker(sections(plan),differential_config(PathConfig(cruise_mps=.8)),GEO,
                         constant_straight_speed=True,constant_paint_speed=True)
    tr=route.trackers[-1];tr.set_nozzle(0.,0.);tr.st.state='RUNNING'
    return tr,route

def tick(tr,t,s,e=.015,trusted=True,sigma=.01):
    x,y,yaw=tr.path.pose(s)
    p=NS(x=x-e*math.sin(yaw),y=y+e*math.cos(yaw),yaw=yaw,speed=.5,yaw_rate=0.,
         usable_for_control=True,quality='RTK_FIXED' if trusted else 'RTK_FLOAT',
         rtk_trusted=trusted,reasons=(),correction_xy=(0.,0.),correction_yaw=0.)
    tr.sync(t,(t,yaw,0.))
    tr.set_heading_baseline(t,.41);tr.set_rtk_quality(t,sigma,28,.005,True)
    tr.st.s=s;tr.st.e=e;tr.st.heading_err=0.
    tr._straight_first(p,DT,.5,0.,tr.path._piece(s),yaw,0.)
    return tr._straight_status

class LineHeadContinuity(unittest.TestCase):
    def test_trusted_reference_blends_in_after_this_lines_own_samples(self):
        # 换算只从这条线开头的融合车头学（11-27 删跨线记忆：原来有上一条线带来的，攒 10 个就提前渐入）：
        # 攒够 lock_samples 才锁住，之后按 straight_blend_s 渐入；线头位置不变，还没到线头不许开喷
        tr,route=tracker()
        paint_from=tr.paint_from_m
        runup=paint_from-tr.path._piece(paint_from-.01).s0
        self.assertAlmostEqual(runup,5/3)
        n=tr.cfg.straight_imu_first.lock_samples
        for i in range(1,n):tick(tr,i*DT,paint_from-1.4+.01*i/5)
        self.assertFalse(tr._straight_guide.locked)
        self.assertEqual(tr._straight_weight,0.)
        tick(tr,n*DT,paint_from-1.4+.01*n/5)
        self.assertTrue(tr._straight_guide.locked)
        self.assertEqual(tr._straight_guide.source,'learned')
        self.assertEqual(tr._straight_guide.offset_n,n)
        tick(tr,(n+1)*DT,paint_from-1.4+.01*(n+1)/5)
        self.assertGreater(tr._straight_weight,0.)
        self.assertLessEqual(tr._straight_weight,2*DT/.8+1e-9)
        self.assertAlmostEqual(tr.cfg.straight_blend_s,.8)
        self.assertEqual(tr.paint_from_m,paint_from)
        self.assertFalse(tr.paint_start_allowed())

    def test_no_shortcut_without_this_lines_samples(self):
        # 不借别的线：可信、不可信、σ 大都一样，10 个样本不锁、不渐入
        for case in (dict(),dict(trusted=False),dict(sigma=.05)):
            with self.subTest(case=case):
                tr,_=tracker()
                for i in range(1,11):tick(tr,i*DT,tr.paint_from_m-1.+.01*i,**case)
                self.assertFalse(tr._straight_guide.locked)
                self.assertEqual(tr._straight_weight,0.)

    def test_no_jump_in_correction_when_crossing_line_head(self):
        tr,_=tracker()
        for i in range(1,101):tick(tr,i*DT,tr.paint_from_m-.1)
        before=tick(tr,2.02,tr.paint_from_m-.005)
        after=tick(tr,2.04,tr.paint_from_m+.005)
        self.assertLess(abs(after['bias_deg']-before['bias_deg']),.05)
        self.assertLess(abs(after['pull_k_e']-before['pull_k_e']),.01)
        self.assertLess(abs(after['outer_e']-before['outer_e']),.001)
        # 过完短交接后恢复既有线头策略，不把强纠偏带到整条直线。
        later=tick(tr,2.06,tr.paint_from_m+.55)
        self.assertAlmostEqual(later['pull_k_e'],tr.cfg.straight_imu_first.head_k_e)
        self.assertEqual(later['entry_blend_weight'],0.)


if __name__=='__main__':unittest.main()
