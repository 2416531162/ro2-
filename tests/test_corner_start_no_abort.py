"""实录角弧作业：0.833 m 助跑不再被分段长度和起步自动中止夹住。"""
from pathlib import Path
from dataclasses import replace
import json,sys,unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT),str(ROOT/'tests')]
from test_continuous_field_arcs import GEO,DT,sections,pose
from path_tracker import PathConfig,SectionTracker,differential_config

def recorded():
    plan=json.loads((ROOT/'tests/fixtures/corner_start_3b8e96b2.json').read_text())
    secs=sections(plan)
    tr=SectionTracker(secs,differential_config(PathConfig(cruise_mps=.8)),GEO,
                      constant_straight_speed=True,constant_paint_speed=True)
    tr.set_nozzle(.24,-.4575)
    return tr

class CornerStartNoAbort(unittest.TestCase):
    def test_actual_short_runup_and_paint_share_one_control_reference(self):
        route=recorded();tr=route.trackers[0];sec=route.sections[0];mark=tr.paint_from_m
        self.assertAlmostEqual(sec.path._piece(mark-.17).length,.8333333333333334)
        self.assertIs(tr.path._piece(mark-.17),tr.path._piece(mark+.1))
        self.assertGreater(tr.path._piece(mark-.17).length,tr.cfg.straight_min_length_m)
        self.assertEqual(tr.paint_from_m,sec.paint_from_m)
        self.assertEqual(tr.paint_to_m,sec.paint_to_m)
        for i in range(101):
            s=i*sec.path.length/100
            for a,b in zip(sec.path.pose(s),tr.path.pose(s)):self.assertAlmostEqual(a,b,places=10)

    def test_unready_does_not_stop_at_recorded_boundary_or_after_crossing(self):
        route=recorded();tr=route.trackers[0];mark=tr.paint_from_m
        p=pose(*tr.path.pose(mark-.3),.5)
        self.assertTrue(tr.start(p))
        for s in (mark-.18,mark-.17,mark-.05,mark+.02,mark+.5):
            tr._hint=s;p=pose(*tr.path.pose(s),.5)
            v,w=tr.update(p,DT)
            self.assertEqual(tr.st.state,'RUNNING')
            self.assertGreaterEqual(v,.35)
            self.assertFalse(tr.paint_start_allowed())
        # 真正的定位不可用仍按既有保护暂停。
        p.usable_for_control=False
        self.assertEqual(tr.update(p,DT),(0.,0.))
        self.assertEqual(tr.st.state,'PAUSED')

if __name__=='__main__':unittest.main()
