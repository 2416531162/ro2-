"""弧尾提前停喷（2026-10-08 用户）：罚球弧两端落在禁区线上，弧尾越过横线一截。「可以少喷点，横线喷 10 cm 宽能盖住」：
弧（不是整圆）喷到终点前 5 cm 就停；直线线尾仍越线截止，整圆仍过交点多喷。"""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_continuous_field_arcs as fa  # noqa: E402
import paint_quality  # noqa: E402
from field_planner import plan  # noqa: E402
from route_operations import RouteOperations  # noqa: E402


def penalty_arc():
    """禁区弧＋直线（10-08 实车那张图纸的形状：R 3.54 m、扫 106°，两端落在直线上）。"""
    c, r, a0, sweep = (3., -2.13), 3.541, 143.05, -106.1
    ends = [(c[0] + r * math.cos(math.radians(a)), c[1] + r * math.sin(math.radians(a))) for a in (a0, a0 + sweep)]
    return dict(execution_order='optimized', boundary=[[-2, 0], [8, 0], [8, 2], [-2, 2]], elements=[
        dict(id='arc', kind='arc', center=list(c), radius=r, start_deg=a0, sweep_deg=sweep),
        dict(id='line', kind='polyline', points=[[-2., 0.], [8., 0.]])]), ends


def painting_at(job, start, pick):
    """按生产喷线资格逐段查：pick(section) 选要看的段，返回 (段, 在 s 处喷不喷的函数)。"""
    tr = fa.tracker(fa.sections(plan(job, start, fa.CFG)))
    m = NS(route_path=None, tracker=tr, route=None, differential=True, phase='RUNNING', nozzle=(.24, -.4575))
    ops = RouteOperations(m)
    ops.record = dict(id='test', field_plan=True, field_context=dict(forward=.24, right=.4575))
    tr.st.state = 'RUNNING'
    i = next(i for i, sec in enumerate(tr.sections) if pick(sec))
    sec, child = tr.sections[i], tr.trackers[i]
    tr.index = i
    child.st.state = 'RUNNING'

    def at(s, k=[0]):
        k[0] += 1
        child.st.s = s
        child.st.kappa_ref = sec.path.kappa(s)
        m.pose = fa.pose(*sec.car_pose(s), .5, .5 * sec.path.kappa(s))
        ops.update_paint_tail(k[0] * .02 + i * 100)
        return ops.paint_status()['painting']
    return sec, at


def is_arc(sec):
    return sec.field_role == 'paint' and not sec.path._piece(sec.paint_to_m - 1e-6).straight


class ArcEndEarly(unittest.TestCase):
    def test_penalty_arc_stops_5_cm_before_the_line(self):
        job, ends = penalty_arc()
        sec, at = painting_at(job, (3., 1.5, 0.), is_arc)
        q = fa.rp.nozzle_at(*sec.car_pose(sec.paint_to_m), .24, -.4575)
        self.assertLess(min(math.dist(q, e) for e in ends), 1e-6)       # 规划终点仍是图纸端点
        self.assertTrue(at(sec.paint_to_m - .06))
        self.assertFalse(at(sec.paint_to_m - .045))                    # 离终点 5 cm 内不喷
        self.assertFalse(at(sec.paint_to_m))

    def test_corner_arc_stops_early_but_straight_legs_still_run_past_their_end(self):
        sec, at = painting_at(fa.corner(), (-.3, .4, 0.), is_arc)
        self.assertTrue(at(sec.paint_to_m - .06))
        self.assertFalse(at(sec.paint_to_m - .045))
        sec, at = painting_at(fa.corner(), (-.3, .4, 0.),
                              lambda s: s.field_role == 'paint' and s.path._piece(s.paint_to_m - 1e-6).straight)
        self.assertTrue(at(sec.paint_to_m - .01))                       # 直线线尾不提前

    def test_full_circle_is_unchanged(self):
        sec = NS(path=fa.Route((0., 0., 0.), [(2 * math.pi * 2. + .2, .5)]))
        self.assertEqual(paint_quality.arc_end_early(sec, 0., sec.path.length), 0.)
        sec = NS(path=fa.Route((0., 0., 0.), [(2., 0.), (3., .5)]))
        self.assertEqual(paint_quality.arc_end_early(sec, 0., 5.), .05)   # 弧
        self.assertEqual(paint_quality.arc_end_early(sec, 0., 1.5), 0.)   # 区间结束在直线上
        self.assertAlmostEqual(paint_quality.arc_end_early(sec, 4.98, 5.), .02)  # 不超过区间长


if __name__ == '__main__':
    unittest.main()
