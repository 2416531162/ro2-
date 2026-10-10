"""喷线圆弧段停车后重新起步不接上一段的积分（2026-10-11 用户「先做 1」，SectionTracker._paint_curve）：上一段转场急弯攒下的
转向补偿带进进弧助跑会变成固定的拐弯偏置，把喷头推开 2–3 cm（仿真整场最差的角球弧线头即此）。喷线长直线照旧按已在线上学到
的积分（_spray_straight），转场段照旧接续。"""
from pathlib import Path
import math
import sys
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'radar_system'), str(ROOT)]
from path_tracker import Path as Route, PathConfig, Section, SectionTracker, circle, differential_config
from robot_core.kinematics import ChassisGeometry

GEO = ChassisGeometry(drive='differential', track_m=.675, diff_min_radius_m=.5)


def switched(nxt, carried=.02, role='paint'):
    """前一段（转场）跑完、带着积分 carried 停在下一段起点；返回下一段起步时的积分。"""
    first = Section(1, Route((-3., 0., 0.), [(3., 0.)]))
    first.field_role = 'transit'
    nxt.field_role = role
    tr = SectionTracker([first, nxt], differential_config(PathConfig(cruise_mps=.5)), GEO, pivot_rate=.5)
    for t in tr.trackers:
        t.set_nozzle(.24, -.4575)
    x, y, h = nxt.path.pose(0.)
    pose = NS(x=x, y=y, yaw=h, speed=0., yaw_rate=0., usable_for_control=True, quality='RTK_FIXED',
              reasons=(), correction_xy=(0., 0.))
    tr.st.state, tr.index, tr.switching, tr._next, tr._integral = 'RUNNING', 0, True, 1, carried
    for _ in range(100):
        tr.update(pose, .02)
        if not tr.switching:
            break
    assert not tr.switching and tr.index == 1
    return tr.trackers[1].st.integral


class PaintCurveIntegral(unittest.TestCase):
    def test_paint_circle_and_arc_start_from_zero(self):
        self.assertEqual(switched(Section(1, circle(2., left=False, lead_in=2.8, lead_out=.5))), 0.)
        arc = Section(1, Route((0., 0., 0.), [(1.4, 0.), (1.48, -1/.94), (.5, 0.)]))   # 角球弧：直线助跑 + 圆弧
        self.assertEqual(switched(arc), 0.)

    def test_transit_curve_and_painted_straight_keep_their_rules(self):
        self.assertAlmostEqual(switched(Section(1, circle(2., left=False)), role='transit'), .02)
        straight = Section(1, Route((0., 0., 0.), [(12., 0.)]))
        self.assertEqual(switched(straight), 0.)      # 喷线长直线：没有在线上学到的积分就从 0（原来的规则）

    def test_run_in_straight_does_not_learn_but_the_curve_before_paint_does(self):
        # 2026-10-11 用户「补上」：进弧助跑直线部分偏开的暂时偏差不积进去；进到圆上照常学（补小半径转不够）
        def integral_at(role):
            sec = Section(1, Route((0., 0., 0.), [(2., 0.), (2., -1/1.2), (.5, 0.)]))
            sec.field_role, sec.paint_from_m = role, 3.
            tr = SectionTracker([sec], differential_config(PathConfig(cruise_mps=.3)), GEO)
            tr.trackers[0].set_nozzle(.24, -.4575)
            seen, s = {}, 0.
            def pose(s, speed):
                x, y, h = sec.path.pose(s)
                return NS(x=x-math.sin(h)*.01, y=y+math.cos(h)*.01, yaw=h, speed=speed, yaw_rate=0., usable_for_control=True,
                          quality='RTK_FIXED', reasons=(), correction_xy=(0., 0.), rtk_trusted=True)
            self.assertTrue(tr.start(pose(0., 0.)))
            while s < 2.95:
                s += .006
                tr.update(pose(s, .3), .02)
                for mark in (1.9, 2.9):
                    if mark not in seen and s >= mark:
                        seen[mark] = tr.trackers[0].st.integral
            return seen
        paint, transit = integral_at('paint'), integral_at('transit')
        self.assertEqual(paint[1.9], 0.)                 # 直线助跑：不学
        self.assertNotEqual(paint[2.9], 0.)              # 圆上、开喷前：照常学
        self.assertNotEqual(transit[1.9], 0.)            # 转场段照原控制


if __name__ == '__main__':
    unittest.main()
