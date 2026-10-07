"""接入、转场（不喷）的速度（2026-10-07 用户）：离点远提速到 1.5 m/s；离正式路线（喷线起点）3 m 内不按 1.5，按刹车曲线
降到画线速度准备接入；整段不到 3 m 就不提速。"""
from collections import deque
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT)]
from live_cruise import label_sections
from path_tracker import Path as Route, PathConfig, Section, SectionTracker, differential_config
from robot_core.kinematics import ChassisGeometry

GEO = ChassisGeometry(drive='differential', track_m=.675, diff_min_radius_m=.5)
DT = .02


def run(sections, stop_at):
    """理想差速车（执行延迟 0.3 s）沿 x 轴开 SectionTracker，到 x = stop_at 为止。返回 [(离 stop_at, 车速)]。"""
    tr = SectionTracker(sections, differential_config(PathConfig(cruise_mps=.8)), GEO)
    x = v = 0.
    sent = deque([(0., 0.)])
    t, rows = 0., []

    def pose():
        return NS(x=x, y=0., yaw=0., speed=v, yaw_rate=0., usable_for_control=True, quality='RTK_FIXED', reasons=(),
                  correction_xy=(0., 0.), correction_yaw=0., rtk_trusted=True, hold_age_s=0.)
    assert tr.start(pose())
    while t < 60. and tr.st.state == 'RUNNING' and x < stop_at:
        vc, _ = tr.update(pose(), DT)
        sent.append((t, vc))
        while len(sent) > 1 and sent[1][0] <= t - .3:
            sent.popleft()
        v = sent[0][1]
        x += v * DT
        t += DT
        rows.append((stop_at - x, v))
    return rows


def drive(transit_m, paint_from=1.5, line_m=6.):
    """直线：前进转场 transit_m（1.5 m/s）接前进喷线段（助跑 paint_from 后开喷，0.8 m/s）。返回 [(离喷线起点, 车速)]。"""
    a = Section(1, Route((0., 0., 0.), [(transit_m, 0.)]))
    a.field_role, a.cruise_mps, a.paint_from_m = 'transit', 1.5, transit_m
    b = Section(1, Route((transit_m, 0., 0.), [(paint_from + line_m, 0.)]))
    b.field_role, b.cruise_mps, b.paint_from_m = 'paint', .8, paint_from
    return run([a, b], transit_m + paint_from)


def entry_in_line(paint_from, line_m=6.):
    """车已在线的延长线上、线头后方 paint_from：一段喷线段，线头前那截按接入速度 1.5（label_sections）。"""
    b = Section(1, Route((0., 0., 0.), [(paint_from + line_m, 0.)]))
    b.field_role, b.cruise_mps, b.paint_from_m, b.entry_speed_mps = 'paint', .8, paint_from, 1.5
    return run([b], paint_from)


class ApproachSpeed(unittest.TestCase):
    def test_far_transit_runs_fast_then_slows_within_three_metres(self):
        rows = drive(12.)
        self.assertGreater(max(v for d, v in rows if d > 5.), 1.45)
        self.assertLessEqual(max(v for d, v in rows if d <= 3.), .8 + 1e-6)

    def test_entry_on_the_line_extension(self):
        self.assertLessEqual(max(v for d, v in entry_in_line(2.5)), .8 + 1e-6)   # 不到 3 m：不提速
        far = entry_in_line(10.)
        self.assertGreater(max(v for d, v in far if d > 5.), 1.45)
        self.assertLessEqual(max(v for d, v in far if d <= 3.), .8 + 1e-6)

    def test_route_entry_is_labelled_transit_speed(self):
        a = Section(1, Route((0., 0., 0.), [(5., 0.)]))
        b = Section(1, Route((5., 0., 0.), [(8., 0.)]))
        r = Section(-1, Route((0., 0., math.pi), [(1., 0.)]))
        label_sections([a, r, b], 6.5, .8)
        self.assertEqual((a.field_role, a.cruise_mps), ('transit', 1.5))
        self.assertEqual((r.field_role, r.cruise_mps), ('transit', .8))     # 倒车段按倒车速度封顶（SectionTracker）
        self.assertEqual((b.field_role, b.cruise_mps, b.entry_speed_mps), ('paint', .8, 1.5))
        self.assertAlmostEqual(b.paint_from_m, .5)


if __name__ == '__main__':
    unittest.main()
