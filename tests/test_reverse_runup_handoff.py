"""倒车接入后的交接回归：助跑仍为 1.5 m，只调整转向交接及初始估计。"""
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT)]
import route_plan as rp
from live_cruise import label_sections
from path_tracker import PathConfig, SectionTracker, differential_config
from robot_core.kinematics import ChassisGeometry


def tracker():
    nozzle = (.24, -.4575)
    car = rp.axle_for((-.8, 0.), 0., *nozzle) + (0.,)
    spec = rp.RouteSpec(kind='path', points=((0., 0.), (25., 0.)), closed=False, cruise=.8)
    sections, entry = rp.plan_current(spec, car, .556, nozzle_x=nozzle)
    label_sections(sections, entry, .8)
    route = SectionTracker(sections, differential_config(PathConfig(cruise_mps=.8)),
                           ChassisGeometry(drive='differential', track_m=.675, diff_min_radius_m=.5),
                           constant_straight_speed=True, reverse_runup=True)
    tr = route.trackers[1]
    tr.set_nozzle(*nozzle)
    tr._creep_from = 0.
    return tr


def fused(tr, s=1.2, e=.003):
    x, y, yaw = tr.path.pose(s)
    return NS(x=x, y=y + e, yaw=yaw, speed=.3, yaw_rate=0., usable_for_control=True,
              quality='RTK_FIXED', rtk_trusted=True, reasons=(), correction_xy=(0., 0.), correction_yaw=0.)


class ReverseHandoff(unittest.TestCase):
    def test_residual_heading_is_not_released_at_two_degrees(self):
        tr = tracker()
        tr.st.s, tr.st.e, tr.st.heading_err = 1.2, .003, math.radians(2.)
        self.assertEqual(tr._straight_eligible(tr.path.pieces[0], .3), (False, True))

    def test_crossing_paint_start_does_not_force_handoff(self):
        tr = tracker()
        tr.st.s, tr.st.e, tr.st.heading_err = 1.6, .025, 0.
        self.assertEqual(tr._straight_eligible(tr.path.pieces[0], .8), (False, True))
        tr.st.e = .003
        self.assertEqual(tr._straight_eligible(tr.path.pieces[0], .8), (True, False))
        tr.st.e = .02
        self.assertEqual(tr._straight_eligible(tr.path.pieces[0], .8), (True, False))

    def test_trusted_runup_fits_position_while_lateral_integral_stays_frozen(self):
        tr = tracker()
        tr.st.integral_frozen = True
        tr.st.integral = .012
        g = tr._straight_guide
        g.offset, g.locked, g.source = 0., True, 'learned'
        for i in range(1, 160):
            t = i * .02
            tr.st.s, tr.st.e, tr.st.heading_err = .1 + .006 * i, .03, 0.
            tr.sync(t, (t, 0., 0.))
            tr.set_heading_baseline(t, .41)
            tr.set_rtk_quality(t, .01, 28, .01)
            tr._straight_first(fused(tr), .02, .3, 0., tr.path.pieces[0], 0., 0.)
        self.assertGreater(g.fit_points, 0)
        self.assertTrue(tr.st.integral_frozen)
        self.assertEqual(tr.st.integral, .012)
        # 禁止积分不等于禁止可信拟合，但浮点位置仍不能进入拟合。
        points = g.fit_points
        for i in range(160, 210):
            t = i * .02
            tr.st.s = 1.1
            tr.sync(t, (t, 0., 0.))
            tr.set_heading_baseline(t, .41)
            tr.set_rtk_quality(t, .01, 28, .01)
            bad = fused(tr)
            bad.quality, bad.rtk_trusted = 'RTK_FLOAT', False
            tr._straight_first(bad, .02, .3, 0., tr.path.pieces[0], 0., 0.)
        self.assertEqual(g.fit_points, points)

    def test_runup_fitting_requires_fresh_current_good_quality_not_just_debounce(self):
        tr = tracker()
        p = fused(tr)
        tr.sync(1., (1., 0., 0.))
        self.assertFalse(tr._runup_position_trusted(p))
        tr.set_heading_baseline(1., .41)
        tr.set_rtk_quality(1., .01, 28, .01)
        self.assertTrue(tr._runup_position_trusted(p))
        p.rtk_trusted = False
        self.assertFalse(tr._runup_position_trusted(p))
        p.rtk_trusted = True
        tr.set_rtk_quality(1.1, .04, 28, .01)
        tr.sync(1.1, (1.1, 0., 0.))
        self.assertFalse(tr._runup_position_trusted(p))
        tr.set_rtk_quality(1.2, .01, 28, .01)
        tr.sync(2., (2., 0., 0.))
        self.assertFalse(tr._runup_position_trusted(p))


if __name__ == '__main__':
    unittest.main()
