"""Compare route steering with the existing A/B controller; all checks are offline."""
import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT)]
from line_cascade import CascadeConfig, ImuLineTracker
from line_tracker import lateral_integral_allowed
from path_tracker import Path as Route, PathConfig, PathTracker, differential_config, wrap
from robot_core.kinematics import ChassisGeometry

DT = .02
NX, NY = .23, -.4525
GEO = ChassisGeometry(drive='differential', track_m=.675, diff_min_radius_m=.5)


def pose(x, y=0., yaw=0., speed=.8, gyro=0., frozen=False):
    return SimpleNamespace(x=x, y=y, yaw=yaw, speed=speed, yaw_rate=gyro,
        usable_for_control=True, quality='RTK_FIXED', reasons=(), correction_yaw=0.,
        correction_xy=(.02, 0.) if frozen else (0., 0.))


def route(path=None, enabled=True, law='imu_first'):
    """law='ab'：原 A/B 控制律（straight_imu_first=None）；默认用现在差速车配置的直线 IMU 为主。"""
    cfg = replace(differential_config(PathConfig(cruise_mps=.8)), straight_imu_enabled=enabled)
    if law == 'ab':
        cfg = replace(cfg, straight_imu_first=None)
    tr = PathTracker(path or Route((0., 0., 0.), [(30., 0.)]), cfg, GEO)
    tr.set_nozzle(NX, NY)
    return tr


class RouteABSteering(unittest.TestCase):
    def test_steady_route_matches_ab_with_rotations_sample_age_and_frozen_integral(self):
        # Real A/B wrapper is the reference, including its own coordinate transforms and PID history.
        for angle in (0., math.pi / 2, math.pi - .01, -math.pi + .01):
            with self.subTest(angle=angle):
                c, s = math.cos(angle), math.sin(angle)
                tr = route(Route((0., 0., angle), [(30., 0.)]), law='ab')
                ab = ImuLineTracker(CascadeConfig(nozzle_x_m=NX, nozzle_y_m=NY,
                                   track_x_m=NX, track_y_m=NY), GEO)
                a = (NX*c-NY*s, NX*s+NY*c)
                ab.set_line(a, (a[0]+30*c, a[1]+30*s))
                p = pose(2*c, 2*s, angle)
                tr.sync(0., (0., angle, 0.)); ab.sync(0., (0., angle, 0.))
                self.assertTrue(tr.start(p)); self.assertTrue(ab.start(p))
                # Isolate shared steady steering from the route's intentional entry blend.
                tr._straight_weight = 1.
                tr._straight_piece = tr.path.pieces[0]
                tr._straight_offset, tr._straight_offset_n = 0., 1
                tr._straight_target = angle
                tr._straight_core.reset(now=0.)
                for i in range(1, 181):
                    t = i*DT
                    age = .005 if i % 2 else .015
                    gy = 0. if i == 1 else math.radians(.12)*math.cos(t)
                    raw = angle if i == 1 else wrap(angle + math.radians(.12)*math.sin(t))
                    yaw = wrap(raw + gy*age)
                    along = 2.+.8*(t-DT)
                    across = 0. if i == 1 else .008*math.sin(t*2.)
                    p = pose(along*c-across*s, along*s+across*c, yaw,
                             .6 if i % 2 else 1.2, gy, frozen=70 <= i < 110)
                    imu = (t-age, raw, gy)
                    tr.sync(t, imu); ab.sync(t, imu)
                    tr._clock = t; tr.st.s = along
                    tr.st.integral_frozen = not lateral_integral_allowed(p)
                    v_ab, w_ab = ab.update(p, DT)
                    w_route = tr._straight_control(p, DT, .8, .123)
                    tr._sent.append((t, .8, w_route))
                    tr._straight_core.record_sent(t, w_route)
                    self.assertAlmostEqual(v_ab, .8, places=12)
                    self.assertAlmostEqual(w_route, w_ab, places=10)
                    self.assertAlmostEqual(tr._straight_target, ab._target, places=10)
                    self.assertAlmostEqual(tr._straight_status['outer_e'], ab.st.e, places=10)
                    self.assertAlmostEqual(tr._straight_integral, -ab.cfg.k_i*ab.st.integral, places=10)

    def test_route_speed_and_task_state_remain_with_path_controller(self):
        a, b = route(), route(enabled=False)
        a.constant_straight_speed = b.constant_straight_speed = True
        p = pose(0., speed=0.)
        a.sync(0., (0., 0., 0.))
        self.assertTrue(a.start(p)); self.assertTrue(b.start(p))
        for i in range(1, 201):
            t = i*DT; p = pose(t*.8, .005*math.sin(t), speed=.8)
            a.sync(t, (t-.005, 0., 0.))
            va, _ = a.update(p, DT); vb, _ = b.update(p, DT)
            self.assertEqual(va, vb)
            self.assertEqual(a.st.state, b.st.state)

    def test_short_straight_uses_ab_from_start_without_hall_speed_gate(self):
        tr = route(Route((0., 0., 0.), [(1.5, 0.)]), law='ab')
        p = pose(0., speed=0.)
        tr.sync(0., (0., 0., 0.)); self.assertTrue(tr.start(p))
        tr.sync(DT, (DT, 0., 0.)); tr.update(p, DT)
        self.assertTrue(tr._straight_status['active'])
        self.assertEqual(tr._straight_status['control_law'], 'ab')

    def test_arc_only_output_is_identical_to_original_path_controller(self):
        path = Route((0., 0., 0.), [(4., .5)])
        a, b = route(path), route(path, enabled=False)
        p = pose(0., speed=0.)
        a.sync(0., (0., 0., 0.)); self.assertTrue(a.start(p)); self.assertTrue(b.start(p))
        for i in range(1, 151):
            t = i*DT; x, y, yaw = path.pose(t*.3)
            p = pose(x, y, yaw, .3, .15)
            a.sync(t, (t, yaw, .15))
            self.assertEqual(a.update(p, DT), b.update(p, DT))
            self.assertFalse(a._straight_status['active'])

    def test_straight_blend_finishes_before_curve_feedforward(self):
        path = Route((0., 0., 0.), [(8., 0.), (4., .5)])
        tr = route(path)
        tr.sync(0., (0., 0., 0.)); self.assertTrue(tr.start(pose(0., speed=0.)))
        active = False
        for i in range(1, 951):
            t = i*DT; along = t*.5
            x, y, yaw = path.pose(along)
            gy = 0. if along < 8. else .25
            tr.sync(t, (t, yaw, gy)); tr.update(pose(x, y, yaw, .5, gy), DT)
            active |= tr._straight_weight > .99
            if along >= 7.7:
                self.assertEqual(tr._straight_weight, 0.)
        self.assertTrue(active)

    def test_imu_missing_or_stale_falls_back_without_vetoing_good_pose(self):
        a, b = route(), route(enabled=False)
        p = pose(2.)
        self.assertTrue(a.start(p)); self.assertTrue(b.start(p))
        for i in range(1, 31):
            t = i*DT; p = pose(2.+t*.8, .01)
            a.sync(t, None)
            self.assertEqual(a.update(p, DT), b.update(p, DT))
            self.assertEqual(a.st.state, 'RUNNING')
        a.sync(2., None)
        self.assertEqual(a._straight_control(p, DT, .8, .0123), .0123)
        a.sync(2., (2., 0., 0.))
        a.sync(2.21, None)
        self.assertEqual(a._straight_control(p, DT, .8, .0123), .0123)
        self.assertEqual(a.st.state, 'RUNNING')

    def test_pause_resume_clears_outer_distance_and_target_history(self):
        tr = route(law='ab'); p = pose(2., .01)
        tr.sync(0., (0., 0., 0.)); self.assertTrue(tr.start(p))
        for i in range(1, 61):
            t = i*DT; tr.sync(t, (t, 0., 0.)); tr.update(pose(2.+.8*t, .01), DT)
        self.assertTrue(tr._straight_weight > 0.)
        tr.pause()
        self.assertEqual(tr._straight_weight, 0.)
        self.assertIsNone(tr._straight_last_s)
        tr.sync(3., (3., 0., 0.)); self.assertTrue(tr.start(pose(3., .01)))
        tr.sync(3.+DT, (3.+DT, 0., 0.)); tr.update(pose(3., .01), DT)
        self.assertEqual(tr._straight_status['outer_distance_m'], 0.)

    def _runup(self):
        # 同 SectionTracker 给 Gen5 喷线段设的助跑：1.5 m 助跑后喷头到线头
        tr = route(Route((0., 0., 0.), [(14., 0.)]))
        tr.constant_straight_speed = True
        tr.runup_speed_mps, tr.runup_distance_m, tr.paint_from_m = .3, 1.5, 1.5
        tr.sync(0., (0., 0., 0.)); self.assertTrue(tr.start(pose(0., speed=0.)))
        return tr

    def _drive(self, tr, offset, upto, t0=0., v=.5):
        """车平行于线、横偏 offset 走到 upto，返回每拍 (路程, A/B 权重, 是否在等助跑收稳)。"""
        out, i = [], 0
        while True:
            i += 1; t = t0 + i*DT
            x = (out[-1][0] if out else tr.st.s) + v*DT
            if x > upto: return out, t
            tr.sync(t, (t, 0., 0.)); tr.update(pose(x, offset(x), speed=v), DT)
            out.append((x, tr._straight_weight, tr._straight_status.get('runup_wait', False)))

    def test_runup_keeps_path_control_until_start_offset_is_corrected(self):
        # 2026-10-06 实车：起步偏 4–5 cm 时 A/B 从 0.15 m 接管，横偏先涨到 7.8 cm，线头喷头偏 5.4 cm
        tr = self._runup()
        out, t = self._drive(tr, lambda x: .04, 1.45)
        self.assertTrue(all(w == 0. for _, w, _ in out))
        self.assertTrue(out[-1][2])
        # 到线头不再等：照常渐入 A/B
        out, _ = self._drive(tr, lambda x: .04, 2.4, t)
        self.assertGreater(out[-1][1], .5)

    def test_runup_hands_over_after_start_hold_once_settled_and_stays(self):
        tr = self._runup()
        out, t = self._drive(tr, lambda x: .01, 1.2)
        before = [w for x, w, _ in out if x < tr.cfg.start_hold_m]
        self.assertTrue(before and all(w == 0. for w in before))
        self.assertGreater(out[-1][1], 0.)
        # 交过以后助跑里横偏变大也不收回
        out, _ = self._drive(tr, lambda x: .03, 1.45, t)
        self.assertTrue(all(w > 0. and not wait for _, w, wait in out))

    def test_reverse_section_still_disables_forward_imu_steering(self):
        from path_tracker import Section, SectionTracker
        sections = [Section(-1, Route((0., 0., 0.), [(8., 0.)]))]
        st = SectionTracker(sections, differential_config(PathConfig()), GEO)
        self.assertFalse(st.trackers[0].cfg.straight_imu_enabled)


if __name__ == '__main__':
    unittest.main()
