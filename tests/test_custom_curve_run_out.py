"""两点角弧/禁区弧的启动与停喷后收尾回归，只使用软件底盘。"""
from dataclasses import replace
from pathlib import Path
import json
import math
import sys
import tempfile
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT)]
import route_plan as rp
from path_tracker import PathConfig, SectionTracker, differential_config
from robot_core.kinematics import ChassisGeometry
from line_mission import LineMission
from line_tracker import LineConfig, LineTracker
from test_continuous_field_arcs import GEO, drive, pose


def two_point_arc(shape, left):
    """与 App 两点弧线相同的输入采样：两端相距 5.5 m，弦误差不超过 5 mm。"""
    alpha = math.pi / 4 if shape == 'corner' else math.acos(5.5 / 9.15)
    radius = 5.5 / (2 * math.sin(alpha))
    offset = radius * math.cos(alpha)
    count = max(2, math.ceil(2 * alpha / (2 * math.acos(1 - .005 / radius))))
    count += count % 2
    points = [(2.75 + radius * math.sin(-alpha + 2 * alpha * i / count),
               (1 if left else -1) * (radius * math.cos(-alpha + 2 * alpha * i / count) - offset))
              for i in range(count + 1)]
    points[0], points[-1] = (0., 0.), (5.5, 0.)
    return rp.RouteSpec(kind='path', points=tuple(points), closed=False, cruise=.8)


class CustomCurveRunOut(unittest.TestCase):
    def test_app_start_command_accepts_arc_and_persists_its_dry_tail(self):
        for shape in ('corner', 'penalty'):
            for left in (True, False):
                for reverse in (False, True):
                    with self.subTest(shape=shape, left=left, reverse=reverse), tempfile.TemporaryDirectory() as folder:
                        spec = two_point_arc(shape, left)
                        pts = spec.points[::-1] if reverse else spec.points
                        a, b = pts[:2]
                        heading = math.atan2(b[1]-a[1], b[0]-a[0])
                        nozzle = (.24, -.4575)
                        car = rp.axle_for(a, heading, *nozzle) + (heading,)
                        fusion = SimpleNamespace(pose=lambda _: pose(*car),
                            to_geodetic=lambda x, y: (30.+y*1e-5, 120.+x*1e-5),
                            to_local=lambda lat, lon: ((lon-120.)/1e-5, (lat-30.)/1e-5))
                        mission = LineMission(fusion, SimpleNamespace(), LineTracker(LineConfig(), GEO),
                            route_path=str(Path(folder) / 'route.json'), nozzle_x=nozzle)
                        ok, message = mission.command(dict(cmd='route_start_current',
                            route=spec.to_dict(digits=12), nozzle=dict(forward=nozzle[0], right=-nozzle[1])), 0.)
                        self.assertTrue(ok, message)
                        self.assertEqual(mission.phase, 'SELECTING')
                        saved = mission.operations.decode(mission.operations.record)
                        self.assertAlmostEqual(saved[-1].path.length-saved[-1].paint_to_m, .5)
                        self.assertEqual(saved[-1].paint_spans, mission.tracker.sections[-1].paint_spans)

    def test_complete_arc_job_stops_spraying_before_decelerating(self):
        for shape in ('corner', 'penalty'):
            for left in (True, False):
                for cruise in (.3, .8, 1.):
                    with self.subTest(shape=shape, left=left, cruise=cruise):
                        spec = replace(two_point_arc(shape, left), cruise=cruise)
                        a, b = spec.points[:2]
                        heading = math.atan2(b[1]-a[1], b[0]-a[0])
                        nozzle = (.24, -.4575)
                        axle = rp.axle_for(a, heading, *nozzle)
                        car = (axle[0]-3*math.cos(heading), axle[1]-3*math.sin(heading), heading)
                        sections, _ = rp.plan_current(spec, car, .556, nozzle_x=nozzle)
                        cfg = replace(differential_config(PathConfig(cruise_mps=cruise)), straight_imu_enabled=False)
                        tracker = SectionTracker(sections, cfg, GEO, pivot_rate=.8,
                            constant_straight_speed=True, constant_paint_speed=True)
                        tracker.set_nozzle(*nozzle)
                        rows = drive(tracker, 120.)
                        self.assertEqual(tracker.st.state, 'DONE', tracker.status())
                        painting = [r for r in rows if r['paint']]
                        self.assertGreater(len(painting), 100)
                        self.assertAlmostEqual(min(r['v'] for r in painting), max(r['v'] for r in painting))
                        last = max(i for i, r in enumerate(rows) if r['paint'])
                        self.assertTrue(any(r['v'] > .02 and not r['paint'] for r in rows[last+1:]))
                        self.assertEqual(rows[-1]['v'], 0.)

    def test_shared_app_fixture_matches_vehicle_geometry_and_spray_bounds(self):
        cases = json.loads((ROOT / 'tests/fixtures/custom_curve_run_out.json').read_text())
        self.assertEqual(len(cases), 16)
        for case in cases:
            with self.subTest(shape=case['shape'], left=case['left'],
                              nozzle=case['nozzle'], reverse=case['reverse']):
                spec = rp.parse_spec(case['route'])
                sections, entry = rp.plan_current(spec, tuple(case['car']), .556,
                                                  nozzle_x=tuple(case['nozzle']))
                self.assertAlmostEqual(entry, case['entry_m'], places=6)
                self.assertEqual(len(sections), len(case['sections']))
                for sec, want in zip(sections, case['sections']):
                    self.assertEqual(sec.direction, want['direction'])
                    for a, b in zip(sec.path.pose(0), want['start']):
                        self.assertAlmostEqual(a, b, places=6)
                    self.assertEqual(len(sec.path.pieces), len(want['pieces']))
                    for piece, (length, curvature) in zip(sec.path.pieces, want['pieces']):
                        self.assertAlmostEqual(piece.length, length, places=6)
                        self.assertAlmostEqual(piece.kappa, curvature, places=9)
                    self.assertAlmostEqual(sec.paint_from_m, want['paint_from'], places=6)
                    if want['paint_to'] is not None:
                        self.assertAlmostEqual(sec.paint_to_m, want['paint_to'], places=6)

    def test_generated_arcs_keep_paint_endpoint_and_pass_start_checks(self):
        for shape in ('corner', 'penalty'):
            for left in (True, False):
                original = two_point_arc(shape, left)
                for nozzle in ((.24, -.4575), (.3, 0.), (0., 0.)):
                    for reverse in (False, True):
                        for planner in (rp.plan, rp.plan_current):
                            with self.subTest(shape=shape, left=left, nozzle=nozzle,
                                              reverse=reverse, planner=planner.__name__):
                                spec = replace(original, points=original.points[::-1]) if reverse else original
                                a, b = spec.points[:2]
                                heading = math.atan2(b[1]-a[1], b[0]-a[0])
                                car = rp.axle_for(a, heading, *nozzle) + (heading,)
                                sections, _ = planner(spec, car, .556, nozzle_x=nozzle)
                                last = sections[-1]
                                run_out = rp.LINE_RUN_OUT_M if last.path.pieces[-1].straight else rp.CIRCLE_RUN_OUT_M
                                self.assertAlmostEqual(last.path.length-last.paint_to_m, run_out)
                                tip = rp.nozzle_at(*last.car_pose(last.paint_to_m), *nozzle)
                                self.assertLess(math.dist(tip, spec.points[-1]), .01)
                                self.assertLess(last.paint_spans[-1][1], last.path.length)
                                SectionTracker(sections, differential_config(PathConfig()),
                                               ChassisGeometry(drive='differential'),
                                               constant_paint_speed=True)
                                self.assertEqual(original.points[0], (0., 0.))
                                self.assertEqual(original.points[-1], (5.5, 0.))

    def test_straight_tail_remains_forty_centimetres(self):
        spec = rp.RouteSpec(kind='path', points=((0., 0.), (6.1, 0.)), closed=False)
        sections, _ = rp.plan_current(spec, (-.24, .4575, 0.), .556, nozzle_x=(.24, -.4575))
        last = sections[-1]
        self.assertAlmostEqual(last.path.length-last.paint_to_m, .4)
        self.assertTrue(last.path.pieces[-1].straight)


if __name__ == '__main__':
    unittest.main()
