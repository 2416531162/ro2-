"""独立画圆沿切线离开 1 m，整场、补画及两点弧线保持原收尾规则。只使用软件底盘。"""
from dataclasses import replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'radar_system'), str(ROOT), str(ROOT/'tests')]
import route_plan as rp
import circle_spray_repair
from field_planner import Settings, plan as field_plan
from live_cruise import label_sections
from path_tracker import Path as Route, PathConfig, Section, SectionTracker, differential_config
from route_operations import rework_sections
from test_continuous_field_arcs import GEO, drive, sections as field_sections
from test_circle_run_out import spec

NOZZLE = (.24, -.4575)


def circle_plan(radius, clockwise, laps=1, cruise=.8):
    circle = replace(spec(radius, clockwise, laps), cruise=cruise)
    start, loop = rp.loop_pieces(circle, .556, NOZZLE)
    (_, (x, y, yaw), _), = rp._circle_run_in(circle, [(1, start, loop)])[0]
    car = (x-3*math.cos(yaw), y-3*math.sin(yaw), yaw)
    planned, entry = rp.plan(circle, car, .556, nozzle_x=NOZZLE)
    label_sections(planned, entry, cruise)
    return circle, planned


class SoloCircleExit(unittest.TestCase):
    def test_distance_braking_exception_only_matches_the_full_circle_exit(self):
        for sweep, want in ((math.pi/2, 1.), (2*math.pi, 0.)):
            section = Section(1, Route((0., 0., 0.), [(2*sweep, -.5), (1., 0.)]))
            section.field_role = 'paint'
            section.paint_to_m = 2*sweep
            tracker = SectionTracker([section], differential_config(PathConfig()), GEO, constant_paint_speed=True)
            self.assertAlmostEqual(tracker.trackers[0].run_out_m, want)

    def test_complete_circle_keeps_paint_speed_and_finishes_the_one_metre_exit(self):
        for clockwise in (True, False):
            for radius in (1.5, 3.):
                for cruise in (.3, .8, 1.):
                    with self.subTest(clockwise=clockwise, radius=radius, cruise=cruise):
                        circle, planned = circle_plan(radius, clockwise, cruise=cruise)
                        cfg = replace(differential_config(PathConfig(cruise_mps=cruise)), straight_imu_enabled=False)
                        tracker = SectionTracker(planned, cfg, GEO, pivot_rate=.8,
                            constant_straight_speed=True, constant_paint_speed=True)
                        tracker.set_nozzle(*NOZZLE)
                        rows = drive(tracker, 180.)
                        self.assertEqual(tracker.st.state, 'DONE', tracker.status())
                        painting = [r for r in rows if r['paint']]
                        self.assertGreater(len(painting), 100)
                        self.assertAlmostEqual(min(r['v'] for r in painting), max(r['v'] for r in painting))
                        last = planned[-1]
                        self.assertAlmostEqual(last.path.length-last.paint_to_m, 1.)
                        # 参考路径精确为 1 m；带控制前视和 0.3 s 执行延迟的模型沿用既有 5 cm 停车验收范围。
                        self.assertAlmostEqual(rows[-1]['s']-last.paint_to_m, 1., delta=.05)
                        self.assertEqual(rows[-1]['v'], 0.)
                        end = max(i for i, row in enumerate(rows) if row['paint'])
                        self.assertTrue(any(r['v'] > .1 for r in rows[end+1:]))
                        self.assertFalse(any(r['paint'] for r in rows[end+1:]))
                        self.assertFalse(any(r['switching'] for r in rows))

    def test_field_circle_and_partial_circle_rework_keep_curved_half_metre_tail(self):
        cfg = Settings(.24, .4575, (.911, .604, .39), .557, turn='arc', turn_radius=1.)
        job = dict(boundary=[[-6,-6],[6,-6],[6,6],[-6,6]],
                   elements=[dict(id='circle', kind='circle', center=[0,0], radius=2.)])
        result = field_plan(job, (4., 4., 0.), cfg)
        last = field_sections(result)[-1]
        self.assertAlmostEqual(last.path.length-last.paint_to_m, .5)
        self.assertFalse(last.path.pieces[-1].straight)
        repaired, _ = rework_sections(spec(), 0, .5, 1.5, .556, (.911,.604,.39), nozzle=NOZZLE)
        self.assertAlmostEqual(repaired[-1].path.length-repaired[-1].paint_to_m, .5)
        self.assertFalse(repaired[-1].path.pieces[-1].straight)

    def test_optional_repair_circle_is_inserted_before_the_straight_exit(self):
        circle, planned = circle_plan(2., True)
        last = planned[-1]
        record = dict(kind='route', total_m=sum(s.path.length for s in planned), cursor_m=0.)
        mission = NS(_route_local=lambda:circle, nozzle=NOZZLE)
        circle_spray_repair.initialise(record, mission, planned, last.paint_from_m)
        self.assertIn('circle_spray', record)
        lap = record['circle_spray']['lap_m']
        extended = circle_spray_repair.append_path(last.path, lap)
        self.assertAlmostEqual(extended.length-last.path.length, lap)
        self.assertTrue(extended.pieces[-1].straight)
        self.assertAlmostEqual(extended.pieces[-1].length, 1.)
        before, after = last.path.pose(last.path.length), extended.pose(extended.length)
        for a, b in zip(before[:2], after[:2]):
            self.assertAlmostEqual(a, b, places=8)
        self.assertAlmostEqual(rp.wrap(before[2]-after[2]), 0., places=8)


if __name__ == '__main__':
    unittest.main()
