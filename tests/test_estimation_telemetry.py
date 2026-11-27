#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实时推算遥测（line.estimation）单元测试：只读透传融合与直线/圆弧控制器状态，不改控制行为。"""
import math
import os
import sys
import unittest
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

import app_link as al  # noqa: E402
import line_mission as lm  # noqa: E402


class EstimationTelemetryTest(unittest.TestCase):
    def _base_line(self, **overrides):
        base = {
            '_rx_monotonic': 100.0,
            'phase': 'RUNNING',
            'message': '行驶中',
            'pose': {
                'x': 5.0, 'y': 0.01, 'yaw_deg': 0.2, 'speed': 0.8,
                'quality': 'RTK_FIXED', 'usable': True, 'sigma_xy': 0.011, 'sigma_yaw_deg': 0.09,
                'reasons': [], 'rtk_trusted': True, 'hold_age_s': 0.05, 'hold_limit_s': 9.0,
                'speed_source': 'wheel', 'yaw_source': 'imu',
            },
            'tracker': {
                'state': 'RUNNING', 'reason': '', 'e': 0.008, 'heading_err_deg': 0.2,
                's': 8.0, 'length': 30.0, 'v_cmd': 0.8, 'gear': 'forward',
                'section': 1, 'sections': 2, 'switching': False, 'pivoting': False,
                'straight_hold': {
                    'active': True, 'weight': 1.0, 'control_law': 'imu_first',
                    'first_mode': 'rtk', 'first_locked': True, 'first_rtk_accepted': True,
                    'first_poor_sky': True, 'first_poor_sky_exit': None,
                    'grass': False, 'ground_a': 2.6, 'first_position_follow': False,
                    'pull_boost': False, 'rtk_antenna': 'main', 'first_source': 'carried',
                    'baseline_ok': True, 'first_fast': False,
                    'first_e': 0.0082, 'outer_e_rtk': 0.0214, 'first_innovation': 0.0132,
                    'first_beta_deg': 0.246, 'first_crab_deg': -0.118,
                    'first_imu_only_s': 1.44, 'first_jumps': 2, 'sigma_lp_cm': 1.842,
                },
                'arc_hold': {'active': False, 'weight': 0.0},
            },
        }
        base.update(overrides)
        return base

    def test_forwards_fusion_and_straight_estimation(self):
        tel = al.line_telemetry(self._base_line(), 100.0)
        est = tel['estimation']
        self.assertTrue(est['rtk_trusted'])
        self.assertEqual(est['hold_age_s'], 0.05)
        self.assertEqual(est['hold_limit_s'], 9.0)
        self.assertEqual(est['speed_source'], 'wheel')
        self.assertEqual(est['yaw_source'], 'imu')
        st = est['straight']
        self.assertTrue(st['active'])
        self.assertEqual(st['law'], 'imu_first')
        self.assertEqual(st['mode'], 'rtk')
        self.assertTrue(st['rtk_accepted'])
        self.assertTrue(st['poor_sky'])
        self.assertFalse(st['grass'])
        self.assertEqual(st['ground_a'], 2.6)
        self.assertEqual(st['antenna'], 'main')
        self.assertEqual(st['source'], 'carried')
        self.assertEqual(st['e_imu_m'], 0.0082)
        self.assertEqual(st['e_rtk_m'], 0.0214)
        self.assertEqual(st['diff_m'], 0.0132)
        self.assertEqual(st['beta_deg'], 0.25)
        self.assertEqual(st['crab_deg'], -0.12)
        self.assertEqual(st['imu_only_s'], 1.4)
        self.assertEqual(st['jumps'], 2)
        self.assertEqual(st['sigma_lp_cm'], 1.84)
        self.assertNotIn('arc', est)

    def test_forwards_arc_estimation_and_suppresses_during_switching_or_idle(self):
        ln = self._base_line()
        ln['tracker']['straight_hold'] = {'active': False, 'weight': 0.0}
        ln['tracker']['arc_hold'] = {
            'active': True, 'weight': 1.0, 'partial_arc': True,
            'radial_source': 'rtk_paired', 'radial_rtk_weight': 0.85,
            'good_sky': False, 'holding': False, 'rtk_ok': True,
            'e_est': -0.006, 'e_rtk': -0.011, 'jumps': 1,
        }
        est = al.line_telemetry(ln, 100.0)['estimation']
        self.assertNotIn('straight', est)
        arc = est['arc']
        self.assertTrue(arc['active'])
        self.assertTrue(arc['partial'])
        self.assertEqual(arc['radial_source'], 'rtk_paired')
        self.assertEqual(arc['radial_rtk_weight'], 0.85)
        self.assertEqual(arc['e_imu_m'], -0.006)
        self.assertEqual(arc['e_rtk_m'], -0.011)
        self.assertEqual(arc['jumps'], 1)

        # 原地转向 / 换段 / 空闲时不透传上一段的直线或圆弧状态
        for patch in ({'switching': True}, {'pivoting': True}, {'state': 'PAUSED'}):
            ln2 = self._base_line()
            ln2['tracker'].update(patch)
            est2 = al.line_telemetry(ln2, 100.0)['estimation']
            self.assertNotIn('straight', est2)
            self.assertNotIn('arc', est2)

    def test_legacy_snapshot_omits_estimation_and_sanitizes_bad_values(self):
        legacy = {
            '_rx_monotonic': 100.0, 'phase': 'IDLE', 'message': '',
            'pose': {'x': 1.0, 'y': 2.0, 'yaw_deg': 0.0, 'speed': 0.0,
                     'quality': 'RTK_FIXED', 'usable': True, 'sigma_xy': 0.01, 'sigma_yaw_deg': 0.1, 'reasons': []},
            'tracker': {'state': 'IDLE'},
        }
        self.assertNotIn('estimation', al.line_telemetry(legacy, 100.0))

        bad = self._base_line()
        bad['pose'].update(hold_age_s=math.nan, hold_limit_s=-1.0, position_antenna='third',
                           speed_source='gps', yaw_source='mag', rtk_trusted='yes')
        bad['tracker']['straight_hold'].update(first_e=math.inf, first_jumps=-3, first_mode='unknown',
                                               first_poor_sky_exit='other', rtk_antenna='ant3')
        est = al.line_telemetry(bad, 100.0)['estimation']
        self.assertNotIn('rtk_trusted', est)
        self.assertNotIn('position_antenna', est)
        self.assertNotIn('hold_age_s', est)
        self.assertNotIn('hold_limit_s', est)
        self.assertNotIn('speed_source', est)
        self.assertNotIn('yaw_source', est)
        self.assertIsNone(est['straight']['e_imu_m'])
        self.assertIsNone(est['straight']['jumps'])
        self.assertIsNone(est['straight']['mode'])
        self.assertIsNone(est['straight']['poor_sky_exit'])
        self.assertIsNone(est['straight']['antenna'])

    def test_line_mission_status_exports_hold_and_sensor_sources(self):
        fusion = SimpleNamespace(config=SimpleNamespace(rtk_hold_s=9.0), to_geodetic=lambda x, y: (30.0, 120.0))
        m = lm.LineMission(fusion=fusion, samples=None)
        m.pose = SimpleNamespace(
            x=1.0, y=2.0, yaw=0.0, speed=0.5, quality='DEAD_RECKONING', usable_for_control=True,
            sigma_xy=0.04, sigma_yaw=0.01, reasons=('rtk_hold',), rtk_trusted=False,
            position_antenna='second', hold_age_s=2.456, speed_source='gnss_doppler', yaw_source='imu',
        )
        p = m.status()['pose']
        self.assertEqual(p['position_antenna'], 'second')
        self.assertEqual(p['hold_age_s'], 2.46)
        self.assertEqual(p['hold_limit_s'], 9.0)
        self.assertEqual(p['speed_source'], 'gnss_doppler')
        self.assertEqual(p['yaw_source'], 'imu')


if __name__ == '__main__':
    unittest.main()
