#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""2026-11-22 删掉没接上的推算误差预算：车上 fusion.json 还写着 hold_budget_enabled / hold_* 预算参数时照常加载（丢掉不用），
固定解刚断不再报 rtk_hold_pending。"""
import json
import os
import sys
import tempfile
import unittest
from unittest import mock

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, ROOT)

import ros_stubs  # noqa: E402

ros_stubs.install()
sys.modules['sensor_msgs.msg'].Imu = ros_stubs._simple('Imu')

import gnss_fusion  # noqa: E402
import line_node  # noqa: E402
import runtime_config  # noqa: E402
from gnss_fusion import FusionConfig  # noqa: E402

DIFF = dict(runtime_config.PROFILE, drive=dict(runtime_config.PROFILE.get('drive') or {}, type='differential'))
REMOVED = ('hold_budget_enabled', 'hold_lateral_limit_m', 'hold_along_limit_m', 'hold_sigma_multiplier',
           'hold_slip_per_m', 'hold_slip_mps', 'hold_turn_slip_m_per_rad', 'hold_turn_scale_error', 'hold_slow_ratio')


class ObsoleteHoldBudget(unittest.TestCase):

    def test_removed_fields_are_gone_and_dropped_from_car_file(self):
        fields = FusionConfig.__dataclass_fields__
        for key in REMOVED:
            with self.subTest(key=key):
                self.assertNotIn(key, fields)
                self.assertIn(key, line_node.OBSOLETE_FUSION_KEYS)
        self.assertIn('hold_watch_s', fields)
        self.assertIn('hold_wheel_scale_error', fields)

    def test_old_car_file_still_loads(self):
        old = dict(hold_budget_enabled=True, hold_lateral_limit_m=.03, hold_along_limit_m=.1, hold_sigma_multiplier=2.,
                   hold_slip_per_m=.002, hold_slip_mps=.001, hold_turn_slip_m_per_rad=.01, hold_turn_scale_error=.03,
                   hold_slow_ratio=.65)
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'fusion.json')
            with open(path, 'w') as f:
                json.dump(old, f)
            with mock.patch.object(runtime_config, 'PROFILE', DIFF):    # Gen5 差速车
                cfg = line_node.load_fusion_config(gnss_fusion, path)
        self.assertEqual(cfg.hold_watch_s, .5)
        self.assertEqual(cfg.rtk_hold_s, 9.)


if __name__ == '__main__':
    unittest.main()
