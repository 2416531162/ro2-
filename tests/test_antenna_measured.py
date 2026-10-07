#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gen5 主天线杠杆臂（2026-10-06 用户：「修复，改车端」）：车上 fusion.json 还是 9-29 手量的 (0.24, 0.2075) 时，
融合用原地转向反推的实测值 (0.235, 0.197)；文件里写了别的值照用；其余标定原样保留，车上的文件不改。"""
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

import gnss_fusion as gf  # noqa: E402
import line_node  # noqa: E402
import runtime_config  # noqa: E402

DIFF = dict(runtime_config.PROFILE, drive=dict(runtime_config.PROFILE.get('drive') or {}, type='differential'))
CAR = dict(antenna_xy=[0.24, 0.2075], baseline_yaw_body_deg=-90.0, mount_offset_deg=0.0,
           antenna_height_m=0.53, gnss_hold_speed_mps=0.45, tilt_compensation_enabled=True)


class AntennaMeasured(unittest.TestCase):
    def load(self, data, profile=DIFF):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, 'fusion.json')
            if data is not None:
                with open(path, 'w', encoding='utf-8') as fh:
                    json.dump(data, fh)
            with mock.patch.object(runtime_config, 'PROFILE', profile):
                cfg = line_node.load_fusion_config(gf, path)
            after = None if data is None else json.load(open(path, encoding='utf-8'))
        return cfg, after

    def test_hand_measured_value_uses_measured_lever(self):
        cfg, after = self.load(CAR)
        self.assertEqual(cfg.antenna_xy, line_node.GEN5_ANTENNA_MEASURED)
        # 其余标定照用，车上的文件不改
        self.assertEqual(cfg.antenna_height_m, 0.53)
        self.assertEqual(cfg.baseline_yaw_body_deg, -90.0)
        self.assertTrue(cfg.tilt_compensation_enabled)
        self.assertEqual(after, CAR)

    def test_other_value_in_file_is_kept(self):
        cfg, _ = self.load(dict(CAR, antenna_xy=[0.25, 0.21]))
        self.assertEqual(cfg.antenna_xy, (0.25, 0.21))

    def test_missing_file_or_key_uses_measured_lever(self):
        self.assertEqual(self.load(None)[0].antenna_xy, line_node.GEN5_ANTENNA_MEASURED)
        car = dict(CAR); car.pop('antenna_xy')
        self.assertEqual(self.load(car)[0].antenna_xy, line_node.GEN5_ANTENNA_MEASURED)

    def test_old_chassis_unchanged(self):
        ack = dict(runtime_config.PROFILE, drive=dict(runtime_config.PROFILE.get('drive') or {}, type='ackermann'))
        self.assertEqual(self.load(CAR, ack)[0].antenna_xy, (0.24, 0.2075))


if __name__ == '__main__':
    unittest.main()
