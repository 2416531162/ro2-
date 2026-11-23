#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gen5 测向安装偏差 +1.7°（2026-11-22 用户「那就补偿这1.7度」，heading_mount）：

- 车上 fusion.json 写的是没标定过的 0（或没写、没文件）时，融合和 rtk_node 都用 +1.7°；写了别的值照用；
  车上的文件不改。
- 两根天线的连线在车体系是 baseline_yaw_body − 安装偏差：补了 1.7° 以后副天线（右天线）照样和主天线对得上、
  主天线被污染时照样换右天线定位。

车沿 +x 以 0.8 m/s 直走（真车头 0），两根天线的连线比车头的垂线往后转了 1.7°（右天线靠后约 1.2 cm），
接收机报的测向就是这条连线：HEADING = 181.7°。"""
import dataclasses
import json
import math
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
for _name in ('Imu', 'NavSatFix', 'NavSatStatus'):
    setattr(sys.modules['sensor_msgs.msg'], _name, ros_stubs._simple(_name))

import gnss_fusion as gf  # noqa: E402
import heading_mount  # noqa: E402
import line_node  # noqa: E402
import rtk_node  # noqa: E402
import runtime_config  # noqa: E402
from test_secondary_position import ANT, BASE, config, lateral_cm  # noqa: E402

DIFF = dict(runtime_config.PROFILE, drive=dict(runtime_config.PROFILE.get('drive') or {}, type='differential'))
CAR = dict(antenna_xy=[0.24, 0.2075], baseline_yaw_body_deg=-90.0, mount_offset_deg=0.0,
           antenna_height_m=0.53, gnss_hold_speed_mps=0.45, tilt_compensation_enabled=True)   # 车上现在的文件
MOUNT = heading_mount.GEN5_MOUNT_OFFSET_DEG


def write(tmp, data):
    path = os.path.join(tmp, 'fusion.json')
    if data is not None:
        with open(path, 'w', encoding='utf-8') as fh:
            json.dump(data, fh)
    return path


class Rule(unittest.TestCase):
    def test_value(self):
        self.assertEqual(MOUNT, 1.7)
        self.assertEqual(heading_mount.mount_offset(0.0), 1.7)
        self.assertEqual(heading_mount.mount_offset(None), 1.7)
        self.assertEqual(heading_mount.mount_offset(0.5), 0.5)       # 人工标定过的值优先
        self.assertEqual(heading_mount.mount_offset(-1.2), -1.2)


class FusionConfig(unittest.TestCase):
    def load(self, data, profile=DIFF):
        with tempfile.TemporaryDirectory() as tmp:
            path = write(tmp, data)
            with mock.patch.object(runtime_config, 'PROFILE', profile):
                cfg = line_node.load_fusion_config(gf, path)
            after = None if data is None else json.load(open(path, encoding='utf-8'))
        return cfg, after

    def test_car_file_zero_uses_measured(self):
        cfg, after = self.load(CAR)
        self.assertEqual(cfg.mount_offset_deg, 1.7)
        self.assertEqual(cfg.baseline_yaw_body_deg, -90.0)
        self.assertEqual(after, CAR)                                       # 车上的文件不改

    def test_missing_key_or_file_uses_measured(self):
        car = dict(CAR); car.pop('mount_offset_deg')
        self.assertEqual(self.load(car)[0].mount_offset_deg, 1.7)
        self.assertEqual(self.load(None)[0].mount_offset_deg, 1.7)

    def test_other_value_kept(self):
        self.assertEqual(self.load(dict(CAR, mount_offset_deg=0.9))[0].mount_offset_deg, 0.9)

    def test_old_drive_key_in_file_ignored(self):
        # 删阿克曼车前文件里可能写过的 drive_differential（OBSOLETE_FUSION_KEYS）：丢掉，照样 +1.7°
        self.assertEqual(self.load(dict(CAR, drive_differential=False))[0].mount_offset_deg, 1.7)

    def test_fusion_heading_compensated(self):
        # 测向 181.7°（连线往后转了 1.7°）：融合给出的车头是真车头 0
        cfg, _ = self.load(CAR)
        f = gf.GnssFusion(cfg)
        f.set_origin(31., 120.)
        yaw = None
        for i in range(1, 40):
            t = i * .1
            f.add_imu(gf.ImuSample(t, 0., 0.))
            f.add_wheel(gf.WheelSample(t, 0., 0.))
            f.add_heading(gf.DualHeading(t, 181.7, 0., BASE, 'NARROW_INT', .3, 30))
            lat, lon = f.to_geodetic(*cfg.antenna_xy)
            f.add_fix(gf.GnssFix(t, lat, lon, 0., 'NARROW_INT', .01, .01, .02, 1., 30))
            p = f.pose(t)
            yaw = p.yaw if p is not None else yaw
        self.assertIsNotNone(yaw)
        self.assertLess(abs(math.degrees(yaw)), 0.05)


class RtkNodeHeading(unittest.TestCase):
    def load(self, data):
        with tempfile.TemporaryDirectory() as tmp:
            return rtk_node.load_heading_calibration(write(tmp, data))

    def test_same_rule_as_fusion(self):
        self.assertEqual(self.load(CAR), (1.7, -90.0))
        self.assertEqual(self.load(None), (1.7, -90.0))
        self.assertEqual(self.load(dict(CAR, mount_offset_deg=0.9)), (0.9, -90.0))
        self.assertEqual(self.load(dict(CAR, drive_differential=False)), (1.7, -90.0))   # 删掉的旧键不管

    def test_recorded_heading_matches_fusion(self):
        # /gnss/heading 和状态里的车头：测向 181.7° → 真车头 0
        mount, base = self.load(CAR)
        self.assertAlmostEqual(math.degrees(rtk_node.vehicle_yaw(181.7, mount, base)), 0.0, places=9)


class TiltedRig:
    """两根天线连线比车头垂线往后转了 1.7°：主天线在后轴前 0.235、左 0.205，右天线在连线方向 −91.7° 上 0.41 m。"""

    def __init__(self, mount):
        self.f = gf.GnssFusion(dataclasses.replace(config(True), mount_offset_deg=mount))
        self.f.set_origin(31., 120.)
        self.t, self.speed = 0., .8
        b = math.radians(-90. - MOUNT)
        self.ant2 = (ANT[0] + BASE * math.cos(b), ANT[1] + BASE * math.sin(b))
        self.heading = (90. - math.degrees(b)) % 360.                    # 181.7°

    def step(self, n=1, left=(0., 0.), heading=True):
        out = []
        for _ in range(n):
            self.t = t = round(self.t + .1, 6)
            f, x = self.f, self.speed * self.t
            f.add_wheel(gf.WheelSample(t, self.speed, 0.))
            f.add_imu(gf.ImuSample(t, 0., 0.))
            if heading:
                f.add_heading(gf.DualHeading(t, self.heading, 0., BASE, 'NARROW_INT', .3, 30))
            lat2, lon2 = f.to_geodetic(x + self.ant2[0], self.ant2[1])
            f.add_fix2(gf.GnssFix(t, lat2, lon2, 0., 'NARROW_INT', .01, .01, .02, 1., 30))
            lat, lon = f.to_geodetic(x + ANT[0] + left[0], ANT[1] + left[1])
            f.add_fix(gf.GnssFix(t, lat, lon, 0., 'NARROW_INT', .01, .01, .02, 1., 30))
            out.append(f.pose(t))
        return out


class SecondaryWithMount(unittest.TestCase):
    def test_antennas_agree_after_compensation(self):
        rig = TiltedRig(MOUNT)
        poses = rig.step(60)
        self.assertLess(abs(math.degrees(poses[-1].yaw)), 0.1)            # 车头补对了
        self.assertEqual({p.position_antenna for p in poses}, {'main'})
        cal = rig.f.calibration()['secondary_position']
        self.assertGreaterEqual(cal['samples'], 20)
        self.assertLess(max(abs(v) for v in cal['offset_m']), .002)       # 两根没有额外的固定差要学
        self.assertLess(lateral_cm(poses[-10:]), .3)

    def test_left_jump_still_uses_right(self):
        rig = TiltedRig(MOUNT)
        rig.step(60)
        poses = rig.step(40, left=(0., .05), heading=False)
        self.assertGreater(sum(p.position_antenna == 'second' for p in poses) / len(poses), .9)
        self.assertLess(lateral_cm(poses), 1.0)


if __name__ == '__main__':
    unittest.main()
