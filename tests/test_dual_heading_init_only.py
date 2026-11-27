"""双天线测向只用来开机定初值和做检查（11-27 用户「测向偏多少不纠结，解决不了怎么处理」→「开始改」，
user_rules.DUAL_HEADING_INIT_ONLY、TRACK_HEADING_*，gnss_fusion._imu_course(track=True)）。

测向的固定差会自己变（同一块场地 12:08 −1.7°、00:43 起约 0°，天线没动过）。车头改成 IMU 航向角 + 按 RTK 位移修的偏移：
走够 TRACK_HEADING_READY_M 后测向不再修车头、只给「测向 − 融合车头」；IMU 断了照旧按测向。
车沿 +x 走（10 Hz），主天线在后轴前 0.235、左 0.205，副天线在它右边 0.41 m；测向报的车头比真车头多 bias。"""
import json
import math
import os
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT), str(ROOT / 'tests')]
import gnss_fusion as gf  # noqa: E402
import user_rules as rules  # noqa: E402
from test_secondary_position import ANT, BASE, config  # noqa: E402

IMU_OFF = 1.0                    # IMU 航向角 = 真车头 + 1 rad（上电时任意）


def cfg(init_only=True, **kw):
    return replace(config(True), dual_heading_init_only=init_only,
                   track_heading_distance_m=rules.TRACK_HEADING_DISTANCE_M,
                   track_heading_ready_m=rules.TRACK_HEADING_READY_M,
                   track_heading_sigma_floor_deg=rules.TRACK_HEADING_SIGMA_FLOOR_DEG,
                   track_heading_switch_sigma_deg=rules.TRACK_HEADING_SWITCH_SIGMA_DEG,
                   dual_heading_check_deg=rules.DUAL_HEADING_CHECK_DEG, **kw)


class Rig:
    def __init__(self, bias_deg=2., **kw):
        self.f = gf.GnssFusion(cfg(**kw))
        self.f.set_origin(31., 120.)
        self.t, self.x, self.y, self.h = 0., 0., 0., 0.
        self.bias = bias_deg

    def step(self, n=1, v=.8, w=0., imu=True, second_off=0., second_every=0):
        out = []
        for i in range(n):
            self.t = t = round(self.t + .1, 6)
            dt = .1
            self.h += w * dt
            self.x += v * math.cos(self.h) * dt
            self.y += v * math.sin(self.h) * dt
            f = self.f
            f.add_wheel(gf.WheelSample(t, v, w))
            if imu:
                f.add_imu(gf.ImuSample(t, w, yaw=self.h + IMU_OFF))
            f.add_heading(gf.DualHeading(t, (180. - math.degrees(self.h) - self.bias) % 360., 0., BASE, 'NARROW_INT', .3, 30))
            c, s = math.cos(self.h), math.sin(self.h)
            ax, ay = ANT
            bx, by = ANT[0], ANT[1] - BASE
            off = second_off if second_every and i % second_every == 0 else 0.
            lat2, lon2 = f.to_geodetic(self.x + c * bx - s * by - s * off, self.y + s * bx + c * by + c * off)
            f.add_fix2(gf.GnssFix(t, lat2, lon2, 0., 'NARROW_INT', .01, .01, .02, 1., 30))
            lat, lon = f.to_geodetic(self.x + c * ax - s * ay, self.y + s * ax + c * ay)
            f.add_fix(gf.GnssFix(t, lat, lon, 0., 'NARROW_INT', .01, .01, .02, 1., 30))
            out.append(f.pose(t))
        return out

    def err_deg(self, pose):
        return math.degrees(math.remainder(pose.yaw - self.h, 2 * math.pi))

    def role(self):
        return self.f.calibration()['dual_heading']


class Rules(unittest.TestCase):
    def test_rules_table_and_car_config(self):
        self.assertIs(rules.DUAL_HEADING_INIT_ONLY, True)
        self.assertFalse(gf.FusionConfig().dual_heading_init_only)          # 单独用融合（仿真、旧车）照旧
        import ros_stubs
        ros_stubs.install()
        for name in ('Imu', 'NavSatFix', 'NavSatStatus'):
            setattr(sys.modules['sensor_msgs.msg'], name, ros_stubs._simple(name))
        import line_node
        import runtime_config
        diff = dict(runtime_config.PROFILE, drive=dict(runtime_config.PROFILE.get('drive') or {}, type='differential'))
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, 'fusion.json')
            json.dump(dict(antenna_xy=[.24, .2075], mount_offset_deg=0.), open(path, 'w'))
            with mock.patch.object(runtime_config, 'PROFILE', diff):
                c = line_node.load_fusion_config(gf, path)
        self.assertIs(c.dual_heading_init_only, rules.DUAL_HEADING_INIT_ONLY)
        self.assertEqual((c.track_heading_distance_m, c.track_heading_ready_m, c.track_heading_sigma_floor_deg,
                          c.track_heading_switch_sigma_deg, c.dual_heading_check_deg),
                         (rules.TRACK_HEADING_DISTANCE_M, rules.TRACK_HEADING_READY_M, rules.TRACK_HEADING_SIGMA_FLOOR_DEG,
                          rules.TRACK_HEADING_SWITCH_SIGMA_DEG, rules.DUAL_HEADING_CHECK_DEG))
        with self.assertRaises(ValueError):
            gf.FusionConfig(dual_heading_init_only=1)
        for name in ('track_heading_distance_m', 'track_heading_ready_m', 'track_heading_sigma_floor_deg'):
            with self.assertRaises(ValueError):
                gf.FusionConfig(**{name: 0.})


class InitOnly(unittest.TestCase):
    def test_biased_dual_no_longer_sets_the_heading(self):
        # 测向多报 2°：原来融合车头跟着偏 2°；现在走过 3 m 后按 IMU + RTK 位移，车头对、测向只给差值
        old, new = Rig(init_only=False), Rig()
        po, pn = old.step(150), new.step(150)
        self.assertGreater(old.err_deg(po[-1]), 1.5)
        self.assertLess(abs(new.err_deg(pn[-1])), .3)
        st = new.role()
        self.assertEqual(st['role'], 'check')
        self.assertGreaterEqual(st['track_aligns'], 2)
        self.assertAlmostEqual(st['minus_fused_deg'], 2., delta=.5)
        self.assertFalse(st['disagree'])

    def test_before_the_first_track_alignment_dual_still_sets_the_heading(self):
        rig = Rig()
        p = rig.step(10)                 # 0.8 m：还没对过
        self.assertEqual(rig.role()['role'], 'steer')
        self.assertAlmostEqual(rig.err_deg(p[-1]), 2., delta=.3)

    def test_unbiased_dual_is_about_the_same(self):
        old, new = Rig(bias_deg=0., init_only=False), Rig(bias_deg=0.)
        po, pn = old.step(150), new.step(150)
        self.assertLess(abs(new.err_deg(pn[-1]) - old.err_deg(po[-1])), .2)
        self.assertLess(abs(new.err_deg(pn[-1])), .2)

    def test_pivot_after_takeover_follows_the_imu(self):
        # 对好后原地转 180°（陀螺记着转角，卫星位移不到窗口一半不算对），再往回走：车头照样对，测向的 2° 不进来
        rig = Rig()
        rig.step(60)
        rig.step(20, v=0., w=math.pi / 2.)        # 2 s × 90°/s = 180°
        p = rig.step(30, v=.8)
        self.assertEqual(rig.role()['role'], 'check')
        self.assertLess(abs(rig.err_deg(p[-1])), .3)

    def test_inconsistent_second_antenna_frames_do_not_block_the_alignment(self):
        # 楼下两根天线互检超 2 cm 的帧很多：这些帧不当一段的起点、终点，段照样攒满
        rig = Rig()
        p = rig.step(150, second_off=.04, second_every=2)
        self.assertEqual(rig.role()['role'], 'check')
        self.assertLess(abs(rig.err_deg(p[-1])), .3)

    def test_imu_loss_falls_back_to_the_dual_heading(self):
        rig = Rig()
        rig.step(60)
        self.assertEqual(rig.role()['role'], 'check')
        rig.step(40, imu=False)
        st = rig.role()
        self.assertEqual(st['role'], 'steer')
        self.assertEqual(st['track_m'], 0.)

    def test_large_disagreement_is_flagged(self):
        rig = Rig(bias_deg=4.)
        rig.step(200)
        st = rig.role()
        self.assertEqual(st['role'], 'check')
        self.assertTrue(st['disagree'])
        self.assertLess(abs(rig.err_deg(rig.f.pose(rig.t))), .4)


if __name__ == '__main__':
    unittest.main()
