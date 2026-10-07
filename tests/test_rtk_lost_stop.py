"""没有固定解（浮点、单点、平板中转断流）时 Gen5 按 IMU + 轮速推算接着走、接着喷，不降速；满 8 s 先停喷，再走 1 s 才停车，
恢复后自动继续（2026-10-06 晚用户：「即使浮点都可以靠 imu 继续」「也不降速」；10-07：「8 秒内没有一个可用的卫星坐标就停止，
停喷也有讲究，先停喷 1 秒，再停止」）。"""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT)]
import gnss_fusion as gf
import paint_quality
from line_mission import LineMission
from rtk_hold_budget import DegradedSpeedLimiter

HOLD_S = paint_quality.RTK_LOST_PAINT_STOP_S + paint_quality.RTK_LOST_STOP_DELAY_S      # line_node 给融合设的 rtk_hold_s


def fusion_after_lost_fix(seconds):
    """固定解只在 t=0 给一次，之后 seconds 秒只有浮点；轮速、IMU、测向照常。返回最后的位姿。"""
    f = gf.GnssFusion(gf.FusionConfig(drive_differential=True, yaw_input='angle', rtk_hold_s=HOLD_S,
                                      antenna_xy=(0., 0.), baseline_yaw_body_deg=-90.))
    f.set_origin(31., 120.)
    lat, lon = f.to_geodetic(0., 0.)
    t = 0.
    f.add_wheel(gf.WheelSample(t, 0., 0.)); f.add_imu(gf.ImuSample(t, 0., 0.))
    f.add_heading(gf.DualHeading(t, 180., 0., .41, 'NARROW_INT', .1, 35))
    f.add_fix(gf.GnssFix(t, lat, lon, 0., 'NARROW_INT', .01, .01, .02, 1., 35))
    while t < seconds - 1e-9:
        t = round(t + .1, 6)
        f.add_wheel(gf.WheelSample(t, 0., 0.)); f.add_imu(gf.ImuSample(t, 0., 0.))
        f.add_heading(gf.DualHeading(t, 180., 0., .41, 'NARROW_INT', .1, 35))
        f.add_fix(gf.GnssFix(t, lat, lon, 0., 'NARROW_FLOAT', .2, .2, .3, 1., 35))
    return f.pose(t)


def paint(hold_age, trusted=False):
    pose = NS(x=0., y=0., yaw=0., speed=.8, usable_for_control=True, rtk_trusted=trusted,
              quality='RTK_FIXED' if trusted else 'RTK_FLOAT', reasons=() if trusted else ('rtk_hold',),
              hold_age_s=hold_age, motion_evidence=False)
    m = NS(pose=pose, nozzle=(.24, -.4575), fusion=NS(config=NS(rtk_hold_s=HOLD_S)), differential=True,
           _route_local=lambda: NS(kind='path'))
    tracker = NS(straight_guide_estimate=lambda: .003, _fresh_straight_imu=lambda: (0., 0., 0.), st=NS(s=5.))
    section = NS(direction=1, path=NS(length=20.), paint_from_m=0., paint_to_m=None, paint_spans=None)
    return paint_quality.status(m, {'route_nozzle': {'forward': .24, 'right': .4575}}, True, section, tracker)


class LostFix(unittest.TestCase):
    def test_rule_is_eight_seconds_then_one_more_second(self):
        self.assertEqual((paint_quality.RTK_LOST_PAINT_STOP_S, paint_quality.RTK_LOST_STOP_DELAY_S), (8., 1.))

    def test_keeps_driving_on_imu_until_nine_seconds_then_stops(self):
        p = fusion_after_lost_fix(8.5)
        self.assertTrue(p.usable_for_control)
        self.assertIn('rtk_hold', p.reasons)
        p = fusion_after_lost_fix(9.5)
        self.assertFalse(p.usable_for_control)
        self.assertIn('rtk_fixed_timeout', p.reasons)

    def test_paint_stops_at_eight_seconds_while_still_driving(self):
        self.assertTrue(paint(7.9)['painting'])
        out = paint(8.0)
        self.assertFalse(out['painting'])
        self.assertEqual(out['paint_blocked'], 'paint_position_untrusted')
        self.assertTrue(paint(20., trusted=True)['painting'])        # 卫星可信（双固定）不按这个算


class NoSlowdown(unittest.TestCase):
    def mission(self):
        m = LineMission.__new__(LineMission)
        m.differential, m.task, m.tracker, m.cruise = True, 'line', NS(st=NS(v_cmd=.8)), .8
        m._hold_speed_limit, m._soft_prev = None, None
        m._speed_hold = DegradedSpeedLimiter()
        m.fusion = NS(config=NS(sensor_hold_speed_mps=.3, gnss_hold_speed_mps=.35))
        return m

    def test_lost_fix_does_not_cap_speed_but_sensor_takeover_still_does(self):
        def pose(reasons):
            return NS(usable_for_control=True, reasons=reasons, speed=.8, rtk_trusted=False)
        m = self.mission()
        self.assertIsNone(m._stable_speed_limit(1., pose(('rtk_hold_pending',)), .02))
        self.assertIsNone(m._stable_speed_limit(1.02, pose(('rtk_hold', 'rtk_float')), .02))
        m = self.mission()
        self.assertEqual(m._stable_speed_limit(1., pose(('wheel_hold',)), .02), .3)


if __name__ == '__main__':
    unittest.main()
