"""固定解一断就靠 IMU + 轮速接着走：从断的那一拍起都可控，不停车、不停喷；最多维持 rtk_hold_s 照旧（2026-10-07 用户：
「不能停车，丝滑切换，也不能停喷」）。原来固定解刚断、还没满 fixed_timeout_s（0.5 s）的那几拍，位置不确定度一过
control_sigma_xy_m 就判不可控 → 跟踪暂停、停车再起步。"""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT)]
import gnss_fusion as gf
import paint_quality

HOLD_S = paint_quality.RTK_LOST_PAINT_STOP_S + paint_quality.RTK_LOST_STOP_DELAY_S
V = 0.8          # 东行 0.8 m/s
LOST_AT = 3.0


def drive(until, control_sigma=0.012, record_from=LOST_AT):
    """固定解 20 Hz 到 LOST_AT，之后只有浮点（测向也浮点）；轮速、IMU 照常。control_sigma 调小，让不确定度在 0.5 s 内
    就过门槛（实车 σ 从约 0.8 cm 起、约 4.5 cm/s 涨，过 3 cm 也在 0.45–0.5 s，正卡边上）。返回 [(t − LOST_AT, pose)]，从 record_from 起每 20 ms。"""
    f = gf.GnssFusion(gf.FusionConfig(drive_differential=True, yaw_input='angle', rtk_hold_s=HOLD_S,
                                      antenna_xy=(0., 0.), baseline_yaw_body_deg=-90., control_sigma_xy_m=control_sigma))
    f.set_origin(31., 120.)
    out, k = [], 0
    while True:
        t = round(k * 0.01, 6)
        if t > until + 1e-9:
            return out
        x = V * t
        f.add_wheel(gf.WheelSample(t, V, 0.))
        f.add_imu(gf.ImuSample(t, 0., 0.))
        if k % 5 == 0:                       # 20 Hz 卫星
            fixed = t < LOST_AT
            lat, lon = f.to_geodetic(x, 0.)
            f.add_heading(gf.DualHeading(t, 180., 0., .41, 'NARROW_INT' if fixed else 'NARROW_FLOAT', .1, 35))
            f.add_fix(gf.GnssFix(t, lat, lon, 0., 'NARROW_INT' if fixed else 'NARROW_FLOAT',
                                 .008 if fixed else .1, .008 if fixed else .1, .02 if fixed else .2, 1., 35))
        if k % 2 == 0 and t >= record_from:
            out.append((t - LOST_AT, f.pose(t)))
        k += 1


class FloatSwitch(unittest.TestCase):
    def test_no_tick_unusable_from_the_moment_fix_is_lost(self):
        poses = drive(LOST_AT + HOLD_S - 0.2)
        early = [p for dt, p in poses if dt < 0.5]
        # 这几拍不确定度确实已经过门槛（测的就是这段）
        self.assertTrue(any('position_uncertain' in p.reasons for p in early))
        bad = [(round(dt, 2), p.reasons) for dt, p in poses if not p.usable_for_control]
        self.assertEqual(bad, [])

    def test_hold_limit_still_stops_after_rtk_hold_s(self):
        dt, last = drive(LOST_AT + HOLD_S + 0.5)[-1]
        self.assertFalse(last.usable_for_control)
        self.assertIn('rtk_fixed_timeout', last.reasons)

    def test_fixed_solution_keeps_normal_limit(self):
        # 固定解一直在：照常可控，不确定度远在 3 cm 门槛下（放宽的只是断了以后，平时不变）
        poses = drive(LOST_AT - 0.05, control_sigma=0.03, record_from=1.0)
        self.assertGreater(len(poses), 90)
        self.assertTrue(all(p.usable_for_control and 'position_uncertain' not in p.reasons for _, p in poses))

if __name__ == '__main__':
    unittest.main()
