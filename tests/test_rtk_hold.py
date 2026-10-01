#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""卫星位置不可信时按推算维持（gnss_fusion rtk_hold_s，Gen5 5 s）、掉信号原地停与双天线位置互检（DualAntennaCheck）。

2026-09-30 19:37 实车：主天线固定解错了 20–100 cm，接收机照报 NARROW_INT；两根天线（车上相距约 0.40 m）的固定解位置之差
对不上测向基线。旧做法离群 2 s 后对齐到错的位置（横偏一下跳 14 cm，车拐过去追，5 s 一次）。用户规则：卫星定位出问题
不要信卫星，立马按 IMU 推算继续走，最多 5 s；超过就停车，定位恢复后自动继续。
2026-10-01 用户：固定解中断（平板中转断流、浮点）同样按 IMU 维持 5 秒（「我不是让 imu 可以允许维持 5 秒吗」）。"""
import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, ROOT)

import gnss_fusion as gf  # noqa: E402

ANT = (0.24, 0.2075)          # 主天线（车体系前、左）
BASE = 0.41                   # 双天线基线：副天线在主天线右边 0.41 m（baseline_yaw_body_deg −90）
GEN5 = dict(drive_differential=True, yaw_input='angle', yaw_scale_sigma=0.03, heading_sync_s=0.02,
            yaw_walk_deg_sqrt_s=0.1, antenna_xy=ANT, rtk_hold_s=5.0)


def simulate(config, T=30.0, v=0.8, yaw_deg=30.0, primary_err=None, secondary_on=lambda t: True,
             fixes_on=lambda t: True, slip=None, marks=None, stall=None, secondary_err=None, heading_len=None):
    """直行 v（先静止 3 s），IMU 航向角、轮速 50 Hz，主天线定位、测向 20 Hz，副天线定位 5 Hz。
    primary_err(t) → (东, 北) 米：加在主天线固定解上（卫星位置错了，副天线照常）；fixes_on(t) 为假时这一帧主天线
    没有固定解；slip=(t0, t1, 米/秒)：这段时间车横着滑（轮子、IMU 看不见，两根天线都看得见）；
    stall=(t0, t1)：平板中转断流，这段时间测到的 GNSS 数据（定位、测向、副天线）都压着，t1 一批到（带原测量时刻）；
    secondary_err(t) → (东, 北) 米：加在副天线固定解上；heading_len(t)：测向报的基线长度（默认 BASE）。
    返回 (融合, 行)：每 0.05 s 一行 (t, 真后轴 x, y, 位姿)。"""
    f = gf.GnssFusion(gf.FusionConfig(**config))
    f.set_origin(30.0, 120.0, 0.0)
    yaw = math.radians(yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    x = y = 0.0
    dt, t, k = 0.004, 0.0, 0
    rows = []
    held = []

    def send(fn, sample):
        if stall and stall[0] <= t < stall[1]:
            held.append((fn, sample))
        else:
            fn(sample)
    while t < T - 1e-9:
        speed = 0.0 if t < 3.0 else v
        x += speed * c * dt
        y += speed * s * dt
        if slip and slip[0] <= t < slip[1]:
            x -= slip[2] * s * dt           # 向左滑
            y += slip[2] * c * dt
        t += dt
        k += 1
        if k % 5 == 0:                      # IMU、轮速 50 Hz
            f.add_wheel(gf.WheelSample(t, speed, 0.0))
            f.add_imu(gf.ImuSample(t, 0.0, 0.0))
        if k % 12 == 0:                     # 主天线、测向 ≈ 20 Hz
            ax, ay = x + c * ANT[0] - s * ANT[1], y + s * ANT[0] + c * ANT[1]
            if primary_err is not None:
                ex, ey = primary_err(t)
                ax, ay = ax + ex, ay + ey
            if fixes_on(t):
                lat, lon = f.to_geodetic(ax, ay)
                send(f.add_fix, gf.GnssFix(t, lat, lon, 0.0, 'NARROW_INT', 0.01, 0.01, 0.02, 1.0, 30))
            heading = (180.0 - math.degrees(yaw)) % 360.0
            send(f.add_heading, gf.DualHeading(t, heading, 0.0, heading_len(t) if heading_len else BASE, 'NARROW_INT', 0.1, 30))
        if k % 50 == 0 and secondary_on(t):  # 副天线 5 Hz：主天线右边 BASE
            bx, by = x + c * ANT[0] - s * (ANT[1] - BASE), y + s * ANT[0] + c * (ANT[1] - BASE)
            if secondary_err is not None:
                ex, ey = secondary_err(t)
                bx, by = bx + ex, by + ey
            lat, lon = f.to_geodetic(bx, by)
            send(f.add_fix2, gf.GnssFix(t, lat, lon, 0.0, 'NARROW_INT', 0.01, 0.01, 0.02, 1.0, 30))
        if held and t >= stall[1]:          # 断流结束：积压的一批按原测量时刻送进去
            for fn, sample in held:
                fn(sample)
            held.clear()
        if k % 12 == 0:
            p = f.pose(t)
            if p is not None:
                rows.append((t, x, y, p))
    return f, rows


def lateral(row, yaw_deg=30.0):
    """融合后轴相对真值的左偏，m。"""
    t, x, y, p = row
    yaw = math.radians(yaw_deg)
    return -(p.x - x) * math.sin(yaw) + (p.y - y) * math.cos(yaw)


def ramp(t0, t1, rate=0.06, yaw_deg=30.0):
    """主天线固定解从 t0 起以 rate 米/秒往右漂，t1 回来（19:37 那样）。"""
    yaw = math.radians(yaw_deg)

    def err(t):
        if not t0 <= t < t1:
            return 0.0, 0.0
        d = rate * (t - t0)
        return d * math.sin(yaw), -d * math.cos(yaw)
    return err


class TestDualAntennaCheck(unittest.TestCase):

    def check(self, dev_at, T=6.0):
        cfg = gf.FusionConfig(**GEN5)
        chk = gf.DualAntennaCheck(cfg)
        out = []
        t = 0.0
        while t < T:
            t = round(t + 0.05, 6)
            chk.add_heading(t, 90.0, BASE, 0.0)                 # 基线朝东
            chk.add_primary(t, 0.0, 0.0)
            if abs(t / 0.2 - round(t / 0.2)) < 1e-6:
                chk.add_secondary(t, BASE, dev_at(t))
            out.append((t, chk.state_at(t)))
        return chk, out

    def test_consistent_antennas_pass(self):
        chk, out = self.check(lambda t: 0.02)
        self.assertEqual({s for t, s in out if t > 0.5}, {'ok'})
        self.assertAlmostEqual(chk.last_dev, 0.02, places=6)

    def test_mismatch_is_flagged_and_recovers_after_one_second_good(self):
        chk, out = self.check(lambda t: 0.25 if 2.0 <= t < 3.0 else 0.01)
        state = dict(out)
        self.assertEqual(state[1.9], 'ok')
        self.assertEqual(state[2.25], 'suspect')                # 下一帧副天线（5 Hz）就判出来
        self.assertEqual(state[3.6], 'suspect')                 # 好了还要保持 1 s
        self.assertEqual(state[4.3], 'ok')
        self.assertEqual(chk.flags, 1)

    def test_between_thresholds_does_not_clear(self):
        chk, out = self.check(lambda t: 0.25 if 2.0 <= t < 2.5 else 0.08)
        self.assertEqual(dict(out)[5.0], 'suspect')             # 8 cm：没到 10 cm 不判，但也不算恢复（< 6 cm 才算）

    def test_tolerance_grows_with_turn_rate(self):
        """2026-10-01 新平板接收机：原地转、小圆时副天线位置晃 10–14 cm（主天线照常）。转得快时门限按
        dual_check_rate_m × 转速放宽，直线上照旧 10 cm。"""
        def run(rate):
            chk = gf.DualAntennaCheck(gf.FusionConfig(**GEN5))
            states, t = [], 0.0
            while t < 4.0:
                t = round(t + 0.05, 6)
                chk.add_heading(t, math.degrees(rate * t) % 360.0, BASE, 0.0)
                chk.add_primary(t, 0.0, 0.0)
                if abs(t / 0.2 - round(t / 0.2)) < 1e-6:
                    az = rate * t                                   # 测向基线方向（北零顺时针）
                    ex, ey = BASE * math.sin(az), BASE * math.cos(az)
                    chk.add_secondary(t, ex + 0.12, ey)             # 副天线比基线偏 12 cm
                states.append(chk.state_at(t))
            return states
        self.assertNotIn('suspect', run(0.8))                   # 原地转 0.8 rad/s：门限约 16 cm
        self.assertIn('suspect', run(0.0))                      # 直线：12 cm 超过 10 cm，连续两次就判

    def test_no_secondary_means_unknown(self):
        cfg = gf.FusionConfig(**GEN5)
        chk = gf.DualAntennaCheck(cfg)
        for i in range(1, 40):
            chk.add_heading(i * 0.05, 90.0, BASE, 0.0)
            chk.add_primary(i * 0.05, 0.0, 0.0)
        self.assertIsNone(chk.state_at(2.0))


class TestRtkHold(unittest.TestCase):

    def test_clean_rtk_never_holds(self):
        f, rows = simulate(GEN5)
        self.assertTrue(all(r[3].usable_for_control for r in rows if r[0] > 1.0))
        self.assertFalse(any('rtk_hold' in r[3].reasons for r in rows))
        cal = f.calibration()
        self.assertEqual((cal['position_resets'], cal['rtk_rejoins'], cal['dual_antenna']['flags']), (0, 0, 0))

    def test_short_satellite_error_keeps_driving_on_dead_reckoning(self):
        """主天线漂 3 s（6 cm/s，最多 18 cm）：判出来就不用 RTK，按推算照常走、不停车、不重置；
        横偏跟真值差不到 3 cm；卫星恢复后接回。"""
        f, rows = simulate(GEN5, primary_err=ramp(10.0, 13.0))
        for t, x, y, p in rows:
            if t > 1.0:
                self.assertTrue(p.usable_for_control, (t, p.reasons))
            self.assertNotIn('position_reset', p.reasons)
        self.assertLess(max(abs(lateral(r)) for r in rows if r[0] > 5.0), 0.03)
        self.assertTrue(any('rtk_hold' in r[3].reasons for r in rows if 10.0 < r[0] < 15.0))
        self.assertTrue(any('rtk_suspect' in r[3].reasons for r in rows))
        self.assertFalse(any('rtk_hold' in r[3].reasons for r in rows if r[0] > 16.0))

    def test_long_satellite_error_stops_after_five_seconds_then_resumes(self):
        f, rows = simulate(GEN5, primary_err=ramp(10.0, 20.0, rate=0.03), T=30.0)
        blocked = [r[0] for r in rows if not r[3].usable_for_control and r[0] > 1.0]
        self.assertTrue(blocked, '超过 5 s 要停车')
        # 互检判出（漂过 10 cm，约 3.3 s 后）+ 0.5 s 后开始维持，维持 5 s
        self.assertGreater(min(blocked), 13.3 + 5.0 - 0.5)
        self.assertIn('rtk_hold_timeout', next(r for r in rows if r[0] == min(blocked))[3].reasons)
        self.assertLess(max(blocked), 22.5)                     # 恢复（再好 1 s）后接回、能开
        self.assertTrue(rows[-1][3].usable_for_control)
        self.assertEqual(f.calibration()['position_resets'], 0)

    def test_short_signal_loss_keeps_driving_on_dead_reckoning(self):
        """固定解中断 5 s 以内（中转断流、浮点）：按 IMU + 轮速推算照常走、不停车，恢复后接回；横偏跟真值差不到 2 cm。"""
        for gap in (0.6, 1.0, 3.0, 4.8):
            f, rows = simulate(GEN5, fixes_on=lambda t, g=gap: not 10.0 <= t < 10.0 + g)
            with self.subTest(gap=gap):
                self.assertTrue(all(r[3].usable_for_control for r in rows if r[0] > 1.0),
                                [(r[0], r[3].reasons) for r in rows if not r[3].usable_for_control and r[0] > 1.0][:3])
                self.assertTrue(any('rtk_hold' in r[3].reasons for r in rows if 10.0 < r[0] < 10.0 + gap))
                self.assertLess(max(abs(lateral(r)) for r in rows if r[0] > 5.0), 0.02)
                self.assertTrue(rows[-1][3].usable_for_control)

    def test_long_signal_loss_stops_after_five_seconds_and_resumes(self):
        f, rows = simulate(GEN5, fixes_on=lambda t: not 10.0 <= t < 17.0)
        blocked = [r[0] for r in rows if not r[3].usable_for_control and r[0] > 1.0]
        self.assertTrue(blocked, '断超过 5 s 要原地停')
        self.assertGreater(min(blocked), 10.0 + 5.0 - 0.1)
        self.assertLess(min(blocked), 10.0 + 5.0 + 0.1)
        self.assertIn('rtk_fixed_timeout', next(r for r in rows if r[0] == min(blocked))[3].reasons)
        self.assertLess(max(blocked), 17.0 + 1.0)                   # 恢复后接回、能开
        self.assertTrue(rows[-1][3].usable_for_control)
        self.assertLess(abs(lateral(rows[-1])), 0.02)

    def test_relay_stall_late_burst_does_not_jump_or_stop(self):
        """平板中转断流（2026-10-01 实车：0.4–0.8 s 常见，07:42 一次 3.85 s）：所有 GNSS 数据压着、结束时一批到。
        5 s 以内照常走；迟到 1 s 内的按测量时刻插回重算，位置不跳；更早的丢掉。"""
        for stall in ((10.0, 10.55), (10.0, 10.8), (10.0, 13.85)):
            f, rows = simulate(GEN5, stall=stall)
            with self.subTest(stall=stall):
                self.assertTrue(all(r[3].usable_for_control for r in rows if r[0] > 1.0),
                                [(r[0], r[3].reasons) for r in rows if not r[3].usable_for_control and r[0] > 1.0][:3])
                self.assertLess(max(abs(lateral(r)) for r in rows if r[0] > 5.0), 0.02)
                after = [r for r in rows if stall[1] <= r[0] < stall[1] + 0.5]
                steps = [math.hypot(b[3].x - a[3].x, b[3].y - a[3].y) - math.hypot(b[1] - a[1], b[2] - a[2])
                         for a, b in zip(after, after[1:])]
                self.assertLess(max(abs(d) for d in steps), 0.02)    # 一批到时没有跳

    def test_signal_loss_during_satellite_error_hold_counts_from_last_good_fix(self):
        """卫星位置错了、正在推算维持，信号又断了：照样维持，从最后一个可信固定解起算满 5 s 才停。"""
        f, rows = simulate(GEN5, primary_err=ramp(10.0, 20.0, rate=0.06), fixes_on=lambda t: not 13.0 <= t < 16.0)
        self.assertTrue(any('rtk_hold' in r[3].reasons for r in rows if 12.0 < r[0] < 13.0))
        self.assertTrue(all(r[3].usable_for_control for r in rows if 12.0 < r[0] < 16.0))
        blocked = [r[0] for r in rows if not r[3].usable_for_control and r[0] > 1.0]
        self.assertTrue(blocked)
        self.assertGreater(min(blocked), 16.0)
        self.assertIn('rtk_hold_timeout', next(r for r in rows if r[0] == min(blocked))[3].reasons)
        # 断流恢复后第一帧（还是错的）不能因为「最近没比过」就当可信对齐过去（原来横偏一下跳 36 cm）
        self.assertLess(max(abs(lateral(r)) for r in rows if r[0] > 5.0), 0.03)
        self.assertTrue(rows[-1][3].usable_for_control)

    def test_primary_dropout_that_comes_back_wrong_is_not_trusted(self):
        """只有主天线断了 2 s（副天线一直在），回来时已经错了 30 cm：先等跟副天线比过，不按它对齐。"""
        yaw = math.radians(30.0)

        def wrong(t):
            return (0.3 * math.sin(yaw), -0.3 * math.cos(yaw)) if t >= 11.0 else (0.0, 0.0)
        f, rows = simulate(GEN5, primary_err=wrong, fixes_on=lambda t: not 10.0 <= t < 12.0, T=20.0)
        self.assertLess(max(abs(lateral(r)) for r in rows if r[0] > 5.0), 0.03)
        self.assertTrue(any('rtk_suspect' in r[3].reasons for r in rows if r[0] > 12.0))
        self.assertTrue(all(r[3].usable_for_control for r in rows if 1.0 < r[0] < 15.0))   # 维持 5 s 内照常走

    def test_dead_reckoning_wrong_with_consistent_rtk_realigns_without_stopping(self):
        """车真的横着滑了 20 cm（两根天线都看得见，互检过）：推算错了，按 RTK 对齐，不停车。"""
        f, rows = simulate(GEN5, slip=(10.0, 12.0, 0.10))
        self.assertTrue(all(r[3].usable_for_control for r in rows if r[0] > 1.0))
        self.assertLess(abs(lateral(rows[-1])), 0.02)
        self.assertEqual(f.calibration()['position_resets'], 0)

    def test_without_secondary_antenna_old_reset_applies(self):
        """没有副天线数据（比不了）：卫星错不错判断不了，主天线持续对不上照旧对齐到 RTK 并暂停（旧做法）。"""
        f, rows = simulate(GEN5, primary_err=ramp(10.0, 30.0, rate=0.06), secondary_on=lambda t: False)
        self.assertGreaterEqual(f.calibration()['position_resets'], 1)
        self.assertTrue(any('position_reset' in r[3].reasons for r in rows))

    def test_old_config_unchanged(self):
        """rtk_hold_s = 0（旧车、默认）：固定解断 0.5 s 就不可控，互检不参与。"""
        cfg = dict(GEN5, rtk_hold_s=0.0)
        f, rows = simulate(cfg, fixes_on=lambda t: not 10.0 <= t < 12.0)
        self.assertTrue(any('rtk_fixed_timeout' in r[3].reasons and not r[3].usable_for_control for r in rows))
        f, rows = simulate(cfg, primary_err=ramp(10.0, 13.0))
        self.assertEqual(f.calibration()['rejected']['position_suspect'], 0)



# 2026-10-01 用户：IMU 航向很稳，加大 IMU 权重、维持 8 s、基线 0.40–0.43 m 外立马信 IMU；卫星质量差时逐步少信卫星、
# 推算维持后平滑接回（见 line_node.load_fusion_config 的 Gen5 设置）
GEN5_NEW = dict(GEN5, rtk_hold_s=8.0, yaw_walk_deg_sqrt_s=0.03, yaw_scale_sigma=0.03,
                heading_length_band_m=(0.40, 0.43), dual_weight_m=0.02, rejoin_rate_mps=0.04)


class TestSatelliteQuality(unittest.TestCase):

    def test_baseline_out_of_band_uses_imu_heading(self):
        f, rows = simulate(GEN5_NEW, heading_len=lambda t: 0.395 if 10.0 <= t < 14.0 else BASE)
        bad = [r for r in rows if 10.2 < r[0] < 13.8]
        self.assertTrue(all('baseline_off' in r[3].reasons and 'heading_unavailable' in r[3].reasons for r in bad))
        self.assertTrue(all(r[3].usable_for_control for r in rows if r[0] > 1.0))
        self.assertFalse(any('baseline_off' in r[3].reasons for r in rows if r[0] > 14.5))
        self.assertGreater(f.calibration()['rejected']['heading_baseline_off'], 50)
        f, rows = simulate(GEN5, heading_len=lambda t: 0.395)              # 不设范围（旧车、默认）：照用
        self.assertFalse(any('baseline_off' in r[3].reasons for r in rows))

    def test_poor_dual_agreement_downweights_rtk_without_stopping(self):
        """两根天线差 4 cm（还不到判卫星错的 10 cm）：主天线一直偏着 4 cm，降了权的融合被拉过去得慢，照常能开。
        刚跳的那一下软门限（soft_gate_sigma，Huber）本来就按离群放大方差，降权后新息不再离群、软门限不再放大，
        头 1 s 只少拉约 20%；偏差一直在、软门限松开以后才是降权起作用的地方（3 s 时少拉约 1/3）。"""
        yaw = math.radians(30.0)
        shift = lambda t: (0.04 * math.sin(yaw), -0.04 * math.cos(yaw)) if t >= 10.0 else (0.0, 0.0)
        pulled = {}
        for name, cfg in (('off', dict(GEN5_NEW, dual_weight_m=0.0)), ('on', GEN5_NEW)):
            f, rows = simulate(cfg, primary_err=shift, T=16.0)
            pulled[name] = [abs(lateral(next(r for r in rows if r[0] >= t))) for t in (11.0, 13.0)]
            if name == 'on':
                self.assertTrue(any('rtk_weighted' in r[3].reasons for r in rows if r[0] > 10.5))
                self.assertTrue(all(r[3].usable_for_control for r in rows if r[0] > 1.0),
                                [(r[0], r[3].reasons) for r in rows if not r[3].usable_for_control and r[0] > 1.0][:3])
            else:
                self.assertFalse(any('rtk_weighted' in r[3].reasons for r in rows))
        self.assertLess(pulled['on'][0], 0.9 * pulled['off'][0], pulled)
        self.assertLess(pulled['on'][1], 0.75 * pulled['off'][1], pulled)

    def test_clean_satellites_are_not_downweighted(self):
        f, rows = simulate(GEN5_NEW, secondary_err=lambda t: (0.008, -0.006))   # 1 cm 互检偏差（空旷处 p95 1.4 cm）
        self.assertFalse(any('rtk_weighted' in r[3].reasons for r in rows))

    def test_rejoin_after_hold_is_smooth(self):
        """维持期间车横着滑了 10 cm（推算看不见）：接回时按每秒 4 cm 挪过去，不一下跳 10 cm，也不停车。"""
        for cfg, smooth in ((GEN5, False), (GEN5_NEW, True)):
            f, rows = simulate(cfg, fixes_on=lambda t: not 10.0 <= t < 13.0, slip=(10.5, 12.5, 0.05), T=20.0)
            lat = [(r[0], lateral(r)) for r in rows if r[0] > 12.9]
            step = max(abs(b[1] - a[1]) for a, b in zip(lat, lat[1:]))
            with self.subTest(smooth=smooth):
                if smooth:
                    self.assertLess(step, 0.01)
                    self.assertTrue(all(r[3].usable_for_control for r in rows if r[0] > 1.0))
                else:
                    self.assertGreater(step, 0.05)
                self.assertLess(abs(lateral(rows[-1])), 0.02)                # 最后都接回到 RTK

    def test_rejoin_with_poor_satellites_does_not_stop(self):
        """接回那会儿卫星质量差（降了权），接回途中卫星又好了：位置不确定度要跟着固定解照常收，不能停车。
        2026-10-01 回放 11:42 靠楼那圈：接回期间不更新不确定度，降权一停就因 position_uncertain 停了 0.35 s。"""
        yaw = math.radians(30.0)
        noisy = lambda t: ((lambda d: (d * math.sin(yaw), -d * math.cos(yaw)))(0.06 * math.sin(2 * math.pi * t / 0.7))
                           if 12.8 <= t < 14.0 else (0.0, 0.0))
        f, rows = simulate(GEN5_NEW, fixes_on=lambda t: not 10.0 <= t < 13.0, slip=(10.5, 12.5, 0.05),
                           primary_err=noisy, T=22.0)
        self.assertTrue(any('rtk_weighted' in r[3].reasons for r in rows if 13.0 < r[0] < 14.5))
        bad = [(round(r[0], 2), round(r[3].sigma_xy, 3), r[3].reasons) for r in rows
               if r[0] > 1.0 and not r[3].usable_for_control]
        self.assertEqual([], bad[:3])
        lat = [lateral(r) for r in rows if r[0] > 12.9]
        self.assertLess(max(abs(b - a) for a, b in zip(lat, lat[1:])), 0.01)     # 也没有一下跳过去
        self.assertLess(abs(lateral(rows[-1])), 0.02)

    def test_rejoin_into_persistent_multipath_does_not_jump(self):
        """接回时 RTK 自己一直在跳（靠楼多路径：这里主天线沿车头 ±3 cm、横向 ±2 cm 来回跳），挪不进 1 cm：到时限
        交回普通更新，不往某一帧有噪声的固定解上一步跳过去，也不停车（2026-10-01 回放 11:42 靠楼那圈：RTK 在
        1.6–6.9 cm 间来回跳，4 s 都没挪进 1 cm）。"""
        yaw = math.radians(30.0)

        def multipath(t):
            if t < 13.0:
                return 0.0, 0.0
            along = 0.03 if int(t / 0.2) % 2 else -0.03
            left = 0.02 if int(t / 0.05) % 2 else -0.02
            return (along * math.cos(yaw) - left * math.sin(yaw), along * math.sin(yaw) + left * math.cos(yaw))
        f, rows = simulate(GEN5_NEW, fixes_on=lambda t: not 10.0 <= t < 13.0, slip=(10.5, 12.5, 0.05),
                           primary_err=multipath, T=30.0)
        self.assertEqual(1, f.calibration()['rtk_rejoins'])                 # 到时限也没一步对齐
        lat = [lateral(r) for r in rows if r[0] > 12.9]
        self.assertLess(max(abs(b - a) for a, b in zip(lat, lat[1:])), 0.01)
        self.assertTrue(all(r[3].usable_for_control for r in rows if r[0] > 1.0))
        self.assertFalse(f._s.blending)

    def test_rejoin_that_cannot_catch_up_aligns_in_one_step(self):
        """推算一直往外跑（这里车一直横滑 6 cm/s，比接回的 4 cm/s 快），慢慢挪永远挪不到：到时限就按旧做法一步对齐，
        不能一直挂在半路、越差越远。"""
        f, rows = simulate(GEN5_NEW, fixes_on=lambda t: not 10.0 <= t < 13.0, slip=(10.5, 40.0, 0.06), T=40.0)
        self.assertGreaterEqual(f.calibration()['rtk_rejoins'], 2)
        self.assertLess(max(abs(lateral(r)) for r in rows if r[0] > 13.0), 0.35)

    def test_large_gap_after_hold_aligns_in_one_step(self):
        """维持期间车横着被挪了 1 m（推车、轮子没电）：差得远不平滑，接回那一帧就对齐，不按 4 cm/s 挪 25 s。"""
        f, rows = simulate(GEN5_NEW, fixes_on=lambda t: not 10.0 <= t < 13.0, slip=(10.5, 12.5, 0.5), T=16.0)
        self.assertGreater(abs(lateral(next(r for r in rows if r[0] >= 12.9))), 0.8)      # 维持期间确实差 1 m
        # 断流回来先跟副天线重新比过一次（约 0.2 s），比过就对齐
        self.assertLess(max(abs(lateral(r)) for r in rows if r[0] >= 13.4), 0.02)
        self.assertFalse(f._s.blending)
        self.assertEqual(1, f.calibration()['rtk_rejoins'])

    def test_leaving_indoor_repeater_aligns_instead_of_crawling(self):
        """室内转发器：两根天线的固定解都在转发器天线那里（这里偏 20 m、10 m），开出门 RTK 位置一下差 22 m：
        持续对不上 2 s 后一步对齐（旧做法），不能按 4 cm/s 挪十几分钟。2026-10-01 13:40 实车：融合位置偏 20–100 m
        十几分钟，车每遍跑在不同的地方。"""
        indoor = lambda t: (20.0, 10.0) if t < 10.0 else (0.0, 0.0)
        f, rows = simulate(GEN5_NEW, primary_err=indoor, secondary_err=indoor, T=20.0)
        gap = lambda r: math.hypot(r[3].x - r[1], r[3].y - r[2])
        self.assertGreater(gap(next(r for r in rows if r[0] >= 9.0)), 20.0)              # 室内：融合在转发器那里
        self.assertLess(max(gap(r) for r in rows if r[0] >= 13.0), 0.05)
        self.assertFalse(f._s.blending)


if __name__ == '__main__':
    unittest.main()
