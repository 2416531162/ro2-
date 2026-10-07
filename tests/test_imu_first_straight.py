"""直线 IMU 为主、卫星辅助（2026-10-06 用户：「只要和 IMU 推算对不上，直线上也听 IMU 的，靠 imu 走」）。
纯逻辑测试：偏移学习与锁住、卫星轨迹估 β、卫星跳点改按 IMU、闭环路线直线、偏离中止和喷线资格。
当晚又定「直接靠 imu 画线，卫星只看啥时候停水泵」（rtk_trim=False，差速车默认）：见 ImuOnly；
「中途卫星好的时候接入修正」（rtk_fit）：见 ImuFit。"""
from collections import deque
from dataclasses import replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT)]
import paint_quality
from line_cascade import AlignmentConfig, ImuAlignment, ImuFirstConfig, ImuFirstGuide, imu_first_target, wrap
from path_tracker import Path as Route, PathConfig, PathTracker, differential_config
from robot_core.kinematics import ChassisGeometry

DEG = math.pi / 180.0
DT = .02
NX, NY = .23, -.4525
GEO = ChassisGeometry(drive='differential', track_m=.675, diff_min_radius_m=.5)


def guide_on_line(beta_true=0.0, measured=lambda s, e: e, rtk_ok=lambda s: True, metres=12.0, v=.8, config=ImuFirstConfig()):
    """车按 IMU 车头 rel=0 直走，真实车头比 IMU 参考偏 beta_true；返回 (guide, [(路程, 真横偏, 估计)])。"""
    g = ImuFirstGuide(config)
    g.offset, g.offset_n, g.locked = 0.0, 50, True
    e, s, out = 0.0, 0.0, []
    g.update(measured(s, e), 0.0, 0.0, DT, (NX, NY), rtk_ok(s))
    while s < metres:
        d = v * DT
        s += d
        e += d * math.sin(beta_true)
        out.append((s, e, g.update(measured(s, e), 0.0, d, DT, (NX, NY), rtk_ok(s))))
    return g, out


class Guide(unittest.TestCase):
    def test_offset_locks_after_quiet_samples_ignoring_turns_and_heading_jumps(self):
        g = ImuFirstGuide()
        for _ in range(10):
            self.assertTrue(g.learn_offset(.10, 0.))
        self.assertFalse(g.learn_offset(.50, 0.))            # 测向跳了：离平均 > 0.5°
        self.assertFalse(g.learn_offset(.10, 20. * DEG))     # 原地转：不学
        self.assertTrue(g.learn_offset(.10, 8. * DEG))       # 起步车头来回甩（融合航向和 IMU 没有滞后）：照学
        self.assertEqual(g.offset_rejected, 1)
        while not g.locked:
            g.learn_offset(.10 + .001, 0.)
        self.assertEqual(g.offset_n, ImuFirstConfig().lock_samples)
        self.assertAlmostEqual(g.offset, .10 + .001 * 39 / 50, places=9)
        self.assertFalse(g.learn_offset(.2, 0.))             # 锁住以后不再跟卫星测向变

    def test_rtk_track_teaches_heading_error_beta(self):
        g, out = guide_on_line(beta_true=.5 * DEG)
        self.assertAlmostEqual(g.beta, .5 * DEG, delta=.02 * DEG)
        self.assertLess(abs(out[-1][2] - out[-1][1]), .001)
        self.assertEqual(g.mode, 'rtk')

    def test_short_rtk_jump_is_ignored_then_rtk_rejoins(self):
        jump = lambda s, e: e + (.05 if 4. <= s < 6. else 0.)    # noqa: E731
        g, out = guide_on_line(measured=jump)
        during = [abs(est - e) for s, e, est in out if 4. <= s < 6.]
        self.assertLess(max(during), .002)                   # 跳的时候按 IMU 推算，不跟卫星
        self.assertEqual(g.jumps, 1)
        self.assertEqual(g.mode, 'rtk')                      # 卫星回来、对上 1 s 后重新用
        self.assertAlmostEqual(g.beta, 0., delta=.01 * DEG)

    def test_persistent_disagreement_keeps_imu_to_the_end_of_the_line(self):
        g, out = guide_on_line(measured=lambda s, e: e + (.05 if s >= 4. else 0.))
        self.assertEqual(g.mode, 'imu')
        self.assertLess(abs(out[-1][2] - out[-1][1]), .002)
        self.assertGreater(g.imu_only_s, 8.)

    def test_untrusted_rtk_freezes_corrections(self):
        g, _ = guide_on_line(beta_true=.5 * DEG, metres=1.)
        early = g.beta
        g, out = guide_on_line(beta_true=.5 * DEG, rtk_ok=lambda s: s < 1.)
        self.assertAlmostEqual(g.beta, early, delta=.02 * DEG)   # 1 m 以后卫星不可信：β 不再学
        self.assertLess(abs(g.beta), .45 * DEG)
        self.assertEqual(g.mode, 'rtk')                      # 不可信不算跳

    def test_without_rtk_trim_rtk_only_places_the_line_and_beta_is_never_learned(self):
        g = ImuFirstGuide(ImuFirstConfig(rtk_trim=False))
        g.offset, g.offset_n, g.locked = 0.0, 50, True
        e, s = .01, 0.
        g.update(e, 0., 0., DT, (NX, NY), True)
        while s < 3.:                                        # 真车头比 IMU 参考偏 0.5°，卫星照常给
            s += .8 * DT
            e += .8 * DT * math.sin(.5 * DEG)
            g.update(e, 0., .8 * DT, DT, (NX, NY), True)
        self.assertEqual(g.beta, 0.)                         # 不考虑 IMU 偏移：β 一直 0
        self.assertLess(abs(g.e - e), .005)                  # 卫星可用时（接管前）照常定横偏

    def test_target_pulls_back_outside_deadband_and_limits_rate_after_alignment(self):
        cfg = ImuFirstConfig()
        g = ImuFirstGuide(cfg)
        g.offset, g.locked, g.fast = 0., True, False
        g.e = .001
        target, bias, aligning, aligned = imu_first_target(cfg, g, 0., 0., DT, 3., 1., True)
        self.assertEqual(bias, 0.)                           # 死区内不拉
        g.e = .05
        target, bias, aligning, aligned = imu_first_target(cfg, g, 0., 0., DT, 3., 1., True)
        self.assertAlmostEqual(bias, -cfg.max_bias_deg * DEG)
        self.assertAlmostEqual(target, -cfg.rate_deg_s * DEG * DT)
        self.assertFalse(aligning)

    def test_config_validation(self):
        with self.assertRaises(ValueError):
            ImuFirstConfig(rejoin_m=.03, jump_m=.02)
        with self.assertRaises(ValueError):
            ImuFirstConfig(observer_m=0.)
        with self.assertRaises(ValueError):
            ImuFirstConfig(lock_samples=0)
        with self.assertRaises(ValueError):
            ImuFirstConfig(rtk_trim=1)
        with self.assertRaises(ValueError):
            ImuFirstConfig(rtk_trim=True, rtk_fit=True)
        with self.assertRaises(ValueError):
            ImuFirstConfig(rtk_trim=False, rtk_fit=True, fit_noise_m=0.)
        with self.assertRaises(ValueError):
            replace(differential_config(PathConfig()), straight_imu_first='on')


def closed_loop(law, rtk_jump=lambda t: 0., dual_bias_deg=.4, seconds=40., length=20., alignment=None, t0=0.,
                baseline=None, imu_off=1.7, quality=None, imu_drift_deg_min=0.):
    """延迟 0.36 s 的差速车走 20 m 直线；卫星位置往左跳 rtk_jump(t) m，双天线测向偏 dual_bias_deg。
    law：imu_first 卫星修正开着 / imu_only 只靠 IMU 画线 / imu_fit 差速车默认（卫星好时拟合角度）/ ab 原 A/B。
    返回 (tracker, [(t, 路程, 真喷头横偏)], 结束前最后一拍的直线状态)。"""
    cfg = differential_config(PathConfig(cruise_mps=.8))
    if law == 'ab':
        cfg = replace(cfg, straight_imu_first=None)
    elif law == 'imu_first':
        cfg = replace(cfg, straight_imu_first=replace(cfg.straight_imu_first, rtk_trim=True, rtk_fit=False))
    elif law == 'imu_only':
        cfg = replace(cfg, straight_imu_first=replace(cfg.straight_imu_first, rtk_trim=False, rtk_fit=False))
    tr = PathTracker(Route((0., 0., 0.), [(length, 0.)]), cfg, GEO)
    if alignment is not None:
        tr.share_imu_alignment(alignment)
    tr.set_nozzle(NX, NY)
    x = y = psi = v = w = 0.
    y, psi = .003, .3 * DEG
    sent = deque([(0., 0., 0.)])
    t, rows, hold = t0, [], {}

    def fused():
        return NS(x=x, y=y + rtk_jump(t), yaw=psi + dual_bias_deg * DEG, speed=v, yaw_rate=w,
                  usable_for_control=True, quality='RTK_FIXED', reasons=(), correction_xy=(0., 0.),
                  correction_yaw=0., rtk_trusted=True)
    drift = lambda t: imu_drift_deg_min * DEG * t / 60.                        # noqa: E731
    tr.sync(t, (t, psi + imu_off + drift(t), 0.))
    assert tr.start(fused())
    while t < t0 + seconds and tr.st.state == 'RUNNING':
        tr.sync(t, (t, psi + imu_off + drift(t), w))
        if alignment is not None:
            alignment.feed_imu(t, wrap(psi + imu_off + drift(t)))
        if baseline is not None:
            tr.set_heading_baseline(t, baseline(t - t0))
        if quality is not None:
            tr.set_rtk_quality(t, *quality(t - t0))
        vc, wc = tr.update(fused(), DT)
        if tr.st.state == 'RUNNING':
            hold = dict(tr._straight_status)
        sent.append((t, vc, wc))
        while len(sent) > 1 and sent[1][0] <= t - .36:
            sent.popleft()
        _, v, w = sent[0]
        x += v * math.cos(psi) * DT
        y += v * math.sin(psi) * DT
        psi += w * DT
        t += DT
        rows.append((t, tr.st.s, y + NX * math.sin(psi) + NY * (math.cos(psi) - 1.)))
    return tr, rows, hold


class RouteStraight(unittest.TestCase):
    def test_engages_only_after_the_offset_is_locked(self):
        tr, rows, _ = closed_loop('imu_first', seconds=.9)
        self.assertEqual(tr._straight_weight, 0.)
        self.assertFalse(tr._straight_guide.locked)
        tr, rows, _ = closed_loop('imu_first', seconds=3.)
        self.assertGreater(tr._straight_weight, 0.)
        self.assertEqual(tr._straight_status['control_law'], 'imu_first')

    def test_rtk_lateral_jump_is_ignored_while_old_law_follows_it(self):
        jump = lambda t: .05 if 10. <= t < 13. else 0.    # noqa: E731
        worst = {}
        for law in ('imu_first', 'ab'):
            tr, rows, _ = closed_loop(law, rtk_jump=jump)
            self.assertEqual(tr.st.state, 'DONE')
            worst[law] = max(abs(e) for t, s, e in rows if 8. <= t < 18.)
        self.assertLess(worst['imu_first'], .012)
        self.assertGreater(worst['ab'], .03)

    def test_huge_rtk_jump_does_not_abort_off_line_on_imu_straight(self):
        jump = lambda t: .6 if t >= 10. else 0.            # noqa: E731
        tr, rows, hold = closed_loop('imu_first', rtk_jump=jump)
        self.assertEqual(tr.st.state, 'DONE')
        self.assertEqual(hold['first_mode'], 'imu')
        self.assertLess(max(abs(e) for t, s, e in rows if t >= 8.), .012)
        tr, _, _ = closed_loop('ab', rtk_jump=jump)
        self.assertEqual((tr.st.state, tr.st.reason), ('ABORTED', 'off_line'))

    def test_dual_antenna_heading_bias_is_learned_from_the_track(self):
        tr, rows, hold = closed_loop('imu_first', dual_bias_deg=1.0)
        self.assertEqual(tr.st.state, 'DONE')
        self.assertAlmostEqual(hold['first_beta_deg'], -1.0, delta=.15)
        self.assertLess(max(abs(e) for t, s, e in rows if s >= 3.), .01)


class ImuOnly(unittest.TestCase):
    """rtk_trim、rtk_fit 都关：IMU 接管以后卫星不修横偏、不估 β，只管走到哪（停泵、停车）。"""

    def test_rtk_jumps_after_takeover_change_nothing(self):
        jump = lambda t: .05 if 10. <= t < 13. else (.6 if t >= 16. else 0.)    # noqa: E731
        alignment = ImuAlignment()
        tr, rows, hold = closed_loop('imu_only', rtk_jump=jump, dual_bias_deg=0., alignment=alignment)
        self.assertEqual(tr.st.state, 'DONE')                # 卫星跳 0.6 m 也不中止、照常走到头
        self.assertLess(max(abs(e) for t, s, e in rows if t >= 8.), .012)
        self.assertEqual((hold['first_rtk_trim'], hold['first_beta_deg'], hold['first_source']), (False, 0., 'learned'))
        self.assertAlmostEqual(hold['first_innovation'], .6, delta=.02)   # 卫星看到了 0.6 m，但没拿来修
        self.assertAlmostEqual(alignment.value, 1.7 * -1, delta=.01)      # 学到的换算记下，下一条线基线不对时借

    def test_dual_antenna_bias_is_drawn_as_is(self):
        # 换算用的双天线测向偏 0.4°：IMU 照这个方向画，卫星看得到偏差但不纠（用户定的取舍）
        tr, rows, hold = closed_loop('imu_only', dual_bias_deg=.4)
        self.assertEqual(tr.st.state, 'DONE')
        self.assertEqual(hold['first_beta_deg'], 0.)
        t_end, s_end, e_end = rows[-1]
        self.assertLess(e_end, -.06)                         # 真喷头往右偏出去 ≈ 走过的路 × sin 0.4°
        self.assertLess(abs(hold['first_e']), .005)          # IMU 推算认为在线上
        self.assertAlmostEqual(hold['first_innovation'], e_end, delta=.01)
        tr, rows, _ = closed_loop('imu_first', dual_bias_deg=.4)
        self.assertLess(abs(rows[-1][2]), .01)               # 卫星修正开着时会纠回来


FIT = ImuFirstConfig(rtk_trim=False, rtk_fit=True)


class ImuFit(unittest.TestCase):
    """rtk_fit（差速车默认）：卫星好的时候拟合起点横偏和角度差，IMU 推算照常、线保持直。"""

    def test_default_differential_config_fits_heading_when_rtk_is_good(self):
        first = differential_config(PathConfig()).straight_imu_first
        self.assertEqual((first.rtk_trim, first.rtk_fit), (False, True))

    def test_fit_learns_the_heading_error_from_the_track(self):
        g, out = guide_on_line(beta_true=.5 * DEG, config=FIT)
        self.assertAlmostEqual(g.beta, .5 * DEG, delta=.03 * DEG)
        self.assertLess(abs(out[-1][2] - out[-1][1]), .002)
        self.assertGreater(g.fit_points, 40)
        self.assertLess(g.status()['first_fit_sigma_deg'], .1)      # 角度允许慢慢变：误差不会缩到 0

    def test_fit_ignores_a_short_rtk_jump_and_untrusted_rtk(self):
        jump = lambda s, e: e + (.05 if 4. <= s < 6. else 0.)    # noqa: E731
        g, out = guide_on_line(beta_true=.3 * DEG, measured=jump, config=FIT)
        self.assertEqual(g.jumps, 1)
        self.assertLess(max(abs(est - e) for s, e, est in out if s >= 1.), .004)
        self.assertAlmostEqual(g.beta, .3 * DEG, delta=.05 * DEG)
        g, out = guide_on_line(beta_true=.3 * DEG, rtk_ok=lambda s: s < 1., config=FIT)
        frozen = g.beta
        self.assertEqual(g.fit_points, 3)                    # 卫星不可信以后不再攒
        self.assertNotEqual(frozen, 0.)

    def test_slow_divergence_after_a_frozen_start_is_fitted_not_called_a_jump(self):
        # 10-06 傍晚实车 B→A：掉头后基线冻结前 4 m，换算错 0.9°，基线一好差已 6 cm；原来判成跳点、整条线不用卫星
        g, out = guide_on_line(beta_true=.9 * DEG, rtk_ok=lambda s: s > 4., metres=14., config=FIT)
        self.assertEqual(g.jumps, 0)
        self.assertEqual(g.mode, 'rtk')
        self.assertAlmostEqual(g.beta, .9 * DEG, delta=.08 * DEG)
        # 开环（车不转回去）推算上的修正按距离限速、追在后面；卡尔曼自己跟得上
        self.assertLess(abs(out[-1][2] + g.status()['first_fit_lag'] - out[-1][1]), .004)

    def test_correction_is_rate_limited_so_one_bad_stretch_cannot_yank_the_car(self):
        # 卫星 2 m 内偏 6 cm 又回来（坏地方）：推算每米最多挪 fit_rate_m，角度每米最多 fit_beta_rate_deg
        bad = lambda s, e: e + (.06 * math.sin(math.pi * (s - 6.) / 4.) if 6. <= s < 10. else 0.)   # noqa: E731
        g, out = guide_on_line(measured=bad, metres=16., config=FIT)
        worst = max(abs((b[2] - b[1]) - (a[2] - a[1])) / (b[0] - a[0]) for a, b in zip(out, out[1:]))
        # 跟卫星走：额外的纠正每米最多 1 cm，加上被带偏的角度（< 0.5°）本身带来的横移
        self.assertLessEqual(worst, FIT.fit_rate_m + math.sin(.5 * DEG))
        betas = []
        g = ImuFirstGuide(FIT)
        g.offset, g.offset_n, g.locked = 0.0, 50, True
        g.update(0., 0., 0., DT, (NX, NY), True)
        s = 0.
        while s < 16.:
            s += .8 * DT
            g.update(bad(s, 0.), 0., .8 * DT, DT, (NX, NY), True)
            betas.append(abs(math.degrees(g.beta)))
        self.assertLess(max(betas), .5)                      # 车头最多被带偏零点几度，不会急拐

    def test_rtk_in_a_bad_patch_is_not_followed_when_receiver_quality_says_so(self):
        # 傍晚 100 m 线中间那块：卫星横偏几米内来回十几厘米，接收机 σ 3–8 cm、卫星少——不跟，按 IMU 走直
        # 慢晃（14 s 一个来回，跳点检查挡不住）：10 cm
        wobble = lambda t: .10 * math.sin(2 * math.pi * (t - 8.) / 14.) if 8. <= t < 22. else 0.   # noqa: E731
        bad = lambda t: (.04, 16, .06) if 7.5 <= t < 22.5 else (.015, 28, .005)                    # noqa: E731
        tr, rows, hold = closed_loop('imu_fit', rtk_jump=wobble, dual_bias_deg=0., quality=bad)
        self.assertEqual(tr.st.state, 'DONE')
        self.assertLess(max(abs(e) for t, s, e in rows if t >= 3.), .01)
        tr, rows, _ = closed_loop('imu_fit', rtk_jump=wobble, dual_bias_deg=0.)   # 不看质量：会被带着走
        self.assertGreater(max(abs(e) for t, s, e in rows if t >= 3.), .05)

    def test_a_sudden_persistent_step_is_accepted_after_it_settles_and_followed_slowly(self):
        # 卫星跳 4 cm 不回来、新位置稳定：3 s 后认它，推算每米最多挪 1 cm 挪过去（用户：喷线偏了卫星轨迹就得纠）
        g, out = guide_on_line(beta_true=.3 * DEG, measured=lambda s, e: e + (.04 if s >= 6. else 0.), metres=20., config=FIT)
        self.assertEqual((g.jumps, g.jumps_accepted, g.mode), (1, 1, 'rtk'))
        self.assertLess(abs(out[-1][2] - (out[-1][1] + .04)), .005)
        worst = max(abs((b[2] - b[1]) - (a[2] - a[1])) / (b[0] - a[0]) for a, b in zip(out, out[1:]))
        self.assertLessEqual(worst, FIT.fit_rate_m + math.sin(.5 * DEG))   # 不是一下跳过去

    def test_a_huge_persistent_step_is_not_followed(self):
        g, out = guide_on_line(beta_true=.3 * DEG, measured=lambda s, e: e + (.6 if s >= 6. else 0.), metres=20., config=FIT)
        self.assertEqual((g.jumps, g.jumps_accepted, g.mode), (1, 0, 'imu'))   # 一下跳 60 cm：整周解错了，按 IMU 走
        self.assertLess(abs(out[-1][2] - out[-1][1]), .006)

    def test_closed_loop_fit_corrects_dual_antenna_bias_and_ignores_jumps(self):
        tr, rows, hold = closed_loop('imu_fit', dual_bias_deg=.4)
        self.assertEqual(tr.st.state, 'DONE')
        self.assertAlmostEqual(hold['first_beta_deg'], -.4, delta=.08)
        self.assertLess(abs(rows[-1][2]), .01)
        jump = lambda t: .05 if 10. <= t < 12. else (.6 if t >= 16. else 0.)    # noqa: E731
        tr, rows, hold = closed_loop('imu_fit', rtk_jump=jump, dual_bias_deg=.4)
        self.assertEqual(tr.st.state, 'DONE')
        self.assertLess(max(abs(e) for t, s, e in rows if t >= 8.), .015)


class BaselineGate(unittest.TestCase):
    """双天线基线出界冻结卫星修正：连续 baseline_bad_frames 帧才冻（零星一两帧不冻），同一帧重复传只数一次。"""

    def tracker(self):
        tr = PathTracker(Route((0., 0., 0.), [(20., 0.)]), differential_config(PathConfig()), GEO)
        tr.sync(0., (0., 0., 0.))
        return tr

    def test_isolated_bad_frames_do_not_freeze(self):
        tr = self.tracker()
        for i, length in enumerate((.41, .397, .397, .41, .425, .41, .399, .43, .41)):
            for _ in range(3):                               # 任务层每拍都传最近一帧
                tr.set_heading_baseline(i * .13, length)
        self.assertIsNone(tr._baseline_bad_t)
        tr.sync(1.2, (1.2, 0., 0.))
        self.assertTrue(tr._baseline_ok())

    def test_three_consecutive_bad_frames_freeze_for_the_hold_time(self):
        tr = self.tracker()
        for i, length in enumerate((.41, .35, .36, .34)):
            tr.set_heading_baseline(i * .13, length)
        self.assertAlmostEqual(tr._baseline_bad_t, 3 * .13)
        tr.sync(.39 + .5, (.89, 0., 0.))
        self.assertFalse(tr._baseline_ok())
        tr.sync(.39 + 1.1, (1.49, 0., 0.))
        self.assertTrue(tr._baseline_ok())


class QualityGate(unittest.TestCase):
    """接收机定位质量：σ ≤ 2.5 cm、卫星 ≥ 20 颗，连续 2 帧不合格停用，之后连续合格 2 s 再用；两天线位置距离误差 ≤ 5 cm，
    连续 2 帧超限停用，0.5 s 后再用（10-07 从 3 cm / 2 s 放宽），两类分开数。"""

    def tracker(self):
        tr = PathTracker(Route((0., 0., 0.), [(20., 0.)]), differential_config(PathConfig()), GEO)
        tr.sync(0., (0., 0., 0.))
        return tr

    def test_one_bad_frame_is_ignored_two_stop_the_correction(self):
        tr = self.tracker()
        for i, q in enumerate([(.015, 28, .004), (.04, 28, .004), (.015, 28, .004)]):
            for _ in range(3):                               # 同一帧重复传只数一次
                tr.set_rtk_quality(i * .1, *q)
        self.assertIsNone(tr._quality_bad_t)
        tr.set_rtk_quality(.3, .015, 19, .004)               # 卫星少
        tr.set_rtk_quality(.4, .03, 28, .004)                # σ 大
        self.assertAlmostEqual(tr._quality_bad_t, .4)
        tr.sync(1.5, (1.5, 0., 0.))
        self.assertFalse(tr._rtk_quality_ok())
        tr.sync(2.5, (2.5, 0., 0.))
        self.assertTrue(tr._rtk_quality_ok())

    def test_pair_error_below_5cm_keeps_the_correction(self):
        """10-07 上午：σ 1.8–1.9 cm、22–23 颗，两天线距离误差零星 3.3–3.9 cm，原来 3 cm 一超就停、再等 2 s，5 m 没用卫星。"""
        tr = self.tracker()
        for i, pair in enumerate((.034, .039, .036, .05, .037)):
            tr.set_rtk_quality(i * .05, .019, 23, pair)
        self.assertIsNone(tr._pair_bad_t)
        tr.sync(.3, (.3, 0., 0.))
        self.assertTrue(tr._rtk_quality_ok())

    def test_pair_error_over_5cm_stops_for_half_a_second(self):
        tr = self.tracker()
        tr.set_rtk_quality(.30, .019, 23, .055)
        self.assertIsNone(tr._pair_bad_t)                    # 一帧不算
        tr.set_rtk_quality(.35, .019, 23, .06)
        self.assertAlmostEqual(tr._pair_bad_t, .35)
        self.assertIsNone(tr._quality_bad_t)
        tr.sync(.8, (.8, 0., 0.))
        self.assertFalse(tr._rtk_quality_ok())
        tr.sync(.9, (.9, 0., 0.))
        self.assertTrue(tr._rtk_quality_ok())

    def test_pair_and_receiver_frames_are_counted_separately(self):
        tr = self.tracker()
        tr.set_rtk_quality(.0, .015, 19, .004)               # 卫星少
        tr.set_rtk_quality(.05, .015, 28, .06)               # 两天线位置对不上
        tr.set_rtk_quality(.10, .015, 19, .004)
        self.assertIsNone(tr._quality_bad_t)
        self.assertIsNone(tr._pair_bad_t)

    def test_mission_passes_receiver_quality_to_the_tracker(self):
        import line_mission
        src = __import__('inspect').getsource(line_mission.LineMission.on_gnss)
        self.assertIn('position_baseline', src)
        self.assertIn('_rtk_quality', src)


class LongLineDrift(unittest.TestCase):
    """10-06 傍晚 eaf2d772 的 101 m 线：IMU 换算一路漂（约 −0.37°/min），拟合攒满点以后不再改，App 上喷线（卫星）偏到
    14 cm 车也不纠。现在角度和修正都跟着卫星慢慢走。"""

    def test_imu_drift_on_a_long_line_is_followed(self):
        g = ImuFirstGuide(FIT)
        g.offset, g.offset_n, g.locked = 0.0, 50, True
        g.update(0., 0., 0., DT, (NX, NY), True)
        s = e = 0.
        drift = -.4 * DEG / 60 / .8                          # 0.4°/min，0.8 m/s：每米多偏这么多
        worst = 0.
        while s < 100.:
            d = .8 * DT
            s += d
            e += d * math.sin(drift * s)                      # 车按 IMU 走直，真实车头一路漂
            est = g.update(e, 0., d, DT, (NX, NY), True)
            if s > 20.:
                worst = max(worst, abs(est - e))
        self.assertLess(worst, .02)                          # 推算一直贴着卫星（开环：车没往回拉）
        self.assertAlmostEqual(math.degrees(g.beta), math.degrees(drift * 100.), delta=.12)


class CarryAcrossLines(unittest.TestCase):
    """上一条线卫星拟合好的换算按时间、转角校准带过来，和开头双天线测的加权（10-06 傍晚：掉头后双天线只攒到几个样本，
    开头换算错 0.45–0.86°）。"""

    def lines(self, d_deg_min, k, n=12, sigma_deg=.02, dt_min=1.2, turn=math.pi, seed=1):
        rng = __import__('random').Random(seed)
        al, t, yaw, c = ImuAlignment(AlignmentConfig(drift_deg_min=0., scale=0.)), 0., 0., -.25
        errs = []
        al.feed_imu(t, yaw)
        for i in range(n):
            tok = al.begin_line()
            pred = al.predict(t)
            if pred is not None:
                errs.append(math.degrees(abs(wrap(pred[0] - c))))
            al.update_line(tok, t, c + rng.gauss(0., sigma_deg * DEG), sigma_deg * DEG)
            steps = int(dt_min * 60 / .02)
            rot = turn if i % 2 == 0 else -turn * .2
            for _ in range(steps):                       # 掉头 + 等：IMU 读到的转角含比例误差，c 随时间、转角变
                t += .02
                yaw += rot / steps
                c += d_deg_min * DEG * .02 / 60 - k * rot / steps
                al.feed_imu(t, wrap(yaw))
        return al, errs

    def test_calibration_learns_drift_and_scale(self):
        al, errs = self.lines(-.1, .002)
        self.assertAlmostEqual(math.degrees(al.x[0]), -.1, delta=.04)
        self.assertAlmostEqual(-al.x[1], .002, delta=.0008)  # c 随转角变 −k：IMU 多读了 k
        self.assertLess(max(errs[-4:]), .08)
        self.assertGreater(errs[0], .25)                     # 校准前带过来差得多

    def test_imu_gap_breaks_the_carry(self):
        al = ImuAlignment()
        al.feed_imu(0., 0.)
        tok = al.begin_line()
        al.update_line(tok, 0., .1, .001)
        al.begin_line()
        al.feed_imu(.02, 0.)
        self.assertIsNotNone(al.predict(.02))
        al.feed_imu(5., 0.)                                  # IMU 断了 5 s：转角接不上
        self.assertIsNone(al.predict(5.))

    def test_blend_partial_dual_with_carried(self):
        cfg = AlignmentConfig()
        g = ImuFirstGuide(FIT)
        g.offset, g.offset_n, g.locked, g.source = .8 * DEG, 12, True, 'partial'
        g.blend_start((0., .1 * DEG), cfg)
        self.assertLess(abs(math.degrees(g.offset)), .05)    # 双天线只攒 12 个：几乎全听带过来的
        self.assertGreater(g.start['weight'], .9)
        g = ImuFirstGuide(FIT)
        g.offset, g.offset_n, g.locked, g.source = .8 * DEG, 50, True, 'learned'
        g.blend_start((-1.2 * DEG, .1 * DEG), cfg)           # 差 2°：IMU 重启或测向坏，不混
        self.assertAlmostEqual(math.degrees(g.offset), .8, places=9)
        self.assertEqual(g.start['weight'], 0.)

    def test_drift_rate_learned_on_one_line_carries_into_a_bad_start_on_the_next(self):
        # IMU 漂 −0.4°/min：第一条线卫星好、学出漂移速度；第二条线开头 15 s 卫星质量差，按带过来的漂移速度推
        bad = lambda t: (.04, 16, .06) if t < 15. else (.015, 28, .005)        # noqa: E731
        worst = {}
        for carry in (True, False):
            al = ImuAlignment(AlignmentConfig(drift_deg_min=0., scale=0.))
            tr, rows, hold = closed_loop('imu_fit', dual_bias_deg=0., alignment=al, length=40., seconds=70., imu_drift_deg_min=-.4)
            self.assertEqual(tr.st.state, 'DONE')
            self.assertLess(abs(math.degrees(tr._straight_guide.fit_rate()[0] if tr._straight_guide.locked else 0.)), 1.)
            t1 = rows[-1][0] + .02
            tr, rows, hold = closed_loop('imu_fit', dual_bias_deg=0., alignment=al if carry else ImuAlignment(), length=40.,
                                         seconds=70., t0=t1, imu_drift_deg_min=-.4, quality=bad)
            self.assertEqual(tr.st.state, 'DONE')
            worst[carry] = max(abs(e) for t, s, e in rows if s >= 1.)
        self.assertLess(worst[True], .015)
        self.assertGreater(worst[False], worst[True] * 1.5)

    def test_second_line_starts_with_the_first_lines_fitted_conversion(self):
        al = ImuAlignment(AlignmentConfig(drift_deg_min=0., scale=0.))
        tr, rows, hold = closed_loop('imu_fit', dual_bias_deg=.0, alignment=al)
        self.assertEqual(tr.st.state, 'DONE')
        # 第二条线：双天线偏 0.9°，开头 2 s 基线出界（只攒到几个样本）——没有带过来的会先往外偏
        bad = lambda t: .36 if t < 2. else .41               # noqa: E731
        tr, rows, hold = closed_loop('imu_fit', dual_bias_deg=.9, alignment=al, t0=rows[-1][0] + .02, baseline=bad)
        self.assertEqual(tr.st.state, 'DONE')
        self.assertGreater(hold['first_start_weight'], .5)
        self.assertLess(max(abs(e) for t, s, e in rows if s >= 1.), .015)
        tr, rows, hold = closed_loop('imu_fit', dual_bias_deg=.9, baseline=bad)
        self.assertGreater(max(abs(e) for t, s, e in rows if s >= 1.), .02)


def runup_start(first=None, wobble_rad_s=.12, seconds=12.):
    """助跑 1.5 m 后喷线的直线（路线作业的样子）：起步车头偏 −2°，前 3 m 车身左右甩（10-06 晚实车助跑陀螺 ±5–10°/s，
    每 0.6 m 一个来回、慢慢收）。返回 (tracker, 全接管时的路程或 None)。"""
    cfg = differential_config(PathConfig(cruise_mps=.8))
    if first is not None:
        cfg = replace(cfg, straight_imu_first=first)
    tr = PathTracker(Route((0., 0., 0.), [(13.5, 0.)]), cfg, GEO)
    tr.set_nozzle(NX, NY)
    tr.constant_straight_speed = True
    tr.runup_speed_mps, tr.runup_distance_m, tr.paint_from_m = .3, 1.5, 1.5
    tr.paint_offset_limit_m = .03
    tr.cfg = replace(tr.cfg, straight_settle_m=min(tr.cfg.straight_settle_m, max(tr.cfg.start_hold_m, .5)))
    x, y, psi, v, w, d = 0., .005, -2. * DEG, 0., 0., 0.
    sent = deque([(0., 0., 0.)])
    t, full = 0., None

    def fused():
        return NS(x=x, y=y, yaw=psi + .3 * DEG, speed=v, yaw_rate=w, usable_for_control=True, quality='RTK_FIXED',
                  reasons=(), correction_xy=(0., 0.), correction_yaw=0., rtk_trusted=True)
    tr.sync(t, (t, psi + 1.7, 0.))
    assert tr.start(fused())
    while t < seconds and tr.st.state == 'RUNNING':
        tr.sync(t, (t, psi + 1.7, w))
        vc, wc = tr.update(fused(), DT)
        if full is None and tr._straight_weight >= 1.:
            full = tr.st.s
        sent.append((t, vc, wc))
        while len(sent) > 1 and sent[1][0] <= t - .36:
            sent.popleft()
        _, v, wcmd = sent[0]
        w = wcmd + wobble_rad_s * math.exp(-d / 3.) * math.sin(2. * math.pi * d / .6) * (v > .05)
        x += v * math.cos(psi) * DT
        y += v * math.sin(psi) * DT
        psi += w * DT
        d += v * DT
        t += DT
    return tr, full


class RunupStart(unittest.TestCase):
    """10-06 晚 eaf2d772 实车：「大多数起步都不直」。助跑起步车身甩 3–5°/s，学偏移原来只要陀螺 < 2°/s 的样本，攒满 50 个
    要到喷线后 0.4–1.4 m，线头 1 m 多由原路径跟踪开、跟着摆（线头 2 m 弯 0.6–1.5 cm，线中 0.1–0.4 cm）。"""

    def test_imu_takes_over_before_the_paint_starts_despite_start_wobble(self):
        tr, full = runup_start()
        self.assertIsNotNone(full)
        self.assertLess(full, tr.paint_from_m)               # 喷头到线头前 IMU 已经全接管
        old = replace(differential_config(PathConfig()).straight_imu_first, gyro_max_deg_s=2.)
        tr, full = runup_start(old)
        self.assertTrue(full is None or full > tr.paint_from_m)   # 原来 2°/s：线头还是原跟踪在开


def paint(guided, rtk_trusted=True, quality='RTK_FIXED', reasons=(), fresh=True, hold_age=0.):
    pose = NS(x=0., y=.05, yaw=0., speed=.8, usable_for_control=True, rtk_trusted=rtk_trusted, quality=quality,
              reasons=reasons, hold_age_s=hold_age, motion_evidence=False)
    m = NS(pose=pose, nozzle=(NX, NY), fusion=NS(config=NS(rtk_hold_s=5.)), differential=True,
           _route_local=lambda: NS(kind='path'))
    tracker = NS(straight_guide_estimate=lambda: guided, _fresh_straight_imu=lambda: (0., 0., 0.) if fresh else None,
                 st=NS(s=5.))
    section = NS(direction=1, path=NS(length=20.), paint_from_m=0., paint_to_m=None, paint_spans=None)
    record = {'route_nozzle': {'forward': NX, 'right': -NY}}
    return paint_quality.status(m, record, True, section, tracker)


class PaintGate(unittest.TestCase):
    def test_imu_estimate_decides_painting_on_imu_straight(self):
        out = paint(.004)                                    # 卫星说偏 5 cm，IMU 推算 4 mm：照常喷
        self.assertTrue(out['painting'])
        self.assertEqual(out['paint_control_source'], 'imu_line')
        self.assertAlmostEqual(out['nozzle_deviation_m'], .004)
        out = paint(.05)
        self.assertEqual(out['paint_blocked'], 'paint_off_line')

    def test_short_rtk_loss_keeps_painting_within_fusion_hold(self):
        out = paint(.004, rtk_trusted=False, quality='RTK_FLOAT', reasons=('rtk_hold',), hold_age=1.)
        self.assertTrue(out['painting'])
        self.assertEqual(out['paint_control_source'], 'imu_line_hold')
        out = paint(.004, rtk_trusted=False, quality='DEAD_RECKONING', reasons=('rtk_fixed_timeout',), hold_age=6.)
        self.assertEqual(out['paint_blocked'], 'paint_position_untrusted')
        out = paint(.004, rtk_trusted=False, fresh=False, hold_age=1.)
        self.assertEqual(out['paint_blocked'], 'paint_position_untrusted')


if __name__ == '__main__':
    unittest.main()
