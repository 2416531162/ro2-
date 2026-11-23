"""精度够的固定解一定采用（2026-11-23 用户「如果精度 ≤2cm，应该都被采用」，user_rules.RTK_ACCEPT_SIGMA_M）。

11-22 185721（草地）：σ ≤ 1.24 cm、35 颗以上、两天线差 ≤ 1.9 cm，测向基线读 0.416–0.435，当时起步放行按 0.40–0.42
一直不过，整条线按「还在起步」处理，卫星纠偏关了整条（采用 0%、拟合点 0），车自己估计 ±2 cm、卫星看到 −5.6 cm。
这里测：基线出了 0.35–0.45、融合不算「卫星可信」、起步放行没过、积分冻结时，σ ≤ 2 cm 的固定解照样进拟合；
σ > 2 cm、卫星 < 17 颗、两天线差 > 5 cm、浮点解 不在这条规则里，照旧按原来的门。"""
import unittest

from test_reverse_runup_handoff import fused, tracker  # 先导入：把 radar_system 加进 sys.path
import user_rules  # noqa: E402


def run(sigma=.012, sats=35, pair=.015, baseline=.46, quality='RTK_FIXED', trusted=False, s0=.1,
        entry_unreleased=False, frozen=True, steps=160):
    tr = tracker()
    tr.st.integral_frozen = frozen
    g = tr._straight_guide
    g.offset, g.locked, g.source = 0., True, 'borrowed'
    if entry_unreleased:                                       # 185721：开喷后接入稳速一直没放行
        tr._entry_speed_active, tr._entry_ramp_released = True, False
    accepted = 0
    for i in range(1, steps):
        t = i * .02
        tr.st.s, tr.st.e, tr.st.heading_err = s0 + .006 * i, .03, 0.
        tr.sync(t, (t, 0., 0.))
        if i % 5 == 0:                                          # 测向 10 Hz
            tr.set_heading_baseline(t, baseline)
        tr.set_rtk_quality(t, sigma, sats, pair)
        p = fused(tr)
        p.quality, p.rtk_trusted = quality, trusted
        tr._straight_first(p, .02, .3, 0., tr.path.pieces[0], 0., 0.)
        accepted += bool(g.rtk_accepted)
    return tr, g, accepted


class RtkPreciseAccept(unittest.TestCase):
    def test_rule_value(self):
        self.assertEqual(user_rules.RTK_ACCEPT_SIGMA_M, .02)

    def test_precise_fix_used_although_baseline_out_of_band(self):
        tr, g, accepted = run()
        self.assertFalse(tr._runup_position_trusted(fused(tr)))  # 原来的起步门不过（基线 0.46、不算卫星可信）
        self.assertGreater(g.fit_points, 0)
        self.assertGreater(accepted, 100)

    def test_precise_fix_used_while_entry_never_released(self):
        tr, g, accepted = run(s0=4.0, entry_unreleased=True, frozen=False)
        self.assertTrue(tr._entry_speed_active and not tr._entry_ramp_released)
        self.assertGreater(g.fit_points, 0)
        self.assertGreater(accepted, 100)

    def test_sigma_at_2cm_is_used(self):
        _, g, _ = run(sigma=.02)
        self.assertGreater(g.fit_points, 0)

    def test_not_forced_outside_the_rule(self):
        for kw in (dict(sigma=.021), dict(sats=16), dict(pair=.06), dict(quality='RTK_FLOAT')):
            _, g, accepted = run(**kw)
            self.assertEqual(g.fit_points, 0, kw)
            self.assertEqual(accepted, 0, kw)

    def test_old_gates_still_apply_above_2cm(self):
        # σ 2.2 cm（≤ 2.5 cm 原规则可用）：基线在带内、卫星可信时照旧采用；基线出界照旧不用
        _, g, _ = run(sigma=.022, baseline=.418, trusted=True)
        self.assertGreater(g.fit_points, 0)
        _, g, _ = run(sigma=.022, baseline=.46, trusted=True)
        self.assertEqual(g.fit_points, 0)


if __name__ == '__main__':
    unittest.main()
