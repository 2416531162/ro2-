"""助跑 / 开喷前「卫星可信」按卫星可信的基线范围 0.35–0.45（用户 11-21 定的规则），学开头换算按 0.40–0.44
（用户 2026-11-22「学开头换算还是按 0.40–0.44」，原 0.40–0.42）。

2026-11-22：Gen5 测向基线中位 0.418 m、σ 0.0035，23–51 % 的帧在 0.42 以上。原来助跑可信按 0.40–0.42（连 3 帧出界冻 1 s），
助跑里卫星一半时间被挡、拟合攒不出点，起步锁住的角度误差（重启后第一条没有上一条线的记忆，测向误差 1.5°）
一直带进线头（201657 线头 −3.3 cm、全线 4.9 cm）。"""
import unittest

from test_reverse_runup_handoff import fused, tracker


def frames(tr, t0, lengths, dt=.1, quality=True):
    t = t0
    for length in lengths:
        t += dt
        tr.sync(t, (t, 0., 0.))
        tr.set_heading_baseline(t, length)
        if quality:
            tr.set_rtk_quality(t, .01, 28, .01)
    return t


class RunupBaselineBand(unittest.TestCase):
    def test_gen5_baseline_above_042_is_trusted(self):
        tr = tracker()
        p = fused(tr)
        t = frames(tr, 1., [.418, .425, .426, .424, .425])      # 这台车常见：连着几帧 0.42 以上
        self.assertTrue(tr._runup_position_trusted(p))
        self.assertTrue(tr._baseline_ok())                      # 学开头换算按 0.40–0.44：照样学
        t = frames(tr, t, [.445, .446, .447])                   # 0.44 以上：卫星照样可信，开头换算不学
        self.assertTrue(tr._runup_position_trusted(p))
        self.assertFalse(tr._baseline_ok())
        frames(tr, t, [.435, .44])
        self.assertTrue(tr._runup_position_trusted(p))

    def test_current_frame_outside_trusted_band_is_not_trusted(self):
        tr = tracker()
        p = fused(tr)
        t = frames(tr, 1., [.418, .418])
        self.assertTrue(tr._runup_position_trusted(p))
        t = frames(tr, t, [.46])                                 # 当前帧出界：不等去抖
        self.assertFalse(tr._runup_position_trusted(p))
        t = frames(tr, t, [.418])
        self.assertTrue(tr._runup_position_trusted(p))
        t = frames(tr, t, [.34, .33, .34])                       # 连续出界：冻 baseline_hold_s
        t = frames(tr, t, [.418, .418])
        self.assertFalse(tr._runup_position_trusted(p))
        frames(tr, t, [.418] * 11)
        self.assertTrue(tr._runup_position_trusted(p))

    def test_runup_fit_collects_points_with_gen5_baseline(self):
        tr = tracker()
        tr.st.integral_frozen = True
        g = tr._straight_guide
        g.offset, g.locked, g.source = 0., True, 'learned'
        band = [.418, .422, .424, .421, .417, .423, .425, .419]   # 一半以上在 0.42 以上
        for i in range(1, 160):
            t = i * .02
            tr.st.s, tr.st.e, tr.st.heading_err = .1 + .006 * i, .03, 0.
            tr.sync(t, (t, 0., 0.))
            if i % 5 == 0:                                      # 测向 10 Hz
                tr.set_heading_baseline(t, band[(i // 5) % len(band)])
            tr.set_rtk_quality(t, .01, 28, .01)
            tr._straight_first(fused(tr), .02, .3, 0., tr.path.pieces[0], 0., 0.)
        self.assertGreater(g.fit_points, 0)

    def learn(self, length):
        tr = tracker()
        g = tr._straight_guide
        frames(tr, 0., [length] * 3, dt=.1)
        for i in range(1, 120):
            t = 1. + i * .02
            tr.st.s, tr.st.e, tr.st.heading_err = .1 + .003 * i, 0., 0.
            tr.sync(t, (t, 0., 0.))
            if i % 5 == 0:
                tr.set_heading_baseline(t, length)
            tr.set_rtk_quality(t, .01, 28, .01)
            tr._straight_first(fused(tr, e=0.), .02, .3, 0., tr.path.pieces[0], 0., 0.)
        return g

    def test_start_offset_learned_inside_040_044(self):
        for length in (.425, .435, .439):                       # 这台车常见的 0.42 以上：照样学、锁住
            g = self.learn(length)
            self.assertTrue(g.locked, length)
            self.assertEqual(g.source, 'learned')

    def test_start_offset_not_learned_outside_040_044(self):
        for length in (.445, .395):                             # 卫星可信（0.35–0.45）但不在 0.40–0.44：不学
            g = self.learn(length)
            self.assertFalse(g.locked, length)
            self.assertIsNone(g.offset)


if __name__ == '__main__':
    unittest.main()
