"""楼下模式出口门槛：卫星横偏是两根天线原始解时 6 → 2.5 cm（11-27 用户「把楼下模式的退出门槛从 6 cm 降到 2.5 cm」，
user_rules.POOR_SKY_EXIT_RAW_E_M）。

29a2c10f 00:44:24 那条整条线判成楼下（这块场地 σ 一直 1.8–1.9 cm），线头段后车自己的原始 RTK 横偏 −3–−5 cm 走了
13.6 m：拉回增益 0、只按 0.08°/m 慢修角度，偏的拉不回来（TDCP 核对 RTK 那段没漂，车是真偏了）。6 cm 是怕楼下两根天线
各偏 2–4 cm 时误退；原始解要两根对得上（≤ 2 cm），这种不进来。用融合位置照旧 6 cm。
仿真同 test_straight_raw_lateral（0.36 s 延迟车模型、两根天线原始解、σ 1.8 cm 进楼下模式）。"""
from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT), str(ROOT / 'tests')]
from line_cascade import ImuFirstConfig, ImuFirstGuide  # noqa: E402
import user_rules as rules  # noqa: E402
from test_straight_raw_lateral import run, worst  # noqa: E402
from test_poor_sky import CFG, DT, NX, NY  # noqa: E402

OLD = dict(poor_sky_exit_raw_e_m=rules.POOR_SKY_EXIT_E_M)     # 改之前：原始解也按 6 cm


def tail(rows, start=28.):
    return max(abs(e) for x, e, _ in rows if x > start)


def exits(rows):
    return {s.get('first_poor_sky_exit') for _, _, s in rows} - {None}


class Rules(unittest.TestCase):
    def test_rules_table(self):
        self.assertEqual(rules.POOR_SKY_EXIT_RAW_E_M, .025)
        self.assertEqual(rules.POOR_SKY_EXIT_E_M, .06)              # 融合位置照旧
        cfg = ImuFirstConfig()
        self.assertEqual(cfg.poor_sky_exit_raw_e_m, rules.POOR_SKY_EXIT_RAW_E_M)
        self.assertEqual(cfg.poor_sky_follow_e_m, rules.POOR_SKY_EXIT_E_M)   # 楼下的位置跟随门槛不动
        with self.assertRaises(ValueError):
            ImuFirstConfig(poor_sky_exit_raw_e_m=0.)


def guide_offset(offset, raw, length=20., at=12.):
    """单独跑 ImuFirstGuide：IMU 直走，卫星横偏 at 米起多出 offset（车真被推出去）。返回退出原因。"""
    g = ImuFirstGuide(CFG)
    g.locked, g.offset, g.source = True, 0., 'learned'
    x, step = 0., .5 * DT
    while x < length:
        g.set_poor_sky(True, False)
        g.update(offset if x >= at else 0., 0., step, DT, (NX, NY), True, fast=x < 2., raw=raw)
        x += step
    return g.poor_sky_exit


class Guide(unittest.TestCase):
    def test_raw_measured_uses_2_5_cm(self):
        self.assertEqual(guide_offset(.03, raw=True), 'offset')
        self.assertIsNone(guide_offset(.02, raw=True))

    def test_fused_measured_still_6_cm(self):
        self.assertIsNone(guide_offset(.03, raw=False))
        self.assertEqual(guide_offset(.08, raw=False), 'offset')


class ClosedLoop(unittest.TestCase):
    def test_real_3_5cm_push_is_pulled_back(self):
        # 车真被推出去 3.5 cm（IMU 看不见）：原来到线尾还偏 3 cm 多，现在退出楼下、拉回来
        old, _ = run(peak=0., first=OLD, push=(10., .035))
        new, _ = run(peak=0., push=(10., .035))
        self.assertFalse(exits(old))
        self.assertGreater(tail(old), .025)
        self.assertIn('offset', exits(new))
        self.assertLess(tail(new), .01)
        head = [r[:2] for r in old if r[0] < CFG.head_m + CFG.head_blend_m]
        self.assertEqual(head, [r[:2] for r in new][:len(head)])          # 线头一样

    def test_below_threshold_is_unchanged(self):
        # 真偏 2 cm（不到 2.5 cm）：和原来一样
        old, _ = run(peak=0., first=OLD, push=(10., .02))
        new, _ = run(peak=0., push=(10., .02))
        self.assertEqual([r[:2] for r in old], [r[:2] for r in new])

    def test_common_mode_rtk_bias_is_unchanged(self):
        # 楼下两根天线一起偏 3 cm（多路径共模，原始解过滤不掉）：角度拟合先把车带过去一些，原始横偏到不了 2.5 cm，和原来一样
        for bias in (lambda x: .03 * max(0., min(1., (x - 10.) / 2., (16. - x) / 2.)),
                     lambda x: .03 * max(0., min(1., (x - 10.) / 2.))):
            old, _ = run(peak=0., first=OLD, rtk_bias=bias)
            new, _ = run(peak=0., rtk_bias=bias)
            self.assertEqual([r[:2] for r in old], [r[:2] for r in new])

    def test_11_24_level_multipath_does_not_exit(self):
        # 每根天线各自 1.2 cm、4 m 的慢偏（11-24 楼下实测那么大）：8 趟都不误退、和原来一样
        # （更差的 1.5 cm 时 8 趟里 3 趟会误退：最大偏差平均 2.31 → 2.03 cm，鼓包平均 1.71 → 2.32 cm，见 user_rules）
        for seed in range(8):
            with self.subTest(seed=seed):
                old, _ = run(peak=0., first=OLD, multipath=(.012, 4.), seed=seed)
                new, _ = run(peak=0., multipath=(.012, 4.), seed=seed)
                self.assertFalse(exits(new))
                self.assertEqual([r[:2] for r in old], [r[:2] for r in new])


if __name__ == '__main__':
    unittest.main()
