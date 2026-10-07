"""10-07 用户：「app 地图显示已经偏离轨迹，但是没有及时纠正」。上午 25 m 线（车时 09:05）三处：
起步换算差 0.9°、拟合按 0.15°/m 用了 6 m 才用上；14–17 m 卫星和推算差 1.1–1.6 cm 持续 3 m 拟合不动；
17.7–22.7 m 两天线距离误差 3.3–3.9 cm 被 3 cm 门挡掉、再等 2 s（门的测试在 test_imu_first_straight.QualityGate）。"""
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT)]
from line_cascade import AlignmentConfig, ImuFirstConfig, ImuFirstGuide

DEG = math.pi / 180.0
DT = .02
NX, NY = .24, -.4575
FIT = ImuFirstConfig(rtk_trim=False, rtk_fit=True)


def run(true_e, metres, v=.8, start=None):
    """车按 IMU 车头 rel=0 直走（开环），卫星横偏 = true_e(路程)；start=(带过来的换算, 误差 rad) 时按开头加权定先验。
    返回 (guide, [(路程, 卫星, 推算, β°)])。"""
    g = ImuFirstGuide(FIT)
    g.offset, g.offset_n, g.locked, g.source = 0.0, 50, True, 'learned'
    if start is not None:
        g.blend_start(start, AlignmentConfig())
    s, out = 0.0, []
    g.update(true_e(s), 0.0, 0.0, DT, (NX, NY), True)
    while s < metres:
        d = v * DT
        s += d
        est = g.update(true_e(s), 0.0, d, DT, (NX, NY), True)
        out.append((s, true_e(s), est, math.degrees(g.beta)))
    return g, out


class PersistentDeviation(unittest.TestCase):
    def test_lasting_offset_is_followed_within_two_metres(self):
        """直走 12 m 拟合收稳以后，卫星显示喷头 0.5 m 内往左挪 1.5 cm、一直在那：2 m 内推算要贴上（原来 3 m 后还差 1 cm 多）。"""
        shift = lambda s: 0.0 if s < 12.0 else .015 * min(1.0, (s - 12.0) / .5)  # noqa: E731
        g, out = run(shift, 16.0)
        gap = [abs(sat - est) for s, sat, est, _ in out if 14.5 <= s <= 16.0]
        self.assertLess(max(gap), .005)
        self.assertGreaterEqual(g.persists, 1)

    def test_wobble_around_the_line_is_not_followed(self):
        """卫星绕线来回晃 ±0.8 cm（周期 3 m / 6 m）不是真偏：不放开，拟合收稳以后推算保持 IMU 那样直
        （前几米本来就跟一点，和原来一样）。"""
        for period in (3.0, 6.0):
            g, out = run(lambda s: .008 * math.sin(2 * math.pi * s / period), 30.0)
            self.assertEqual(g.persists, 0)
            self.assertLess(max(abs(est) for s, _, est, _ in out if s > 8.0), .005)


class StartAngle(unittest.TestCase):
    def test_start_angle_error_is_used_within_three_metres(self):
        """开头换算自认准到 0.2°（双天线和带过来的加权），实际差 0.9°：线头后 3 m 内用上（原来 0.15°/m，3 m 最多 0.45°）。"""
        beta = .9 * DEG
        g, out = run(lambda s: s * math.sin(beta), 6.0, start=(0.0, .25 * DEG))
        self.assertLess(g.start['sigma'], .25 * DEG)
        b3 = next(b for s, _, _, b in out if s >= 3.0)
        self.assertGreater(b3, .7)
        self.assertLess(max(abs(sat - est) for s, sat, est, _ in out), .015)

    def test_prior_is_never_tighter_than_half_a_degree(self):
        g, _ = run(lambda s: 0.0, .1, start=(0.0, .1 * DEG))
        self.assertGreaterEqual(math.sqrt(g._fit['P'][1][1]) / DEG, .49)


class UntrustedStart(unittest.TestCase):
    def run_from(self, trusted):
        """拟合起点那一拍卫星横偏错了 10 cm（多路径区暂停后重开），之后卫星可信、车其实一直在线上直走。返回 β 最大（°）。"""
        g = ImuFirstGuide(FIT)
        g.offset, g.offset_n, g.locked, g.source = 0.0, 50, True, 'learned'
        g.update(.10, 0.0, 0.0, DT, (NX, NY), trusted)
        s, worst = 0.0, 0.0
        while s < 8.0:
            d = .8 * DT
            s += d
            g.update(0.0, 0.0, d, DT, (NX, NY), s > 1.0)
            worst = max(worst, abs(math.degrees(g.beta)))
        return worst

    def test_wrong_untrusted_start_is_not_taken_as_an_angle(self):
        self.assertLess(self.run_from(False), .1)

    def test_a_trusted_start_is_believed(self):
        """对照：同样错 10 cm 但那一拍说可信，起点按 0.5 cm 信，角度会被带歪（说明上面那条测的是这件事）。"""
        self.assertGreater(self.run_from(True), .5)

if __name__ == '__main__':
    unittest.main()
