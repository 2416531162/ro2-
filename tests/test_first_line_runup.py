"""每条直线助跑里按卫星轨迹定角度（ImuFirstGuide.runup_fit）。

2026-11-22 用户「就没有办法直接第一条就很好吗」→「做，先仿真」：原来只给重启后第一条（没有上一条线带来的换算）这样拟合；
11-23 用户「不能靠加长助跑解决问题」：助跑照常 RUN_UP_M。11-27 用户「就不能有什么记忆这种东西，一旦出错就乱套」、
「不允许空跑」：删掉开机第一条空跑和跨线记忆，每条直线都只从这条线开头的融合车头学换算、助跑里都这样拟合：

- 规划：没有空跑段、不记 first_line，助跑照常 RUN_UP_M；
- 拟合（runup_fit）：助跑把推算往卫星拉时不动卡尔曼状态（角度差不被当成位置吃掉）；助跑里角度按 fit_runup_beta_rate_deg 用上；
- 跟踪器：每条直线都开 runup_fit。"""
from dataclasses import replace
import inspect
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT), str(ROOT / 'tests')]
import route_plan as rp
import user_rules as rules
from line_cascade import ImuFirstConfig, ImuFirstGuide

NOZZLE = (.24, -.4575)
MIN_R = .556
LINE = rp.RouteSpec(kind='path', points=((0., 0.), (12., 0.)), closed=False, cruise=.8)
FAR = (5., 4., math.pi / 2)          # 车在线旁边


def paint_section(sections):
    """助跑 + 第一段所在的那一段（规划标成要喷的第一段）。"""
    for sec in sections:
        if getattr(sec, 'field_role', None) == 'paint':
            return sec
    raise AssertionError('没有助跑段')


class NoDryRun(unittest.TestCase):
    """开机第一条不空跑（11-27 用户「不允许空跑」）：规划里没有空跑段、没有 first_line，和别的线一样。"""

    def test_no_learn_section_or_first_line(self):
        self.assertFalse(hasattr(rp, 'LEARN_M'))
        self.assertFalse(hasattr(rules, 'FIRST_LINE_LEARN_M'))
        for fn in (rp.plan_current, rp.plan):
            self.assertNotIn('first_line', inspect.signature(fn).parameters)
        for car in (FAR, (-6., .05, 0.), (14., 1., math.pi)):
            with self.subTest(car=car):
                info = {}
                spec = replace(LINE, entry_space_m=rules.ENTRY_ARC_MIN_SPACE_M)
                sections, _ = rp.plan_current(spec, car, MIN_R, nozzle_x=NOZZLE, info=info)
                self.assertNotIn('learn_m', info)
                self.assertFalse(any(getattr(s, 'learn', False) for s in sections))
                self.assertFalse(any(getattr(s, 'first_line', False) for s in sections))
                self.assertEqual(sum(1 for s in sections if getattr(s, 'field_role', None) == 'paint'), 1)

    def test_runup_stays_run_up_m(self):
        sections, _ = rp.plan_current(LINE, rp.axle_for((-6., 0.), 0., *NOZZLE) + (0.,), MIN_R, nozzle_x=NOZZLE)
        sec = paint_section(sections)
        self.assertAlmostEqual(sec.paint_from_m, sec.path.length - 12. - rp.LINE_RUN_OUT_M, places=6)
        self.assertFalse(hasattr(rp, 'FIRST_RUN_UP_M'))
        car = rp.axle_for((-.8, 0.), 0., *NOZZLE) + (0.,)       # 喷头在线头后 0.8 m、已顺线：照旧倒 1.2 m 到助跑起点
        reverse = [s for s in rp.plan_current(LINE, car, MIN_R, nozzle_x=NOZZLE)[0] if s.direction < 0]
        self.assertEqual(len(reverse), 1)
        self.assertAlmostEqual(reverse[0].path.length, rp.RUN_UP_M - .8, places=6)


def guide(runup_fit=True):
    g = ImuFirstGuide(replace(ImuFirstConfig(), rtk_trim=False, rtk_fit=True))
    g.offset, g.locked, g.source = 0., True, 'learned'
    g.runup_fit = runup_fit
    return g


class RunupFit(unittest.TestCase):
    def drive(self, g, slope_deg, metres, pull, fast=True, step=.01):
        """IMU 推算直走（rel 0），卫星横偏按 slope_deg 拉开：起步换算错了 slope_deg。"""
        u = 0.
        while u < metres - 1e-9:
            u += step
            measured = math.tan(math.radians(slope_deg)) * u
            g.update(measured, 0., step, .02, (0., 0.), True, fast=fast)
            if pull:
                g.pull_to(measured, 1. - math.exp(-step / .3))
        return g

    def test_pull_keeps_angle_information(self):
        g = self.drive(guide(), 1.8, 2.0, pull=True)
        self.assertGreater(math.degrees(g._fit['x'][1]), 1.3)   # 原来每拍把卡尔曼的 c 一起拉，只学到 ~0.6°
        self.assertLess(abs(g.e - math.tan(math.radians(1.8)) * 2.0), .005)   # 推算照样靠到卫星

    def test_runup_applies_angle_fast(self):
        g = self.drive(guide(), 1.8, 2.5, pull=True)
        self.assertGreater(math.degrees(g.beta), 1.2)          # 0.5°/m 时到这里最多 ~0.9°

    def test_painting_rate_unchanged(self):
        g = self.drive(guide(), 1.8, 2.5, pull=False, fast=False)
        self.assertLess(math.degrees(g.beta), 1.0)              # 开喷以后照旧限速（直线度）

    def test_flag_off_is_the_old_fit(self):
        # 不开 runup_fit（单独用 ImuFirstGuide 时的默认；跟踪器的直线每条都开）：原来「有记忆的线」那种拟合，
        # 仿真里第一条最差 3.6 cm，比每条都开差
        g = self.drive(guide(runup_fit=False), 1.8, 2.5, pull=True)
        self.assertLess(math.degrees(g._fit['x'][1]), 1.0)      # 助跑的拉近连拟合一起挪
        self.assertLess(math.degrees(g.beta), 1.0)              # 助跑里也限速
        g = self.drive(guide(runup_fit=False), 1.8, 2.5, pull=False)
        self.assertGreater(math.degrees(g._fit['x'][1]), 1.3)   # 拟合看得出来，
        self.assertLess(math.degrees(g.beta), 1.0)              # 助跑里也照旧按 0.5°/m 用上


class EveryLineFitsInRunup(unittest.TestCase):
    """跟踪器：每条直线都开 runup_fit，换算只从这条线开头的融合车头学（没有上一条线带过来的）。"""

    def test_every_straight_sets_runup_fit(self):
        from test_reverse_runup_handoff import fused, tracker
        for flag in (False, True):
            with self.subTest(preset=flag):
                tr = tracker()
                guide = tr._straight_guide
                guide.runup_fit = flag
                tr.sync(1., (1., 0., 0.))
                tr._straight_first(fused(tr), .02, .3, 0., tr.path.pieces[0], 0., 0.)
                self.assertTrue(guide.runup_fit)
                self.assertFalse(hasattr(tr, 'share_imu_alignment'))
                self.assertFalse(hasattr(tr, 'first_line'))


if __name__ == '__main__':
    unittest.main()
