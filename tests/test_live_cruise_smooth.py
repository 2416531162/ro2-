"""作业中在线调速平滑（2026-11-22 用户「app 点击顶部的车速……0.5m 到 1m，支持实时变速，但是车端需要平滑变速」）。

喷线中也立即开始变速，巡航值按 S 形曲线过渡：加速度 ≤ LIVE_CRUISE_ACCEL_MPS2、加速度每秒变化 ≤ LIVE_CRUISE_JERK_MPS3，
中途改目标不跳变；连续衔接的下一段接着过渡；画圆/弧线仍是固定圆弧速度。仅运行数学模型，不访问车辆。"""
from pathlib import Path
from dataclasses import replace
import math, sys, unittest
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'radar_system'), str(ROOT), str(ROOT/'tests')]
from test_continuous_field_arcs import GEO, DT, pose
from test_paint_speed_invariant import strict, section, at
import path_tracker as pt
from path_tracker import Path as Route, Section
from motion_speeds import PAINT_ARC_SPEED_MPS

A, J = pt.LIVE_CRUISE_ACCEL_MPS2, pt.LIVE_CRUISE_JERK_MPS3


def run(tr, s, v, ticks, change=None):
    """理想车（照指令走），返回每拍 (路程, 指令速度, 巡航值)；change = {拍号: 新目标}。"""
    rows = []
    for i in range(ticks):
        if change and i in change:
            tr.set_cruise(change[i])
        s += v*DT
        v, _ = tr.update(at(tr, s, v), DT)
        rows.append((s, v, tr.cfg.cruise_mps))
    return rows


def smooth(test, values, key, jerk=J, accel=A):
    acc = [(b-a)/DT for a, b in zip(values, values[1:])]
    test.assertLessEqual(max(abs(x) for x in acc), accel+1e-6, key)                       # 加速度有上限
    test.assertLessEqual(max(abs(b-a) for a, b in zip(acc, acc[1:])), jerk*DT+1e-6, key)   # 加速度连续变化（S 形）


def steady(test, values, key):
    """下发车速再经跟踪器原有的速度平滑，加速度也不超过调速上限（没有速度跳变）。"""
    test.assertLessEqual(max(abs(b-a)/DT for a, b in zip(values, values[1:])), A+.01, key)


def both(test, tr, rows, start, key):
    smooth(test, [start]+[c for _, _, c in rows], key)          # 巡航值严格按 S 形曲线
    steady(test, [start]+[v for _, v, _ in rows], key)


class LiveCruiseSmooth(unittest.TestCase):
    def straight(self, cruise):
        tr = strict([section([(40., 0.)], 1., 38., cruise)]).trackers[0]
        self.assertTrue(tr.start(at(tr, 4., cruise)))
        tr.update(at(tr, 4., cruise), DT)
        return tr

    def test_up_and_down_while_painting_follow_an_s_curve(self):
        for old, new in ((.5, 1.), (1., .5), (.8, .65), (.5, .55)):
            with self.subTest(old=old, new=new):
                tr = self.straight(old)
                rows = run(tr, 4., old, int(8/DT), {0: new})
                speeds = [old]+[v for _, v, _ in rows]
                both(self, tr, rows, old, (old, new))
                self.assertTrue(all(min(old, new)-1e-9 <= v <= max(old, new)+1e-9 for v in speeds))   # 不冲过头
                done = next(i for i, (_, v, _) in enumerate(rows) if abs(v-new) < 1e-9)
                self.assertLess(done*DT, abs(new-old)/A+A/J+.5)                                         # 0.5→1.0 约 3.8 s
                self.assertGreater(done*DT, abs(new-old)/A*.9)                                          # 不是一拍跳过去
                self.assertTrue(all(abs(v-new) < 1e-9 for _, v, _ in rows[done:]))
                self.assertIsNone(tr._cruise_target)
                self.assertTrue(tr.paint_speed_ready)

    def test_retarget_midway_has_no_jump(self):
        tr = self.straight(.5)
        rows = run(tr, 4., .5, int(10/DT), {0: 1., int(1.5/DT): .6, int(4/DT): .9})
        both(self, tr, rows, .5, 'retarget')
        self.assertAlmostEqual(rows[-1][1], .9, 9)

    def test_continuous_next_section_keeps_ramping(self):
        a = section([(6., 0.)], 1., 6., .5); a.continuous_after = True
        b = Section(1, Route(a.car_pose(6.), [(30., 0.)])); b.field_role = 'paint'; b.paint_from_m = 0.; b.paint_to_m = 28.
        b.cruise_mps = .5
        st = strict([a, b])
        s, v = 4., .5
        first = st.trackers[0]; first._hint = s
        self.assertTrue(st.start(pose(s, 0., 0., v)))
        speeds, index = [], []
        for i in range(int(8/DT)):
            if i == int(1./DT):
                for tr in st.trackers:
                    tr.set_cruise(1.)            # 与 live_cruise.change 一样每段都设
            s += v*DT
            v, _ = st.update(pose(s, 0., 0., v), DT)
            speeds.append(v); index.append(st.index)
        self.assertIn(1, index)                  # 过了分界（过分界时还在过渡）
        switch = index.index(1)
        self.assertLess(speeds[switch], 1.-.05)
        steady(self, speeds[int(1./DT)-1:], 'handover')
        self.assertAlmostEqual(speeds[-1], 1., 9)

    def test_arc_stroke_keeps_fixed_arc_speed(self):
        tr = strict([section([(12., -.4)], .5, 11.)]).trackers[0]
        self.assertTrue(tr.start(at(tr, 1., PAINT_ARC_SPEED_MPS)))
        tr.set_cruise(1.)
        for s in (1.2, 3., 6., 9.):
            v, _ = tr.update(at(tr, s, PAINT_ARC_SPEED_MPS), DT)
            self.assertAlmostEqual(v, PAINT_ARC_SPEED_MPS)


if __name__ == '__main__':
    unittest.main()
