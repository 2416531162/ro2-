"""画圆 IMU 为主（PathConfig.arc_imu_first，2026-10-06 晚用户：起点终点交接错开 1–2 cm，选「画圆也 IMU 为主」）。
原弯道控制律不变，只换它看到的位姿：车头 = IMU + 换算，横偏 = IMU 推算 + 卫星慢慢拟合；卫星突然跳开不跟。"""
from collections import deque
from dataclasses import replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT)]
from path_tracker import PathConfig, PathTracker, circle, differential_config
from motion_speeds import PAINT_ARC_SPEED_MPS
from robot_core.kinematics import ChassisGeometry

DEG = math.pi / 180.0
DT = .02
NX, NY = .24, -.4575
GEO = ChassisGeometry(drive='differential', track_m=.675, diff_min_radius_m=.5)
R_AXLE = 2.4


def drive_circle(arc, rtk_jump=lambda t: 0., seconds=None):
    """延迟 0.3 s 的差速车：直线 2 m 接右转整圆（后轴半径 R_AXLE）。卫星位置沿半径往外跳 rtk_jump(t) m。
    返回 (tracker, [(t, 路程, 后轴到圆心距离 − R_AXLE)], 圆上最后一拍的 arc_hold)。"""
    cfg = replace(differential_config(PathConfig(cruise_mps=.5)), arc_imu_first=arc)
    path = circle(R_AXLE, left=False, lead_in=2.0, lead_out=1.0)
    # 画圆固定 0.5；测试期限跟随路线长度和作业速度，另留加减速时间。
    if seconds is None:
        seconds = path.length / PAINT_ARC_SPEED_MPS + 15.
    tr = PathTracker(path, cfg, GEO)
    tr.set_nozzle(NX, NY)
    cx, cy = 2.0, -R_AXLE
    x = y = psi = v = w = 0.
    sent = deque([(0., 0., 0.)])
    t, rows, hold = 0., [], {}

    def fused():
        dx, dy = x - cx, y - cy
        r = math.hypot(dx, dy) or 1.
        j = rtk_jump(t) if x > cx - 1e-9 or y < -.5 else 0.
        return NS(x=x + j * dx / r, y=y + j * dy / r, yaw=psi + .3 * DEG, speed=v, yaw_rate=w, usable_for_control=True,
                  quality='RTK_FIXED', reasons=(), correction_xy=(0., 0.), correction_yaw=0., rtk_trusted=True)
    tr.sync(t, (t, psi + 1.7, 0.))
    assert tr.start(fused())
    while t < seconds and tr.st.state == 'RUNNING':
        tr.sync(t, (t, psi + 1.7, w))
        tr.set_heading_baseline(t, .412)
        vc, wc = tr.update(fused(), DT)
        if tr._arc_status.get('active'):
            hold = dict(tr._arc_status)
        sent.append((t, vc, wc))
        while len(sent) > 1 and sent[1][0] <= t - .3:
            sent.popleft()
        _, v, w = sent[0]
        x += v * math.cos(psi) * DT
        y += v * math.sin(psi) * DT
        psi += w * DT
        t += DT
        rows.append((t, tr.st.s, math.hypot(x - cx, y - cy) - R_AXLE))
    return tr, rows, hold


class ArcImuFirst(unittest.TestCase):
    def test_circle_is_driven_by_imu_after_learning_the_offset(self):
        tr, rows, hold = drive_circle(True)
        self.assertEqual(tr.st.state, 'DONE')
        self.assertTrue(hold.get('active'))
        self.assertGreaterEqual(hold['weight'], 1.)
        on = [abs(r) for t, s, r in rows if 4. <= s <= 2. + 2 * math.pi * R_AXLE]
        self.assertLess(max(on), .01)

    def test_rtk_jump_on_the_circle_is_not_followed(self):
        # 10-06 晚实车 05:41 那圈：卫星 0.4 s 内往外跳 3.2 cm（陀螺稳），原控制律跟着拐
        jump = lambda t: .03 if 14. <= t < 16.5 else 0.    # noqa: E731
        worst = {}
        for arc in (True, False):
            tr, rows, _ = drive_circle(arc, rtk_jump=jump)
            self.assertEqual(tr.st.state, 'DONE')
            worst[arc] = max(abs(r) for t, s, r in rows if 12. <= t < 22.)
        self.assertLess(worst[True], .01)
        self.assertGreater(worst[False], .015)

    def test_lines_and_partial_arcs_do_not_use_it(self):
        cfg = differential_config(PathConfig())
        self.assertTrue(cfg.arc_imu_first)
        self.assertFalse(PathConfig().arc_imu_first)


if __name__ == '__main__':
    unittest.main()
