#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MPPI 控制器的回归测试。

部署目标是 Orin + Torch CUDA,但这里全部跑在 numpy 后端上 —— 两条后端共用
同一份代价函数和同一份运动学,所以这里验的是**算法本身**,不是某块 GPU。
不需要 ROS、CUDA、雷达或实车:

    python3 tests/test_mppi.py

闭环那几条是重点:单点求解正确不代表跟随起来不画龙、不撞门框。
"""

import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "radar_system"))

from mppi_backend import get_backend, torch_available            # noqa: E402
from mppi_controller import (                                    # noqa: E402
    MPPIController, MPPIConfig, DistanceField, FieldConfig,
    rectangle_clearance,
)
from motion_safety import ChassisGeometry, yaw_from_steer        # noqa: E402


def make(**kw):
    cfg = MPPIConfig()
    cfg.samples = 256
    for k, v in kw.items():
        setattr(cfg, k, v)
    return MPPIController(cfg, prefer='numpy')


def circle(cx, cy, r, n=16):
    return [(cx + r * math.cos(2 * math.pi * i / n),
             cy + r * math.sin(2 * math.pi * i / n)) for i in range(n)]


# =============================================================================
# 后端
# =============================================================================

class TestBackend(unittest.TestCase):

    def test_numpy_backend_always_available(self):
        b = get_backend('numpy')
        self.assertEqual(b.kind, 'numpy')

    def test_seed_makes_sampling_reproducible(self):
        """采样式控制器不可复现的话,现场出一次异常根本没法复盘。"""
        a = get_backend('numpy', seed=7)
        b = get_backend('numpy', seed=7)
        self.assertEqual(a.to_numpy(a.randn((4, 3))).tolist(),
                         b.to_numpy(b.randn((4, 3))).tolist())

    def test_reseed_resets_the_stream(self):
        b = get_backend('numpy', seed=1)
        first = b.to_numpy(b.randn((3,))).tolist()
        b.seed(1)
        self.assertEqual(b.to_numpy(b.randn((3,))).tolist(), first)

    def test_asking_for_cuda_without_torch_is_an_error_not_a_silent_downgrade(self):
        if torch_available():
            self.skipTest("本机有 torch,这条测的是没有的情况")
        with self.assertRaises(Exception):
            get_backend('cuda')


# =============================================================================
# 运动学 —— 必须与固件/上位机其余部分是同一个模型
# =============================================================================

class TestKinematics(unittest.TestCase):

    def test_yaw_rate_matches_motion_safety(self):
        """批量版必须和 yaw_from_steer 逐位对得上。

        对不上的后果很隐蔽:MPPI 规划的弧和车实际走的弧不是一条,
        代价函数算得再对也没用,车就是会往旁边偏。
        """
        c = make()
        geo = ChassisGeometry(wheelbase_m=c.cfg.wheelbase_m,
                              track_m=c.cfg.track_m,
                              max_steer_rad=c.cfg.max_steer_rad)
        speeds = [0.0, 0.1, 0.3, 0.55]
        steers = [-0.35, -0.2, -0.05, 0.0, 0.05, 0.2, 0.35]
        b = c.b
        for v in speeds:
            for d in steers:
                got = b.item(c.yaw_rate(b.array([v]), b.array([d])).reshape(-1)[0])
                want = yaw_from_steer(v, d, geo)
                self.assertAlmostEqual(got, want, places=5,
                                       msg=f"v={v} steer={d}")

    def test_zero_steer_is_exactly_straight(self):
        c = make()
        b = c.b
        self.assertEqual(b.item(c.yaw_rate(b.array([0.5]), b.array([0.0])).reshape(-1)[0]),
                         0.0)


# =============================================================================
# 距离场
# =============================================================================

class TestDistanceField(unittest.TestCase):

    def setUp(self):
        self.b = get_backend('numpy')
        self.f = DistanceField(self.b, FieldConfig(resolution_m=0.05))

    def test_matches_brute_force(self):
        pts = [(1.0, 0.0), (2.0, 1.0), (0.5, -0.8)]
        self.f.build(pts)
        for qx, qy in [(1.0, 0.5), (0.0, 0.0), (2.5, -1.0), (1.5, 0.2)]:
            brute = min(math.hypot(qx - px, qy - py) for px, py in pts)
            got = self.f.probe(qx, qy)
            # 最近邻查表的误差上限是半个格子对角线
            self.assertLess(abs(got - brute), 0.05,
                            msg=f"({qx},{qy}) 查到 {got:.3f} 实际 {brute:.3f}")

    def test_empty_is_max_distance_everywhere(self):
        self.f.build([])
        self.assertAlmostEqual(self.f.probe(1.0, 0.0),
                               self.f.cfg.max_distance_m, places=6)

    def test_distance_is_capped(self):
        self.f.build([(4.0, 2.5)])
        self.assertLessEqual(self.f.probe(-1.0, -2.5), self.f.cfg.max_distance_m)

    def test_points_are_deduplicated_and_capped(self):
        dense = [(1.0 + 0.001 * i, 0.0) for i in range(4000)]
        self.f.build(dense)
        self.assertLessEqual(self.f.obstacle_count, self.f.cfg.max_points)
        self.assertGreater(self.f.obstacle_count, 0)

    def test_far_away_points_are_ignored(self):
        self.f.build([(50.0, 50.0)])
        self.assertEqual(self.f.obstacle_count, 0)

    def test_lookup_outside_the_grid_is_clamped_not_crashing(self):
        self.f.build([(1.0, 0.0)])
        for q in [(-99.0, 0.0), (99.0, 0.0), (0.0, -99.0), (0.0, 99.0)]:
            self.assertGreaterEqual(self.f.probe(*q), 0.0)


# =============================================================================
# 精确车体几何
# =============================================================================

class TestRectangleClearance(unittest.TestCase):

    FRONT, REAR, HW = 0.67, 0.18, 0.335

    def clear(self, poses, pts):
        return rectangle_clearance(poses, pts, self.FRONT, self.REAR, self.HW)

    def test_point_straight_ahead(self):
        self.assertAlmostEqual(self.clear([(0, 0, 0)], [(1.67, 0.0)]), 1.0, places=6)

    def test_point_beside(self):
        self.assertAlmostEqual(self.clear([(0, 0, 0)], [(0.0, 0.835)]), 0.5, places=6)

    def test_point_inside_the_body_is_negative(self):
        self.assertLess(self.clear([(0, 0, 0)], [(0.3, 0.0)]), 0.0)

    def test_corner_distance_is_euclidean_not_axis_aligned(self):
        # 点在右前角外侧 (0.3, 0.4) 处
        got = self.clear([(0, 0, 0)], [(0.67 + 0.3, 0.335 + 0.4)])
        self.assertAlmostEqual(got, math.hypot(0.3, 0.4), places=6)

    def test_rotation_is_applied(self):
        """车转 90 度之后,原来在正前方的点应该落到侧面。"""
        pts = [(1.67, 0.0)]
        straight = self.clear([(0, 0, 0)], pts)
        turned = self.clear([(0, 0, math.pi / 2)], pts)
        self.assertNotAlmostEqual(straight, turned, places=3)
        self.assertGreater(turned, straight)

    def test_no_points_is_wide_open(self):
        self.assertGreater(self.clear([(0, 0, 0)], []), 5.0)

    def test_minimum_over_the_whole_trajectory(self):
        poses = [(0, 0, 0), (1.0, 0, 0), (2.0, 0, 0)]
        # 点在 x=2.8:第三个位姿时车头离它只剩 0.13
        self.assertAlmostEqual(self.clear(poses, [(2.8, 0.0)]), 0.13, places=6)


class TestBodyCircles(unittest.TestCase):

    def test_circles_cover_the_rectangle(self):
        cfg = MPPIConfig()
        circles = cfg.body_offsets()
        hw = cfg.footprint_half_width_m + cfg.safety_margin_m
        for i in range(41):
            x = -cfg.footprint_rear_m + i * (cfg.footprint_front_m
                                             + cfg.footprint_rear_m) / 40.0
            for y in (-hw, 0.0, hw):
                covered = any(math.hypot(x - cx, y) <= r + 1e-9
                              for cx, r in circles)
                self.assertTrue(covered, f"车体上的点 ({x:.2f},{y:.2f}) 没被覆盖")

    def test_bulge_is_small_enough_for_a_narrow_door(self):
        """圆覆盖必然鼓出矩形。鼓出量超过门缝余量,车就永远过不了门。"""
        cfg = MPPIConfig()
        hw = cfg.footprint_half_width_m + cfg.safety_margin_m
        bulge = cfg.body_offsets()[0][1] - hw
        self.assertLess(bulge, 0.015, f"鼓出 {bulge*100:.1f}cm,80cm 门过不去")


# =============================================================================
# 跟随点
# =============================================================================

class TestFollowPoints(unittest.TestCase):

    def test_stationary_person_stands_off_along_the_line_of_sight(self):
        c = make()
        gx, gy, _, _ = c.follow_points((3.0, 0.0), (0.0, 0.0), 0.0)
        x0 = c.b.item(gx.reshape(-1)[0])
        # 期望的是**车头**离人 follow_distance,后轴中心要再退一个前悬
        self.assertAlmostEqual(
            x0, 3.0 - c.cfg.follow_distance_m - c.cfg.footprint_front_m, places=5)
        self.assertAlmostEqual(c.b.item(gy.reshape(-1)[0]), 0.0, places=5)

    def test_walking_person_is_followed_from_behind(self):
        c = make()
        gx, gy, px, _ = c.follow_points((3.0, 0.0), (0.8, 0.0), 0.0)
        last = -1
        # 人往前走,跟随点也往前移
        self.assertGreater(c.b.item(gx.reshape(-1)[last]),
                           c.b.item(gx.reshape(-1)[0]))
        # 跟随点始终在人身后
        for t in (0, 5, 10):
            self.assertLess(c.b.item(gx.reshape(-1)[t]),
                            c.b.item(px.reshape(-1)[t]))

    def test_person_prediction_is_capped(self):
        """人不会一直匀速走。外推信太久,拐弯时目标点会飞出去。"""
        c = make(person_predict_cap_s=1.0)
        _, _, px, _ = c.follow_points((2.0, 0.0), (1.0, 0.0), 0.0)
        far = c.b.item(px.reshape(-1)[-1])
        self.assertAlmostEqual(far, 3.0, places=5)

    def test_sideways_walker_is_followed_from_their_side(self):
        c = make()
        gx, gy, _, _ = c.follow_points((2.0, 0.0), (0.0, 0.9), 0.0)
        self.assertLess(c.b.item(gy.reshape(-1)[0]), 0.0)


# =============================================================================
# 单步求解
# =============================================================================

class TestSolve(unittest.TestCase):

    def test_open_space_drives_forward(self):
        c = make()
        sol = None
        for _ in range(6):
            sol = c.solve([], (3.0, 0.0), (0.0, 0.0), 0.0, 0.0)
        self.assertTrue(sol.feasible)
        self.assertGreater(sol.speed, 0.05)

    def test_person_on_the_left_steers_left(self):
        c = make()
        for _ in range(8):
            sol = c.solve([], (2.5, 1.2), (0.0, 0.0), 0.3, 0.0)
        self.assertGreater(sol.steer, 0.02)

    def test_person_on_the_right_steers_right(self):
        c = make()
        for _ in range(8):
            sol = c.solve([], (2.5, -1.2), (0.0, 0.0), 0.3, 0.0)
        self.assertLess(sol.steer, -0.02)

    def test_unusable_scan_is_not_open_space(self):
        """没有障碍点 != 前面确认是空的。雷达不可信时必须当成不可行。"""
        c = make()
        sol = c.solve([], (3.0, 0.0), (0.0, 0.0), 0.3, 0.0, usable=False)
        self.assertFalse(sol.feasible)
        self.assertEqual(sol.speed, 0.0)
        self.assertEqual(sol.reason, 'scan_unusable')

    def test_wall_right_in_front_stops_the_robot(self):
        c = make()
        wall = [(1.1, y / 100.0) for y in range(-80, 81, 5)]
        sol = None
        for _ in range(10):
            sol = c.solve(wall, (3.0, 0.0), (0.0, 0.0), 0.0, 0.0)
        self.assertEqual(sol.speed, 0.0)

    def test_never_commands_reverse(self):
        """本轮 MPPI 不负责倒车。倒车只能由脱困层在测到停稳之后发起。"""
        c = make()
        wall = [(1.0, y / 100.0) for y in range(-60, 61, 5)]
        for _ in range(15):
            sol = c.solve(wall, (3.0, 0.0), (0.0, 0.0), 0.0, 0.0)
            self.assertGreaterEqual(sol.speed, 0.0)

    def test_limits_are_respected(self):
        c = make()
        for i in range(20):
            sol = c.solve([], (4.0, 2.0), (0.5, 0.5), 0.5, 0.2)
            self.assertLessEqual(sol.speed, c.cfg.max_speed_mps + 1e-6)
            self.assertLessEqual(abs(sol.steer), c.cfg.max_steer_rad + 1e-6)

    def test_same_seed_same_answer(self):
        a, b = make(), make()
        for _ in range(5):
            sa = a.solve([], (3.0, 0.5), (0.2, 0.0), 0.2, 0.0)
            sb = b.solve([], (3.0, 0.5), (0.2, 0.0), 0.2, 0.0)
        self.assertAlmostEqual(sa.speed, sb.speed, places=9)
        self.assertAlmostEqual(sa.steer, sb.steer, places=9)

    def test_reports_solve_time_and_obstacle_count(self):
        c = make()
        sol = c.solve(circle(2.0, 0.0, 0.2), (3.5, 0.0), (0.0, 0.0), 0.0, 0.0)
        self.assertGreater(sol.solve_ms, 0.0)
        self.assertGreater(sol.obstacles, 0)

    def test_infeasible_does_not_wedge_the_planner(self):
        """判过一次不可行之后,名义序列必须被拉回零。

        不拉的话下个周期还会拿同一条撞墙的计划去复验,车永久瘫在原地 ——
        这是实现过程中真的踩到过的坑。
        """
        c = make()
        wall = [(0.9, y / 100.0) for y in range(-60, 61, 5)]
        for _ in range(10):
            c.solve(wall, (3.0, 0.0), (0.0, 0.0), 0.0, 0.0)
        # 墙撤走之后必须能重新起步
        sol = None
        for _ in range(10):
            sol = c.solve([], (3.0, 0.0), (0.0, 0.0), 0.0, 0.0)
        self.assertTrue(sol.feasible)
        self.assertGreater(sol.speed, 0.05)


# =============================================================================
# 闭环 —— 单步算对不代表跟随起来不画龙
# =============================================================================

class Walker:
    def __init__(self, waypoints, speed):
        self.wp, self.speed, self.i = list(waypoints), speed, 0
        self.pos, self.vel = list(waypoints[0]), (0.0, 0.0)

    def step(self, dt):
        if self.i >= len(self.wp) - 1:
            self.vel = (0.0, 0.0)
            return
        tx, ty = self.wp[self.i + 1]
        dx, dy = tx - self.pos[0], ty - self.pos[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            self.i += 1
            return
        ux, uy = dx / dist, dy / dist
        travel = min(self.speed * dt, dist)
        self.pos[0] += ux * travel
        self.pos[1] += uy * travel
        self.vel = (ux * self.speed, uy * self.speed)
        if travel >= dist - 1e-9:
            self.i += 1


def simulate(waypoints, walk_speed=0.45, obstacles=(), ticks=200, dt=0.05,
             controller=None):
    """闭环:人走人的,车按 MPPI 指令走,每周期把世界重投到车体系。

    记录的净空是**实际执行**位姿下的精确矩形净空,不是规划器对自己的评价 ——
    后者会把规划器的乐观当成安全。
    """
    c = controller or make()
    cfg = c.cfg
    cfg.control_dt_s = dt  # The simulator runs at 20 Hz, independently of rollout dt.
    x = y = th = v = d = 0.0
    walker = Walker(waypoints, walk_speed)
    obstacles = list(obstacles)
    log = []
    for _ in range(ticks):
        cs, sn = math.cos(-th), math.sin(-th)

        def body(px, py):
            dx, dy = px - x, py - y
            return dx * cs - dy * sn, dx * sn + dy * cs

        bx, by = body(*walker.pos)
        vbx = walker.vel[0] * cs - walker.vel[1] * sn
        vby = walker.vel[0] * sn + walker.vel[1] * cs
        sol = c.solve([body(ox, oy) for ox, oy in obstacles], (bx, by),
                      (vbx, vby), v, d)
        v, d = sol.speed, sol.steer
        a = abs(d)
        if a < 1e-4:
            om = 0.0
        else:
            R = cfg.wheelbase_m / math.tan(min(a, cfg.max_steer_rad)) \
                + 0.5 * cfg.track_m
            om = abs(v) / R * (1.0 if d > 0 else -1.0)
        thm = th + 0.5 * om * dt
        x += v * math.cos(thm) * dt
        y += v * math.sin(thm) * dt
        th += om * dt
        walker.step(dt)
        bumper = (x + cfg.footprint_front_m * math.cos(th),
                  y + cfg.footprint_front_m * math.sin(th))
        gap = math.hypot(walker.pos[0] - bumper[0], walker.pos[1] - bumper[1])
        clear = rectangle_clearance([(x, y, th)], obstacles,
                                    cfg.footprint_front_m, cfg.footprint_rear_m,
                                    cfg.footprint_half_width_m)
        log.append(dict(x=x, y=y, th=th, v=v, d=d, gap=gap, clear=clear,
                        feasible=sol.feasible))
    return log


class TestClosedLoop(unittest.TestCase):

    def tail(self, log, frac=0.5):
        return log[int(len(log) * frac):]

    def test_holds_the_commanded_distance_to_a_standing_person(self):
        log = simulate([(2.5, 0.0)], ticks=160)
        gaps = [r['gap'] for r in self.tail(log)]
        mean = sum(gaps) / len(gaps)
        self.assertAlmostEqual(mean, 1.0, delta=0.25,
                               msg=f"稳态间距 {mean:.2f}m,期望 1.0m")

    def test_settles_without_dithering(self):
        """到位之后舵不准乱摆。只罚转角变化率是不够的,必须罚转角本身 ——
        否则任何恒定舵角代价都为零,随机游走会把舵停在满位。"""
        log = simulate([(2.5, 0.0)], ticks=160)
        tail = self.tail(log, 0.7)
        rms = (sum(r['d'] ** 2 for r in tail) / len(tail)) ** 0.5
        self.assertLess(rms, 0.12, f"舵角 RMS {rms:.3f} rad,车在原地摆头")

    def test_follows_a_walking_person(self):
        log = simulate([(2.0, 0.0), (8.0, 0.0)], walk_speed=0.45, ticks=240)
        gaps = [r['gap'] for r in self.tail(log)]
        self.assertLess(max(gaps), 1.6)
        self.assertGreater(min(gaps), 0.5)

    def test_follows_around_a_corner(self):
        log = simulate([(2.0, 0.0), (4.0, 0.0), (4.0, 3.0)], ticks=260)
        self.assertLess(log[-1]['gap'], 1.6)

    def test_follows_a_person_around_a_pillar_without_touching_it(self):
        pillar = circle(3.5, 0.0, 0.25)
        log = simulate([(2.0, 0), (3.0, -0.9), (4.5, -0.9), (5.5, 0), (7.5, 0)],
                       obstacles=pillar, ticks=240)
        self.assertGreater(min(r['clear'] for r in log), 0.0,
                           "跟着人绕柱子时压到了柱子")
        self.assertLess(log[-1]['gap'], 1.8)

    def test_goes_through_a_narrow_doorway(self):
        """车全宽 0.74m,门净宽 0.80m,两边各 3cm。

        这是整个避障设计的试金石:栅格距离场在 5cm 分辨率下最近邻误差可达
        3.5cm,比余量还大,所以最终计划必须用精确矩形几何复验,否则车会在
        门口"觉得自己过不去"而永远停住。
        """
        door = ([(3.0, y / 100.0) for y in range(40, 160, 6)]
                + [(3.0, -y / 100.0) for y in range(40, 160, 6)])
        log = simulate([(2.0, 0.0), (6.0, 0.0)], obstacles=door, ticks=280)
        self.assertGreater(log[-1]['x'], 3.0, "车没能穿过门")
        self.assertGreater(min(r['clear'] for r in log), 0.0, "过门时刮到了门框")

    def test_stops_safely_when_the_person_is_unreachable(self):
        """人在一堵完整的墙后面。正确答案是安全停住,不是硬顶上去。"""
        wall = [(3.0, y / 100.0) for y in range(-150, 151, 8)]
        log = simulate([(4.5, 0.0)], obstacles=wall, ticks=160)
        self.assertGreater(min(r['clear'] for r in log), 0.0)
        self.assertLess(abs(log[-1]['v']), 0.10)

    def test_keeps_the_person_inside_the_camera_frame(self):
        """抄近道会把人甩出画面。跟丢之后再好的控制律也没有意义,
        所以视野保持必须是显式代价 —— 教科书 MPPI 没有这一项。"""
        c = make()
        log = simulate([(2.0, 0.0), (4.0, 0.0), (4.0, 3.0)], ticks=260,
                       controller=c)
        worst = 0.0
        for r in log:
            # 人在车体系下的视线角
            pass
        # 用轨迹重放一次视线角
        walker = Walker([(2.0, 0.0), (4.0, 0.0), (4.0, 3.0)], 0.45)
        for r in log:
            cs, sn = math.cos(-r['th']), math.sin(-r['th'])
            dx, dy = walker.pos[0] - r['x'], walker.pos[1] - r['y']
            bx, by = dx * cs - dy * sn, dx * sn + dy * cs
            worst = max(worst, abs(math.atan2(by, bx)))
            walker.step(0.05)
        self.assertLess(worst, c.cfg.camera_hfov_rad / 2.0 + 0.10,
                        f"视线角最大 {math.degrees(worst):.1f}°,人已经出画面了")


if __name__ == "__main__":
    unittest.main(verbosity=2)
