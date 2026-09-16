#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车体足迹与扫掠路径碰撞检查的回归测试。

每条测试都对应一个「把车当成点」会撞、按足迹算就不会撞的具体场景 ——
尤其是过门时轮子刮墙。

    python3 tests/test_footprint.py
"""

import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "radar_system"))

from footprint import (  # noqa: E402
    VehicleFootprint, SensorMount, scan_to_vehicle_frame,
    corridor_clearance, arc_clearance, swept_path_clearance,
    widest_passable_steer, limit_steer_for_clearance, optical_to_vehicle,
    _swept_radii,
)
from motion_safety import ChassisGeometry  # noqa: E402


FP = VehicleFootprint(front_m=0.45, rear_m=0.25, half_width_m=0.30, margin_m=0.06)
GEO = ChassisGeometry(wheelbase_m=0.25, track_m=0.17, max_steer_rad=0.35)


class TestFootprintBasics(unittest.TestCase):

    def test_min_gap_includes_margin(self):
        self.assertAlmostEqual(FP.min_gap_needed(), 0.72, places=6)

    def test_rejects_nonsense_dimensions(self):
        with self.assertRaises(ValueError):
            VehicleFootprint(front_m=0.0)
        with self.assertRaises(ValueError):
            VehicleFootprint(half_width_m=-0.1)
        with self.assertRaises(ValueError):
            VehicleFootprint(margin_m=-0.01)

    def test_corners(self):
        corners = FP.corners()
        self.assertEqual(len(corners), 4)
        self.assertIn((0.45, 0.30), corners)
        self.assertIn((-0.25, -0.30), corners)


class TestCorridorClearance(unittest.TestCase):
    """直行:锥形检查漏掉的就是这些。"""

    def test_obstacle_dead_ahead(self):
        self.assertAlmostEqual(corridor_clearance([(1.45, 0.0)], FP), 1.0, places=3)

    def test_obstacle_beside_the_wheel_is_detected(self):
        """核心场景:障碍物在车宽之内但偏出锥形中心,必须算数。

        旧的 ±30° 锥形在 1.0m 处只覆盖 ±0.58m 看似够,但判定用的是
        「极坐标角度」而不是「横向距离」—— 近处根本罩不住轮子。
        """
        self.assertAlmostEqual(corridor_clearance([(1.45, 0.34)], FP), 1.0, places=3)

    def test_obstacle_just_outside_the_body_is_ignored(self):
        self.assertEqual(corridor_clearance([(1.45, 0.40)], FP), 8.0)

    def test_margin_is_respected(self):
        """余量之内的东西要算,余量之外不算。"""
        bare = VehicleFootprint(front_m=0.45, rear_m=0.25,
                                half_width_m=0.30, margin_m=0.0)
        self.assertEqual(corridor_clearance([(1.45, 0.33)], bare), 8.0)
        self.assertAlmostEqual(corridor_clearance([(1.45, 0.33)], FP), 1.0, places=3)

    def test_doorway_too_narrow_is_blocked(self):
        """0.70m 净宽的门:车需要 0.72m,过不去,必须报 0 净空。"""
        points = [(1.2, 0.35), (1.2, -0.35)]     # 两侧门框
        self.assertLess(corridor_clearance(points, FP), 8.0)

    def test_doorway_wide_enough_is_clear(self):
        """0.90m 净宽的门:车 0.72m,让得开。"""
        points = [(1.2, 0.45), (1.2, -0.45)]
        self.assertEqual(corridor_clearance(points, FP), 8.0)

    def test_obstacle_already_touching_bumper(self):
        self.assertEqual(corridor_clearance([(0.30, 0.0)], FP), 0.0)

    def test_behind_the_vehicle_is_ignored_for_forward_motion(self):
        self.assertEqual(corridor_clearance([(-1.0, 0.0)], FP), 8.0)

    def test_empty_scan_is_clear(self):
        self.assertEqual(corridor_clearance([], FP), 8.0)


class TestSweptRadii(unittest.TestCase):
    """转弯扫掠:门框是被内侧后轮和外侧前角刮的。"""

    def test_swept_band_is_wider_than_the_vehicle(self):
        """这是过门刮墙的关键:转弯需要的通道比直行更宽。"""
        for deg in (5, 10, 14, 20):
            radius = GEO.wheelbase_m / math.tan(math.radians(deg)) + 0.5 * GEO.track_m
            inner, outer = _swept_radii(FP, radius)
            band = outer - inner
            self.assertGreater(band, FP.width_m,
                               f"{deg}° 时扫掠带宽 {band:.3f} 应当大于车宽 {FP.width_m}")

    def test_sharper_turn_sweeps_wider(self):
        bands = []
        for deg in (5, 10, 20):
            radius = GEO.wheelbase_m / math.tan(math.radians(deg)) + 0.5 * GEO.track_m
            inner, outer = _swept_radii(FP, radius)
            bands.append(outer - inner)
        self.assertEqual(bands, sorted(bands), "转角越大,扫掠带应当越宽")

    def test_inner_radius_never_negative(self):
        inner, _ = _swept_radii(FP, 0.1)      # 半径比半车宽还小
        self.assertGreaterEqual(inner, 0.0)


class TestArcClearance(unittest.TestCase):

    def _radius(self, deg):
        return GEO.wheelbase_m / math.tan(math.radians(deg)) + 0.5 * GEO.track_m

    def test_obstacle_on_the_arc_is_detected(self):
        r = self._radius(14)
        # 左转 90 度后车会到达的位置附近放一个障碍
        point = (r, r)          # 相对转弯中心 (0, r) 正好在正前方 r 处
        clear = arc_clearance([point], FP, r, left=True)
        self.assertLess(clear, 8.0)

    def test_obstacle_inside_the_hole_is_ignored(self):
        """转弯中心附近那块地方车扫不到,不该误报。"""
        r = self._radius(14)
        clear = arc_clearance([(0.0, r)], FP, r, left=True)
        self.assertEqual(clear, 8.0)

    def test_obstacle_outside_the_swept_band_is_ignored(self):
        r = self._radius(14)
        _, outer = _swept_radii(FP, r)
        clear = arc_clearance([(outer + 1.0, r)], FP, r, left=True)
        self.assertEqual(clear, 8.0)

    def test_left_and_right_are_mirror_images(self):
        r = self._radius(14)
        pt = (1.0, 0.5)
        left = arc_clearance([pt], FP, r, left=True)
        right = arc_clearance([(pt[0], -pt[1])], FP, r, left=False)
        self.assertAlmostEqual(left, right, places=6)

    def test_turning_sees_what_straight_driving_misses(self):
        """核心场景:障碍物在左前方、直行让得开,但左转就会扫上去。

        这正是过门刮墙的几何:车头中线能过去,不代表转弯时四个角都能过去。
        """
        pt = (0.60, 0.50)                     # 横向 0.50m,超出 0.36m 的直行走廊
        r = self._radius(20)
        straight = corridor_clearance([pt], FP)
        turning = arc_clearance([pt], FP, r, left=True)
        self.assertEqual(straight, 8.0, "直行确实撞不到")
        self.assertLess(turning, 8.0, "但左转会扫到它")
        self.assertGreater(turning, 0.0, "还没贴上,应当给出一段可行距离")

    def test_turning_away_from_the_obstacle_is_clear(self):
        """同一个左前方的障碍,右转就应当让得开。"""
        pt = (0.60, 0.50)
        r = self._radius(20)
        self.assertEqual(arc_clearance([pt], FP, r, left=False), 8.0)

    def test_zero_radius_is_safe(self):
        self.assertEqual(arc_clearance([(1.0, 0.0)], FP, 0.0, left=True), 0.0)


class TestSweptPathClearance(unittest.TestCase):

    def test_small_steer_uses_corridor(self):
        pt = [(1.45, 0.0)]
        self.assertAlmostEqual(
            swept_path_clearance(pt, FP, GEO, math.radians(0.2)), 1.0, places=3)

    def test_steer_is_clamped_to_physical_limit(self):
        pts = [(1.0, 0.3), (0.8, -0.2)]
        a = swept_path_clearance(pts, FP, GEO, math.radians(45))
        b = swept_path_clearance(pts, FP, GEO, GEO.max_steer_rad)
        self.assertAlmostEqual(a, b, places=6)

    def test_clearance_is_measured_from_the_bumper(self):
        """净空必须从车头最前端算,不是从后轴中心或雷达算。"""
        near = VehicleFootprint(front_m=0.20, rear_m=0.25,
                                half_width_m=0.30, margin_m=0.06)
        far = VehicleFootprint(front_m=0.60, rear_m=0.25,
                               half_width_m=0.30, margin_m=0.06)
        pt = [(1.5, 0.0)]
        self.assertAlmostEqual(corridor_clearance(pt, near), 1.30, places=3)
        self.assertAlmostEqual(corridor_clearance(pt, far), 0.90, places=3)


class TestSensorMount(unittest.TestCase):

    def test_forward_mounted_lidar_shifts_points(self):
        """雷达装在车头附近时,它报的 1.0m 对后轴中心其实是 1.4m。"""
        mount = SensorMount(x_m=0.40, y_m=0.0, yaw_rad=0.0)
        pts = scan_to_vehicle_frame([(0.0, 1.0)], mount)
        self.assertAlmostEqual(pts[0][0], 1.40, places=6)
        self.assertAlmostEqual(pts[0][1], 0.0, places=6)

    def test_lateral_offset(self):
        mount = SensorMount(x_m=0.0, y_m=0.10)
        pts = scan_to_vehicle_frame([(math.pi / 2, 1.0)], mount)
        self.assertAlmostEqual(pts[0][1], 1.10, places=6)

    def test_yaw_offset_rotates(self):
        mount = SensorMount(yaw_rad=math.pi / 2)
        pts = scan_to_vehicle_frame([(0.0, 1.0)], mount)
        self.assertAlmostEqual(pts[0][0], 0.0, places=6)
        self.assertAlmostEqual(pts[0][1], 1.0, places=6)

    def test_invalid_readings_are_dropped(self):
        mount = SensorMount()
        pts = scan_to_vehicle_frame(
            [(0.0, float('inf')), (0.0, float('nan')), (0.0, -1.0),
             (0.0, 99.0), (0.0, 1.0)], mount, max_range=8.0)
        self.assertEqual(len(pts), 1)


class TestWidestPassableSteer(unittest.TestCase):

    def test_prefers_the_open_side(self):
        """左边堵死、右边空着时,应当挑右转。"""
        points = [(1.0, 0.25), (1.2, 0.30), (1.4, 0.28)]   # 左前方一排障碍
        steer, clear = widest_passable_steer(points, FP, GEO)
        self.assertLessEqual(steer, 0.0, "应当选择向右避让")
        self.assertGreater(clear, 0.0)

    def test_open_field_prefers_straight_or_equal(self):
        steer, clear = widest_passable_steer([], FP, GEO)
        self.assertEqual(clear, 8.0)




# =============================================================================
# 实车配置锁定 —— 防止有人改回错误的默认值
# =============================================================================

REAL = dict(width=0.67, front=0.67, rear=0.18, wheelbase=0.54, track=0.59,
            lidar_x=0.53, lidar_y=0.0)


class TestRealVehicleConfig(unittest.TestCase):
    """锁住 2026-09-16 实测的这台车的尺寸。

    改造前代码里写的是轴距 0.25 / 轮距 0.17,比实车小 2.2 倍和 3.5 倍,
    导致最小转弯半径算成 0.77m(实际 1.77m),整套转向与扫掠计算全错。
    这组测试就是防止再次跑偏。
    """

    def test_chassis_geometry_matches_measurements(self):
        geo = ChassisGeometry()
        self.assertAlmostEqual(geo.wheelbase_m, REAL['wheelbase'], places=3)
        self.assertAlmostEqual(geo.track_m, REAL['track'], places=3)

    def test_footprint_defaults_match_measurements(self):
        fp = VehicleFootprint()
        self.assertAlmostEqual(fp.width_m, REAL['width'], places=3)
        self.assertAlmostEqual(fp.front_m, REAL['front'], places=3)
        self.assertAlmostEqual(fp.rear_m, REAL['rear'], places=3)

    def test_min_turn_radius(self):
        self.assertAlmostEqual(ChassisGeometry().min_turn_radius_m, 1.773, places=2)

    def test_track_is_narrower_than_width(self):
        """轮距是轮中心距,必然小于含轮胎外沿的全宽。"""
        self.assertLess(ChassisGeometry().track_m, VehicleFootprint().width_m)

    def test_lidar_sits_near_the_front_axle(self):
        """雷达在后轴前 0.53m,前轴在 0.54m —— 基本重合,与照片一致。"""
        self.assertLess(abs(REAL['lidar_x'] - REAL['wheelbase']), 0.05)

    def test_doorway_reality_check(self):
        """这台车通过标准门只能笔直走 —— 这是几何硬约束,不是软件缺陷。"""
        fp, geo = VehicleFootprint(), ChassisGeometry()
        straight_need = fp.min_gap_needed()
        radius = geo.wheelbase_m / math.tan(geo.max_steer_rad) + 0.5 * geo.track_m
        inner, outer = _swept_radii(fp, radius)
        turning_need = outer - inner

        self.assertLess(straight_need, 0.80, "直行应当能过 0.80m 的标准门")
        self.assertGreater(turning_need, 0.90, "满舵需要的通道应当明显更宽")
        self.assertGreater(turning_need - straight_need, 0.10,
                           "转弯比直行多占的宽度就是刮门框的量")


class TestLimitSteerForClearance(unittest.TestCase):
    """过门收舵:不该傻停在门口,先摆正穿过去。"""

    def _doorway(self, clear_width, distance=1.5):
        """构造一个净宽 clear_width 的门框(两侧各一排点)。"""
        half = clear_width / 2.0
        pts = []
        for i in range(12):
            x = distance + i * 0.05
            pts.append((x, half))
            pts.append((x, -half))
        return pts

    def test_straightens_up_to_get_through_a_narrow_door(self):
        """车已经开到门口 0.8m,人偏了车想追,此时必须先摆正。

        门净宽 0.85m,车直行需要 0.79m —— 笔直走过得去;
        但打 15° 舵会把外侧前角甩到门框上。
        """
        fp, geo = VehicleFootprint(), ChassisGeometry()
        pts = self._doorway(0.85, distance=0.8)
        desired = math.radians(15)

        turning = swept_path_clearance(pts, fp, geo, desired)
        safe, clear = limit_steer_for_clearance(pts, fp, geo, desired, 0.35)

        self.assertLess(turning, 0.35, "打舵时确实会撞上门框")
        self.assertLess(abs(safe), abs(desired), "应当收舵")
        self.assertGreater(clear, turning, "收舵后净空必须变大")
        self.assertGreaterEqual(clear, 0.35, "收舵后应当走得通")

    def test_no_limiting_when_the_door_is_still_far(self):
        """门还在 1.5m 外时打舵不会碰到,不该提前干预。"""
        fp, geo = VehicleFootprint(), ChassisGeometry()
        pts = self._doorway(0.85, distance=1.5)
        desired = math.radians(15)
        safe, _ = limit_steer_for_clearance(pts, fp, geo, desired, 0.35)
        self.assertAlmostEqual(safe, desired, places=9)

    def test_keeps_full_steer_when_there_is_room(self):
        fp, geo = VehicleFootprint(), ChassisGeometry()
        desired = math.radians(15)
        safe, _ = limit_steer_for_clearance([], fp, geo, desired, 0.35)
        self.assertAlmostEqual(safe, desired, places=9,
                               msg="空旷时不该无故收舵")

    def test_gives_up_only_when_even_straight_is_blocked(self):
        """门比车还窄,收到笔直也过不去 —— 此时净空为 0,交给包络停车。"""
        fp, geo = VehicleFootprint(), ChassisGeometry()
        pts = self._doorway(0.50, distance=1.0)    # 0.50m < 车宽 0.67m
        safe, clear = limit_steer_for_clearance(pts, fp, geo, math.radians(15), 0.35)
        self.assertLess(clear, 0.35)

    def test_zero_desired_steer_is_untouched(self):
        fp, geo = VehicleFootprint(), ChassisGeometry()
        safe, _ = limit_steer_for_clearance(self._doorway(0.85), fp, geo, 0.0, 0.35)
        self.assertEqual(safe, 0.0)


# =============================================================================
# 距离语义 —— 所有间距以「车头最前端」为基准
# =============================================================================

class TestDistanceSemantics(unittest.TestCase):

    FRONT, CAMERA_X, LIDAR_X = 0.67, 0.54, 0.53

    def test_camera_and_lidar_offsets_are_close(self):
        """相机 0.54 与雷达 0.53 只差 1cm,交叉证伪的零点差可忽略。"""
        self.assertLess(abs(self.CAMERA_X - self.LIDAR_X), 0.03)

    def test_camera_to_bumper(self):
        self.assertAlmostEqual(self.FRONT - self.CAMERA_X, 0.13, places=6)

    def test_overstating_camera_offset_brings_the_car_too_close(self):
        """填大 offset(以为相机更靠前)会高估间距,车开得更近。"""
        true_gap = 1.00 - (self.FRONT - self.CAMERA_X)
        wrong_gap = 1.00 - (self.FRONT - self.FRONT)
        self.assertGreater(wrong_gap, true_gap)


class TestObstacleStandoff(unittest.TestCase):
    """回归:曾把 clearance + standoff 喂给包络,归零点变成 0,贴上去才停。"""

    def setUp(self):
        from motion_safety import BrakeProfile, brake_envelope
        self.brake_envelope = brake_envelope
        self.profile = BrakeProfile(decel_mps2=1.0, latency_s=0.35,
                                    stop_m=0.30, hard_stop_m=0.15)

    def test_speed_reaches_zero_while_clearance_remains(self):
        self.assertEqual(self.brake_envelope(0.30, self.profile), 0.0)

    def test_the_old_buggy_form_would_have_touched(self):
        self.assertGreater(self.brake_envelope(0.10 + 0.30, self.profile), 0.0,
                           "旧写法在只剩 10cm 时仍然放行")

    def test_hard_stop_is_inside_the_normal_standoff(self):
        self.assertLess(self.profile.hard_stop_m, self.profile.stop_m)


# =============================================================================
# 相机俯角换算 —— 深度 z 沿光轴,不是水平距离
# =============================================================================

class TestOpticalToVehicle(unittest.TestCase):

    PITCH = math.radians(15)
    MOUNT = SensorMount(x_m=0.54)

    def _roundtrip(self, horiz, height, lateral=0.0):
        c, s = math.cos(self.PITCH), math.sin(self.PITCH)
        return (-lateral, -horiz * s - height * c, horiz * c - height * s)

    def test_recovers_true_horizontal_distance(self):
        for height in (-0.2, 0.0, 0.2, 0.4, 0.6, 0.8):
            for horiz in (0.8, 1.13, 2.0, 3.5):
                xo, yo, zo = self._roundtrip(horiz, height)
                x, _, _ = optical_to_vehicle(xo, yo, zo, self.MOUNT, self.PITCH)
                self.assertAlmostEqual(x - self.MOUNT.x_m, horiz, places=9,
                                       msg=f"高度 {height} 距离 {horiz} 还原失败")

    def test_recovers_lateral_offset(self):
        xo, yo, zo = self._roundtrip(1.13, 0.6, lateral=0.3)
        _, y, _ = optical_to_vehicle(xo, yo, zo, self.MOUNT, self.PITCH)
        self.assertAlmostEqual(y, 0.3, places=9)

    def test_raw_depth_underreads_for_a_standing_person(self):
        """人的躯干在光轴上方,俯角让相机把距离读小近 20cm。"""
        _, _, z_opt = self._roundtrip(1.13, 0.6)
        self.assertLess(z_opt, 1.13 - 0.15)

    def test_error_grows_with_target_height(self):
        """误差不是固定值,不能用常数补偿。"""
        errs = [1.13 - self._roundtrip(1.13, h)[2] for h in (0.0, 0.3, 0.6, 0.9)]
        self.assertEqual(errs, sorted(errs))
        self.assertGreater(errs[-1] - errs[0], 0.15)

    def test_level_camera_is_a_no_op(self):
        x, _, _ = optical_to_vehicle(0.0, -0.5, 1.2, SensorMount(x_m=0.54), 0.0)
        self.assertAlmostEqual(x - 0.54, 1.2, places=9)


if __name__ == "__main__":
    unittest.main(verbosity=2)
