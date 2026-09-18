#!/usr/bin/env bash
""":"
# Bash wrapper to ensure ROS 2 Jazzy environment is loaded
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
exec python3 -u "$0" "$@"
"""
# -*- coding: utf-8 -*-
"""激光雷达免移车/免拆装零点标定工具 (LiDAR Yaw Calibrator)

用途:
  雷达物理位置被碰动/旋转后, 在小车完全静止、不拆卸雷达的情况下,
  通过软件计算偏航偏差角 (Yaw Offset) 并自动写入配置。
"""

import sys
import os
import math
import time
import json
import argparse
from collections import deque
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class LidarCalibrator(Node):

    def __init__(self, mode="person"):
        super().__init__('lidar_calibrator')
        self.mode = mode
        self.samples = deque(maxlen=30)
        self.latest_scan = None
        self.latest_targets = None
        self.last_target_time = 0.0

        self.create_subscription(
            LaserScan, '/scan', self.on_scan,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.create_subscription(
            String, '/camera/ai_detection/targets', self.on_targets, 10)

        self.timer = self.create_timer(0.1, self.tick)
        self.finished = False
        self.result_yaw_deg = None

    def on_scan(self, msg):
        self.latest_scan = msg

    def on_targets(self, msg):
        try:
            items = json.loads(msg.data)
            if isinstance(items, list):
                self.latest_targets = items
                self.last_target_time = time.monotonic()
        except Exception:
            pass

    def tick(self):
        if self.finished:
            return

        if self.mode == "person":
            self.tick_person()
        elif self.mode == "wall":
            self.tick_wall()

    def tick_person(self):
        now = time.monotonic()
        if self.latest_scan is None:
            print("\r[等待中] 正在接收激光雷达 /scan 数据...", end="", flush=True)
            return

        if self.latest_targets is None or now - self.last_target_time > 1.0:
            print("\r[等待目标] 请一个人站到小车正前方 1.0~2.5 米处 (面向相机)...", end="", flush=True)
            return

        # 寻找相机识别到的人体目标
        person = None
        for item in self.latest_targets:
            if item.get('label') == 'person' or 'bearing_rad' in item:
                person = item
                break

        if person is None:
            print("\r[等待目标] 相机画面未检测到人体, 请站到相机画面中...", end="", flush=True)
            return

        cam_bearing = float(person.get('bearing_rad', 0.0))
        cam_bearing_deg = math.degrees(cam_bearing)
        cam_dist = float(person.get('z', 0.0) or 0.0)
        if cam_dist <= 0.0:
            raw_dist = person.get('dist_m')
            cam_dist = float(raw_dist) if raw_dist else 1.5

        # 在雷达点云中, 寻找同距离 (cam_dist ± 0.5m) 且同大致方向个人体反射群
        msg = self.latest_scan
        n = len(msg.ranges)
        ainc = msg.angle_increment or (2 * math.pi / n)
        amin = msg.angle_min

        candidates = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= 0.3:
                continue
            deg = (math.degrees(amin + i * ainc) + 180.0) % 360.0 - 180.0
            # 只看目标方位角 ±35° 扇形
            if abs((deg - cam_bearing_deg + 180.0) % 360.0 - 180.0) <= 35.0:
                # 距离窗口 (若相机给出了有效距离则用相机距离窄带, 否则在 0.8~3.0m 内聚类)
                if 0.5 <= cam_dist <= 3.5:
                    if abs(r - cam_dist) <= 0.45:
                        candidates.append((deg, r))
                elif 0.8 <= r <= 3.0:
                    candidates.append((deg, r))

        if len(candidates) < 3:
            print(f"\r[对齐中] 相机已看到人 ({cam_bearing_deg:+.1f}°), 正在锁定雷达点云...", end="", flush=True)
            return

        # 取角度中位数作为雷达目标中心
        c_degs = [c[0] for c in candidates]
        lidar_bearing_deg = float(np.median(c_degs))
        diff_deg = cam_bearing_deg - lidar_bearing_deg

        self.samples.append(diff_deg)
        count = len(self.samples)
        std = float(np.std(self.samples)) if count > 5 else 99.0
        mean = float(np.mean(self.samples))

        print(f"\r[采样进度 {count}/20] 相机:{cam_bearing_deg:+5.1f}° | 雷达:{lidar_bearing_deg:+5.1f}° | 偏差:{diff_deg:+5.1f}° (标准差:{std:.2f}°)", end="", flush=True)

        if count >= 20 and std < 1.2:
            print("\n")
            self.finished = True
            self.result_yaw_deg = round(mean, 1)

    def tick_wall(self):
        if self.latest_scan is None:
            print("\r[等待中] 正在接收激光雷达 /scan 数据...", end="", flush=True)
            return

        msg = self.latest_scan
        n = len(msg.ranges)
        ainc = msg.angle_increment or (2 * math.pi / n)
        amin = msg.angle_min

        # 提取车头正前方 ±30° 内 0.8m ~ 3.5m 的点 (平墙面)
        pts_x = []
        pts_y = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < 0.6 or r > 3.5:
                continue
            rad = amin + i * ainc
            deg = (math.degrees(rad) + 180.0) % 360.0 - 180.0
            if abs(deg) <= 30.0:
                sx = r * math.cos(rad)
                sy = r * math.sin(rad)
                pts_x.append(sx)
                pts_y.append(sy)

        if len(pts_x) < 20:
            print("\r[分析中] 前方未探测到足够的平墙点云 (需 0.8~3.5m 内有墙或大平面)...", end="", flush=True)
            return

        # 拟合直线: x = m * y + c
        # 当小车正对墙壁时, 墙面垂直于 x 轴, m 应该为 0
        # 如果雷达偏航 yaw 偏了, m = tan(yaw)
        poly = np.polyfit(pts_y, pts_x, 1)
        slope_m = poly[0]
        tilt_deg = math.degrees(math.atan(slope_m))

        self.samples.append(tilt_deg)
        count = len(self.samples)
        std = float(np.std(self.samples)) if count > 5 else 99.0
        mean = float(np.mean(self.samples))

        print(f"\r[平墙采样 {count}/20] 墙面法线偏差: {tilt_deg:+5.1f}° (标准差:{std:.2f}°)", end="", flush=True)

        if count >= 20 and std < 0.8:
            print("\n")
            self.finished = True
            self.result_yaw_deg = round(mean, 1)


from runtime_config import load_profile
from robot_core.config import DEFAULT_PATH
CALIB_FILE = os.environ.get('RK3588_ROBOT_CONFIG', str(DEFAULT_PATH))


def update_config_file(delta_yaw_deg):
    from pathlib import Path
    path = Path(CALIB_FILE)
    data = load_profile(path)
    data['sensors']['raw_lidar_yaw_deg'] = round(data['sensors']['raw_lidar_yaw_deg'] + float(delta_yaw_deg), 1)
    temporary = path.with_suffix('.pending.json')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    load_profile(temporary)
    temporary.replace(path)
    print(f'已更新统一配置 {path}；停车后重启底盘和感知服务以使用同一配置版本。')
    return True


def main():
    parser = argparse.ArgumentParser(description="激光雷达免移车零点标定工具")
    parser.add_argument('--mode', choices=['person', 'wall'], default=None,
                        help='标定模式: person (人体对齐) 或 wall (平墙对齐)')
    parser.add_argument('--auto-save', action='store_true',
                        help='自动保存标定结果到配置文件')
    args, _ = parser.parse_known_args()

    mode = args.mode
    if mode is None:
        print("=" * 60)
        print("   🎯 RK3588 激光雷达免移车/免拆装零点标定工具")
        print("=" * 60)
        print("请选择标定方式:")
        print("  1. 【人体对齐标定】 (推荐: 人站到车头前 1.5 米, 3秒自动对齐)")
        print("  2. 【平墙对齐标定】 (车头大致正对一面平墙, 自动测出墙面倾斜角)")
        try:
            choice = input("请输入序号 [1/2] (默认 1): ").strip()
        except EOFError:
            choice = "1"
        mode = "wall" if choice == "2" else "person"

    rclpy.init()
    node = LidarCalibrator(mode=mode)
    print(f"\n🚀 开始执行【{mode}】模式标定...")

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("\n标定已取消。")
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if node.result_yaw_deg is not None:
        yaw = node.result_yaw_deg
        direction = "逆时针" if yaw > 0 else "顺时针"
        print("=" * 60)
        print(f"🎉 标定完成！")
        print(f"   雷达实际偏航安装角偏差 (Yaw Offset): {yaw:+.1f}° (相对车头中轴线偏向 {direction} {abs(yaw):.1f}°)")
        print(f"   在当前原始零点校正基础上追加: {yaw:+.1f}°")
        print("=" * 60)

        if args.auto_save:
            update_config_file(yaw)
        else:
            try:
                ans = input(f"\n是否将偏差 {yaw:+.1f}° 累加到统一配置 {CALIB_FILE}？[Y/n]: ").strip().lower()
            except EOFError:
                ans = "y"
            if ans in ('', 'y', 'yes'):
                update_config_file(yaw)
                print("\n提示: 如果跟随服务已经在后台运行, 重启即可生效:")
                print("  停车后重启整套底盘与感知服务，确保配置摘要一致。")
        return 0

    return 1


if __name__ == '__main__':
    sys.exit(main())
