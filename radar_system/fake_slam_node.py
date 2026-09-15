#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
实时 2D SLAM 空间建图仿真引擎 (Real-time SLAM & Space Modeling Engine)
- 模拟移动机器人在 6m x 5m 空间内巡航走动
- 激光雷达 Ray-casting 实时驱散迷雾，构建 OccupancyGrid 栅格地图
- 发布标准 ROS 2 话题:
    1. /map (nav_msgs/msg/OccupancyGrid): 实时动态建图
    2. /scan (sensor_msgs/msg/LaserScan): 随移动位置变化的激光雷达
    3. /robot_pose (geometry_msgs/msg/PoseStamped): 机器人航迹位姿
"""

import math
import time
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseStamped

class RealtimeSLAMNode(Node):
    def __init__(self):
        super().__init__('realtime_slam_node')
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 5)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)

        # 地图参数: 8m x 7m，分辨率 0.05m (5cm)，160x140 栅格
        self.res = 0.05
        self.width = 160
        self.height = 140
        self.origin_x = -4.0
        self.origin_y = -3.5
        # 初始化地图全为 -1 (未知区域，战争迷雾)
        self.grid = [-1] * (self.width * self.height)

        self.start_time = time.time()
        self.timer = self.create_timer(0.1, self.loop_callback) # 10Hz
        self.trajectory = []
        self.get_logger().info('>>> [SLAM Modeling] 实时空间建图与三维建模引擎已启动...')

    def loop_callback(self):
        t = time.time() - self.start_time

        # 模拟机器人在房间里巡航移动 (带平滑角速度的平滑航线)
        # 航线：围绕房间中心做 1.6m x 1.2m 的巡航回环
        robot_x = 1.4 * math.sin(t * 0.35)
        robot_y = 1.0 * math.sin(t * 0.70)
        # 计算小车车头航向角 (yaw)
        vx = 1.4 * 0.35 * math.cos(t * 0.35)
        vy = 1.0 * 0.70 * math.cos(t * 0.70)
        robot_yaw = math.atan2(vy, vx)

        self.trajectory.append((round(robot_x, 2), round(robot_y, 2)))
        if len(self.trajectory) > 300:
            self.trajectory.pop(0)

        # 1. 生成并发布机器人位姿
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = robot_x
        pose_msg.pose.position.y = robot_y
        pose_msg.pose.orientation.z = math.sin(robot_yaw / 2.0)
        pose_msg.pose.orientation.w = math.cos(robot_yaw / 2.0)
        self.pose_pub.publish(pose_msg)

        # 2. 从机器人当前位置发射 360 根激光射线，进行环境扫描
        room_x_min, room_x_max = -3.0, 3.0
        room_y_min, room_y_max = -2.5, 2.5
        obs1_x, obs1_y, obs1_r = 1.5, 1.2, 0.3
        obs2_x, obs2_y, obs2_r = -1.2, -1.0, 0.25

        scan_msg = LaserScan()
        scan_msg.header.stamp = pose_msg.header.stamp
        scan_msg.header.frame_id = 'laser'
        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 2.0 * math.pi
        scan_msg.angle_increment = (2.0 * math.pi) / 360.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.15
        scan_msg.range_max = 10.0

        ranges = []
        intensities = []

        # 机器人当前栅格坐标
        r_gx = int((robot_x - self.origin_x) / self.res)
        r_gy = int((robot_y - self.origin_y) / self.res)

        for i in range(360):
            # 雷达朝向为自身角度 + 车头 yaw
            laser_angle = scan_msg.angle_min + i * scan_msg.angle_increment
            global_angle = laser_angle + robot_yaw
            cos_a = math.cos(global_angle)
            sin_a = math.sin(global_angle)

            # 与四面墙壁交点
            d_candidates = []
            if abs(cos_a) > 1e-5:
                d1 = (room_x_max - robot_x) / cos_a
                if d1 > 0: d_candidates.append(d1)
                d2 = (room_x_min - robot_x) / cos_a
                if d2 > 0: d_candidates.append(d2)

            if abs(sin_a) > 1e-5:
                d3 = (room_y_max - robot_y) / sin_a
                if d3 > 0: d_candidates.append(d3)
                d4 = (room_y_min - robot_y) / sin_a
                if d4 > 0: d_candidates.append(d4)

            dist_wall = min(d_candidates) if d_candidates else 10.0
            dist_obs1 = self._intersect_circle(robot_x, robot_y, cos_a, sin_a, obs1_x, obs1_y, obs1_r)
            dist_obs2 = self._intersect_circle(robot_x, robot_y, cos_a, sin_a, obs2_x, obs2_y, obs2_r)

            d_hit = min(dist_wall, dist_obs1, dist_obs2)
            d_hit += random.gauss(0, 0.01)
            d_hit = max(scan_msg.range_min, min(scan_msg.range_max, d_hit))
            ranges.append(round(d_hit, 3))
            intensities.append(180.0)

            # 3. SLAM 光线追踪 (Ray-casting) 更新占据栅格地图！
            # 射线沿途点打标为 0 (通行空闲)，命中终点打标为 100 (障碍物墙体)
            hit_x = robot_x + d_hit * cos_a
            hit_y = robot_y + d_hit * sin_a
            h_gx = int((hit_x - self.origin_x) / self.res)
            h_gy = int((hit_y - self.origin_y) / self.res)

            # 每隔 10 度抽样一根光线做地图更新，兼顾性能与画质
            if i % 2 == 0:
                self._trace_ray(r_gx, r_gy, h_gx, h_gy)

        scan_msg.ranges = ranges
        scan_msg.intensities = intensities
        self.scan_pub.publish(scan_msg)

        # 4. 发布占据栅格地图 (每 0.3 秒发布一次以降低网络带宽)
        if int(t * 10) % 3 == 0:
            map_msg = OccupancyGrid()
            map_msg.header.stamp = pose_msg.header.stamp
            map_msg.header.frame_id = 'map'
            map_msg.info.resolution = self.res
            map_msg.info.width = self.width
            map_msg.info.height = self.height
            map_msg.info.origin.position.x = self.origin_x
            map_msg.info.origin.position.y = self.origin_y
            map_msg.data = self.grid
            self.map_pub.publish(map_msg)

    def _trace_ray(self, x0, y0, x1, y1):
        """Bresenham 直线算法更新栅格"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        cur_x, cur_y = x0, y0
        while True:
            # 终点打标为墙体/障碍物 (100)
            if cur_x == x1 and cur_y == y1:
                if 0 <= cur_x < self.width and 0 <= cur_y < self.height:
                    self.grid[cur_y * self.width + cur_x] = 100
                break

            # 沿途打标为空闲可行走区域 (0)
            if 0 <= cur_x < self.width and 0 <= cur_y < self.height:
                if self.grid[cur_y * self.width + cur_x] != 100:
                    self.grid[cur_y * self.width + cur_x] = 0

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                cur_x += sx
            if e2 < dx:
                err += dx
                cur_y += sy

    def _intersect_circle(self, rx, ry, cos_a, sin_a, cx, cy, r):
        # 射线从 (rx, ry) 出发
        dx = rx - cx
        dy = ry - cy
        b = 2.0 * (dx * cos_a + dy * sin_a)
        c = dx * dx + dy * dy - r * r
        delta = b * b - 4.0 * c
        if delta < 0: return 9999.0
        s = math.sqrt(delta)
        d1 = (-b - s) / 2.0
        if d1 > 0.05: return d1
        d2 = (-b + s) / 2.0
        if d2 > 0.05: return d2
        return 9999.0

def main(args=None):
    rclpy.init(args=args)
    node = RealtimeSLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
