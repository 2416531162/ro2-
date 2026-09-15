#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
虚拟激光雷达仿真节点 (Virtual LiDAR Simulator)
在手头暂无物理雷达时，发布标准 ROS 2 /scan (sensor_msgs/LaserScan) 话题
"""
import math
import random
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class FakeLidarNode(Node):
    def __init__(self):
        super().__init__('fake_lidar_node')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback) # 10Hz
        self.points_num = 360
        self.start_time = time.time()
        self.get_logger().info('>>> [Virtual LiDAR] 虚拟雷达发生器已启动 (10Hz 模拟 /scan 话题)...')

    def timer_callback(self):
        now_time = time.time()
        elapsed = now_time - self.start_time

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'

        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = (2.0 * math.pi) / float(self.points_num)
        msg.time_increment = 0.1 / float(self.points_num)
        msg.scan_time = 0.1
        msg.range_min = 0.15
        msg.range_max = 12.0

        # 模拟房间：长 6m (-3 到 +3)，宽 5m (-2.5 到 +2.5)
        room_x_min, room_x_max = -3.0, 3.0
        room_y_min, room_y_max = -2.5, 2.5

        # 模拟 2 个动态障碍物 (慢速移动的圆柱物体)
        obs1_x = 1.5 + 0.4 * math.sin(elapsed * 0.7)
        obs1_y = 1.0 + 0.3 * math.cos(elapsed * 0.5)
        obs1_r = 0.30

        obs2_x = -1.3 + 0.3 * math.cos(elapsed * 0.6)
        obs2_y = -1.2 + 0.4 * math.sin(elapsed * 0.8)
        obs2_r = 0.25

        ranges = []
        intensities = []

        for i in range(self.points_num):
            angle = msg.angle_min + i * msg.angle_increment
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)

            # 房间四壁交点
            d_candidates = []
            if abs(cos_a) > 1e-5:
                d1 = room_x_max / cos_a
                if d1 > 0: d_candidates.append(d1)
                d2 = room_x_min / cos_a
                if d2 > 0: d_candidates.append(d2)

            if abs(sin_a) > 1e-5:
                d3 = room_y_max / sin_a
                if d3 > 0: d_candidates.append(d3)
                d4 = room_y_min / sin_a
                if d4 > 0: d_candidates.append(d4)

            dist_wall = min(d_candidates) if d_candidates else msg.range_max
            dist_obs1 = self._intersect_circle(cos_a, sin_a, obs1_x, obs1_y, obs1_r)
            dist_obs2 = self._intersect_circle(cos_a, sin_a, obs2_x, obs2_y, obs2_r)

            final_dist = min(dist_wall, dist_obs1, dist_obs2)
            noise = random.gauss(0, 0.012)
            final_dist = max(msg.range_min, min(msg.range_max, final_dist + noise))

            ranges.append(round(float(final_dist), 3))
            intensity = max(50.0, min(255.0, 220.0 - final_dist * 12.0 + random.uniform(-5, 5)))
            intensities.append(round(float(intensity), 1))

        msg.ranges = ranges
        msg.intensities = intensities
        self.publisher_.publish(msg)

    def _intersect_circle(self, cos_a, sin_a, cx, cy, r):
        b = -2.0 * (cx * cos_a + cy * sin_a)
        c = cx * cx + cy * cy - r * r
        delta = b * b - 4.0 * c
        if delta < 0:
            return 9999.0
        sqrt_delta = math.sqrt(delta)
        d1 = (-b - sqrt_delta) / 2.0
        if d1 > 0.05:
            return d1
        d2 = (-b + sqrt_delta) / 2.0
        if d2 > 0.05:
            return d2
        return 9999.0

def main(args=None):
    rclpy.init(args=args)
    node = FakeLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
