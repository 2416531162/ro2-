#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RK3588 激光雷达 (LiDAR) + 奥比中光 Astra S 3D深度相机 联合 3D 空间建图节点
- 融合 2D 平面雷达 360° 宽域感知与 3D 深度相机立体空间纵向感知
- 建立机器人物理坐标系 TF 树: map -> base_link -> laser & camera_link
- 实时合成高精度、带高度维度的稠密点云 /fused_pointcloud
- 提供给 OctoMap 3D 栅格体素建图引擎 (/octomap_full, /projected_map)
"""

import time
import json
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, Image, CameraInfo
from std_msgs.msg import String, Header
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import sensor_msgs_py.point_cloud2 as pc2

class Joint3DMappingNode(Node):
    def __init__(self):
        super().__init__("joint_3d_mapping_node")

        # 1. 广播机器人底盘与各传感器相对位姿 TF
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.broadcast_static_transforms()

        # 2. 状态变量与帧缓冲
        self.latest_scan = None
        self.latest_depth = None
        self.fx = 570.3
        self.fy = 570.3
        self.cx = 319.5
        self.cy = 239.5
        self.fusion_count = 0
        self.last_stat_time = time.time()
        self.fusion_hz = 0.0

        # 3. 发布者
        self.pub_fused = self.create_publisher(PointCloud2, "/fused_pointcloud", 10)
        self.pub_status = self.create_publisher(String, "/joint_mapping/status", 10)

        # 4. 订阅者
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.sub_info = self.create_subscription(CameraInfo, "/camera/rgb/camera_info", self.info_cb, 5)
        self.sub_depth = self.create_subscription(Image, "/camera/depth_registered/image_raw", self.depth_cb, 5)
        self.sub_depth_raw = self.create_subscription(Image, "/camera/depth_raw/image", self.depth_cb, 5)
        self.sub_depth_alt = self.create_subscription(Image, "/camera/depth/image_raw", self.depth_cb, 5)

        # 5. 定时高频融合触发 (5 Hz 定时融合，兼顾实时性与极低 CPU 开销)
        self.timer = self.create_timer(0.2, self.fuse_and_publish)

        self.get_logger().info(">>> [Joint 3D Mapping] 激光雷达 + 深度相机联合建图节点已启动！")
        self.get_logger().info("    TF 链路: map -> base_link -> laser (z=0.12m) & camera_link (x=0.08m, z=0.08m)")

    def broadcast_static_transforms(self):
        transforms = []
        now = self.get_clock().now().to_msg()

        # map -> base_link
        t_map_base = TransformStamped()
        t_map_base.header.stamp = now
        t_map_base.header.frame_id = "map"
        t_map_base.child_frame_id = "base_link"
        t_map_base.transform.translation.x = 0.0
        t_map_base.transform.translation.y = 0.0
        t_map_base.transform.translation.z = 0.0
        t_map_base.transform.rotation.w = 1.0
        transforms.append(t_map_base)

        # base_link -> laser (雷达安装在机身中心上方 12cm)
        t_base_laser = TransformStamped()
        t_base_laser.header.stamp = now
        t_base_laser.header.frame_id = "base_link"
        t_base_laser.child_frame_id = "laser"
        t_base_laser.transform.translation.x = 0.0
        t_base_laser.transform.translation.y = 0.0
        t_base_laser.transform.translation.z = 0.12
        t_base_laser.transform.rotation.w = 1.0
        transforms.append(t_base_laser)

        # base_link -> camera_link (相机安装在机身前端偏上 8cm 前凸、8cm 离地高)
        t_base_cam = TransformStamped()
        t_base_cam.header.stamp = now
        t_base_cam.header.frame_id = "base_link"
        t_base_cam.child_frame_id = "camera_link"
        t_base_cam.transform.translation.x = 0.08
        t_base_cam.transform.translation.y = 0.0
        t_base_cam.transform.translation.z = 0.08
        t_base_cam.transform.rotation.w = 1.0
        transforms.append(t_base_cam)

        self.tf_broadcaster.sendTransform(transforms)

    def scan_cb(self, msg):
        self.latest_scan = msg

    def info_cb(self, msg):
        if msg.k[0] > 0:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]

    def depth_cb(self, msg):
        # 只缓存，5Hz 融合时再反投影，避免 30Hz 在 CPU 上拼点云
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
            self.latest_depth = arr
        except Exception:
            pass

    def fuse_and_publish(self):
        if self.latest_scan is None and self.latest_depth is None:
            return

        t_start = time.time()
        lidar_pts = np.empty((0, 3), dtype=np.float32)
        cam_pts = np.empty((0, 3), dtype=np.float32)

        # 1. 提取并投影 2D LiDAR 点云 (转换到 base_link 坐标系，z=0.12m)
        if self.latest_scan is not None:
            ranges = np.array(self.latest_scan.ranges, dtype=np.float32)
            angles = np.arange(len(ranges), dtype=np.float32) * self.latest_scan.angle_increment + self.latest_scan.angle_min
            valid_mask = (ranges > self.latest_scan.range_min) & (ranges < self.latest_scan.range_max)
            r_valid = ranges[valid_mask]
            a_valid = angles[valid_mask]
            if len(r_valid) > 0:
                lx = r_valid * np.cos(a_valid)
                ly = r_valid * np.sin(a_valid)
                lz = np.full_like(lx, 0.12)
                lidar_pts = np.column_stack([lx, ly, lz])

        # 2. 5Hz 从深度图抽样反投影（步长 8，约 1/64 像素），不再吃 30Hz XYZRGB
        if self.latest_depth is not None:
            try:
                depth = self.latest_depth
                h, w = depth.shape
                step = 8
                ys = np.arange(0, h, step)
                xs = np.arange(0, w, step)
                grid_y, grid_x = np.meshgrid(ys, xs, indexing="ij")
                z = depth[grid_y, grid_x].astype(np.float32) / 1000.0
                valid = (z > 0.2) & (z < 6.0)
                z = z[valid]
                u = grid_x[valid].astype(np.float32)
                v = grid_y[valid].astype(np.float32)
                if z.size > 0:
                    x_opt = (u - self.cx) * z / self.fx
                    y_opt = (v - self.cy) * z / self.fy
                    cx = z + 0.08
                    cy = -x_opt
                    cz = -y_opt + 0.08
                    cam_pts = np.column_stack([cx, cy, cz])
            except Exception as e:
                self.get_logger().warn(f"深度反投影异常: {e}")

        # 3. 点云拼接与融合
        if len(lidar_pts) > 0 and len(cam_pts) > 0:
            fused_pts = np.vstack([lidar_pts, cam_pts])
        elif len(lidar_pts) > 0:
            fused_pts = lidar_pts
        elif len(cam_pts) > 0:
            fused_pts = cam_pts
        else:
            return

        # 4. 打包并发布融合点云
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_link"
        fused_msg = pc2.create_cloud_xyz32(header, fused_pts)
        self.pub_fused.publish(fused_msg)

        # 5. 诊断与性能统计
        self.fusion_count += 1
        now = time.time()
        if now - self.last_stat_time >= 1.0:
            self.fusion_hz = round(self.fusion_count / (now - self.last_stat_time), 1)
            self.fusion_count = 0
            self.last_stat_time = now

        z_min = float(np.min(fused_pts[:, 2]))
        z_max = float(np.max(fused_pts[:, 2]))
        status_info = {
            "hz": self.fusion_hz,
            "total_points": len(fused_pts),
            "lidar_points": len(lidar_pts),
            "camera_points": len(cam_pts),
            "height_range": [round(z_min, 2), round(z_max, 2)],
            "compute_time_ms": round((time.time() - t_start) * 1000, 2)
        }
        status_msg = String()
        status_msg.data = json.dumps(status_info)
        self.pub_status.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Joint3DMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
