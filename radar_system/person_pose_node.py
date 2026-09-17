#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RK3588 相机人体姿态检测与 RGB-D 测距
- 结合奥比中光 Astra S 3D 深度相机 (RGB-D)
- 运行 YOLOv8n-pose RKNN NPU 人体姿态检测（17 个关键点）
- 实时输出检测框并反投影计算空间 3D 物理坐标 [X, Y, Z] (单位: 米) 与真实直线距离
- 采用最新帧缓冲与异步推理，避免旧帧积压
- 发布 /camera/ai_detection/image 可视化话题
"""

import sys
import os
import time
import math
import threading
import cv2
import numpy as np

cv2.setNumThreads(2)
cv2.ocl.setUseOpenCL(False)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

DIR = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR = os.path.join(DIR, "models")

from pose_inference import PoseRKNN, draw_pose, KEYPOINT_NAMES
from depth_measurement import DepthMeasurement, decode_depth, decode_rgb


class PersonPoseNode(Node, DepthMeasurement):
    def __init__(self):
        super().__init__("person_pose_node")
        
        # 相机内参 (默认 Astra S 640x480 内参)
        self.fx = 570.3
        self.fy = 570.3
        self.cx = 319.5
        self.cy = 239.5
        
        # 加载 AI 模型
        self.yolo_model = os.environ.get("RK3588_POSE_MODEL", os.path.join(MODELS_DIR, "yolov8n_pose_rk3588_fp16.rknn"))
        self.yolo = PoseRKNN(self.yolo_model)
        self.get_logger().info(">>> YOLOv8n-pose / RK3588 NPU / person + 17 keypoints ready")

        self.latest_rgb = None
        self.latest_depth = None
        self.latest_depth_stamp = None
        self.latest_header = None
        self.frame_seq = 0
        self.lock = threading.Lock()
        self.running = True
        self._previous_frame = None
        self._frame_intervals = []

        # 发布者
        self.pub_annotated = self.create_publisher(Image, "/camera/ai_detection/image", 10)
        self.pub_json = self.create_publisher(String, "/camera/ai_detection/targets", 10)
        self.pub_status = self.create_publisher(String, "/camera/ai_detection/status", 1)
        
        # Latest-frame sensor mailboxes; never queue old camera frames.
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_cam_info = self.create_subscription(CameraInfo, "/camera/rgb/camera_info", self.cam_info_cb, sensor_qos)
        self.sub_depth = self.create_subscription(Image, "/camera/depth_raw/image", self.depth_cb, sensor_qos)
        self.sub_rgb = self.create_subscription(Image, "/camera/rgb/image_raw", self.rgb_cb, sensor_qos)

        # 启动异步推理工作线程
        self.worker = threading.Thread(target=self.inference_loop, daemon=True)
        self.worker.start()

        self.get_logger().info(">>> 人体姿态测距节点已就绪，正在监听传感器话题...")

    def cam_info_cb(self, msg):
        if msg.k[0] > 0:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]

    def depth_cb(self, msg):
        try:
            depth_arr = decode_depth(msg)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            with self.lock:
                self.latest_depth = depth_arr
                self.latest_depth_stamp = stamp
        except Exception:
            pass

    def rgb_cb(self, msg):
        try:
            rgb_arr = decode_rgb(msg)
            with self.lock:
                self.latest_rgb = rgb_arr
                self.latest_header = msg.header
                self.frame_seq += 1
        except Exception:
            pass

    def inference_loop(self):
        last_seq = -1
        last_frame_time = time.monotonic()
        has_received_first_frame = False
        while self.running:
            with self.lock:
                seq = self.frame_seq
                rgb = None if self.latest_rgb is None else self.latest_rgb
                depth = self.latest_depth
                depth_stamp = self.latest_depth_stamp
                header = self.latest_header

            if rgb is not None:
                has_received_first_frame = True

            if rgb is None or seq == last_seq:
                if has_received_first_frame and (time.monotonic() - last_frame_time > 6.0):
                    self.get_logger().warn("超过 6 秒未收到相机图像帧（可能相机服务已重启），退出以便 systemd 自动重连 DDS...")
                    os._exit(1)
                time.sleep(0.01)
                continue
            last_seq = seq
            last_frame_time = time.monotonic()

            t_start = time.time()
            try:
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                h, w = bgr.shape[:2]

                # RGB 与深度来自两个独立回调,没有硬件同步。相差太多时
                # 像素位置对不上,bbox 会框到错误的深度区域 —— 宁可不给距离。
                rgb_stamp = None
                if header is not None:
                    rgb_stamp = header.stamp.sec + header.stamp.nanosec / 1e9
                depth_ok = False
                pair_skew = None
                if rgb_stamp and depth_stamp:
                    pair_skew = abs(rgb_stamp - depth_stamp)
                    depth_ok = (pair_skew <= self.DEPTH_MAX_PAIR_AGE_S
                                and depth is not None and depth.shape == (h, w))

                detections = self.yolo.infer(bgr)

                target_list = []
                ranged_count = 0
                unranged_people = 0
                # 3. 3D 逆投影几何空间测距
                for detection in detections:
                    label, conf = detection['label'], detection['conf']
                    x1, y1, x2, y2 = detection['box']
                    keypoints = detection['keypoints']
                    color = (0, 242, 254)
                    draw_pose(bgr, keypoints)
                    if x2 <= x1 or y2 <= y1:
                        continue
                    
                    xc = (x1 + x2) // 2
                    yc = (y1 + y2) // 2

                    dist_m = None
                    X = Y = Z = 0.0
                    depth_ratio = 0.0

                    if depth is not None and depth.shape == (h, w) and depth_ok:
                        Zq, depth_ratio = self.robust_depth(depth, x1, y1, x2, y2)
                        if Zq is not None:
                            Z = Zq
                            X = (xc - self.cx) * Z / self.fx
                            Y = (yc - self.cy) * Z / self.fy
                            dist_m = math.sqrt(X*X + Y*Y + Z*Z)

                    # 绘制矩形框与中心准星
                    cv2.rectangle(bgr, (x1, y1), (x2, y2), color, 2)
                    cv2.drawMarker(bgr, (xc, yc), color, cv2.MARKER_CROSS, 12, 2)

                    # 2D 人体检测与 3D 深度是否有效是两件事。旧逻辑在深度图
                    # 有空洞时把已经识别到的人整个丢掉，跟随器因此长期显示
                    # “无人”。现在始终发布人体的 2D 方位；若 Astra 深度
                    # 无效，跟随器可用同方位激光雷达距离完成安全兜底。
                    bearing_rad = math.atan2(-(xc - self.cx), max(self.fx, 1.0))
                    item = {
                        "label": label,
                        "keypoints": keypoints,
                        "conf": round(conf, 2),
                        "x1": int(x1),
                        "y1": int(y1),
                        "x2": int(x2),
                        "y2": int(y2),
                        "bearing_rad": round(bearing_rad, 5),
                        "stamp": round(rgb_stamp, 4) if rgb_stamp else None,
                        "depth_ratio": round(depth_ratio, 3),
                        "range_valid": False,
                    }

                    if dist_m is not None and 0.2 < dist_m < 12.0:
                        dist_str = f"{dist_m:.2f}m"
                        coord_str = f"X:{X:+.2f} Y:{Y:+.2f} Z:{Z:.2f}m"
                        tag_text = f"[{label}] {dist_str} | {coord_str}"
                        item.update({
                            "distance": round(dist_m, 2),
                            "x": round(X, 2),
                            "y": round(Y, 2),
                            "z": round(Z, 2),
                            "range_valid": True,
                        })
                        target_list.append(item)
                        ranged_count += 1
                    else:
                        tag_text = f"[{label}] (Out of Range)"
                        if label.lower() == "person":
                            target_list.append(item)
                            unranged_people += 1

                    (tw, th), _ = cv2.getTextSize(tag_text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
                    cv2.rectangle(bgr, (x1, max(0, y1 - 22)), (x1 + tw + 8, y1), (15, 23, 42), -1)
                    cv2.rectangle(bgr, (x1, max(0, y1 - 22)), (x1 + tw + 8, y1), color, 1)
                    cv2.putText(bgr, tag_text, (x1 + 4, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

                # 4. 顶部状态栏
                now = time.monotonic()
                if self._previous_frame is not None:
                    self._frame_intervals.append(now-self._previous_frame)
                    self._frame_intervals = self._frame_intervals[-30:]
                self._previous_frame = now
                fps = round(len(self._frame_intervals)/sum(self._frame_intervals), 1) if self._frame_intervals else 0.0
                hud_text = f"YOLOv8n-pose / NPU | {fps} FPS | Targets: {len(detections)}"
                cv2.putText(bgr, hud_text, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 242, 254), 1, cv2.LINE_AA)

                # 5. 发布带人体骨架与测距标注的图像
                rgb_out = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                out_msg = Image()
                if header:
                    out_msg.header = header
                out_msg.height = h
                out_msg.width = w
                out_msg.encoding = "rgb8"
                out_msg.is_bigendian = 0
                out_msg.step = w * 3
                out_msg.data = rgb_out.tobytes()
                self.pub_annotated.publish(out_msg)

                # 发布 JSON 目标数据
                import json
                json_msg = String()
                json_msg.data = json.dumps(target_list)
                self.pub_json.publish(json_msg)
                status = String()
                status.data = json.dumps(dict(model="YOLOv8n-pose", backend="RKNN NPU", classes=1,
                                              keypoint_names=KEYPOINT_NAMES,
                                              pipeline_ms=round(self.yolo.last_total_ms, 1),
                                              fps=fps, inference_ms=round(self.yolo.last_inference_ms, 1),
                                              detected=len(detections), ranged=ranged_count,
                                              unranged_people=unranged_people,
                                              depth_ok=depth_ok,
                                              pair_skew_ms=round(pair_skew * 1000, 1) if pair_skew is not None else None))
                self.pub_status.publish(status)

            except Exception as e:
                self.get_logger().warn(f"AI推理异常: {e}")

            leftover = 0.05 - (time.time() - t_start)
            if leftover > 0:
                time.sleep(leftover)

def main(args=None):
    rclpy.init(args=args)
    node = PersonPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.worker.join()
        node.yolo.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
