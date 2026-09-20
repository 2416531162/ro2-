#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Jetson 相机人物检测与 RGB-D 测距
- 结合奥比中光 Astra S / Mini 3D 深度相机 (RGB-D)
- 运行 YOLO26s CUDA / TensorRT，仅检测 person
- 实时输出检测框并反投影计算空间 3D 物理坐标 [X, Y, Z] (单位: 米) 与真实直线距离
- 最新帧邮箱与事件驱动推理，限制最大 30 FPS，不积压旧帧
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

from person_detection import PersonYOLO
from pose_inference import PoseRKNN, PoseONNX, draw_pose, KEYPOINT_NAMES
from depth_measurement import DepthMeasurement, decode_depth, decode_rgb


class PersonPoseNode(Node, DepthMeasurement):
    MAX_FPS = 30.0
    ANNOTATED_PERIOD_S = 0.20

    def __init__(self):
        super().__init__("person_pose_node")
        
        # 相机内参 (默认 Astra S 640x480 内参)
        self.fx = 570.3
        self.fy = 570.3
        self.cx = 319.5
        self.cy = 239.5
        
        # 加载 AI 模型
        self.yolo_model = os.environ.get("RK3588_POSE_MODEL", "models/yolo26s.pt")
        if not os.path.isabs(self.yolo_model):
            self.yolo_model = os.path.abspath(os.path.join(DIR, self.yolo_model))
        suffix = os.path.splitext(self.yolo_model)[1].lower()
        if suffix in ('.pt', '.engine'):
            self.yolo = PersonYOLO(self.yolo_model)
        elif suffix == '.onnx':
            self.yolo = PoseONNX(self.yolo_model)
        elif suffix == '.rknn':
            self.yolo = PoseRKNN(self.yolo_model)
        else:
            raise ValueError(f'Unsupported RK3588_POSE_MODEL suffix {suffix!r}: {self.yolo_model}; expected .pt, .engine, .onnx or .rknn')
        self.pose_outputs = suffix in ('.onnx', '.rknn')
        self.model_name = os.path.splitext(os.path.basename(self.yolo_model))[0]
        self.backend_name = getattr(self.yolo, 'backend_name', suffix[1:].upper())
        self.get_logger().info(f">>> {self.model_name} / {self.backend_name} / {self.yolo_model} / person-only ready")

        self.frame_lock = threading.Lock()
        self.pending_rgb_msg = None
        self.pending_depth_msg = None
        self.frame_event = threading.Event()
        self.running = True
        self._previous_frame = None
        self._frame_intervals = []
        self._last_annotated = 0.0

        # 发布者
        self.pub_annotated = self.create_publisher(Image, "/camera/ai_detection/image", 10)
        self.pub_json = self.create_publisher(String, "/camera/ai_detection/targets", 10)
        self.pub_status = self.create_publisher(String, "/camera/ai_detection/status", 1)
        
        # Camera publisher offers RELIABLE QoS; matching RELIABLE with small depth avoids DDS frame dropping.
        sensor_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.sub_cam_info = self.create_subscription(CameraInfo, "/camera/rgb/camera_info", self.cam_info_cb, sensor_qos)
        self.sub_depth = self.create_subscription(Image, "/camera/depth_raw/image", self.depth_cb, sensor_qos)
        self.sub_rgb = self.create_subscription(Image, "/camera/rgb/image_raw", self.rgb_cb, sensor_qos)

        # 启动异步推理工作线程
        self.worker = threading.Thread(target=self.inference_loop, daemon=True)
        self.worker.start()

        self.get_logger().info(">>> 人物检测测距节点已就绪，正在监听传感器话题...")

    def cam_info_cb(self, msg):
        if msg.k[0] > 0:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]

    def depth_cb(self, msg):
        with self.frame_lock:
            self.pending_depth_msg = msg

    def rgb_cb(self, msg):
        with self.frame_lock:
            self.pending_rgb_msg = msg
            self.frame_event.set()

    def inference_loop(self):
        last_frame_time = time.monotonic()
        has_received_first_frame = False
        next_frame_at = 0.0
        while self.running:
            if not self.frame_event.wait(timeout=0.1):
                if has_received_first_frame and (time.monotonic() - last_frame_time > 6.0):
                    self.get_logger().warn("超过 6 秒未收到相机图像帧（可能相机服务已重启），退出以便 systemd 自动重连 DDS...")
                    os._exit(1)
                continue
            remaining = next_frame_at - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            with self.frame_lock:
                rgb_msg = self.pending_rgb_msg
                depth_msg = self.pending_depth_msg
                self.pending_rgb_msg = None
                self.frame_event.clear()
            if rgb_msg is None:
                continue

            has_received_first_frame = True
            last_frame_time = time.monotonic()
            cycle_start = last_frame_time
            next_frame_at = last_frame_time + 1.0 / self.MAX_FPS

            try:
                rgb = decode_rgb(rgb_msg)
            except Exception:
                continue
            header = rgb_msg.header

            depth = None
            depth_stamp = None
            if depth_msg is not None:
                try:
                    depth = decode_depth(depth_msg)
                    depth_stamp = depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec / 1e9
                except Exception:
                    pass

            try:
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                h, w = bgr.shape[:2]

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
                annotate = (self.pub_annotated.get_subscription_count() > 0 and
                            time.monotonic() - self._last_annotated >= self.ANNOTATED_PERIOD_S)

                # 3. 3D 逆投影几何空间测距
                for detection in detections:
                    label, conf = detection['label'], detection['conf']
                    x1, y1, x2, y2 = detection['box']
                    keypoints = detection.get('keypoints')
                    if annotate and keypoints:
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

                    bearing_rad = math.atan2(-(xc - self.cx), max(self.fx, 1.0))
                    item = {
                        "label": label,
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
                    if keypoints:
                        item["keypoints"] = keypoints

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
                        if label.lower() == "person" and conf >= 0.25:
                            target_list.append(item)
                            unranged_people += 1

                    if annotate:
                        color = (0, 242, 254)
                        cv2.rectangle(bgr, (x1, y1), (x2, y2), color, 2)
                        cv2.drawMarker(bgr, (xc, yc), color, cv2.MARKER_CROSS, 12, 2)
                        (tw, th), _ = cv2.getTextSize(tag_text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
                        cv2.rectangle(bgr, (x1, max(0, y1 - 22)), (x1 + tw + 8, y1), (15, 23, 42), -1)
                        cv2.rectangle(bgr, (x1, max(0, y1 - 22)), (x1 + tw + 8, y1), color, 1)
                        cv2.putText(bgr, tag_text, (x1 + 4, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

                # 4. 计算稳定平滑 FPS
                now = time.monotonic()
                if self._previous_frame is not None:
                    self._frame_intervals.append(now - self._previous_frame)
                    self._frame_intervals = self._frame_intervals[-30:]
                self._previous_frame = now
                if self._frame_intervals:
                    total_dur = sum(self._frame_intervals)
                    fps = round(len(self._frame_intervals) / total_dur, 1) if total_dur > 0 else 0.0
                else:
                    fps = 0.0

                # 标注图降频发布，目标和状态仍逐帧发布。
                if annotate:
                    hud_text = f"{self.model_name} / {self.backend_name} | {fps} FPS | Targets: {len(detections)}"
                    cv2.putText(bgr, hud_text, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 242, 254), 1, cv2.LINE_AA)
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
                    self._last_annotated = now

                # 发布 JSON 目标数据
                import json
                json_msg = String()
                json_msg.data = json.dumps(target_list)
                self.pub_json.publish(json_msg)
                status = String()
                status.data = json.dumps(dict(model=self.model_name, backend=self.backend_name, classes=1,
                                              keypoint_names=KEYPOINT_NAMES if self.pose_outputs else [],
                                              pipeline_ms=round(self.yolo.last_total_ms, 1),
                                              cycle_ms=round((time.monotonic() - cycle_start) * 1000, 1),
                                              fps=fps, inference_ms=round(self.yolo.last_inference_ms, 1),
                                              detected=len(detections), ranged=ranged_count,
                                              unranged_people=unranged_people,
                                              depth_ok=depth_ok,
                                              pair_skew_ms=round(pair_skew * 1000, 1) if pair_skew is not None else None))
                self.pub_status.publish(status)

            except Exception as e:
                self.get_logger().warn(f"AI推理异常: {e}")

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
