#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RK3588 实时 AI + 3D 物理空间测距定位系统
- 结合奥比中光 Astra S 3D 深度相机 (RGB-D)
- 运行 MobileNet-SSD 与 ResNet-10 深度人脸神经网络
- 实时输出检测框并反投影计算空间 3D 物理坐标 [X, Y, Z] (单位: 米) 与真实直线距离
- 采用帧缓冲与多线程异步推理引擎，保证 0 阻塞、超流畅运行
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
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

DIR = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR = os.path.join(DIR, "models")

CLASSES = [
    "background", "aeroplane", "bicycle", "bird", "boat",
    "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
    "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
    "sofa", "train", "tvmonitor"
]

CLASS_NAMES_CN = {
    "person": "人", "chair": "椅子", "bottle": "瓶子", "sofa": "沙发",
    "tvmonitor": "显示屏", "diningtable": "桌子", "pottedplant": "盆栽",
    "bicycle": "自行车", "car": "汽车", "cat": "猫", "dog": "狗"
}

class AI3DDetectorNode(Node):
    def __init__(self):
        super().__init__("ai_3d_detector_node")
        
        # 相机内参 (默认 Astra S 640x480 内参)
        self.fx = 570.3
        self.fy = 570.3
        self.cx = 319.5
        self.cy = 239.5
        
        # 加载 AI 模型
        self.face_proto = os.path.join(MODELS_DIR, "face_deploy.prototxt")
        self.face_model = os.path.join(MODELS_DIR, "res10_300x300_ssd_iter_140000_fp16.caffemodel")
        self.ssd_proto = os.path.join(MODELS_DIR, "MobileNetSSD_deploy.prototxt")
        self.ssd_model = os.path.join(MODELS_DIR, "MobileNetSSD_deploy.caffemodel")
        
        self.get_logger().info("正在加载 AI 视觉模型...")
        self.face_net = cv2.dnn.readNetFromCaffe(self.face_proto, self.face_model)
        self.ssd_net = cv2.dnn.readNetFromCaffe(self.ssd_proto, self.ssd_model)
        self.get_logger().info(">>> AI 目标与人脸神经网络加载成功！")

        self.latest_rgb = None
        self.latest_depth = None
        self.latest_header = None
        self.frame_seq = 0
        self.lock = threading.Lock()
        self.running = True
        self.tick = 0

        # 发布者
        self.pub_annotated = self.create_publisher(Image, "/camera/ai_detection/image", 10)
        self.pub_json = self.create_publisher(String, "/camera/ai_detection/targets", 10)
        
        # 订阅者
        self.sub_cam_info = self.create_subscription(CameraInfo, "/camera/rgb/camera_info", self.cam_info_cb, 5)
        self.sub_depth = self.create_subscription(Image, "/camera/depth_raw/image", self.depth_cb, 10)
        self.sub_rgb = self.create_subscription(Image, "/camera/rgb/image_raw", self.rgb_cb, 10)

        # 启动异步推理工作线程
        self.worker = threading.Thread(target=self.inference_loop, daemon=True)
        self.worker.start()

        self.get_logger().info(">>> AI + 3D 空间测距节点已就绪，正在监听传感器话题...")

    def cam_info_cb(self, msg):
        if msg.k[0] > 0:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]

    def depth_cb(self, msg):
        try:
            depth_arr = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
            with self.lock:
                self.latest_depth = depth_arr
        except Exception:
            pass

    def rgb_cb(self, msg):
        try:
            rgb_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3)).copy()
            with self.lock:
                self.latest_rgb = rgb_arr
                self.latest_header = msg.header
                self.frame_seq += 1
        except Exception:
            pass

    def inference_loop(self):
        last_seq = -1
        while self.running:
            with self.lock:
                seq = self.frame_seq
                rgb = None if self.latest_rgb is None else self.latest_rgb
                depth = self.latest_depth
                header = self.latest_header

            if rgb is None or seq == last_seq:
                time.sleep(0.01)
                continue
            last_seq = seq

            t_start = time.time()
            try:
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                h, w = bgr.shape[:2]

                detections = []
                self.tick += 1

                # 人脸网较重，隔帧跑；物体检测每帧跑
                if self.tick % 3 == 0:
                    blob_face = cv2.dnn.blobFromImage(bgr, 1.0, (300, 300), (104.0, 177.0, 123.0))
                    self.face_net.setInput(blob_face)
                    face_out = self.face_net.forward()
                    for i in range(face_out.shape[2]):
                        conf = float(face_out[0, 0, i, 2])
                        if conf > 0.52:
                            box = face_out[0, 0, i, 3:7] * np.array([w, h, w, h])
                            x1, y1, x2, y2 = box.astype("int")
                            detections.append(("Face", conf, max(0, x1), max(0, y1), min(w-1, x2), min(h-1, y2), (255, 191, 0)))

                blob_ssd = cv2.dnn.blobFromImage(bgr, 0.007843, (300, 300), 127.5)
                self.ssd_net.setInput(blob_ssd)
                ssd_out = self.ssd_net.forward()

                for i in range(ssd_out.shape[2]):
                    conf = float(ssd_out[0, 0, i, 2])
                    if conf > 0.40:
                        idx = int(ssd_out[0, 0, i, 1])
                        if idx < len(CLASSES):
                            raw_name = CLASSES[idx]
                            if raw_name in ["background"]:
                                continue
                            display_name = raw_name.capitalize()
                            box = ssd_out[0, 0, i, 3:7] * np.array([w, h, w, h])
                            x1, y1, x2, y2 = box.astype("int")
                            detections.append((display_name, conf, max(0, x1), max(0, y1), min(w-1, x2), min(h-1, y2), (0, 242, 254)))

                target_list = []
                # 3. 3D 逆投影几何空间测距
                for label, conf, x1, y1, x2, y2, color in detections:
                    if x2 <= x1 or y2 <= y1:
                        continue
                    
                    xc = (x1 + x2) // 2
                    yc = (y1 + y2) // 2

                    dist_m = None
                    X = Y = Z = 0.0

                    if depth is not None:
                        half = 8
                        ry1, ry2 = max(0, yc - half), min(h, yc + half)
                        rx1, rx2 = max(0, xc - half), min(w, xc + half)
                        roi = depth[ry1:ry2, rx1:rx2]
                        valid = roi[roi > 150]
                        if len(valid) > 4:
                            med_mm = float(np.median(valid))
                            Z = med_mm / 1000.0
                            X = (xc - self.cx) * Z / self.fx
                            Y = (yc - self.cy) * Z / self.fy
                            dist_m = math.sqrt(X*X + Y*Y + Z*Z)

                    # 绘制矩形框与中心准星
                    cv2.rectangle(bgr, (x1, y1), (x2, y2), color, 2)
                    cv2.drawMarker(bgr, (xc, yc), color, cv2.MARKER_CROSS, 12, 2)

                    if dist_m is not None and 0.2 < dist_m < 12.0:
                        dist_str = f"{dist_m:.2f}m"
                        coord_str = f"X:{X:+.2f} Y:{Y:+.2f} Z:{Z:.2f}m"
                        tag_text = f"[{label}] {dist_str} | {coord_str}"
                        target_list.append({
                            "label": label,
                            "conf": round(conf, 2),
                            "distance": round(dist_m, 2),
                            "x": round(X, 2),
                            "y": round(Y, 2),
                            "z": round(Z, 2),
                            "x1": int(x1),
                            "y1": int(y1),
                            "x2": int(x2),
                            "y2": int(y2),
                        })
                    else:
                        tag_text = f"[{label}] (Out of Range)"

                    (tw, th), _ = cv2.getTextSize(tag_text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
                    cv2.rectangle(bgr, (x1, max(0, y1 - 22)), (x1 + tw + 8, y1), (15, 23, 42), -1)
                    cv2.rectangle(bgr, (x1, max(0, y1 - 22)), (x1 + tw + 8, y1), color, 1)
                    cv2.putText(bgr, tag_text, (x1 + 4, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

                # 4. 顶部状态栏
                t_used = time.time() - t_start
                fps = round(1.0 / max(1e-4, t_used), 1)
                hud_text = f"RK3588 AI 3D Spatial | {fps} FPS | Targets: {len(detections)}"
                cv2.putText(bgr, hud_text, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 242, 254), 1, cv2.LINE_AA)

                # 5. 发布带 3D 标注的图像
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

            except Exception as e:
                self.get_logger().warn(f"AI推理异常: {e}")

            leftover = 0.10 - (time.time() - t_start)
            if leftover > 0:
                time.sleep(leftover)

def main(args=None):
    rclpy.init(args=args)
    node = AI3DDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
