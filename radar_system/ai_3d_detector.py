#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RK3588 实时 AI + 3D 物理空间测距定位系统
- 结合奥比中光 Astra S 3D 深度相机 (RGB-D)
- 运行 YOLOv8n RKNN NPU 目标检测与 ResNet-10 人脸检测
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

"""Build-time source for YOLOv8 RKNN inference; embedded in the deployed node."""

YOLO_CLASSES = (
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck',
    'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench',
    'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra',
    'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
    'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
    'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
    'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
    'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
    'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
    'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
    'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
    'hair drier', 'toothbrush',
)


def yolo_letterbox(bgr, size=640):
    """Preserve aspect ratio and retain the exact padding used by the model."""
    h, w = bgr.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = round(w * scale), round(h * scale)
    left, top = (size - nw) // 2, (size - nh) // 2
    resized = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
    padded = np.full((size, size, 3), 114, dtype=np.uint8)
    padded[top:top+nh, left:left+nw] = resized
    rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
    return np.ascontiguousarray(rgb[None]), (scale, left, top, w, h)


def yolo_class_nms(boxes, scores, classes, threshold=0.45, limit=100):
    """Suppress overlaps only inside the same class, with stable score ordering."""
    kept = []
    for cls in np.unique(classes):
        indices = np.flatnonzero(classes == cls)
        indices = indices[np.argsort(-scores[indices], kind='stable')]
        while indices.size:
            i = int(indices[0])
            kept.append(i)
            rest = indices[1:]
            if not rest.size:
                break
            lo = np.maximum(boxes[i, :2], boxes[rest, :2])
            hi = np.minimum(boxes[i, 2:], boxes[rest, 2:])
            overlap = np.maximum(hi - lo, 0).prod(axis=1)
            area_i = np.maximum(boxes[i, 2:] - boxes[i, :2], 0).prod()
            areas = np.maximum(boxes[rest, 2:] - boxes[rest, :2], 0).prod(axis=1)
            iou = overlap / np.maximum(area_i + areas - overlap, 1e-9)
            indices = rest[iou <= threshold]
    return sorted(kept, key=lambda i: (-float(scores[i]), i))[:limit]


def yolo_postprocess(outputs, transform, confidence=0.40, iou=0.45, size=640):
    """Decode six raw YOLOv8 heads: box DFL logits / class logits per scale."""
    if outputs is None or len(outputs) != 6:
        raise ValueError('YOLO model must expose six raw detection heads')
    all_boxes, all_scores, all_classes = [], [], []
    for branch in range(3):
        regression = np.asarray(outputs[2*branch], dtype=np.float32)
        logits = np.asarray(outputs[2*branch+1], dtype=np.float32)
        if (regression.ndim != 4 or regression.shape[:2] != (1, 64)
                or logits.shape != (1, 80, *regression.shape[2:])):
            raise ValueError('Unexpected YOLOv8 head dimensions')
        _, _, gh, gw = regression.shape
        raw_scores = logits[0].reshape(80, -1).T
        classes = raw_scores.argmax(axis=1)
        best_logits = raw_scores[np.arange(len(classes)), classes]
        scores = 1.0 / (1.0 + np.exp(-np.clip(best_logits, -80, 80)))
        selected = np.flatnonzero(np.isfinite(scores) & (scores >= confidence))
        if not selected.size:
            continue
        dfl = regression[0].reshape(4, 16, -1)[:, :, selected]
        dfl -= dfl.max(axis=1, keepdims=True)
        distribution = np.exp(dfl)
        distribution /= distribution.sum(axis=1, keepdims=True)
        distances = (distribution * np.arange(16, dtype=np.float32)[None, :, None]).sum(axis=1).T
        centers = np.column_stack((selected % gw + .5, selected // gw + .5))
        stride = np.array([size / gw, size / gh], dtype=np.float32)
        boxes = np.column_stack(((centers-distances[:, :2])*stride,
                                 (centers+distances[:, 2:])*stride))
        all_boxes.append(boxes)
        all_scores.append(scores[selected])
        all_classes.append(classes[selected])
    if not all_boxes:
        return []
    boxes = np.concatenate(all_boxes)
    scores = np.concatenate(all_scores)
    classes = np.concatenate(all_classes)
    scale, left, top, w, h = transform
    boxes[:, [0, 2]] = (boxes[:, [0, 2]] - left) / scale
    boxes[:, [1, 3]] = (boxes[:, [1, 3]] - top) / scale
    boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, w-1)
    boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, h-1)
    valid = np.isfinite(boxes).all(axis=1) & (boxes[:, 2] > boxes[:, 0]) & (boxes[:, 3] > boxes[:, 1])
    boxes, scores, classes = boxes[valid], scores[valid], classes[valid]
    keep = yolo_class_nms(boxes, scores, classes, iou)
    return [(YOLO_CLASSES[int(classes[i])].capitalize(), float(scores[i]),
             *np.rint(boxes[i]).astype(int).tolist(), (0, 242, 254)) for i in keep]


class YoloRKNN:
    def __init__(self, model_path):
        from rknnlite.api import RKNNLite
        self.runtime = RKNNLite(verbose=False)
        try:
            if self.runtime.load_rknn(model_path) != 0:
                raise RuntimeError('YOLOv8n RKNN model load failed')
            if self.runtime.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2) != 0:
                raise RuntimeError('RK3588 NPU initialization failed')
        except Exception:
            self.runtime.release()
            raise
        self.last_inference_ms = 0.0

    def infer_boxes(self, bgr):
        image, transform = yolo_letterbox(bgr)
        started = time.monotonic()
        outputs = self.runtime.inference(inputs=[image], data_format=['nhwc'])
        self.last_inference_ms = (time.monotonic()-started)*1000
        return yolo_postprocess(outputs, transform)

    def close(self):
        self.runtime.release()


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
        self.yolo_model = os.environ.get("RK3588_YOLO_MODEL", os.path.join(MODELS_DIR, "yolov8n_rk3588_fp16.rknn"))
        
        self.get_logger().info("正在加载 AI 视觉模型...")
        self.face_net = cv2.dnn.readNetFromCaffe(self.face_proto, self.face_model)
        self.yolo = YoloRKNN(self.yolo_model)
        self.get_logger().info(">>> YOLOv8n / RK3588 NPU / 80 classes + face detector ready")

        self.latest_rgb = None
        self.latest_depth = None
        self.latest_header = None
        self.frame_seq = 0
        self.lock = threading.Lock()
        self.running = True
        self.tick = 0
        self._previous_frame = None
        self._frame_intervals = []

        # 发布者
        self.pub_annotated = self.create_publisher(Image, "/camera/ai_detection/image", 10)
        self.pub_json = self.create_publisher(String, "/camera/ai_detection/targets", 10)
        self.pub_status = self.create_publisher(String, "/camera/ai_detection/status", 1)
        
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

                detections.extend(self.yolo.infer_boxes(bgr))

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
                now = time.monotonic()
                if self._previous_frame is not None:
                    self._frame_intervals.append(now-self._previous_frame)
                    self._frame_intervals = self._frame_intervals[-30:]
                self._previous_frame = now
                fps = round(len(self._frame_intervals)/sum(self._frame_intervals), 1) if self._frame_intervals else 0.0
                hud_text = f"YOLOv8n / NPU | {fps} FPS | Targets: {len(detections)}"
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
                status = String()
                status.data = json.dumps(dict(model="YOLOv8n", backend="RKNN NPU", classes=80,
                                              fps=fps, inference_ms=round(self.yolo.last_inference_ms, 1),
                                              detected=len(detections), ranged=len(target_list)))
                self.pub_status.publish(status)

            except Exception as e:
                self.get_logger().warn(f"AI推理异常: {e}")

            leftover = 0.05 - (time.time() - t_start)
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
        node.worker.join()
        node.yolo.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
