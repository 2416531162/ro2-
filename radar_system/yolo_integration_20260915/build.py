from pathlib import Path
import ast

p = Path(__file__).resolve().parent
source = (p/'BASELINE.py').read_text()
core = (p/'yolo_core.py').read_text()
source = source.replace('- 运行 MobileNet-SSD 与 ResNet-10 深度人脸神经网络',
                        '- 运行 YOLOv8n RKNN NPU 目标检测与 ResNet-10 人脸检测')
source = source.replace('class AI3DDetectorNode(Node):', core+'\n\nclass AI3DDetectorNode(Node):')
source = source.replace('        self.ssd_proto = os.path.join(MODELS_DIR, "MobileNetSSD_deploy.prototxt")\n'
                        '        self.ssd_model = os.path.join(MODELS_DIR, "MobileNetSSD_deploy.caffemodel")',
                        '        self.yolo_model = os.environ.get("RK3588_YOLO_MODEL", os.path.join(MODELS_DIR, "yolov8n_rk3588_fp16.rknn"))')
source = source.replace('        self.ssd_net = cv2.dnn.readNetFromCaffe(self.ssd_proto, self.ssd_model)',
                        '        self.yolo = YoloRKNN(self.yolo_model)')
source = source.replace('>>> AI 目标与人脸神经网络加载成功！', '>>> YOLOv8n / RK3588 NPU / 80 classes + face detector ready')
start = source.index('                blob_ssd = ')
end = source.index('                target_list = []', start)
source = source[:start] + '                detections.extend(self.yolo.infer_boxes(bgr))\n\n' + source[end:]
source = source.replace('        self.tick = 0', '        self.tick = 0\n        self._previous_frame = None\n        self._frame_intervals = []')
source = source.replace('        self.pub_json = self.create_publisher(String, "/camera/ai_detection/targets", 10)',
                        '        self.pub_json = self.create_publisher(String, "/camera/ai_detection/targets", 10)\n'
                        '        self.pub_status = self.create_publisher(String, "/camera/ai_detection/status", 1)')
source = source.replace('                fps = round(1.0 / max(1e-4, t_used), 1)',
                        '                now = time.monotonic()\n'
                        '                if self._previous_frame is not None:\n'
                        '                    self._frame_intervals.append(now-self._previous_frame)\n'
                        '                    self._frame_intervals = self._frame_intervals[-30:]\n'
                        '                self._previous_frame = now\n'
                        '                fps = round(len(self._frame_intervals)/sum(self._frame_intervals), 1) if self._frame_intervals else 0.0')
source = source.replace('RK3588 AI 3D Spatial | {fps} FPS | Targets: {len(detections)}',
                        'YOLOv8n / NPU | {fps} FPS | Targets: {len(detections)}')
source = source.replace('                self.pub_json.publish(json_msg)',
                        '                self.pub_json.publish(json_msg)\n'
                        '                status = String()\n'
                        '                status.data = json.dumps(dict(model="YOLOv8n", backend="RKNN NPU", classes=80,\n'
                        '                                              fps=fps, inference_ms=round(self.yolo.last_inference_ms, 1),\n'
                        '                                              detected=len(detections), ranged=len(target_list)))\n'
                        '                self.pub_status.publish(status)')
source = source.replace('            leftover = 0.10 - ', '            leftover = 0.05 - ')
source = source.replace('        node.running = False\n        node.destroy_node()',
                        '        node.running = False\n        node.worker.join()\n        node.yolo.close()\n        node.destroy_node()')
ast.parse(source)
(p/'MODIFIED_FILE.py').write_text(source)
print('BUILT_YOLO_NODE', len(source.encode()), 'bytes')
