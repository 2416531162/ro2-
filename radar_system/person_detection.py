"""Person-only Ultralytics detector for Jetson CUDA and TensorRT."""

import os
import time

import numpy as np


INPUT_SIZE = 512
PERSON_CONFIDENCE = 0.20


class PersonYOLO:
    def __init__(self, model_path):
        if not os.path.isfile(model_path):
            raise FileNotFoundError(f'Person detector model missing: {model_path}')
        suffix = os.path.splitext(model_path)[1].lower()
        if suffix not in ('.pt', '.engine'):
            raise ValueError(f'Expected YOLO detection .pt or .engine: {model_path}')
        try:
            import torch
            from ultralytics import YOLO
        except (ImportError, OSError) as exc:
            raise RuntimeError(f'Jetson CUDA torch/torchvision or ultralytics missing/incompatible: {exc}') from exc
        if not torch.cuda.is_available():
            raise RuntimeError(f'CUDA unavailable for person detector: {model_path}')
        if suffix == '.engine':
            try:
                import tensorrt
            except (ImportError, OSError) as exc:
                raise RuntimeError(f'TensorRT Python runtime missing/incompatible for {model_path}: {exc}') from exc

        self.backend_name = 'TensorRT CUDA' if suffix == '.engine' else 'PyTorch CUDA FP16'
        self.model = YOLO(model_path, task='detect' if suffix == '.engine' else None)
        if self.model.task != 'detect':
            raise ValueError(f'Expected a detection model, got {self.model.task}: {model_path}')
        self.last_inference_ms = self.last_total_ms = 0.0
        # Fail before subscribing if the engine/runtime or output contract is wrong.
        self.infer(np.zeros((480, 640, 3), dtype=np.uint8))

    def infer(self, bgr):
        start = time.monotonic()
        results = self.model.predict(source=bgr, device=0, imgsz=INPUT_SIZE,
                                     half=self.backend_name != 'TensorRT CUDA',
                                     conf=PERSON_CONFIDENCE, classes=[0],
                                     iou=.45, max_det=10, verbose=False)
        if len(results) != 1 or results[0].boxes is None:
            raise ValueError('Person detector did not return boxes')
        result = results[0]
        if result.keypoints is not None or str(result.names[0]).lower() != 'person':
            raise ValueError('Expected a COCO person detection model, not a pose/custom model')
        boxes = result.boxes.xyxy.detach().cpu().numpy()
        scores = result.boxes.conf.detach().cpu().numpy()
        if boxes.shape != (len(scores), 4):
            raise ValueError('Person detector returned invalid boxes')

        h, w = bgr.shape[:2]
        detections = []
        for box, score in zip(boxes, scores):
            if not np.isfinite(score) or score < PERSON_CONFIDENCE or not np.isfinite(box).all():
                continue
            coords = np.rint(np.clip(box, [0, 0, 0, 0], [w-1, h-1, w-1, h-1])).astype(int)
            if coords[2] <= coords[0] or coords[3] <= coords[1]:
                continue
            detections.append(dict(label='person', conf=float(score), box=coords.tolist()))
        self.last_inference_ms = float((result.speed or {}).get('inference') or 0.0)
        self.last_total_ms = (time.monotonic() - start) * 1000
        return detections

    def close(self):
        self.model = None
