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
