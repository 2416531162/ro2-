"""YOLOv8n-pose RKNN decoder, independent of ROS.

Contract: Rockchip model-zoo yolov8_pose export, three (1,65,H,W) heads
(strides 8/16/32) and decoded keypoints (1,17,3,8400), input RGB uint8.
Generic Ultralytics end-to-end exports and detection-only weights are rejected.
"""
import os
import time
import numpy as np

SIZE = 640
KEYPOINT_NAMES = ('nose', 'left_eye', 'right_eye', 'left_ear', 'right_ear',
                  'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow',
                  'left_wrist', 'right_wrist', 'left_hip', 'right_hip',
                  'left_knee', 'right_knee', 'left_ankle', 'right_ankle')
SKELETON = ((15,13),(13,11),(16,14),(14,12),(11,12),(5,11),(6,12),
            (5,6),(5,7),(6,8),(7,9),(8,10),(1,2),(0,1),(0,2),(1,3),(2,4))


def letterbox(bgr):
    import cv2
    h, w = bgr.shape[:2]
    scale = min(SIZE/w, SIZE/h)
    nw, nh = round(w*scale), round(h*scale)
    left, top = (SIZE-nw)//2, (SIZE-nh)//2
    padded = np.full((SIZE, SIZE, 3), 114, dtype=np.uint8)
    padded[top:top+nh, left:left+nw] = cv2.resize(bgr, (nw, nh))
    return np.ascontiguousarray(padded[None, :, :, ::-1]), (scale, left, top, w, h)


def nms(boxes, scores, threshold=.45, limit=30):
    indices = np.argsort(-scores, kind='stable')
    kept = []
    areas = np.maximum(boxes[:, 2:]-boxes[:, :2], 0).prod(axis=1)
    while indices.size and len(kept) < limit:
        i, rest = int(indices[0]), indices[1:]
        kept.append(i)
        overlap = np.maximum(np.minimum(boxes[i, 2:], boxes[rest, 2:])
                             -np.maximum(boxes[i, :2], boxes[rest, :2]), 0).prod(axis=1)
        iou = overlap/np.maximum(areas[i]+areas[rest]-overlap, 1e-9)
        indices = rest[iou <= threshold]
    return kept


PERSON_LOW_CONFIDENCE = 0.15


def postprocess(outputs, transform, confidence=PERSON_LOW_CONFIDENCE, iou=.45):
    if outputs is None or len(outputs) != 4:
        raise ValueError('Expected YOLOv8n-pose model-zoo export: 3 box heads + 17 keypoints')
    heads = {}
    for out in outputs[:3]:
        out = np.asarray(out, dtype=np.float32)
        if out.ndim != 4 or out.shape[:2] != (1,65) or out.shape[2] != out.shape[3]:
            raise ValueError('Pose head must have shape (1,65,H,W)')
        heads[out.shape[2]] = out
    if set(heads) != {80,40,20}:
        raise ValueError('Pose input must be 640x640 with strides 8/16/32')
    keypoints = np.asarray(outputs[3], dtype=np.float32)
    if keypoints.shape not in ((1,17,3,8400),(1,51,8400)):
        raise ValueError('Expected decoded COCO keypoints (1,17,3,8400) or (1,51,8400)')
    keypoints = keypoints.reshape(17,3,8400)
    boxes_all, scores_all, kpts_all = [], [], []
    offset = 0
    for grid in (80,40,20):
        head = heads[grid][0].reshape(65,-1)
        scores = 1/(1+np.exp(-np.clip(head[64], -80, 80)))
        selected = np.flatnonzero(np.isfinite(head[64]) & (scores >= confidence))
        if selected.size:
            dfl = head[:64,selected].reshape(4,16,-1).copy()
            finite = np.isfinite(dfl).all(axis=(0,1))
            selected, dfl = selected[finite], dfl[:,:,finite]
            if selected.size:
                dfl -= dfl.max(axis=1, keepdims=True)
                weights = np.exp(dfl)
                weights /= weights.sum(axis=1, keepdims=True)
                distances = (weights*np.arange(16)[None,:,None]).sum(axis=1).T
                centers = np.column_stack((selected % grid+.5, selected//grid+.5))
                boxes_all.append(np.column_stack((centers-distances[:,:2],
                                                  centers+distances[:,2:]))*(SIZE/grid))
                scores_all.append(scores[selected])
                kpts_all.append(keypoints[:,:,selected+offset].transpose(2,0,1).copy())
        offset += grid*grid
    if not boxes_all:
        return []
    boxes, scores, kpts = map(np.concatenate, (boxes_all, scores_all, kpts_all))
    scale, left, top, w, h = transform
    boxes[:,[0,2]] = np.clip((boxes[:,[0,2]]-left)/scale,0,w-1)
    boxes[:,[1,3]] = np.clip((boxes[:,[1,3]]-top)/scale,0,h-1)
    kpts[:,:,0] = (kpts[:,:,0]-left)/scale
    kpts[:,:,1] = (kpts[:,:,1]-top)/scale
    # Out-of-frame/invalid joints must never become apparently visible at an edge.
    valid_kpt = (np.isfinite(kpts).all(axis=2) & (kpts[:,:,0]>=0) & (kpts[:,:,0]<w)
                 & (kpts[:,:,1]>=0) & (kpts[:,:,1]<h))
    kpts = np.where(valid_kpt[:,:,None], kpts, 0.)
    kpts[:,:,2] = np.clip(kpts[:,:,2],0,1)
    valid = (boxes[:,2]>boxes[:,0]) & (boxes[:,3]>boxes[:,1])
    boxes, scores, kpts = boxes[valid], scores[valid], kpts[valid]
    return [dict(label='person', conf=float(scores[i]),
                 box=np.rint(boxes[i]).astype(int).tolist(),
                 keypoints=kpts[i].round(3).tolist()) for i in nms(boxes,scores,iou)]


def draw_pose(bgr, keypoints, threshold=.5):
    import cv2
    for a,b in SKELETON:
        if keypoints[a][2] >= threshold and keypoints[b][2] >= threshold:
            cv2.line(bgr, tuple(map(int,keypoints[a][:2])), tuple(map(int,keypoints[b][:2])),
                     (0,220,130), 2, cv2.LINE_AA)
    for x,y,score in keypoints:
        if score >= threshold:
            cv2.circle(bgr,(int(x),int(y)),3,(0,220,255),-1,cv2.LINE_AA)


class PoseRKNN:
    def __init__(self, model_path):
        if not os.path.isfile(model_path):
            raise FileNotFoundError(f'Pose RKNN missing: {model_path}; see docs/TRACKING.md')
        from rknnlite.api import RKNNLite
        self.runtime = RKNNLite(verbose=False)
        self.last_inference_ms = self.last_total_ms = 0.
        try:
            if self.runtime.load_rknn(model_path) != 0:
                raise RuntimeError('YOLOv8n-pose RKNN load failed')
            if self.runtime.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2) != 0:
                raise RuntimeError('RK3588 NPU initialization failed')
            # Validate export layout before subscribing/publishing target observations.
            self.infer(np.zeros((480,640,3),dtype=np.uint8))
        except Exception:
            self.runtime.release()
            raise

    def infer(self, bgr):
        start = time.monotonic()
        image, transform = letterbox(bgr)
        infer_start = time.monotonic()
        outputs = self.runtime.inference(inputs=[image], data_format=['nhwc'])
        self.last_inference_ms = (time.monotonic()-infer_start)*1000
        detections = postprocess(outputs, transform)
        self.last_total_ms = (time.monotonic()-start)*1000
        return detections

    def close(self):
        self.runtime.release()
