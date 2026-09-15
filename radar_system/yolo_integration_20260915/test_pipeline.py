"""Replay one captured RGB-D frame with isolated publishers, never live topics."""
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import time
import types
from unittest.mock import patch
import numpy as np

p = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('detector_test', sys.argv[1])
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
m.MODELS_DIR = str(p.parent/'models')
os.environ['RK3588_YOLO_MODEL'] = str(p/'yolov8n_rk3588_fp16.rknn')
fixture = np.load(p/'fixture.npz')
published, errors = {}, []
node = None

class Publisher:
    def __init__(self, topic): self.topic = topic
    def publish(self, msg):
        published[self.topic] = msg
        if self.topic.endswith('/targets'):
            node.running = False

logger = types.SimpleNamespace(info=lambda *_: None, warn=lambda msg: errors.append(msg))
with patch.object(m.Node, '__init__', lambda *_: None), \
     patch.object(m.Node, 'create_publisher', lambda _, cls, topic, qos: Publisher(topic)), \
     patch.object(m.Node, 'create_subscription', lambda *_: None), \
     patch.object(m.Node, 'get_logger', lambda _: logger), \
     patch.object(m.threading.Thread, 'start', lambda _: None):
    node = m.AI3DDetectorNode()
    node.latest_rgb = fixture['rgb']
    node.latest_depth = fixture['depth']
    node.latest_header = m.Image().header
    node.frame_seq = 1
    node.tick = 2  # Include the existing periodic face detector in the replay.
    k = fixture['k']
    node.fx, node.fy, node.cx, node.cy = k[0], k[4], k[2], k[5]
    original_depth = node.latest_depth.copy()
    started = time.monotonic()
    try:
        # An exception must terminate the single-frame replay rather than hang.
        def warn(msg):
            errors.append(msg)
            node.running = False
        logger.warn = warn
        node.inference_loop()
        assert not errors, errors
        targets = json.loads(published['/camera/ai_detection/targets'].data)
        assert targets, 'Expected at least one ranged object in the captured room'
        out = published['/camera/ai_detection/image']
        assert out.encoding == 'rgb8' and (out.height, out.width) == fixture['rgb'].shape[:2]
        assert len(out.data) == out.height*out.step
        for t in targets:
            xc, yc = (t['x1']+t['x2'])//2, (t['y1']+t['y2'])//2
            roi = original_depth[max(0,yc-8):min(out.height,yc+8), max(0,xc-8):min(out.width,xc+8)]
            z = float(np.median(roi[roi>150]))/1000
            x, y = (xc-k[2])*z/k[0], (yc-k[5])*z/k[4]
            assert abs(t['z']-z)<=.0051 and abs(t['x']-x)<=.0051 and abs(t['y']-y)<=.0051
            assert abs(t['distance']-math.sqrt(x*x+y*y+z*z))<=.0051
        assert np.array_equal(node.latest_depth, original_depth)
        status_msg = published.get('/camera/ai_detection/status')
        status = json.loads(status_msg.data) if status_msg else dict(model='MobileNet-SSD', backend='OpenCV CPU')
        result = dict(result='PASS', model=status['model'], backend=status['backend'],
                      labels=[t['label'] for t in targets], metric_geometry='PASS',
                      raw_depth_unchanged=True, published_topics=sorted(published),
                      elapsed_ms=round((time.monotonic()-started)*1000,1))
        print('PIPELINE_REPLAY '+json.dumps(result,sort_keys=True))
        (p/(Path(sys.argv[1]).stem+'-replay.json')).write_text(json.dumps(dict(result=result,targets=targets,status=status),indent=2)+'\n')
    finally:
        if hasattr(node, 'yolo'): node.yolo.close()
