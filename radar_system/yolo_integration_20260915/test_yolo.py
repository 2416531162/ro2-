import importlib.util
import json
from pathlib import Path
import sys
import time
import cv2
import numpy as np

p = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('yolo_test', p/'MODIFIED_FILE.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

# Known-coordinate checks independent of the network output.
image, transform = m.yolo_letterbox(np.zeros((480,640,3),np.uint8))
assert image.shape == (1,640,640,3) and transform == (1.0,0,80,640,480)
assert np.all(image[0,:80] == 114) and np.all(image[0,80:560] == 0)
portrait, pt = m.yolo_letterbox(np.zeros((640,480,3),np.uint8))
assert pt == (1.0,80,0,480,640)
boxes = np.array([[0,0,100,100],[1,1,99,99],[0,0,100,100],[150,150,200,200]],np.float32)
assert m.yolo_class_nms(boxes,np.array([.9,.8,.7,.6]),np.array([0,0,1,0])) == [0,2,3]
assert m.yolo_class_nms(np.empty((0,4)),np.array([]),np.array([])) == []
outputs = []
for size in (80,40,20):
    reg=np.full((1,64,size,size),-30,np.float32)
    reg.reshape(1,4,16,size,size)[:,:,2,:,:]=30
    scores=np.full((1,80,size,size),-30,np.float32)
    outputs.extend([reg,scores])
assert m.yolo_postprocess(outputs,transform) == []
outputs[1][0,56,30,40] = 10  # Chair at stride 8; DFL distances exactly 2 cells.
decoded = m.yolo_postprocess(outputs,transform)
assert len(decoded)==1 and decoded[0][0]=='Chair' and decoded[0][2:6] == (308,148,340,180),decoded
print('YOLO_GEOMETRY PASS letterbox_landscape portrait class_nms empty_heads dfl_decode')

fixture=np.load(p/'fixture.npz')
bgr=cv2.cvtColor(fixture['rgb'],cv2.COLOR_RGB2BGR)
detector=m.YoloRKNN(str(p/'yolov8n_rk3588_fp16.rknn'))
try:
    detector.infer_boxes(bgr)
    timings=[]
    for _ in range(8):
        t=time.monotonic();detections=detector.infer_boxes(bgr);timings.append((time.monotonic()-t)*1000)
    assert detections
    result=dict(result='PASS', model='YOLOv8n',backend='RKNN NPU',
                inference_plus_postprocess_median_ms=round(float(np.median(timings)),1),
                labels=[d[0] for d in detections],detections=detections)
    if '--reference' in sys.argv:
        reference=[(r['label'],r['confidence'],np.array(r['box']))
                   for r in json.loads((p/'onnx-reference.json').read_text())]
        matched=[]
        for label,confidence,box in reference:
            candidates=[d for d in detections if d[0]==label]
            assert candidates,('missing reference label',label)
            def overlap(d):
                other=np.array(d[2:6]);lo=np.maximum(box[:2],other[:2]);hi=np.minimum(box[2:],other[2:])
                intersection=np.maximum(hi-lo,0).prod()
                union=(box[2:]-box[:2]).prod()+(other[2:]-other[:2]).prod()-intersection
                return float(intersection/max(union,1e-9))
            best=max(candidates,key=overlap);value=overlap(best)
            assert value>=.9 and abs(best[1]-confidence)<.04,(label,value,best[1],confidence)
            matched.append(dict(label=label,iou=round(value,4),confidence_delta=round(abs(best[1]-confidence),4)))
        assert len(reference)==len(detections),(reference,detections)
        result['onnx_reference']=matched
    print('YOLO_NPU '+json.dumps(result))
    (p/'yolo-test-result.json').write_text(json.dumps(result,indent=2)+'\n')
finally:
    detector.close()
