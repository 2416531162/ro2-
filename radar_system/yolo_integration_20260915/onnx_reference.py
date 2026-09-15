import ast, types, time, json
from pathlib import Path
import cv2, numpy as np
p=Path(__file__).resolve().parent
m=types.ModuleType('yolo_core')
m.__dict__.update(cv2=cv2,np=np,time=time)
exec(compile((p/'yolo_core.py').read_text(),str(p/'yolo_core.py'),'exec'),m.__dict__)
fixture=np.load(p/'fixture.npz')
bgr=cv2.cvtColor(fixture['rgb'],cv2.COLOR_RGB2BGR)
import onnxruntime as ort
options=ort.SessionOptions();options.intra_op_num_threads=2
session=ort.InferenceSession(str(p.parent/'models/yolov8n.onnx'),sess_options=options,providers=['CPUExecutionProvider'])
tensor,tr=m.yolo_letterbox(bgr)
output=session.run(None,{'images':tensor.transpose(0,3,1,2).astype(np.float32)/255.})[0][0].T
cls=output[:,4:].argmax(axis=1)
score=output[np.arange(len(output)),cls+4]
valid=score>=.4;output,cls,score=output[valid],cls[valid],score[valid]
# Reference uses the original model's decoded output and OpenCV NMS.
xy=output[:,:2]-output[:,2:4]/2
xy[:,0]=(xy[:,0]-tr[1])/tr[0];xy[:,1]=(xy[:,1]-tr[2])/tr[0]
wh=output[:,2:4]/tr[0]
reference=[]
for c in np.unique(cls):
    indices=np.flatnonzero(cls==c)
    kept=cv2.dnn.NMSBoxes(np.column_stack((xy[indices],wh[indices])).tolist(),score[indices].tolist(),.4,.45)
    for index in np.asarray(kept).reshape(-1):
        i=indices[index]
        a=xy[i];b=xy[i]+wh[i]
        box=np.clip(np.r_[a,b],[0,0,0,0],[639,479,639,479])
        reference.append((m.YOLO_CLASSES[int(c)].capitalize(),float(score[i]),box))
reference=[dict(label=l,confidence=c,box=b.tolist()) for l,c,b in reference]
(p/'onnx-reference.json').write_text(json.dumps(reference,indent=2)+'\n')
print('ONNX_REFERENCE',json.dumps(reference))
