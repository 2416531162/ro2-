import sys,json,pathlib,types,time
sys.path.insert(0,sys.argv[1])
import cv2,numpy as np
from camera_pipeline import image_array,range_target
from ai_3d_detector import AI3DDetectorNode,MODELS_DIR
base=pathlib.Path('/tmp/camera-baseline');meta=json.loads((base/'meta.json').read_text());arrays={}
for k,m in meta['images'].items():
    arrays[k]=image_array(types.SimpleNamespace(width=m['w'],height=m['h'],step=m['step'],encoding=m['encoding'],is_bigendian=m['bigendian'],data=(base/(k+'.bin')).read_bytes()))
node=object.__new__(AI3DDetectorNode);node.tick=0
node.ssd_net=cv2.dnn.readNetFromCaffe(MODELS_DIR+'/MobileNetSSD_deploy.prototxt',MODELS_DIR+'/MobileNetSSD_deploy.caffemodel')
node.face_net=cv2.dnn.readNetFromCaffe(MODELS_DIR+'/face_deploy.prototxt',MODELS_DIR+'/res10_300x300_ssd_iter_140000_fp16.caffemodel')
bgr=cv2.cvtColor(arrays['rgb'],cv2.COLOR_RGB2BGR);times=[];boxes=[]
for i in range(6):
    t=time.monotonic();boxes=node.infer_boxes(bgr);times.append((time.monotonic()-t)*1000)
results=[]
for label,conf,*box in boxes:
    sample=range_target(arrays['depth'],box,meta['infos']['rgb_info']['k']);results.append(dict(label=label,conf=round(conf,3),box=box,**sample))
print(json.dumps(dict(replay='captured_RGBD',iterations=6,inference_ms_median=round(float(np.median(times)),1),observations=results),ensure_ascii=False))
