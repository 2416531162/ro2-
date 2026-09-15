import time,json,sys
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
rclpy.init(); n=Node('camera_optimization_probe'); rows={}; frames={}; infos={}; targets=[]
def stamp(m): return m.header.stamp.sec+m.header.stamp.nanosec/1e9
def img(m,key):
    now=time.monotonic(); rows.setdefault(key,[]).append((now,stamp(m)))
    frames[key]=m
    if len(rows[key])==1: print(key,json.dumps(dict(w=m.width,h=m.height,encoding=m.encoding,step=m.step,bigendian=m.is_bigendian,frame_id=m.header.frame_id,stamp=stamp(m))),flush=True)
def info(m,key):
    if key not in infos:
        infos[key]=dict(w=m.width,h=m.height,frame_id=m.header.frame_id,k=list(m.k),p=list(m.p),d=list(m.d),model=m.distortion_model)
        print(key,json.dumps(infos[key]),flush=True)
for key,topic in [('rgb','/camera/rgb/image_raw'),('depth','/camera/depth_raw/image')]:n.create_subscription(Image,topic,lambda m,k=key:img(m,k),qos_profile_sensor_data)
for key,topic in [('rgb_info','/camera/rgb/camera_info'),('depth_info','/camera/depth_raw/camera_info')]:n.create_subscription(CameraInfo,topic,lambda m,k=key:info(m,k),qos_profile_sensor_data)
def target(m):
    targets.append((time.monotonic(),json.loads(m.data)))
n.create_subscription(String,'/camera/ai_detection/targets',target,10)
t=time.monotonic()
while time.monotonic()-t<9:rclpy.spin_once(n,timeout_sec=.1)
for key,values in rows.items():print('rate',key,round((len(values)-1)/(values[-1][0]-values[0][0]),2),'frames',len(values))
if 'rgb' in rows and 'depth' in rows:
    deltas=[min(abs(t1[1]-t2[1]) for t2 in rows['depth']) for t1 in rows['rgb']]
    print('nearest_sync_ms_median_max',float(np.median(deltas)*1000),float(max(deltas)*1000))
if targets:print('ai_rate',round((len(targets)-1)/(targets[-1][0]-targets[0][0]),2) if len(targets)>1 else 0,'last_targets',json.dumps(targets[-1][1]))
out=Path(sys.argv[1]);out.mkdir(exist_ok=True)
meta=dict(infos=infos,images={})
for key,m in frames.items():
    (out/(key+'.bin')).write_bytes(bytes(m.data));meta['images'][key]=dict(w=m.width,h=m.height,encoding=m.encoding,step=m.step,bigendian=m.is_bigendian,frame_id=m.header.frame_id,stamp=stamp(m))
(out/'meta.json').write_text(json.dumps(meta,indent=2));(out/'targets.json').write_text(json.dumps(targets[-1][1] if targets else []))
n.destroy_node();rclpy.shutdown();sys.exit(0 if len(frames)==2 else 1)
