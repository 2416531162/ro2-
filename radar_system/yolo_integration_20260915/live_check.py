"""Read-only ROS 2 live verification after deployment."""
import json
from pathlib import Path
import statistics
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

rclpy.init()
n=rclpy.create_node('yolo_deployment_verification')
events={name:[] for name in ['ai','rgb','lidar','status','targets']}
statuses=[]
labels=set()
ranged=[]
latest={}
def image(key,msg):
    assert msg.encoding=='rgb8' and len(msg.data)==msg.height*msg.step
    events[key].append(time.monotonic());latest[key]=msg
def status(msg):
    value=json.loads(msg.data)
    assert value['model']=='YOLOv8n' and value['backend']=='RKNN NPU' and value['classes']==80
    statuses.append(value);events['status'].append(time.monotonic())
def targets(msg):
    value=json.loads(msg.data)
    assert isinstance(value,list)
    for target in value:
        assert all(k in target for k in ('label','distance','x','y','z','x1','y1','x2','y2'))
        assert .2<target['distance']<12 and target['z']>0
        labels.add(target['label']);ranged.append(target)
    events['targets'].append(time.monotonic())
for topic,key in [('/camera/ai_detection/image','ai'),('/camera/rgb/image_raw','rgb')]:
    n.create_subscription(Image,topic,lambda msg,key=key:image(key,msg),qos_profile_sensor_data)
n.create_subscription(LaserScan,'/scan',lambda _:events['lidar'].append(time.monotonic()),qos_profile_sensor_data)
n.create_subscription(String,'/camera/ai_detection/status',status,10)
n.create_subscription(String,'/camera/ai_detection/targets',targets,10)
end=time.monotonic()+15
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.15)
assert all(len(e)>=10 for e in events.values()),{k:len(v) for k,v in events.items()}
assert ranged,'No live ranged detections during verification'
assert all(time.monotonic()-v[-1]<1 for v in events.values())
rate=lambda e:round((len(e)-1)/(e[-1]-e[0]),1)
result=dict(result='PASS',model='YOLOv8n',backend='RKNN NPU',classes=80,
            observed_hz={k:rate(v) for k,v in events.items()},
            npu_inference_median_ms=statistics.median(s['inference_ms'] for s in statuses),
            labels=sorted(labels),latest_targets=ranged[-6:],
            frame_size=[latest['ai'].width,latest['ai'].height])
p=Path(__file__).resolve().parent
(p/'live-result.json').write_text(json.dumps(result,indent=2)+'\n')
print('LIVE_YOLO '+json.dumps(result,sort_keys=True))
n.destroy_node();rclpy.shutdown()
