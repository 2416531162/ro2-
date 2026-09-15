#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RK3588 实时 AI + 3D 物理空间测距定位系统
- 结合奥比中光 Astra S 3D 深度相机 (RGB-D)
- 运行 MobileNet-SSD 与 ResNet-10 深度人脸神经网络
- 实时输出检测框并反投影计算空间 3D 物理坐标 [X, Y, Z] (单位: 米) 与真实直线距离
- 采用帧缓冲与多线程异步推理引擎，保证 0 阻塞、超流畅运行
- 发布 /camera/ai_detection/image 可视化话题
"""

import sys
import os
import time
import math
import threading
import json
from collections import deque
from camera_pipeline import image_array, stamp_seconds, alignment_reason, range_target, filter_detections
import cv2
import numpy as np

cv2.setNumThreads(2)
cv2.ocl.setUseOpenCL(False)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

DIR = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR = os.path.join(DIR, "models")

CLASSES = [
    "background", "aeroplane", "bicycle", "bird", "boat",
    "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
    "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
    "sofa", "train", "tvmonitor"
]

CLASS_NAMES_CN = {
    "person": "人", "chair": "椅子", "bottle": "瓶子", "sofa": "沙发",
    "tvmonitor": "显示屏", "diningtable": "桌子", "pottedplant": "盆栽",
    "bicycle": "自行车", "car": "汽车", "cat": "猫", "dog": "狗"
}

class AI3DDetectorNode(Node):
    def __init__(self):
        super().__init__('ai_3d_detector_node')
        self.get_logger().info('Loading existing MobileNet-SSD / ResNet face models')
        self.face_net=cv2.dnn.readNetFromCaffe(os.path.join(MODELS_DIR,'face_deploy.prototxt'),os.path.join(MODELS_DIR,'res10_300x300_ssd_iter_140000_fp16.caffemodel'))
        self.ssd_net=cv2.dnn.readNetFromCaffe(os.path.join(MODELS_DIR,'MobileNetSSD_deploy.prototxt'),os.path.join(MODELS_DIR,'MobileNetSSD_deploy.caffemodel'))
        self.lock=threading.Lock(); self.event=threading.Event()
        self.rgbs=deque(maxlen=3); self.depths=deque(maxlen=8)
        self.infos={}; self.running=True; self.frame_seq=0; self.tick=0
        self.last_rgb_received=0.; self.stale_sent=False
        self.last_result={}; self.last_publish=0.; self.rate=0.
        self.errors=0
        self.pub_annotated=self.create_publisher(Image,'/camera/ai_detection/image',1)
        self.pub_json=self.create_publisher(String,'/camera/ai_detection/targets',1)
        self.pub_observations=self.create_publisher(String,'/camera/ai_detection/observations',1)
        self.pub_status=self.create_publisher(String,'/camera/ai_detection/status',1)
        q=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CameraInfo,'/camera/rgb/camera_info',lambda m:self.cam_info_cb(m,'rgb'),q)
        self.create_subscription(CameraInfo,'/camera/depth_raw/camera_info',lambda m:self.cam_info_cb(m,'depth'),q)
        self.create_subscription(Image,'/camera/rgb/image_raw',self.rgb_cb,q)
        self.create_subscription(Image,'/camera/depth_raw/image',self.depth_cb,q)
        self.create_timer(.5,self.watchdog)
        self.worker=threading.Thread(target=self.inference_loop,daemon=True); self.worker.start()

    def cam_info_cb(self,msg,key='rgb'):
        info=dict(w=msg.width,h=msg.height,k=list(msg.k),d=list(msg.d),frame=msg.header.frame_id)
        with self.lock:self.infos[key]=info

    def depth_cb(self,msg):
        try:
            record=dict(array=image_array(msg),stamp=stamp_seconds(msg),frame=msg.header.frame_id)
            with self.lock:self.depths.append(record)
            self.event.set()
        except (ValueError,TypeError) as exc:
            self.errors+=1; self.get_logger().warn(str(exc),throttle_duration_sec=5.)

    def rgb_cb(self,msg):
        try:
            record=dict(array=image_array(msg),stamp=stamp_seconds(msg),frame=msg.header.frame_id,header=msg.header)
            with self.lock:
                self.frame_seq+=1; record['seq']=self.frame_seq
                self.rgbs.append(record); self.last_rgb_received=time.monotonic()
            self.event.set()
        except (ValueError,TypeError) as exc:
            self.errors+=1; self.get_logger().warn(str(exc),throttle_duration_sec=5.)

    def send_json(self,publisher,value):
        m=String(); m.data=json.dumps(value,allow_nan=False); publisher.publish(m)

    def watchdog(self):
        stale=time.monotonic()-self.last_rgb_received>.6
        if stale and not self.stale_sent:
            self.send_json(self.pub_json,[])
            self.send_json(self.pub_observations,dict(targets=[],stale=True))
        self.stale_sent=stale
        self.send_json(self.pub_status,dict(self.last_result,stale=stale,errors=self.errors))

    def infer_boxes(self,bgr):
        h,w=bgr.shape[:2]; self.tick+=1; detections=[]
        networks=[(self.ssd_net,.007843,127.5,False)]
        if self.tick%3==0:networks.append((self.face_net,1.,(104.,177.,123.),True))
        for net,scale,mean,is_face in networks:
            net.setInput(cv2.dnn.blobFromImage(bgr,scale,(300,300),mean))
            out=net.forward()
            for i in range(out.shape[2]):
                confidence=float(out[0,0,i,2]); idx=int(out[0,0,i,1])
                if confidence < (.65 if is_face else .5):continue
                if not is_face and not 0<idx<len(CLASSES):continue
                name='Face' if is_face else CLASSES[idx].capitalize()
                box=out[0,0,i,3:7]*np.array([w,h,w,h])
                x1,y1,x2,y2=box.astype(int)
                detections.append((name,confidence,int(np.clip(x1,0,w-1)),int(np.clip(y1,0,h-1)),int(np.clip(x2,0,w-1)),int(np.clip(y2,0,h-1))))
        return filter_detections(detections)

    def inference_loop(self):
        last_seq=-1
        while self.running:
            self.event.wait(.1); self.event.clear()
            with self.lock:
                rgb=self.rgbs[-1] if self.rgbs else None
                depth=min(self.depths,key=lambda d:abs(d['stamp']-rgb['stamp'])) if rgb and self.depths else None
                infos=dict(self.infos)
            if rgb is None or rgb['seq']==last_seq:continue
            last_seq=rgb['seq']; started=time.monotonic()
            try:
                age=self.get_clock().now().nanoseconds/1e9-rgb['stamp']
                if age>.6 or age<-.1:
                    self.send_json(self.pub_json,[]); continue
                reason=alignment_reason(rgb,depth,infos.get('rgb'),infos.get('depth'))
                bgr=cv2.cvtColor(rgb['array'],cv2.COLOR_RGB2BGR); h,w=bgr.shape[:2]
                observations=[]; valid_targets=[]
                for label,conf,x1,y1,x2,y2 in self.infer_boxes(bgr):
                    sample=range_target(depth['array'],(x1,y1,x2,y2),infos['rgb']['k']) if reason is None else dict(valid=False,reason=reason)
                    target=dict(label=label,conf=round(conf,3),x1=x1,y1=y1,x2=x2,y2=y2,
                                distance=None,x=None,y=None,z=None,source_stamp=rgb['stamp'])
                    target.update(sample)
                    observations.append(target)
                    if sample['valid']:valid_targets.append(target)
                    color=(210,220,80) if sample['valid'] else (70,180,240)
                    cv2.rectangle(bgr,(x1,y1),(x2,y2),color,2)
                    xc,yc=(x1+x2)//2,(y1+y2)//2
                    cv2.drawMarker(bgr,(xc,yc),color,cv2.MARKER_CROSS,10,1)
                    text=(f"{label} {conf:.0%} | Z {sample['z']:.2f}m R {sample['distance']:.2f}m" if sample['valid'] else f"{label} {conf:.0%} | depth: {sample['reason']}")
                    tw=cv2.getTextSize(text,cv2.FONT_HERSHEY_SIMPLEX,.43,1)[0][0]
                    tx=min(x1,max(0,w-tw-8)); ty=max(18,y1-5)
                    cv2.rectangle(bgr,(tx,max(0,ty-16)),(min(w-1,tx+tw+6),ty+4),(15,23,35),-1)
                    cv2.putText(bgr,text,(tx+3,ty),cv2.FONT_HERSHEY_SIMPLEX,.43,(240,245,250),1,cv2.LINE_AA)
                used=time.monotonic()-started
                now=time.monotonic()
                if self.last_publish:
                    rate=1/max(.001,now-self.last_publish); self.rate=rate if not self.rate else .8*self.rate+.2*rate
                self.last_publish=now
                delta=abs(rgb['stamp']-depth['stamp'])*1000 if depth else None
                self.last_result=dict(inference_ms=round(used*1000,1),inference_hz=round(self.rate,1),
                                      sync_ms=round(delta,1) if delta is not None else None,
                                      alignment=reason or 'registered',detections=len(observations),
                                      measured=len(valid_targets),source_stamp=rgb['stamp'])
                # Entire annotation uses exactly this source RGB and paired depth.
                image=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
                out=Image(); out.header=rgb['header']; out.height=h;out.width=w
                out.encoding='rgb8';out.is_bigendian=0;out.step=w*3;out.data=image.tobytes()
                self.pub_annotated.publish(out)
                self.send_json(self.pub_json,valid_targets)  # legacy API remains numeric-only
                self.send_json(self.pub_observations,dict(targets=observations,stale=False,**self.last_result))
            except Exception as exc:
                self.errors+=1;self.get_logger().warn('AI: '+str(exc),throttle_duration_sec=2.)
                self.send_json(self.pub_json,[])
            # Bound CPU; no FIFO of stale frames accumulates during inference.
            remaining=.1-(time.monotonic()-started)
            if remaining>0:time.sleep(remaining)


def main(args=None):
    rclpy.init(args=args);node=AI3DDetectorNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.running=False;node.event.set();node.worker.join(timeout=2.)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
