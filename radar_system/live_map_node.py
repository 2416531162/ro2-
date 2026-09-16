#!/usr/bin/env python3
"""One read-only, TF-correct display bridge shared by Qt and HTTP.

No /cmd_vel publisher. Person observations and candidate standoff points are
visualization interfaces, NOT identity recognition or collision-checked goals.
"""
import json
import math
import threading
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener, TransformException
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from live_map_core import encode_grid, yaw, transform_points, optical_to_base, following_point, recent_stamp


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec*1e-9


def apply_tf(points, tf):
    t, q = tf.transform.translation, tf.transform.rotation
    return transform_points(points, (t.x,t.y,t.z), (q.x,q.y,q.z,q.w))


class LiveMapNode(Node):
    def __init__(self):
        super().__init__('live_map_display')
        self.declare_parameter('camera_x_m', .54)
        self.declare_parameter('camera_pitch_deg', 15.)
        self.lock = threading.RLock()
        self.tf = Buffer(cache_time=Duration(seconds=15))
        self.listener = TransformListener(self.tf, self)
        self.pending_map = None
        self.map_info = None
        self.images = deque(maxlen=3)
        self.revision = 0
        self.map_epoch = 0
        self.robot = None
        self.trace = deque(maxlen=2000)
        self.scan = []
        self.scan_at = self.pose_at = self.people_at = self.cloud_at = self.plan_at = 0.
        self.people = []
        self.goal = None
        self.plan = []
        self.cloud = []
        self.cloud_revision = 0
        self.error = '等待 /map 与 map→odom→base_link TF'
        self.mapping_status = {}
        self.follower = {}
        self.follower_at = 0.
        self.last_scan_stamp = 0.
        self.pending_people = []
        self.people_rx = 0.
        self.pending_scan = None
        self.last_processed_scan = None
        self.pending_cloud = None
        self.last_map_rx = 0.
        self.map_source = ''
        self.map_fault = ''
        self.static_maps = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                      durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,'/map',self.map_cb,self.static_maps)
        self.create_subscription(LaserScan,'/scan',self.scan_cb,qos_profile_sensor_data)
        self.create_subscription(String,'/camera/ai_detection/targets',self.people_cb,1)
        self.create_subscription(String,'/follower/status',self.follower_cb,1)
        self.create_subscription(String,'/joint_mapping/status',self.mapping_cb,1)
        self.create_subscription(Path,'/plan',self.plan_cb,1)
        self.create_subscription(MarkerArray,'/occupied_cells_vis_array',self.cloud_cb,self.static_maps)
        self.pub_pose = self.create_publisher(PoseStamped,'/robot_pose',1)
        self.pub_goal = self.create_publisher(PoseStamped,'/follow/candidate_goal',1)
        self.pub_person = self.create_publisher(PoseStamped,'/person_tracker/pose',1)
        self.pub_initial = self.create_publisher(PoseWithCovarianceStamped,'/initialpose',1)
        self.create_timer(.2,self.tick)
        self.create_timer(1.,self.encode_pending)
        self.create_timer(1.,self.encode_cloud)

    def map_cb(self,msg):
        owners=self.get_publishers_info_by_topic('/map')
        if any('fake' in owner.node_name.lower() for owner in owners) or len(owners)>1:
            self.map_fault='拒绝模拟地图或多个 /map 发布者；请只保留一个真实 SLAM/AMCL 地图源'
            self.error=self.map_fault
            self.robot=None
            return
        if msg.header.frame_id != 'map':
            self.error = '拒绝非 map 坐标地图: '+msg.header.frame_id
            return
        self.map_fault=''
        with self.lock:
            self.pending_map = msg
            self.last_map_rx = time.monotonic()

    def scan_cb(self,msg):
        if not recent_stamp(self.get_clock().now().nanoseconds*1e-9,stamp_s(msg.header.stamp)):
            return
        with self.lock:
            self.pending_scan = msg
            self.scan_at = time.monotonic()

    def people_cb(self,msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data,list):
                return
            with self.lock:
                self.pending_people = data[:32]
                self.people_rx = time.monotonic()
        except (ValueError,TypeError):
            return

    def follower_cb(self,msg):
        try:
            data=json.loads(msg.data)
            if isinstance(data,dict):
                with self.lock:
                    self.follower, self.follower_at = data,time.monotonic()
        except (ValueError,TypeError):
            pass

    def mapping_cb(self,msg):
        try:
            data=json.loads(msg.data)
            if isinstance(data,dict):
                self.mapping_status=data
        except (ValueError,TypeError):
            pass

    def plan_cb(self,msg):
        # Do not silently relabel odom paths as map.
        if msg.header.frame_id == 'map':
            with self.lock:
                self.plan=[[p.pose.position.x,p.pose.position.y] for p in msg.poses[::max(1,len(msg.poses)//1000)]]
                self.plan_at=time.monotonic()

    def cloud_cb(self,msg):
        with self.lock:
            self.pending_cloud=msg

    def encode_cloud(self):
        with self.lock:
            msg,self.pending_cloud=self.pending_cloud,None
        if msg is None:
            return
        # Only complete OctoMap marker arrays in map coordinates are accepted.
        result=[]
        for marker in msg.markers:
            if marker.action != 0 or marker.header.frame_id != 'map':
                continue
            count=len(marker.points)
            if not count:
                continue
            step=max(1,math.ceil(count/6000))
            points=[[p.x,p.y,p.z] for p in marker.points[::step]]
            q=marker.pose.orientation
            p=marker.pose.position
            try:
                a=transform_points(points,(p.x,p.y,p.z),(q.x,q.y,q.z,q.w))
                result.extend(a.tolist())
            except ValueError:
                continue
        if len(result)>6000:
            result=result[::math.ceil(len(result)/6000)]
        with self.lock:
            self.cloud=result
            self.cloud_revision+=1
            self.cloud_at=time.monotonic()

    def encode_pending(self):
        with self.lock:
            msg,self.pending_map=self.pending_map,None
            epoch=self.map_epoch
        if msg is None:
            return
        try:
            png,info=encode_grid(msg.data,msg.info.width,msg.info.height,msg.info.resolution)
            p,q=msg.info.origin.position,msg.info.origin.orientation
            info.update(origin=[p.x,p.y,yaw(q)],frame='map',source='/map')
            with self.lock:
                if epoch!=self.map_epoch:
                    return
                # An identical static map is not re-encoded into a new revision.
                if self.images and self.images[-1][1]==png and self.map_info and all(self.map_info.get(k)==v for k,v in info.items()):
                    return
                self.revision+=1
                info['revision']=self.revision
                self.images.append((self.revision,png))
                self.map_info=info
        except (ValueError,MemoryError) as exc:
            self.error=str(exc)

    def pose_message(self,xy_heading,stamp=None):
        msg=PoseStamped()
        msg.header.frame_id='map'
        msg.header.stamp=stamp or self.get_clock().now().to_msg()
        msg.pose.position.x,msg.pose.position.y=float(xy_heading[0]),float(xy_heading[1])
        msg.pose.orientation.z=math.sin(xy_heading[2]/2)
        msg.pose.orientation.w=math.cos(xy_heading[2]/2)
        return msg

    def tick(self):
        now=time.monotonic()
        if self.map_info is None or self.map_fault:
            self.error=self.map_fault or '等待真实 /map；不使用静态假位姿'
            return
        ros_now=self.get_clock().now().nanoseconds*1e-9
        try:
            tf=self.tf.lookup_transform('map','base_link',Time())
            age=ros_now-stamp_s(tf.header.stamp)
            # A zero-stamped static map→base_link is never a live localization.
            if stamp_s(tf.header.stamp)<=0 or not -.1<=age<=1.:
                raise ValueError('定位 TF 已过期或是静态假位姿')
            p,q=tf.transform.translation,tf.transform.rotation
            robot=[p.x,p.y,yaw(q)]
            self.pub_pose.publish(self.pose_message(robot,tf.header.stamp))
            with self.lock:
                if self.robot and math.hypot(robot[0]-self.robot[0],robot[1]-self.robot[1])>1.:
                    self.trace.clear()  # relocalization/loop-closure jump, do not draw through a wall
                self.robot,self.pose_at=robot,now
                if not self.trace or math.hypot(robot[0]-self.trace[-1][0],robot[1]-self.trace[-1][1])>.03:
                    self.trace.append(robot[:2])
                self.error=''
        except (TransformException,ValueError) as exc:
            with self.lock:
                self.error=str(exc)
                self.robot=None
                self.goal=None
                self.people=[]
                self.scan=[]
            return
        with self.lock:
            scan=self.pending_scan
            targets=list(self.pending_people) if now-self.people_rx<.6 else []
            follower=dict(self.follower) if now-self.follower_at<.6 else {}
        if scan is not None and scan is not self.last_processed_scan and now-self.scan_at<.6:
            try:
                st=self.tf.lookup_transform('map',scan.header.frame_id,Time.from_msg(scan.header.stamp))
                points=[]
                for i,r in enumerate(scan.ranges):
                    if math.isfinite(r) and scan.range_min<r<scan.range_max:
                        a=scan.angle_min+i*scan.angle_increment
                        points.append([r*math.cos(a),r*math.sin(a),0.])
                with self.lock:
                    self.scan=apply_tf(points[::max(1,math.ceil(len(points)/900))],st)[:,:2].tolist() if points else []
                self.last_processed_scan=scan
            except (TransformException,ValueError):
                with self.lock:
                    self.scan=[]
        people=[]
        for target in targets:
            try:
                stamp=float(target.get('stamp') or 0.)
                if (str(target.get('label','')).lower()!='person' or not target.get('range_valid')
                        or float(target.get('conf',0))<.35 or not -.1<=ros_now-stamp<=.6):
                    continue
                x,y=optical_to_base(float(target['x']),float(target['y']),float(target['z']),
                    self.get_parameter('camera_x_m').value,
                    math.radians(self.get_parameter('camera_pitch_deg').value))
                t=self.tf.lookup_transform('map','base_link',Time(seconds=stamp))
                point=apply_tf([[x,y,0.]],t)[0]
                people.append(dict(x=float(point[0]),y=float(point[1]),stamp=stamp,conf=target['conf'],
                                   box=[target.get(k) for k in ('x1','y1','x2','y2')]))
            except (TransformException,ValueError,TypeError,KeyError):
                continue
        goal=None
        # Only highlight the existing follower's confirmed visual target. YOLO
        # class detection is NOT person identity, and ambiguous matches are rejected.
        selected=follower.get('target') or {}
        box=[selected.get(k) for k in ('x1','y1','x2','y2')]
        matches=[p for p in people if p['box']==box and None not in box]
        if follower.get('target_locked') and len(matches)==1 and not selected.get('coasting'):
            person=matches[0]
            person['locked']=True
            self.pub_person.publish(self.pose_message([person['x'],person['y'],0.],
                                    Time(seconds=person['stamp']).to_msg()))
            candidate=following_point(robot,[person['x'],person['y']])
            goal=dict(x=candidate[0],y=candidate[1],yaw=candidate[2],validated=False)
            self.pub_goal.publish(self.pose_message(candidate))
        with self.lock:
            self.people,self.goal,self.people_at=people,goal,now

    def initial_pose(self,x,y,heading):
        if not all(math.isfinite(float(v)) and abs(float(v))<1e6 for v in (x,y,heading)):
            raise ValueError('invalid initial pose')
        pose=self.pose_message([x,y,heading])
        msg=PoseWithCovarianceStamped()
        msg.header,msg.pose.pose=pose.header,pose.pose
        msg.pose.covariance[0]=msg.pose.covariance[7]=.25
        msg.pose.covariance[35]=math.radians(15)**2
        self.pub_initial.publish(msg)

    def snapshot(self):
        now=time.monotonic()
        with self.lock:
            ready=self.robot is not None and now-self.pose_at<1.
            return dict(map=self.map_info,robot=self.robot if ready else None,
                        localized=ready,error=self.error,trajectory=list(self.trace),
                        scan=self.scan if ready and now-self.scan_at<.6 else [],
                        scan_live=now-self.scan_at<.6,people=self.people if ready else [],
                        goal=self.goal if ready else None,plan=self.plan if ready and now-self.plan_at<3. else [],
                        cloud_revision=self.cloud_revision,cloud_age_s=round(now-self.cloud_at,1) if self.cloud_at else None,
                        map_age_s=round(now-self.last_map_rx,1) if self.last_map_rx else None,
                        mapping=self.mapping_status,frame='map',control='display_only')

    def reset_display(self):
        with self.lock:
            self.map_info=self.pending_map=None
            self.map_epoch+=1
            self.map_fault=''
            self.last_processed_scan=None
            self.images.clear()
            self.trace.clear()
            self.robot=None
            self.people=[]
            self.goal=None
            self.plan=[]
            self.scan=[]
            self.cloud=[]
            self.pending_cloud=None
            self.cloud_revision+=1
            self.error='切换地图，等待新地图与定位'

    def image(self,revision):
        with self.lock:
            return next((png for rev,png in self.images if rev==revision),None)

    def cloud_snapshot(self):
        with self.lock:
            return dict(revision=self.cloud_revision,frame='map',points=list(self.cloud))
