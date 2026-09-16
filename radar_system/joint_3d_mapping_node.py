#!/usr/bin/env python3
"""Timestamped LiDAR/depth observations for OctoMap, with real sensor origins.

No map->base_link or sensor static transforms are published here. SLAM/AMCL
owns map->odom; Wheeltec owns odom->base_link; calibrated URDF owns sensors.
Each cloud retains its own frame and acquisition stamp so OctoMap ray clearing
starts at the correct sensor origin, not the centre of the map or chassis.
"""
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer,TransformListener,TransformException
from sensor_msgs.msg import LaserScan,Image,CameraInfo,PointCloud2
from std_msgs.msg import String
import sensor_msgs_py.point_cloud2 as pc2
from live_map_core import decode_depth,recent_stamp


class Joint3DMappingNode(Node):
    def __init__(self):
        super().__init__('joint_3d_mapping_node')
        self.declare_parameter('depth_topic','/camera/depth_registered/image_raw')
        self.declare_parameter('camera_info_topic','/camera/rgb/camera_info')
        self.declare_parameter('sample_step',8)
        self.declare_parameter('max_range_m',5.5)
        self.tf=Buffer(cache_time=Duration(seconds=15))
        self.listener=TransformListener(self.tf,self)
        self.scan=self.depth=self.info=None
        self.scan_at=self.depth_at=0.
        self.used_scan=self.used_depth=None
        self.pub=self.create_publisher(PointCloud2,'/fused_pointcloud',qos_profile_sensor_data)
        self.status_pub=self.create_publisher(String,'/joint_mapping/status',1)
        self.create_subscription(LaserScan,'/scan',self.scan_cb,qos_profile_sensor_data)
        self.create_subscription(Image,self.get_parameter('depth_topic').value,self.depth_cb,qos_profile_sensor_data)
        self.create_subscription(CameraInfo,self.get_parameter('camera_info_topic').value,self.info_cb,qos_profile_sensor_data)
        self.create_timer(.2,self.tick)

    def scan_cb(self,msg):
        self.scan,self.scan_at=msg,time.monotonic()

    def depth_cb(self,msg):
        self.depth,self.depth_at=msg,time.monotonic()

    def info_cb(self,msg):
        self.info=msg

    def publish(self,header,points):
        # TF must be available AT ACQUISITION TIME. Never relabel a frame or
        # re-publish old data with a new timestamp to make it appear current.
        stamp=header.stamp.sec+header.stamp.nanosec*1e-9
        if not recent_stamp(self.get_clock().now().nanoseconds*1e-9,stamp):
            raise ValueError('过期采样，拒绝重复入图')
        self.tf.lookup_transform('map',header.frame_id,Time.from_msg(header.stamp))
        if len(points):
            self.pub.publish(pc2.create_cloud_xyz32(header,points.astype(np.float32)))
        return len(points)

    def tick(self):
        now=time.monotonic()
        errors=[]
        counts={'lidar_points':0,'camera_points':0}
        scan,depth,info=self.scan,self.depth,self.info
        if scan is not None and scan is not self.used_scan and now-self.scan_at<.6:
            try:
                r=np.asarray(scan.ranges,dtype=np.float32)
                a=scan.angle_min+np.arange(len(r))*scan.angle_increment
                valid=np.isfinite(r)&(r>scan.range_min)&(r<min(scan.range_max,self.get_parameter('max_range_m').value))
                points=np.column_stack((r[valid]*np.cos(a[valid]),r[valid]*np.sin(a[valid]),np.zeros(valid.sum())))
                counts['lidar_points']=self.publish(scan.header,points)
                self.used_scan=scan
            except (TransformException,ValueError) as exc:
                errors.append('lidar TF: '+str(exc))
        if depth is not None and depth is not self.used_depth and now-self.depth_at<.6:
            try:
                if (info is None or info.header.frame_id!=depth.header.frame_id
                        or info.width!=depth.width or info.height!=depth.height or info.k[0]<=0 or info.k[4]<=0):
                    raise ValueError('需与深度图同坐标系、同尺寸的 CameraInfo；不猜测内参')
                step=max(2,int(self.get_parameter('sample_step').value))
                image=decode_depth(depth)
                vv,uu=np.mgrid[0:depth.height:step,0:depth.width:step]
                z=image[::step,::step]
                valid=np.isfinite(z)&(z>.2)&(z<self.get_parameter('max_range_m').value)
                zs=z[valid]
                points=np.column_stack(((uu[valid]-info.k[2])*zs/info.k[0],
                                        (vv[valid]-info.k[5])*zs/info.k[4],zs))
                counts['camera_points']=self.publish(depth.header,points)
                self.used_depth=depth
            except (TransformException,ValueError,TypeError) as exc:
                errors.append('depth: '+str(exc))
        counts.update(total_points=sum(counts.values()),errors=errors,
                      note='真实传感器帧/原始时间戳；不发布静态 map→base_link',
                      depth_fresh=now-self.depth_at<.6,lidar_fresh=now-self.scan_at<.6)
        self.status_pub.publish(String(data=json.dumps(counts,ensure_ascii=False)))


def main():
    rclpy.init()
    node=Joint3DMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=='__main__':
    main()
