#!/usr/bin/env python3
"""Read-only 3D observation bridge alongside the existing 2D navigation map.

Default: Astra registered depth -> acquisition-time TF -> recent map-frame
voxel samples. An existing registered PointCloud2 or OctoMap may be selected
instead. Never mix sources or use N10P's one scanning plane as 3D geometry.
"""
import math
import os
import time
import numpy as np
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformException
from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from live_map_node import LiveMapNode,stamp_s
from cloud_scene import (VoxelHistory,ScenePackets,read_xyzi,depth_xyzi,apply_matrix,
                         pose_jump,body_mask)


class LiveCloudNode(LiveMapNode):
    def __init__(self):
        super().__init__()
        defaults={
            'cloud_source':os.environ.get('RO2_CLOUD_SOURCE','depth'),
            'depth_topic':os.environ.get('DEPTH_TOPIC','/camera/depth_raw/image'),
            'depth_info_topic':os.environ.get('DEPTH_INFO_TOPIC','/camera/depth_raw/camera_info'),
            'cloud_topic':os.environ.get('RO2_CLOUD_TOPIC','/mapping/depth_points'),
            'sensor_tf_calibrated':os.environ.get('SENSOR_TF_CALIBRATED','0')=='1',
            'display_voxel_m':float(os.environ.get('RO2_CLOUD_VOXEL_M','0.05')),
            'display_point_limit':int(os.environ.get('RO2_CLOUD_POINTS','60000')),
            # RO2_CLOUD_HISTORY_S=0 打开长期累积(走过的地方不再过期)
            'display_history_s':float(os.environ.get('RO2_CLOUD_HISTORY_S','45')),
            # 0 = 不过期,长期累积成一张三维地图。走过的房间不会消失,
            # 代价是移动的人会留下短时拖影(见 scene.note)。
            'display_radius_m':float(os.environ.get('RO2_CLOUD_RADIUS_M','20')),
            # 车体实测尺寸,用于剔除自身反射。俯视的相机看得见自己的车头,
            # 不剔掉的话车一走就在地图里拖出一条跟着车动的假墙。
            'body_front_m':.67,'body_rear_m':.18,'body_half_width_m':.335,
            'body_height_m':.45,'drop_self_hits':True,
        }
        for k,v in defaults.items():
            self.declare_parameter(k,v)
        self.source=self.get_parameter('cloud_source').value
        if self.source not in ('depth','pointcloud','octomap'):
            raise ValueError('cloud_source must be depth, pointcloud or octomap')
        self.history=VoxelHistory(self.get_parameter('display_voxel_m').value,
                                  self.get_parameter('display_point_limit').value,
                                  self.get_parameter('display_history_s').value)
        self.packets=ScenePackets()
        self.active_frame='base_link'
        self.camera_infos={}
        self.latest_depth=self.latest_info=self.latest_points=None
        self.last_stamp=-1.
        self.observed_at=None
        self.cloud_error='等待三维数据'
        self.reset_reason='启动新会话'
        self.correction_anchor=None
        self.clock_anchor=None
        self.scene_dirty=False
        self.self_hits=0
        self.depth_at=self.points_at=0.
        depth_topic=self.get_parameter('depth_topic').value
        self.create_subscription(Image,depth_topic,self.depth_cb,qos_profile_sensor_data)
        for alt in ('/camera/depth_raw/image','/camera/depth_registered/image_raw'):
            if alt!=depth_topic:
                self.create_subscription(Image,alt,self.depth_cb,qos_profile_sensor_data)
        info_topic=self.get_parameter('depth_info_topic').value
        self.create_subscription(CameraInfo,info_topic,self.info_cb,qos_profile_sensor_data)
        for alt in ('/camera/depth_raw/camera_info','/camera/rgb/camera_info'):
            if alt!=info_topic:
                self.create_subscription(CameraInfo,alt,self.info_cb,qos_profile_sensor_data)
        self.create_subscription(PointCloud2,self.get_parameter('cloud_topic').value,
                                 self.points_cb,qos_profile_sensor_data)
        self.create_timer(.5,self.publish_scene)

    def depth_cb(self,msg):
        self.depth_at=time.monotonic()
        if self.source=='depth':
            self.latest_depth=msg

    def info_cb(self,msg):
        self.latest_info=msg
        if hasattr(msg,'header') and hasattr(msg.header,'frame_id') and msg.header.frame_id:
            self.camera_infos[msg.header.frame_id]=msg

    def points_cb(self,msg):
        self.points_at=time.monotonic()
        if self.source=='pointcloud':
            self.latest_points=msg

    def clear_scene(self,reason):
        with self.lock:
            self.history.clear()
            self.packets.reset()
            self.cloud=[]
            self.cloud_revision+=1
            self.observed_at=None
            self.reset_reason=reason
            self.scene_dirty=True

    def reset_display(self):
        super().reset_display()
        self.clear_scene('地图会话切换，清除旧三维数据')
        self.latest_depth=self.latest_points=self.pending_cloud=None
        self.last_stamp=-1.
        self.correction_anchor=None

    def initial_pose(self,x,y,heading):
        super().initial_pose(x,y,heading)
        self.clear_scene('重新设置初始位置，等待新观测')
        self.correction_anchor=None

    def _transform(self,points,header):
        target_frame=getattr(self,'active_frame','map')
        try:
            tf=self.tf.lookup_transform(target_frame,header.frame_id,Time.from_msg(header.stamp))
        except Exception:
            try:
                tf=self.tf.lookup_transform(target_frame,header.frame_id,Time())
            except Exception:
                if target_frame=='base_link' and 'camera' in header.frame_id:
                    y_offset=-0.045 if 'rgb' in header.frame_id else -0.02
                    return apply_matrix(points,(0.08,y_offset,0.08),(-0.5,0.5,-0.5,0.5))
                raise
        t,q=tf.transform.translation,tf.transform.rotation
        return apply_matrix(points,(t.x,t.y,t.z),(q.x,q.y,q.z,q.w))

    def tick(self):
        super().tick()
        now=time.monotonic()
        ros_now=self.get_clock().now().nanoseconds*1e-9
        if self.clock_anchor is not None and ros_now<self.clock_anchor-.1:
            self.clear_scene('ROS 时间回退，清除历史观测')
            self.last_stamp=-1.
        self.clock_anchor=ros_now
        with self.lock:
            has_map=self.map_info is not None
            slam_ready=(has_map and not self.map_fault and
                        self.robot is not None and now-self.pose_at<1.)

        if has_map:
            if not slam_ready:
                with self.lock:
                    self.robot=None
                self.cloud_error='等待真实地图与新鲜定位 TF；历史点云不可用于控制'
                return
            target_frame='map'
            try:
                correction=self.tf.lookup_transform('map','odom',Time())
                t,q=correction.transform.translation,correction.transform.rotation
                pose=(t.x,t.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
                if pose_jump(self.correction_anchor,pose):
                    self.clear_scene('定位/回环修正，重新累积三维观测，避免错位叠影')
                    self.correction_anchor=pose
                if self.correction_anchor is None:
                    self.correction_anchor=pose
            except TransformException as exc:
                self.cloud_error='缺少 map→odom: '+str(exc)
                return
        else:
            if self.map_fault:
                with self.lock:
                    self.robot=None
                self.cloud_error=self.map_fault
                return
            target_frame='base_link'

        if target_frame!=getattr(self,'active_frame','base_link'):
            self.clear_scene(f'坐标系切换: {self.active_frame} -> {target_frame}')
            self.active_frame=target_frame
            self.packets.meta['frame']=target_frame

        if self.source=='octomap':
            return  # marker callbacks are handled at 1Hz below
        msg=self.latest_depth if self.source=='depth' else self.latest_points
        if msg is None:
            self.cloud_error='等待 '+self.get_parameter('depth_topic' if self.source=='depth' else 'cloud_topic').value
            return
        stamp=stamp_s(msg.header.stamp)
        if not math.isfinite(stamp) or stamp<=0 or not -.1<=ros_now-stamp<=.8:
            self.cloud_error='三维输入已过期；保留有限历史，不冒充实时'
            return
        if stamp<=self.last_stamp:
            return
        with self.lock:
            input_epoch=self.packets.epoch
        try:
            if self.source=='depth':
                if not self.get_parameter('sensor_tf_calibrated').value:
                    raise ValueError('相机外参未确认：校准真实 TF 后设置 SENSOR_TF_CALIBRATED=1')
                info=self.camera_infos.get(msg.header.frame_id,self.latest_info)
                points=depth_xyzi(msg,info)
            else:
                points=read_xyzi(msg)
            if not msg.header.frame_id:
                raise ValueError('missing source frame')
            points=self._transform(points,msg.header)
            with self.lock:
                robot=self.robot if target_frame=='map' else [0.,0.,0.]
            if robot is not None:
                radius=float(self.get_parameter('display_radius_m').value)
                mask=np.linalg.norm(points[:,:2]-np.array(robot[:2]),axis=1)<=radius
                points=points[mask]
                if self.get_parameter('drop_self_hits').value:
                    keep=~body_mask(points,robot,
                                    self.get_parameter('body_front_m').value,
                                    self.get_parameter('body_rear_m').value,
                                    self.get_parameter('body_half_width_m').value,
                                    self.get_parameter('body_height_m').value)
                    self.self_hits+=int((~keep).sum())
                    points=points[keep]
            with self.lock:
                if input_epoch!=self.packets.epoch or (target_frame=='map' and (self.map_info is None or self.map_fault)):
                    return
                self.history.add(points,now)
                self.last_stamp=stamp
                if len(points):
                    self.observed_at=now-max(0.,ros_now-stamp)
                self.scene_dirty=True
                self.cloud_error='' if len(points) else '本帧没有有效三维深度点'
        except (TransformException,ValueError,TypeError,OverflowError) as exc:
            self.cloud_error=str(exc)

        # 在车体局部坐标系下，将2D激光雷达投影至base_link地面
        if target_frame=='base_link':
            with self.lock:
                scan=getattr(self,'pending_scan',None)
                scan_at=getattr(self,'scan_at',0.)
            if scan is not None and scan is not getattr(self,'last_processed_scan',None) and now-scan_at<.6:
                try:
                    st=self.tf.lookup_transform('base_link',scan.header.frame_id,Time())
                    pts=[]
                    for i,r in enumerate(scan.ranges):
                        if math.isfinite(r) and scan.range_min<r<scan.range_max:
                            a=scan.angle_min+i*scan.angle_increment
                            pts.append([r*math.cos(a),r*math.sin(a),0.])
                    with self.lock:
                        self.scan=apply_tf(pts[::max(1,math.ceil(len(pts)/900))],st)[:,:2].tolist() if pts else []
                    self.last_processed_scan=scan
                except Exception:
                    pass

    def encode_cloud(self):
        # Base class already subscribes to OctoMap's transient-local MarkerArray.
        # It is an alternative, complete snapshot, not a second source to blend.
        with self.lock:
            msg,self.pending_cloud=self.pending_cloud,None
            ready=self.map_info is not None and self.robot is not None and not self.map_fault
            input_epoch=self.packets.epoch
        if self.source!='octomap' or msg is None or not ready:
            return
        points=[]; stamps=[]
        # OctoMap publishes a complete set of resolution levels. Empty levels
        # and DELETEALL must clear old points, never append a second map.
        total=sum(len(m.points) for m in msg.markers if m.action==0)
        step=max(1,math.ceil(total/self.history.limit))
        for m in msg.markers:
            if m.action!=0:
                continue
            if m.header.frame_id!='map':
                self.cloud_error='OctoMap snapshot 必须在 map 坐标系'
                return
            if not m.points:
                continue
            a=np.array([[p.x,p.y,p.z,-1.] for p in m.points[::step]],np.float32)
            t,q=m.pose.position,m.pose.orientation
            try:
                points.append(apply_matrix(a,(t.x,t.y,t.z),(q.x,q.y,q.z,q.w)))
                stamps.append(stamp_s(m.header.stamp))
            except ValueError as exc:
                self.cloud_error=str(exc)
                return
        a=np.concatenate(points)[:self.history.limit] if points else np.empty((0,4),np.float32)
        ros_now=self.get_clock().now().nanoseconds*1e-9
        age=ros_now-max(stamps) if stamps else None
        with self.lock:
            if input_epoch!=self.packets.epoch:
                return
            self.packets.update(a)
            self.observed_at=time.monotonic()-max(0.,age) if age is not None and age>=-.1 else None
            self.cloud_error='' if len(a) else 'OctoMap 为空'

    def publish_scene(self):
        if self.source=='octomap':
            return
        with self.lock:
            self.history.expire(time.monotonic())
            if not (self.scene_dirty or self.history.dirty):
                return
            a=self.history.array()
            self.packets.update(a)
            self.history.dirty=self.scene_dirty=False

    def snapshot(self):
        data=super().snapshot()
        now=time.monotonic()
        with self.lock:
            target_frame=getattr(self,'active_frame','map')
            age=None if self.observed_at is None else max(0.,now-self.observed_at)
            scene=dict(self.packets.meta)
            is_live=bool(age is not None and age<1.5 and (data['localized'] if target_frame=='map' else True) and not self.cloud_error)
            scene.update(source=self.source,age_s=round(age,2) if age is not None else None,
                         live=is_live,
                         error=self.cloud_error,reset_reason=self.reset_reason,
                         voxel_m=self.history.voxel,limit=self.history.limit,history_s=self.history.ttl,
                         kind='recent_observations' if self.source!='octomap' else 'octomap_snapshot',
                         calibrated=bool(self.get_parameter('sensor_tf_calibrated').value),
                         mode='persistent' if self.history.ttl<=0 else 'recent_window',
                         self_hits=self.self_hits,radius_m=self.get_parameter('display_radius_m').value,
                         depth_offline=now-self.depth_at>1.5,
                         # 外参没确认时,深度源根本不会入图,前端必须明说,
                         # 而不是让人对着一张空地图猜是不是相机坏了。
                         extrinsics_pending=(self.source=='depth' and
                             not self.get_parameter('sensor_tf_calibrated').value),
                         note=('显示层，不是导航碰撞图；人体移动可能留下短时观测轨迹'
                               +('；长期累积模式，走过的区域不会自动过期'
                                 if self.history.ttl<=0 and target_frame=='map' else '')))
            if target_frame=='base_link' and data.get('robot') is None:
                data['robot']=[0.,0.,0.]
            data['frame']=target_frame
            scene['frame']=target_frame
            data['scene']=scene
            data['epoch']=self.packets.epoch
        return data

    def scene_bytes(self,epoch,revision,compressed=False):
        with self.lock:
            return self.packets.get(epoch,revision,compressed)
