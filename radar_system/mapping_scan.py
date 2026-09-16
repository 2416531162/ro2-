#!/usr/bin/env python3
"""Filter physical chassis self-hits for SLAM only. Raw /scan stays untouched."""
import copy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer,TransformListener,TransformException
from sensor_msgs.msg import LaserScan
from live_map_core import transform_points,body_self_hit_mask,recent_stamp


class MappingScan(Node):
    def __init__(self):
        super().__init__('mapping_scan_filter')
        self.tf=Buffer()
        self.listener=TransformListener(self.tf,self)
        self.pub=self.create_publisher(LaserScan,'/mapping/scan',qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan',self.receive,qos_profile_sensor_data)

    def receive(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if not recent_stamp(self.get_clock().now().nanoseconds*1e-9,stamp):
            return
        try:
            tf=self.tf.lookup_transform('base_link',msg.header.frame_id,Time.from_msg(msg.header.stamp))
            ranges=np.asarray(msg.ranges,dtype=float)
            angles=msg.angle_min+np.arange(len(ranges))*msg.angle_increment
            valid=np.isfinite(ranges)&(ranges>=msg.range_min)&(ranges<=msg.range_max)
            indices=np.flatnonzero(valid)
            points=np.column_stack((ranges[valid]*np.cos(angles[valid]),ranges[valid]*np.sin(angles[valid]),np.zeros(len(indices))))
            t,q=tf.transform.translation,tf.transform.rotation
            body=transform_points(points,(t.x,t.y,t.z),(q.x,q.y,q.z,q.w))
            ranges[indices[body_self_hit_mask(body)]]=float('nan')
            out=copy.deepcopy(msg)
            out.ranges=ranges.tolist()
            self.pub.publish(out)
        except (TransformException,ValueError) as exc:
            self.get_logger().warn(str(exc),throttle_duration_sec=3.)


def main():
    rclpy.init()
    node=MappingScan()
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
