#!/usr/bin/env python3
"""N10P real-time reader: one complete revolution = one /scan message."""
import json
import math
import os
import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from n10p_pipeline import N10PDecoder, SweepAssembler, BINS, RANGE_MIN, RANGE_MAX

PORT = os.environ.get('N10P_PORT', '/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0001-if00')
BAUD = 460800


class RealLidarNode(Node):
    def __init__(self):
        super().__init__('real_lidar_node')
        self.pub = self.create_publisher(LaserScan, '/scan', QoSProfile(depth=1))
        self.status_pub = self.create_publisher(String, '/lidar/status', 1)
        self.decoder, self.assembler = N10PDecoder(), SweepAssembler()
        self.ser = None
        self.last_reconnect = -math.inf
        self.last_scan = None
        self.scan_time = 0.0
        self.valid = 0
        self.overruns = 0
        self.open_serial()
        self.timer = self.create_timer(0.01, self.spin_serial)
        self.status_timer = self.create_timer(1.0, self.publish_status)

    def open_serial(self):
        if self.ser:
            self.ser.close()
        self.ser = None
        self.decoder.buffer.clear()
        self.assembler = SweepAssembler()
        self.last_scan = None
        self.last_reconnect = time.monotonic()
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0, exclusive=True)
            self.ser.dtr = True
            self.ser.rts = True
            self.ser.reset_input_buffer()
            self.get_logger().info('N10P opened %s @ %d' % (PORT, BAUD))
        except (serial.SerialException, OSError) as exc:
            if self.ser:
                self.ser.close()
            self.ser = None
            self.get_logger().warn(str(exc), throttle_duration_sec=2.0)

    def spin_serial(self):
        if self.ser is None:
            if time.monotonic() - self.last_reconnect >= 1.0:
                self.open_serial()
            return
        try:
            waiting = self.ser.in_waiting
            if waiting > 16384:
                # Discard a stale backlog rather than replaying it as live scans.
                self.ser.reset_input_buffer()
                self.decoder.buffer.clear()
                self.assembler = SweepAssembler()
                self.overruns += 1
                self.last_scan = None
                return
            if not waiting:
                return
            data = self.ser.read(min(waiting, 8192))
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warn('N10P I/O: %s' % exc)
            self.open_serial()
            return
        now = time.monotonic()
        for points in self.decoder.feed(data):
            for scan in self.assembler.add(points, now):
                self.publish_scan(scan)

    def publish_scan(self, scan):
        msg = LaserScan()
        # Host receive estimate of sweep start, not a hardware clock timestamp.
        start_ns = self.get_clock().now().nanoseconds - int(scan['scan_time'] * 1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(start_ns, 10**9)
        msg.header.frame_id = 'laser'
        msg.angle_min = 0.0
        msg.angle_increment = 2 * math.pi / BINS
        msg.angle_max = (BINS - 1) * msg.angle_increment
        msg.scan_time = scan['scan_time']
        # Bins are reversed/resampled into ROS angular order, not time order.
        msg.time_increment = 0.0
        msg.range_min, msg.range_max = RANGE_MIN, RANGE_MAX
        msg.ranges, msg.intensities = scan['ranges'], scan['intensities']
        self.pub.publish(msg)
        self.last_scan = scan['received']
        self.scan_time = scan['scan_time']
        self.valid = sum(math.isfinite(r) for r in scan['ranges'])

    def publish_status(self):
        age = time.monotonic() - self.last_scan if self.last_scan is not None else None
        d = self.decoder
        status = dict(model='N10P', connected=self.ser is not None,
                      stale=age is None or age > 0.5, age_ms=round(age*1000) if age is not None else None,
                      hz=round(1/self.scan_time, 2) if self.scan_time and age is not None and age < 0.5 else 0,
                      valid=self.valid if age is not None and age < 0.5 else 0, bins=BINS,
                      bytes=d.bytes_received, frames=d.frames, crc_errors=d.crc_errors,
                      angle_errors=d.angle_errors, discarded_bytes=d.discarded_bytes,
                      echo_fallbacks=d.echo_fallbacks, overruns=self.overruns)
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)
        self.get_logger().info('N10P '+msg.data, throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = RealLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
