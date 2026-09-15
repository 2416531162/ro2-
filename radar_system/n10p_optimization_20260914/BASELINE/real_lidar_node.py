#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LSLIDAR N10P: 108-byte dual-echo frames on /dev/ttyACM0 @ 460800."""

import math
import time

import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

PORT = "/dev/ttyACM0"
BAUD = 460800
HEADER = b"\xa5\x5a"
FRAME = 108
POINTS = 16
RANGE_MIN = 0.15
RANGE_MAX = 12.0
DATA_START = 7
DEGREE_START = 5
END_DEGREE = 105


def n10_crc8(pkt):
    return sum(pkt) & 0xFF


def be16(pkt, offset):
    return (pkt[offset] << 8) | pkt[offset + 1]


class RealLidarNode(Node):
    def __init__(self):
        super().__init__("real_lidar_node")
        self.pub = self.create_publisher(LaserScan, "/scan", 10)
        self.ser = None
        self.buf = bytearray()
        self.bins = {}
        self.display_bins = {}
        self.rev_start = time.time()
        self.last_ang = None
        self.last_reconnect = 0.0
        self.open_serial()
        self.timer = self.create_timer(0.01, self.spin_serial)
        self.pub_timer = self.create_timer(0.1, self.publish_scan)
        self.get_logger().info(">>> [N10P] decoder ready")

    def open_serial(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        try:
            ser = serial.Serial(PORT, BAUD, timeout=0.2)
            ser.dtr = True
            ser.rts = True
            time.sleep(0.05)
            ser.reset_input_buffer()
            self.ser = ser
            self.buf = bytearray()
            self.get_logger().info("opened %s @ %d" % (PORT, BAUD))
            return True
        except Exception as exc:
            self.get_logger().warn("open %s failed: %s" % (PORT, exc), throttle_duration_sec=2.0)
            return False

    def spin_serial(self):
        if self.ser is None:
            now = time.time()
            if now - self.last_reconnect < 1.0:
                return
            self.last_reconnect = now
            self.open_serial()
            return
        try:
            waiting = self.ser.in_waiting
            if waiting:
                self.buf.extend(self.ser.read(waiting))
        except Exception as exc:
            self.get_logger().warn("serial I/O: %s, reconnecting" % exc)
            self.open_serial()
            return
        while True:
            idx = self.buf.find(HEADER)
            if idx < 0:
                if len(self.buf) > FRAME:
                    del self.buf[: len(self.buf) - 1]
                return
            if idx:
                del self.buf[:idx]
            if len(self.buf) < FRAME:
                return
            pkt = bytes(self.buf[:FRAME])
            del self.buf[:FRAME]
            self.handle_frame(pkt)

    def handle_frame(self, pkt):
        if pkt[0] != 0xA5 or pkt[1] != 0x5A:
            return
        if n10_crc8(pkt[: FRAME - 1]) != pkt[FRAME - 1]:
            return
        start = (be16(pkt, DEGREE_START) / 100.0) % 360.0
        end = (be16(pkt, END_DEGREE) / 100.0) % 360.0
        if start > end:
            span = end + 360.0 - start
        else:
            span = end - start
        if span <= 0.2 or span > 40.0:
            span = 15.0

        wrapped = (
            self.last_ang is not None
            and self.last_ang > 300.0
            and start < 60.0
        )
        timed_out = (time.time() - self.rev_start) > 0.18 and len(self.bins) >= 80
        if wrapped or timed_out:
            if len(self.bins) >= 80:
                self.display_bins = self.bins
                self.bins = {}
                self.rev_start = time.time()
                self.publish_scan()
        self.last_ang = start

        denom = max(1, POINTS - 1)
        kept = 0
        for i in range(POINTS):
            off = DATA_START + i * 6
            dist_code = be16(pkt, off)
            if dist_code == 0xFFFF:
                continue
            rng = dist_code / 1000.0
            if rng < RANGE_MIN or rng > RANGE_MAX:
                continue
            inten = pkt[off + 2]
            ang = (start + span * i / float(denom)) % 360.0
            disp = (360.0 - ang) % 360.0
            key = int(round(disp * 2.0)) % 720
            self.bins[key] = (rng, float(inten))
            kept += 1
        if kept == 0:
            return

    def publish_scan(self):
        n = 720
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser"
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi - (2.0 * math.pi / n)
        msg.angle_increment = 2.0 * math.pi / n
        now = time.time()
        msg.scan_time = max(0.05, min(1.0, now - self.rev_start if self.rev_start else 0.1))
        msg.time_increment = msg.scan_time / n
        msg.range_min = RANGE_MIN
        msg.range_max = RANGE_MAX
        ranges = []
        intens = []
        src = self.display_bins if self.display_bins else self.bins
        if not src:
            return
        for i in range(n):
            item = src.get(i)
            if item is None:
                ranges.append(float("inf"))
                intens.append(0.0)
            else:
                ranges.append(item[0])
                intens.append(item[1])
        msg.ranges = ranges
        msg.intensities = intens
        self.pub.publish(msg)
        valid = sum(1 for r in ranges if r < RANGE_MAX)
        self.get_logger().info("N10P scan valid=%d/%d dt=%.3f" % (valid, n, msg.scan_time), throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = RealLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if node.ser:
        try:
            node.ser.close()
        except Exception:
            pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
