#!/usr/bin/env python3
import math
import time
import rclpy
from sensor_msgs.msg import LaserScan

rclpy.init()
node = rclpy.create_node("dump_scan_stats")
box = {}


def cb(msg):
    if box or not msg.ranges:
        return
    vals = [(i, r) for i, r in enumerate(msg.ranges) if math.isfinite(r) and msg.range_min < r < msg.range_max]
    n = len(msg.ranges)
    print("n", n, "valid", len(vals), "range_max", msg.range_max, "scan_time", round(msg.scan_time, 3))
    if vals and n > 0:
        rs = [r for _, r in vals]
        print("min", round(min(rs), 3), "max", round(max(rs), 3), "mean", round(sum(rs) / len(rs), 3))
        print("angle_span", round(vals[0][0] * 360.0 / n, 1), round(vals[-1][0] * 360.0 / n, 1))
        buckets = [0] * 17
        for r in rs:
            buckets[min(16, int(r))] += 1
        print("hist_m", buckets)
        intens = [msg.intensities[i] for i, _ in vals] if msg.intensities else []
        if intens:
            print("inten min/max/mean", min(intens), max(intens), round(sum(intens) / len(intens), 2))
    box["ok"] = True


node.create_subscription(LaserScan, "/scan", cb, 10)
t0 = time.time()
while not box and time.time() - t0 < 5:
    rclpy.spin_once(node, timeout_sec=0.2)
print("got" if box else "NO")
node.destroy_node()
rclpy.shutdown()
