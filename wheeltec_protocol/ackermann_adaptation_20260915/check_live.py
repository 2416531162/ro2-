#!/usr/bin/env python3
"""Read-only acceptance check of actual ROS telemetry and deployment guards."""
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool

rclpy.init()
node = Node('wheeltec_acceptance')
counts = {'odom': 0, 'imu': 0, 'voltage': 0, 'status': 0}
state = {}


def receive(name, message):
    counts[name] += 1
    if name == 'status':
        state.update(json.loads(message.data))


for name, topic, cls in [('odom', '/odom', Odometry), ('imu', '/imu', Imu),
                         ('voltage', '/voltage', Float32), ('status', '/wheeltec/status', String)]:
    node.create_subscription(cls, topic, lambda msg, n=name: receive(n, msg), 10)
deadline = time.monotonic() + 5
try:
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
    assert all(v >= 5 for v in counts.values()), counts
    assert state['connected'] and state['config']['receive_only'] and not state['armed'], state
    assert state['tx_packets'] == 0 and state['frames_bad'] == 0 and state['io_errors'] == 0, state
    assert state['age_ms'] < 300 and 15 < state['hz'] < 25, state
    print('LIVE_ROS '+json.dumps({'counts': counts, 'state': state}, ensure_ascii=False, sort_keys=True))
    print('LIVE_ROS_RESULT=PASS telemetry_only=true tx_packets=0')
finally:
    node.destroy_node()
    rclpy.shutdown()
