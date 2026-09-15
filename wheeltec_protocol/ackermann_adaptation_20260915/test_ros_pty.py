#!/usr/bin/env python3
"""Real ROS + pyserial test against an isolated pseudo-terminal, never a vehicle."""
import importlib.util
import json
import os
import pty
import select
import signal
import struct
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_srvs.srv import SetBool, Trigger

SAMPLE = bytes.fromhex('7b 00 00 00 00 00 00 00 ff 7c ff ce 3f d0 ff fd ff ff 00 06 58 00 7a 7d')
master, slave = pty.openpty()
port = os.ttyname(slave)
assert port.startswith('/dev/pts/')
capture = bytearray()
flags = {'running': True, 'feedback': True}


def simulator():
    next_rx = 0
    while flags['running']:
        if flags['feedback'] and time.monotonic() >= next_rx:
            os.write(master, SAMPLE)
            next_rx = time.monotonic() + 0.05
        if select.select([master], [], [], 0.005)[0]:
            capture.extend(os.read(master, 65536))


def frames_since(index):
    data = bytes(capture[index:])
    frames = []
    for i in range(len(data)-10):
        f = data[i:i+11]
        if f[0] == 123 and f[-1] == 125 and M.bcc(f[:9]) == f[9]:
            frames.append(f)
    return frames


spec = importlib.util.spec_from_file_location('adapter', sys.argv[1])
M = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = M
spec.loader.exec_module(M)
rclpy.init(args=['--ros-args', '-p', 'port:='+port, '-p', 'allow_test_port:=true',
                 '-p', 'receive_only:=false', '-p', 'protocol:=steering_angle',
                 '-p', 'protocol_confirmed:=true', '-p', 'mode_byte:=1',
                 '-p', 'steering_scale:=0.5', '-p', 'wheelbase_m:=0.3'])
driver = M.WheeltecDriver()
client = Node('adapter_test_client')
executor = SingleThreadedExecutor()
executor.add_node(driver)
executor.add_node(client)
publisher = client.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 1)
arm_client = client.create_client(SetBool, '/wheeltec/arm')
stop_client = client.create_client(Trigger, '/wheeltec/stop')
thread = threading.Thread(target=simulator, daemon=True)
thread.start()
results = {}


def spin(seconds, speed=None, steer=0):
    end = time.monotonic() + seconds
    next_cmd = 0
    while time.monotonic() < end:
        now = time.monotonic()
        if speed is not None and now >= next_cmd:
            msg = AckermannDriveStamped()
            msg.header.stamp = client.get_clock().now().to_msg()
            msg.drive.speed, msg.drive.steering_angle = float(speed), float(steer)
            publisher.publish(msg)
            next_cmd = now + 0.05
        executor.spin_once(timeout_sec=0.005)


def arm():
    future = arm_client.call_async(SetBool.Request(data=True))
    deadline = time.monotonic() + 2
    while not future.done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
    assert future.done(), 'arm service timeout'
    return future.result()


try:
    spin(0.5)
    assert driver.policy.connected and driver.parser.good >= 5
    assert not arm().success
    spin(2.8)
    assert arm().success
    index = len(capture)
    spin(0.6, speed=0, steer=0.2)
    frames = frames_since(index)
    assert len(frames) >= 20
    assert all(f[3:7] == bytes(4) for f in frames)
    assert any(struct.unpack('>h', f[7:9])[0] > 0 for f in frames)
    results['stationary_steering_no_drive'] = 'PASS'
    spin(0.6)
    frames = frames_since(index)
    assert frames[-10:] == [M.STOP_FRAME]*10
    assert not driver.policy.armed
    results['deadman_persistent_stop'] = 'PASS'
    assert arm().success
    spin(0.3, speed=0.1, steer=0)
    assert any(struct.unpack('>h', f[3:5])[0] > 0 for f in frames_since(index)[-10:])
    flags['feedback'] = False
    spin(0.6, speed=0.1, steer=0)
    assert not driver.policy.armed
    assert frames_since(index)[-10:] == [M.STOP_FRAME]*10
    results['telemetry_loss_stops_with_live_commands'] = 'PASS'
    flags['feedback'] = True
    spin(0.4)
    assert arm().success
    msg = AckermannDriveStamped()
    msg.header.stamp.sec = 1
    msg.drive.speed = 0.1
    publisher.publish(msg)
    spin(0.1)
    assert not driver.policy.armed
    results['stale_timestamp_rejected'] = 'PASS'
    assert driver.io_errors == 0, driver.last_error
    index = len(capture)
    driver.shutdown()
    spin(0.03)
    frames = frames_since(index)
    assert len(frames) >= 100 and all(f == M.STOP_FRAME for f in frames)
    results['shutdown_persistent_stop'] = 'PASS'
    print('ROS_PTY '+json.dumps(results, sort_keys=True))
    print('ROS_PTY_RESULT=PASS physical_device_access=0')
finally:
    if driver.running:
        driver.shutdown()
    flags['running'] = False
    thread.join(1)
    driver.destroy_node()
    client.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
    os.close(master)
    os.close(slave)
