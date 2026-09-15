#!/usr/bin/env python3
"""Local ROS client for the RK3588 chassis service. Motion always has a deadline."""
import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from ackermann_msgs.msg import AckermannDriveStamped


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['status', 'arm', 'stop', 'drive'])
    parser.add_argument('--speed', type=float, default=0, help='m/s')
    parser.add_argument('--steering-deg', type=float, default=0, help='front steering angle in degrees')
    parser.add_argument('--seconds', type=float, default=0.5, help='bounded command duration, max 3 seconds')
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.speed, args.steering_deg, args.seconds)) or not 0 < args.seconds <= 3:
        parser.error('finite values and duration in (0, 3] required')
    rclpy.init()
    node = Node('wheeltec_control_client')
    state = {}
    node.create_subscription(String, '/wheeltec/status', lambda m: state.update(data=json.loads(m.data), received=time.monotonic()), 1)
    arm = node.create_client(SetBool, '/wheeltec/arm')
    stop = node.create_client(Trigger, '/wheeltec/stop')
    publisher = node.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 1)

    def service(client, request):
        if not client.wait_for_service(timeout_sec=2):
            raise RuntimeError('service unavailable')
        future = client.call_async(request)
        deadline = time.monotonic() + 2
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not future.done():
            raise RuntimeError('service response timeout')
        result = future.result()
        print(json.dumps({'success': result.success, 'message': result.message}, ensure_ascii=False))
        if not result.success:
            raise RuntimeError(result.message)

    try:
        deadline = time.monotonic() + 4
        while not state and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if args.action == 'stop':
            service(stop, Trigger.Request())
        elif not state:
            raise RuntimeError('no current chassis status')
        elif args.action == 'status':
            print(json.dumps(state['data'], indent=2, ensure_ascii=False))
        elif args.action == 'arm':
            service(arm, SetBool.Request(data=True))
        else:
            data = state['data']
            if not data['armed'] or data['config']['receive_only']:
                raise RuntimeError('driver must already be armed with a confirmed firmware profile')
            cfg = data['config']
            if abs(args.speed) > cfg['max_speed_m_s'] or abs(math.radians(args.steering_deg)) > cfg['max_steering_rad']:
                raise RuntimeError('requested command exceeds configured commissioning limits')
            try:
                end = time.monotonic() + args.seconds
                while time.monotonic() < end:
                    if time.monotonic() - state['received'] > 0.4 or not state['data']['armed']:
                        raise RuntimeError('driver status lost or disarmed')
                    message = AckermannDriveStamped()
                    message.header.stamp = node.get_clock().now().to_msg()
                    message.drive.speed = float(args.speed)
                    message.drive.steering_angle = math.radians(args.steering_deg)
                    publisher.publish(message)
                    until = time.monotonic() + 0.05
                    while time.monotonic() < until:
                        rclpy.spin_once(node, timeout_sec=0.005)
            finally:
                service(stop, Trigger.Request())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
