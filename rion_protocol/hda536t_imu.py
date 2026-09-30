#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gen5 的 IMU：瑞芬 HDA536T 动态倾角传感器（CAN），与两台欧艾迪驱动器挂同一条总线。

    发布  /imu        sensor_msgs/Imu：陀螺 + 加速度，不带姿态（orientation_covariance[0] = −1，ROS 约定），
                      与旧车 Wheeltec 驱动发的 /imu 相同；line_node 的融合用它的 z 轴角速度。
          /imu/data   sensor_msgs/Imu：同上，再带传感器自己算好的姿态（内置卡尔曼滤波，离心加速度已补偿），
                      代替旧车上 imu_filter_madgwick 算的姿态，app 显示横滚、俯仰。偏航上电为 0，是相对值。
          /imu/status std_msgs/String（JSON，1 Hz）：收没收到、各帧频率、自检、安装方式、温度、问题。

坐标系 ROS REP-103（x 车头、y 左、z 上），frame_id 用共享配置的 frames.imu。时间戳是本节点读到那一帧的时刻。
只收不发：不改传感器设置（改了要重新上电才生效），设置用 hda536t_setup.py。

每个输出周期传感器按「角度帧、加速度帧、陀螺帧」的顺序连发，本节点在陀螺帧到时发一次（同一周期的角度、
加速度一起带上）。自检失败、安装方式不对、没有陀螺帧时不发 /imu：融合报「IMU 数据中断」，车不会开，
/imu/status 里写明原因。
"""
import json
import math
import os
import socket
import struct
import sys
import threading
import time
from collections import deque
from functools import wraps

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

import hda536t_codec as hc  # noqa: E402
from robot_core.service_watchdog import watchdog

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    ROS_AVAILABLE = True
except ImportError:         # 电脑上跑单元测试、设置脚本
    Node = object
    ROS_AVAILABLE = False

RATE_WINDOW_S = 2.0
MIN_GYRO_HZ = 20.0          # 融合要求 IMU 0.2 s 内有新数据；低于这个频率在状态里提醒
GYRO_VAR = math.radians(0.05) ** 2
ACCEL_VAR = (0.005 * hc.G) ** 2
TILT_VAR = math.radians(0.1) ** 2       # 手册：横滚俯仰动态精度 ±0.1° rms
YAW_VAR = 1.0                           # 偏航只是相对上电时，没有绝对参考


class CanSocket:
    """Linux SocketCAN 原始套接字（python 自带）。标准帧、扩展帧都收；extended 决定发送用哪种。
    接口要先起来：sudo ip link set can0 up type can bitrate 500000"""
    FMT = '=IB3x8s'

    def __init__(self, channel='can0', extended=False, timeout_s=0.2):
        if not hasattr(socket, 'AF_CAN'):
            raise RuntimeError('本机没有 SocketCAN（只能在 Linux 上用）')
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((channel,))
        self.sock.settimeout(timeout_s)
        self.channel, self.extended = channel, extended

    def send(self, can_id, data):
        data = bytes(data)
        cid = (can_id & socket.CAN_EFF_MASK) | socket.CAN_EFF_FLAG if self.extended else can_id & 0x7FF
        self.sock.send(struct.pack(self.FMT, cid, len(data), data.ljust(8, b'\0')))

    def recv(self):
        """等一帧，超时返回 None。返回 (can_id, data)，远程帧、错误帧跳过。"""
        while True:
            try:
                raw = self.sock.recv(16)
            except socket.timeout:
                return None
            cid, dlc, data = struct.unpack(self.FMT, raw)
            if cid & (socket.CAN_RTR_FLAG | socket.CAN_ERR_FLAG):
                continue
            mask = socket.CAN_EFF_MASK if cid & socket.CAN_EFF_FLAG else 0x7FF
            return cid & mask, data[:dlc]

    def close(self):
        self.sock.close()


def _synchronized(method):
    @wraps(method)
    def call(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)
    return call


class Hda536t:
    """不依赖 ROS 的接收逻辑：认帧、凑齐一个周期、换到 REP-103、记状态。t 是单调时钟秒。"""

    def __init__(self, node_id=5, pdo_id=0, stale_s=0.1, invert_accel=False, mount_yaw_deg=0.0):
        if not 1 <= node_id <= 0x7F:
            raise ValueError('节点号范围 1–127')
        if not math.isfinite(mount_yaw_deg):
            raise ValueError('mount_yaw_deg 必须是有限数')
        self.node_id, self.pdo_id, self.stale_s, self.invert_accel = node_id, pdo_id, stale_s, invert_accel
        self.mount_yaw_deg = mount_yaw_deg      # 外壳 X 箭头在车上朝哪（逆时针为正），见 hda536t_codec.to_ros
        self._lock = threading.RLock()  # CAN reader and ROS status timer share this state
        self.last = {}                  # kind -> (t, Frame)
        self.times = {k: deque() for k in hc.SCALES}
        self.bad = 0
        self.can_id = None
        self.started = None

    @_synchronized
    def feed(self, t, can_id, data):
        """收到一帧。陀螺帧到了、传感器正常时返回一组数据（dict：t、angular_velocity、linear_acceleration、
        orientation、rpy、temp_c，没有的项不带），其余返回 None。"""
        if self.started is None:
            self.started = t
        if hc.pdo_node(can_id, self.pdo_id) != self.node_id:
            return None
        frame = hc.decode_pdo(data)
        if frame is None:
            self.bad += 1
            return None
        self.can_id = can_id
        self.last[frame.kind] = (t, frame)
        q = self.times[frame.kind]
        q.append(t)
        while q and t - q[0] > RATE_WINDOW_S:
            q.popleft()
        if frame.kind != hc.GYRO or self._blocking(frame):
            return None
        angle, accel = self._fresh(hc.ANGLE, t), self._fresh(hc.ACCEL, t)
        out = dict(t=t, **hc.to_ros(angle, accel, frame, self.invert_accel, self.mount_yaw_deg))
        if angle is not None:
            out['temp_c'] = angle.temp_c
        return out

    def _fresh(self, kind, t):
        v = self.last.get(kind)
        return v[1] if v is not None and 0.0 <= t - v[0] <= self.stale_s else None

    @staticmethod
    def _blocking(frame):
        """不能发 /imu 的原因；能发返回 None。"""
        if frame.self_test == 2:
            return '传感器自检失败'
        if frame.self_test != 0:
            return '自检标志是 %s，手册里没有这个值' % format(frame.self_test, '02b')
        if frame.mount != 0:
            return '传感器报的安装方式是「%s」，本驱动只按水平安装换算' % hc.MOUNTS.get(frame.mount, str(frame.mount))
        return None

    @_synchronized
    def rate(self, kind, now):
        q = [x for x in self.times[kind] if 0.0 <= now - x <= RATE_WINDOW_S]
        if len(q) < 2:
            return 0.0
        return (len(q) - 1) / max(q[-1] - q[0], 1e-6)

    @_synchronized
    def problem(self, now):
        """/imu 发不出来或不可靠的原因（给人看）；正常时 None。"""
        if not self.last:
            waited = now - self.started if self.started is not None else 0.0
            return '没有收到 IMU 数据帧（查供电、CAN 接线、波特率、节点号 %d）%s' % (
                self.node_id, '，已等 %.0f s' % waited if waited >= 1 else '')
        newest = max(t for t, _ in self.last.values())
        if now - newest > 0.5:
            return 'IMU 数据中断 %.1f s' % (now - newest)
        frame = max(self.last.values(), key=lambda v: v[0])[1]
        blocking = self._blocking(frame)
        if blocking:
            return blocking
        gyro = self.last.get(hc.GYRO)
        if gyro is None or now - gyro[0] > 0.5:
            return '没有陀螺帧：传感器还是出厂设置（只发角度帧），用 hda536t_setup.py gen5 打开'
        hz = self.rate(hc.GYRO, now)
        if hz < MIN_GYRO_HZ:
            return '陀螺帧只有 %.0f Hz，融合要 %.0f Hz 以上：用 hda536t_setup.py gen5 设成 50 Hz' % (hz, MIN_GYRO_HZ)
        return None

    @_synchronized
    def status(self, now):
        angle = self.last.get(hc.ANGLE)
        frame = max(self.last.values(), key=lambda v: v[0])[1] if self.last else None
        out = {'node_id': self.node_id, 'can_id': self.can_id,
               'age_s': round(now - max(t for t, _ in self.last.values()), 3) if self.last else None,
               'rates_hz': {hc.KIND_NAMES[k]: round(self.rate(k, now), 1) for k in hc.SCALES},
               'self_test': None if frame is None else ('ok' if frame.self_test_ok else format(frame.self_test, '02b')),
               'mount': None if frame is None else hc.MOUNTS.get(frame.mount, str(frame.mount)),
               'temp_c': angle[1].temp_c if angle else None, 'bad_frames': self.bad,
               'problem': self.problem(now)}
        if angle and now - angle[0] <= 0.5:
            roll, pitch, yaw = hc.to_ros(angle=angle[1], mount_yaw_deg=self.mount_yaw_deg)['rpy']
            out.update(roll_deg=round(math.degrees(roll), 2), pitch_deg=round(math.degrees(pitch), 2),
                       yaw_deg=round(math.degrees(yaw), 2))
        return out


def fill_imu(msg, sample, stamp, frame_id, with_orientation):
    msg.header.stamp, msg.header.frame_id = stamp, frame_id
    wx, wy, wz = sample['angular_velocity']
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = wx, wy, wz
    for i in (0, 4, 8):
        msg.angular_velocity_covariance[i] = GYRO_VAR
    acc = sample.get('linear_acceleration')
    if acc is None:
        msg.linear_acceleration_covariance[0] = -1.0        # 这一周期没有加速度帧
    else:
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = acc
        for i in (0, 4, 8):
            msg.linear_acceleration_covariance[i] = ACCEL_VAR
    q = sample.get('orientation') if with_orientation else None
    if q is None:
        msg.orientation_covariance[0] = -1.0
    else:
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
        msg.orientation_covariance[0] = msg.orientation_covariance[4] = TILT_VAR
        msg.orientation_covariance[8] = YAW_VAR
    return msg


class Hda536tNode(Node):
    def __init__(self, bus=None, clock=time.monotonic):
        super().__init__('hda536t_imu')
        from robot_core.config import PROFILE
        for name, value in (('can_channel', 'can0'), ('node_id', 5), ('pdo_id', 0), ('extended', False),
                            ('stale_s', 0.1), ('invert_accel', False), ('mount_yaw_deg', 0.0),
                            ('frame_id', PROFILE['frames']['imu'])):
            self.declare_parameter(name, value)
        p = {n: self.get_parameter(n).value for n in ('can_channel', 'node_id', 'pdo_id', 'extended',
                                                       'stale_s', 'invert_accel', 'mount_yaw_deg', 'frame_id')}
        self.frame_id, self.clock = p['frame_id'], clock
        self.imu = Hda536t(int(p['node_id']), int(p['pdo_id']), float(p['stale_s']), bool(p['invert_accel']),
                           float(p['mount_yaw_deg']))
        self.bus = bus or CanSocket(p['can_channel'], bool(p['extended']))
        self.pub_raw = self.create_publisher(Imu, '/imu', 20)
        self.pub_data = self.create_publisher(Imu, '/imu/data', 20)
        self.pub_status = self.create_publisher(String, '/imu/status', 1)
        self.create_timer(1.0, self.publish_status)
        self.running = True
        self._reported = None

    def poll(self):
        """读一帧（超时返回 False），陀螺帧凑齐一组就发 /imu 和 /imu/data。"""
        got = self.bus.recv()
        if got is None:
            return False
        sample = self.imu.feed(self.clock(), *got)
        if sample is not None:
            stamp = self.get_clock().now().to_msg()
            self.pub_raw.publish(fill_imu(Imu(), sample, stamp, self.frame_id, False))
            self.pub_data.publish(fill_imu(Imu(), sample, stamp, self.frame_id, True))
        return True

    def spin_bus(self):
        while self.running:
            watchdog.tick("can")
            try:
                self.poll()
            except OSError as exc:      # 接口掉了（USB-CAN 拔出）：等一会再读，状态里会报中断
                self.get_logger().error('CAN 读取失败：%s' % exc)
                time.sleep(0.5)

    def publish_status(self):
        st = self.imu.status(self.clock())
        self.pub_status.publish(String(data=json.dumps(st, ensure_ascii=False)))
        if st['problem'] != self._reported:
            (self.get_logger().warn if st['problem'] else self.get_logger().info)(st['problem'] or 'IMU 正常')
            self._reported = st['problem']


def main():
    if not ROS_AVAILABLE:
        raise SystemExit('需要 ROS 2（rclpy）')
    watchdog.start("ros", "can")
    rclpy.init()
    node = Hda536tNode()
    node.create_timer(0.5, lambda: watchdog.tick("ros"))
    reader = threading.Thread(target=node.spin_bus, daemon=True)
    reader.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        watchdog.close()
        node.running = False
        reader.join(timeout=1.0)
        node.bus.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
