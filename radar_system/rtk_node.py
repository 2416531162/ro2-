#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RTK 节点：独占 UM982 串口（雷迅 CUAV 双天线），发布测量和状态；有账号文件时跑 NTRIP 差分。

    /gnss/raw      std_msgs/String JSON，每条测量一条（fix / heading / vel，字段见 um982.Um982Stream）；
                   时间是本机单调时钟的测量时刻，走直线节点（融合）用它
    /gnss/status   std_msgs/String JSON，2 Hz：定位等级、卫星数、差分龄期、航向、各日志频率、NTRIP 状态
    /gnss/fix      sensor_msgs/NavSatFix，ANT1 位置（frame_id gnss），协方差 ENU 对角
    /gnss/heading  sensor_msgs/Imu，只有车头方向（ENU，东为 0 逆时针），已扣 ~/rtk/fusion.json 里的安装偏差；
                   航向无效时不发。后两个是 ROS 2 标准消息，Foxglove 地图、rviz2、rosbag、robot_localization 可直接用

CORS 账号在 ~/rtk/ntrip.env（权限 600，由人手工创建，本节点只读、不打印密码）。没有账号文件时照常
输出单点定位，状态里写明未配置差分。接收机的日志频率和基线配置记录见车上 ~/rtk/README.md。
"""
import argparse
import json
import math
import os
import signal
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import String

from um982 import RTK_FIXED, RTK_FLOAT, NtripClient, Um982Stream
from ntrip_config import NtripReload

DEFAULT_PORT = ('/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
                'b0d06fec6eabed1183a10e9aa7669f5d-if00-port0')
DEFAULT_ENV = '~/rtk/ntrip.env'
FUSION_CONFIG = '~/rtk/fusion.json'     # 标定（安装偏差）与走直线节点共用这一份

# sensor_msgs/NavSatStatus 的取值（照 nmea_navsat_driver 的对应：单点 FIX、伪距差分 SBAS、RTK 浮点/固定 GBAS）
NO_FIX, FIX, SBAS_FIX, GBAS_FIX = -1, 0, 1, 2
SERVICES = 1 | 2 | 4 | 8               # GPS | GLONASS | 北斗(COMPASS) | Galileo
DIAGONAL_KNOWN = 2
UNKNOWN_VAR = 1e6                       # 横滚、俯仰不知道：给很大的方差


def navsat_status(sol):
    if sol in RTK_FIXED or sol in RTK_FLOAT:
        return GBAS_FIX
    if sol == 'PSRDIFF':
        return SBAS_FIX
    if sol == 'SINGLE':
        return FIX
    return NO_FIX


def navsat_fix(m, stamp):
    """/gnss/raw 的 fix 消息 → sensor_msgs/NavSatFix。"""
    msg = NavSatFix()
    msg.header.stamp, msg.header.frame_id = stamp, 'gnss'
    msg.status = NavSatStatus(status=navsat_status(m['sol']), service=SERVICES)
    msg.latitude, msg.longitude, msg.altitude = m['lat'], m['lon'], m['height']
    msg.position_covariance = [m['sigma_e'] ** 2, 0.0, 0.0, 0.0, m['sigma_n'] ** 2, 0.0, 0.0, 0.0, m['sigma_u'] ** 2]
    msg.position_covariance_type = DIAGONAL_KNOWN
    return msg


def heading_valid(m):
    """Um982Stream 已按两天线固定解的实时距离核对过测向（见 um982.BASELINE_REF_*），这里只看它的结论。"""
    return bool(m.get('valid'))


def heading_imu(m, stamp, mount_offset_deg=0.0, baseline_yaw_body_deg=-90.0):
    """/gnss/raw 的 heading 消息 → 只带航向的 sensor_msgs/Imu。

    HEADING 是 ANT1→ANT2 方向（正北顺时针）；车头 ENU 偏航 = 90° − HEADING − 基线在车体系的方向 + 安装偏差，
    与 gnss_fusion 的换算相同。
    """
    yaw = math.radians(90.0 - m['heading'] - baseline_yaw_body_deg + mount_offset_deg)
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    msg = Imu()
    msg.header.stamp, msg.header.frame_id = stamp, 'base_link'
    msg.orientation.x = msg.orientation.y = 0.0
    msg.orientation.z, msg.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    var = math.radians(max(0.1, m['sigma_heading'])) ** 2
    msg.orientation_covariance = [UNKNOWN_VAR, 0.0, 0.0, 0.0, UNKNOWN_VAR, 0.0, 0.0, 0.0, var]
    msg.angular_velocity_covariance = [-1.0] + [0.0] * 8        # 不提供角速度、加速度
    msg.linear_acceleration_covariance = [-1.0] + [0.0] * 8
    return msg


def load_heading_calibration(path=FUSION_CONFIG):
    """从融合标定文件读安装偏差和基线方向；没有文件就用默认值。"""
    try:
        with open(os.path.expanduser(path), encoding='utf-8') as fh:
            cfg = json.load(fh)
        return float(cfg.get('mount_offset_deg', 0.0)), float(cfg.get('baseline_yaw_body_deg', -90.0))
    except FileNotFoundError:
        return 0.0, -90.0


class RtkNode(Node):
    def __init__(self, port, baud=921600, ntrip_env=DEFAULT_ENV, serial_factory=None, fusion_config=FUSION_CONFIG):
        super().__init__('rtk_node')
        self.port, self.baud = port, baud
        self.serial_factory = serial_factory
        self.pub_raw = self.create_publisher(String, '/gnss/raw', 50)
        self.pub_status = self.create_publisher(String, '/gnss/status', 1)
        self.pub_fix = self.create_publisher(NavSatFix, '/gnss/fix', 10)
        self.pub_heading = self.create_publisher(Imu, '/gnss/heading', 10)
        try:
            self.mount_offset_deg, self.baseline_yaw_body_deg = load_heading_calibration(fusion_config)
        except (OSError, ValueError, TypeError) as exc:
            self.mount_offset_deg, self.baseline_yaw_body_deg = 0.0, -90.0
            self.get_logger().error('读不了标定文件 %s：%s（航向先不扣安装偏差）' % (fusion_config, exc))
        self.stream = Um982Stream()
        self.stream_lock = threading.Lock()
        self.write_lock = threading.Lock()
        self.ser = None
        self.port_error = None
        self.running = True
        self.cors = NtripReload(ntrip_env, lambda env: NtripClient(env, self.gga_report, self.write_corrections))
        self.create_timer(0.5, self.publish_status)
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

    # ---- 串口 ----
    def open_serial(self):
        if self.serial_factory is not None:
            return self.serial_factory()
        import serial
        return serial.Serial(self.port, self.baud, timeout=0.1)

    def read_loop(self):
        # 读线程不能悄悄死掉：任何异常都写进状态（port_error）再重试，app 和日志里看得到
        while self.running:
            if self.ser is None:
                try:
                    ser = self.open_serial()
                    with self.write_lock:
                        # 只增加副天线 20 Hz 日志（与主天线 BESTPOS 同频，按测量时刻配对检查两天线距离），
                        # 不清除已有输出、不写接收机闪存。
                        ser.write(b'BESTNAVHA 0.05\r\n')
                        self.ser = ser
                    self.port_error = None
                except Exception as exc:        # 串口不存在、没权限、pyserial 没装……
                    self.port_error = '%s: %s' % (type(exc).__name__, exc)
                    time.sleep(2.0)
                    continue
            try:
                data = self.ser.read(4096)
            except Exception as exc:
                # USB 掉线：关掉重开，状态里报出来
                self.port_error = '%s: %s' % (type(exc).__name__, exc)
                with self.write_lock:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                continue
            if data:
                try:
                    self.on_serial_data(data, time.monotonic())
                except Exception as exc:        # 解析或发布出了意外：丢掉这一块数据，记下来，接着读
                    self.port_error = 'data: %s: %s' % (type(exc).__name__, exc)
                    self.get_logger().error('处理串口数据出错：%r' % exc)

    def on_serial_data(self, data, t_rx):
        # 不能叫 handle：会遮住 rclpy.Node.handle，Node.__init__ 里 `with self.handle:` 就崩了
        with self.stream_lock:
            messages = self.stream.feed(data, t_rx)
        for m in messages:
            self.pub_raw.publish(String(data=json.dumps(m, allow_nan=False)))
            if m['kind'] == 'fix':
                self.pub_fix.publish(navsat_fix(m, self.stamp(m['t'])))
            elif m['kind'] == 'heading' and heading_valid(m):
                self.pub_heading.publish(heading_imu(m, self.stamp(m['t']), self.mount_offset_deg,
                                                     self.baseline_yaw_body_deg))

    def stamp(self, t_meas):
        """单调时钟的测量时刻 → ROS 时间戳（标准消息的 header 用，和其他节点的时间一致）。"""
        age_ns = int(max(0.0, time.monotonic() - t_meas) * 1e9)
        return Time(nanoseconds=self.get_clock().now().nanoseconds - age_ns).to_msg()

    def gga_report(self):
        with self.stream_lock:
            return self.stream.gga_report()

    def write_corrections(self, data):
        with self.write_lock:
            if self.ser is not None:
                try:
                    self.ser.write(data)
                except OSError as exc:
                    self.port_error = str(exc)

    # ---- 状态 ----
    def status(self, now=None):
        now = time.monotonic() if now is None else now
        with self.stream_lock:
            st = self.stream.status(now)
        st['port_open'] = self.ser is not None
        st['port_error'] = self.port_error
        if self.cors.client is not None:
            n = self.cors.client.status()
            st['ntrip'] = dict(configured=True, connected=n['connected'], mount=n['mount'],
                               bytes=n['bytes'], rtcm_types=n['rtcm_types'], error=n['error'],
                               age_s=None if n['last_rx'] is None else round(now - n['last_rx'], 1))
        else:
            st['ntrip'] = dict(configured=self.cors.configured, connected=False, error=self.cors.error)
        st['ntrip']['enabled'] = self.cors.enabled
        st['ntrip']['config_revision'] = self.cors.applied_revision
        st['ntrip']['config_error'] = self.cors.error
        return st

    def publish_status(self):
        self.cors.poll()
        self.pub_status.publish(String(data=json.dumps(self.status(), allow_nan=False)))

    def shutdown(self):
        self.running = False
        self.cors.stop()


def main():
    p = argparse.ArgumentParser(description='UM982 RTK 节点')
    p.add_argument('--port', default=os.environ.get('RK3588_RTK_PORT', DEFAULT_PORT))
    p.add_argument('--baud', type=int, default=921600)
    p.add_argument('--ntrip-env', default=os.environ.get('RK3588_NTRIP_ENV', DEFAULT_ENV))
    argv = __import__('sys').argv[1:]
    args = p.parse_args(argv[:argv.index('--ros-args')] if '--ros-args' in argv else argv)
    rclpy.init()
    node = RtkNode(args.port, args.baud, args.ntrip_env)

    def on_signal(_sig=None, _frame=None):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
