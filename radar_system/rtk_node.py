#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RTK 节点：独占 UM982 串口（雷迅 CUAV 双天线），发布测量和状态；有账号文件时跑 NTRIP 差分。

接收机可以接在车上的 USB 串口，也可以接在安卓平板的串口上：平板 app 把串口原始字节经 TCP 48622
原样转过来（gnss_bridge.TabletSerialServer），或者走蓝牙 RFCOMM（gnss_bridge.BluetoothSerialServer，
平板不用连车的 WiFi），或者走 CAN（gnss_bridge.CanSerialBridge，平板 MCU 的 CAN 口接车的 can0）。几路同时监听，正在出数据的那一路一直用下去，它 2 s 没数据才换另一路；几路的数据从不
混在一起（换路时解析状态、钟差全部重来）。平板转来的数据（WiFi 或蓝牙）的差分由平板用同一个 CORS 账号自己连、
直接写进平板串口，车端这时暂停自己的 CORS（同一账号两处登录会互相顶掉），换回 USB 再连。

    /gnss/raw      std_msgs/String JSON，每条测量一条（fix / heading / vel，字段见 um982.Um982Stream）；
                   时间是本机单调时钟的测量时刻，走直线节点（融合）用它
    /gnss/status   std_msgs/String JSON，2 Hz：定位等级、卫星数、差分龄期、航向、各日志频率、NTRIP 状态；
                   测向有效时 heading.yaw_deg 是双天线测出的车头方向（与 /gnss/heading 同一换算），否则为 null
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
import runtime_config  # bootstraps the release root for direct script launches
from robot_core.service_watchdog import watchdog
from gnss_bridge import BT_CHANNEL, CAN_CHANNEL, TABLET_PORT, BluetoothSerialServer, CanSerialBridge, TabletSerialServer

DEFAULT_PORT = ('/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
                'b0d06fec6eabed1183a10e9aa7669f5d-if00-port0')
DEFAULT_ENV = '~/rtk/ntrip.env'
FUSION_CONFIG = '~/rtk/fusion.json'     # 标定（安装偏差）与走直线节点共用这一份
# 只增加副天线 20 Hz 日志（与主天线 BESTPOS 同频，按测量时刻配对检查两天线距离），不清除已有输出、不写接收机闪存
ANT2_LOG = b'BESTNAVHA 0.05\r\n'
SOURCE_HOLD_S = 2.0                     # 正在用的一路（USB / 平板 WiFi / 平板蓝牙 / 平板 CAN）这么久没数据才换另一路
# 平板转来的几路（WiFi TCP、蓝牙、CAN）：差分由平板自己连 CORS，车端暂停自己的 CORS、不往这几路写改正数
TABLET_SOURCES = ('tablet', 'bluetooth', 'can')
SOURCE_TEXT = {'tablet': '串口', 'bluetooth': '蓝牙', 'can': 'CAN'}

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
    """Um982Stream 已按整周固定 + 基线长度范围判过测向（见 um982.heading_usable），这里只看它的结论。"""
    return bool(m.get('valid'))


def vehicle_yaw(heading_deg, mount_offset_deg=0.0, baseline_yaw_body_deg=-90.0):
    """HEADING（ANT1→ANT2 方向，正北顺时针，度）→ 车头 ENU 偏航（弧度，−π…π）：
    90° − HEADING − 基线在车体系的方向 + 安装偏差，与 gnss_fusion 的换算相同。"""
    yaw = math.radians(90.0 - heading_deg - baseline_yaw_body_deg + mount_offset_deg)
    return math.atan2(math.sin(yaw), math.cos(yaw))


def heading_imu(m, stamp, mount_offset_deg=0.0, baseline_yaw_body_deg=-90.0):
    """/gnss/raw 的 heading 消息 → 只带航向的 sensor_msgs/Imu（车头方向见 vehicle_yaw）。"""
    yaw = vehicle_yaw(m['heading'], mount_offset_deg, baseline_yaw_body_deg)
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
    def __init__(self, port, baud=921600, ntrip_env=DEFAULT_ENV, serial_factory=None, fusion_config=FUSION_CONFIG,
                 tablet_port=None, bt_channel=None, bt_factory=None, can_channel=None, can_factory=None):
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
        self.source = None                  # 正在用的接收机数据：'usb' / 'tablet' / 'bluetooth' / 'can'，还没收到过为 None
        self.source_rx = {}                 # 每一路最后收到数据的时刻
        self.running = True
        self.cors = NtripReload(ntrip_env, lambda env: NtripClient(env, self.gga_report, self.write_corrections))
        # 平板串口转发（tablet_port=None 不监听；0 由系统分配端口，测试用）
        self.tablet, self.tablet_error = None, None
        if tablet_port is not None:
            try:
                self.tablet = TabletSerialServer(tablet_port, on_connect=self.tablet_connected)
            except OSError as exc:          # 端口被占：只用 USB，状态里写明
                self.tablet_error = '%s: %s' % (type(exc).__name__, exc)
                self.get_logger().error('平板串口转发端口 %s 打不开：%s' % (tablet_port, exc))
        # 平板蓝牙转发（bt_channel=None 不开；bt_factory 测试用，给一个不碰 BlueZ 的假服务）
        self.bt = None
        if bt_channel is not None:
            factory = bt_factory or (lambda: BluetoothSerialServer(bt_channel, on_connect=self.bt_connected))
            self.bt = factory()
            if self.bt.error:
                self.get_logger().error('平板蓝牙转发不可用：%s' % self.bt.error)
        # 平板 CAN 转发（can_channel=None 不开；can_factory 测试用）
        self.can = None
        if can_channel is not None:
            self.can = (can_factory or (lambda: CanSerialBridge(can_channel, on_connect=self.can_connected,
                                                                in_use=lambda: self.source == 'can')))()
            if self.can.error:
                self.get_logger().error('平板 CAN 转发暂不可用（会重试）：%s' % self.can.error)
        self.create_timer(0.5, self.publish_status)
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()
        if self.tablet is not None:
            self.tablet_reader = threading.Thread(target=self.bridge_loop, args=(self.tablet, 'tablet'), daemon=True)
            self.tablet_reader.start()
        if self.bt is not None:
            self.bt_reader = threading.Thread(target=self.bridge_loop, args=(self.bt, 'bluetooth'), daemon=True)
            self.bt_reader.start()
        if self.can is not None:
            self.can_reader = threading.Thread(target=self.bridge_loop, args=(self.can, 'can'), daemon=True)
            self.can_reader.start()

    # ---- 串口 ----
    def open_serial(self):
        if self.serial_factory is not None:
            return self.serial_factory()
        import serial
        return serial.Serial(self.port, self.baud, timeout=0.1)

    def read_loop(self):
        # 读线程不能悄悄死掉：任何异常都写进状态（port_error）再重试，app 和日志里看得到
        while self.running:
            watchdog.tick("serial")
            if self.ser is None:
                try:
                    ser = self.open_serial()
                    with self.write_lock:
                        ser.write(ANT2_LOG)
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

    # ---- 平板串口转发 ----
    def tablet_connected(self):
        # 单向：车不往平板写。副天线日志（BESTNAVHA）由平板 app 按串口带宽自己补（平板串口 115200，
        # 照 USB 那样固定发 20 Hz 会把串口挤满）；差分由平板自己连 CORS
        self.get_logger().info('平板串口转发已连接：%s' % self.tablet.peer)

    def bt_connected(self):
        self.get_logger().info('平板蓝牙转发已连接：%s' % self.bt.peer)

    def can_connected(self):
        self.get_logger().info('平板 CAN 转发开始收到数据：%s' % self.can.channel)

    def bridge_loop(self, bridge, source):
        """平板转来的一路（WiFi 'tablet' / 蓝牙 'bluetooth' / CAN 'can'）的读线程。"""
        while self.running:
            watchdog.tick(source)
            try:
                data = bridge.read(4096)
            except Exception as exc:        # 不让读线程悄悄死掉
                self._bridge_error(source, '%s: %s' % (type(exc).__name__, exc))
                time.sleep(0.5)
                continue
            if data:
                try:
                    self.on_serial_data(data, time.monotonic(), source)
                except Exception as exc:
                    self._bridge_error(source, 'data: %s: %s' % (type(exc).__name__, exc))
                    self.get_logger().error('处理平板%s数据出错：%r' % (SOURCE_TEXT[source], exc))

    def _bridge_error(self, source, text):
        if source == 'bluetooth':
            self.bt.error = text
        elif source == 'can':
            self.can.error = text
        else:
            self.tablet_error = text

    def on_serial_data(self, data, t_rx, source='usb'):
        # 不能叫 handle：会遮住 rclpy.Node.handle，Node.__init__ 里 `with self.handle:` 就崩了
        with self.stream_lock:
            self.source_rx[source] = t_rx
            if source != self.source:
                last = self.source_rx.get(self.source)
                if last is not None and t_rx - last < SOURCE_HOLD_S:
                    return                  # 另一路正在出数据：两台接收机（或同一台的两个口）不混用
                if self.source is not None:
                    self.get_logger().warning('接收机数据来源 %s → %s' % (self.source, source))
                self.source = source
                self.stream = Um982Stream()     # 换一路：钟差、两天线配对、上一条测量全部重来
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
        # 车端 CORS 只给 USB 上的接收机；平板转来的几路由平板自己连 CORS（这时车端 CORS 已暂停）
        if self.source in TABLET_SOURCES:
            return
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
            hd = self.stream.last.get('heading')
        if st.get('heading') is not None:
            # 双天线测出的车头方向（ENU 度，已扣安装偏差）：只在测向有效时给，无效时的数不能拿来显示
            ok = st['heading'].get('valid') and hd is not None and heading_valid(hd)
            st['heading']['yaw_deg'] = round(math.degrees(vehicle_yaw(
                hd['heading'], self.mount_offset_deg, self.baseline_yaw_body_deg)), 2) if ok else None
        tablet = self.tablet.status() if self.tablet is not None else None
        bt = self.bt.status() if self.bt is not None else None
        can = self.can.status() if self.can is not None else None
        st['source'] = self.source
        if self.source == 'tablet':
            st['port_open'] = bool(tablet and tablet['connected'])
            st['port_error'] = None if st['port_open'] else 'tablet disconnected'
        elif self.source == 'bluetooth':
            st['port_open'] = bool(bt and bt['connected'])
            st['port_error'] = None if st['port_open'] else 'bluetooth disconnected'
        elif self.source == 'can':
            st['port_open'] = bool(can and can['connected'])
            st['port_error'] = None if st['port_open'] else 'can silent'
        else:
            st['port_open'] = self.ser is not None
            st['port_error'] = self.port_error
        st['usb'] = dict(open=self.ser is not None, error=self.port_error)
        st['tablet'] = dict(listening=tablet is not None, error=self.tablet_error,
                            **(tablet or dict(port=None, connected=False, peer=None)))
        st['bluetooth'] = dict(listening=bt is not None,
                               **(bt or dict(channel=None, registered=False, connected=False, peer=None, error=None)))
        st['can'] = dict(listening=can is not None,
                         **(can or dict(channel=None, open=False, connected=False, peer=None, frames=0, gaps=0,
                                        bytes_in=0, acks=0, error=None)))
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
        st['ntrip']['paused'] = self.cors.paused
        return st

    def sync_cors(self):
        """数据来自平板（WiFi、蓝牙或 CAN）时暂停车端 CORS（平板自己连），换回 USB（或还没数据）时恢复。"""
        if self.source in TABLET_SOURCES:
            self.cors.pause('tablet')
        else:
            self.cors.resume()

    def publish_status(self):
        self.sync_cors()
        self.cors.poll()
        self.pub_status.publish(String(data=json.dumps(self.status(), allow_nan=False)))

    def shutdown(self):
        self.running = False
        self.cors.stop()
        if self.tablet is not None:
            self.tablet.close()
        if self.bt is not None:
            self.bt.close()
        if self.can is not None:
            self.can.close()


def main():
    p = argparse.ArgumentParser(description='UM982 RTK 节点')
    p.add_argument('--port', default=os.environ.get('RK3588_RTK_PORT', DEFAULT_PORT))
    p.add_argument('--baud', type=int, default=921600)
    p.add_argument('--ntrip-env', default=os.environ.get('RK3588_NTRIP_ENV', DEFAULT_ENV))
    p.add_argument('--tablet-port', type=int, default=int(os.environ.get('RK3588_RTK_TABLET_PORT', TABLET_PORT)),
                   help='平板串口转发的 TCP 端口，0 不监听')
    p.add_argument('--bt-channel', type=int, default=int(os.environ.get('RK3588_RTK_BT_CHANNEL', BT_CHANNEL)),
                   help='平板蓝牙转发的 RFCOMM 通道，0 不开')
    p.add_argument('--can-channel', default=os.environ.get('RK3588_RTK_CAN_CHANNEL', CAN_CHANNEL),
                   help='收平板 CAN 转发的接口（500 kbit/s，与驱动器同一条总线），空字符串不收')
    argv = __import__('sys').argv[1:]
    args = p.parse_args(argv[:argv.index('--ros-args')] if '--ros-args' in argv else argv)
    rclpy.init()
    node = RtkNode(args.port, args.baud, args.ntrip_env, tablet_port=args.tablet_port or None,
                   bt_channel=args.bt_channel or None, can_channel=args.can_channel or None)
    watchdog.start("ros", "serial", *(("tablet",) if node.tablet is not None else ()),
                   *(("bluetooth",) if node.bt is not None else ()), *(("can",) if node.can is not None else ()))
    node.create_timer(0.5, lambda: watchdog.tick("ros"))

    def on_signal(_sig=None, _frame=None):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        watchdog.close()
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
