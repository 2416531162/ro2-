#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 轮趣科技（WHEELTEC）底盘 ROS2 驱动节点 —— RK3588 版本
================================================================================
 用途：在 ATK-DLRK3588 上替代原「ROS2 控制板」，直接与轮趣电机驱动板通信。

 协议：轮趣官方 24 字节遥测帧 / 11 字节控制帧，BCC 异或校验
       详见同目录 PROTOCOL.md

 订阅：/cmd_vel          (geometry_msgs/Twist)
 发布：/odom             (nav_msgs/Odometry)
       /imu              (sensor_msgs/Imu)
       /voltage          (std_msgs/Float32)
       /wheeltec/status  (std_msgs/String, JSON)

 运行：
   source /opt/ros/jazzy/setup.bash
   python3 wheeltec_driver.py --ros-args -p port:=auto -p baud:=115200

 依赖：pip install pyserial   (ROS2 Jazzy 自带 rclpy)
================================================================================
"""

import json
import math
import os
import struct
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("缺少 pyserial： pip install pyserial", file=sys.stderr)
    raise

# ------------------------------------------------------------------ 协议常量
FRAME_HEADER      = 0x7B
FRAME_TAIL        = 0x7D
RECEIVE_DATA_SIZE = 24          # 下位机 → 上位机
SEND_DATA_SIZE    = 11          # 上位机 → 下位机

GYROSCOPE_RATIO   = 0.00026644  # 陀螺仪 raw → rad/s   (量程 ±500°)
ACCEL_RATIO       = 1671.84     # 加速度计 raw → m/s²  (量程 ±2g)

# 轮趣 USB 转串口芯片
WHEELTEC_VID_PID = (0x1A86, 0x55D4)
BY_ID_HINT       = "usb-WCH.CN_USB_Single_Serial_0002-if00"


# ================================================================== 协议工具
def bcc(data: bytes) -> int:
    """轮趣 BCC 校验：逐字节异或。"""
    c = 0
    for b in data:
        c ^= b
    return c


def s16(hi: int, lo: int) -> int:
    """两个字节按【大端】拼成有符号 16 位整数。"""
    v = (hi << 8) | lo
    return v - 0x10000 if v >= 0x8000 else v


def u16(hi: int, lo: int) -> int:
    return (hi << 8) | lo


def odom_trans(raw: int) -> float:
    """
    速度换算 mm/s → m/s。

    官方 C 代码是 (raw/1000) + (raw%1000)*0.001，这里必须注意：
      C 的整数除法向零取整，Python 的 // 向下取整 —— 负数会算错。
      数学上等价于有符号值 / 1000.0，直接用浮点除最稳妥。
    """
    return raw / 1000.0


# ================================================================== 串口发现
def find_port(explicit: str = "auto") -> str:
    """
    找轮趣底盘串口。

    注意：本项目里 /dev/ttyACM0 是 N10P 激光雷达（460800），
          轮趣底盘是 /dev/ttyACM1（115200）。两者芯片型号相同，
          只有 USB 序列号不同（雷达 0001 / 底盘 0002），
          所以优先用 by-id 名字绑定，避免重启后编号互换。
    """
    if explicit and explicit != "auto":
        return explicit

    by_id = f"/dev/serial/by-id/{BY_ID_HINT}"
    if os.path.exists(by_id):
        return by_id

    # 退路：按序列号扫描
    for p in list_ports.comports():
        if (p.vid, p.pid) == WHEELTEC_VID_PID and (p.serial_number or "").endswith("0002"):
            return p.device

    # 再退路：凡是 WCH 串口，排除已知的雷达
    for p in list_ports.comports():
        if (p.vid, p.pid) == WHEELTEC_VID_PID and not (p.serial_number or "").endswith("0001"):
            return p.device

    raise RuntimeError("找不到轮趣底盘串口，请用 -p port:=/dev/ttyACMx 显式指定")


# ================================================================== ROS2 节点
class WheeltecDriver(Node):

    def __init__(self):
        super().__init__("wheeltec_driver")

        self.declare_parameter("port", "auto")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_footprint")
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("publish_imu", True)
        self.declare_parameter("publish_voltage", True)
        self.declare_parameter("cmd_vel_timeout", 0.5)   # 超时未收到 cmd_vel 就发停车

        gp = self.get_parameter
        self.frame_id      = gp("frame_id").value
        self.base_frame_id = gp("base_frame_id").value
        self.cmd_vel_timeout = gp("cmd_vel_timeout").value

        # ---------------- 发布者
        self.pub_odom    = self.create_publisher(Odometry, "/odom", 10)
        self.pub_imu     = self.create_publisher(Imu, "/imu", 10)
        self.pub_voltage = self.create_publisher(Float32, "/voltage", 10)
        self.pub_status  = self.create_publisher(String, "/wheeltec/status", 10)

        # ---------------- 订阅者：速度指令
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)

        # ---------------- 状态
        self.ser = None
        self.ser_lock = threading.Lock()
        self.running = True

        self.last_cmd = (0.0, 0.0, 0.0)
        self.last_cmd_time = 0.0
        self.estop_sent = False

        # 统计
        self.frames_ok = 0
        self.frames_bad = 0
        self.bytes_in = 0
        self.lost_bytes = 0
        self.hz_window = []
        self.last_frame_time = 0.0

        # 里程计状态
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_yaw = 0.0
        self.last_odom_time = None

        self.last_voltage = 0.0
        self.last_vel = (0.0, 0.0, 0.0)
        self.last_imu = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.connected = False

        self.connect_serial()

        # ---------------- 线程与定时器
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()
        self.create_timer(0.1, self.watchdog)
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            f"轮趣底盘驱动已启动 | port={self.ser.port} baud={self.ser.baudrate}"
        )

    # -------------------------------------------------------------- 串口连接
    def connect_serial(self):
        port = find_port(self.get_parameter("port").value)
        baud = self.get_parameter("baud").value
        self.ser = serial.Serial(
            port=port, baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            exclusive=True,
        )
        self.ser.reset_input_buffer()
        self.connected = True

    def reconnect(self):
        self.connected = False
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        time.sleep(0.5)
        try:
            self.connect_serial()
            self.get_logger().warn(f"串口重连成功: {self.ser.port}")
        except Exception as e:
            self.get_logger().error(f"串口重连失败: {e}")
            time.sleep(1.0)

    # -------------------------------------------------------------- 读循环
    def read_loop(self):
        buf = bytearray()
        while self.running:
            if not self.connected:
                self.reconnect()
                continue
            try:
                chunk = self.ser.read(256)
            except Exception as e:
                self.get_logger().error(f"串口读取异常: {e}，准备重连")
                self.reconnect()
                buf.clear()
                continue

            if not chunk:
                continue
            self.bytes_in += len(chunk)
            buf.extend(chunk)

            # 逐字节找帧头、凑满 24 字节
            while True:
                if not buf:
                    break
                if buf[0] != FRAME_HEADER:
                    idx = buf.find(bytes([FRAME_HEADER]))
                    if idx < 0:
                        self.lost_bytes += len(buf)
                        buf.clear()
                        break
                    self.lost_bytes += idx
                    del buf[:idx]
                if len(buf) < RECEIVE_DATA_SIZE:
                    break

                frame = bytes(buf[:RECEIVE_DATA_SIZE])
                if frame[23] != FRAME_TAIL or bcc(frame[0:22]) != frame[22]:
                    # 校验不过：丢掉第一个字节重新同步
                    self.frames_bad += 1
                    del buf[:1]
                    continue

                del buf[:RECEIVE_DATA_SIZE]
                self.handle_frame(frame)

    # -------------------------------------------------------------- 帧解析
    def handle_frame(self, f: bytes):
        self.frames_ok += 1
        now = time.time()
        self.last_frame_time = now
        self.hz_window.append(now)
        if len(self.hz_window) > 50:
            self.hz_window.pop(0)

        # ---- 速度（mm/s → m/s）
        vx = odom_trans(s16(f[2],  f[3]))
        vy = odom_trans(s16(f[4],  f[5]))
        vz = odom_trans(s16(f[6],  f[7]))
        self.last_vel = (vx, vy, vz)

        # ---- IMU
        ax = s16(f[8],  f[9])  / ACCEL_RATIO
        ay = s16(f[10], f[11]) / ACCEL_RATIO
        az = s16(f[12], f[13]) / ACCEL_RATIO
        gx = s16(f[14], f[15]) * GYROSCOPE_RATIO
        gy = s16(f[16], f[17]) * GYROSCOPE_RATIO
        gz = s16(f[18], f[19]) * GYROSCOPE_RATIO
        self.last_imu = (ax, ay, az, gx, gy, gz)

        # ---- 电压 mV → V
        self.last_voltage = u16(f[20], f[21]) / 1000.0

        # ---- 里程计积分（官方逻辑）
        if self.last_odom_time is None:
            dt = 0.0
        else:
            dt = now - self.last_odom_time
        self.last_odom_time = now

        if 0.0 < dt < 0.5:
            self.pos_x   += (vx * math.cos(self.pos_yaw) - vy * math.sin(self.pos_yaw)) * dt
            self.pos_y   += (vx * math.sin(self.pos_yaw) + vy * math.cos(self.pos_yaw)) * dt
            self.pos_yaw += vz * dt
            self.pos_yaw = math.atan2(math.sin(self.pos_yaw), math.cos(self.pos_yaw))

        stamp = self.get_clock().now().to_msg()

        # ---- /odom
        if self.get_parameter("publish_odom").value:
            od = Odometry()
            od.header.stamp = stamp
            od.header.frame_id = self.frame_id
            od.child_frame_id = self.base_frame_id
            od.pose.pose.position.x = self.pos_x
            od.pose.pose.position.y = self.pos_y
            od.pose.pose.position.z = 0.0
            q = Quaternion()
            q.z = math.sin(self.pos_yaw / 2.0)
            q.w = math.cos(self.pos_yaw / 2.0)
            od.pose.pose.orientation = q
            od.twist.twist.linear.x = vx
            od.twist.twist.linear.y = vy
            od.twist.twist.angular.z = vz
            od.pose.covariance[0]  = 1e-3
            od.pose.covariance[7]  = 1e-3
            od.pose.covariance[35] = 1e-3
            self.pub_odom.publish(od)

        # ---- /imu
        if self.get_parameter("publish_imu").value:
            im = Imu()
            im.header.stamp = stamp
            im.header.frame_id = "imu_link"
            im.linear_acceleration.x = ax
            im.linear_acceleration.y = ay
            im.linear_acceleration.z = az
            im.angular_velocity.x = gx
            im.angular_velocity.y = gy
            im.angular_velocity.z = gz
            for i in (0, 4, 8):
                im.linear_acceleration_covariance[i] = 1e-3
                im.angular_velocity_covariance[i] = 1e-3
            im.orientation_covariance[0] = -1.0   # 无姿态估计
            self.pub_imu.publish(im)

        # ---- /voltage
        if self.get_parameter("publish_voltage").value:
            self.pub_voltage.publish(Float32(data=self.last_voltage))

    # -------------------------------------------------------------- 下发指令
    def on_cmd_vel(self, msg: Twist):
        self.last_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.last_cmd_time = time.time()
        self.estop_sent = False

        tx = bytearray(SEND_DATA_SIZE)
        tx[0] = FRAME_HEADER
        tx[1] = 0
        tx[2] = 0

        def put16(off, val):
            v = int(round(val * 1000.0))          # m/s → mm/s
            v = max(-32768, min(32767, v))        # 防溢出
            v &= 0xFFFF
            tx[off]     = (v >> 8) & 0xFF         # 高字节在前
            tx[off + 1] = v & 0xFF

        put16(3, msg.linear.x)      # X 目标速度
        put16(5, msg.linear.y)      # Y 目标速度
        put16(7, msg.angular.z)     # Z 目标角速度

        tx[9]  = bcc(tx[0:9])
        tx[10] = FRAME_TAIL

        self.write(bytes(tx))

    def write(self, data: bytes) -> bool:
        if not self.connected:
            return False
        try:
            with self.ser_lock:
                self.ser.write(data)
            return True
        except Exception as e:
            self.get_logger().error(f"串口写入失败: {e}")
            self.connected = False
            return False

    # -------------------------------------------------------------- 看门狗
    def watchdog(self):
        """cmd_vel 超时 → 主动刹车，避免底盘失控。"""
        if self.last_cmd_time == 0.0 or self.estop_sent:
            return
        if time.time() - self.last_cmd_time > self.cmd_vel_timeout:
            if any(abs(v) > 1e-6 for v in self.last_cmd):
                self.get_logger().warn("cmd_vel 超时，下发停车指令")
                self.on_cmd_vel(Twist())
                self.estop_sent = True

        # 数据流断了也要告警
        if self.connected and self.last_frame_time and \
           time.time() - self.last_frame_time > 1.0:
            self.get_logger().warn("超过 1 秒未收到遥测帧")

    # -------------------------------------------------------------- 状态发布
    def publish_status(self):
        now = time.time()
        hz = 0.0
        if len(self.hz_window) >= 2:
            hz = (len(self.hz_window) - 1) / (self.hz_window[-1] - self.hz_window[0])

        st = {
            "connected": self.connected,
            "port": self.ser.port if self.ser else None,
            "baud": self.ser.baudrate if self.ser else None,
            "hz": round(hz, 2),
            "frames_ok": self.frames_ok,
            "frames_bad": self.frames_bad,
            "bytes_in": self.bytes_in,
            "lost_bytes": self.lost_bytes,
            "age_ms": int((now - self.last_frame_time) * 1000) if self.last_frame_time else None,
            "voltage_v": round(self.last_voltage, 3),
            "vel": [round(v, 4) for v in self.last_vel],
            "imu_acc": [round(v, 3) for v in self.last_imu[:3]],
            "imu_gyro": [round(v, 5) for v in self.last_imu[3:]],
            "odom": [round(self.pos_x, 3), round(self.pos_y, 3), round(self.pos_yaw, 3)],
        }
        self.pub_status.publish(String(data=json.dumps(st, ensure_ascii=False)))

    # -------------------------------------------------------------- 退出
    def shutdown(self):
        self.running = False
        try:
            if self.connected:
                tx = bytearray(SEND_DATA_SIZE)
                tx[0] = FRAME_HEADER
                tx[9] = bcc(tx[0:9])
                tx[10] = FRAME_TAIL
                self.write(bytes(tx))       # 停车
        except Exception:
            pass
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass


def main():
    rclpy.init()
    node = WheeltecDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
