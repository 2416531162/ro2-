#!/usr/bin/env python3
"""RK3588 Wheeltec adapter: latest command, sole serial owner, explicit arming.

/cmd_vel is SI body velocity; /ackermann_cmd is speed + front steering angle.
The firmware profile must be confirmed before enabling nonzero transmission.
This file's protocol and ControlPolicy also run without ROS for regression tests.
"""
import json
import math
import os
import struct
import threading
import time
from dataclasses import dataclass

FRAME_HEADER, FRAME_TAIL = 0x7B, 0x7D
BY_ID_HINT = "usb-WCH.CN_USB_Single_Serial_0002-if00"
DEFAULT_PORT = "/dev/serial/by-id/" + BY_ID_HINT
STOP_FRAME = bytes.fromhex("7b 00 00 00 00 00 00 00 00 7b 7d")


def bcc(data):
    result = 0
    for value in data:
        result ^= value
    return result


def finite(*values):
    if not all(math.isfinite(v) for v in values):
        raise ValueError("non-finite command")


def build_frame(speed, turn, mode=0):
    finite(speed, turn)
    values = [round(speed * 1000), round(turn * 1000)]
    if not 0 <= mode <= 255 or any(abs(v) > 32767 for v in values):
        raise ValueError("command exceeds wire range")
    # Explicitly zero bytes 5/6: Ackermann has no lateral drive channel.
    frame = bytes([FRAME_HEADER, mode, 0]) + struct.pack(">hhh", values[0], 0, values[1])
    return frame + bytes([bcc(frame), FRAME_TAIL])


def decode_frame(frame):
    if len(frame) != 24 or frame[0] != FRAME_HEADER or frame[-1] != FRAME_TAIL or bcc(frame[:22]) != frame[22]:
        raise ValueError("invalid telemetry frame")
    v = struct.unpack(">9hH", frame[2:22])
    return {"velocity": [x / 1000 for x in v[:3]],
            "acceleration": [x / 1671.84 for x in v[3:6]],
            "gyro": [x * 0.00026644 for x in v[6:9]],
            "voltage": v[9] / 1000, "stop_flag_raw": frame[1]}


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()
        self.good = self.bad = self.discarded = 0

    def feed(self, data):
        self.buffer.extend(data)
        output = []
        while len(self.buffer) >= 24:
            if self.buffer[0] != FRAME_HEADER:
                del self.buffer[0]
                self.discarded += 1
                continue
            candidate = bytes(self.buffer[:24])
            try:
                decoded = decode_frame(candidate)
            except ValueError:
                del self.buffer[0]
                self.bad += 1
                continue
            del self.buffer[:24]
            self.good += 1
            output.append(decoded)
        return output


@dataclass(frozen=True)
class Config:
    protocol: str = "unconfigured"  # twist or steering_angle; firmware-specific
    protocol_confirmed: bool = False
    receive_only: bool = True
    mode_byte: int = 0
    steering_scale: float = 1.0
    wheelbase_m: float = 0.0        # no guessed wheelbase
    max_speed_m_s: float = 0.15
    max_steering_rad: float = 0.35
    max_yaw_rate_rad_s: float = 0.35
    acceleration_m_s2: float = 0.20
    steering_rate_rad_s: float = 0.50
    cmd_timeout_s: float = 0.30
    feedback_timeout_s: float = 0.30
    startup_stop_s: float = 3.0
    tx_hz: float = 50.0

    def __post_init__(self):
        if self.protocol not in ("unconfigured", "twist", "steering_angle"):
            raise ValueError("unknown protocol")
        if not 0 <= self.mode_byte <= 255:
            raise ValueError("invalid mode byte")
        nums = [v for k, v in vars(self).items() if isinstance(v, (float, int)) and not isinstance(v, bool)]
        finite(*nums)
        if self.wheelbase_m < 0 or not 0 < abs(self.steering_scale) <= 10:
            raise ValueError("invalid geometry or steering scale")
        if not 0 < self.max_speed_m_s <= 2.5 or not 0 < self.max_steering_rad < math.pi / 3:
            raise ValueError("invalid speed or steering limit")
        if not 0 < self.max_yaw_rate_rad_s <= 3.0:
            raise ValueError("invalid yaw limit")
        if not 0 < self.acceleration_m_s2 <= 6.0 or not 0 < self.steering_rate_rad_s <= 6.0:
            raise ValueError("invalid slew limit")
        if not 0.05 <= self.cmd_timeout_s <= 1 or not 0.05 <= self.feedback_timeout_s <= 1:
            raise ValueError("invalid deadline")
        if not 3 <= self.startup_stop_s <= 10 or not 10 <= self.tx_hz <= 50:
            raise ValueError("invalid startup or transmit rate")


class ControlPolicy:
    """Call under the serial worker's shared lock. No command FIFO is used."""
    def __init__(self, config, now):
        self.config = config
        self.connected = self.armed = False
        self.reason = "disconnected"
        self.latest = None
        self.last_rx = None
        self.stationary_frames = 0
        self.connect_at = now
        self.last_tick = now
        self.output = (0.0, 0.0)
        self.last_received = None

    def stop(self, reason):
        self.armed = False
        self.latest = None
        self.output = (0.0, 0.0)
        self.reason = reason

    def link(self, connected, now):
        self.connected = connected
        self.connect_at = self.last_tick = now
        self.last_rx = None
        self.stationary_frames = 0
        self.stop("startup" if connected else "disconnected")

    def feedback(self, telemetry, now):
        self.last_rx = now
        self.last_received = telemetry
        v = telemetry["velocity"]
        self.stationary_frames = self.stationary_frames + 1 if max(abs(x) for x in v) < 0.015 else 0

    def ready(self, now):
        c = self.config
        if c.receive_only:
            return "receive_only"
        if not c.protocol_confirmed or c.protocol == "unconfigured":
            return "firmware_profile_unconfirmed"
        if not self.connected:
            return "disconnected"
        if now - self.connect_at < c.startup_stop_s:
            return "startup_stop"
        if self.last_rx is None or now - self.last_rx > c.feedback_timeout_s:
            return "feedback_stale"
        if self.stationary_frames < 5:
            return "waiting_for_stationary_feedback"
        return "ready"

    def arm(self, now):
        reason = self.ready(now)
        if reason != "ready":
            self.stop(reason)
            return False, reason
        self.stop("armed_waiting_command")
        self.armed = True
        self.last_tick = now
        return True, self.reason

    def command(self, kind, speed, turn, now, lateral=0.0):
        try:
            finite(speed, turn, lateral)
        except ValueError:
            self.stop("non_finite_command")
            raise
        c = self.config
        if abs(lateral) > 1e-9:
            self.stop("lateral_command_rejected")
            raise ValueError("Ackermann does not accept lateral velocity")
        if not self.armed:
            raise ValueError("not armed")
        speed = max(-c.max_speed_m_s, min(c.max_speed_m_s, speed))
        if kind == "ackermann":
            steering = max(-c.max_steering_rad, min(c.max_steering_rad, turn))
            if c.protocol == "steering_angle":
                value = steering
            else:
                if abs(speed) < 1e-6 and abs(steering) > 1e-6:
                    self.stop("stationary_steering_needs_angle_protocol")
                    raise ValueError(self.reason)
                if c.wheelbase_m <= 0:
                    self.stop("wheelbase_unconfigured")
                    raise ValueError(self.reason)
                value = speed * math.tan(steering) / c.wheelbase_m
        elif kind == "twist":
            yaw = max(-c.max_yaw_rate_rad_s, min(c.max_yaw_rate_rad_s, turn))
            if abs(speed) < 1e-6 and abs(yaw) > 1e-6:
                self.stop("use_ackermann_cmd_for_stationary_steering")
                raise ValueError(self.reason)
            if c.protocol == "steering_angle":
                if c.wheelbase_m <= 0:
                    self.stop("wheelbase_unconfigured")
                    raise ValueError(self.reason)
                value = math.atan(c.wheelbase_m * yaw / speed) if abs(speed) > 1e-6 else 0.0
                value = max(-c.max_steering_rad, min(c.max_steering_rad, value))
            else:
                value = yaw
        else:
            self.stop("unknown_command")
            raise ValueError(self.reason)
        if c.protocol == "twist":
            value = max(-c.max_yaw_rate_rad_s, min(c.max_yaw_rate_rad_s, value))
        self.latest = (now, speed, value)
        self.reason = "command_active"

    def tick(self, now):
        c = self.config
        dt = min(max(now - self.last_tick, 0.0), 1 / c.tx_hz)
        self.last_tick = now
        if self.armed and (self.last_rx is None or now - self.last_rx > c.feedback_timeout_s):
            self.stop("feedback_stale")
        if self.armed and self.latest and now - self.latest[0] > c.cmd_timeout_s:
            self.latest = None
            self.output = (0.0, 0.0)
            self.reason = "armed_waiting_command"
        if not self.armed and self.reason not in ("operator_stop", "operator_disarmed") and self.ready(now) == "ready":
            self.armed = True
            self.reason = "armed_waiting_command"
        if not self.connected or not self.armed or not self.latest:
            self.output = (0.0, 0.0)
            return STOP_FRAME
        _, speed, turn = self.latest
        old_speed, old_turn = self.output
        # Brake immediately on zero or reversal; never continue accelerating an old direction.
        if speed == 0 or old_speed * speed < 0:
            out_speed = 0.0
        else:
            out_speed = old_speed + max(-c.acceleration_m_s2 * dt, min(c.acceleration_m_s2 * dt, speed - old_speed))
        out_turn = old_turn + max(-c.steering_rate_rad_s * dt, min(c.steering_rate_rad_s * dt, turn - old_turn))
        self.output = (out_speed, out_turn)
        wire_turn = out_turn * c.steering_scale if c.protocol == "steering_angle" else out_turn
        return build_frame(out_speed, wire_turn, c.mode_byte)


try:
    import serial
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy
    from rclpy.duration import Duration
    from rcl_interfaces.msg import ParameterDescriptor
    from geometry_msgs.msg import Twist, TransformStamped
    from ackermann_msgs.msg import AckermannDriveStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Float32, String
    from std_srvs.srv import SetBool, Trigger
    from tf2_ros import TransformBroadcaster
    ROS_AVAILABLE = True
except ImportError:
    Node = object
    ROS_AVAILABLE = False


class WheeltecDriver(Node):
    def __init__(self):
        if not ROS_AVAILABLE:
            raise RuntimeError("ROS 2 Jazzy, pyserial and ackermann_msgs are required")
        super().__init__("wheeltec_driver")
        defaults = vars(Config()).copy()
        defaults.update(port=DEFAULT_PORT, baud=115200, frame_id="odom", base_frame_id="base_footprint", publish_tf=False, allow_test_port=False)
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        gp = lambda k: self.get_parameter(k).value
        self.config = Config(**{k: gp(k) for k in vars(Config())})
        self.port = gp("port")
        if self.port == "auto":
            self.port = DEFAULT_PORT
        if self.port != DEFAULT_PORT and not (gp("allow_test_port") and self.port.startswith("/dev/pts/")):
            raise ValueError("serial must bind exactly to Wheeltec serial 0002")
        if gp("baud") != 115200:
            raise ValueError("verified Wheeltec baud is 115200")
        self.frame_id, self.base_frame_id = gp("frame_id"), gp("base_frame_id")
        self.publish_tf = gp("publish_tf")
        self.lock = threading.RLock()
        self.policy = ControlPolicy(self.config, time.monotonic())
        self.parser = FrameParser()
        self.ser = None
        self.running = True
        self.tx_packets = self.tx_bytes = self.rx_bytes = self.io_errors = 0
        self.last_tx_hex = ""
        self.last_error = ""
        self.started = time.monotonic()
        self.last_odom = None
        self.position = [0.0, 0.0, 0.0]
        self.rx_times = []
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_imu = self.create_publisher(Imu, "/imu", 10)
        self.pub_voltage = self.create_publisher(Float32, "/voltage", 10)
        self.pub_status = self.create_publisher(String, "/wheeltec/status", 10)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE, lifespan=Duration(seconds=self.config.cmd_timeout_s))
        self.create_subscription(Twist, "/cmd_vel", self.on_twist, qos)
        self.create_subscription(AckermannDriveStamped, "/ackermann_cmd", self.on_ackermann, qos)
        self.create_service(SetBool, "/wheeltec/arm", self.on_arm)
        self.create_service(Trigger, "/wheeltec/stop", self.on_stop)
        self.create_timer(0.2, self.publish_status)
        self.worker = threading.Thread(target=self.io_loop, daemon=True)
        self.worker.start()

    def on_arm(self, request, response):
        with self.lock:
            if request.data:
                response.success, response.message = self.policy.arm(time.monotonic())
            else:
                self.policy.stop("operator_disarmed")
                response.success, response.message = True, "disarmed; zero frames scheduled"
        return response

    def on_stop(self, request, response):
        with self.lock:
            self.policy.stop("operator_stop")
            # Serial writes have no software backlog; state change takes effect at next tick.
        response.success, response.message = True, "stop latched; check telemetry and physical stop"
        return response

    def submit(self, kind, speed, turn, lateral=0.0):
        with self.lock:
            try:
                self.policy.command(kind, speed, turn, time.monotonic(), lateral)
            except ValueError as exc:
                self.policy.stop(str(exc))
                self.last_error = str(exc)

    def on_twist(self, message):
        self.submit("twist", message.linear.x, message.angular.z, message.linear.y)

    def on_ackermann(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        if stamp <= 0 or not -0.1 <= age <= self.config.cmd_timeout_s:
            with self.lock:
                self.policy.stop("ackermann_timestamp_stale")
            return
        self.submit("ackermann", message.drive.speed, message.drive.steering_angle)

    def write_frame(self, frame):
        if self.ser.out_waiting:
            self.ser.reset_output_buffer()
            self.policy.stop("serial_output_backlog")
            frame = STOP_FRAME
        n = self.ser.write(frame)
        if n != len(frame):
            raise IOError("partial serial write")
        self.tx_packets += 1
        self.tx_bytes += n
        self.last_tx_hex = frame.hex(" ")

    def io_loop(self):
        next_tx = time.monotonic()
        while self.running:
            try:
                if self.ser is None:
                    self.ser = serial.Serial(self.port, 115200, timeout=0, write_timeout=0.02, exclusive=True)
                    # Drop only data from a prior session. All serial access belongs to this worker.
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                    with self.lock:
                        self.parser.buffer.clear()
                        self.last_odom = None
                        self.rx_times.clear()
                        self.policy.link(True, time.monotonic())
                    next_tx = time.monotonic()
                data = self.ser.read(min(self.ser.in_waiting, 4096))
                now = time.monotonic()
                with self.lock:
                    self.rx_bytes += len(data)
                    for telemetry in self.parser.feed(data):
                        self.policy.feedback(telemetry, now)
                        self.rx_times.append(now)
                        self.rx_times = self.rx_times[-100:]
                        self.publish_telemetry(telemetry, now)
                    if now >= next_tx:
                        frame = self.policy.tick(now)
                        if not self.config.receive_only:
                            self.write_frame(frame)
                        # Never replay missed ticks in a burst.
                        next_tx = now + 1 / self.config.tx_hz
                time.sleep(0.002)
            except Exception as exc:
                with self.lock:
                    self.io_errors += 1
                    self.last_error = str(exc)
                    self.policy.link(False, time.monotonic())
                try:
                    if self.ser:
                        self.ser.close()
                except Exception:
                    pass
                self.ser = None
                time.sleep(0.25)
        # Shutdown braking is best effort; controller-side timeout is still required.
        if self.ser and not self.config.receive_only:
            with self.lock:
                self.policy.stop("shutdown")
            deadline = time.monotonic() + self.config.startup_stop_s
            while time.monotonic() < deadline:
                try:
                    self.write_frame(STOP_FRAME)
                except Exception:
                    break
                time.sleep(1 / self.config.tx_hz)
        if self.ser:
            self.ser.close()

    def publish_telemetry(self, t, now):
        vx, vy, wz = t["velocity"]
        dt = now - self.last_odom if self.last_odom is not None else 0.0
        self.last_odom = now
        x, y, yaw = self.position
        if 0 < dt < self.config.feedback_timeout_s:
            x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
            y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
            yaw = math.atan2(math.sin(yaw + wz * dt), math.cos(yaw + wz * dt))
        self.position = [x, y, yaw]
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, self.frame_id, self.base_frame_id
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = vx, vy, wz
        for i in (0, 7, 35):
            odom.pose.covariance[i] = 0.05
            odom.twist.covariance[i] = 0.05
        for i in (14, 21, 28):
            odom.pose.covariance[i] = odom.twist.covariance[i] = 1e6
        self.pub_odom.publish(odom)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, "imu_link"
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = t["acceleration"]
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = t["gyro"]
        imu.orientation_covariance[0] = -1.0
        for i in (0, 4, 8):
            imu.linear_acceleration_covariance[i] = imu.angular_velocity_covariance[i] = 0.05
        self.pub_imu.publish(imu)
        self.pub_voltage.publish(Float32(data=t["voltage"]))
        if self.tf:
            tf = TransformStamped()
            tf.header, tf.child_frame_id = odom.header, self.base_frame_id
            tf.transform.translation.x, tf.transform.translation.y = x, y
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(tf)

    def publish_status(self):
        with self.lock:
            now = time.monotonic()
            p = self.policy
            hz = (len(self.rx_times) - 1) / (self.rx_times[-1] - self.rx_times[0]) if len(self.rx_times) > 1 and self.rx_times[-1] > self.rx_times[0] and p.last_rx and now - p.last_rx < self.config.feedback_timeout_s else 0.0
            age = now - p.last_rx if p.last_rx is not None else None
            data = {"connected": p.connected, "armed": p.armed, "reason": p.reason,
                    "ready": p.ready(now), "port": self.port, "device": os.path.realpath(self.port),
                    "baud": 115200, "config": vars(self.config), "hz": round(hz, 2),
                    "frames_ok": self.parser.good, "frames_bad": self.parser.bad,
                    "bytes_in": self.rx_bytes, "age_ms": round(age * 1000, 1) if age is not None else None,
                    "telemetry": p.last_received, "output_speed_turn": p.output,
                    "tx_packets": self.tx_packets, "tx_bytes": self.tx_bytes, "last_tx_hex": self.last_tx_hex,
                    "io_errors": self.io_errors, "last_error": self.last_error,
                    "steering_feedback_available": False, "stop_confirmed": bool(age is not None and age < self.config.feedback_timeout_s and p.stationary_frames >= 5)}
        self.pub_status.publish(String(data=json.dumps(data, ensure_ascii=False)))

    def shutdown(self):
        with self.lock:
            self.policy.stop("shutdown")
        self.running = False
        self.worker.join(timeout=self.config.startup_stop_s + 2)


def main():
    if not ROS_AVAILABLE:
        raise SystemExit("Source ROS Jazzy and install ros-jazzy-ackermann-msgs / python3-serial")
    rclpy.init()
    node = None
    try:
        node = WheeltecDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
