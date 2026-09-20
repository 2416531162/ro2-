#!/usr/bin/env python3
"""RK3588 Wheeltec adapter: latest command, sole serial owner, explicit arming.

Normal motion uses leased /follow|manual|navigation/command requests.
Legacy /cmd_vel and /ackermann_cmd are available only in exclusive commissioning mode.
The firmware profile must be confirmed before enabling nonzero transmission.
This file's protocol and ControlPolicy also run without ROS for regression tests.
"""
import json
import math
import os
import struct
import threading
import time
import uuid
from dataclasses import dataclass
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from runtime_config import PROFILE, profile_hash
from robot_core.kinematics import ChassisGeometry, yaw_from_steer, steer_from_yaw, max_yaw_at_speed
from motion_authority import MotionAuthority
from scan_guard import GuardConfig, ScanGuard  # noqa: E402

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
    track_m: float = PROFILE["geometry"]["track_m"]
    wheelbase_m: float = 0.0        # no guessed wheelbase
    max_speed_m_s: float = 0.15
    max_steering_rad: float = 0.35
    max_yaw_rate_rad_s: float = 0.35
    acceleration_m_s2: float = 0.20
    steering_rate_rad_s: float = 0.50
    cmd_timeout_s: float = 0.30
    feedback_timeout_s: float = 0.60
    startup_stop_s: float = 3.0
    tx_hz: float = 50.0
    # Transient-fault handling. A recoverable fault holds output at zero while
    # staying armed; only after the grace window does it become a hard disarm
    # that requires stationary telemetry to clear. Without this, a single USB-CDC
    # write backlog or one stuttered telemetry frame disarmed the chassis
    # mid-drive, and re-arming demanded the car be physically stopped first --
    # which is what made web driving feel "extremely slow" and jerky.
    feedback_grace_s: float = 1.00
    backlog_tolerance: int = 8
    stall_timeout_s: float = 0.45
    stall_min_command_m_s: float = 0.06
    stall_min_progress_m: float = 0.01

    def __post_init__(self):
        if self.protocol not in ("unconfigured", "twist", "steering_angle"):
            raise ValueError("unknown protocol")
        if not 0 <= self.mode_byte <= 255:
            raise ValueError("invalid mode byte")
        nums = [v for k, v in vars(self).items() if isinstance(v, (float, int)) and not isinstance(v, bool)]
        finite(*nums)
        if self.wheelbase_m < 0 or self.track_m < 0 or not 0 < abs(self.steering_scale) <= 10:
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
        if not 0 <= self.feedback_grace_s <= 3:
            raise ValueError("invalid feedback grace")
        if not 0 <= self.backlog_tolerance <= 100:
            raise ValueError("invalid backlog tolerance")
        if not 0.30 <= self.stall_timeout_s <= 1.0:
            raise ValueError("invalid stall timeout")
        if not 0.04 <= self.stall_min_command_m_s <= 0.15:
            raise ValueError("invalid stall command threshold")
        if not 0.005 <= self.stall_min_progress_m <= 0.05:
            raise ValueError("invalid stall progress threshold")


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
        # Recoverable-fault bookkeeping: see Config.feedback_grace_s.
        self.hold_reason = None
        self.hold_since = None
        # 独立防撞层(scan_guard.ScanGuard.limit 的包装):speed_filter(speed, turn, now)
        # -> (限制后速度, 原因)。为 None 时行为与原来完全一致。
        self.speed_filter = None
        self.guard_reason = None
        self.guard_eval_ms = 0.0
        self.guard_eval_peak_ms = 0.0
        self.guard_eval_overruns = 0
        self.stall_since = None
        self.stall_feedback_at = None
        self.stall_feedback_frames = 0
        self.stall_progress_m = 0.0
        self.stall_direction = 0
        self.stall_latched = False
        self.stall_report = None

    def clear_stall_observation(self):
        self.stall_since = None
        self.stall_feedback_at = None
        self.stall_feedback_frames = 0
        self.stall_progress_m = 0.0
        self.stall_direction = 0

    def stop(self, reason):
        """Hard disarm. Recovery requires stationary telemetry, so reserve this
        for genuine safety faults -- not for transient I/O hiccups."""
        self.armed = False
        self.latest = None
        self.output = (0.0, 0.0)
        self.reason = reason
        self.hold_reason = None
        self.hold_since = None
        self.clear_stall_observation()

    def reject(self, reason):
        """Refuse one command without disarming. The chassis stays armed and
        simply receives no new setpoint; tick() ramps it down via cmd_timeout_s."""
        self.latest = None
        self.output = (0.0, 0.0)
        self.reason = reason
        raise ValueError(reason)

    def hold(self, reason, now):
        """Recoverable fault: command zero but stay armed. If the fault persists
        past feedback_grace_s it escalates to a hard stop in tick()."""
        if self.hold_since is None:
            self.hold_since = now
        self.hold_reason = reason
        self.output = (0.0, 0.0)
        self.clear_stall_observation()

    def release_hold(self):
        self.hold_reason = None
        self.hold_since = None

    @property
    def holding(self):
        return self.hold_reason is not None

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

    def ready(self, now, *, ignore_stall=False):
        c = self.config
        if self.stall_latched and not ignore_stall:
            return "motion_stall_latched"
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

    def arm(self, now, *, clear_stall=False):
        reason = self.ready(now, ignore_stall=clear_stall)
        if reason != "ready":
            self.stop(reason)
            return False, reason
        if clear_stall:
            self.stall_latched = False
            self.stall_report = None
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
            self.reject("lateral_command_rejected")
        if not self.armed:
            raise ValueError("not armed")
        speed = max(-c.max_speed_m_s, min(c.max_speed_m_s, speed))
        geometry = ChassisGeometry(c.wheelbase_m, c.track_m, c.max_steering_rad) if c.wheelbase_m > 0 else None
        if kind == "ackermann":
            steering = max(-c.max_steering_rad, min(c.max_steering_rad, turn))
            if c.protocol == "steering_angle":
                value = steering
            else:
                if abs(speed) < 1e-6 and abs(steering) > 1e-6:
                    self.reject("stationary_steering_needs_angle_protocol")
                if c.wheelbase_m <= 0:
                    self.stop("wheelbase_unconfigured")
                    raise ValueError(self.reason)
                value = yaw_from_steer(speed, steering, geometry)
        elif kind == "twist":
            yaw = max(-c.max_yaw_rate_rad_s, min(c.max_yaw_rate_rad_s, turn))
            if abs(speed) < 1e-6 and abs(yaw) > 1e-6:
                self.reject("use_ackermann_cmd_for_stationary_steering")
            if c.protocol == "steering_angle":
                if c.wheelbase_m <= 0:
                    self.stop("wheelbase_unconfigured")
                    raise ValueError(self.reason)
                value = steer_from_yaw(speed, yaw, geometry)
                value = max(-c.max_steering_rad, min(c.max_steering_rad, value))
            else:
                limit = max_yaw_at_speed(speed, geometry) if geometry else c.max_yaw_rate_rad_s
                value = max(-limit, min(limit, yaw))
        else:
            self.stop("unknown_command")
            raise ValueError(self.reason)
        if c.protocol == "twist":
            value = max(-c.max_yaw_rate_rad_s, min(c.max_yaw_rate_rad_s, value))
        self.latest = (now, speed, value)
        self.reason = "command_active"

    def tick(self, now):
        c = self.config
        # The old cap was `1 / c.tx_hz`, the *nominal* period. Whenever the I/O
        # loop ran even slightly slower than nominal -- routine under Python plus
        # USB-CDC on an RK3588 -- elapsed time was silently under-counted and the
        # acceleration ramp stretched out in wall-clock terms. Cap at a few
        # periods instead, so a genuine stall is still bounded.
        dt = min(max(now - self.last_tick, 0.0), 3 / c.tx_hz)
        self.last_tick = now

        # Telemetry gap: hold at zero first, escalate to disarm only if it lasts.
        if self.armed:
            stale = self.last_rx is None or now - self.last_rx > c.feedback_timeout_s
            if stale:
                self.hold("feedback_stale", now)
                if now - self.hold_since > c.feedback_grace_s:
                    self.stop("feedback_lost")
            elif self.hold_reason == "feedback_stale":
                self.release_hold()

        if self.armed and self.holding and now - self.hold_since > c.feedback_grace_s:
            self.stop(self.hold_reason or "hold_expired")

        if self.armed and self.latest and now - self.latest[0] > c.cmd_timeout_s:
            self.latest = None
            self.output = (0.0, 0.0)
            self.reason = "armed_waiting_command"
            self.clear_stall_observation()
        if (not self.armed and not self.stall_latched
                and self.reason not in ("operator_stop", "operator_disarmed")
                and self.ready(now) == "ready"):
            self.armed = True
            self.reason = "armed_waiting_command"
        if not self.connected or not self.armed or not self.latest or self.holding:
            self.output = (0.0, 0.0)
            self.clear_stall_observation()
            return STOP_FRAME
        _, speed, turn = self.latest
        old_speed, old_turn = self.output
        self.guard_reason = None
        if self.speed_filter is not None:
            original_speed = speed
            guard_started = time.perf_counter()
            try:
                speed, self.guard_reason = self.speed_filter(speed, turn, now)
            finally:
                self.guard_eval_ms = (time.perf_counter() - guard_started) * 1000
                self.guard_eval_peak_ms = max(self.guard_eval_peak_ms, self.guard_eval_ms)
                if self.guard_eval_ms > 1000 / c.tx_hz:
                    self.guard_eval_overruns += 1
            if c.protocol == 'twist' and abs(original_speed) > 1e-9:
                turn *= abs(speed / original_speed)
        # Brake immediately on zero or reversal; never continue accelerating an old direction.
        if speed == 0 or old_speed * speed < 0:
            out_speed = 0.0
        else:
            out_speed = old_speed + max(-c.acceleration_m_s2 * dt, min(c.acceleration_m_s2 * dt, speed - old_speed))
            if self.guard_reason is not None and abs(out_speed) > abs(speed):
                # 防撞限速立即生效,不走减速斜坡
                out_speed = speed
        out_turn = old_turn + max(-c.steering_rate_rad_s * dt, min(c.steering_rate_rad_s * dt, turn - old_turn))
        if c.protocol == 'twist' and c.wheelbase_m > 0:
            limit = max_yaw_at_speed(out_speed, ChassisGeometry(c.wheelbase_m, c.track_m, c.max_steering_rad))
            out_turn = max(-limit, min(limit, out_turn))
        self.output = (out_speed, out_turn)
        # Check the FINAL transmitted speed, after the scan guard and ramp. A
        # blocked/unknown scan commands zero and must not be mistaken for a
        # mechanical stall. Count fresh chassis feedback, not repeated ticks.
        direction = 1 if out_speed > 0 else -1
        if abs(out_speed) < c.stall_min_command_m_s or self.last_rx is None:
            self.clear_stall_observation()
        elif self.stall_direction != direction:
            self.clear_stall_observation()
            self.stall_direction = direction
            self.stall_feedback_at = self.last_rx
            self.stall_since = now
        elif self.stall_feedback_at != self.last_rx:
            elapsed = max(0.0, self.last_rx - self.stall_feedback_at)
            self.stall_feedback_at = self.last_rx
            measured = self.last_received["velocity"][0]
            if math.isfinite(measured):
                self.stall_progress_m += max(0.0, measured * direction) * elapsed
                self.stall_feedback_frames += 1
                if self.stall_progress_m >= c.stall_min_progress_m:
                    self.stall_since = now
                    self.stall_progress_m = 0.0
                    self.stall_feedback_frames = 0
                elif now - self.stall_since >= c.stall_timeout_s and self.stall_feedback_frames >= 5:
                    self.stall_report = {"command_m_s": round(out_speed, 3),
                                         "feedback_m_s": round(measured, 3),
                                         "progress_m": round(self.stall_progress_m, 3),
                                         "duration_s": round(now - self.stall_since, 3),
                                         "feedback_frames": self.stall_feedback_frames}
                    self.stall_latched = True
                    self.stop("motion_stall")
                    return STOP_FRAME
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
    from sensor_msgs.msg import Imu, LaserScan
    from rclpy.qos import ReliabilityPolicy
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
        defaults.update(PROFILE['driver'])
        defaults.update(wheelbase_m=PROFILE['geometry']['wheelbase_m'],
                        max_steering_rad=PROFILE['geometry']['max_steer_rad'])
        defaults.update(port=DEFAULT_PORT, baud=115200, frame_id=PROFILE["frames"]["odom"], base_frame_id=PROFILE["frames"]["base"], publish_tf=False, allow_test_port=False, legacy_commands=False)
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
        # 独立防撞层参数(guard_ 前缀,与 GuardConfig 字段一一对应)
        guard_defaults = vars(GuardConfig())
        for name, value in guard_defaults.items():
            self.declare_parameter("guard_" + name, value, ParameterDescriptor(read_only=True))
        self.guard = ScanGuard(GuardConfig(**{k: gp("guard_" + k) for k in guard_defaults}))
        for field, expected in {
                **PROFILE['driver'],
                'frame_id': PROFILE['frames']['odom'],
                'base_frame_id': PROFILE['frames']['base'],
                'guard_decel_m_s2': PROFILE['safety']['decel_mps2'],
                'guard_latency_s': PROFILE['safety']['guard_latency_s'],
                'guard_scan_timeout_s': PROFILE['safety']['scan_timeout_s'],
                'wheelbase_m': PROFILE['geometry']['wheelbase_m'],
                'track_m': PROFILE['geometry']['track_m'],
                'max_steering_rad': PROFILE['geometry']['max_steer_rad'],
                'guard_front_m': PROFILE['geometry']['front_m'],
                'guard_rear_m': PROFILE['geometry']['rear_m'],
                'guard_half_width_m': PROFILE['geometry']['half_width_m'],
                'guard_wheelbase_m': PROFILE['geometry']['wheelbase_m'],
                'guard_track_m': PROFILE['geometry']['track_m'],
                'guard_max_steer_rad': PROFILE['geometry']['max_steer_rad'],
                'guard_lidar_x_m': PROFILE['sensors']['lidar_x_m'],
                'guard_lidar_y_m': PROFILE['sensors']['lidar_y_m'],
                'guard_lidar_yaw_rad': PROFILE['sensors']['lidar_yaw_rad']}.items():
            if gp(field) != expected:
                raise ValueError(field + ': change shared robot profile, not an isolated ROS override')
        self.publish_tf = gp("publish_tf")
        # Commissioning-only compatibility is exclusive with the leased interface.
        self.legacy_commands = gp("legacy_commands")
        self.authority = MotionAuthority(
            PROFILE['safety']['command_timeout_s'],
            PROFILE['safety']['stationary_s'],
            self.config.feedback_grace_s)
        self.scan_health_at = None
        self.motion_health_faults = ()
        self.lock = threading.RLock()
        self.policy = ControlPolicy(self.config, time.monotonic())
        if self.guard.cfg.enabled:
            self.policy.speed_filter = self.guard_speed_filter
        self.parser = FrameParser()
        self.ser = None
        self.running = True
        self.tx_packets = self.tx_bytes = self.rx_bytes = self.io_errors = 0
        self.last_tx_hex = ""
        self.last_error = ""
        self.backlog_streak = 0
        self.write_timeouts = 0
        self.write_timeout_streak = 0
        self.last_write_timeout_at = None
        self.started = time.monotonic()
        self.last_odom = None
        self.odometry_epoch = uuid.uuid4().hex
        self.position = [0.0, 0.0, 0.0]
        self.rx_times = []
        self.pub_odom = self.create_publisher(Odometry, PROFILE["localization"]["driver_topic"], 10)
        self.pub_imu = self.create_publisher(Imu, "/imu", 10)
        self.pub_voltage = self.create_publisher(Float32, "/voltage", 10)
        self.pub_status = self.create_publisher(String, "/wheeltec/status", 10)
        self.pub_motion = self.create_publisher(String, "/motion/status", 1)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE, lifespan=Duration(seconds=self.config.cmd_timeout_s))
        if self.legacy_commands:
            self.create_subscription(Twist, "/cmd_vel", self.on_twist, qos)
            self.create_subscription(AckermannDriveStamped, "/ackermann_cmd", self.on_ackermann, qos)
        else:
            for source in MotionAuthority.SOURCES:
                self.create_subscription(String, '/' + source + '/command',
                                         lambda msg, src=source: self.on_motion(src, msg), qos)
            self.create_service(SetBool, '/motion/follow',
                                lambda req, res: self.select_motion('follow', req, res))
            self.create_service(SetBool, '/motion/navigation',
                                lambda req, res: self.select_motion('navigation', req, res))
            self.create_service(Trigger, '/motion/reset', self.reset_motion)
            self.create_service(Trigger, '/motion/stop', self.on_stop)
        self.create_subscription(LaserScan, "/scan", self.on_scan,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_service(SetBool, "/wheeltec/arm", self.on_arm)
        self.create_service(Trigger, "/wheeltec/stop", self.on_stop)
        self.create_timer(0.2, self.publish_status)
        self.worker = threading.Thread(target=self.io_loop, daemon=True)
        self.worker.start()

    def guard_speed_filter(self, speed, turn, now):
        # Manual drive is the operator's direct override of software collision
        # monitoring. Keep the policy's driver, stall, speed and timeout checks.
        if not self.legacy_commands and self.authority.mode == 'MANUAL':
            self.guard.last_reason = 'guard_manual_override'
            self.guard.last_gap = None
            self.guard.last_block = None
            return speed, None
        old_speed, old_turn = self.policy.output
        if self.config.protocol == 'steering_angle':
            steer, old_steer = turn, old_turn
        else:
            geometry = ChassisGeometry(self.config.wheelbase_m, self.config.track_m,
                                       self.config.max_steering_rad)
            steer = steer_from_yaw(speed, turn, geometry)
            old_steer = steer_from_yaw(old_speed, old_turn, geometry)
        # The output may still be decelerating or slewing its steering. Check
        # both the existing and requested arc before sending the next frame.
        checked = math.copysign(max(abs(speed), abs(old_speed)), speed)
        limited, reason = self.guard.limit(checked, steer, now, current_steer=old_steer)
        return math.copysign(min(abs(speed), abs(limited)), speed), reason

    def on_scan(self, msg):
        points_ready = time.monotonic()
        with self.lock:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            age = self.get_clock().now().nanoseconds / 1e9 - stamp
            if stamp <= 0 or not 0 <= age < PROFILE['safety']['scan_timeout_s']:
                self.guard.invalidate()
                self.scan_health_at = None
                return
            self.guard.update_scan(msg.ranges, msg.angle_min, msg.angle_increment,
                                   msg.range_min, msg.range_max, points_ready - age)
            valid = (math.isfinite(msg.angle_min) and math.isfinite(msg.angle_increment)
                     and msg.angle_increment != 0 and math.isfinite(msg.range_min)
                     and math.isfinite(msg.range_max) and 0 <= msg.range_min < msg.range_max
                     and len(self.guard.points) >= PROFILE['safety']['min_scan_points'])
            self.scan_health_at = points_ready - age if valid else None

    def refresh_motion_health(self, now, *, manual_request=False):
        p = self.policy
        telemetry = p.last_received or {}
        voltage = telemetry.get('voltage', float('nan'))
        faults = []
        if not p.connected:
            faults.append('driver_disconnected')
        if p.holding:
            faults.append('driver_hold:' + (p.hold_reason or 'unknown'))
        if p.last_rx is None:
            faults.append('feedback_missing')
        elif not 0 <= now - p.last_rx <= self.config.feedback_timeout_s:
            faults.append('feedback_stale')
        manual = (not self.legacy_commands
                  and (manual_request or self.authority.mode == 'MANUAL'))
        if not manual:
            if self.scan_health_at is None:
                faults.append('scan_unavailable')
            elif not 0 <= now - self.scan_health_at < PROFILE['safety']['scan_timeout_s']:
                faults.append('scan_stale')
        if not math.isfinite(voltage):
            faults.append('battery_invalid')
        elif voltage < PROFILE['safety']['battery_min_v']:
            faults.append('battery_low')
        if self.config.receive_only:
            faults.append('driver_receive_only')
        if not self.config.protocol_confirmed or self.config.protocol == 'unconfigured':
            faults.append('protocol_unconfirmed')
        if now - p.connect_at < self.config.startup_stop_s:
            faults.append('startup_stop')
        self.motion_health_faults = tuple(faults)
        self.authority.health(not faults, p.stationary_frames >= 5, now)

    def clear_motion_output(self):
        self.policy.latest = None
        self.policy.output = (0.0, 0.0)

    def on_motion(self, source, message):
        try:
            data = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if not isinstance(data, dict) or data.get('profile_hash') != profile_hash(PROFILE):
            return
        with self.lock:
            now = time.monotonic()
            self.refresh_motion_health(now, manual_request=source == 'manual')
            epoch = self.authority.epoch
            self.authority.submit(source, data, now, self.get_clock().now().nanoseconds / 1e9)
            if epoch != self.authority.epoch:
                self.clear_motion_output()
                self.publish_status()

    def select_motion(self, source, request, response):
        with self.lock:
            now = time.monotonic()
            self.authority.output(now)
            self.refresh_motion_health(now)
            if request.data:
                response.success = self.authority.select(source)
            else:
                self.authority.release(source)
                response.success = True
            # Only the selected source may be released; a follower shutdown
            # or a rejected selection must not interrupt active manual drive.
            if (response.success and request.data) or self.authority.mode in ('IDLE', 'FAULT', 'ESTOP'):
                self.clear_motion_output()
            response.message = self.authority.mode + ': ' + self.authority.reason
        return response

    def reset_motion(self, request, response):
        with self.lock:
            now = time.monotonic()
            self.refresh_motion_health(now)
            response.success = self.authority.reset()
            if response.success:
                self.clear_motion_output()
                response.success, _ = self.policy.arm(now, clear_stall=True)
                if not response.success:
                    self.authority.fault('reset_failed', now=now)
            response.message = 'IDLE; select a new task' if response.success else 'wait for healthy sensors and stationary chassis'
        return response

    def apply_motion(self, now):
        # Runs under the SAME lock and serial tick as the final safety filter.
        self.refresh_motion_health(now)
        command = self.authority.output(now)
        active = self.authority.mode in ('MANUAL', 'FOLLOW', 'NAVIGATION')
        if active and not self.policy.armed and self.policy.ready(now) == 'ready':
            self.policy.arm(now)
        if not active or not self.policy.armed:
            self.clear_motion_output()
            return
        self.policy.command('twist', command.vx, command.wz, now)

    def on_arm(self, request, response):
        with self.lock:
            if request.data:
                if not self.legacy_commands and self.authority.mode in ('ESTOP', 'FAULT'):
                    response.success, response.message = False, 'use /motion/reset first'
                    return response
                response.success, response.message = self.policy.arm(
                    time.monotonic(), clear_stall=self.legacy_commands)
            else:
                self.authority.stop(emergency=True)
                self.policy.stop("operator_disarmed")
                response.success, response.message = True, "disarmed; zero frames scheduled"
        return response

    def on_stop(self, request, response):
        with self.lock:
            self.authority.stop(emergency=True)
            self.policy.stop("operator_stop")
            # Serial writes have no software backlog; state change takes effect at next tick.
        response.success, response.message = True, "stop latched; check telemetry and physical stop"
        return response

    # Rejecting one malformed command is not a safety fault -- ignoring that
    # command is. Latching a hard stop here meant a single stray message (say a
    # stationary-steering Twist from a UI) disarmed the chassis and demanded a
    # full stationary re-arm cycle. These reasons are recoverable: drop the
    # command, keep the arm.
    RECOVERABLE_REJECTS = (
        "use_ackermann_cmd_for_stationary_steering",
        "stationary_steering_needs_angle_protocol",
        "ackermann_timestamp_stale",
        "lateral_command_rejected",
    )

    def submit(self, kind, speed, turn, lateral=0.0):
        with self.lock:
            try:
                self.policy.command(kind, speed, turn, time.monotonic(), lateral)
            except ValueError as exc:
                reason = str(exc)
                self.last_error = reason
                if reason in self.RECOVERABLE_REJECTS:
                    self.policy.latest = None
                    self.policy.output = (0.0, 0.0)
                    self.policy.reason = "armed_waiting_command"
                else:
                    self.policy.stop(reason)

    def on_twist(self, message):
        if not self.legacy_commands:
            return
        self.submit("twist", message.linear.x, message.angular.z, message.linear.y)

    def on_ackermann(self, message):
        if not self.legacy_commands:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        if stamp <= 0 or not -0.1 <= age <= self.config.cmd_timeout_s:
            with self.lock:
                self.policy.latest = None
                self.policy.output = (0.0, 0.0)
                self.policy.reason = "ackermann_timestamp_stale"
            return
        self.submit("ackermann", message.drive.speed, message.drive.steering_angle)

    def write_frame(self, frame):
        # An 11-byte frame at 115200 baud drains in under 1 ms, against a 20 ms
        # transmit period -- so out_waiting is normally 0. But USB-CDC on the
        # RK3588 buffers in the host stack and reports a non-zero backlog now and
        # then for entirely benign reasons. Latching a stop on the first one was
        # the main cause of the stuttering, near-motionless web driving: stop ->
        # disarm -> car coasts to rest -> 5 stationary frames -> re-arm -> repeat.
        # Tolerate isolated backlogs; only a sustained run means the link is sick.
        try:
            if self.ser.out_waiting:
                self.backlog_streak += 1
                if self.backlog_streak > self.config.backlog_tolerance:
                    self.ser.reset_output_buffer()
                    self.policy.hold("serial_output_backlog", time.monotonic())
                    frame = STOP_FRAME
            else:
                if not self.write_timeout_streak and self.policy.hold_reason in ("serial_output_backlog", "serial_write_timeout"):
                    self.policy.release_hold()
                self.backlog_streak = 0
            n = self.ser.write(frame)
            if n != len(frame):
                raise IOError("partial serial write")
            self.write_timeout_streak = 0
            self.tx_packets += 1
            self.tx_bytes += n
            self.last_tx_hex = frame.hex(" ")
        except serial.SerialTimeoutException:
            self.write_timeouts += 1
            self.write_timeout_streak += 1
            self.last_write_timeout_at = time.monotonic()
            self.last_error = f"Write timeout ({self.write_timeout_streak})"
            # A timed-out write may contain a partial frame. Stop output until
            # a fresh zero frame succeeds; never replay the old speed.
            self.policy.hold("serial_write_timeout", self.last_write_timeout_at)
            try:
                self.ser.reset_output_buffer()
            except Exception:
                pass
            # The CH343 occasionally times out for more than one write and
            # recovers on the same fd. Keep output held at zero, but reconnect
            # only after the established sustained-backlog tolerance.
            if self.write_timeout_streak > self.config.backlog_tolerance:
                raise

    def io_loop(self):
        next_tx = time.monotonic()
        while self.running:
            try:
                if self.ser is None:
                    # The board's CH343 has exceeded shorter write deadlines
                    # while parked. Keep the previously stable bounded wait;
                    # the controller watchdog stops motion if feedback stalls.
                    self.ser = serial.Serial(self.port, 115200, timeout=0, write_timeout=1.00, exclusive=True)
                    # Drop only data from a prior session. All serial access belongs to this worker.
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                    with self.lock:
                        self.parser.buffer.clear()
                        self.last_odom = None
                        self.odometry_epoch = uuid.uuid4().hex
                        self.rx_times.clear()
                        self.write_timeout_streak = 0
                        self.backlog_streak = 0
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
                        if not self.legacy_commands:
                            self.apply_motion(now)
                        frame = self.policy.tick(now)
                        stall_detected = (self.policy.stall_latched
                                          and self.authority.mode not in ('FAULT', 'ESTOP'))
                        if stall_detected:
                            self.authority.fault('motion_stall', now=now)
                        if not self.config.receive_only:
                            self.write_frame(frame)
                        if stall_detected:
                            self.get_logger().error(
                                'Motion stall latched: output/feedback mismatch %s' % self.policy.stall_report)
                            self.publish_status()
                        # Never replay missed ticks in a burst.
                        next_tx = now + 1 / self.config.tx_hz
                time.sleep(0.002)
            except Exception as exc:
                with self.lock:
                    self.io_errors += 1
                    self.last_error = str(exc)
                    self.policy.link(False, time.monotonic())
                    self.authority.health(False, False, time.monotonic())
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
        imu.header.stamp, imu.header.frame_id = stamp, PROFILE["frames"]["imu"]
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
            arm_ready = p.ready(now)
            reported_ready = (p.hold_reason if p.holding else
                              "ready" if p.armed and arm_ready == "waiting_for_stationary_feedback"
                              else arm_ready)
            data = {"connected": p.connected, "armed": p.armed,
                    "reason": p.hold_reason or p.reason,
                    "holding": p.holding, "hold_reason": p.hold_reason,
                    "backlog_streak": self.backlog_streak,
                    "ready": reported_ready, "arm_ready": arm_ready,
                    "port": self.port, "device": os.path.realpath(self.port),
                    "baud": 115200, "config": vars(self.config), "hz": round(hz, 2),
                    "frames_ok": self.parser.good, "frames_bad": self.parser.bad,
                    "bytes_in": self.rx_bytes, "age_ms": round(age * 1000, 1) if age is not None else None,
                    "telemetry": p.last_received, "output_speed_turn": p.output,
                    "stall_latched": p.stall_latched, "stall_report": p.stall_report,
                    "tx_packets": self.tx_packets, "tx_bytes": self.tx_bytes, "last_tx_hex": self.last_tx_hex,
                    "io_errors": self.io_errors, "last_error": self.last_error,
                    "write_timeouts": self.write_timeouts,
                    "write_timeout_age_s": (round(now - self.last_write_timeout_at, 2)
                                            if self.last_write_timeout_at is not None else None),
                    "guard": {"enabled": self.guard.cfg.enabled,
                              "manual_override": (not self.legacy_commands
                                                  and self.authority.mode == 'MANUAL'
                                                  and p.armed and p.latest is not None
                                                  and not p.holding),
                              "reason": (p.guard_reason if p.armed and p.latest and not p.holding else None),
                              "last_reason": self.guard.last_reason,
                              "eval_ms": round(p.guard_eval_ms, 2),
                              "peak_eval_ms": round(p.guard_eval_peak_ms, 2),
                              "eval_overruns": p.guard_eval_overruns,
                              "min_passage_width_m": round(2 * (self.guard.cfg.half_width_m +
                                                                  self.guard.cfg.lateral_margin_m), 3),
                              "gap_m": (round(self.guard.last_gap, 3)
                                        if self.guard.last_gap not in (None, float("inf")) else None),
                              "block": ({"kind": self.guard.last_block[0],
                                         "x_m": round(self.guard.last_block[1], 3),
                                         "y_m": round(self.guard.last_block[2], 3),
                                         "steer_deg": round(math.degrees(self.guard.last_block[3]), 1)}
                                        if self.guard.last_block is not None else None),
                              "scan_age_ms": (round((now - self.guard.scan_time) * 1000)
                                              if self.guard.scan_time is not None else None),
                              "points": len(self.guard.points),
                              "interventions": self.guard.interventions},
                    "steering_feedback_available": False, "stop_confirmed": bool(age is not None and age < self.config.feedback_timeout_s and p.stationary_frames >= 5)}
            data['odometry_epoch'] = self.odometry_epoch
            data['profile_hash'] = profile_hash(PROFILE)
            data['legacy_commands'] = self.legacy_commands
            motion = self.authority.status(now)
            motion['legacy_commands'] = self.legacy_commands
            motion['profile_hash'] = profile_hash(PROFILE)
            motion['health_faults'] = list(self.motion_health_faults)
            data['motion'] = motion
        self.pub_status.publish(String(data=json.dumps(data, ensure_ascii=False)))
        self.pub_motion.publish(String(data=json.dumps(motion)))

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
