#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gen5 差速底盘驱动节点：两台欧艾迪单路驱动器（左右轮各一台），每台走 CAN（Linux SocketCAN）或 RS485
（Modbus RTU，板载 485 = /dev/ttyS3），由 left_bus / right_bus 选（2026-09-29：左轮 485、右轮 CAN）。

对上与 wheeltec_driver 完全相同：/follow|manual|navigation/command 租约指令、/motion/* 服务、急停、
/wheeltec/status 与 /motion/status 状态、/odom 里程计（后轴中点，twist 的角速度是左右轮速差算的，
原地转时也有），所以 line_node、网页、app 都不用改。控制权仲裁、上电等静止、命令超时、反馈超时、
雷达防撞沿用 wheeltec_driver 的实现（继承）；这里只换底盘：两轮差速（能原地转）、驱动器心跳、故障停车。

没有 IMU：wheeltec 板子自带的陀螺由它的驱动发 /imu；Gen5 的 /imu 由 imu 组件（rion_protocol/hda536t_imu.py，
瑞芬 HDA536T，同一条 CAN）发，融合必须有。

安全：confirmed=false（默认）时只收不发——不写驱动器配置、不发心跳和转速。架空轮子核对过方向、
磁极对数、左右轮后才设 true。
"""
import math
import os
import sys
import threading
import time
import uuid
from dataclasses import dataclass

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'wheeltec_protocol'))
sys.path.insert(0, os.path.dirname(HERE))

from robot_core.service_watchdog import watchdog
from runtime_config import PROFILE, profile_hash  # noqa: E402,F401
import wheeltec_driver as wd  # noqa: E402
from wheeltec_driver import ControlPolicy, finite  # noqa: E402
from oid_link import CanLink, OidChassis, Rs485Link, SerialPort, SocketCan, SplitLink, WheelConfig  # noqa: E402


# 两台驱动器走的总线：都 CAN、都 485，或一台 485 一台 CAN
BUS_KINDS = ('can', 'rs485', 'rs485+can')


@dataclass(frozen=True)
class DiffConfig:
    """字段名与 wheeltec_driver.Config 共用的那部分相同（ControlPolicy 和状态页按这些名字读）。"""
    protocol: str = 'can'                   # 两台驱动器走的总线：can、rs485 或 rs485+can（状态页按这个名字读）
    protocol_confirmed: bool = False        # 参数 confirmed
    receive_only: bool = True
    max_speed_m_s: float = PROFILE['driver']['max_speed_m_s']
    max_yaw_rate_rad_s: float = PROFILE['driver']['max_yaw_rate_rad_s']
    acceleration_m_s2: float = PROFILE['driver']['acceleration_m_s2']
    yaw_acceleration_rad_s2: float = 2.0
    cmd_timeout_s: float = PROFILE['driver']['cmd_timeout_s']
    feedback_timeout_s: float = PROFILE['driver']['feedback_timeout_s']
    startup_stop_s: float = PROFILE['driver']['startup_stop_s']
    tx_hz: float = 25.0
    feedback_grace_s: float = 1.0
    backlog_tolerance: int = 8

    def __post_init__(self):
        if self.protocol not in ('unconfigured',) + BUS_KINDS:
            raise ValueError('总线只能是 CAN、RS485 或两者各一台')
        finite(*[v for v in vars(self).values() if isinstance(v, (int, float)) and not isinstance(v, bool)])
        if not 0 < self.max_speed_m_s <= 2.5 or not 0 < self.max_yaw_rate_rad_s <= 3.0:
            raise ValueError('速度或角速度上限不合理')
        if not 0 < self.acceleration_m_s2 <= 6.0 or not 0 < self.yaw_acceleration_rad_s2 <= 10.0:
            raise ValueError('加速度不合理')
        if not 0.05 <= self.cmd_timeout_s <= 1 or not 0.05 <= self.feedback_timeout_s <= 1:
            raise ValueError('超时不合理')
        if not 3 <= self.startup_stop_s <= 10 or not 10 <= self.tx_hz <= 50:
            raise ValueError('启动静止时间或发送频率不合理')


# 停稳且没有驾驶指令时放空电机：纯霍尔的驱动器锁在零速闭环里会来回找位置，停着也咔哒响、耗电。
# 先按零速刹停，连续这么多帧（25 Hz，约 1 s）读到静止才放空，之后只读反馈；一有指令、或车被推动
# （不再静止）就回到速度闭环锁零速
RELEASE_STATIONARY_FRAMES = 25


class DiffControlPolicy(ControlPolicy):
    """与 wheeltec 的 ControlPolicy 同一套上电、超时、保持、防撞逻辑；差速车允许原地转（车速 0、角速度不为 0），
    输出 (车速, 角速度) 而不是 wheeltec 帧。驱动器报故障时停车，故障消失前不重新上电。"""

    def __init__(self, config, now):
        super().__init__(config, now)
        self.fault = None

    def ready(self, now):
        reason = super().ready(now)
        if reason == 'ready' and self.fault:
            return 'driver_fault:' + self.fault
        return reason

    def command(self, kind, speed, turn, now, lateral=0.0):
        try:
            finite(speed, turn, lateral)
        except ValueError:
            self.stop('non_finite_command')
            raise
        c = self.config
        if abs(lateral) > 1e-9:
            self.reject('lateral_command_rejected')
        if not self.armed:
            raise ValueError('not armed')
        if kind != 'twist':
            self.stop('unknown_command')
            raise ValueError(self.reason)
        speed = max(-c.max_speed_m_s, min(c.max_speed_m_s, speed))
        yaw = max(-c.max_yaw_rate_rad_s, min(c.max_yaw_rate_rad_s, turn))
        self.latest = (now, speed, yaw)
        self.reason = 'command_active'

    def tick(self, now):
        """返回这一周期要下发的 (车速, 角速度)。"""
        c = self.config
        dt = min(max(now - self.last_tick, 0.0), 3 / c.tx_hz)
        self.last_tick = now
        if self.armed and self.fault:
            self.stop('driver_fault:' + self.fault)
        if self.armed:
            stale = self.last_rx is None or now - self.last_rx > c.feedback_timeout_s
            if stale:
                self.hold('feedback_stale', now)
                if now - self.hold_since > c.feedback_grace_s:
                    self.stop('feedback_lost')
            elif self.hold_reason == 'feedback_stale':
                self.release_hold()
        if self.armed and self.holding and now - self.hold_since > c.feedback_grace_s:
            self.stop(self.hold_reason or 'hold_expired')
        if self.armed and self.latest and now - self.latest[0] > c.cmd_timeout_s:
            self.latest = None
            self.output = (0.0, 0.0)
            self.reason = 'armed_waiting_command'
        if (not self.armed and self.reason not in ('operator_stop', 'operator_disarmed')
                and self.ready(now) == 'ready'):
            self.armed = True
            self.reason = 'armed_waiting_command'
        if not self.connected or not self.armed or not self.latest or self.holding:
            self.output = (0.0, 0.0)
            return self.output
        _, speed, turn = self.latest
        old_speed, old_turn = self.output
        self.guard_reason = None
        if self.speed_filter is not None and abs(speed) > 1e-9:
            original = speed
            started = time.perf_counter()
            try:
                speed, self.guard_reason = self.speed_filter(speed, turn, now)
            finally:
                self.guard_eval_ms = (time.perf_counter() - started) * 1000
                self.guard_eval_peak_ms = max(self.guard_eval_peak_ms, self.guard_eval_ms)
                if self.guard_eval_ms > 1000 / c.tx_hz:
                    self.guard_eval_overruns += 1
            turn *= abs(speed / original)       # 防撞减速保持弯的半径
        # 车速：给 0 或换向立即 0（驱动器自己按减速度斜坡停下），否则按加速度爬坡；角速度按角加速度爬坡
        if speed == 0 or old_speed * speed < 0:
            out_speed = 0.0
        else:
            out_speed = old_speed + max(-c.acceleration_m_s2 * dt, min(c.acceleration_m_s2 * dt, speed - old_speed))
            if self.guard_reason is not None and abs(out_speed) > abs(speed):
                out_speed = speed
        step = c.yaw_acceleration_rad_s2 * dt
        out_turn = old_turn + max(-step, min(step, turn - old_turn))
        self.output = (out_speed, out_turn)
        return self.output


class _Counters:
    """wheeltec 状态页里的 frames_ok / frames_bad。"""

    def __init__(self):
        self.good = self.bad = 0


def wheel_buses(params):
    """{1: 左轮总线, 2: 右轮总线}，只能是 can / rs485。"""
    buses = {1: str(params['left_bus']).lower(), 2: str(params['right_bus']).lower()}
    for m, bus in buses.items():
        if bus not in ('can', 'rs485'):
            raise ValueError('%s_bus 只能是 can 或 rs485：%r' % (('left', 'right')[m - 1], bus))
    return buses


def bus_kind(params):
    kinds = set(wheel_buses(params).values())
    return kinds.pop() if len(kinds) == 1 else 'rs485+can'


def bus_label(params):
    """状态页显示的接口：can0、/dev/ttyS3，或两者。"""
    kinds = set(wheel_buses(params).values())
    return ' + '.join(x for x in (params['rs485_port'] if 'rs485' in kinds else None,
                                  params['can_channel'] if 'can' in kinds else None) if x)


def build_link(params):
    """按 left_bus / right_bus 建链路：CAN 用节点 ID（单路驱动器用同一个 ID 回复），485 用站号；两台走不同的总线时拼成
    SplitLink。一条打开失败时关掉已打开的另一条再报错。"""
    buses, links = wheel_buses(params), []
    can = {m: int(params[('can_left_id', 'can_right_id')[m - 1]]) for m, b in buses.items() if b == 'can'}
    rs485 = {m: int(params[('rs485_left_id', 'rs485_right_id')[m - 1]]) for m, b in buses.items() if b == 'rs485'}
    try:
        if rs485:
            port = SerialPort(params['rs485_port'], int(params['rs485_baud']), str(params['rs485_parity']),
                              echo=bool(params['rs485_echo']))
            links.append(Rs485Link(port, rs485, heartbeat_s=params['heartbeat_s'],
                                   word_order=str(params['rs485_word_order']), latency_s=float(params['rs485_timeout_s'])))
        if can:
            links.append(CanLink(SocketCan(params['can_channel']), can, heartbeat_s=params['heartbeat_s']))
    except Exception:
        for link in links:
            link.close()
        raise
    return links[0] if len(links) == 1 else SplitLink(links)


PARAMS = dict(confirmed=False, left_bus='can', right_bus='can',
              can_channel='can0', can_left_id=1, can_right_id=2,
              rs485_port='/dev/ttyS3', rs485_baud=115200, rs485_parity='N', rs485_left_id=1, rs485_right_id=2,
              rs485_word_order='big', rs485_timeout_s=0.03, rs485_echo=False,
              heartbeat_s=0.1, pole_pairs=0, wheel_radius_m=0.2046, gear_ratio=1.0,
              left_sign=1, right_sign=-1, wheel_accel_mps2=1.0, wheel_decel_mps2=2.0,
              tx_hz=25.0, yaw_acceleration_rad_s2=2.0, publish_tf=False, navigation_uses_lidar=False)


class OidDriver(wd.WheeltecDriver):
    def __init__(self):     # noqa: C901 - 与 WheeltecDriver 相同的 ROS 接口，逐项建
        if not wd.ROS_AVAILABLE:
            raise RuntimeError('需要 ROS 2 Jazzy（rclpy）')
        wd.Node.__init__(self, 'oid_driver')
        if PROFILE['drive']['type'] != 'differential':
            raise ValueError('oid_driver 只用于差速底盘：RK3588_ROBOT_CONFIG 要指向 robot_gen5.json')
        for name, value in PARAMS.items():
            self.declare_parameter(name, value, wd.ParameterDescriptor(read_only=True))
        self.declare_parameter('lidar_installed', os.environ.get('RK3588_LIDAR_INSTALLED', '1') != '0',
                               wd.ParameterDescriptor(read_only=True))
        gp = lambda k: self.get_parameter(k).value  # noqa: E731
        self.params = {k: gp(k) for k in PARAMS}
        confirmed = bool(gp('confirmed'))
        self.config = DiffConfig(protocol=bus_kind(self.params), protocol_confirmed=confirmed, receive_only=not confirmed,
                                 tx_hz=float(gp('tx_hz')), yaw_acceleration_rad_s2=float(gp('yaw_acceleration_rad_s2')))
        self.wheels = WheelConfig(wheel_radius_m=float(gp('wheel_radius_m')), track_m=PROFILE['geometry']['track_m'],
                                  pole_pairs=int(gp('pole_pairs')), gear_ratio=float(gp('gear_ratio')),
                                  left_sign=int(gp('left_sign')),
                                  right_sign=int(gp('right_sign')), max_wheel_mps=PROFILE['driver']['max_speed_m_s'],
                                  accel_mps2=float(gp('wheel_accel_mps2')), decel_mps2=float(gp('wheel_decel_mps2')))
        if confirmed and not self.wheels.configured:
            raise ValueError('confirmed=true 之前先填 pole_pairs（上位机识别的磁极对数）')
        self.legacy_commands = False
        self.navigation_uses_lidar = bool(gp('navigation_uses_lidar'))
        self.lidar_installed = bool(gp('lidar_installed'))
        self.publish_tf = bool(gp('publish_tf'))
        self.frame_id, self.base_frame_id = PROFILE['frames']['odom'], PROFILE['frames']['base']
        self.guard = wd.ScanGuard(wd.GuardConfig(decel_m_s2=PROFILE['safety']['decel_mps2'],
                                                 latency_s=PROFILE['safety']['guard_latency_s'],
                                                 scan_timeout_s=PROFILE['safety']['scan_timeout_s']))
        # 差速车手动打横是原地转（车速 0、角速度不为 0），仲裁不能把它当成停车
        self.authority = wd.MotionAuthority(PROFILE['safety']['command_timeout_s'], PROFILE['safety']['stationary_s'],
                                            self.config.feedback_grace_s, allow_pivot=True)
        self.scan_health_at = None
        self.motion_health_faults = ()
        self.lock = threading.RLock()
        self.policy = DiffControlPolicy(self.config, time.monotonic())
        if self.guard.cfg.enabled:
            self.policy.speed_filter = self.guard_speed_filter
        self.parser = _Counters()
        self.chassis, self.port = None, bus_label(self.params)
        self.released = False               # 停稳后已放空电机（见 RELEASE_STATIONARY_FRAMES）
        self.running = True
        self.tx_packets = self.tx_bytes = self.rx_bytes = self.io_errors = 0
        self.last_tx_hex = ''
        self.last_error = ''
        self.backlog_streak = self.write_timeouts = self.write_timeout_streak = 0
        self.last_write_timeout_at = None
        self.started = time.monotonic()
        self.last_odom = None
        self.odometry_epoch = uuid.uuid4().hex
        self.position = [0.0, 0.0, 0.0]
        self.rx_times = []
        self.pub_odom = self.create_publisher(wd.Odometry, PROFILE['localization']['driver_topic'], 10)
        self.pub_voltage = self.create_publisher(wd.Float32, '/voltage', 10)
        self.pub_status = self.create_publisher(wd.String, '/wheeltec/status', 10)
        self.pub_motion = self.create_publisher(wd.String, '/motion/status', 1)
        self.pub_oid = self.create_publisher(wd.String, '/oid/status', 10)
        self.tf = wd.TransformBroadcaster(self) if self.publish_tf else None
        qos = wd.QoSProfile(depth=1, history=wd.HistoryPolicy.KEEP_LAST, durability=wd.DurabilityPolicy.VOLATILE,
                            lifespan=wd.Duration(seconds=self.config.cmd_timeout_s))
        for source in wd.MotionAuthority.SOURCES:
            self.create_subscription(wd.String, '/' + source + '/command',
                                     lambda msg, src=source: self.on_motion(src, msg), qos)
        self.create_service(wd.SetBool, '/motion/follow', lambda req, res: self.select_motion('follow', req, res))
        self.create_service(wd.SetBool, '/motion/navigation', lambda req, res: self.select_motion('navigation', req, res))
        self.create_service(wd.Trigger, '/motion/reset', self.reset_motion)
        self.create_service(wd.Trigger, '/motion/stop', self.on_stop)
        self.create_subscription(wd.LaserScan, '/scan', self.on_scan,
                                 wd.QoSProfile(depth=1, reliability=wd.ReliabilityPolicy.BEST_EFFORT))
        self.create_service(wd.SetBool, '/wheeltec/arm', self.on_arm)
        self.create_service(wd.Trigger, '/wheeltec/stop', self.on_stop)
        self.create_timer(0.2, self.publish_status)
        self.create_timer(0.5, self.publish_oid_status)
        self.worker = threading.Thread(target=self.io_loop, daemon=True)
        self.worker.start()

    def refresh_motion_health(self, now, *, manual_request=False, navigation_request=False):
        """与 WheeltecDriver 相同，另把驱动器故障算进健康检查：任务转 FAULT，故障消失后也要操作员复位才能再动。"""
        p = self.policy
        telemetry = p.last_received or {}
        voltage = telemetry.get('voltage', float('nan'))
        faults = []
        if not p.connected:
            faults.append('driver_disconnected')
        if p.fault:
            faults.append('driver_fault:' + p.fault)
        if p.holding:
            faults.append('driver_hold:' + (p.hold_reason or 'unknown'))
        if p.last_rx is None:
            faults.append('feedback_missing')
        elif not 0 <= now - p.last_rx <= self.config.feedback_timeout_s:
            faults.append('feedback_stale')
        if self.chassis is not None:
            # 两台单路驱动器各自可能掉线：说清是哪一台
            faults.extend('driver_silent:%s轮' % ('左', '右')[m - 1] for m in self.chassis.silent(now))
        manual = manual_request or self.authority.mode == 'MANUAL'
        no_lidar = manual or self.lidar_bypassed() or (navigation_request and self.lidar_bypassed('NAVIGATION'))
        if not no_lidar:
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

    def io_loop(self):
        next_tx = time.monotonic()
        while self.running:
            watchdog.tick("io")
            try:
                now = time.monotonic()
                if self.chassis is None or now >= next_tx:
                    self._io_step(now)
                    next_tx = now + 1 / self.config.tx_hz
                time.sleep(0.002)
            except Exception as exc:
                with self.lock:
                    self.io_errors += 1
                    self.parser.bad += 1
                    self.last_error = str(exc)
                    self.policy.link(False, time.monotonic())
                    self.authority.health(False, False, time.monotonic())
                self._close()
                time.sleep(0.25)
        send = not self.config.receive_only
        # 退出：两轮转速 0，再放空电机；之后不再发心跳，驱动器超时后也会放空
        if self.chassis and send:
            try:
                end = time.monotonic() + 1.0
                while time.monotonic() < end:
                    self.chassis.step(time.monotonic(), 0.0, 0.0)
                    time.sleep(1 / self.config.tx_hz)
                self.chassis.release(time.monotonic())
            except Exception:
                pass
        self._close()

    def _io_step(self, now):
        """一个控制周期：连上（第一次）→ 仲裁、控制策略给出 (车速, 角速度) → 发给驱动器、取反馈 → 更新状态。
        总线收发不占锁；异常由 io_loop 处理（断开、0.25 s 后重连）。"""
        send = not self.config.receive_only
        if self.chassis is None:
            chassis = OidChassis(build_link(self.params), self.wheels, feedback_timeout_s=self.config.feedback_timeout_s)
            if send:
                chassis.open(now)
            with self.lock:
                self.chassis = chassis
                self.last_odom = None
                self.odometry_epoch = uuid.uuid4().hex
                self.rx_times.clear()
                self.policy.link(True, now)
                self.released = False
        with self.lock:
            self.apply_motion(now)
            v, w = self.policy.tick(now)
            idle = (send and v == 0.0 and w == 0.0 and self.policy.latest is None
                    and self.policy.stationary_frames >= RELEASE_STATIONARY_FRAMES)
        if idle:
            if not self.released:
                self.chassis.release(now)
                self.released = True
            updated = self.chassis.step(now, 0.0, 0.0, send=False)     # 只读反馈，不发心跳和零速
        else:
            self.released = False
            updated = self.chassis.step(now, v, w, send=send)
        with self.lock:
            self.tx_packets = self.chassis.link.tx
            if updated:
                telemetry = self.chassis.telemetry(now)
                self.policy.fault = ('、'.join('%s轮驱动器：%s' % kv for kv in telemetry['faults'].items())
                                     if self.chassis.faults() else None)
                self.policy.feedback(telemetry, now)
                self.parser.good += 1
                self.rx_times.append(now)
                self.rx_times = self.rx_times[-100:]
                self.publish_telemetry(telemetry, now)
        return v, w

    def _close(self):
        chassis, self.chassis = self.chassis, None
        link = getattr(chassis, 'link', None)
        try:
            if link is not None:
                link.close()
        except Exception:
            pass

    def publish_telemetry(self, t, now):
        o = self.chassis.odom
        stamp = self.get_clock().now().to_msg()
        odom = wd.Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, self.frame_id, self.base_frame_id
        odom.pose.pose.position.x, odom.pose.pose.position.y = o.x, o.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(o.yaw / 2), math.cos(o.yaw / 2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = o.v, o.w
        for i in (0, 7, 35):
            odom.pose.covariance[i] = odom.twist.covariance[i] = 0.05
        for i in (14, 21, 28):
            odom.pose.covariance[i] = odom.twist.covariance[i] = 1e6
        self.pub_odom.publish(odom)
        if math.isfinite(t['voltage']):
            self.pub_voltage.publish(wd.Float32(data=float(t['voltage'])))
        self.position = [o.x, o.y, o.yaw]
        if self.tf:
            tf = wd.TransformStamped()
            tf.header, tf.child_frame_id = odom.header, self.base_frame_id
            tf.transform.translation.x, tf.transform.translation.y = o.x, o.y
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(tf)

    def publish_oid_status(self):
        import json
        with self.lock:
            c = self.chassis
            data = dict(bus=self.config.protocol, channel=self.port, confirmed=self.config.protocol_confirmed,
                        wheel_buses={('左', '右')[m - 1]: b for m, b in wheel_buses(self.params).items()},
                        wheels=vars(self.wheels), connected=c is not None, last_error=self.last_error,
                        io_errors=self.io_errors, released=self.released)
            if c is not None:
                data.update(telemetry=c.telemetry(time.monotonic()), tx=c.link.tx, rx=c.link.rx,
                            link_errors=getattr(c.link, 'errors', 0), link_last_error=getattr(c.link, 'last_error', ''))
        self.pub_oid.publish(wd.String(data=json.dumps(data, ensure_ascii=False, default=str)))

    def shutdown(self):
        with self.lock:
            self.policy.stop('shutdown')
        self.running = False
        self.worker.join(timeout=self.config.startup_stop_s + 2)


def main():
    if not wd.ROS_AVAILABLE:
        raise SystemExit('先 source ROS Jazzy')
    watchdog.start("ros", "io")
    wd.rclpy.init()
    node = None
    try:
        node = OidDriver()
        node.create_timer(0.5, lambda: watchdog.tick("ros"))
        wd.rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        watchdog.close()
        if node:
            node.shutdown()
            node.destroy_node()
        if wd.rclpy.ok():
            wd.rclpy.shutdown()


if __name__ == '__main__':
    main()
