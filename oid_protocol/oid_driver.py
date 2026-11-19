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
import json
import math
import os
import sys
import threading
import time
import uuid
import traceback
from dataclasses import dataclass

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'wheeltec_protocol'))
sys.path.insert(0, os.path.dirname(HERE))

from robot_core.service_watchdog import watchdog
from runtime_config import PROFILE, profile_hash  # noqa: E402,F401
import wheeltec_driver as wd  # noqa: E402
from speed_trim import SpeedTrim  # noqa: E402
from wheeltec_driver import ControlPolicy, finite  # noqa: E402
from oid_link import (CanLink, DriverTuning, OidChassis, Rs485Link, SerialPort, SocketCan, SplitLink,  # noqa: E402
                      TUNING_MAX_CURRENT_A, WheelConfig, HALL_STILL_MAX_MPS)


# 两台驱动器走的总线：都 CAN、都 485，或一台 485 一台 CAN
BUS_KINDS = ('can', 'rs485', 'rs485+can')
# /oid/trace 里电流等负载读数超过这么久没更新就写 null，不把旧值当现在的（CAN 每 0.1 s 查一次）
LOAD_MAX_AGE_S = 0.5


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
    manual_stop_decel_mps2: float = 1.5       # 普通松手停车；急停仍由驱动器减速度制动
    cmd_timeout_s: float = PROFILE['driver']['cmd_timeout_s']
    feedback_timeout_s: float = PROFILE['driver']['feedback_timeout_s']
    startup_stop_s: float = PROFILE['driver']['startup_stop_s']
    tx_hz: float = 50.0
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
        if not 0 < self.manual_stop_decel_mps2 <= 6.0:
            raise ValueError('手动停车减速度不合理')
        # 反馈超时最多 2 s（2026-10-06 用户：驱动器不超过 2 s 没回话不报故障、继续作业；右轮走 CAN，
        # 总线忙时回话会断 0.8–1.1 s，1 s 判定时 08:14 报过一次 driver_silent:右轮）
        if not 0.05 <= self.cmd_timeout_s <= 1 or not 0.05 <= self.feedback_timeout_s <= 2:
            raise ValueError('超时不合理')
        if not 3 <= self.startup_stop_s <= 10 or not 10 <= self.tx_hz <= 50:
            raise ValueError('启动静止时间或发送频率不合理')


# 停车放空（2026-11-19 方案一）。纯霍尔的驱动器锁在零速闭环里会来回找位置；11-19 为草地起步调硬的零速 Kp/Ki
# （不改回去，不然草地开不动）在地上锁零速时自己约 1 Hz 前后摇，原来「停稳满 1 s 才放空」永远等不到、一直摇。改为：
# - 开机还没有任何操作：直接放空，不先发零速；
# - 停车（空闲、手动松手、急停、故障）：先发零速刹车，刹到零（wheels_slow：两轮读回都不超过 HALL_STILL_MAX_MPS，
#   或已经反转）马上放空，之后只读反馈，手推时保持放空。不等霍尔安静：11-19 地上手动停车实测，驱动器零速闭环在
#   轮子过零后会往回打 0.2–0.4 m/s（车往后蹿一下），等霍尔 0.25 s 不跨格正好让它打完；刹到零就放空，最后顺着滑停几厘米；
# - 自动作业（导航、跟随）中途发零速照旧锁零速，不放空；
# - 兜底（任何状态）：在发零速、过了正常刹车时间轮子还一直在动，就放空；作业中同时转故障（zero_hold_oscillation），
#   作业中断、断点保留，复位后可「继续作业」；急停时驱动器不回话也放空。
# 新的手动指令或自动作业重新取得控制时恢复速度闭环：起步从放空直接发目标转速。
ZERO_HOLD_FRESH_S = 0.2             # 判「刹到零」只认这么新的反馈
ZERO_HOLD_MOVING_RELEASE_S = 0.5    # 空闲、手动、急停、故障：发零速后轮子一直在动这么久就放空
# 作业中放空要连带中断作业，判得更严：只认霍尔跨格（读回转速在零速闭环里会跳），一直在动这么久才转故障。
# 2026-10-02 作业中一拍停车后零速闭环回拉过 9–11 cm（约 1 s），不能当成振荡把作业停掉。
ZERO_HOLD_JOB_FAULT_S = 2.0
ZERO_HOLD_MOVING_GAP_S = 0.4        # 振荡换向前后会短暂不跨格：两次「在动」隔得不超过这么久仍算一直在动
ZERO_HOLD_BRAKE_RESPONSE_S = 0.3    # 驱动器响应约 0.25 s（2026-09-29 实车换向实测），算正常刹车时间时另加这一段
ZERO_HOLD_RELEASE_RESEND_S = 0.5    # 放空以后轮子还在动或驱动器还不回话：隔这么久再发一次放空


class ZeroHoldGuard:
    """一段连续发零速（同一仲裁代次）里轮子的运动记录：判「零速压不住」。纯计算，调用方持锁。"""

    def __init__(self):
        self.clear()

    def clear(self):
        self.epoch = self.started = None
        self.brake_until = None
        self.moving_since = self.moving_at = None
        self.reason = None          # 已因此放空驱动器：wheels_moving（零速压不住）/ driver_silent（急停时驱动器不回话）
        self.sent_at = None
        self.directions = (0, 0)    # 开始发零速时左右轮的转向（+1 / -1；读回不超过 HALL_STILL_MAX_MPS 为 0）

    def start(self, epoch, now, speed_mps, decel_mps2, directions=(0, 0)):
        self.clear()
        self.epoch, self.started = epoch, now
        self.brake_until = now + abs(speed_mps) / max(decel_mps2, 0.1) + ZERO_HOLD_BRAKE_RESPONSE_S
        self.directions = tuple(directions)

    def observe(self, motion_at):
        """每次读回反馈调一次；motion_at = 最近一次确实在动的时刻，没有为 None。开始发零速之前的运动不算。"""
        if self.epoch is None or motion_at is None or motion_at < self.started:
            return
        if self.moving_at is not None and motion_at <= self.moving_at:
            return
        if self.moving_at is None or motion_at - self.moving_at > ZERO_HOLD_MOVING_GAP_S:
            self.moving_since = motion_at
        self.moving_at = motion_at

    def moving(self, now):
        return self.moving_at is not None and 0 <= now - self.moving_at <= ZERO_HOLD_MOVING_GAP_S

    def kept_moving(self, now, hold_s):
        """发零速以后轮子一直在动满 hold_s，并且过了正常刹车该停下的时刻还在动（正常刹车在那之前就停了）。"""
        return (self.moving(now) and self.moving_at >= self.brake_until
                and now - self.moving_since >= hold_s)


class DiffControlPolicy(ControlPolicy):
    """与 wheeltec 的 ControlPolicy 同一套上电、超时、保持、防撞逻辑；差速车允许原地转（车速 0、角速度不为 0），
    输出 (车速, 角速度) 而不是 wheeltec 帧。驱动器报故障时停车，故障消失前不重新上电。"""

    def __init__(self, config, now):
        super().__init__(config, now)
        self.fault = None
        self.manual_stop_deadline = None

    def manual_stopping(self, now):
        """只允许已经收到零速的手动停车收尾，截止时间不能被重复零速续期。"""
        return (self.manual_stop_deadline is not None and now < self.manual_stop_deadline
                and self.output != (0.0, 0.0) and self.armed and not self.holding)

    def feedback(self, telemetry, now):
        frames = self.stationary_frames
        super().feedback(telemetry, now)
        raw_moving=any(isinstance(v,(int,float)) and math.isfinite(v) and abs(v)>HALL_STILL_MAX_MPS
                       for v in telemetry.get('wheels_mps') or [])
        if raw_moving and not telemetry.get('hall_still'):
            # 第一个跨格前尚不能确定霍尔速度。另一份反馈已明确报告运动时，不能误续静止计时。
            self.stationary_frames=0
        if not self.stationary_frames and telemetry.get('hall_still'):
            # 回读转速在零速闭环里跳、霍尔计数不动：按与导航共用的霍尔静止证据计时
            self.stationary_frames = frames + 1

    def ready(self, now):
        reason = super().ready(now)
        if reason == 'ready' and self.fault:
            return 'driver_fault:' + self.fault
        return reason

    def command(self, kind, speed, turn, now, lateral=0.0, *, manual_stop=False):
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
        if manual_stop and speed == 0.0 and yaw == 0.0 and self.output != (0.0, 0.0):
            if self.manual_stop_deadline is None:
                duration = max(abs(self.output[0]) / c.manual_stop_decel_mps2,
                               abs(self.output[1]) / c.yaw_acceleration_rad_s2)
                self.manual_stop_deadline = now + duration + 2 / c.tx_hz
        else:
            self.manual_stop_deadline = None
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
        if self.manual_stop_deadline is not None and speed == 0.0 and turn == 0.0:
            # 两轴用同一比例收敛：左右轮同步减速，不能因车速先归零、转向还没归零而让一轮反转。
            duration = max(abs(old_speed) / c.manual_stop_decel_mps2,
                           abs(old_turn) / c.yaw_acceleration_rad_s2)
            scale = max(0.0, 1.0 - dt / duration) if duration > dt and now < self.manual_stop_deadline else 0.0
            self.output = (old_speed * scale, old_turn * scale)
            if scale == 0.0:
                self.manual_stop_deadline = None
            return self.output
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
                                   word_order=str(params['rs485_word_order']), latency_s=float(params['rs485_timeout_s']),
                                   clock=time.monotonic))
        if can:
            links.append(CanLink(SocketCan(params['can_channel']), can, heartbeat_s=params['heartbeat_s'], clock=time.monotonic,
                                 frame_gap_s=float(params.get('can_frame_gap_s', 0.0))))
    except Exception:
        for link in links:
            link.close()
        raise
    return links[0] if len(links) == 1 else SplitLink(links)


PARAMS = dict(confirmed=False, left_bus='can', right_bus='can',
              can_channel='can0', can_left_id=1, can_right_id=2, can_frame_gap_s=0.0,
              rs485_port='/dev/ttyS3', rs485_baud=115200, rs485_parity='N', rs485_left_id=1, rs485_right_id=2,
              rs485_word_order='big', rs485_timeout_s=0.03, rs485_echo=False,
              heartbeat_s=0.1, pole_pairs=0, erpm_per_rpm=0.0, wheel_radius_m=0.2046, gear_ratio=1.0,
              left_sign=1, right_sign=-1, wheel_accel_mps2=1.0, wheel_decel_mps2=2.0,
              tx_hz=50.0, yaw_acceleration_rad_s2=2.0, manual_stop_decel_mps2=1.5,
              publish_tf=False, navigation_uses_lidar=False,
              # App「驱动器参数」：闭环最大电流上限（0 = 不许改，最多 TUNING_MAX_CURRENT_A）、保存位置（空 = 不能改）
              max_current_limit_a=0.0, tuning_file='',
              # 低速转速修正（speed_trim.py）：按位置读数算真实轮速修正下发转速；表存 speed_trim_file（空 = 不存）
              speed_trim=False, speed_trim_file='')


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
                                 tx_hz=float(gp('tx_hz')), yaw_acceleration_rad_s2=float(gp('yaw_acceleration_rad_s2')),
                                 manual_stop_decel_mps2=float(gp('manual_stop_decel_mps2')))
        self.wheels = WheelConfig(wheel_radius_m=float(gp('wheel_radius_m')), track_m=PROFILE['geometry']['track_m'],
                                  pole_pairs=int(gp('pole_pairs')), erpm_per_rpm=float(gp('erpm_per_rpm')),
                                  gear_ratio=float(gp('gear_ratio')),
                                  left_sign=int(gp('left_sign')),
                                  right_sign=int(gp('right_sign')), max_wheel_mps=PROFILE['driver']['max_speed_m_s'],
                                  accel_mps2=float(gp('wheel_accel_mps2')), decel_mps2=float(gp('wheel_decel_mps2')))
        if confirmed and not self.wheels.configured:
            raise ValueError('confirmed=true 之前先填 pole_pairs（上位机识别的磁极对数）')
        # App「驱动器参数」（只支持两台都接 CAN）：存在车上，连上驱动器时、驱动器断电重启回话时重发
        self.tuning_limit_a = min(TUNING_MAX_CURRENT_A, max(0.0, float(gp('max_current_limit_a'))))
        self.tuning_path = os.path.expanduser(str(gp('tuning_file') or ''))
        self.tuning_supported = confirmed and set(wheel_buses(self.params).values()) == {'can'}
        self.tuning, self.tuning_error = self.load_tuning()
        # 重连时沿用同一个对象：学到的增益不丢
        self.speed_trim = (SpeedTrim(self.wheels, path=os.path.expanduser(str(gp('speed_trim_file') or '')))
                           if bool(gp('speed_trim')) and self.wheels.configured else None)
        self.tune_pending = None            # 已保存、等 IO 线程发给驱动器的指令 id
        self.tune_result = None             # 最近一次修改的结果 {id, ok, message}，ok=None 为正在下发
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
        self.released = False               # 已放空电机（见 ZERO_HOLD_* 上面的说明）
        self.zero_hold = ZeroHoldGuard()    # 发零速时零速压不住 / 急停时驱动器不回话就放空
        self.push_mode = False              # 明确退出前保持放空，手推移动不能恢复零速锁定
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
        self.pub_wheel_motion = self.create_publisher(wd.String, '/wheel/odometry', 50)
        # 每读回一次反馈发一条（约 50 Hz）：限速后的指令、左右轮下发 / 读回转速，作业记录存下来分析走不直（/oid/status 只有 2 Hz）
        self.pub_trace = self.create_publisher(wd.String, '/oid/trace', 50)
        self._cmd = (0.0, 0.0)
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
        self.create_service(wd.SetBool, '/motion/push', self.on_push)
        self.create_subscription(wd.String, '/oid/tune', self.on_tune, 10)
        self.create_subscription(wd.LaserScan, '/scan', self.on_scan,
                                 wd.QoSProfile(depth=1, reliability=wd.ReliabilityPolicy.BEST_EFFORT))
        self.create_service(wd.SetBool, '/wheeltec/arm', self.on_arm)
        self.create_service(wd.Trigger, '/wheeltec/stop', self.on_stop)
        self.create_timer(0.2, self.publish_status)
        self.create_timer(0.5, self.publish_oid_status)
        self.worker = threading.Thread(target=self.io_loop, daemon=True)
        self.worker.start()

    def apply_motion(self, now):
        """普通手动零速渐停；急停、故障、未收到零速的超时仍立即清零。"""
        p = self.policy
        if self.push_mode:
            self.refresh_motion_health(now, manual_request=True)
            self.clear_motion_output()
            p.manual_stop_deadline = None
            return
        braking = p.manual_stopping(now)
        self.refresh_motion_health(now, manual_request=braking)
        command = self.authority.output(now)
        mode = self.authority.mode
        # App 松手只发一帧零速。仲裁 0.35 s 后回空闲，也要完成这次有截止时间的停车，
        # 不能把余下速度再突然清零。失联时没有收到零速，就没有这条收尾路径。
        finishing = (braking and mode == 'IDLE' and self.authority.reason == 'command_timeout'
                     and self.authority.previous_mode == 'MANUAL' and self.authority.healthy)
        if finishing:
            p.command('twist', 0.0, 0.0, now, manual_stop=True)
            return
        active = mode in ('MANUAL', 'FOLLOW', 'NAVIGATION')
        if active and not p.armed and p.ready(now) == 'ready':
            p.arm(now)
        if not active or not p.armed:
            p.manual_stop_deadline = None
            self.clear_motion_output()
            return
        normal_stop = (mode == 'MANUAL' and self.authority.healthy and not self.authority.wait_stationary
                       and self.authority.received is not None and command.vx == 0.0 and command.wz == 0.0)
        p.command('twist', command.vx, command.wz, now, manual_stop=normal_stop)

    def on_push(self, request, response):
        """推行是显式锁定状态：停稳才能切换，进入/退出均撤销旧控制代次。"""
        with self.lock:
            now = time.monotonic()
            self.refresh_motion_health(now, manual_request=True)
            p = self.policy
            if request.data == self.push_mode:
                response.success, response.message = True, '推行模式已开启' if self.push_mode else '推行模式已关闭'
                return response
            if (not self.authority.healthy or self.authority.mode not in ('IDLE', 'MANUAL')
                    or p.last_rx is None or not 0 <= now - p.last_rx <= self.config.feedback_timeout_s
                    or p.stationary_frames < 5 or p.output != (0.0, 0.0)
                    or self.chassis is None):
                response.success, response.message = False, '请先停车，确认底盘正常且没有作业再切换推行模式'
                return response
            self.push_mode = bool(request.data)
            self.authority.stop()
            self.clear_motion_output()
            p.manual_stop_deadline = None
            response.success = True
            response.message = '正在放松底盘' if self.push_mode else '已退出推行模式，请重新解锁手动'
        self.publish_status()
        return response

    def load_tuning(self):
        """车端保存的驱动器参数；没有文件 = 都不下发。读不出来或底盘不支持时不下发，在状态里说明。"""
        if not self.tuning_path or not os.path.exists(self.tuning_path):
            return DriverTuning(), None
        try:
            with open(self.tuning_path, encoding='utf-8') as f:
                tuning = DriverTuning.parse(json.load(f), self.tuning_limit_a)
        except (OSError, ValueError) as exc:
            return DriverTuning(), '保存的驱动器参数读不出来，没有下发（重新设置会覆盖）：%s' % exc
        if not tuning.empty and not self.tuning_supported:
            return DriverTuning(), '保存了驱动器参数，但两台驱动器不是都接 CAN，没有下发'
        return tuning, None

    def save_tuning(self, tuning):
        """整份写临时文件再换名，断电不会留下半个文件。"""
        os.makedirs(os.path.dirname(self.tuning_path) or '.', exist_ok=True)
        tmp = self.tuning_path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(dict(tuning.as_dict(), saved_at=time.strftime('%Y-%m-%d %H:%M:%S')), f, ensure_ascii=False, indent=1)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self.tuning_path)

    def tune_blocker(self, now):
        if not self.tuning_supported:
            return '两台驱动器都接 CAN 并核对过（oid.yaml confirmed）才能改驱动器参数'
        if not self.tuning_path:
            return '车端没有设保存位置（oid.yaml 的 tuning_file），不能改驱动器参数'
        if self.tune_pending is not None:
            return '上一次修改还在下发，请稍后再试'
        p = self.policy
        if (self.chassis is None or not self.authority.healthy or self.authority.mode not in ('IDLE', 'MANUAL', 'ESTOP')
                or p.last_rx is None or not 0 <= now - p.last_rx <= self.config.feedback_timeout_s
                or p.stationary_frames < 5 or p.output != (0.0, 0.0)):
            return '请先停车，确认底盘正常且没有作业再改驱动器参数'
        return None

    def on_tune(self, message):
        """/oid/tune（网页服务转来的 App 指令）：{id, max_current_a, config_table, accel_mps2, decel_mps2}，
        null = 不下发 / 用默认。先存盘，再由 IO 线程发给两台驱动器；结果按 id 放进 /oid/status 的 tuning.result。"""
        try:
            cmd = json.loads(message.data)
            ident = str(cmd.get('id') or '')[:40]
        except (TypeError, ValueError, AttributeError):
            return
        if not ident:
            return
        with self.lock:
            problem = self.tune_blocker(time.monotonic())
            tuning = None
            if problem is None:
                try:
                    tuning = DriverTuning.parse(cmd, self.tuning_limit_a)
                except ValueError as exc:
                    problem = str(exc)
            if problem is None:
                try:
                    self.save_tuning(tuning)
                except OSError as exc:
                    problem = '保存失败，参数没有改：%s' % exc
            if problem is None:
                self.tuning, self.tuning_error = tuning, None
                self.tune_pending = ident
                self.tune_result = dict(id=ident, ok=None, message='已保存，正在发给驱动器')
            else:
                self.tune_result = dict(id=ident, ok=False, message=problem)
        self.publish_oid_status()

    def tuning_status(self):
        """给 App 的驱动器参数状态（/oid/status 的 tuning）。sent：这次连上驱动器以后已经按 values 发过。"""
        with self.lock:
            c = self.chassis
            active = self.tuning if self.tuning_supported else DriverTuning()
            sent = (c is not None and not self.config.receive_only and self.tune_pending is None
                    and getattr(c, 'tuning', None) == active)
            return dict(supported=self.tuning_supported, limit_a=self.tuning_limit_a, values=self.tuning.as_dict(),
                        defaults=dict(accel_mps2=self.wheels.accel_mps2, decel_mps2=self.wheels.decel_mps2),
                        sent=sent, reapplied=getattr(c, 'reapplied', 0) if c is not None else 0,
                        error=self.tuning_error, result=self.tune_result)

    def on_motion(self, source, message):
        with self.lock:
            if self.push_mode:
                return
            return super().on_motion(source, message)

    def select_motion(self, source, request, response):
        with self.lock:
            if self.push_mode and request.data:
                response.success, response.message = False, '推行模式中，请先退出推行模式'
                return response
            return super().select_motion(source, request, response)

    def reset_motion(self, request, response):
        with self.lock:
            if self.push_mode:
                response.success, response.message = False, '请先退出推行模式再复位'
                return response
            return super().reset_motion(request, response)

    def on_stop(self, request, response):
        with self.lock:
            self.push_mode = False
            # 急停重新发一次：车在走就先零速刹车、刹到轮速很小再放空，本来就停着的下一拍就放空；
            # 零速压不住（轮子还在动）或驱动器不回话时提前放空（zero_hold_release）。
            self.released = False
            return super().on_stop(request, response)

    def on_arm(self, request, response):
        with self.lock:
            if self.push_mode and request.data:
                response.success, response.message = False, '推行模式中，请先退出推行模式'
                return response
            if not request.data:
                self.push_mode = False
            return super().on_arm(request, response)

    def motion_extensions(self, now):
        # zero_hold_release_reason：这一段发零速里因为零速压不住（wheels_moving）或急停时驱动器不回话（driver_silent）
        # 已让驱动器放空，是哪一种（作业中压不住时仲裁另转 FAULT，原因 zero_hold_oscillation）；
        # estop_released / estop_release_reason：同上，只看急停。复位或重新取得控制后回到 false / null
        hold = self.zero_hold_released_reason()
        estop = hold if self.authority.mode == 'ESTOP' else None
        return {'push_mode': self.push_mode, 'push_released': self.push_mode and self.released,
                'auto_release': True, 'wheels_released': self.released,
                'zero_hold_release_reason': hold,
                'estop_released': estop is not None, 'estop_release_reason': estop}

    def zero_hold_released_reason(self):
        if self.released and self.zero_hold.epoch == self.authority.epoch:
            return self.zero_hold.reason
        return None

    def track_zero_hold(self, now, v, w):
        """本拍要发零速：新的一段（刚转成零速或换了仲裁代次）按当时的轮速和驱动器减速度记下正常刹车要多久；
        不是零速就清掉。在 policy.tick 之后、self._cmd 更新之前调：self._cmd 还是上一拍下发的车速、角速度。"""
        z, a = self.zero_hold, self.authority
        if v != 0.0 or w != 0.0:
            if z.epoch is not None:
                z.clear()
            return
        if z.epoch == a.epoch:
            return
        readback = [x if isinstance(x, (int, float)) and math.isfinite(x) else 0.0
                    for x in (self.policy.last_received or {}).get('wheels_mps') or []]
        directions = tuple(0 if abs(x) <= HALL_STILL_MAX_MPS else 1 if x > 0 else -1 for x in readback)
        pv, pw = self._cmd
        speed = max([abs(x) for x in readback] + [abs(pv) + abs(pw) * self.wheels.track_m / 2])
        wheels = self.chassis.wheels if self.chassis is not None else self.wheels     # App 改过减速度时用改过的
        z.start(a.epoch, now, speed, wheels.decel_mps2, directions if len(directions) == 2 else (0, 0))

    def drivers_unresponsive(self, now):
        """连上超过反馈超时以后，有驱动器不回话或两轮反馈过期（刚连上还没到超时不算）。"""
        p, c, timeout = self.policy, self.chassis, self.config.feedback_timeout_s
        if c is None or not now - p.connect_at > timeout:
            return False
        return bool(c.silent(now)) or p.last_rx is None or not 0 <= now - p.last_rx <= timeout

    def wheels_motion_at(self, telemetry, now, *, hall_only=False):
        """最近一次确实在动的时刻：读回轮速超过 HALL_STILL_MAX_MPS 就是这次读回（now），否则看霍尔/位置最后一次跨格
        （停着在格边抖不算）。hall_only：只认霍尔跨格（作业中用，读回转速在零速闭环里会跳）。"""
        if not hall_only and any(isinstance(x, (int, float)) and math.isfinite(x) and abs(x) > HALL_STILL_MAX_MPS
                                 for x in telemetry.get('wheels_mps') or []):
            return now
        return self.chassis.last_hall_motion(now)

    def wheels_slow(self, now):
        """刹到零：ZERO_HOLD_FRESH_S 内有反馈，两轮读回都不超过 HALL_STILL_MAX_MPS，或已经比开始发零速时反过来转了
        （驱动器零速闭环过零往回打：这时放空，反冲最小）。只在发零速时用（automatic_release_allowed）。"""
        p = self.policy
        if self.chassis is None or p.last_rx is None or not 0 <= now - p.last_rx <= ZERO_HOLD_FRESH_S:
            return False
        speeds = (p.last_received or {}).get('wheels_mps') or []
        if len(speeds) != 2 or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in speeds):
            return False
        z = self.zero_hold
        directions = z.directions if z.epoch == self.authority.epoch else (0, 0)
        return all((x * d if d else abs(x)) <= HALL_STILL_MAX_MPS for x, d in zip(speeds, directions))

    def zero_hold_release(self, now, v, w):
        """发零速时要不要因为零速压不住（轮子一直在动）或急停时驱动器不回话而放空：
        返回 (原因, 这一拍要不要发放空指令)；原因 None = 不走兜底（停车是否放空看 automatic_release_allowed）。"""
        z, mode = self.zero_hold, self.authority.mode
        if z.epoch != self.authority.epoch or v != 0.0 or w != 0.0:
            return None, False
        unresponsive = mode == 'ESTOP' and self.drivers_unresponsive(now)
        hold_s = ZERO_HOLD_JOB_FAULT_S if mode in ('NAVIGATION', 'FOLLOW') else ZERO_HOLD_MOVING_RELEASE_S
        reason = z.reason or ('driver_silent' if unresponsive else
                              'wheels_moving' if z.kept_moving(now, hold_s) else None)
        if reason is None:
            return None, False
        send = (z.reason is None or not self.released
                or ((unresponsive or z.moving(now)) and now - z.sent_at >= ZERO_HOLD_RELEASE_RESEND_S))
        return reason, send

    def automatic_release_allowed(self, now, v, w):
        """不在作业、没有手动运动请求、发的是零速时放空：开机还没有任何操作就直接放空，其余刹到轮速很小再放空；
        已放空后手推不能触发零速锁轮。"""
        if self.push_mode:  # 仅兼容旧客户端的显式推行协议，新 App 不再提供此入口。
            return True
        if self.authority.mode in ('NAVIGATION', 'FOLLOW'):
            return False
        if v != 0.0 or w != 0.0 or self.policy.manual_stopping(now):
            return False
        request = self.authority.request
        if self.authority.mode == 'MANUAL' and (request.vx != 0.0 or request.wz != 0.0):
            return False
        if self.authority.mode == 'IDLE' and self.authority.reason == 'startup':
            return True
        return self.released or self.wheels_slow(now)

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
                link = getattr(self.chassis, 'link', None)
                print('driver_io_failure type=%s error=%s can_tx_retries=%s can_tx_retry_timeouts=%s' %
                      (type(exc).__name__, exc, getattr(link, 'tx_retries', 0),
                       getattr(link, 'tx_retry_timeouts', 0)), file=sys.stderr, flush=True)
                traceback.print_exc(file=sys.stderr)
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
        opened_with = None
        if self.chassis is None:
            with self.lock:
                tuning = self.tuning if self.tuning_supported else None
            chassis = OidChassis(build_link(self.params), self.wheels, feedback_timeout_s=self.config.feedback_timeout_s,
                                 clock=time.monotonic, tuning=tuning, speed_trim=self.speed_trim)
            if send:
                chassis.open(now)
                opened_with = chassis.tuning
            with self.lock:
                self.chassis = chassis
                self.last_odom = None
                self.odometry_epoch = uuid.uuid4().hex
                self.rx_times.clear()
                self.policy.link(True, now)
                self.released = False
        with self.lock:
            pending, tuning = self.tune_pending, self.tuning
        if pending is not None:
            # App 改了驱动器参数：刚连上时已按新参数发过就不重发；发帧出错由 io_loop 重连，重连时按新参数发
            if opened_with != tuning:
                self.chassis.retune(tuning)
            with self.lock:
                if self.tune_pending == pending:
                    self.tune_pending = None
                    self.tune_result = dict(id=pending, ok=True, message='已发给两台驱动器')
        with self.lock:
            self.apply_motion(now)
            v, w = self.policy.tick(now)
            self.track_zero_hold(now, v, w)
            self._cmd = (v, w)
            hold_reason = self.zero_hold_release(now, v, w)[0] if send else None
            if (hold_reason is not None and self.zero_hold.reason is None
                    and self.authority.mode in ('NAVIGATION', 'FOLLOW')):
                # 作业中零速压不住：转故障（作业中断、断点保留），这一拍就放空，不再从头等一遍
                job = self.authority.mode
                self.authority.fault('zero_hold_oscillation', now)
                self.zero_hold.epoch = self.authority.epoch
                print('zero_hold_fault mode=%s' % job, file=sys.stderr, flush=True)
            idle = send and (hold_reason is not None or self.automatic_release_allowed(now, v, w))
            release_epoch = self.authority.epoch
        if idle:
            # 与急停/退出串行化这条放空命令，不能在控制代次已经变更后再执行旧的放空决定。
            with self.lock:
                # MANUAL 同一代次也可能收到新的非零请求，不能执行它到来前的放空决定。
                idle = release_epoch == self.authority.epoch
                hold_reason, hold_send = self.zero_hold_release(now, v, w) if idle else (None, False)
                idle = idle and (hold_reason is not None or self.automatic_release_allowed(now, v, w))
                if idle and (not self.released or hold_send):
                    self.chassis.release(now)
                    self.released = True
                    if hold_reason is not None:
                        if self.zero_hold.reason is None:
                            print('zero_hold_release reason=%s mode=%s' % (hold_reason, self.authority.mode),
                                  file=sys.stderr, flush=True)
                        self.zero_hold.reason, self.zero_hold.sent_at = hold_reason, now
        if idle:
            updated = self.chassis.step(now, 0.0, 0.0, send=False)     # 只读反馈，不发心跳和零速
        else:
            self.released = False
            updated = self.chassis.step(now, v, w, send=send)
        # 485 查询会阻塞；反馈和详细轮程使用读回时刻，不沿用开始查询之前的周期时间。
        now = max(now, self.chassis.updated or now)
        with self.lock:
            self.tx_packets = self.chassis.link.tx
            detail = getattr(self.chassis, 'wheel_motion', None)
            if detail is not None:
                self.pub_wheel_motion.publish(wd.String(data=json.dumps(
                    dict(detail, epoch=self.odometry_epoch), allow_nan=False)))
            if updated:
                telemetry = self.chassis.telemetry(now)
                self.policy.fault = ('、'.join('%s轮驱动器：%s' % kv for kv in telemetry['faults'].items())
                                     if self.chassis.faults() else None)
                self.policy.feedback(telemetry, now)
                job = self.authority.mode in ('NAVIGATION', 'FOLLOW')
                self.zero_hold.observe(self.wheels_motion_at(telemetry, now, hall_only=job))
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

    def publish_trace(self, t, now):
        """/oid/trace：t（单调时钟，与 /line/trace、/gnss/raw 同一时钟）、v_cmd / w_cmd（限速后发给底盘的）、
        erpm_sent / erpm_read（左、右）、erpm_target（低速修正前的目标）/ erpm_true（位置算的真实转速，修正打开才有）、wheels_mps（读回换算的左右轮速）、v / w（里程计）、
        pos_cdeg（左、右驱动器读回的电机轴位置，0.01°，累计值，驱动器方向；轮毂电机直驱即轮子转角，
        用来看车身摆动是否跟轮子每转一圈锁相）。负载（左、右，驱动器方向，读数超过 LOAD_MAX_AGE_S 没更新写 null）：
        motor_a 电机电流 A、bus_a 母线电流 A、duty 占空比原始值、temp_c 驱动器温度 ℃；CAN 那台只查电机电流，
        另外三项是 null。出错只丢这一条。"""
        try:
            def num(x):
                return round(x, 4) if isinstance(x, (int, float)) and math.isfinite(x) else None
            o, state = self.chassis.odom, self.chassis.link.state
            fresh = {m: state[m].t_load is not None and now - state[m].t_load <= LOAD_MAX_AGE_S for m in (1, 2)}

            def load(name):
                return [num(getattr(state[m], name)) if fresh[m] else None for m in (1, 2)]
            rec = {'t': round(now, 4), 'v_cmd': num(self._cmd[0]), 'w_cmd': num(self._cmd[1]),
                   'erpm_sent': [num(x) for x in t.get('erpm_sent') or []],
                   'erpm_target': [num(x) for x in t.get('erpm_target') or []],
                   'erpm_true': [num(x) for x in ((t.get('speed_trim') or {}).get('measured_erpm') or [])],
                   'erpm_read': [num(x) for x in t.get('erpm_read') or []],
                   'wheels_mps': [num(x) for x in t.get('wheels_mps') or []],
                   'v': num(o.v), 'w': num(o.w), 'stale': bool(t.get('stale')),
                   'pos_cdeg': [state[m].position_cdeg for m in (1, 2)],
                   'motor_a': load('motor_current_a'), 'bus_a': load('bus_current_a'),
                   'duty': load('duty'), 'temp_c': load('temp_c')}
            self.pub_trace.publish(wd.String(data=json.dumps(rec, allow_nan=False)))
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
        measurement=self.chassis.odometry_measurement
        if measurement:
            # 低速量化和回读降级的不确定度同步给 ROS 速度输出，不能只换数值不换可信度。
            odom.twist.covariance[0]=measurement['sigma_speed_mps']**2
            odom.twist.covariance[35]=measurement['sigma_yaw_rate_rps']**2
        for i in (14, 21, 28):
            odom.pose.covariance[i] = odom.twist.covariance[i] = 1e6
        self.pub_odom.publish(odom)
        self.publish_trace(t, now)
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
                        io_errors=self.io_errors, released=self.released, tuning=self.tuning_status())
            if c is not None:
                data.update(telemetry=c.telemetry(time.monotonic()), tx=c.link.tx, rx=c.link.rx,
                            can_tx_retries=getattr(c.link, 'tx_retries', 0),
                            can_tx_retry_timeouts=getattr(c.link, 'tx_retry_timeouts', 0),
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
