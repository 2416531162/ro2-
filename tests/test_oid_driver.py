#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""欧艾迪驱动器（Gen5 差速底盘：左右轮各一台单路 MINI-CAN，同一条 CAN）：编解码与厂家 CAN 编程手册的示例报文
逐字节一致；按手册行为写的两台模拟驱动器（各自心跳超时放空、速度环斜坡、位置累加、故障、断电）上跑整条链路：
轮速换算、左右与方向、里程计、原地转、故障停车、一台掉线停车。"""
import math
import os
import struct
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'oid_protocol'))
sys.path.insert(0, os.path.join(ROOT, 'wheeltec_protocol'))
sys.path.insert(0, ROOT)

import oid_codec as oc  # noqa: E402
from oid_driver import DiffConfig, DiffControlPolicy  # noqa: E402
from oid_link import CanLink, OidChassis, WheelConfig  # noqa: E402
from oid_sim import OidSimCar, SimOidBus  # noqa: E402

H = bytes.fromhex
POLES = 15
WHEELS = WheelConfig(pole_pairs=POLES)


def FakeCan(**kw):
    """同一条 CAN 上的两台单路驱动器：左轮节点 ID 74、右轮 75（oid_sim 里按手册行为写的模拟驱动器）。"""
    return SimOidBus(POLES, 74, 75, **kw)


def run(chassis, device, v, w, seconds, t0=0.0, dt=0.04, send=True):
    t = t0
    while t < t0 + seconds - 1e-9:
        chassis.step(t, v, w, send=send)
        device.advance(dt)
        t += dt
    return t


class TestCodecMatchesManual(unittest.TestCase):

    def test_can_frames(self):
        self.assertEqual(oc.can_speed(4000), H('02 00 00 0F A0'))
        self.assertEqual(oc.can_speed(-4000), H('02 FF FF F0 60'))
        self.assertEqual(oc.can_speed_accel(2000), H('0A 00 00 07 D0'))
        self.assertEqual(oc.can_speed_decel(800), H('10 00 00 03 20'))
        self.assertEqual(oc.can_current(-100), H('01 FF 9C'))
        self.assertEqual(oc.can_brake(100), H('08 00 64'))
        self.assertEqual(oc.can_heartbeat(), H('00'))
        self.assertEqual(oc.can_query(oc.Q_POSITION), H('0F 08'))

    def test_can_replies(self):
        self.assertEqual(oc.parse_can_reply(H('0F 01 00 00 13 88')), (oc.Q_SPEED, 5000))
        self.assertEqual(oc.parse_can_reply(H('0F 08 00 00 8C A0')), (oc.Q_POSITION, 36000))
        self.assertEqual(oc.parse_can_reply(H('0F 04 00 30')), (oc.Q_VOLTAGE, 48))
        self.assertEqual(oc.parse_can_reply(H('0F 00 00 00')), (oc.Q_FAULT, 0))
        self.assertEqual(oc.parse_can_reply(H('0F 09 03 E8')), (oc.Q_ANGLE, 1000))
        self.assertEqual(oc.parse_can_reply(H('0F 0A 01')), (oc.Q_Z, 1))
        self.assertIsNone(oc.parse_can_reply(H('02 00 00 0F A0')))

class TestWheels(unittest.TestCase):

    def test_speed_conversion(self):
        # 1 m/s、轮半径 0.2046 m：46.67 rpm，15 对极 700 erpm
        self.assertAlmostEqual(WHEELS.erpm_per_mps(), 60 / (2 * math.pi * 0.2046) * POLES, places=9)
        self.assertEqual(WHEELS.to_erpm(1.0), 700)
        self.assertAlmostEqual(WHEELS.to_mps(700), 1.0, places=3)
        self.assertAlmostEqual(WHEELS.centideg_to_m(36000), 2 * math.pi * 0.2046, places=9)

    def test_left_right_and_mirrored_motors(self):
        erpm = WHEELS.motor_erpm(0.0, 0.8)          # 原地左转：左轮往后、右轮往前
        left, right = WHEELS.wheels(0.0, 0.8)
        self.assertAlmostEqual(left, -0.27)
        self.assertAlmostEqual(right, 0.27)
        self.assertEqual(erpm, {1: WHEELS.to_erpm(-0.27), 2: -WHEELS.to_erpm(0.27)})   # 右边电机镜像安装
        flipped = WheelConfig(pole_pairs=POLES, left_sign=-1, right_sign=1)
        self.assertEqual(flipped.motor_erpm(0.5, 0.0), {1: -350, 2: 350})

    def test_wheel_limit_keeps_the_curve(self):
        left, right = WHEELS.wheels(1.2, 1.0)
        self.assertAlmostEqual(max(abs(left), abs(right)), 1.2)
        self.assertAlmostEqual((right - left) / 0.675 / ((left + right) / 2), 1.0 / 1.2)      # 曲率不变

    def test_unconfigured_pole_pairs_refuse_to_drive(self):
        with self.assertRaisesRegex(ValueError, 'pole_pairs'):
            OidChassis(CanLink(FakeCan(), (74, 75)), WheelConfig()).step(0.0, 0.3, 0.0)


class TestCan(unittest.TestCase):

    def make(self):
        device = FakeCan()
        return device, OidChassis(CanLink(device, (74, 75), heartbeat_s=0.1), WHEELS)

    def test_straight_run_odometry(self):
        device, chassis = self.make()
        chassis.open(0.0)
        t = run(chassis, device, 0.5, 0.0, 4.0)
        o = chassis.odom
        self.assertAlmostEqual(o.v, 0.5, delta=0.01)
        self.assertAlmostEqual(o.w, 0.0, delta=1e-3)
        self.assertAlmostEqual(o.y, 0.0, delta=1e-3)
        self.assertGreater(o.x, 1.5)
        self.assertFalse(chassis.telemetry(t)['stale'])
        self.assertAlmostEqual(chassis.telemetry(t)['voltage'], 48.0)
        # 两路电机方向相反（镜像安装）
        self.assertLess(device.motors[1].speed * device.motors[2].speed, 0)

    def test_pivot_turns_in_place(self):
        device, chassis = self.make()
        chassis.open(0.0)
        run(chassis, device, 0.0, 0.8, 3.0)
        o = chassis.odom
        self.assertAlmostEqual(o.v, 0.0, delta=0.01)
        self.assertAlmostEqual(o.w, 0.8, delta=0.02)          # 原地转时 /odom 也有角速度（融合要用）
        self.assertLess(math.hypot(o.x, o.y), 0.01)
        self.assertGreater(o.yaw, 1.5)

    def test_heartbeat_keeps_the_driver_alive_and_its_loss_frees_motors(self):
        device, chassis = self.make()
        chassis.open(0.0)
        run(chassis, device, 0.5, 0.0, 3.0)
        self.assertTrue(device.alive)
        self.assertGreater(abs(device.motors[1].speed), 300)
        for _ in range(60):                                     # 上位机不再发任何东西
            device.advance(0.04)
        self.assertFalse(device.alive)
        self.assertLess(abs(device.motors[1].speed), 1.0)

    def test_receive_only_sends_nothing_but_queries(self):
        device, chassis = self.make()
        run(chassis, device, 0.5, 0.0, 1.0, send=False)
        self.assertFalse(device.alive)
        self.assertEqual(device.motors[1].target, 0.0)
        self.assertTrue(all(data[0] == oc.CAN_QUERY for _, data in device.sent))

    def test_fault_is_reported(self):
        device, chassis = self.make()
        chassis.open(0.0)
        device.motors[2].fault = 16
        run(chassis, device, 0.3, 0.0, 1.0)
        self.assertEqual(chassis.faults(), {2: 16})
        self.assertEqual(chassis.telemetry(1.0)['faults'], {'右': '堵转'})

    def test_one_driver_powered_off_stops_the_updates(self):
        # 两台是独立的盒子：只有一台掉电或掉线时，不能拿另一台的反馈当整车反馈
        device, chassis = self.make()
        chassis.open(0.0)
        t = run(chassis, device, 0.3, 0.0, 2.0)
        device.motors[2].powered = False
        updates = []
        for _ in range(20):
            device.advance(0.04)
            t += 0.04
            updates.append(chassis.step(t, 0.3, 0.0))
        self.assertFalse(any(updates[10:]))                     # 0.3 s 后右轮反馈过期，不再更新
        telemetry = chassis.telemetry(t)
        self.assertEqual(telemetry['silent'], ['右'])
        self.assertTrue(telemetry['stale'])

    def test_speed_frames_go_to_each_node(self):
        device, chassis = self.make()
        chassis.open(0.0)
        device.sent.clear()
        chassis.step(1.0, 0.5, 0.0)
        speeds = [(i, d) for i, d in device.sent if d[0] == oc.CAN_SPEED]
        self.assertEqual(speeds, [(74, oc.can_speed(350)), (75, oc.can_speed(-350))])


class TestSimCar(unittest.TestCase):
    """不接硬件模拟 Gen5：下发的 (车速, 角速度) 经真的 CanLink 编成帧、两台模拟驱动器转出轮速，车按轮速走。"""

    def drive(self, car, v, w, seconds, t=0.0, dt=0.01):
        while seconds > 1e-9:
            car.command(t, v, w)
            car.step(t, dt)
            t += dt
            seconds -= dt
        return t

    def test_straight_and_pivot(self):
        car = OidSimCar()
        t = self.drive(car, 0.5, 0.0, 4.0)
        self.assertAlmostEqual(car.v, 0.5, delta=0.01)
        self.assertAlmostEqual(car.y, 0.0, delta=1e-6)
        t = self.drive(car, 0.0, 0.0, 1.0, t)                  # 先停稳（跟踪器原地转前也是这样）
        x = car.x
        self.drive(car, 0.0, 0.8, 3.0, t)
        self.assertAlmostEqual(car.x, x, delta=0.05)           # 原地转：后轴中点基本不动
        self.assertGreater(car.yaw, 1.5)
        self.assertEqual(car.steer, 0.0)
        self.assertTrue(car.bus.alive)


class TestPolicy(unittest.TestCase):
    """差速车的控制策略：原地转（车速 0、角速度不为 0）照常执行；故障停车、故障消失前不重新上电。"""

    def armed(self):
        cfg = DiffConfig(protocol='can', protocol_confirmed=True, receive_only=False, startup_stop_s=3.0)
        p = DiffControlPolicy(cfg, 0.0)
        p.link(True, 0.0)
        for i in range(6):
            p.feedback({'velocity': [0.0, 0.0, 0.0], 'voltage': 48.0}, 3.0 + 0.1 * i)
        ok, why = p.arm(3.6)
        self.assertTrue(ok, why)
        return p

    def test_pivot_command_is_executed_with_yaw_ramp(self):
        p = self.armed()
        p.command('twist', 0.0, 0.8, 3.6)
        v, w = p.tick(3.64)
        self.assertEqual(v, 0.0)
        self.assertAlmostEqual(w, 2.0 * 0.04)                 # 角加速度 2 rad/s²
        t = 3.64
        for _ in range(20):
            t += 0.04
            p.feedback({'velocity': [0.0, 0.0, w], 'voltage': 48.0}, t)
            p.command('twist', 0.0, 0.8, t)
            v, w = p.tick(t)
        self.assertAlmostEqual(w, 0.8)

    def test_limits_and_non_twist(self):
        p = self.armed()
        p.command('twist', 5.0, 9.0, 3.6)
        self.assertEqual(p.latest[1:], (p.config.max_speed_m_s, p.config.max_yaw_rate_rad_s))
        with self.assertRaises(ValueError):
            p.command('ackermann', 0.3, 0.1, 3.6)
        self.assertFalse(p.armed)

    def test_driver_fault_stops_and_blocks_rearm(self):
        p = self.armed()
        p.command('twist', 0.3, 0.0, 3.6)
        p.fault = '堵转'
        self.assertEqual(p.tick(3.64), (0.0, 0.0))
        self.assertFalse(p.armed)
        self.assertEqual(p.ready(3.7), 'driver_fault:堵转')
        p.fault = None
        self.assertEqual(p.ready(3.7), 'ready')

    def test_command_timeout_and_stale_feedback(self):
        p = self.armed()
        p.command('twist', 0.3, 0.0, 3.6)
        self.assertGreater(p.tick(3.64)[0], 0.0)
        self.assertEqual(p.tick(4.5), (0.0, 0.0))             # 指令 0.5 s 没更新
        self.assertEqual(p.reason, 'armed_waiting_command')


if __name__ == '__main__':
    unittest.main()


class TestRosNodeShape(unittest.TestCase):
    """驱动节点（OidDriver）在替身 ROS 上走一遍真实路径：上电等静止 → app 手动打横原地转 → 急停。

    **不是 ROS 集成测试**：只替换 Node 基类、消息容器和时钟的接口形状，验证节点自己的接线（继承来的仲裁、
    健康检查、状态页，加上 CAN 收发、里程计）。QoS、执行器和实车都不在覆盖范围内。
    """

    def setUp(self):
        import json
        from types import SimpleNamespace as NS
        from unittest import mock
        import oid_driver
        import wheeltec_driver as wd
        from robot_core.config import load_profile
        self.json, self.wd, self.od = json, wd, oid_driver
        self.clock = [100.0]
        self.device = FakeCan()
        overrides = dict(confirmed=True, pole_pairs=POLES, can_left_id=74, can_right_id=75, lidar_installed=False)
        published = {}

        class Pub:
            def __init__(self, topic):
                self.topic = topic

            def publish(self, msg):
                published.setdefault(self.topic, []).append(msg)

        clock = self.clock

        class FakeNode:
            def __init__(self, name):
                self._params = {}

            def declare_parameter(self, name, value, descriptor=None):
                self._params[name] = overrides.get(name, value)

            def get_parameter(self, name):
                return NS(value=self._params[name])

            def create_publisher(self, typ, topic, qos):
                return Pub(topic)

            def create_subscription(self, *a, **k):
                return None

            def create_service(self, *a, **k):
                return None

            def create_timer(self, *a, **k):
                return None

            def get_clock(self):
                return NS(now=lambda: NS(nanoseconds=int(clock[0] * 1e9), to_msg=lambda: NS(sec=0, nanosec=0)))

        def odometry():
            pose = NS(position=NS(x=0.0, y=0.0), orientation=NS(z=0.0, w=1.0))
            return NS(header=NS(stamp=None, frame_id=''), child_frame_id='',
                      pose=NS(pose=pose, covariance=[0.0] * 36),
                      twist=NS(twist=NS(linear=NS(x=0.0, y=0.0), angular=NS(z=0.0)), covariance=[0.0] * 36))

        box = lambda **k: NS(**k)  # noqa: E731
        stubs = dict(ROS_AVAILABLE=True, Node=FakeNode, ParameterDescriptor=box, Odometry=odometry,
                     Float32=box, String=box, QoSProfile=box, Duration=box,
                     HistoryPolicy=NS(KEEP_LAST=1), DurabilityPolicy=NS(VOLATILE=2), ReliabilityPolicy=NS(BEST_EFFORT=2),
                     SetBool=object, Trigger=object, LaserScan=object,
                     TransformStamped=lambda: NS(header=None, child_frame_id='', transform=NS(
                         translation=NS(x=0.0, y=0.0), rotation=None)),
                     TransformBroadcaster=lambda node: NS(sendTransform=lambda tf: None))
        patches = [mock.patch.multiple(wd, create=True, **stubs),
                   mock.patch.object(oid_driver, 'PROFILE', load_profile(os.path.join(ROOT, 'robot_core', 'robot_gen5.json'))),
                   mock.patch.object(oid_driver.threading, 'Thread', lambda **k: NS(start=lambda: None, join=lambda timeout: None)),
                   mock.patch.object(oid_driver, 'build_link',
                                     lambda p: CanLink(self.device, (p['can_left_id'], p['can_right_id']), None, heartbeat_s=p['heartbeat_s'])),
                   mock.patch('time.monotonic', lambda: clock[0]),
                   # 没装 ROS 时 WheeltecDriver 的基类是 object：把替身 Node 的方法挂到节点类上
                   mock.patch.multiple(oid_driver.OidDriver, create=True, **{
                       k: v for k, v in vars(FakeNode).items() if callable(v) and not k.startswith('__')})]
        for p in patches:
            p.start()
            self.addCleanup(p.stop)
        self.node = oid_driver.OidDriver()
        self.published = published

    def step(self, seconds, command=None):
        end = self.clock[0] + seconds
        while self.clock[0] < end - 1e-9:
            if command is not None:
                data = dict(vx=command[0], wz=command[1], epoch=self.node.authority.epoch,
                            profile_hash=self.wd.profile_hash(self.wd.PROFILE), stamp=self.clock[0] - 0.01)
                self.node.on_motion('manual', self.wd.String(data=self.json.dumps(data)))
            self.node._io_step(self.clock[0])
            self.device.advance(0.04)
            self.clock[0] += 0.04

    def test_manual_pivot_then_emergency_stop(self):
        node = self.node
        self.step(4.0)                                          # 启动静止 3 s，等反馈
        self.assertTrue(node.policy.armed, node.policy.reason)
        self.step(3.0, command=(0.0, 0.5))                      # app 摇杆打横：原地左转
        self.assertEqual(node.authority.mode, 'MANUAL')
        odom = self.published['/odom'][-1]
        self.assertAlmostEqual(odom.twist.twist.linear.x, 0.0, delta=0.01)
        self.assertAlmostEqual(odom.twist.twist.angular.z, 0.5, delta=0.03)
        self.assertGreater(self.device.motors[1].speed * self.device.motors[2].speed, 0)   # 镜像安装：同号就是一前一后
        node.on_stop(None, NS_response())
        self.assertEqual(node.authority.mode, 'ESTOP')
        self.step(1.0)
        self.assertLess(max(abs(m.speed) for m in self.device.motors.values()), 1.0)
        node.publish_status()                                   # 继承来的状态页照样能出
        node.publish_oid_status()
        status = self.json.loads(self.published['/wheeltec/status'][-1].data)
        self.assertEqual((status['connected'], status['motion']['mode']), (True, 'ESTOP'))
        oid = self.json.loads(self.published['/oid/status'][-1].data)
        self.assertEqual((oid['bus'], oid['channel'], oid['wheels']['pole_pairs']), ('can', 'can0', POLES))

    def test_motors_are_released_when_parked_and_hold_again_on_command(self):
        """停稳又没指令：放空电机、不再发零速（纯霍尔锁零速会咔哒响）；一推摇杆马上回到速度闭环。"""
        node, dev = self.node, self.device
        self.step(5.0)                                          # 上电静止 3 s + 停稳 1 s
        self.assertTrue(node.released)
        self.assertFalse(any(m.speed_mode for m in dev.motors.values()))
        sent = len(dev.sent)
        self.step(1.0)
        self.assertFalse([d for _, d in dev.sent[sent:] if d[0] in (0x00, 0x02)])   # 没有心跳、没有转速帧
        self.step(1.0, command=(0.3, 0.0))
        self.assertFalse(node.released)
        self.assertTrue(all(m.speed_mode for m in dev.motors.values()))
        self.assertGreater(min(abs(m.speed) for m in dev.motors.values()), 50)
        self.step(0.5, command=(0.0, 0.0))                      # 回中：刹停，还不放空
        self.assertFalse(node.released)
        self.step(2.5)                                          # 松手：超时回空闲，停稳 1 s 后放空
        self.assertTrue(node.released)
        self.assertLess(max(abs(m.speed) for m in dev.motors.values()), 1.0)

    def test_driver_fault_goes_to_fault_and_needs_reset(self):
        self.step(4.0)
        self.step(1.0, command=(0.3, 0.0))
        self.device.motors[1].fault = 16                        # 左轮堵转
        self.step(2.0, command=(0.3, 0.0))
        self.assertFalse(self.node.policy.armed)
        self.assertEqual(self.node.authority.mode, 'FAULT')
        self.device.motors[1].fault = 0
        self.step(1.0, command=(0.3, 0.0))
        self.assertEqual(self.node.authority.mode, 'FAULT')     # 故障消失也不自己接着开

    def test_one_driver_offline_while_driving_is_a_fault(self):
        self.step(4.0)
        self.step(1.0, command=(0.3, 0.0))
        self.assertEqual(self.node.authority.mode, 'MANUAL')
        self.device.motors[2].powered = False                   # 右轮驱动器断电
        self.step(2.5, command=(0.3, 0.0))
        self.assertFalse(self.node.policy.armed)
        self.assertEqual(self.node.authority.mode, 'FAULT')
        self.assertIn('driver_silent:右轮', self.node.motion_health_faults)
        self.assertLess(abs(self.device.motors[1].speed), 1.0)  # 左轮也停了


def NS_response():
    from types import SimpleNamespace
    return SimpleNamespace(success=None, message='')
