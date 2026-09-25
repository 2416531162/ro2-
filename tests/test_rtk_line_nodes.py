#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RTK 节点与走直线节点的 ROS 外壳测试（ROS 替身，不是集成测试）。"""
import json
import math
import os
import sys
import tempfile
import threading
import time
import types
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, ROOT)

import ros_stubs  # noqa: E402

ros_stubs.install()
sys.modules['sensor_msgs.msg'].Imu = ros_stubs._simple('Imu')

import rtk_node  # noqa: E402
import line_node  # noqa: E402
from line_mission import LineMission  # noqa: E402
from test_um982 import BESTPOS, HEADING, BESTVEL, GGA, epoch_lines  # noqa: E402
from test_line_mission import FakeFusion, SAMPLES  # noqa: E402


class FakeSerial:
    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.written = []
        self.lock = threading.Lock()

    def read(self, n):
        with self.lock:
            if self.chunks:
                return self.chunks.pop(0)
        time.sleep(0.01)
        return b''

    def write(self, data):
        self.written.append(data)

    def close(self):
        pass


class TestEntrypoints(unittest.TestCase):

    def test_import_like_the_systemd_service(self):
        # 服务是直接运行 radar_system/xxx_node.py：sys.path 里只有 radar_system，没有仓库根目录。
        # 本文件的其他测试把根目录加进了 sys.path，测不出"robot_core 找不到"这种上车才崩的问题。
        import subprocess
        radar = os.path.join(ROOT, 'radar_system')
        code = ('import sys; sys.path.append(%r); import ros_stubs; ros_stubs.install(); '
                'sys.modules["sensor_msgs.msg"].Imu = ros_stubs._simple("Imu"); '
                'import rtk_node, line_node; print("ok")' % os.path.join(ROOT, 'tests'))
        env = {k: v for k, v in os.environ.items() if k != 'PYTHONPATH'}
        r = subprocess.run([sys.executable, '-c', code], cwd=radar, env=env, capture_output=True, text=True,
                           timeout=60)
        self.assertEqual((r.returncode, r.stdout.strip()), (0, 'ok'), r.stderr[-800:])

    def test_nodes_do_not_shadow_rclpy_node(self):
        # ROS 替身的 Node 很简单，子类方法和真 rclpy.Node 同名测不出来（RtkNode.handle 曾让车上的节点一启动就崩）。
        # 名单取自车上 ROS 2 Humble 的 dir(rclpy.node.Node)。
        rclpy_node = set((
            'add_on_set_parameters_callback add_waitable clients context count_publishers count_subscribers '
            'create_client create_guard_condition create_publisher create_rate create_service create_subscription '
            'create_timer declare_parameter declare_parameters default_callback_group describe_parameter '
            'describe_parameters destroy_client destroy_guard_condition destroy_node destroy_publisher destroy_rate '
            'destroy_service destroy_subscription destroy_timer executor get_client_names_and_types_by_node '
            'get_clock get_fully_qualified_name get_logger get_name get_namespace get_node_names '
            'get_node_names_and_namespaces get_node_names_and_namespaces_with_enclaves get_parameter '
            'get_parameter_or get_parameter_type get_parameter_types get_parameters get_parameters_by_prefix '
            'get_publisher_names_and_types_by_node get_publishers_info_by_topic get_service_names_and_types '
            'get_service_names_and_types_by_node get_subscriber_names_and_types_by_node '
            'get_subscriptions_info_by_topic get_topic_names_and_types guards handle has_parameter publishers '
            'remove_on_set_parameters_callback remove_waitable resolve_service_name resolve_topic_name services '
            'set_descriptor set_parameters set_parameters_atomically subscriptions timers undeclare_parameter '
            'waitables').split())
        overridable = {'destroy_node'}
        for cls in (rtk_node.RtkNode, line_node.LineNode):
            clash = (set(vars(cls)) & rclpy_node) - overridable
            self.assertEqual(clash, set(), cls.__name__)
        # 构造后设的实例属性也不能同名（比如 self.handle = …）
        with tempfile.TemporaryDirectory() as tmp:
            node = rtk_node.RtkNode('/dev/null', ntrip_env=os.path.join(tmp, 'none'),
                                    serial_factory=lambda: FakeSerial([]))
            try:
                own = set(vars(node)) - set(vars(ros_stubs.Node()))
                self.assertEqual(own & rclpy_node, set())
            finally:
                node.shutdown()
        own = set(vars(line_node.LineNode(LineMission(FakeFusion(), SAMPLES)))) - set(vars(ros_stubs.Node()))
        self.assertEqual(own & rclpy_node, set())

    def test_deploy_smoke_script(self):
        # deployment/smoke_nodes.py 在车上用真 rclpy 跑；这里用替身确认脚本本身能跑通、坏节点会报失败
        import importlib.util
        spec = importlib.util.spec_from_file_location('smoke_nodes', os.path.join(ROOT, 'deployment', 'smoke_nodes.py'))
        smoke = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(smoke)
        self.assertEqual(smoke.main(['rtk', 'line']), 0)
        smoke.CHECKS['broken'] = lambda: 1 / 0
        self.assertEqual(smoke.main(['broken']), 1)


class TestRtkNode(unittest.TestCase):

    def test_publishes_measurements_and_status(self):
        # 6 个历元的主天线 + 副天线（相距 0.3312 m）+ 测向，测向才有实时基线可核对
        data = ('\r\n'.join([GGA] + epoch_lines(6) + [BESTVEL]) + '\r\n').encode()
        fake = FakeSerial([data[:50], data[50:]])
        node = rtk_node.RtkNode('/dev/null', ntrip_env='/nonexistent/ntrip.env',
                                serial_factory=lambda: fake)
        try:
            raw = node.publishers_['/gnss/raw'].sent
            deadline = time.time() + 2
            while len(raw) < 19 and time.time() < deadline:
                time.sleep(0.01)
            kinds = [json.loads(m.data)['kind'] for m in raw]
            self.assertEqual(kinds, ['fix', 'fix2', 'heading'] * 6 + ['vel'])
            self.assertTrue(json.loads(raw[-2].data)['valid'])
            node.publish_status()
            st = json.loads(node.publishers_['/gnss/status'].sent[-1].data)
            self.assertEqual((st['rtk'], st['heading_ok'], st['port_open']), ('FIXED', True, True))
            self.assertEqual(st['ntrip'], dict(configured=False, connected=False, error='not_configured',
                                              config_revision=None, config_error='not_configured'))
            node.write_corrections(b'\xd3\x00')
            self.assertEqual(fake.written, [b'BESTNAVHA 0.05\r\n', b'\xd3\x00'])
            self.assertIsNotNone(node.gga_report())
            # ROS 2 标准消息：NavSatFix（RTK 固定 → GBAS_FIX，协方差 ENU 对角）
            fix = node.publishers_['/gnss/fix'].sent[-1]
            self.assertAlmostEqual(fix.latitude, 30.00000012345)
            self.assertEqual((fix.status.status, fix.header.frame_id, fix.position_covariance_type),
                             (rtk_node.GBAS_FIX, 'gnss', rtk_node.DIAGONAL_KNOWN))
            self.assertAlmostEqual(fix.position_covariance[0], 0.0092 ** 2)       # 东向 = sigma_lon
            # 航向 Imu：HEADING 91.25° → 车头 ENU 偏航 = 90 − 91.25 + 90 = 88.75°
            imu = node.publishers_['/gnss/heading'].sent[-1]
            yaw = math.degrees(2 * math.atan2(imu.orientation.z, imu.orientation.w))
            self.assertAlmostEqual(yaw, 88.75, places=6)
            self.assertAlmostEqual(imu.orientation_covariance[8], math.radians(0.28) ** 2)
            self.assertEqual(imu.angular_velocity_covariance[0], -1.0)
        finally:
            node.shutdown()

    def test_heading_uses_fusion_calibration_and_skips_invalid(self):
        m = dict(kind='heading', t=1.0, heading=91.25, pitch=0.0, length=0.3312, sol='L1_INT', sigma_heading=0.28)
        yaw = lambda msg: math.degrees(2 * math.atan2(msg.orientation.z, msg.orientation.w))  # noqa: E731
        self.assertAlmostEqual(yaw(rtk_node.heading_imu(m, None, mount_offset_deg=1.5)), 90.25, places=6)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, 'fusion.json')
            with open(path, 'w') as fh:
                json.dump({'mount_offset_deg': 1.5}, fh)
            self.assertEqual(rtk_node.load_heading_calibration(path), (1.5, -90.0))
        self.assertEqual(rtk_node.load_heading_calibration('/nonexistent/fusion.json'), (0.0, -90.0))
        # 是否有效由 Um982Stream 按两天线实时距离判定（valid），节点只看结论；没有这个字段按无效处理
        self.assertTrue(rtk_node.heading_valid(dict(m, valid=True)))
        self.assertFalse(rtk_node.heading_valid(dict(m, valid=False)))
        self.assertFalse(rtk_node.heading_valid(m))
        self.assertEqual([rtk_node.navsat_status(s) for s in ('NARROW_INT', 'L1_FLOAT', 'PSRDIFF', 'SINGLE', 'NONE')],
                         [rtk_node.GBAS_FIX, rtk_node.GBAS_FIX, rtk_node.SBAS_FIX, rtk_node.FIX, rtk_node.NO_FIX])

    def test_old_fusion_json_with_cancelled_baseline_keys_still_loads(self):
        import gnss_fusion
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, 'fusion.json')
            with open(path, 'w') as fh:
                json.dump({'mount_offset_deg': 1.5, 'antenna_xy': [0.3, 0.1], 'baseline_m': 0.33,
                           'baseline_tol_m': 0.03, 'baseline_check': True}, fh)
            cfg = line_node.load_fusion_config(gnss_fusion, path)
        self.assertEqual((cfg.mount_offset_deg, cfg.antenna_xy), (1.5, (0.3, 0.1)))
        self.assertFalse(hasattr(cfg, 'baseline_m'))
        self.assertEqual(line_node.load_fusion_config(gnss_fusion, '/nonexistent/fusion.json'), gnss_fusion.FusionConfig())

    def test_port_error_is_reported_and_retried(self):
        calls = []

        def broken():
            calls.append(1)
            raise OSError('no such device')

        node = rtk_node.RtkNode('/dev/null', ntrip_env='/nonexistent', serial_factory=broken)
        try:
            time.sleep(0.05)
            st = node.status()
            self.assertFalse(st['port_open'])
            self.assertIn('no such device', st['port_error'])
            self.assertFalse(st['receiver_ok'])
        finally:
            node.shutdown()


class FakeMotion:
    def __init__(self):
        self.mode, self.selects, self.published = 'IDLE', [], []

    def fresh(self):
        return True

    def active(self):
        return self.mode == 'NAVIGATION'

    @property
    def state(self):
        return {'mode': self.mode}

    def select(self, enabled=True):
        self.selects.append(enabled)
        return object()

    def publish(self, vx, wz):
        self.published.append((vx, wz))
        return True


class TestLineNode(unittest.TestCase):

    def setUp(self):
        self.fusion = FakeFusion()
        self.t = [100.0]
        self.node = line_node.LineNode(LineMission(self.fusion, SAMPLES), clock=lambda: self.t[0])
        self.motion = self.node.motion = FakeMotion()

    def cmd(self, **kw):
        self.node.on_command(ros_stubs.String(data=json.dumps(kw)))
        return self.node.last_result

    def test_sensor_callbacks_feed_fusion(self):
        self.node.on_gnss(ros_stubs.String(data=json.dumps(dict(
            kind='vel', t=1.0, speed=0.5, track=1.0, sol='DOPPLER_VELOCITY'))))
        imu = sys.modules['sensor_msgs.msg'].Imu()
        imu.angular_velocity.z = 0.01
        self.node.on_imu(imu)
        odom = sys.modules['nav_msgs.msg'].Odometry()
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = 0.4, 0.0
        self.node.on_odom(odom)
        self.node.on_gnss(ros_stubs.String(data='not json'))
        self.assertEqual([n for n, _ in self.fusion.inputs], ['add_velocity', 'add_imu', 'add_wheel'])
        self.assertEqual(self.fusion.inputs[1][1].t, 100.0)

    def test_start_selects_navigation_then_drives_and_releases_on_stop(self):
        self.fusion.current = FakeFusion.make(0, 0, 0)
        self.assertTrue(self.cmd(cmd='mark', point='A')['ok'])
        self.fusion.current = FakeFusion.make(10, 0, 0)
        self.assertTrue(self.cmd(cmd='mark', point='B')['ok'])
        self.fusion.current = FakeFusion.make(-0.5, 0, 0)
        res = self.cmd(cmd='start', cruise=0.4, id=7)
        self.assertEqual((res['ok'], res['id']), (True, 7))
        for _ in range(5):                       # 申请控制权：节流，不是每个周期都调服务
            self.t[0] += 0.05
            self.node.control()
        self.assertEqual(self.motion.selects, [True])
        self.motion.mode = 'NAVIGATION'
        for _ in range(10):
            self.t[0] += 0.05
            self.node.control()
        self.assertEqual(self.node.mission.phase, 'RUNNING')
        self.assertGreater(self.motion.published[-1][0], 0.0)
        self.cmd(cmd='stop')
        self.t[0] += 0.05
        self.node.control()
        self.assertEqual(self.motion.selects[-1], False)
        self.assertEqual(self.motion.published[-1], (0.0, 0.0))
        status = json.loads(self.node.pub_status.sent[-1].data)
        self.assertEqual((status['phase'], status['motion_mode']), ('IDLE', 'NAVIGATION'))

    def test_sensor_time_uses_header_stamp(self):
        # 替身的 ROS 时钟固定在 1000.0 s；本节点的单调时钟是 self.t[0] = 100.0
        def imu(sec, nanosec):
            msg = sys.modules['sensor_msgs.msg'].Imu()
            msg.header.stamp = types.SimpleNamespace(sec=sec, nanosec=nanosec)
            return msg
        self.assertAlmostEqual(self.node.msg_time(imu(999, 970000000)), 100.0 - 0.03)   # 30 ms 前读到的
        self.assertEqual(self.node.msg_time(imu(0, 0)), 100.0)                           # 没打时间戳
        self.assertEqual(self.node.msg_time(imu(998, 0)), 100.0)                         # 太旧，不信
        self.assertEqual(self.node.msg_time(imu(1001, 0)), 100.0)                        # 在未来，不信
        self.assertEqual(self.node.msg_time(types.SimpleNamespace()), 100.0)             # 没有消息头

    def test_bad_command_reported(self):
        self.node.on_command(ros_stubs.String(data='[1,2]'))
        self.assertEqual(self.node.last_result['ok'], False)
        self.assertEqual(self.cmd(cmd='fly')['message'], '未知指令')


if __name__ == '__main__':
    unittest.main()
