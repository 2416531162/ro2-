#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""上车前的冒烟测试：用车上真的 rclpy 把 Python ROS 节点构造一遍、发一次状态、再销毁。

单元测试用的是 ROS 替身，测不出只有真 rclpy 才会暴露的问题：子类方法和 rclpy.Node 的属性同名
（RtkNode.handle 曾让节点一启动就崩）、服务直接运行脚本时 sys.path 里没有仓库根目录（line_node 曾找不到
robot_core）。这里按 systemd 直接运行 radar_system/xxx_node.py 的方式设置 sys.path；不打开串口、
不连 CORS、不 spin。由 smoke.sh 换到单独的 ROS 域，不和正在运行的服务互相看见。

    bash deployment/smoke.sh <发布目录> [rtk line imu oid jobrec web]  # activate 之前在车上跑，失败就不要激活

imu（Gen5 的 HDA536T 驱动）用假 CAN 总线喂一个周期，查真 sensor_msgs/Imu 能填好、发出去；oid（Gen5 底盘驱动）
只在共享配置是差速车时查，默认只收不发，没有 can0 也能构造、发状态。换到正点原子 RK3588（ROS 2 Jazzy）后第一次
上板，先跑这个。
"""
import os
import sys
import time
import traceback

RELEASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# 与 systemd 运行 radar_system/xxx_node.py 时一样：sys.path 里只有 radar_system，没有仓库根目录
sys.path.insert(0, os.path.join(RELEASE, 'radar_system'))


def check_rtk():
    import rtk_node
    node = rtk_node.RtkNode('/nonexistent/rtk-smoke-port', ntrip_env='/nonexistent/ntrip.env')
    try:
        time.sleep(0.3)
        node.publish_status()
        st = node.status()
        assert st['port_open'] is False and st['ntrip']['configured'] is False, st
    finally:
        node.shutdown()
        node.destroy_node()


def check_line():
    import json
    import tempfile
    import gnss_fusion
    import line_node
    from line_mission import LineMission
    from std_msgs.msg import String
    with tempfile.TemporaryDirectory() as tmp:
        # 车上存过路线（经纬度）：开机还没有定位原点时要能读回、能发状态，开始作业被拒绝
        route_path = os.path.join(tmp, 'route.json')
        with open(route_path, 'w', encoding='utf-8') as fh:
            json.dump({'version': 1, 'rev': 1, 'route': {'kind': 'circle', 'center': [30.0, 120.0]}}, fh)
        node = line_node.LineNode(LineMission(gnss_fusion.GnssFusion(), gnss_fusion, route_path=route_path))
        try:
            node.control()           # 没有定位、没有指令：不申请控制权、不发运动指令
            node.publish_status()    # 真的走一遍 JSON 化和发布
            node.on_command(String(data=json.dumps({'cmd': 'route_start', 'id': 'smoke'})))
            node.control()
            assert node.last_result['ok'] is False, node.last_result
            assert node.mission.phase == 'IDLE' and node.mission.route is not None
        finally:
            node.destroy_node()


def _script_dir(name):
    """与 systemd 直接运行 <目录>/xxx.py 一样：脚本所在目录在 sys.path 最前面。"""
    path = os.path.join(RELEASE, name)
    if path not in sys.path:
        sys.path.insert(0, path)


def check_imu():
    _script_dir('rion_protocol')
    import hda536t_codec as hc
    import hda536t_imu

    class Bus:
        """一个输出周期：角度、加速度、陀螺三帧（左转 10°/s，车头抬起 2°，静止水平）。"""
        def __init__(self):
            self.frames = [(0x585, hc.encode_pdo(hc.ANGLE, 1.0, 2.0, 3.0, 25)),
                           (0x585, hc.encode_pdo(hc.ACCEL, 0.0, 0.0, -1.0)),
                           (0x585, hc.encode_pdo(hc.GYRO, 0.0, 0.0, -10.0))]
            self.sent = []

        def recv(self):
            return self.frames.pop(0) if self.frames else None

        def send(self, can_id, data):
            self.sent.append((can_id, bytes(data)))

        def close(self):
            pass

    bus = Bus()
    node = hda536t_imu.Hda536tNode(bus=bus)
    try:
        raw, data = [], []
        for pub, out in ((node.pub_raw, raw), (node.pub_data, data)):
            real = pub.publish
            pub.publish = lambda msg, real=real, out=out: (out.append(msg), real(msg))
        while node.poll():
            pass
        assert len(raw) == 1 and len(data) == 1, (len(raw), len(data))
        assert abs(raw[0].angular_velocity.z - 0.17453) < 1e-3, raw[0].angular_velocity.z
        assert raw[0].orientation_covariance[0] == -1.0          # 融合用的原始 /imu 不带姿态
        assert abs(raw[0].linear_acceleration.z - 9.807) < 0.01, raw[0].linear_acceleration.z
        assert data[0].orientation_covariance[0] > 0 and abs(data[0].orientation.w - 1.0) > 1e-4
        node.publish_status()
        assert bus.sent == [], bus.sent                           # 从不往总线发
    finally:
        node.destroy_node()


def check_oid():
    _script_dir('oid_protocol')
    import oid_driver
    if oid_driver.PROFILE['drive']['type'] != 'differential':
        print('（共享配置不是差速车，跳过 oid）', flush=True)
        return
    node = oid_driver.OidDriver()
    try:
        assert node.config.receive_only, '冒烟测试不带参数文件，必须只收不发'
        time.sleep(0.5)
        node.publish_status()
        node.publish_oid_status()
    finally:
        node.shutdown()
        node.destroy_node()


def check_jobrec():
    """作业自动记录：真 rosbag2 写一段 mcap（开始 → 停下 → 跑完）再读回来；记录的话题类型都找得到、能建原始字节订阅。"""
    import json
    import shutil
    import tempfile
    import job_recorder
    import rosbag2_py
    from rclpy.node import Node
    from rclpy.serialization import serialize_message
    from std_msgs.msg import String
    root = tempfile.mkdtemp(prefix='jobrec-smoke-')
    try:
        clock = [0.0]
        rec = job_recorder.JobRecorder(root=root, files=lambda: [], clock=lambda: clock[0], log=lambda m: None)
        job = dict(id='smoke-0001', state='RUNNING', label='smoke')
        for t, phase, state in ((0, 'IDLE', None), (1, 'RUNNING', 'RUNNING'), (2, 'IDLE', 'DONE'), (8, 'IDLE', 'DONE')):
            clock[0] = t
            status = dict(phase=phase, task='route', operations=dict(job=dict(job, state=state) if state else None))
            rec.on_message('/line/status', 'std_msgs/msg/String', serialize_message(String(data=json.dumps(status))),
                           int((1790000000 + t) * 1e9), status)
        [name] = os.listdir(root)
        with open(os.path.join(root, name, 'job.json'), encoding='utf-8') as fh:
            info = json.load(fh)
        assert info['result']['state'] == 'DONE' and len(info['segments']) == 1, info
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=os.path.join(root, name, 'seg01'), storage_id='mcap'),
                    rosbag2_py.ConverterOptions('cdr', 'cdr'))
        read = []
        while reader.has_next():
            topic, data, _t = reader.read_next()
            read.append(topic)
        assert read.count('/line/status') == 4, read      # 开始前 1 条 + 开始 + 停下后 2 条
        node = Node('jobrec_smoke')
        try:
            job_recorder.attach(node, job_recorder.JobRecorder(root=root, files=lambda: [], log=lambda m: None))
        finally:
            node.destroy_node()
    finally:
        shutil.rmtree(root, ignore_errors=True)


def check_web():
    # 网页服务：构造 ROS 桥接节点，用它的状态快照编 hello / 遥测（不开 HTTP 端口、不发运动指令）
    os.environ['RK3588_NTRIP_ENV'] = '/nonexistent/ntrip.env'
    import json
    import radar_web_server as web
    import app_link
    node = web.WebBridgeNode()
    try:
        web.bridge_node = node
        snap = web.snapshot_state()
        hello = app_link.build_hello(web.PROFILE, name='smoke', session='smoke')
        tel = app_link.build_telemetry(snap, now=time.monotonic(), seq=1)
        json.dumps(hello, allow_nan=False)
        json.dumps(tel, allow_nan=False)
        assert 'follow' not in tel and 'camera' not in hello, (hello, tel)
    finally:
        web.bridge_node = None
        node.destroy_node()


CHECKS = {'rtk': check_rtk, 'line': check_line, 'imu': check_imu, 'oid': check_oid, 'jobrec': check_jobrec,
          'web': check_web}


def main(names=None):
    import rclpy
    rclpy.init()
    failed = []
    try:
        for name in names or list(CHECKS):
            try:
                CHECKS[name]()
                print('冒烟通过：%s' % name, flush=True)
            except Exception:
                failed.append(name)
                print('冒烟失败：%s' % name, flush=True)
                traceback.print_exc()
    finally:
        rclpy.shutdown()
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
