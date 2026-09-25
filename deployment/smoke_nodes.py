#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""上车前的冒烟测试：用车上真的 rclpy 把 Python ROS 节点构造一遍、发一次状态、再销毁。

单元测试用的是 ROS 替身，测不出只有真 rclpy 才会暴露的问题：子类方法和 rclpy.Node 的属性同名
（RtkNode.handle 曾让节点一启动就崩）、服务直接运行脚本时 sys.path 里没有仓库根目录（line_node 曾找不到
robot_core）。这里按 systemd 直接运行 radar_system/xxx_node.py 的方式设置 sys.path；不打开串口、
不连 CORS、不 spin。由 smoke.sh 换到单独的 ROS 域，不和正在运行的服务互相看见。

    bash deployment/smoke.sh <发布目录> [rtk line]      # activate 之前在车上跑，失败就不要激活
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


CHECKS = {'rtk': check_rtk, 'line': check_line}


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
