#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RTK 走直线 / 走路线节点：融合定位 + LineMission，经运动仲裁的 NAVIGATION 通道控制底盘。

    订阅  /gnss/raw（rtk_node）、/imu、/odom、
          /line/command（String JSON：mark/start/stop/clear/route_set/route_start/route_clear）
    发布  /navigation/command（经 MotionClient，带仲裁 epoch）、/line/status（5 Hz）

雷达防撞、指令超时、急停都在底盘驱动里，对 NAVIGATION 同样生效；本节点只在任务运行时发指令。
IMU 和轮速用消息头时间戳（驱动读到数据时打的）换算到本机单调时钟，GNSS 测量时刻由 rtk_node 换算。
"""
import json
import math
import os
import signal
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

import runtime_config  # noqa: F401  服务直接运行本脚本时把仓库根目录加进 sys.path（line_tracker 要用 robot_core）
from line_mission import LineMission
from motion_client import MotionClient

CONTROL_DT = 0.05
SELECT_RETRY_S = 0.5
# 已取消的固定基线参数（天线间距改由两根天线固定解的平面实时距离核对）；旧版 fusion.json 里可能还有，读取时丢掉
OBSOLETE_FUSION_KEYS = ('baseline_m', 'baseline_tol_m', 'baseline_check')
MAX_STAMP_AGE_S = 0.5       # 消息头时间戳比现在早超过这么多（或在未来），就不信它，改用收到时刻


class LineNode(Node):
    def __init__(self, mission, clock=time.monotonic):
        super().__init__('line_node')
        self.mission, self.clock = mission, clock
        self.motion = MotionClient(self, 'navigation', timeout_s=1.0)
        self.pub_status = self.create_publisher(String, '/line/status', 1)
        self.create_subscription(String, '/gnss/raw', self.on_gnss, 50)
        self.create_subscription(Imu, '/imu', self.on_imu, 20)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(String, '/line/command', self.on_command, 10)
        self.last_result = None
        self._select_at = None
        self.create_timer(CONTROL_DT, self.control)
        self.create_timer(0.2, self.publish_status)

    def on_gnss(self, msg):
        try:
            self.mission.on_gnss(json.loads(msg.data))
        except (ValueError, TypeError, KeyError) as exc:
            self.get_logger().warn('坏的 GNSS 消息：%s' % exc)

    def msg_time(self, msg):
        """消息头时间戳（底盘驱动读到数据时打的 ROS 时间）换算到本机单调时钟，去掉 DDS 传输和排队的延迟。
        没有时间戳或不可信时用收到时刻。"""
        now = self.clock()
        stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
        if stamp is None or not getattr(stamp, 'sec', 0):
            return now
        age = self.get_clock().now().nanoseconds / 1e9 - (stamp.sec + stamp.nanosec / 1e9)
        return now - age if 0.0 <= age <= MAX_STAMP_AGE_S else now

    def on_imu(self, msg):
        gz = msg.angular_velocity.z
        if math.isfinite(gz):
            self.mission.on_imu(self.msg_time(msg), gz)

    def on_odom(self, msg):
        v, wz = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        if math.isfinite(v) and math.isfinite(wz):
            self.mission.on_wheel(self.msg_time(msg), v, wz)

    def on_command(self, msg):
        try:
            cmd = json.loads(msg.data)
            if not isinstance(cmd, dict):
                raise ValueError('not an object')
        except ValueError:
            self.last_result = dict(ok=False, message='指令格式错误', id=None)
            return
        ok, message = self.mission.command(cmd, self.clock())
        self.last_result = dict(ok=ok, message=message, id=cmd.get('id'), cmd=cmd.get('cmd'))
        self.publish_status()

    def mode(self):
        return self.motion.state.get('mode', 'UNKNOWN') if self.motion.fresh() else 'UNKNOWN'

    def control(self):
        now = self.clock()
        vx, wz, select = self.mission.tick(now, self.mode())
        if select is True:
            if self._select_at is None or now - self._select_at > SELECT_RETRY_S:
                if self.motion.select(True) is not None:
                    self._select_at = now
        elif select is False:
            self._select_at = None
            self.motion.publish(0.0, 0.0)
            self.motion.select(False)
        if self.mission.phase in ('RUNNING', 'PAUSING') and self.motion.active():
            self.motion.publish(vx, wz)

    def publish_status(self):
        st = self.mission.status()
        st['motion_mode'] = self.mode()
        st['last_result'] = self.last_result
        self.pub_status.publish(String(data=json.dumps(st, allow_nan=False, ensure_ascii=False)))

    def stop_robot(self):
        self.mission.command({'cmd': 'stop'}, self.clock())
        self.motion.publish(0.0, 0.0)
        self.motion.select(False)


def load_fusion_config(gnss_fusion, path):
    """标定结果（安装偏差等）写在 path（~/rtk/fusion.json），键名同 FusionConfig 字段；没有文件就用默认值。
    已取消的固定基线参数（OBSOLETE_FUSION_KEYS）丢掉不用。"""
    overrides = {}
    if os.path.exists(path):
        with open(path, encoding='utf-8') as fh:
            overrides = {k: tuple(v) if isinstance(v, list) else v for k, v in json.load(fh).items()}
    for key in OBSOLETE_FUSION_KEYS:
        overrides.pop(key, None)
    return gnss_fusion.FusionConfig(**overrides)


def main():
    import gnss_fusion   # 融合模块（协作开发，接口见 docs/GNSS_FUSION.md）
    home = os.path.expanduser('~')
    fusion = gnss_fusion.GnssFusion(load_fusion_config(gnss_fusion, os.path.join(home, 'rtk', 'fusion.json')))
    mission = LineMission(fusion, gnss_fusion,
                          state_path=os.path.join(home, 'rtk', 'line_state.json'),
                          log_dir=os.path.join(home, 'rtk', 'runs'),
                          route_path=os.path.join(home, 'rtk', 'route.json'))
    rclpy.init()
    node = LineNode(mission)

    def on_signal(_sig=None, _frame=None):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
