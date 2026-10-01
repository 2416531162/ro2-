#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RTK 走直线 / 走路线节点：融合定位 + LineMission，经运动仲裁的 NAVIGATION 通道控制底盘。

    订阅  /gnss/raw（rtk_node）、/imu、/odom、
          /imu/data（算好的姿态：横滚、俯仰和 IMU 自己的偏航。旧车是 imu_filter_madgwick 算的，只给 app 显示；
          Gen5 是 HDA536T 传感器自己算的，它的航向角还交给融合推算航向（融合 yaw_input = 'angle'，这时 /imu 不进融合））、
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
from robot_core.service_watchdog import watchdog
from line_mission import LineMission
from motion_client import MotionClient

CONTROL_DT = 0.05
SELECT_RETRY_S = 0.5
# 已取消的固定基线参数（天线间距改由两根天线固定解的平面实时距离核对）；旧版 fusion.json 里可能还有，读取时丢掉
OBSOLETE_FUSION_KEYS = ('baseline_m', 'baseline_tol_m', 'baseline_check')
MAX_STAMP_AGE_S = 0.5       # 消息头时间戳比现在早超过这么多（或在未来），就不信它，改用收到时刻
ATTITUDE_MAX_AGE_S = 0.5    # /imu/data 超过这么久没来，就不报姿态（app 显示 —）
TILT_ATTACH_S = 0.1         # 旧车 /imu 陀螺帧带上这么久以内收到的 /imu/data 横滚俯仰，给融合做倾斜补偿


def attitude_deg(x, y, z, w):
    """四元数 -> (横滚, 俯仰, 偏航)，度。ROS REP-103 车体系 x 车头、y 左、z 上：
    横滚绕 x 轴，左侧抬起（右侧压低）为正；俯仰绕 y 轴，车头压低为正；偏航绕 z 轴，左转为正（−180…180）。
    imu_filter_madgwick 不用磁力计时偏航没有绝对参考：0 大致是它启动时的车头方向，随陀螺零偏漂移。"""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(n) or n < 1e-6:
        return None
    x, y, z, w = x / n, y / n, z / n, w / n
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class LineNode(Node):
    def __init__(self, mission, clock=time.monotonic):
        super().__init__('line_node')
        self.mission, self.clock = mission, clock
        self.motion = MotionClient(self, 'navigation', timeout_s=1.0)
        self.pub_status = self.create_publisher(String, '/line/status', 1)
        self.create_subscription(String, '/gnss/raw', self.on_gnss, 50)
        self.create_subscription(Imu, '/imu', self.on_imu, 20)
        self.create_subscription(Imu, '/imu/data', self.on_attitude, 10)
        self._attitude = None   # (收到时刻, 横滚°, 俯仰°, 偏航°)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(String, '/line/command', self.on_command, 10)
        # 融合用 IMU 航向角（Gen5）：从 /imu/data 取航向角和陀螺交给融合，/imu 不再进融合（同一帧不送两遍）
        config = getattr(getattr(mission, 'fusion', None), 'config', None)
        self.imu_yaw_input = getattr(config, 'yaw_input', 'rate') == 'angle'
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
        if self.imu_yaw_input:
            return
        gz = msg.angular_velocity.z
        if not math.isfinite(gz):
            return
        a = self._attitude
        if a is not None and 0.0 <= self.clock() - a[0] <= TILT_ATTACH_S:
            self.mission.on_imu(self.msg_time(msg), gz, roll=math.radians(a[1]), pitch=math.radians(a[2]))
        else:
            self.mission.on_imu(self.msg_time(msg), gz)

    def on_attitude(self, msg):
        """记下姿态给状态用；融合用 IMU 航向角时再把航向角、横滚俯仰（倾斜补偿）和陀螺交给融合。
        orientation_covariance[0] = -1 表示消息不带姿态（ROS 约定）：不显示，交给融合时也只给陀螺。"""
        try:
            has_orientation = float(msg.orientation_covariance[0]) >= 0
        except (TypeError, IndexError, ValueError, AttributeError):
            has_orientation = True
        att = None
        if has_orientation:
            o = msg.orientation
            att = attitude_deg(o.x, o.y, o.z, o.w)
            if att is not None:
                self._attitude = (self.clock(), *att)
        if self.imu_yaw_input:
            gz = msg.angular_velocity.z
            if math.isfinite(gz):
                if att is None:
                    self.mission.on_imu(self.msg_time(msg), gz, None)
                else:
                    self.mission.on_imu(self.msg_time(msg), gz, math.radians(att[2]),
                                        roll=math.radians(att[0]), pitch=math.radians(att[1]))

    def attitude_status(self):
        a = self._attitude
        if a is None or not 0.0 <= self.clock() - a[0] <= ATTITUDE_MAX_AGE_S:
            return None
        # imu 组件跟着底盘选：差速车（Gen5）是 HDA536T 驱动发的传感器姿态，旧车是 imu_filter_madgwick
        return {'roll_deg': round(a[1], 2), 'pitch_deg': round(a[2], 2), 'yaw_deg': round(a[3], 2),
                'source': 'hda536t' if getattr(self.mission, 'differential', False) else 'imu_madgwick'}

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
        st['attitude'] = self.attitude_status()
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
    # 差速车（原地转向）按共享底盘配置；标定文件里明确写了的照用
    from runtime_config import PROFILE
    overrides.setdefault('drive_differential', PROFILE['drive']['type'] == 'differential')
    if overrides['drive_differential']:
        # Gen5（HDA536T）：航向推算用 IMU 自己算的航向角（2026-09-28 用户要求）；原地转向转角大，
        # 按 3% 刻度误差加航向不确定度，测向按 20 ms 时间同步误差随转速降权。
        # 航向以双天线为主（同日用户要求「RTK 航向稳定就必须信它」）：航向过程噪声 0.1°/√s，
        # 测向约 0.25 s 就完全跟过去，IMU 只管两次测向之间（50 ms）和测向断掉的时候
        overrides.setdefault('yaw_input', 'angle')
        overrides.setdefault('yaw_scale_sigma', 0.03)
        overrides.setdefault('heading_sync_s', 0.02)
        overrides.setdefault('yaw_walk_deg_sqrt_s', 0.1)
        # 卫星位置不可信（双天线位置互检不过）或固定解中断（平板中转断流、浮点）时只靠推算走，最多 5 s 再停车
        # （2026-09-30 用户规则：IMU 航向 5 秒内肯定没问题；2026-10-01 用户：断流也按 IMU 维持 5 秒）
        overrides.setdefault('rtk_hold_s', 5.0)
    return gnss_fusion.FusionConfig(**overrides)


def main():
    import gnss_fusion   # 融合模块（协作开发，接口见 docs/GNSS_FUSION.md）
    home = os.path.expanduser('~')
    fusion = gnss_fusion.GnssFusion(load_fusion_config(gnss_fusion, os.path.join(home, 'rtk', 'fusion.json')))
    mission = LineMission(fusion, gnss_fusion,
                          state_path=os.path.join(home, 'rtk', 'line_state.json'),
                          log_dir=os.path.join(home, 'rtk', 'runs'),
                          route_path=os.path.join(home, 'rtk', 'route.json'))
    watchdog.start("ros")
    rclpy.init()
    node = LineNode(mission)
    node.create_timer(0.5, lambda: watchdog.tick("ros"))

    def on_signal(_sig=None, _frame=None):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        watchdog.close()
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
