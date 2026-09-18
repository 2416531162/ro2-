#!/usr/bin/env python3
"""ROS transport adapter for the camera/lidar follower engine."""
import argparse
import json
import math
import signal
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32
from runtime_config import PROFILE
from robot_core.contracts import ScanFrame, LocalPose
from follower_config import FollowerConfig as FollowerConfig, build_config
from follower_engine import FollowerEngine
from motion_client import MotionClient

__all__ = ['PersonFollowerNode', 'FollowerConfig', 'build_config']


class PersonFollowerNode(Node):
    def __init__(self, config, dry_run=False, target_class='person', passive=False,
                 simulated_odometry=False):
        super().__init__('person_follower_node')
        self.pub_status = self.create_publisher(String, '/follower/status', 10)
        self.engine = FollowerEngine(config, now=lambda: time.monotonic(),
            ros_time=lambda: self.get_clock().now().nanoseconds / 1e9,
            emit_status=lambda data: self.pub_status.publish(String(data=json.dumps(data, ensure_ascii=False))),
            dry_run=dry_run, target_class=target_class, simulated_odometry=simulated_odometry)
        self.motion = MotionClient(self, 'follow')
        self._select_pending = not (dry_run or passive)
        self._motion_epoch = None
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(String, '/camera/ai_detection/targets', self.on_targets, 1)
        self.create_subscription(LaserScan, '/scan', self.on_scan, sensor_qos)
        self.create_subscription(Float32, '/voltage', self.on_voltage, 1)
        self.create_subscription(String, '/wheeltec/status', self.on_driver_status, 1)
        self.create_subscription(Odometry, PROFILE['localization']['topic'], self.on_odom, sensor_qos)
        self.timer = self.create_timer(self.engine.dt, self.control_loop)
        self.get_logger().info('Follower ready; local odometry: ' +
                               ('simulation only' if simulated_odometry else PROFILE['localization']['topic']))

    # Compatibility for existing diagnostics/replay. New code uses node.engine.
    def __getattr__(self, name):
        engine = self.__dict__.get('engine')
        if engine is not None and hasattr(engine, name):
            return getattr(engine, name)
        raise AttributeError(name)

    def __setattr__(self, name, value):
        engine = self.__dict__.get('engine')
        if engine is not None and name in engine.__dict__:
            setattr(engine, name, value)
        elif engine is not None and name == 'print_dashboard':
            engine.print_dashboard = value
        else:
            super().__setattr__(name, value)

    def on_targets(self, msg):
        try:
            self.engine.observe_targets(json.loads(msg.data))
        except (TypeError, ValueError, OverflowError):
            self.engine.stamp_warnings += 1

    def on_driver_status(self, msg):
        try:
            self.engine.observe_driver(json.loads(msg.data))
        except (TypeError, ValueError):
            self.engine.feedback_healthy = False

    def on_scan(self, msg):
        header = getattr(msg, 'header', None)
        stamp = getattr(header, 'stamp', None)
        seconds = stamp.sec + stamp.nanosec / 1e9 if stamp else 0.
        self.engine.observe_scan(ScanFrame(msg.ranges, msg.angle_min, msg.angle_increment,
                                          msg.range_min, msg.range_max, seconds))

    def on_voltage(self, msg):
        self.engine.observe_voltage(msg.data)

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if not math.isfinite(norm) or abs(norm-1.0) > .05:
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.engine.observe_pose(LocalPose(msg.header.stamp.sec + msg.header.stamp.nanosec/1e9,
            msg.pose.pose.position.x, msg.pose.pose.position.y, yaw,
            msg.header.frame_id, msg.child_frame_id))

    def control_loop(self):
        core = self.engine
        if not core.dry_run:
            if self._select_pending and self.motion.fresh():
                if self.motion.select(True) is not None:
                    self._select_pending = False
            if not self.motion.active():
                core.pause()
                return
            epoch = self.motion.state.get('epoch')
            if epoch != self._motion_epoch:
                self._motion_epoch = epoch
                core.reset_tracking()
        core.step()
        if not core.dry_run:
            self.motion.publish(core.cmd_vx, core.cmd_wz)

    def stop_robot(self):
        if getattr(self, '_stopped', False):
            return
        self._stopped = True
        if not self.engine.dry_run:
            self.motion.publish(0.0, 0.0)
            self.motion.select(False)


def strip_ros_args(argv):
    """去掉 ros2 run 追加的 --ros-args 段,其余参数必须全部可识别。

    旧版 parse_known_args 会静默吞掉写错的参数(如 --max-speed 0.3),
    限速没生效却照常启动,比直接报错危险得多。
    """
    argv = list(argv)
    return argv[:argv.index('--ros-args')] if '--ros-args' in argv else argv


def main():
    p = argparse.ArgumentParser(description="RK3588 电子跟屁虫 - 人体跟随控制节点")
    p.add_argument('--passive', action='store_true', help='等待 /motion/follow 显式选择跟随模式')
    p.add_argument('--no-recovery', action='store_true', help='关闭自动倒车脱困和丢人搜索')
    p.add_argument('--no-turnaround', action='store_true',
                   help='禁用车后目标前向大舵角弧线掉头 (默认开启掉头调转车头面朝人体)')
    p.add_argument('--rear-blind', action='store_true',
                   help='车尾有结构遮挡时启用车尾屏蔽盲区 (默认关闭以启用 360° 全向雷达跟踪与倒车)')
    p.add_argument('--dry-run', action='store_true',
                   help='仿真演练:照常计算与打印,但不向底盘发指令')
    p.add_argument('--safe-mode', action='store_true',
                   help='首次实车验证用的保守参数组 (低速 + 大停车距离)')
    p.add_argument('--target', type=str, default='person',
                   help='追踪类别: person (默认) / face / any')
    p.add_argument('--follow-distance-m', type=float, default=None, dest='follow_distance_m')
    p.add_argument('--follow-stop-m', type=float, default=None, dest='follow_stop_m')
    p.add_argument('--max-speed-mps', type=float, default=None, dest='max_speed_mps')
    p.add_argument('--decel-mps2', type=float, default=None, dest='decel_capability_mps2',
                   help='★ 实测减速度,标定方法见 docs/TUNING.md')
    p.add_argument('--latency-s', type=float, default=None, dest='control_latency_s',
                   help='★ 实测感知到执行的总死时间')
    p.add_argument('--aeb-clearance-m', type=float, default=None, dest='aeb_clearance_m')
    p.add_argument('--obstacle-standoff-m', type=float, default=None, dest='obstacle_standoff_m')
    p.add_argument('--max-steer-deg', type=float, default=None, dest='max_steer_deg',
                   help='★ 实测满舵角度(度)。轴距 0.54 下它对转弯半径很敏感')
    p.add_argument('--lidar-yaw-deg', type=float, default=None, dest='lidar_yaw_deg',
                   help='雷达安装偏航角偏差(度,逆时针为正),用于雷达物理转动后的软件零点校准')
    p.add_argument('--margin-m', type=float, default=None, dest='footprint_margin_m',
                   help='侧向安全余量(米)。过窄门时可临时调小试探')
    p.add_argument('--aeb-margin-m', type=float, default=None, dest='aeb_margin_m',
                   help='AEB专属硬急停侧向物理余量(米),默认0.015')
    p.add_argument('--min-clearance-m', type=float, default=None, dest='min_path_clearance_m',
                   help='最小通行净空门限(米),默认0.15')
    p.add_argument('--camera-pitch-deg', type=float, default=None, dest='camera_pitch_deg',
                   help='相机俯角(度,向下为正),默认 15')
    p.add_argument('--pre-steer', action='store_true',
                   help='静止时用微速度触发预打舵 (PROTOCOL.md 8.3),需实车确认')
    args = p.parse_args(strip_ros_args(sys.argv[1:]))

    cfg = build_config(args)
    try:
        # 由本节点自己处理 SIGINT,保证刹停帧在 ROS 上下文关闭之前发出去
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    node = PersonFollowerNode(cfg, dry_run=args.dry_run, target_class=args.target, passive=args.passive)

    def on_signal(_sig=None, _frame=None):
        # 只打断 spin;刹停和销毁统一在 finally 里做一次
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n>>> 捕获中断,安全刹停...")
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
