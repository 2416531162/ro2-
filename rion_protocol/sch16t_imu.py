#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gen5 的 IMU：SCH16T 倾角 / 姿态模块（CAN 原生协议，见 sch16t_codec.py），与两台欧艾迪驱动器挂同一条总线。
代替 HDA536T 驱动（hda536t_imu.py），话题、消息、坐标系都不变，line_node、app 不用改：

    发布  /imu        sensor_msgs/Imu：陀螺 + 加速度，不带姿态（orientation_covariance[0] = −1，ROS 约定）。
          /imu/data   sensor_msgs/Imu：同上，再带模块自己算好的姿态（横滚俯仰给 app 和倾斜补偿，航向角进融合）。
                      偏航是上电标定完成时为 0 的相对值。
          /imu/status std_msgs/String（JSON，1 Hz）：各帧频率、flag、状态位、温度、丢帧、问题。

坐标系 ROS REP-103（x 车头、y 左、z 上），frame_id 用共享配置的 frames.imu。时间戳是本节点读到那一帧的时刻。
只收不发：模块设置（周期、掩码、方位角清零等）按协议发 0x401，本节点不碰。

每个周期模块按「角度 → 加速度 → 陀螺 → 高精度姿态 → 状态」的顺序发，本节点在陀螺帧到时发一次（同一周期的
角度、加速度一起带上）。flag 报故障、未就绪（上电标定、中断后重新捕获、陀螺零偏清除）、安装方式不认识、
没有陀螺帧时不发 /imu：融合报「IMU 数据中断」，车不会开，/imu/status 里写明原因（状态帧开着时细到哪一位）。
"""
import json
import math
import os
import sys
import threading
import time
from collections import deque
from functools import wraps

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

import sch16t_codec as sc  # noqa: E402
from hda536t_imu import CanSocket, fill_imu  # noqa: E402  同一条 CAN、同样的 Imu 消息填法
from sch16t_rawlog import RawLog  # noqa: E402
from robot_core.service_watchdog import watchdog  # noqa: E402

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    ROS_AVAILABLE = True
except ImportError:         # 电脑上跑单元测试
    Node = object
    ROS_AVAILABLE = False

RATE_WINDOW_S = 2.0
MIN_GYRO_HZ = 20.0          # 融合要求 IMU 0.2 s 内有新数据；低于这个频率在状态里提醒
GAP_COUNT_MAX_S = 1.0       # 两帧状态帧间隔超过这个就不按帧计数算丢了几个周期（计数 8 位会绕回）
ENABLE_GYRO_HINT = '向 0x401 发 40 30 10 00 17 00 00 00（角度 + 加速度 + 陀螺 + 状态）'


def _synchronized(method):
    @wraps(method)
    def call(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)
    return call


class Sch16t:
    """不依赖 ROS 的接收逻辑：认帧、凑齐一个周期、换到 REP-103、记状态。t 是单调时钟秒。"""

    def __init__(self, data_id=sc.DATA_ID, stale_s=0.1, mount_yaw_deg=0.0):
        if not 0 < data_id <= 0x1FFFFFFF:
            raise ValueError('数据帧 ID 无效')
        if not math.isfinite(mount_yaw_deg):
            raise ValueError('mount_yaw_deg 必须是有限数')
        self.data_id, self.stale_s, self.mount_yaw_deg = data_id, stale_s, mount_yaw_deg
        self._lock = threading.RLock()  # CAN reader and ROS status timer share this state
        self.last = {}                  # kind -> (t, 帧)
        self.times = {k: deque() for k in sc.KIND_NAMES}
        self.newest = None              # (t, 帧)：最近一帧，含原始数据帧
        self.bad = 0
        self.gaps = 0
        self.started = None

    @_synchronized
    def feed(self, t, can_id, data):
        """收到一帧。陀螺帧到了、模块正常时返回一组数据（dict：t、angular_velocity、linear_acceleration、
        orientation、rpy、temp_c，没有的项不带），其余返回 None。"""
        if self.started is None:
            self.started = t
        if not sc.is_data_id(can_id, self.data_id):
            return None
        frame = sc.decode(data)
        if frame is None:
            self.bad += 1
            return None
        self.newest = (t, frame)
        if frame.kind == 'raw':
            return None
        if frame.kind == sc.STATUS:
            prev = self.last.get(sc.STATUS)
            if prev is not None and 0.0 < t - prev[0] <= GAP_COUNT_MAX_S:
                self.gaps += (frame.counter - prev[1].counter - 1) & 0xFF
        self.last[frame.kind] = (t, frame)
        q = self.times[frame.kind]
        q.append(t)
        while q and t - q[0] > RATE_WINDOW_S:
            q.popleft()
        if frame.kind != sc.GYRO or self._blocking(frame, t):
            return None
        angle, accel = self._fresh(sc.ANGLE, t), self._fresh(sc.ACCEL, t)
        out = dict(t=t, **sc.to_ros(angle, accel, frame, self.mount_yaw_deg))
        if angle is not None:
            out['temp_c'] = angle.temp_c
        return out

    def _fresh(self, kind, t):
        """同一周期（stale_s 内）、flag 正常的那一帧；没有返回 None。"""
        v = self.last.get(kind)
        return v[1] if v is not None and 0.0 <= t - v[0] <= self.stale_s and v[1].test == sc.TEST_OK else None

    def _status(self, now, max_age=0.5):
        v = self.last.get(sc.STATUS)
        return v[1] if v is not None and 0.0 <= now - v[0] <= max_age else None

    def _blocking(self, frame, now):
        """不能发 /imu 的原因；能发返回 None。状态帧开着时说到具体哪一位。"""
        st = self._status(now)
        if frame.test == sc.TEST_FAULT:
            if st is not None and st.has(sc.ST_IMU_FAULT):
                return 'IMU 模块故障：SCH16T 传感器报错或没有数据'
            if st is not None and st.has(sc.ST_TIMING_FAULT):
                return 'IMU 模块故障：采样率或 CPU 超时'
            return 'IMU 模块报故障'
        if frame.test == sc.TEST_NOT_READY:
            if st is not None and st.has(sc.ST_BIAS_CAPTURE):
                return 'IMU 陀螺零偏清除中'
            if st is not None and st.has(sc.ST_CALIBRATING):
                return 'IMU 上电标定中：车保持静止 1 s 以上（7 s 还不静止就按快速启动输出）'
            if st is not None and st.has(sc.ST_GAP):
                return 'IMU 数据中断后重新捕获中'
            return 'IMU 模块未就绪'
        if frame.test != sc.TEST_OK:
            return 'IMU flag 是 11，协议里保留没用'
        if frame.mount != 0:
            return '模块报的安装方式是 %d，本驱动只按水平安装换算' % frame.mount
        return None

    @_synchronized
    def rate(self, kind, now):
        q = [x for x in self.times[kind] if 0.0 <= now - x <= RATE_WINDOW_S]
        if len(q) < 2:
            return 0.0
        return (len(q) - 1) / max(q[-1] - q[0], 1e-6)

    @_synchronized
    def problem(self, now):
        """/imu 发不出来或不可靠的原因（给人看）；正常时 None。"""
        if self.newest is None:
            waited = now - self.started if self.started is not None else 0.0
            return '没有收到 IMU 数据帧（查供电、CAN 接线、数据帧 ID 0x%X）%s' % (
                self.data_id, '，已等 %.0f s' % waited if waited >= 1 else '')
        if now - self.newest[0] > 0.5:
            return 'IMU 数据中断 %.1f s' % (now - self.newest[0])
        blocking = self._blocking(self.newest[1], now)
        if blocking:
            return blocking
        gyro = self.last.get(sc.GYRO)
        if gyro is None or now - gyro[0] > 0.5:
            return '没有陀螺帧：模块掩码还是出厂 0x19，' + ENABLE_GYRO_HINT
        hz = self.rate(sc.GYRO, now)
        if hz < MIN_GYRO_HZ:
            return '陀螺帧只有 %.0f Hz，融合要 %.0f Hz 以上：把输出周期改到 50 ms 以内' % (hz, MIN_GYRO_HZ)
        return None

    @_synchronized
    def status(self, now):
        angle = self.last.get(sc.ANGLE)
        frame = self.newest[1] if self.newest else None
        st = self._status(now)
        out = {'data_id': self.data_id,
               'age_s': round(now - self.newest[0], 3) if self.newest else None,
               'rates_hz': {name: round(self.rate(k, now), 1) for k, name in sc.KIND_NAMES.items()},
               'self_test': None if frame is None else sc.TEST_NAMES.get(frame.test, format(frame.test, '02b')),
               'mount': None if frame is None else sc.MOUNTS.get(frame.mount, str(frame.mount)),
               'temp_c': angle[1].temp_c if angle else None, 'bad_frames': self.bad,
               'sequence_gaps': self.gaps,
               'flags': None if st is None else st.flags,
               'cross_sigma_deg': None if st is None else st.cross_sigma_deg,
               'problem': self.problem(now)}
        if angle and now - angle[0] <= 0.5:
            roll, pitch, yaw = sc.to_ros(angle=angle[1], mount_yaw_deg=self.mount_yaw_deg)['rpy']
            out.update(roll_deg=round(math.degrees(roll), 2), pitch_deg=round(math.degrees(pitch), 2),
                       yaw_deg=round(math.degrees(yaw), 2))
        return out


def make_raw_log(p, log):
    """raw_log_dir 空、或它的上一级目录不存在（比如没有 /userdata 的板子）就不记。记数据帧和指令（数据帧 ID + 1，0x401）、应答（+ 2，0x402）。"""
    d = str(p.get('raw_log_dir') or '')
    if not d:
        return None
    if not os.path.isdir(os.path.dirname(os.path.abspath(d))):
        log.warn('IMU 原始记录不开：%s 的上一级目录不存在' % d)
        return None
    data_id = int(p['data_id'])
    log.info('IMU 原始帧记到 %s' % d)
    return RawLog(d, (data_id, data_id + 1, data_id + 2), float(p['raw_log_max_mb']),
                  float(p['raw_log_min_free_mb']), float(p['raw_log_file_s']),
                  meta={'data_id': data_id, 'mount_yaw_deg': float(p['mount_yaw_deg'])})


class Sch16tNode(Node):
    def __init__(self, bus=None, clock=time.monotonic):
        super().__init__('sch16t_imu')
        from robot_core.config import PROFILE
        defaults = (('can_channel', 'can0'), ('data_id', sc.DATA_ID), ('stale_s', 0.1),
                    ('mount_yaw_deg', 0.0), ('frame_id', PROFILE['frames']['imu']),
                    ('raw_log_dir', ''), ('raw_log_max_mb', 3000.0), ('raw_log_min_free_mb', 2048.0),
                    ('raw_log_file_s', 3600.0))
        for name, value in defaults:
            self.declare_parameter(name, value)
        p = {n: self.get_parameter(n).value for n, _ in defaults}
        self.frame_id, self.clock = p['frame_id'], clock
        self.imu = Sch16t(int(p['data_id']), float(p['stale_s']), float(p['mount_yaw_deg']))
        self.bus = bus or CanSocket(p['can_channel'])
        self.raw_log = make_raw_log(p, self.get_logger())
        self.pub_raw = self.create_publisher(Imu, '/imu', 20)
        self.pub_data = self.create_publisher(Imu, '/imu/data', 20)
        self.pub_status = self.create_publisher(String, '/imu/status', 1)
        self.create_timer(1.0, self.publish_status)
        self.running = True
        self._reported = None

    def poll(self):
        """读一帧（超时返回 False），陀螺帧凑齐一组就发 /imu 和 /imu/data。"""
        got = self.bus.recv()
        if got is None:
            return False
        if self.raw_log is not None:
            self.raw_log.write(*got)
        sample = self.imu.feed(self.clock(), *got)
        if sample is not None:
            stamp = self.get_clock().now().to_msg()
            self.pub_raw.publish(fill_imu(Imu(), sample, stamp, self.frame_id, False))
            self.pub_data.publish(fill_imu(Imu(), sample, stamp, self.frame_id, True))
        return True

    def spin_bus(self):
        while self.running:
            watchdog.tick("can")
            try:
                self.poll()
            except OSError as exc:      # 接口掉了：等一会再读，状态里会报中断
                self.get_logger().error('CAN 读取失败：%s' % exc)
                time.sleep(0.5)

    def publish_status(self):
        st = self.imu.status(self.clock())
        if self.raw_log is not None:
            st['raw_log'] = self.raw_log.status()
        self.pub_status.publish(String(data=json.dumps(st, ensure_ascii=False)))
        if st['problem'] != self._reported:
            (self.get_logger().warn if st['problem'] else self.get_logger().info)(st['problem'] or 'IMU 正常')
            self._reported = st['problem']


def main():
    if not ROS_AVAILABLE:
        raise SystemExit('需要 ROS 2（rclpy）')
    watchdog.start("ros", "can")
    rclpy.init()
    node = Sch16tNode()
    node.create_timer(0.5, lambda: watchdog.tick("ros"))
    reader = threading.Thread(target=node.spin_bus, daemon=True)
    reader.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        if node.raw_log is not None:
            node.raw_log.close()
        watchdog.close()
        node.running = False
        reader.join(timeout=1.0)
        node.bus.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
