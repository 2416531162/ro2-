#!/usr/bin/env python3
"""IMU762 USB 串口接收器：只读取，支持 JSONL 记录和独立 ROS 话题。

必须明确指定专用串口，不扫描或试开其他 ttyUSB，避免占用 GNSS。
拔出后报告中断，重新插入同一设备路径后重连。不发送模块配置命令。
"""
import argparse
from collections import deque
import json
import logging
from logging.handlers import RotatingFileHandler
import math
import os
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent)]
from imu762_codec import StreamDecoder, to_ros

# 独立复制这两个文件也能读取；仅在现有 systemd 看门狗环境下加载车端看门狗。
watchdog = None
if os.environ.get('WATCHDOG_USEC'):
    from robot_core.service_watchdog import watchdog

BAUDRATES = (9600, 19200, 38400, 57600, 76800, 115200, 230400, 460800, 921600)


class ReceiverState:
    """记录接收频率、校验错误和最后一帧；状态与串口读取共用主线程。"""
    def __init__(self):
        self.decoder = StreamDecoder()
        self.latest = self.received_at = self.started_at = self.previous_sequence = None
        self.times = deque(maxlen=2000)
        self.connected = False
        self.error = '等待打开指定串口'
        self.sequence_gaps = self.duplicate_frames = self.sequence_resets = 0
        self.rx_bytes = 0
        self.last_chunk_hex = None

    def disconnected(self, error):
        self.connected = False
        self.error = error
        self.previous_sequence = None
        self.latest = self.received_at = self.started_at = None
        self.times.clear()
        self.decoder.reset_buffer()

    def feed(self, data, now):
        self.rx_bytes += len(data)
        if data:
            self.last_chunk_hex = bytes(data[:32]).hex()
        frames = self.decoder.feed(data)
        for frame in frames:
            if self.started_at is None:
                self.started_at = now
            sequence = frame.sequence
            previous = self.previous_sequence
            if previous is not None and 1 <= sequence <= 60000 and 1 <= previous <= 60000:
                delta = (sequence - previous) % 60000
                if delta == 0:
                    self.duplicate_frames += 1
                elif delta <= 10000:
                    self.sequence_gaps += delta - 1
                else:
                    # 设备重启、序号倒退时不把整圈误算成丢帧。
                    self.sequence_resets += 1
            self.previous_sequence = sequence
            self.latest, self.received_at = frame, now
            self.times.append(now)
        return frames

    def status(self, now, port, baud, stale_s):
        while self.times and now - self.times[0] > 2.:
            self.times.popleft()
        age = None if self.received_at is None else max(0., now - self.received_at)
        elapsed = None if self.started_at is None else min(2., max(0., now - self.started_at))
        rate = len(self.times) / elapsed if elapsed and elapsed >= .1 else 0.
        problem = self.error if not self.connected else None
        if self.connected and (age is None or age > stale_s):
            problem = '未收到有效 IMU762 帧，请核对设备路径、波特率和接线' if age is None else 'IMU762 数据中断'
        return dict(sensor='IMU762',port=port,baud=baud,connected=self.connected,
                    age_s=None if age is None else round(age,3),rate_hz=round(rate,1),
                    good_frames=self.decoder.frames,bad_checksum=self.decoder.bad_checksum,
                    bad_payload=self.decoder.bad_payload,discarded_bytes=self.decoder.discarded_bytes,
                    buffered_bytes=len(self.decoder.buffer),sequence_gaps=self.sequence_gaps,
                    duplicate_frames=self.duplicate_frames,sequence_resets=self.sequence_resets,
                    sequence=None if self.latest is None else self.latest.sequence,
                    fields=[] if self.latest is None else list(self.latest.values),
                    rx_bytes=self.rx_bytes,rx_preview_hex=self.last_chunk_hex,problem=problem)


def sample_record(frame, received_ns, monotonic_ns):
    """设备时间戳原样保留，额外记录主机接收时刻，不把设备启动时间当 Unix 时间。"""
    return dict(sensor='IMU762',received_unix_ns=received_ns,received_monotonic_ns=monotonic_ns,
                sequence=frame.sequence,coordinate='FRD',values=frame.values,
                ros=to_ros(frame),unknown=frame.unknown,raw_hex=frame.raw.hex())


def open_serial(port, baud):
    """只打开用户指定设备；关闭硬件流控，不调用串口 write。"""
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError('需要 python3-serial（pyserial）') from exc
    connection = serial.Serial(port=None,baudrate=baud,bytesize=serial.EIGHTBITS,
                               parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,
                               timeout=.1,xonxoff=False,rtscts=False,dsrdtr=False)
    connection.dtr = connection.rts = False
    if sys.platform.startswith('linux'):
        connection.exclusive = True
    connection.port = port
    try:
        connection.open()
    except Exception:
        connection.close()
        raise
    return connection


def record_logger(path, megabytes, backups):
    path = Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    logger = logging.getLogger('imu762_samples')
    logger.setLevel(logging.INFO)
    logger.propagate = False
    handler = RotatingFileHandler(path,maxBytes=megabytes*1024*1024,backupCount=backups,encoding='utf-8')
    handler.setFormatter(logging.Formatter('%(message)s'))
    logger.addHandler(handler)
    return logger, handler


def create_ros(args):
    """默认发布 /imu762 系列话题；选定替换方案后才使用 /imu 前缀。"""
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Imu, MagneticField
        from std_msgs.msg import String
    except ImportError as exc:
        raise RuntimeError('ROS 模式需要 rclpy、sensor_msgs 和 std_msgs') from exc
    rclpy.init(args=[])

    class Imu762Node(Node):
        def __init__(self):
            super().__init__('imu762_reader')
            prefix = args.topic_prefix.rstrip('/')
            self.raw = self.create_publisher(Imu,prefix,20)
            self.data = self.create_publisher(Imu,prefix+'/data',20)
            self.samples = self.create_publisher(String,prefix+'/samples',20)
            self.status = self.create_publisher(String,prefix+'/status',1)
            self.magnetic = self.create_publisher(MagneticField,prefix+'/magnetic',20)
            self.last_publish = -math.inf

        def publish_frames(self, frames, now, records):
            if not frames or now - self.last_publish < 1./args.publish_hz:
                return
            self.last_publish = now
            self.samples.publish(String(data=json.dumps(records[-1],ensure_ascii=False)))
            frame = next((f for f in reversed(frames) if 'gyro_deg_s' in f.values),None)
            if frame is None:
                return
            converted = to_ros(frame)
            stamp = self.get_clock().now().to_msg()
            for orientation, publisher in ((False,self.raw),(True,self.data)):
                if orientation and 'orientation' not in converted:
                    continue
                msg = Imu()
                msg.header.stamp, msg.header.frame_id = stamp,args.frame_id
                msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z = converted['angular_velocity']
                if 'linear_acceleration' in converted:
                    msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z = converted['linear_acceleration']
                else:
                    msg.linear_acceleration_covariance[0] = -1.
                if orientation:
                    msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w = converted['orientation']
                else:
                    msg.orientation_covariance[0] = -1.
                # 零协方差矩阵按 ROS 约定表示未知，不借用 HDA536T 的精度参数。
                publisher.publish(msg)
            if 'magnetic_field' in converted:
                msg = MagneticField()
                msg.header.stamp,msg.header.frame_id = stamp,args.frame_id
                msg.magnetic_field.x,msg.magnetic_field.y,msg.magnetic_field.z = converted['magnetic_field']
                self.magnetic.publish(msg)

        def publish_status(self, status):
            self.status.publish(String(data=json.dumps(status,ensure_ascii=False)))

    return rclpy, Imu762Node()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    device = parser.add_mutually_exclusive_group(required=True)
    device.add_argument('--port',help='专用 USB 串口，推荐 /dev/serial/by-id/ 下的固定路径')
    device.add_argument('--usb-device',type=Path,help='没有 CH340 内核驱动时，指定其 /sys/bus/usb/devices/ 路径')
    parser.add_argument('--baud',type=int,choices=BAUDRATES,default=460800)
    parser.add_argument('--ros',action='store_true',help='同时发布 ROS 话题')
    parser.add_argument('--topic-prefix',default='/imu762')
    parser.add_argument('--frame-id',default='imu762_link',help='要求传感器水平安装，X 轴朝车头')
    parser.add_argument('--publish-hz',type=float,default=50.,help='ROS 最大发布频率；接收和记录仍覆盖全部帧')
    parser.add_argument('--print-hz',type=float,default=10.,help='终端显示频率，0 关闭逐帧显示')
    parser.add_argument('--stale-s',type=float,default=.2)
    parser.add_argument('--log',type=Path,help='全部有效帧的 JSONL 文件，自动轮换')
    parser.add_argument('--log-mb',type=int,default=64,help='单个记录文件上限 MiB')
    parser.add_argument('--log-backups',type=int,default=4)
    parser.add_argument('--once',action='store_true',help='收到一个有效帧后退出，打开失败也退出')
    parser.add_argument('--duration',type=float,default=0.,help='读取指定秒数后退出，0 持续读取')
    args = parser.parse_args(argv)
    if (not args.topic_prefix.startswith('/') or args.topic_prefix.rstrip('/') == ''
            or any(not math.isfinite(v) for v in (args.publish_hz,args.print_hz,args.stale_s,args.duration))
            or not 0 < args.publish_hz <= 400 or args.print_hz < 0 or args.stale_s <= 0
            or args.log_mb <= 0 or args.log_backups < 1 or args.duration < 0):
        parser.error('串口、话题前缀、频率或记录上限不正确')
    state = ReceiverState()
    bus = node = ros = handler = logger = None
    next_open = next_status = 0.
    last_print = -math.inf
    endpoint = str(args.usb_device) if args.usb_device else args.port
    deadline = time.monotonic()+args.duration if args.duration else math.inf
    try:
        if args.log:
            logger,handler = record_logger(args.log,args.log_mb,args.log_backups)
        if args.ros:
            ros,node = create_ros(args)
        if watchdog is not None:
            watchdog.start('reader')
        while True:
            if watchdog is not None:
                watchdog.tick('reader')
            now = time.monotonic()
            if now >= deadline:
                if not state.decoder.frames:
                    raise SystemExit('读取结束，但未收到有效 IMU762 帧')
                return
            if bus is None and now >= next_open:
                try:
                    if args.usb_device:
                        from ch340_usb import Ch340Usb
                        bus = Ch340Usb(args.usb_device,args.baud)
                    else:
                        bus = open_serial(args.port,args.baud)
                    state.connected,state.error = True,None
                except (OSError,RuntimeError) as exc:
                    state.disconnected(str(exc))
                    next_open = now+1.
                    if args.once:
                        raise SystemExit('串口打开失败：'+str(exc))
            frames = []
            if bus is not None:
                try:
                    chunk = bus.read(max(1,min(4096,bus.in_waiting)))
                    received_ns,monotonic_ns = time.time_ns(),time.monotonic_ns()
                    now = monotonic_ns/1e9
                    frames = state.feed(chunk,now)
                    records = [sample_record(f,received_ns,monotonic_ns) for f in frames]
                    if logger:
                        for record in records:
                            logger.info(json.dumps(record,ensure_ascii=False,separators=(',',':')))
                    if frames and (args.once or args.print_hz and now-last_print >= 1./args.print_hz):
                        print(json.dumps(records[-1],ensure_ascii=False),flush=True)
                        last_print = now
                    if node:
                        node.publish_frames(frames,now,records)
                    if args.once and frames:
                        return
                except OSError as exc:
                    bus.close()
                    bus = None
                    state.disconnected(str(exc))
                    next_open = now+1.
            else:
                time.sleep(.05)
            if now >= next_status:
                status = state.status(now,endpoint,args.baud,args.stale_s)
                status['transport'] = 'libusb-ch340' if args.usb_device else 'serial'
                if args.usb_device and bus is not None:
                    status['actual_baud'] = round(bus.actual_baud,1)
                    status['chip_version'] = hex(bus.version)
                print(json.dumps(status,ensure_ascii=False),file=sys.stderr,flush=True)
                if node:
                    node.publish_status(status)
                next_status = now+1.
            if ros:
                ros.spin_once(node,timeout_sec=0.)
    except KeyboardInterrupt:
        pass
    finally:
        if bus is not None:
            bus.close()
        if handler is not None:
            handler.close()
        if node is not None:
            node.destroy_node()
        if ros is not None:
            ros.shutdown()


if __name__ == '__main__':
    main()
