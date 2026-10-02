#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Gen5 RTK 划线车的网页服务（端口 8088）
- 平板 app：/ws 长连接（hello + 10 Hz 遥测，摇杆 / IMU 直行 / 急停 / 直线、路线、场地作业）、UDP 发现广播
- 车载屏幕只读显示 /display、网页手动遥控 /、CORS 账号、场地规划 HTTP 接口
- 汇聚底盘、RTK、走直线节点的状态；手动指令 50 Hz 发给底盘控制权仲裁
"""

import http.server
import socketserver
import json
import os
import math
import time
import threading
import subprocess
import socket
import sys
import uuid
import gc
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from runtime_config import PROFILE      # 先导入：它把仓库根目录加进 sys.path，下面才找得到 robot_core
from robot_core.kinematics import ChassisGeometry, yaw_from_steer, steer_from_yaw
from robot_core.service_watchdog import watchdog
from motion_client import MotionClient
from manual_drive import ManualDriveLatch
from heading_hold import HeadingHold, yaw_from_quaternion
import app_link
import car_display
import car_network
from display_control import ScreenControl
from ntrip_config import CorsConfigStore, configuration_allowed
from runtime_config import profile_hash

PORT = 8088
CAR_NAME = os.environ.get('RK3588_CAR_NAME') or socket.gethostname()
APP_TELEMETRY_PERIOD_S = 0.10
APP_RECV_TIMEOUT_S = 3.0     # app 每秒 ping 一次;3 秒没有任何消息视为断线
# 车载屏幕只读显示（/display）：连着几台平板、车的网络
APP_SESSIONS = car_display.Sessions()
NETWORK = car_display.Network()
APP_VIEW = car_display.AppView()          # 平板地图上正在预览的图纸，车载屏幕跟着显示
# 车的 WiFi 方式：车自己发热点 / 连现场 WiFi，app 一条 network 指令切换（car_network.py）
CAR_NETWORK = car_network.CarNetwork(app_joined=APP_SESSIONS.joined)
CORS_CONFIG = CorsConfigStore(os.environ.get('RK3588_NTRIP_ENV', '~/rtk/ntrip.env'))
SCREEN_CONTROL = ScreenControl()
from field_plan_service import FieldPlanService
FIELD_PLANS = FieldPlanService()
MANUAL_PUBLISH_PERIOD_S = 0.02
GC_SLOW_S = 0.05            # 一次垃圾回收超过这么久记一条 gc_slow 日志
STALL_LOG_S = 0.15          # 50 Hz 手动循环两次之间隔超过这么久记一条 web_stall 日志（进程卡了）
MANUAL_HEARTBEAT_TIMEOUT_S = 0.75

# =============================================================================
# 手动遥控:速度档与转向档解耦
# =============================================================================
# 改造前前端把 (vx, wz) 成对硬编码成九宫格 x 三档,但阿克曼底盘的前轮转角由
# 固件按 TurnR = Vx / Vz 解算 (见 wheeltec_protocol/PROTOCOL.md 8.1),也就是说
# 转角只取决于**比值**。实测三个速度档的「左拐」换算出来都是 20.1 度,全部撞在
# 舵机 20 度限位上 —— 三档完全一样,而斜向键只有 13 度,所以手感又钝又不跟手。
#
# 现在速度和转角是两个独立维度:UI 指定「开多快」和「方向盘打多少度」,
# 角速度由后端按当前车速实时换算,换速度档不会改变转弯半径。
CHASSIS = ChassisGeometry()


def clamp(value, low, high):
    return low if value < low else (high if value > high else value)

# 与页面按钮的标称保持一致；高档仍低于驱动层 1.30m/s 硬上限。
SPEED_TIERS_MPS = {tier: PROFILE['manual'][tier + '_mps'] for tier in ('low', 'med', 'high')}
REVERSE_SCALE = PROFILE['manual']['reverse_scale']                      # 倒车是盲区方向,统一降速
STEER_TIERS_DEG = {'gentle': 8.0, 'normal': 14.0, 'full': 20.0}

# 每个方向键:(前进符号, 转角占该档的比例, 车速折扣)
# 急转时降速,既是安全考虑,也让小半径转弯真的转得过来
DIRECTIONS = {
    'forward':       (+1,  0.0, 1.00),
    'forward_left':  (+1, +0.6, 0.85),
    'forward_right': (+1, -0.6, 0.85),
    'left':          (+1, +1.0, 0.60),
    'right':         (+1, -1.0, 0.60),
    'reverse':       (-1,  0.0, 1.00),
    'reverse_left':  (-1, +1.0, 0.70),
    'reverse_right': (-1, -1.0, 0.70),
    'stop':          (0,   0.0, 0.00),
}


def resolve_drive(direction, speed_tier='med', steer_tier='normal'):
    """把 (方向键, 速度档, 转角档) 解析成 (vx, wz, 实际转角度数)。"""
    spec = DIRECTIONS.get(direction)
    if spec is None or direction == 'stop':
        return 0.0, 0.0, 0.0
    sign, steer_ratio, speed_scale = spec

    base = SPEED_TIERS_MPS.get(speed_tier, SPEED_TIERS_MPS['med'])
    vx = sign * base * speed_scale * (REVERSE_SCALE if sign < 0 else 1.0)

    steer_deg = STEER_TIERS_DEG.get(steer_tier, STEER_TIERS_DEG['normal']) * steer_ratio
    steer_rad = clamp(math.radians(steer_deg), -CHASSIS.max_steer_rad, CHASSIS.max_steer_rad)

    wz = yaw_from_steer(vx, steer_rad, CHASSIS)
    return round(vx, 4), round(wz, 4), round(math.degrees(steer_rad), 1)
TEMPLATE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'templates', 'index.html')

data_lock = threading.Lock()
state = {
    # 动力电池状态
    'voltage': 0.0,
    'battery_pct': 0,
    'manual_override': {'active': False, 'action': 'none', 'vx': 0.0, 'wz': 0.0}
}

bridge_node = None

def snapshot_state():
    """一份给网页 / app / 车载屏幕用的状态快照。"""
    with data_lock:
        snapshot = dict(state)
        wheeltec_rx = snapshot.get('wheeltec', {}).get('_rx_monotonic')
        snapshot['wheeltec_status_age_s'] = (
            round(time.monotonic() - wheeltec_rx, 2) if wheeltec_rx else None)
    if bridge_node is not None:
        # 直接取仲裁状态,不依赖 50Hz 定时器写回的副本
        with bridge_node.motion.lock:
            snapshot['motion'] = dict(bridge_node.motion.state)
        snapshot['motion_fresh'] = bridge_node.motion.fresh()
        snapshot['straight'] = bridge_node.straight.status()
    snapshot['network'] = CAR_NETWORK.status()
    return snapshot


def emergency_stop():
    """急停:先清零手动指令,再锁存 ESTOP。不等 control_lock —— 复位可能拿着锁等服务应答。"""
    if bridge_node is None:
        return False, 'motion bridge unavailable'
    bridge_node.straight.end('estop')
    bridge_node.manual_drive.set(0.0, 0.0)
    bridge_node.motion.publish(0.0, 0.0)
    return bridge_node.call_motion_service(bridge_node.cli_stop, Trigger.Request())


def motion_reset():
    if bridge_node is None:
        return False, 'motion bridge unavailable'
    with bridge_node.control_lock:
        bridge_node.straight.end('manual')
        bridge_node.manual_drive.set(0.0, 0.0)
        bridge_node.manual_drive.sample()
        return bridge_node.call_motion_service(bridge_node.cli_reset, Trigger.Request())


def apply_manual(vx, wz, steer_deg, action, client_id, seq, hold=None):
    """网页方向键与 app 摇杆共用的手动指令出口,返回 (HTTP 状态码, 应答)。

    hold:IMU 定向直行的作业键 (会话, 作业编号),这时 vx 是直行车速,实际发的车速和角速度由
    bridge_node.straight 按 IMU 偏航算;别的手动指令(摇杆、网页方向键)一来就接管,直行结束。"""
    if bridge_node is None:
        return 503, {'ok': False, 'error': 'motion bridge unavailable'}
    moving = abs(vx) > 1e-6 or abs(wz) > 1e-6
    if hold is None:
        bridge_node.straight.end('manual')
    immediate_stop = False
    if moving:
        # A reset can wait for a ROS service. Keep the short command
        # lock free so a stop can preempt a request waiting here.
        ok, message = bridge_node.recover_latched_fault()
        if not ok:
            return 409, {'ok': False, 'error': message}
    else:
        # First zero goes out before a reset service call holding
        # control_lock can delay the request. The second zero
        # below wins over a command already in that critical section.
        bridge_node.manual_drive.accept(client_id, seq, 0.0, 0.0)
        bridge_node.send_manual_twist(0.0, 0.0)
        immediate_stop = bridge_node.motion.publish(0.0, 0.0)

    with bridge_node.control_lock:
        # 按会话 + 序号丢弃迟到的旧指令:网络抖动时,先发的「前进」
        # 可能晚于「刹车」到达,不能让它在松手后又把车开起来。
        if moving and not bridge_node.manual_drive.accept(client_id, seq, vx, wz):
            return 200, {'ok': True, 'stale': True, 'action': action}
        bridge_node.send_manual_twist(vx, wz, hold)
        out_vx, out_wz = (vx, wz) if hold is None else bridge_node.straight.step(hold)
        sent = bridge_node.motion.publish(out_vx, out_wz)
        if moving and not sent:
            bridge_node.straight.end('chassis')
            bridge_node.send_manual_twist(0.0, 0.0)

    if not sent and (moving or not immediate_stop):
        return 503, {'ok': False, 'error': '底盘控制状态已过期，指令未能发送；请检查连接并使用物理急停'}

    with data_lock:
        state['manual_override'] = {'active': moving, 'action': action, 'vx': vx, 'wz': wz,
                                    'steer_deg': steer_deg, 'time': time.time()}
    return 200, {'ok': True, 'action': action, 'vx': vx, 'wz': wz, 'steer_deg': steer_deg}


def discovery_beacon():
    """每秒向各网卡广播一次,路由器 WiFi 和车载热点下 app 都能自动找到车。"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    payload = app_link.beacon(CAR_NAME, PORT, profile_hash(PROFILE))
    targets, refreshed = ['255.255.255.255'], 0.0
    while True:
        now = time.monotonic()
        if now - refreshed > 10.0:
            refreshed = now
            try:
                out = subprocess.run(['ip', '-o', '-4', 'addr', 'show'], capture_output=True,
                                     text=True, timeout=2).stdout
                targets = app_link.broadcast_addresses(out) + ['255.255.255.255']
            except Exception:
                targets = ['255.255.255.255']
        for addr in targets:
            try:
                sock.sendto(payload, (addr, app_link.DISCOVERY_PORT))
            except OSError:
                pass
        time.sleep(1.0)


class WebBridgeNode(Node):
    def __init__(self):
        super().__init__('web_bridge')
        self.sub_voltage = self.create_subscription(Float32, '/voltage', self.voltage_cb, 10)
        self.sub_wheeltec = self.create_subscription(String, '/wheeltec/status', self.wheeltec_cb, 10)

        self.motion = MotionClient(self, 'manual', timeout_s=1.0)
        self.cli_reset = self.create_client(Trigger, '/motion/reset')
        self.cli_stop = self.create_client(Trigger, '/motion/stop')
        self.control_lock = threading.RLock()
        self.is_armed = False

        self.manual_drive = ManualDriveLatch(MANUAL_HEARTBEAT_TIMEOUT_S, allow_pivot=CHASSIS.differential)
        # IMU 定向直行:只用 /imu/data 的偏航角(不用 RTK)。独立回调组,50 Hz 不被别的回调拖住
        self.straight = HeadingHold()
        self.imu_group = MutuallyExclusiveCallbackGroup()
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.imu_cb, 20,
                                                callback_group=self.imu_group)
        # 里程计车速:直行时判断车走起来没有(轮子卡住时不累积积分,松开后不猛甩)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10,
                                                 callback_group=self.imu_group)
        # 直行状态 10 Hz 发出去：作业记录（job_recorder）据此自动记下每次直行
        self.pub_straight = self.create_publisher(String, '/straight/status', 5)
        self.create_timer(0.1, self.publish_straight, callback_group=self.imu_group)
        # HTTP 工作线程只写入最新期望值，ROS executor 线程以 50Hz 稳定发布。
        # 这避免了短按时只有一个 Twist，与底盘 0.5s 看门狗互相抢时序的卡顿。
        # 放进独立回调组:即便别的订阅正在忙,这个 50Hz 定时器也不会被饿死。
        # 执行器是 MultiThreadedExecutor 时才分线程；EventsExecutor 只有一个线程（见 make_executor）。
        self.manual_group = MutuallyExclusiveCallbackGroup()
        self._manual_loop_at = None
        self.manual_timer = self.create_timer(MANUAL_PUBLISH_PERIOD_S, self.manual_loop,
                                              callback_group=self.manual_group)
        # RTK 走直线：状态转给 app，app 的操作指令转给 line_node
        self.sub_gnss = self.create_subscription(String, '/gnss/status', self.gnss_status_cb, 1)
        self.sub_line = self.create_subscription(String, '/line/status', self.line_status_cb, 1)
        self.pub_line = self.create_publisher(String, '/line/command', 10)

        self.get_logger().info('>>> 底盘、RTK、走直线节点状态已连接')

    def wheeltec_cb(self, msg):
        global state
        try:
            d = json.loads(msg.data)
            armed = bool(d.get('armed', False))
            ready = (d.get('ready', '') == 'ready')
            self.is_armed = armed
            d['_rx_monotonic'] = time.monotonic()
            with data_lock:
                state['wheeltec'] = d
        except Exception:
            pass

    @staticmethod
    def call_motion_service(client, request):
        if not client.service_is_ready():
            return False, 'motion service unavailable; update and restart chassis driver'
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(2.0):
            return False, 'motion service response timeout; inspect /motion/status'
        try:
            result = future.result()
            return result.success, result.message
        except Exception as exc:
            return False, str(exc)

    def recover_latched_fault(self):
        """Reset a recovered, stationary FAULT before an explicit new command.

        MotionAuthority deliberately latches FAULT after a transient sensor/driver
        outage. Keeping that latch is important, but the web UI previously had no
        recovery path: every later manual request was rejected even after all
        sensors were healthy again. A fresh user command is explicit intent to
        continue, so let the driver's /motion/reset service perform its own health
        and stationary checks, then retry the command.
        """
        with self.motion.lock:
            current = dict(self.motion.state)
            fresh = self.motion.fresh()

        if current.get('mode') != 'FAULT':
            if current.get('mode') == 'ESTOP':
                return False, '急停已锁存，请现场检查并单独复位'
            return True, ''
        if current.get('reason') == 'reset_failed':
            return False, '底盘复位失败：请现场检查停车原因，确认安全后单独复位'
        if not fresh:
            return False, '运动状态已过期，请检查底盘服务'
        if not current.get('healthy', False):
            faults = current.get('health_faults', [])
            fault_desc = (', '.join(faults)) if faults else '待确认'
            return False, f'传感器或底盘仍有故障 ({fault_desc})，拒绝复位'
        if current.get('waiting_stationary', True):
            return False, '等待车辆确认静止后才能复位'
        return self.call_motion_service(self.cli_reset, Trigger.Request())

    def manual_loop(self):
        watchdog.tick('ros')
        tick = time.monotonic()
        last, self._manual_loop_at = self._manual_loop_at, tick
        if last is not None and tick - last > STALL_LOG_S:
            print(f'web_stall seconds={tick - last:.3f}', file=sys.stderr, flush=True)
        # 复位在 HTTP 线程里拿着 control_lock 等服务应答：这一拍不发（原来也是等锁、不发），
        # 不能在执行器线程里干等——单线程执行器等着就没人收那个服务应答了
        if not self.control_lock.acquire(blocking=False):
            return
        try:
            now = time.monotonic()
            vx, wz, active, publish_zero, hold = self.manual_drive.sample_hold(now)
            if active and hold is not None:
                vx, wz = self.straight.step(hold, now)      # 直行:车速、方向按 IMU 偏航算
            elif not active:
                # 停止、急停、接管都先结束了直行;这里还在跑说明 app 的 go 断了 0.75 s
                self.straight.end('command_timeout')
            if active or publish_zero:
                self.motion.publish(vx if active else 0.0, wz if active else 0.0)
        finally:
            self.control_lock.release()
        with data_lock:
            state['motion'] = dict(self.motion.state)

    def send_manual_twist(self, vx, wz, hold=None):
        self.manual_drive.set(vx, wz, hold=hold)

    def publish_straight(self):
        self.pub_straight.publish(String(data=json.dumps(self.straight.status(), ensure_ascii=False)))

    def imu_cb(self, msg):
        try:
            if float(msg.orientation_covariance[0]) < 0:     # ROS 约定:这条不带姿态
                return
        except (TypeError, IndexError, ValueError, AttributeError):
            pass
        o = msg.orientation
        self.straight.on_imu(yaw_from_quaternion(o.x, o.y, o.z, o.w), msg.angular_velocity.z, time.monotonic())

    def odom_cb(self, msg):
        self.straight.on_odom(msg.twist.twist.linear.x, time.monotonic())

    def _store_status(self, key, msg):
        try:
            d = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(d, dict):
            d['_rx_monotonic'] = time.monotonic()
            with data_lock:
                state[key] = d

    def gnss_status_cb(self, msg):
        self._store_status('gnss', msg)

    def line_status_cb(self, msg):
        self._store_status('line', msg)

    def line_command(self, cmd):
        """app 的 line 指令 -> /line/command；结果由 line_node 写进 /line/status 的 last_result。"""
        payload = {'cmd': cmd['action'], 'id': cmd.get('id')}
        if 'point' in cmd:
            payload['point'] = cmd['point']
        if 'cruise' in cmd:
            payload['cruise'] = cmd['cruise']
        self.pub_line.publish(String(data=json.dumps(payload)))

    def route_command(self, cmd):
        """app 的 route 指令 -> /line/command（route_set/route_start/route_clear），结果同样看 last_result。"""
        payload = {'cmd': 'route_' + cmd['action'], 'id': cmd.get('id')}
        if cmd['action']=='field_start':
            payload['field_plan']=FIELD_PLANS.get(cmd['preview_token'])
            payload['allow_outside']=cmd.get('allow_outside',False)
        for key in ('route', 'route_rev', 'job_id', 'preview_token', 'operation', 'element', 'from_m', 'to_m', 'overlap_m', 'allow_outside'):
            if key in cmd:
                payload[key] = cmd[key]
        self.pub_line.publish(String(data=json.dumps(payload)))

    def voltage_cb(self, msg):
        global state
        try:
            v = float(msg.data)
            pct = max(0, min(100, int(round((v - 19.0) / 6.2 * 100))))
            with data_lock:
                state['voltage'] = round(v, 2)
                state['battery_pct'] = pct
        except Exception:
            pass


def tune_gc():
    """启动完把已有对象冻结（gc.freeze），之后的完整回收只扫新建的对象；一次回收超过 GC_SLOW_S 记 gc_slow 日志。
    2026-09-30 实车：web 进程每隔几秒卡 0.2–0.4 s（只有执行器线程在跑、占着 GIL），IMU 直行误判 IMU 中断、
    平板遥测断续；同样这些模块完整回收一次在小核上就要约 70 ms。"""
    gc.collect()
    gc.freeze()
    started = []

    def watch(phase, info):
        if phase == 'start':
            started[:] = [time.monotonic()]
        elif started:
            seconds = time.monotonic() - started.pop()
            if seconds > GC_SLOW_S:
                print(f'gc_slow generation={info.get("generation")} seconds={seconds:.3f} '
                      f'collected={info.get("collected")} frozen={gc.get_freeze_count()}',
                      file=sys.stderr, flush=True)
    gc.callbacks.append(watch)


def make_executor():
    """rclpy 的 MultiThreadedExecutor 每个事件都在 Python 里调度：web 节点每秒 110–170 个事件（IMU 50 Hz、
    50 Hz 手动下发、odom、电压……），执行器线程平时就占小核 50–60%，和 HTTP/websocket 线程抢 GIL，每隔几秒卡
    0.2–0.4 s（2026-09-30 实车：直行误判 IMU 中断、平板遥测断续）。EventsExecutor 在 C++ 里排队分发，同样的
    订阅和定时器板上实测 8.7%。它只有一个线程：回调里不能等服务应答（这里的回调都只存数据、发指令；
    manual_loop 拿不到 control_lock 就跳过这一拍）。Jazzy 里是实验功能，导入不到就用原来的。"""
    try:
        from rclpy.experimental import EventsExecutor
        executor = EventsExecutor()
    except Exception as exc:
        print(f'ros_executor=MultiThreadedExecutor (EventsExecutor unavailable: {exc})', file=sys.stderr, flush=True)
        return MultiThreadedExecutor(num_threads=2)
    print('ros_executor=EventsExecutor', file=sys.stderr, flush=True)
    return executor


def ros_worker():
    global bridge_node
    # main owns SIGTERM/SIGINT. rclpy's default handler used to stop just this
    # thread, leaving the HTTP server alive until systemd's forced kill.
    from rclpy.signals import SignalHandlerOptions
    from rclpy.executors import ExternalShutdownException
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = WebBridgeNode()
    bridge_node = node
    executor = make_executor()
    executor.add_node(node)
    tune_gc()
    try:
        executor.spin()
    except ExternalShutdownException:
        pass  # expected when main handles SIGTERM and shuts down the ROS context
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

class RadarHTTPHandler(http.server.BaseHTTPRequestHandler):
    # HTTP/1.1 长连接下,每个响应都必须带 Content-Length(或主动关连接),
    # 否则浏览器不知道响应体在哪结束,fetch 会一直挂着占住连接。
    # 手机浏览器对同一主机只有约 6 条连接,遥控心跳每 100ms 一条,几下就占满,
    # 之后的指令(包括刹车)全部在浏览器里排队 —— 表现为"点了没反应,过一会车才动"。
    protocol_version = 'HTTP/1.1'

    def _send_body(self, body, content_type, status=200, extra_headers=None):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        if self.close_connection:
            self.send_header('Connection', 'close')
        self.send_header('Access-Control-Allow-Origin', '*')
        for key, value in (extra_headers or {}).items():
            self.send_header(key, value)
        self.end_headers()
        if body:
            self.wfile.write(body)

    def do_OPTIONS(self):
        self._send_body(b'', 'text/plain', status=204, extra_headers={
            'Access-Control-Allow-Methods': 'GET, POST, OPTIONS',
            'Access-Control-Allow-Headers': 'Content-Type',
            'Access-Control-Max-Age': '600',
        })

    def do_GET(self):
        if self.path == '/api/display':
            if self.headers.get('Origin'):
                self._send_json({'ok': False, 'error': 'origin_not_allowed'}, status=403)
            else:
                self._send_json(dict(SCREEN_CONTROL.status(), ok=True))
            return
        # 车载屏幕只读显示：页面、事件流、车辆模型；不接收指令
        if self.path.startswith('/display') and car_display.handle(
                self, snapshot=snapshot_state, name=CAR_NAME, profile=PROFILE, sessions=APP_SESSIONS,
                network=NETWORK, view=APP_VIEW):
            return
        if self.path == '/' or self.path.startswith('/index'):
            with open(TEMPLATE_PATH, 'rb') as f:
                body = f.read()
            self._send_body(body, 'text/html; charset=utf-8', extra_headers={
                'Cache-Control': 'no-cache, no-store, must-revalidate',
                'Pragma': 'no-cache', 'Expires': '0'})

        elif self.path.startswith('/static/'):
            clean_rel = self.path.lstrip('/').split('?')[0]
            static_file = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), clean_rel))
            static_root = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static'))
            if static_file.startswith(static_root + os.sep) and os.path.isfile(static_file):
                if static_file.endswith('.json'):
                    kind = 'application/json; charset=utf-8'
                elif static_file.endswith('.js'):
                    kind = 'application/javascript; charset=utf-8'
                elif static_file.endswith('.css'):
                    kind = 'text/css; charset=utf-8'
                else:
                    kind = 'application/octet-stream'
                with open(static_file, 'rb') as f:
                    self._send_body(f.read(), kind)
                return
            else:
                self.send_error(404)
                return

        elif self.path == '/api/rtk/cors':
            if self.headers.get('Origin'):
                self._send_json({'ok': False, 'error': 'origin_not_allowed'}, status=403)
                return
            try:
                result = CORS_CONFIG.public()
                snap = snapshot_state()
                rtk = app_link.rtk_telemetry(snap.get('gnss'), time.monotonic())
                result.update(ok=True, supported=True, can_apply=configuration_allowed(snap, time.monotonic()),
                              actions=['disconnect', 'connect'], ntrip=rtk.get('ntrip'), rtk_stale=rtk['stale'])
                self._send_json(result)
            except (OSError, ValueError, TypeError):
                self._send_json({'ok': False, 'supported': True, 'error': 'config_unreadable'}, status=500)

        elif self.path == '/app/beacon':
            # 内容与 UDP 发现广播相同。手机连车载热点时 app 用它确认"网关就是车"，别的网关不会这样回
            self._send_body(app_link.beacon(CAR_NAME, PORT, profile_hash(PROFILE)), 'application/json')

        elif self.path == '/ws':
            self._serve_app_socket()

        elif self.path == '/api/stream':
            # 事件流没有长度,靠断开连接结束;明确声明,不让这条连接被当作可复用的长连接
            self.close_connection = True
            self.send_response(200)
            self.send_header('Content-type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            try:
                while True:
                    payload = json.dumps(snapshot_state())
                    self.wfile.write(f"data: {payload}\n\n".encode('utf-8'))
                    self.wfile.flush()
                    time.sleep(0.10)     # 10 Hz 够网页显示，别让手机浏览器解析太多 JSON
            except Exception:
                pass
        else:
            self.send_error(404)

    def _serve_app_socket(self):
        """平板 app 长连接:下行 hello + 10Hz 遥测,上行摇杆/直行/急停/作业指令。"""
        key = self.headers.get('Sec-WebSocket-Key')
        if not key or 'websocket' not in (self.headers.get('Upgrade') or '').lower():
            self.send_error(400, 'WebSocket upgrade required')
            return
        self.close_connection = True
        self.send_response(101, 'Switching Protocols')
        self.send_header('Upgrade', 'websocket')
        self.send_header('Connection', 'Upgrade')
        self.send_header('Sec-WebSocket-Accept', app_link.accept_key(key))
        self.end_headers()
        self.wfile.flush()
        self.connection.settimeout(APP_RECV_TIMEOUT_S)
        ws = app_link.WebSocket(self.rfile, self.wfile)
        session = 'app-' + uuid.uuid4().hex[:10]
        done = threading.Event()
        drove = False
        APP_SESSIONS.enter()

        def pump():
            seq = 0
            try:
                ws.send_json(app_link.build_hello(PROFILE, name=CAR_NAME, session=session,
                                                  network=CAR_NETWORK.hotspot_info()))
                while not done.is_set():
                    seq += 1
                    ws.send_json(app_link.build_telemetry(snapshot_state(), now=time.monotonic(), seq=seq))
                    done.wait(APP_TELEMETRY_PERIOD_S)
            except Exception:
                pass
            finally:
                done.set()

        def reply(cmd, ok, message=''):
            try:
                ws.send_json({'type': 'ack', 'id': cmd.get('id'), 'cmd': cmd.get('type'),
                              'ok': bool(ok), 'message': message})
            except app_link.WebSocketClosed:
                pass

        def slow(cmd, fn):
            # 复位要等服务应答(最多 2 s),不能堵住摇杆和急停的接收
            threading.Thread(target=lambda: reply(cmd, *fn()), daemon=True).start()

        threading.Thread(target=pump, daemon=True).start()
        try:
            while not done.is_set():
                try:
                    cmd = app_link.parse_command(ws.recv())
                except app_link.CommandError as exc:
                    reply({}, False, str(exc))
                    continue
                kind = cmd['type']
                if kind == 'drive':
                    vx, wz, steer_deg = app_link.resolve_stick(
                        cmd['throttle'], cmd['steer'], cmd['tier'], SPEED_TIERS_MPS,
                        REVERSE_SCALE, CHASSIS)
                    status, result = apply_manual(vx, wz, steer_deg, 'stick', session, cmd['seq'])
                    drove = drove or abs(vx) > 1e-6 or abs(wz) > 1e-6     # 差速车原地转也算开过
                    if status != 200:
                        reply(cmd, False, result.get('error', ''))
                elif kind == 'straight':
                    # IMU 定向直行:go 每 50 ms 一条(同一个 run 接着开,新 run 按当时的 IMU 偏航定目标),
                    # stop 停车;停没停、为什么停看遥测 straight
                    if bridge_node is None:
                        reply(cmd, False, 'motion bridge unavailable')
                    elif cmd['action'] == 'go':
                        status, result = apply_manual(bridge_node.straight.cfg.speed, 0.0, 0.0, 'straight',
                                                      session, cmd['seq'], hold=(session, cmd['run']))
                        drove = True
                        if status != 200:
                            reply(cmd, False, result.get('error', ''))
                    else:
                        bridge_node.straight.end('user')
                        status, result = apply_manual(0.0, 0.0, 0.0, 'straight', session, cmd['seq'])
                        reply(cmd, status == 200, result.get('error', ''))
                elif kind == 'estop':
                    reply(cmd, *emergency_stop())
                elif kind == 'reset':
                    slow(cmd, motion_reset)
                elif kind == 'line':
                    if bridge_node is None:
                        reply(cmd, False, 'motion bridge unavailable')
                    else:
                        # ack 只表示已转发；执行结果看遥测 line.last_result（按 id 对应）
                        bridge_node.line_command(cmd)
                        reply(cmd, True, 'forwarded')
                elif kind == 'route':
                    if bridge_node is None:
                        reply(cmd, False, 'motion bridge unavailable')
                    else:
                        try:
                            bridge_node.route_command(cmd)
                            reply(cmd, True, 'forwarded')
                        except ValueError as exc:
                            reply(cmd,False,str(exc))
                elif kind == 'view':
                    APP_VIEW.set(session, cmd['file'])
                elif kind == 'network':
                    # 切换车的 WiFi 方式：先回 ack，1.5 s 后才动网卡；切过去 2 分钟没有平板连上来自动切回
                    problem = app_link.network_switch_blocker(snapshot_state(), time.monotonic())
                    if problem:
                        reply(cmd, False, problem)
                    else:
                        reply(cmd, *CAR_NETWORK.request(cmd['mode']))
                elif kind == 'ping':
                    ws.send_json({'type': 'pong', 't': cmd.get('t'), 'server': time.monotonic()})
        except (app_link.WebSocketClosed, OSError):
            pass
        finally:
            done.set()
            APP_SESSIONS.leave()
            APP_VIEW.clear(session)
            if drove and bridge_node is not None:
                # 断线的那一刻就停,不等 0.75s 心跳超时
                run = bridge_node.straight.run
                if isinstance(run, tuple) and run[0] == session:
                    bridge_node.straight.end('disconnected')
                bridge_node.send_manual_twist(0.0, 0.0)
            ws.close()

    def _send_json(self, data, status=200):
        body = json.dumps(data, ensure_ascii=False).encode('utf-8')
        self._send_body(body, 'application/json; charset=utf-8', status=status,
                        extra_headers={'Cache-Control': 'no-store'})

    def _read_body(self, limit=65536):
        size = int(self.headers.get('Content-Length', 0) or 0)
        if not 0 <= size <= limit:
            raise ValueError('请求体过大')
        return self.rfile.read(size) if size else b''

    def do_POST(self):
        global bridge_node
        if self.path == '/api/field/plan':
            if self.headers.get('Origin'):
                self.close_connection = True
                self._send_json({'ok': False, 'message': '不接受网页跨源规划请求'}, status=403)
                return
            try:
                request = json.loads(self._read_body(262144).decode('utf-8'))
                result = FIELD_PLANS.prepare(request, snapshot_state().get('line'), time.monotonic())
                self._send_json({'ok': True, 'plan': result})
            except (ValueError, KeyError, TypeError, UnicodeError) as exc:
                self.close_connection = True
                self._send_json({'ok': False, 'message': str(exc)}, status=400)
            return
        if self.path == '/api/display':
            if self.headers.get('Origin'):
                self.close_connection = True
                self._send_json({'ok': False, 'error': 'origin_not_allowed'}, status=403)
                return
            try:
                req = json.loads(self._read_body(1024).decode('utf-8'))
                if not isinstance(req, dict) or set(req) != {'action'} or req['action'] not in ('desktop', 'show'):
                    raise ValueError('invalid_action')
                reply = SCREEN_CONTROL.request(req['action'])
                self._send_json(dict(reply, ok=True, status='pending'))
            except (ValueError, TypeError, UnicodeError):
                self._send_json({'ok': False, 'error': 'invalid_action'}, status=400)
            except (OSError, RuntimeError):
                self._send_json({'ok': False, 'error': 'window_not_running'}, status=503)
            return
        if self.path == '/api/rtk/cors':
            # 拒绝网页跨源写入；账号只在正文接收，不放 URL、日志或响应里。
            if self.headers.get('Origin'):
                self.close_connection = True
                self._send_json({'ok': False, 'error': 'origin_not_allowed'}, status=403)
                return
            try:
                req = json.loads(self._read_body(8192).decode('utf-8'))
                if not configuration_allowed(snapshot_state(), time.monotonic()):
                    self._send_json({'ok': False, 'error': 'vehicle_not_idle',
                                     'message': '请停车并退出自动作业，等待底盘在线后应用'}, status=409)
                    return
                if isinstance(req, dict) and 'action' in req:
                    action = req['action']
                    if set(req) != {'action'} or action not in ('disconnect', 'connect'):
                        raise ValueError('不支持的差分连接操作')
                    if not CORS_CONFIG.public()['configured']:
                        self._send_json({'ok': False, 'error': 'not_configured', 'message': '请先应用车端账号'}, status=400)
                        return
                    saved = CORS_CONFIG.set_enabled(action == 'connect')
                else:
                    saved = CORS_CONFIG.save(req)
                # 保存成功不代表 CORS 已连通；RTK 节点加载后通过遥测确认 revision。
                self._send_json(dict(saved, ok=True, status='pending_reload'))
            except (ValueError, TypeError, UnicodeError):
                self.close_connection = True
                self._send_json({'ok': False, 'error': 'invalid_config', 'message': '账号参数格式无效'}, status=400)
            except OSError:
                self._send_json({'ok': False, 'error': 'save_failed', 'message': '车端保存失败'}, status=500)

        elif self.path == '/api/drive_profile':
            self._send_json({
                'speed_tiers': SPEED_TIERS_MPS,
                'steer_tiers': STEER_TIERS_DEG,
                'reverse_scale': REVERSE_SCALE,
                'min_turn_radius_m': round(CHASSIS.min_turn_radius_m, 3),
                'max_steer_deg': round(math.degrees(CHASSIS.max_steer_rad), 1),
            })

        elif self.path in ('/api/motion/stop', '/api/motion/reset'):
            if bridge_node is None:
                self._send_json({'ok': False, 'message': 'motion bridge unavailable'}, status=503)
                return
            ok, message = emergency_stop() if self.path.endswith('/stop') else motion_reset()
            self._send_json({'ok': ok, 'message': message})

        elif self.path == '/api/manual_drive':
            try:
                req = json.loads(self._read_body(4096).decode('utf-8'))
                if not isinstance(req, dict):
                    raise ValueError('指令格式错误')
                action = req.get('action', 'custom')
                steer_deg = 0.0

                if 'direction' in req:
                    # 新接口:前端只报方向键与两个档位,速度/角速度由后端权威换算
                    vx, wz, steer_deg = resolve_drive(
                        req.get('direction', 'stop'),
                        req.get('speed_tier', 'med'),
                        req.get('steer_tier', 'normal'))
                    action = req.get('direction', action)
                else:
                    # 旧接口保留兼容,但要把物理上做不到的角速度掐掉:
                    # 超过满舵能达到的 wz 只会让固件把舵机打死,反而丢失档位区分度
                    vx, wz = float(req.get('vx', 0.0)), float(req.get('wz', 0.0))
                    if not all(math.isfinite(v) for v in (vx, wz)):
                        raise ValueError('速度必须是有限数值')
                    vx = max(-0.6, min(1.0, vx))
                    wz = max(-1.5, min(1.5, wz))
                    steer_rad = steer_from_yaw(vx, wz, CHASSIS)
                    wz = yaw_from_steer(vx, steer_rad, CHASSIS)
                    steer_deg = round(math.degrees(steer_rad), 1)

                client_id = str(req.get('client_id', ''))[:64] or None
                seq_raw = req.get('seq')
                seq = int(seq_raw) if isinstance(seq_raw, (int, float)) and math.isfinite(seq_raw) else None
                status, reply = apply_manual(vx, wz, steer_deg, action, client_id, seq)
                self._send_json(reply, status=status)
            except (ValueError, TypeError, KeyError) as e:
                self._send_json({'ok': False, 'error': str(e)}, status=400)
            except Exception as e:
                self._send_json({'ok': False, 'error': str(e)}, status=500)
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        return

class ThreadedHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True

def main():
    import signal
    stopping = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    watchdog.start('http', 'ros')
    server = ThreadedHTTPServer(('0.0.0.0', PORT), RadarHTTPHandler)
    server.timeout = 0.25
    t = threading.Thread(target=ros_worker, daemon=True)
    t.start()
    threading.Thread(target=discovery_beacon, daemon=True).start()
    NETWORK.start()
    CAR_NETWORK.start()
    try:
        print(f"网页服务: http://0.0.0.0:{PORT}")
        while not stopping.is_set():
            if not t.is_alive():
                raise RuntimeError('ROS worker exited while HTTP server was running')
            server.handle_request()
            watchdog.tick('http')
    finally:
        watchdog.close()
        server.server_close()
        if rclpy.ok():
            rclpy.shutdown()
        t.join(timeout=2.0)

if __name__ == '__main__':
    main()
