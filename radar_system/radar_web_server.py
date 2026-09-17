#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RK3588 雷达与摄像头人体跟踪控制台
- 运行端口: 8088
- 汇聚: /scan、人体姿态检测、跟随状态与底盘遥测
"""

import http.server
import socketserver
import json
import os
import math
import time
import threading
import subprocess
import signal
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
from std_srvs.srv import SetBool, Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from motion_safety import ChassisGeometry, yaw_from_steer, steer_from_yaw, clamp
from scan_utils import clean_ranges, sector_min
from manual_drive import ManualDriveLatch

PORT = 8088
MANUAL_PUBLISH_PERIOD_S = 0.02
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

# 与页面按钮的标称保持一致；高档仍低于驱动层 1.30m/s 硬上限。
SPEED_TIERS_MPS = {'low': 0.50, 'med': 0.85, 'high': 1.20}
REVERSE_SCALE = 0.6                      # 倒车是盲区方向,统一降速
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
    'connected': True,
    'hz': 10.0,
    'points_count': 360,
    'front_dist': 2.5,
    'left_dist': 2.0,
    'back_dist': 2.5,
    'right_dist': 2.0,
    'min_dist': 1.8,
    'ranges': [],
    'ai_targets': [],
    'ai_status': {},
    # 动力电池状态
    'voltage': 0.0,
    'battery_pct': 0,
    # 电子跟屁虫状态与运行标识
    'follower': {'state': 'OFFLINE'},
    'follower_running': False,
    'manual_override': {'active': False, 'action': 'none', 'vx': 0.0, 'wz': 0.0}
}

frame_count = 0
last_hz_calc = time.time()

bridge_node = None
follower_proc = None
last_check_proc = 0.0
cached_proc_running = False

FOLLOWER_STALE_S = 1.5      # 跟随节点 20Hz 发状态,1.5 秒没消息即判为已停


def mark_follower_staleness(payload):
    """给跟随状态打上 stale 标记,陈旧时清掉会误导人的字段。

    没有这道处理,节点挂掉后界面会一直显示它最后那一刻的 AEB 状态与测距,
    看起来就像"雷达一直在报前方有障碍物",实际上节点早就不在了。
    """
    if not payload:
        return payload
    rx = payload.get('_rx_monotonic')
    age = (time.monotonic() - rx) if rx else None
    stale = (age is None) or (age > FOLLOWER_STALE_S)
    payload['stale'] = stale
    payload['age_s'] = round(age, 2) if age is not None else None
    if stale:
        # 这些值只在节点活着时才有意义,陈旧时必须清掉而不是接着显示
        payload['aeb_active'] = False
        payload['aeb_min_scan_m'] = None
        payload['path_clearance_m'] = None
        payload['target'] = None
        payload['state'] = 'OFFLINE'
    return payload


def is_follower_running():
    global follower_proc
    if follower_proc is not None and follower_proc.poll() is None:
        return True
    try:
        out = subprocess.check_output(['pgrep', '-f', 'person_follower.py']).decode().strip()
        return len(out) > 0
    except Exception:
        return False

def check_follower_running_cached():
    global last_check_proc, cached_proc_running
    now = time.time()
    if now - last_check_proc > 0.5:
        last_check_proc = now
        cached_proc_running = is_follower_running()
    return cached_proc_running

def start_follower():
    global follower_proc
    if is_follower_running():
        return True, "already_running"
    script_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'person_follower.py')
    cmd = [sys.executable, "-u", script_path]
    env = os.environ.copy()
    try:
        follower_proc = subprocess.Popen(
            cmd,
            cwd=os.path.dirname(os.path.abspath(__file__)),
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )
        return True, "started"
    except Exception as e:
        return False, str(e)

def stop_follower():
    global follower_proc, cached_proc_running, last_check_proc
    try:
        subprocess.run(['pkill', '-9', '-f', 'person_follower.py'], check=False)
    except Exception:
        pass
    if follower_proc is not None:
        try:
            os.killpg(os.getpgid(follower_proc.pid), signal.SIGKILL)
        except Exception:
            pass
        follower_proc = None
    cached_proc_running = False
    last_check_proc = time.time()
    if bridge_node:
        bridge_node.send_manual_twist(0.0, 0.0)
    with data_lock:
        state['follower'] = {'state': 'OFFLINE', 'aeb_min_scan_m': state.get('front_dist', 99.0)}
    return True, "stopped"


class TrackingBridgeNode(Node):
    def __init__(self):
        super().__init__('radar_tracking_web_bridge')
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.sub_ai = self.create_subscription(String, '/camera/ai_detection/targets', self.ai_cb, 10)
        self.sub_ai_status = self.create_subscription(String, '/camera/ai_detection/status', self.ai_status_cb, 1)
        self.sub_voltage = self.create_subscription(Float32, '/voltage', self.voltage_cb, 10)
        self.sub_follower = self.create_subscription(String, '/follower/status', self.follower_cb, 10)
        self.sub_wheeltec = self.create_subscription(String, '/wheeltec/status', self.wheeltec_cb, 10)

        # 手动介入控制发布者与底盘解锁使能客户端
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cli_arm = self.create_client(SetBool, '/wheeltec/arm')
        self.cli_stop = self.create_client(Trigger, '/wheeltec/stop')
        self.is_armed = False
        self.last_arm_request = 0.0

        self.manual_drive = ManualDriveLatch(MANUAL_HEARTBEAT_TIMEOUT_S)
        # HTTP 工作线程只写入最新期望值，ROS executor 线程以 50Hz 稳定发布。
        # 这避免了短按时只有一个 Twist，与底盘 0.5s 看门狗互相抢时序的卡顿。
        # 放进独立回调组:即便别的订阅正在忙,这个 50Hz 定时器也不会被饿死。
        # 配合 main() 里的 MultiThreadedExecutor 使用。
        self.manual_group = MutuallyExclusiveCallbackGroup()
        self.manual_timer = self.create_timer(MANUAL_PUBLISH_PERIOD_S, self.manual_loop,
                                              callback_group=self.manual_group)

        self.get_logger().info('>>> 雷达、人体姿态、跟随与底盘遥测已连接')

    def wheeltec_cb(self, msg):
        global state
        try:
            d = json.loads(msg.data)
            armed = bool(d.get('armed', False))
            ready = (d.get('ready', '') == 'ready')
            self.is_armed = armed
            with data_lock:
                state['wheeltec'] = d
            now = time.monotonic()
            if not armed and ready and (now - self.last_arm_request > 0.50):
                self.last_arm_request = now
                self.arm_chassis(True)
        except Exception:
            pass

    def arm_chassis(self, enable=True):
        if self.cli_arm.service_is_ready():
            req = SetBool.Request()
            req.data = enable
            self.last_arm_request = time.monotonic()
            self.cli_arm.call_async(req)

    def manual_loop(self):
        now = time.monotonic()
        vx, wz, active, publish_zero = self.manual_drive.sample(now)

        if active:
            if not self.is_armed and now - self.last_arm_request > 0.10:
                self.arm_chassis(True)
            cmd = Twist()
            cmd.linear.x = float(vx)
            cmd.angular.z = float(wz)
            self.pub_cmd_vel.publish(cmd)
        elif publish_zero:
            self.pub_cmd_vel.publish(Twist())

    def send_manual_twist(self, vx, wz):
        self.manual_drive.set(vx, wz)

    def follower_cb(self, msg):
        global state
        try:
            d = json.loads(msg.data)
            # 记录收到时刻。跟随节点被 Ctrl-C 或崩溃时不会发"我停了",
            # 最后一条消息会永远冻在界面上 —— AEB 横幅一直亮、正前测距
            # 一直显示那个旧值,哪怕雷达实时看到的是 2 米开外。
            d['_rx_monotonic'] = time.monotonic()
            with data_lock:
                state['follower'] = d
        except Exception:
            pass

    def voltage_cb(self, msg):
        global state
        try:
            v = float(msg.data)
            pct = max(0, min(100, int(round((v - 21.0) / 4.2 * 100))))
            with data_lock:
                state['voltage'] = round(v, 2)
                state['battery_pct'] = pct
        except Exception:
            pass


    def ai_cb(self, msg):
        global state
        try:
            items = json.loads(msg.data)
            with data_lock:
                state['ai_targets'] = items
        except Exception:
            pass


    def ai_status_cb(self, msg):
        try:
            with data_lock:
                state['ai_status'] = json.loads(msg.data)
        except (TypeError, ValueError):
            pass

    def scan_cb(self, msg):
        global state, frame_count, last_hz_calc
        frame_count += 1
        now = time.time()
        if now - last_hz_calc >= 1.0:
            hz = round(frame_count / (now - last_hz_calc), 1)
            frame_count = 0
            last_hz_calc = now
        else:
            hz = state['hz']

        n = len(msg.ranges)
        if n == 0: return

        # 一次转换、一个掩码,四个方位与整体最小值都从同一份数组上取,
        # 避免原来每个方位各跑一遍 Python 循环、再多跑两遍全量遍历。
        clean, ok = clean_ranges(msg.ranges, msg.range_min, msg.range_max)
        # 必须把 angle_min 传进去。LaserScan 第 0 个光束指向 msg.angle_min
        # 而不是 0°,N10P 发布 -π,不传的话「正前方测距」读的其实是车尾 ——
        # 雷达扫到车自己的车身,会被当成正前方 0.17m 的障碍物。
        amin_deg = math.degrees(msg.angle_min)
        front_d = sector_min(clean, 345, 15, angle_min_deg=amin_deg)
        left_d = sector_min(clean, 75, 105, angle_min_deg=amin_deg)
        back_d = sector_min(clean, 165, 195, angle_min_deg=amin_deg)
        right_d = sector_min(clean, 255, 285, angle_min_deg=amin_deg)
        overall_min = float(np.nanmin(clean)) if bool(np.any(ok)) else 99.0
        ranges_out = np.round(np.where(ok, clean, 0.0), 2).tolist()

        with data_lock:
            state['hz'] = hz
            state['points_count'] = n
            state['front_dist'] = round(front_d, 2)
            state['left_dist'] = round(left_d, 2)
            state['back_dist'] = round(back_d, 2)
            state['right_dist'] = round(right_d, 2)
            state['min_dist'] = round(overall_min, 2)
            state['ranges'] = ranges_out
            state['angle_min'] = msg.angle_min
            state['angle_increment'] = msg.angle_increment



def ros_worker():
    global bridge_node
    try:
        rclpy.init()
    except Exception:
        pass
    node = TrackingBridgeNode()
    bridge_node = node
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except Exception:
        pass
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

        elif self.path == '/api/follower/status':
            with data_lock:
                f_data = dict(state.get('follower', {}))
            f_data = mark_follower_staleness(f_data)
            f_data['running'] = check_follower_running_cached()
            self._send_json(f_data)

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
                    # pgrep 是子进程调用,不能放在 data_lock 里,否则会卡住 ROS 回调和其他请求
                    running = check_follower_running_cached()
                    with data_lock:
                        state['follower_running'] = running
                        snapshot = dict(state)
                        snapshot['follower'] = mark_follower_staleness(
                            dict(snapshot.get('follower', {})))
                        payload = json.dumps(snapshot)
                    self.wfile.write(f"data: {payload}\n\n".encode('utf-8'))
                    self.wfile.flush()
                    time.sleep(0.08) # ~12 FPS
            except Exception:
                pass
        else:
            self.send_error(404)

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
        if self.path == '/api/drive_profile':
            self._send_json({
                'speed_tiers': SPEED_TIERS_MPS,
                'steer_tiers': STEER_TIERS_DEG,
                'reverse_scale': REVERSE_SCALE,
                'min_turn_radius_m': round(CHASSIS.min_turn_radius_m, 3),
                'max_steer_deg': round(math.degrees(CHASSIS.max_steer_rad), 1),
            })

        elif self.path == '/api/follower/start':
            ok, msg = start_follower()
            with data_lock:
                f_data = mark_follower_staleness(dict(state.get('follower', {})))
            self._send_json({'ok': ok, 'message': msg, 'running': is_follower_running(), 'follower': f_data})

        elif self.path == '/api/follower/stop':
            ok, msg = stop_follower()
            with data_lock:
                f_data = mark_follower_staleness(dict(state.get('follower', {})))
            self._send_json({'ok': ok, 'message': msg, 'running': is_follower_running(), 'follower': f_data})

        elif self.path == '/api/follower/toggle':
            if is_follower_running():
                ok, msg = stop_follower()
            else:
                ok, msg = start_follower()
            with data_lock:
                f_data = mark_follower_staleness(dict(state.get('follower', {})))
            self._send_json({'ok': ok, 'message': msg, 'running': is_follower_running(), 'follower': f_data})

        elif self.path == '/api/manual_drive':
            try:
                req = json.loads(self._read_body(4096).decode('utf-8'))
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
                    vx = max(-0.6, min(1.0, float(req.get('vx', 0.0))))
                    wz = max(-1.5, min(1.5, float(req.get('wz', 0.0))))
                    steer_rad = steer_from_yaw(vx, wz, CHASSIS)
                    wz = yaw_from_steer(vx, steer_rad, CHASSIS)
                    steer_deg = round(math.degrees(steer_rad), 1)

                # 按页面会话 + 序号丢弃迟到的旧指令:网络抖动时,先发的「前进」
                # 可能晚于「刹车」到达,不能让它在松手后又把车开起来。
                client_id = str(req.get('client_id', ''))[:64] or None
                seq = req.get('seq')
                seq = int(seq) if isinstance(seq, (int, float)) and math.isfinite(seq) else None
                if bridge_node and not bridge_node.manual_drive.accept(client_id, seq, vx, wz):
                    self._send_json({'ok': True, 'stale': True, 'action': action})
                    return

                # 强行介入：若自动跟随正在运行，强行介入必须先停掉自动跟随！
                if check_follower_running_cached():
                    stop_follower()

                if bridge_node:
                    if not bridge_node.is_armed and (abs(vx) > 1e-4 or abs(wz) > 1e-4):
                        bridge_node.arm_chassis(True)
                    bridge_node.send_manual_twist(vx, wz)

                with data_lock:
                    state['manual_override'] = {
                        'active': True,
                        'action': action,
                        'vx': vx,
                        'wz': wz,
                        'steer_deg': steer_deg,
                        'time': time.time()
                    }

                self._send_json({'ok': True, 'action': action, 'vx': vx,
                                 'wz': wz, 'steer_deg': steer_deg})
            except Exception as e:
                self._send_json({'ok': False, 'error': str(e)}, status=500)
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        return

class ThreadedHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True

def main():
    t = threading.Thread(target=ros_worker, daemon=True)
    t.start()
    server = ThreadedHTTPServer(('0.0.0.0', PORT), RadarHTTPHandler)
    print(f"雷达 / 摄像头跟踪服务: http://0.0.0.0:{PORT}")
    server.serve_forever()

if __name__ == '__main__':
    main()
