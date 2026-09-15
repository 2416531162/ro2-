#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RK3588 激光雷达与 2D/3D 空间 SLAM 实时建模 Web 控制大屏
- 运行端口: 8088
- 汇聚: /scan (雷达点云) + /map (占据栅格地图) + /robot_pose (机器人位姿与轨迹)
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
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Float32
from std_srvs.srv import SetBool, Trigger

PORT = 8088
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
    'robot_x': 0.0,
    'robot_y': 0.0,
    'robot_yaw': 0.0,
    'trajectory': [],
    # AI 3D 目标检测
    'ai_targets': [],
    # 联合 3D 体素建图状态
    'mapping_status': {},
    # 地图数据
    'map_width': 0,
    'map_height': 0,
    'map_res': 0.05,
    'map_origin_x': -4.0,
    'map_origin_y': -3.5,
    'map_data': [], # 稀疏压缩后的地图
    'rtk': {},
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

CORS_CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cors_config.json')
bridge_node = None
follower_proc = None
last_check_proc = 0.0
cached_proc_running = False

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


class SLAMBridgeNode(Node):
    def __init__(self):
        super().__init__('radar_slam_web_bridge')
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 5)
        self.sub_proj_map = self.create_subscription(OccupancyGrid, '/projected_map', self.map_cb, 5)
        self.sub_pose = self.create_subscription(PoseStamped, '/robot_pose', self.pose_cb, 10)
        self.sub_ai = self.create_subscription(String, '/camera/ai_detection/targets', self.ai_cb, 10)
        self.sub_stat = self.create_subscription(String, '/joint_mapping/status', self.stat_cb, 10)
        self.sub_rtk = self.create_subscription(String, '/rtk/status', self.rtk_cb, 10)
        self.sub_voltage = self.create_subscription(Float32, '/voltage', self.voltage_cb, 10)
        self.sub_follower = self.create_subscription(String, '/follower/status', self.follower_cb, 10)
        self.sub_wheeltec = self.create_subscription(String, '/wheeltec/status', self.wheeltec_cb, 10)
        self.pub_cors_cmd = self.create_publisher(String, '/rtk/cors_cmd', 10)

        # 手动介入控制发布者与底盘解锁使能客户端
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cli_arm = self.create_client(SetBool, '/wheeltec/arm')
        self.cli_stop = self.create_client(Trigger, '/wheeltec/stop')
        self.is_armed = False
        self.last_arm_request = 0.0

        self.manual_vx = 0.0
        self.manual_wz = 0.0
        self.manual_last_cmd_time = 0.0
        self.manual_timer = self.create_timer(0.05, self.manual_loop)

        self.get_logger().info('>>> [SLAM Web Bridge] 已订阅 /scan, /voltage, /follower/status, /wheeltec/status, /map, /projected_map, /robot_pose, AI, 3D 建图与 RTK 话题 (含CORS与底盘手动控制)...')

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
            if not armed and ready and (now - self.last_arm_request > 1.5):
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
        now = time.time()
        # 只要最近 0.6 秒内有收到长按心跳，且速度非零，以 20Hz 频率持续下发平滑推力
        if (now - self.manual_last_cmd_time <= 0.60) and (abs(self.manual_vx) > 1e-4 or abs(self.manual_wz) > 1e-4):
            cmd = Twist()
            cmd.linear.x = float(self.manual_vx)
            cmd.angular.z = float(self.manual_wz)
            self.pub_cmd_vel.publish(cmd)
        elif abs(self.manual_vx) > 1e-4 or abs(self.manual_wz) > 1e-4:
            # 超时看门狗自动刹停
            self.manual_vx = 0.0
            self.manual_wz = 0.0
            cmd = Twist()
            self.pub_cmd_vel.publish(cmd)

    def send_manual_twist(self, vx, wz):
        if abs(vx) < 1e-4:
            wz = 0.0
        self.manual_vx = float(vx)
        self.manual_wz = float(wz)
        self.manual_last_cmd_time = time.time() if (abs(vx) > 1e-4 or abs(wz) > 1e-4) else 0.0

        if abs(vx) > 1e-4 or abs(wz) > 1e-4:
            self.arm_chassis(True)
        cmd = Twist()
        cmd.linear.x = float(self.manual_vx)
        cmd.angular.z = float(self.manual_wz)
        self.pub_cmd_vel.publish(cmd)

    def follower_cb(self, msg):
        global state
        try:
            d = json.loads(msg.data)
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

    def rtk_cb(self, msg):
        global state
        try:
            r = json.loads(msg.data)
            with data_lock:
                state["rtk"] = r
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

    def stat_cb(self, msg):
        global state
        try:
            stat = json.loads(msg.data)
            with data_lock:
                state['mapping_status'] = stat
        except Exception:
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

        def get_min_range(start_deg, end_deg):
            dists = []
            for deg in range(start_deg, end_deg + 1):
                idx = int((deg % 360) / 360.0 * n)
                if 0 <= idx < n:
                    r = msg.ranges[idx]
                    if msg.range_min < r < msg.range_max:
                        dists.append(r)
            return min(dists) if dists else 99.0

        front_d = min(get_min_range(345, 360), get_min_range(0, 15))
        left_d = get_min_range(75, 105)
        back_d = get_min_range(165, 195)
        right_d = get_min_range(255, 285)

        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        overall_min = min(valid_ranges) if valid_ranges else 99.0

        with data_lock:
            state['hz'] = hz
            state['points_count'] = n
            state['front_dist'] = round(front_d, 2)
            state['left_dist'] = round(left_d, 2)
            state['back_dist'] = round(back_d, 2)
            state['right_dist'] = round(right_d, 2)
            state['min_dist'] = round(overall_min, 2)
            state['ranges'] = [round(float(r), 2) if msg.range_min < r < msg.range_max else 0.0 for r in msg.ranges]

    def pose_cb(self, msg):
        global state
        x = msg.pose.position.x
        y = msg.pose.position.y
        # 从四元数计算 yaw
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)

        with data_lock:
            state['robot_x'] = round(x, 3)
            state['robot_y'] = round(y, 3)
            state['robot_yaw'] = round(yaw, 3)
            state['trajectory'].append([round(x, 2), round(y, 2)])
            if len(state['trajectory']) > 200:
                state['trajectory'].pop(0)

    def map_cb(self, msg):
        global state
        w = msg.info.width
        h = msg.info.height
        # 提取已探索的稀疏点集以减小 JSON 大小：
        # 100 为障碍物坐标，0 为可行走自由空间
        obstacles = []
        frees = []
        step = 2 # 抽样提升渲染流畅度
        for gy in range(0, h, step):
            for gx in range(0, w, step):
                val = msg.data[gy * w + gx]
                if val == 100:
                    obstacles.append([gx, gy])
                elif val == 0:
                    frees.append([gx, gy])

        with data_lock:
            state['map_width'] = w
            state['map_height'] = h
            state['map_res'] = msg.info.resolution
            state['map_origin_x'] = msg.info.origin.position.x
            state['map_origin_y'] = msg.info.origin.position.y
            state['map_obstacles'] = obstacles
            state['map_frees'] = frees

def ros_worker():
    global bridge_node
    try:
        rclpy.init()
    except Exception:
        pass
    node = SLAMBridgeNode()
    bridge_node = node
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

class RadarHTTPHandler(http.server.BaseHTTPRequestHandler):
    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def do_GET(self):
        if self.path == '/' or self.path.startswith('/index'):
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Expires', '0')
            self.end_headers()
            with open(TEMPLATE_PATH, 'rb') as f:
                self.wfile.write(f.read())

        elif self.path == '/api/cors':
            self.send_response(200)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            cfg = {}
            if os.path.exists(CORS_CONFIG_PATH):
                try:
                    with open(CORS_CONFIG_PATH, 'r', encoding='utf-8') as f:
                        cfg = json.load(f)
                except Exception:
                    pass
            with data_lock:
                rtk_status = state.get('rtk', {})
                cors_stat = rtk_status.get('cors', {})
            res = {
                'config': cfg,
                'status': cors_stat
            }
            self.wfile.write(json.dumps(res, ensure_ascii=False).encode('utf-8'))

        elif self.path == '/api/follower/status':
            with data_lock:
                f_data = dict(state.get('follower', {}))
            f_data['running'] = check_follower_running_cached()
            self._send_json(f_data)

        elif self.path == '/api/stream':
            self.send_response(200)
            self.send_header('Content-type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            try:
                while True:
                    with data_lock:
                        state['follower_running'] = check_follower_running_cached()
                        payload = json.dumps(state)
                    self.wfile.write(f"data: {payload}\n\n".encode('utf-8'))
                    self.wfile.flush()
                    time.sleep(0.08) # ~12 FPS
            except Exception:
                pass
        else:
            self.send_error(404)

    def _send_json(self, data, status=200):
        self.send_response(status)
        self.send_header('Content-type', 'application/json; charset=utf-8')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data, ensure_ascii=False).encode('utf-8'))

    def do_POST(self):
        global bridge_node
        if self.path == '/api/cors':
            try:
                content_length = int(self.headers.get('Content-Length', 0))
                body = self.rfile.read(content_length)
                new_cfg = json.loads(body.decode('utf-8'))
                with open(CORS_CONFIG_PATH, 'w', encoding='utf-8') as f:
                    json.dump(new_cfg, f, indent=2, ensure_ascii=False)

                if bridge_node:
                    msg = String()
                    msg.data = "reload"
                    bridge_node.pub_cors_cmd.publish(msg)

                self._send_json({'ok': True, 'config': new_cfg})
            except Exception as e:
                self._send_json({'ok': False, 'error': str(e)}, status=500)

        elif self.path == '/api/follower/start':
            ok, msg = start_follower()
            self._send_json({'ok': ok, 'message': msg, 'running': is_follower_running()})

        elif self.path == '/api/follower/stop':
            ok, msg = stop_follower()
            self._send_json({'ok': ok, 'message': msg, 'running': is_follower_running()})

        elif self.path == '/api/follower/toggle':
            if is_follower_running():
                ok, msg = stop_follower()
            else:
                ok, msg = start_follower()
            self._send_json({'ok': ok, 'message': msg, 'running': is_follower_running()})

        elif self.path == '/api/manual_drive':
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length)
            try:
                req = json.loads(body.decode('utf-8'))
                action = req.get('action', 'custom')
                vx = float(req.get('vx', 0.0))
                wz = float(req.get('wz', 0.0))

                # 强行介入：若自动跟随正在运行，强行介入必须先停掉自动跟随！
                if check_follower_running_cached():
                    stop_follower()

                if bridge_node:
                    bridge_node.send_manual_twist(vx, wz)

                with data_lock:
                    state['manual_override'] = {
                        'active': True,
                        'action': action,
                        'vx': vx,
                        'wz': wz,
                        'time': time.time()
                    }

                self._send_json({'ok': True, 'action': action, 'vx': vx, 'wz': wz})
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
    print(f"🚀 SLAM Web 服务已就绪: http://192.168.0.170:{PORT}")
    server.serve_forever()

if __name__ == '__main__':
    main()

