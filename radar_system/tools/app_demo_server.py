#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""安卓 app 演示/联调服务器:不需要车、不需要 ROS。

用 follow_sim 的真实跟随引擎实时仿真一辆车和两个走动的人,按
docs/APP_PROTOCOL.md 提供与车端相同的接口:

- ``ws://本机:8088/ws``         hello + 10Hz 遥测;摇杆、跟随、急停、复位、点选目标、RTK 走直线、走路线
- ``http://本机:8088/video.mjpg`` 按车上相机视角画出的合成画面(12 FPS)
- ``POST /api/motion/stop``      急停第二通道
- UDP 48620 发现广播

    python3 radar_system/tools/app_demo_server.py            # 真机与电脑同一 WiFi
    python3 radar_system/tools/app_demo_server.py --line-demo  # 预设一条 7 m 直线,车在 A 点后方 2.5 m
    # 安卓模拟器里连 10.0.2.2:8088

走直线、走路线用真实的 LineMission/LineTracker/PathTracker;定位是仿真车真值加厘米级噪声,始终当作 RTK 固定解。
地图原点在房间原点,路线只存在内存里(车端存 ~/rtk/route.json)。

仿真只用于界面联调:画面是示意,人和车的运动来自真实跟随代码。
"""
import argparse
import http.server
import io
import json
import math
import os
import random
import socket
import socketserver
import sys
import threading
import time
import uuid
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, HERE)

from PIL import Image, ImageDraw  # noqa: E402

import app_link  # noqa: E402
import follow_sim as fs  # noqa: E402
from runtime_config import PROFILE, profile_hash  # noqa: E402
from robot_core.kinematics import ChassisGeometry  # noqa: E402
from line_mission import LineMission  # noqa: E402

CAMERA = {'width': 640, 'height': 480, 'fx': 570.3, 'cx': 319.5}
FY, CY = 570.3, 239.5
CAM_HEIGHT_M = 0.55
PERSON_HEIGHT_M = 1.70
TIERS = {k: PROFILE['manual'][k + '_mps'] for k in app_link.SPEED_TIERS}


def looping_walk(start, legs, repeat=40):
    """legs: [(x, y, 秒)] 相对路线,反复走;返回 Walker 用的 (t, x, y) 路点。"""
    t, (x, y) = 0.0, start
    wp = [(t, x, y)]
    for _ in range(repeat):
        for nx, ny, dur in legs:
            t += dur
            wp.append((t, nx, ny))
    return wp


def build_world():
    x0 = fs.front_start(1.1)
    main = fs.Walker(looping_walk((x0, 0.0), [
        (x0, 0.0, 3.0), (x0 + 2.6, 0.0, 2.6), (x0 + 2.6, 1.8, 2.0), (x0 + 2.6, 1.8, 2.0),
        (0.6, 1.9, 3.0), (-1.6, 0.4, 2.8), (-1.6, 0.4, 4.0), (x0, -0.2, 4.0)]))
    other = fs.Walker(looping_walk((6.0, -2.5), [
        (6.0, -2.5, 5.0), (0.5, -2.6, 6.0), (0.5, -2.6, 3.0), (6.0, -2.5, 6.0)]))
    posts = [(3.6 + 0.45 * i, 2.7, 0.025) for i in range(5)] + [(1.2, -1.6, 0.12)]
    return fs.World([main, other], room=(-5, -4, 9, 4), posts=posts), main


class DemoFusion:
    """演示用的"融合定位":仿真车真值 + 厘米级噪声,永远是 RTK 固定解。经纬度按本地线性换算。"""

    LAT0, LON0 = 30.0, 120.0
    M_PER_DEG_LAT, M_PER_DEG_LON = 110574.0, 96486.0      # 北纬 30° 附近

    def __init__(self, car_fn, seed=3):
        self.car_fn, self.rng = car_fn, random.Random(seed)

    def pose(self, t):
        car, g = self.car_fn(), self.rng.gauss
        return SimpleNamespace(x=car.x + g(0, 0.004), y=car.y + g(0, 0.004),
                               yaw=car.yaw + g(0, math.radians(0.1)), speed=car.v, yaw_rate=car.wz,
                               usable_for_control=True, quality='RTK_FIXED', sigma_xy=0.008,
                               sigma_yaw=math.radians(0.15), reasons=())

    def to_local(self, lat, lon):
        return (lon - self.LON0) * self.M_PER_DEG_LON, (lat - self.LAT0) * self.M_PER_DEG_LAT

    def to_geodetic(self, x, y):
        return self.LAT0 + y / self.M_PER_DEG_LAT, self.LON0 + x / self.M_PER_DEG_LON


class DemoCar:
    """仿真车 + 与 MotionAuthority 同样语义的模式:IDLE/MANUAL/FOLLOW/NAVIGATION/ESTOP。"""

    def __init__(self):
        world, target = build_world()
        self.sim = fs.Sim(world, target, fs.make_config())
        self.sim.samples = __import__('collections').deque(maxlen=50)
        self.sim.control_hook = self._control
        self.engine = self.sim.engine
        self.status = {}
        self.engine.emit_status = self._on_status
        self.lock = threading.RLock()
        self.mode = 'IDLE'
        self.manual = (0.0, 0.0, 0.0)       # vx, wz, steer_deg
        self.manual_at = -1e9
        self.manual_owner = None
        self.last_seq = {}
        self.geo = ChassisGeometry()
        self.mission = LineMission(DemoFusion(lambda: self.sim.car), samples=None)
        self.line_result = None

    def preset_line(self, length=7.0):
        """--line-demo:A 在原点、B 在正前方,车放在 A 后 2.5 m(助跑)、左偏 25 cm、车头偏 6°。"""
        f = self.mission.fusion
        self.mission.points = {'A': f.to_geodetic(0.0, 0.0), 'B': f.to_geodetic(length, 0.0)}
        car = self.sim.car
        car.x, car.y, car.yaw = -2.5, 0.25, math.radians(6.0)

    def _on_status(self, data):
        self.status = data

    # ---- 仿真时钟跟着墙钟走 ----
    def run_forever(self):
        wall0, sim0 = time.monotonic(), self.sim.t
        while True:
            with self.lock:
                lag = (time.monotonic() - wall0) - (self.sim.t - sim0)
                if lag > 0:
                    self.sim.run(min(lag, 0.2))
            time.sleep(0.01)

    def _control(self, t):
        e, car = self.engine, self.sim.car
        # 与 line_node 相同:任务每周期都跑,申请/交还 NAVIGATION 按 MotionAuthority 的规则
        vx_l, wz_l, select = self.mission.tick(t, self.mode)
        if select is True and self.mode in ('IDLE', 'FOLLOW'):
            self.mode = 'NAVIGATION'
        elif select is False and self.mode == 'NAVIGATION':
            self.mode = 'IDLE'
        if self.mode == 'NAVIGATION':
            e.pause()
            car.command(t, vx_l, wz_l)
            return
        if self.mode == 'FOLLOW':
            e.step()
            car.command(t, e.cmd_vx, e.cmd_wz)
            return
        e.pause()
        vx, wz, _ = self.manual
        if self.mode == 'MANUAL' and t - self.manual_at <= 0.75 and (vx or wz):
            car.command(t, vx, wz)
        else:
            car.command(t, 0.0, 0.0)
            if self.mode == 'MANUAL' and t - self.manual_at > 0.75 and abs(car.v) < 0.02:
                self.mode = 'IDLE'

    # ---- 指令 ----
    def drive(self, cmd, session):
        with self.lock:
            if self.mode == 'ESTOP':
                return False, '急停已锁存，请先复位'
            seq = cmd.get('seq')
            vx, wz, deg = app_link.resolve_stick(cmd['throttle'], cmd['steer'], cmd['tier'],
                                                 TIERS, PROFILE['manual']['reverse_scale'], self.geo)
            moving = abs(vx) > 1e-6
            last = self.last_seq.get(session)
            if moving and seq is not None and last is not None and seq <= last:
                return True, 'stale'
            if seq is not None:
                self.last_seq[session] = seq
            if moving and self.mode in ('IDLE', 'FOLLOW', 'NAVIGATION', 'MANUAL'):
                self.mode = 'MANUAL'
            self.manual = (vx, wz, deg)
            self.manual_at = self.sim.t
            self.manual_owner = session
            return True, ''

    def release(self, session):
        with self.lock:
            if self.manual_owner == session:
                self.manual = (0.0, 0.0, 0.0)

    def follow(self, enable):
        with self.lock:
            if not enable:
                if self.mode == 'FOLLOW':
                    self.mode = 'IDLE'
                return True, 'IDLE'
            if self.mode in ('ESTOP', 'FAULT'):
                return False, '急停已锁存，请先复位'
            if abs(self.sim.car.v) > 0.02:
                return False, '等待车辆停稳'
            self.engine.reset_tracking()
            self.mode = 'FOLLOW'
            return True, 'FOLLOW: selected'

    def estop(self):
        with self.lock:
            self.mode = 'ESTOP'
            self.manual = (0.0, 0.0, 0.0)
            return True, 'ESTOP latched'

    def reset(self):
        with self.lock:
            if abs(self.sim.car.v) > 0.02:
                return False, '等待车辆确认静止后才能复位'
            self.mode = 'IDLE'
            return True, 'IDLE; select a new task'

    def line_command(self, cmd):
        with self.lock:
            payload = {'cmd': cmd['action'], 'id': cmd.get('id')}
            for k in ('point', 'cruise'):
                if k in cmd:
                    payload[k] = cmd[k]
            ok, message = self.mission.command(payload, self.sim.t)
            self.line_result = dict(ok=ok, message=message, id=cmd.get('id'), cmd=cmd['action'])

    def route_command(self, cmd):
        with self.lock:
            payload = {'cmd': 'route_' + cmd['action'], 'id': cmd.get('id')}
            if 'route' in cmd:
                payload['route'] = cmd['route']
            ok, message = self.mission.command(payload, self.sim.t)
            self.line_result = dict(ok=ok, message=message, id=cmd.get('id'), cmd=payload['cmd'])

    def select(self, bearing):
        with self.lock:
            chosen = self.engine.select_target(bearing)
        return (chosen is not None), ('目标 #%s' % chosen if chosen else '那里没有相机刚看到的人')

    # ---- 视角换算 ----
    def _cam(self, wx, wy):
        bx, by = self.sim.car.to_body(wx, wy)
        s = fs.SENS
        return bx - s['camera_x_m'], by - s['camera_y_m']

    def persons(self):
        out = []
        for w in self.sim.world.walkers:
            wx, wy, _, _ = w.state(self.sim.t)
            fwd, lat = self._cam(wx, wy)
            if fwd < 0.5 or abs(math.atan2(lat, fwd)) > math.radians(29):
                continue
            u = CAMERA['cx'] - CAMERA['fx'] * lat / fwd
            half = CAMERA['fx'] * 0.24 / fwd
            top = CY - FY * (PERSON_HEIGHT_M - CAM_HEIGHT_M) / fwd
            bottom = CY + FY * CAM_HEIGHT_M / fwd
            out.append({'label': 'person', 'conf': 0.86,
                        'x1': int(max(0, u - half)), 'y1': int(max(0, top)),
                        'x2': int(min(639, u + half)), 'y2': int(min(479, bottom)),
                        'distance': round(math.hypot(fwd, lat), 2),
                        'bearing_rad': round(math.atan2(lat, fwd), 4), '_fwd': fwd, '_u': u})
        return out

    def snapshot(self):
        with self.lock:
            car, scan = self.sim.car, self.sim.last_scan
            ranges = ([r if math.isfinite(r) else 0.0 for r in scan.ranges] if scan else [])
            # 与实车一致:跟随节点一直在发状态,非跟随模式时是 PAUSED 而不是无响应
            follow = dict(self.status) or {'state': 'PAUSED'}
            following = self.mode == 'FOLLOW'
            follow['stale'] = False
            vx, wz, deg = self.manual
            return {
                'hz': 10.0, 'voltage': 24.2, 'battery_pct': 83,
                'ranges': ranges, 'angle_min': 0.0, 'angle_increment': 2 * math.pi / fs.BINS,
                'ai_status': {'fps': 30.0}, 'ai_targets': self.persons(),
                'wheeltec': {'connected': True, 'telemetry': {'velocity': [car.v, 0.0, car.wz]}},
                'wheeltec_status_age_s': 0.04,
                'motion': {'mode': self.mode, 'healthy': True, 'epoch': 'demo',
                           'waiting_stationary': abs(car.v) > 0.02},
                'motion_fresh': True,
                'manual_override': {'active': self.mode == 'MANUAL' and abs(vx) > 0, 'steer_deg': deg},
                'follower_running': following,
                'follower': follow,
                'gnss': {'_rx_monotonic': time.monotonic(), 'receiver_ok': True, 'rtk': 'FIXED', 'sats': 31,
                         'diff_age': 1.0, 'sigma_h': 0.008, 'heading_ok': True, 'baseline_m': 0.331,
                         'ntrip': {'configured': True, 'connected': True, 'error': None, 'age_s': 0.5}},
                'line': dict(self.mission.status(), motion_mode=self.mode, last_result=self.line_result,
                             _rx_monotonic=time.monotonic()),
            }

    # ---- 合成相机画面 ----
    def render_jpeg(self):
        with self.lock:
            people = self.persons()
            posts = [(self._cam(px, py), r) for px, py, r in self.sim.world.posts]
        img = Image.new('RGB', (640, 480), (62, 66, 70))
        d = ImageDraw.Draw(img)
        d.rectangle([0, int(CY), 640, 480], fill=(92, 88, 82))
        for dist in (1, 1.5, 2, 3, 4, 6):
            v = CY + FY * CAM_HEIGHT_M / dist
            d.line([(0, v), (640, v)], fill=(104, 100, 94), width=1)
        items = [(fwd, 'post', (fwd, lat), r) for (fwd, lat), r in posts if fwd > 0.4]
        items += [(p['_fwd'], 'person', p, None) for p in people]
        for _, kind, data, r in sorted(items, key=lambda it: -it[0]):
            if kind == 'post':
                fwd, lat = data
                u = CAMERA['cx'] - CAMERA['fx'] * lat / fwd
                w = max(2, CAMERA['fx'] * r / fwd)
                d.rectangle([u - w, CY - FY * 0.2 / fwd, u + w, CY + FY * CAM_HEIGHT_M / fwd],
                            fill=(140, 132, 120))
                continue
            p = data
            x1, y1, x2, y2 = p['x1'], p['y1'], p['x2'], p['y2']
            h = y2 - y1
            head = h * 0.13
            cxp = (x1 + x2) / 2
            d.ellipse([cxp - head * 0.8, y1, cxp + head * 0.8, y1 + head * 1.6], fill=(214, 186, 160))
            d.rounded_rectangle([x1 + (x2 - x1) * 0.12, y1 + head * 1.7, x2 - (x2 - x1) * 0.12, y1 + h * 0.58],
                                radius=6, fill=(58, 96, 150))
            d.rectangle([x1 + (x2 - x1) * 0.22, y1 + h * 0.58, cxp - 2, y2], fill=(40, 42, 48))
            d.rectangle([cxp + 2, y1 + h * 0.58, x2 - (x2 - x1) * 0.22, y2], fill=(40, 42, 48))
        buf = io.BytesIO()
        img.save(buf, format='JPEG', quality=70)
        return buf.getvalue()


CAR = None


class Handler(http.server.BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def log_message(self, *args):
        return

    def do_GET(self):
        if self.path == '/ws':
            return self.serve_ws()
        if self.path.split('?')[0] == '/video.mjpg':
            return self.serve_video()
        kind, body = 'text/plain', b'rk3588 app demo server\n'
        if self.path == '/app/beacon':
            # 与车端相同：内容就是 UDP 发现广播
            kind, body = 'application/json', app_link.beacon('演示车 (仿真)', self.server.server_address[1],
                                                             profile_hash(PROFILE))
        self.send_response(200)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if self.path == '/api/motion/stop':
            ok, msg = CAR.estop()
            body = json.dumps({'ok': ok, 'message': msg}).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_error(404)

    def serve_video(self):
        self.close_connection = True
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Connection', 'close')
        self.end_headers()
        try:
            while True:
                self.wfile.write(app_link.mjpeg_part(CAR.render_jpeg(), time.time()))
                self.wfile.flush()
                time.sleep(1 / 12)
        except (OSError, ValueError):
            pass

    def serve_ws(self):
        key = self.headers.get('Sec-WebSocket-Key')
        if not key:
            return self.send_error(400)
        self.close_connection = True
        self.send_response(101, 'Switching Protocols')
        self.send_header('Upgrade', 'websocket')
        self.send_header('Connection', 'Upgrade')
        self.send_header('Sec-WebSocket-Accept', app_link.accept_key(key))
        self.end_headers()
        self.wfile.flush()
        self.connection.settimeout(3.0)
        ws = app_link.WebSocket(self.rfile, self.wfile)
        session = 'demo-' + uuid.uuid4().hex[:8]
        done = threading.Event()

        def pump():
            seq = 0
            try:
                ws.send_json(app_link.build_hello(PROFILE, name='演示车 (仿真)', camera=CAMERA,
                                                  session=session))
                while not done.is_set():
                    seq += 1
                    ws.send_json(app_link.build_telemetry(CAR.snapshot(), now=time.monotonic(),
                                                          seq=seq, sensors=PROFILE['sensors']))
                    done.wait(0.1)
            except Exception:
                pass
            finally:
                done.set()

        def ack(cmd, ok, message=''):
            try:
                ws.send_json({'type': 'ack', 'id': cmd.get('id'), 'cmd': cmd.get('type'),
                              'ok': bool(ok), 'message': message})
            except app_link.WebSocketClosed:
                pass

        threading.Thread(target=pump, daemon=True).start()
        try:
            while not done.is_set():
                try:
                    cmd = app_link.parse_command(ws.recv())
                except app_link.CommandError as exc:
                    ack({}, False, str(exc))
                    continue
                kind = cmd['type']
                if kind == 'drive':
                    ok, msg = CAR.drive(cmd, session)
                    if not ok:
                        ack(cmd, ok, msg)
                elif kind == 'estop':
                    ack(cmd, *CAR.estop())
                elif kind == 'reset':
                    ack(cmd, *CAR.reset())
                elif kind == 'follow':
                    ack(cmd, *CAR.follow(cmd['enable']))
                elif kind == 'select_target':
                    ack(cmd, *CAR.select(cmd['bearing_rad']))
                elif kind == 'line':
                    CAR.line_command(cmd)
                    ack(cmd, True, 'forwarded')
                elif kind == 'route':
                    CAR.route_command(cmd)
                    ack(cmd, True, 'forwarded')
                elif kind == 'ping':
                    ws.send_json({'type': 'pong', 't': cmd.get('t'), 'server': time.monotonic()})
        except (app_link.WebSocketClosed, OSError):
            pass
        finally:
            done.set()
            CAR.release(session)
            ws.close()


class Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def beacon_loop(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    payload = app_link.beacon('演示车 (仿真)', port, profile_hash(PROFILE))
    while True:
        try:
            sock.sendto(payload, ('255.255.255.255', app_link.DISCOVERY_PORT))
        except OSError:
            pass
        time.sleep(1.0)


def main():
    global CAR
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--port', type=int, default=8088)
    p.add_argument('--line-demo', action='store_true', help='预设一条 7 m 直线,车停在 A 点后方')
    args = p.parse_args()
    CAR = DemoCar()
    if args.line_demo:
        CAR.preset_line()
    threading.Thread(target=CAR.run_forever, daemon=True).start()
    threading.Thread(target=beacon_loop, args=(args.port,), daemon=True).start()
    server = Server(('0.0.0.0', args.port), Handler)
    print(f'app 演示服务器: ws://0.0.0.0:{args.port}/ws  (模拟器连 10.0.2.2:{args.port})', flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
