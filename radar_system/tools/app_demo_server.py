#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""安卓 app 演示/联调服务器:不需要车、不需要 ROS。

仿真一辆车(阿克曼底盘模型,--gen5 时是两台模拟的欧艾迪驱动器),按 docs/APP_PROTOCOL.md 提供与车端相同的接口:

- ``ws://本机:8088/ws``         hello + 10Hz 遥测;摇杆、IMU 直行、急停、复位、RTK 定点画线、路线、场地作业
- ``/display``                   车载屏幕只读页面
- ``POST /api/motion/stop``      急停第二通道
- UDP 48620 发现广播

    python3 radar_system/tools/app_demo_server.py            # 真机与电脑同一 WiFi
    python3 radar_system/tools/app_demo_server.py --line-demo  # 预设一条 7 m 直线,车在 A 点后方 2.5 m
    python3 radar_system/tools/app_demo_server.py --route-demo # 预设连线矩形并开始跑,浏览器看 /display
    python3 radar_system/tools/app_demo_server.py --gen5       # Gen5 差速底盘:两台模拟的欧艾迪驱动器,能原地转
    # 安卓模拟器里连 10.0.2.2:8088

定点画线、路线用真实的 LineMission/LineTracker/PathTracker;定位是仿真车真值加厘米级噪声,始终当作 RTK 固定解。
地图原点在房间原点,路线只存在内存里(车端存 ~/rtk/route.json)。仿真只用于界面联调。
"""
import argparse
import http.server
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
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), 'oid_protocol'))
if '--gen5' in sys.argv:
    # Gen5 差速底盘:共享配置换成 robot_gen5.json(要在导入 runtime_config 之前),车用两台模拟驱动器开
    os.environ.setdefault('RK3588_ROBOT_CONFIG',
                          os.path.join(os.path.dirname(os.path.dirname(HERE)), 'robot_core', 'robot_gen5.json'))

import app_link  # noqa: E402
import car_display  # noqa: E402
from line_sim import Chassis  # noqa: E402
from runtime_config import PROFILE, profile_hash  # noqa: E402
from robot_core.kinematics import ChassisGeometry  # noqa: E402
from line_mission import LineMission  # noqa: E402
from field_plan_service import FieldPlanService  # noqa: E402
from heading_hold import HeadingHold  # noqa: E402

DEMO_ANTENNA = (0.345, 0.165, 0.40)     # 演示用 ANT1 杠杆臂：前、左、离地高度，m
DEMO_IMU_DRIFT_DEG_S = 0.056            # 演示 IMU 偏航零偏（与遥测 attitude 同一个数）
DEMO_STRAIGHT_BIAS = 0.03               # 演示 IMU 直行：两轮不一致带来的偏转，rad/s，让纠偏看得见


def demo_tilt(roll_deg, pitch_deg):
    """演示用倾斜补偿状态（真车由融合的 tilt_status 给，公式同 gnss_fusion._lever）：天线水平位置比平地时偏了多少。"""
    ax, ay, h = DEMO_ANTENNA
    r, p = math.radians(roll_deg), math.radians(pitch_deg)
    fwd = ax * math.cos(p) + (ay * math.sin(r) + h * math.cos(r)) * math.sin(p) - ax
    left = ay * math.cos(r) - h * math.sin(r) - ay
    return {'height_m': h, 'active': True, 'reason': None, 'roll_deg': round(roll_deg, 2),
            'pitch_deg': round(pitch_deg, 2), 'forward_m': round(fwd, 4), 'left_m': round(left, 4)}
TIERS = {k: PROFILE['manual'][k + '_mps'] for k in app_link.SPEED_TIERS}
FIELD_PLANS = FieldPlanService()


class DemoSim:
    """仿真时钟：底盘每 10 ms 积分一次，控制（control_hook）20 Hz，与车上 line_node 的周期相同。"""

    def __init__(self, car):
        self.t, self.car, self.control_hook = 0.0, car, None
        self._next = 0.0

    def run(self, seconds):
        end = self.t + seconds
        while self.t < end - 1e-9:
            if self.t >= self._next - 1e-9:
                self.control_hook(self.t)
                self._next += 0.05
            self.car.step(self.t, 0.01)
            self.t += 0.01


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
    """仿真车 + 与 MotionAuthority 同样语义的模式:IDLE/MANUAL/NAVIGATION/ESTOP。"""

    def __init__(self, gen5=False):
        if gen5:
            from oid_sim import OidSimCar
            chassis = OidSimCar()       # 下发指令经真的 CanLink 编成 CAN 帧,两台模拟驱动器转出轮速
        else:
            chassis = Chassis()
        self.sim = DemoSim(chassis)
        self.sim.control_hook = self._control
        self.lock = threading.RLock()
        self.mode = 'IDLE'
        self.manual = (0.0, 0.0, 0.0)       # vx, wz, steer_deg
        self.manual_at = -1e9
        self.manual_owner = None
        self.manual_hold = None     # IMU 定向直行的作业键 (会话, 作业编号)
        self.straight = HeadingHold(clock=lambda: self.sim.t)
        self.last_seq = {}
        self.geo = ChassisGeometry()
        self.mission = LineMission(DemoFusion(lambda: self.sim.car), samples=None)
        self.line_result = None
        self.imu_yaw0 = None    # 演示 IMU 偏航的零点：第一次出状态时的车头方向

    def preset_line(self, length=7.0):
        """--line-demo:A 在原点、B 在正前方,车放在 A 后 2.5 m(助跑)、左偏 25 cm、车头偏 6°。"""
        f = self.mission.fusion
        self.mission.points = {'A': f.to_geodetic(0.0, 0.0), 'B': f.to_geodetic(length, 0.0)}
        car = self.sim.car
        car.x, car.y, car.yaw = -2.5, 0.25, math.radians(6.0)

    def preset_route(self):
        """--route-demo：一个 9 × 5 m 的连线矩形（四个直角都要开出去倒车掉头），车在起点后方，直接开始跑。"""
        car = self.sim.car
        car.x, car.y, car.yaw = -5.5, -2.0, 0.0
        route = dict(kind='path', points=[[-3.0, -2.0], [6.0, -2.0], [6.0, 3.0], [-3.0, 3.0]], closed=True,
                     laps=1, cruise=0.5)
        ok, message = self.mission.command({'cmd': 'route_start', 'route': route}, self.sim.t)
        self.line_result = dict(ok=ok, message=message, id='demo', cmd='route_start')

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
        car = self.sim.car
        # 与 line_node 相同:任务每周期都跑,申请/交还 NAVIGATION 按 MotionAuthority 的规则
        vx_l, wz_l, select = self.mission.tick(t, self.mode)
        if select is True and self.mode == 'IDLE':
            self.mode = 'NAVIGATION'
        elif select is False and self.mode == 'NAVIGATION':
            self.mode = 'IDLE'
        if self.mode == 'NAVIGATION':
            car.command(t, vx_l, wz_l)
            return
        vx, wz, _ = self.manual
        # 演示 IMU：真值偏航 + 零偏漂移，0.01° 一格（HDA536T 分辨率）
        imu_yaw = math.radians(round(math.degrees(car.yaw) - (self.imu_yaw0 or 0.0) + DEMO_IMU_DRIFT_DEG_S * t, 2))
        self.straight.on_imu(imu_yaw, car.wz, t)
        self.straight.on_odom(car.v, t)
        active = self.mode == 'MANUAL' and t - self.manual_at <= 0.75 and (vx or wz)
        if active and self.manual_hold is not None:
            vx, wz = self.straight.step(self.manual_hold, t)
            if vx:
                wz += DEMO_STRAIGHT_BIAS
        elif not active:
            self.straight.end('command_timeout')
        if active:
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
            moving = abs(vx) > 1e-6 or abs(wz) > 1e-6       # 差速车打横是原地转
            last = self.last_seq.get(session)
            if moving and seq is not None and last is not None and seq <= last:
                return True, 'stale'
            if seq is not None:
                self.last_seq[session] = seq
            if moving and self.mode in ('IDLE', 'NAVIGATION', 'MANUAL'):
                self.mode = 'MANUAL'
            self.straight.end('manual')         # 摇杆一来就接管直行
            self.manual = (vx, wz, deg)
            self.manual_hold = None
            self.manual_at = self.sim.t
            self.manual_owner = session
            return True, ''

    def straight_command(self, cmd, session):
        """IMU 定向直行：go 接着开（新 run 按当时偏航定目标），stop 停车。与 radar_web_server 同一套 HeadingHold。"""
        with self.lock:
            if cmd['action'] == 'stop':
                self.straight.end('user')
                self.manual, self.manual_hold = (0.0, 0.0, 0.0), None
                return True, ''
            if self.mode == 'ESTOP':
                return False, '急停已锁存，请先复位'
            seq = cmd.get('seq')
            last = self.last_seq.get(session)
            if seq is not None and last is not None and seq <= last:
                return True, 'stale'
            if seq is not None:
                self.last_seq[session] = seq
            if self.mode in ('IDLE', 'NAVIGATION', 'MANUAL'):
                self.mode = 'MANUAL'
            self.manual = (self.straight.cfg.speed, 0.0, 0.0)
            self.manual_hold = (session, cmd['run'])
            self.manual_at = self.sim.t
            self.manual_owner = session
            return True, ''

    def release(self, session):
        with self.lock:
            if self.manual_owner == session:
                self.straight.end('disconnected')
                self.manual = (0.0, 0.0, 0.0)
                self.manual_hold = None

    def estop(self):
        with self.lock:
            self.mode = 'ESTOP'
            self.straight.end('estop')
            self.manual = (0.0, 0.0, 0.0)
            self.manual_hold = None
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
            if cmd['action'] == 'field_start':
                value = FIELD_PLANS.get(cmd['preview_token'])
                # Cache expiry uses wall time; the simulated mission runs on its own clock.
                value['created_at'] = self.sim.t
                payload.update(field_plan=value, allow_outside=cmd.get('allow_outside', False))
            for key in ('route', 'route_rev', 'job_id', 'preview_token', 'operation', 'element', 'from_m', 'to_m', 'overlap_m', 'allow_outside'):
                if key in cmd:
                    payload[key] = cmd[key]
            ok, message = self.mission.command(payload, self.sim.t)
            self.line_result = dict(ok=ok, message=message, id=cmd.get('id'), cmd=payload['cmd'])

    def snapshot(self):
        with self.lock:
            car = self.sim.car
            vx, wz, deg = self.manual
            yaw_deg = math.degrees(car.yaw)
            if self.imu_yaw0 is None:
                self.imu_yaw0 = yaw_deg
            wrap = lambda a: (a + 180.0) % 360.0 - 180.0  # noqa: E731
            roll, pitch = 1.2 * math.sin(self.sim.t / 7.0), -0.8 + 0.6 * math.sin(self.sim.t / 5.0)
            return {
                'voltage': 24.2, 'battery_pct': 83,
                'wheeltec': {'connected': True, 'telemetry': {'velocity': [car.v, 0.0, car.wz]}},
                'wheeltec_status_age_s': 0.04,
                'motion': {'mode': self.mode, 'healthy': True, 'epoch': 'demo',
                           'waiting_stationary': abs(car.v) > 0.02},
                'motion_fresh': True,
                'manual_override': {'active': self.mode == 'MANUAL' and abs(vx) > 0, 'steer_deg': deg},
                'straight': self.straight.status(self.sim.t),
                'gnss': {'_rx_monotonic': time.monotonic(), 'receiver_ok': True, 'rtk': 'FIXED', 'sats': 31,
                         'diff_age': 1.0, 'sigma_h': 0.008, 'heading_ok': True, 'baseline_m': 0.331,
                         'antennas': {
                             'ant1': {'available': True, 'source': 'BESTPOSA', 'sol': 'NARROW_INT', 'fix': 'FIXED',
                                      'sats_used': 31, 'sats_tracked': 38, 'age_s': 0.1},
                             'ant2': {'available': True, 'source': 'BESTNAVHA', 'sol': 'NARROW_INT', 'fix': 'FIXED',
                                      'sats_used': 28, 'sats_tracked': 36, 'age_s': 0.1, 'offset_m': 0.331,
                                      'consistent': True}},
                         'heading': {'sol': 'NARROW_INT', 'fix': 'FIXED', 'sats_used': 27, 'sats_tracked': 35,
                                     'length_m': 0.331, 'pitch_deg': 0.4, 'sigma_deg': 0.12, 'ref_m': 0.331,
                                     'valid': True, 'yaw_deg': round(wrap(yaw_deg), 2)},
                         'ntrip': {'configured': True, 'connected': True, 'error': None, 'age_s': 0.5,
                                   'bytes': int(1e6 + 900 * time.monotonic()) % 10 ** 7}},
                'line': dict(self.mission.status(), motion_mode=self.mode, last_result=self.line_result,
                             _rx_monotonic=time.monotonic(),
                             # 演示用车身姿态：地面略有起伏；IMU 偏航相对启动时，按实车陀螺零偏 0.056°/s 漂
                             # （真车由 imu 组件 + line_node 给出）
                             attitude={'roll_deg': round(roll, 2), 'pitch_deg': round(pitch, 2),
                                       'yaw_deg': round(wrap(yaw_deg - self.imu_yaw0 + DEMO_IMU_DRIFT_DEG_S * self.sim.t), 2),
                                       'source': 'imu_madgwick'},
                             tilt=demo_tilt(roll, pitch)),
            }

CAR = None


class Handler(http.server.BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def log_message(self, *args):
        return

    def do_GET(self):
        # 车载屏幕只读显示（与真车同一份页面和事件流）
        if self.path.startswith('/display') and car_display.handle(
                self, snapshot=CAR.snapshot, name='演示车 (仿真)', profile=PROFILE, sessions=SESSIONS, network=NETWORK,
                view=VIEW):
            return
        if self.path == '/ws':
            return self.serve_ws()
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
        if self.path == '/api/field/plan':
            try:
                if self.headers.get('Origin'):
                    raise ValueError('不接受网页跨源规划请求')
                size = int(self.headers.get('Content-Length', 0))
                if not 0 < size <= 262144:
                    raise ValueError('规划请求大小无效')
                request = json.loads(self.rfile.read(size).decode('utf-8'))
                data = dict(ok=True, plan=FIELD_PLANS.prepare(request, CAR.snapshot()['line']))
                status = 200
            except (ValueError, KeyError, TypeError, UnicodeError) as exc:
                self.close_connection = True
                data, status = dict(ok=False, message=str(exc)), 400
            body = json.dumps(data, ensure_ascii=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
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
        SESSIONS.enter()

        def pump():
            seq = 0
            try:
                ws.send_json(app_link.build_hello(PROFILE, name='演示车 (仿真)', camera={},
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
                elif kind == 'straight':
                    ok, msg = CAR.straight_command(cmd, session)
                    if not ok or cmd['action'] == 'stop':
                        ack(cmd, ok, msg)
                elif kind == 'estop':
                    ack(cmd, *CAR.estop())
                elif kind == 'reset':
                    ack(cmd, *CAR.reset())
                elif kind == 'line':
                    CAR.line_command(cmd)
                    ack(cmd, True, 'forwarded')
                elif kind == 'route':
                    try:
                        CAR.route_command(cmd)
                        ack(cmd, True, 'forwarded')
                    except ValueError as exc:
                        ack(cmd, False, str(exc))
                elif kind == 'view':
                    VIEW.set(session, cmd['file'])
                elif kind == 'ping':
                    ws.send_json({'type': 'pong', 't': cmd.get('t'), 'server': time.monotonic()})
        except (app_link.WebSocketClosed, OSError):
            pass
        finally:
            done.set()
            SESSIONS.leave()
            VIEW.clear(session)
            CAR.release(session)
            ws.close()


SESSIONS = car_display.Sessions()
NETWORK = car_display.Network()
VIEW = car_display.AppView()


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
    p.add_argument('--route-demo', action='store_true', help='预设一个连线矩形并直接开始跑（看车载屏幕 /display）')
    p.add_argument('--gen5', action='store_true', help='Gen5 差速底盘（两台模拟的欧艾迪单路驱动器，能原地转）')
    args = p.parse_args()
    CAR = DemoCar(gen5=args.gen5)
    if args.line_demo:
        CAR.preset_line()
    if args.route_demo:
        CAR.preset_route()
    NETWORK.start()
    threading.Thread(target=CAR.run_forever, daemon=True).start()
    threading.Thread(target=beacon_loop, args=(args.port,), daemon=True).start()
    server = Server(('0.0.0.0', args.port), Handler)
    print(f'app 演示服务器: ws://0.0.0.0:{args.port}/ws  (模拟器连 10.0.2.2:{args.port})', flush=True)
    print(f'车载屏幕显示: http://127.0.0.1:{args.port}/display', flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
