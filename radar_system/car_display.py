#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车载屏幕只读显示：与 app 同一个 3D 路线地图、卫星状态、连接状态。

真车（radar_web_server）和演示车（tools/app_demo_server）都用 handle() 提供：

    GET /display                页面（static/display/index.html）
    GET /display/<文件>         页面脚本、样式（static/display 下白名单里的文件）
    GET /display/stream         事件流（SSE，10 Hz）：app 同款遥测（不含雷达扫描）+ 连着几台平板 + 车的网络
                                + 计划路线（车身中心要走的线，路线变了才发）+ 平板正在预览的图纸（变了才发）
    GET /display/vehicle.bin    车辆模型：与 app 相同的 wheeltec_rtk.wrmesh（gzip 原样发，浏览器按 Content-Encoding 解压）

大屏跟着平板走：平板上改的路线 0.6 s 后存到车上，大屏读车上的路线；平板导入图纸还没应用到车时，app 发一条
view 指令（只用于显示），大屏同样画出这张图纸；平板点开始作业后大屏按车上的作业状态跟车显示。
只读：不接收任何指令；页面断开、重连都不影响手动驾驶和路线任务，也不算作平板连接。
车上的全屏窗口见 car_display_window.py。
"""
import gzip
import json
import math
import os
import re
import subprocess
import threading
import time

import app_link
import route_plan

HERE = os.path.dirname(os.path.abspath(__file__))
STATIC = os.path.join(HERE, 'static', 'display')
PAGE_FILES = {'display.js': 'application/javascript; charset=utf-8',
              'display.css': 'text/css; charset=utf-8'}
VEHICLE_ASSET = os.path.join(STATIC, 'wheeltec_rtk.wrmesh')
STREAM_PERIOD_S = 0.1
NETWORK_PERIOD_S = 5.0
PLAN_MAX_POINTS = 8192
PLAN_STEP_M = 0.1


class Sessions:
    """当前连着几台平板（app 的 /ws 长连接）；显示页面自己不算。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._count = 0
        self._joined = 0       # 一共连上来过几次（只增不减）：切换 WiFi 后用它判断有没有平板重新连上

    def enter(self):
        with self._lock:
            self._count += 1
            self._joined += 1

    def leave(self):
        with self._lock:
            self._count = max(0, self._count - 1)

    def count(self):
        with self._lock:
            return self._count

    def joined(self):
        with self._lock:
            return self._joined


class AppView:
    """平板地图上正在预览的图纸（app 的 view 指令）。只记最后一台发来的；那台平板断开就清掉。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._session = self._file = None

    def set(self, session, file):
        with self._lock:
            self._session, self._file = session, file

    def clear(self, session):
        with self._lock:
            if self._session == session:
                self._session = self._file = None

    def file(self):
        with self._lock:
            return self._file


def drawn_points(route):
    """图纸本身的线（与 app 地图上画的相同）：连线按点连、闭合连回起点；圆取 128 段；路点折线按圆弧过弯画。"""
    try:
        spec = route_plan.parse_spec(route)
    except (ValueError, TypeError):
        return []
    if spec.kind == 'circle':
        if spec.center is None:
            return []
        cx, cy = spec.center
        return [[round(cx + spec.radius * math.cos(2 * math.pi * i / 128), 3),
                 round(cy + spec.radius * math.sin(2 * math.pi * i / 128), 3)] for i in range(129)]
    pts = [[round(x, 3), round(y, 3)] for x, y in spec.points]
    if spec.kind == 'points':
        try:
            start, pieces = route_plan.loop_pieces(spec, spec.turn_radius)
            return _sample(start, pieces, 1, 0.0)
        except ValueError:
            pass
    return pts + [pts[0]] if spec.closed and len(pts) >= 3 else pts


def _run(args):
    try:
        return subprocess.run(args, capture_output=True, text=True, timeout=2.0).stdout
    except (OSError, subprocess.SubprocessError):
        return ''


def parse_ipv4(text):
    """`ip -4 -o addr show` 的输出 → [(网卡, 地址)]，去掉回环和 docker 这类内部网卡。"""
    out = []
    for line in text.splitlines():
        m = re.match(r'\d+:\s+(\S+)\s+inet\s+(\d+\.\d+\.\d+\.\d+)/', line)
        if m and m.group(1) != 'lo' and not m.group(1).startswith(('docker', 'br-', 'veth', 'l4tbr')):
            out.append((m.group(1), m.group(2)))
    return out


def parse_wifi(text):
    """`nmcli -t -f ACTIVE,SSID dev wifi` 的输出 → 正在用的 WiFi 名称（没有返回 None）。"""
    for line in text.splitlines():
        active, _, ssid = line.partition(':')
        if active == 'yes' and ssid:
            return ssid.replace('\\:', ':')
    return None


def _default_ip():
    """没有 `ip` 命令时（比如在电脑上跑演示车）：看往外发包会用哪个地址。UDP connect 不真的发数据。"""
    import socket
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(('192.0.2.1', 9))
            ip = s.getsockname()[0]
        return None if ip.startswith('127.') else ip
    except OSError:
        return None


class Network:
    """车的网络：连着的 WiFi、各网卡地址。后台每 NETWORK_PERIOD_S 秒查一次，事件流只读缓存。"""

    def __init__(self, period=NETWORK_PERIOD_S):
        self.period = period
        self._lock = threading.Lock()
        self._value = {'wifi': None, 'addresses': [], 'checked': None}
        self._thread = None

    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()
        return self

    def _loop(self):
        while True:
            self.refresh()
            time.sleep(self.period)

    def refresh(self):
        addresses = [{'dev': d, 'ip': ip} for d, ip in parse_ipv4(_run(['ip', '-4', '-o', 'addr', 'show']))]
        if not addresses:
            ip = _default_ip()
            addresses = [{'dev': None, 'ip': ip}] if ip else []
        value = {'wifi': parse_wifi(_run(['nmcli', '-t', '-f', 'ACTIVE,SSID', 'dev', 'wifi'])),
                 'addresses': addresses, 'checked': time.time()}
        with self._lock:
            self._value = value

    def value(self):
        with self._lock:
            return dict(self._value)


def _sample(start, pieces, direction, point_x, step=PLAN_STEP_M):
    """一段按行进方向画的路径 → 车身上后轴前 point_x 那一点走过的点。"""
    out, (x, y, yaw) = [], start
    back = 0.0 if direction > 0 else math.pi
    for length, kappa in pieces:
        n = max(1, int(length / step))
        for i in range(0 if not out else 1, n + 1):
            px, py, pyaw = route_plan.advance(x, y, yaw, length * i / n, kappa)
            out.append([round(px + point_x * math.cos(pyaw + back), 3), round(py + point_x * math.sin(pyaw + back), 3)])
        x, y, yaw = route_plan.advance(x, y, yaw, length, kappa)
    return out


def plan_points(route, limits):
    """车上存的路线（遥测 line.route，本地坐标）→ 计划要走的线，[[x, y], …]；画不成返回 []。

    连线路线画车身中心要走的线（含掉头补角，与车端 route_plan 同一套规划，不含接入段）；
    路点折线和圆画后轴中心要走的线。点太多时均匀抽稀，首尾保留。
    """
    if not route:
        return []
    limits = limits or {}
    try:
        spec = route_plan.parse_spec(route)
        radius = float(limits.get('min_radius') or 1.92)
        if spec.kind == 'path':
            body = tuple(float(v) for v in (limits.get('body') or ())) or None
            if body is None or len(spec.points) < 2:
                return []
            sections, _ = route_plan._route_sections(spec, radius, body)
            centre = 0.5 * (body[0] - body[1])
            points = []
            for direction, start, pieces in sections:
                part = _sample(start, pieces, direction, centre)
                points.extend(part if not points else part[1:])
        else:
            start, pieces = route_plan.loop_pieces(spec, radius)
            points = _sample(start, pieces, 1, 0.0)
    except (ValueError, TypeError, KeyError):
        return []
    if len(points) > PLAN_MAX_POINTS:
        last = len(points) - 1
        points = [points[i * last // (PLAN_MAX_POINTS - 1)] for i in range(PLAN_MAX_POINTS)]
    return points


def plan_key(route, limits):
    """计划路线的缓存键：路线内容和限制（不含状态字段）。"""
    if not route:
        return None
    keep = {k: v for k, v in route.items() if k not in ('problem', 'length', 'cover')}
    return json.dumps([keep, limits], sort_keys=True, ensure_ascii=False)


def release_id():
    """当前发布号（发布目录的 manifest.json）；本地开发时为 None。页面发现变了就刷新自己。"""
    try:
        with open(os.path.join(os.path.dirname(HERE), 'manifest.json'), encoding='utf-8') as fh:
            return json.load(fh).get('release_id')
    except (OSError, ValueError):
        return None


class Feed:
    """一条事件流的内容：每帧遥测 + 连接信息；计划路线只在变了时附上（同一路线只算一次）。"""

    _plans = {}
    _plans_lock = threading.Lock()

    def __init__(self, *, name, profile, sessions, network, view=None):
        g = profile['geometry']
        self.car = {'name': name, 'front_m': g['front_m'], 'rear_m': g['rear_m'],
                    'half_width_m': g['half_width_m'], 'wheelbase_m': g['wheelbase_m']}
        self.sessions, self.network, self.view = sessions, network, view
        self.release = release_id()
        self.seq = 0
        self._sent_plan = object()
        self._sent_view = object()

    @classmethod
    def _plan(cls, key, route, limits):
        with cls._plans_lock:
            if key in cls._plans:
                return cls._plans[key]
        points = plan_points(route, limits)
        with cls._plans_lock:
            if len(cls._plans) > 8:
                cls._plans.clear()
            cls._plans[key] = points
        return points

    def message(self, snapshot, now):
        self.seq += 1
        tel = app_link.build_telemetry(snapshot, now=now, seq=self.seq)
        out = {'type': 'display', 'seq': self.seq, 'release': self.release, 'car': self.car, 'tel': tel,
               'apps': self.sessions.count(), 'net': self.network.value()}
        line = tel.get('line') or {}
        route, limits = line.get('route'), line.get('route_limits')
        key = plan_key(route, limits) if not line.get('stale') else self._sent_plan
        if key != self._sent_plan:
            out['plan'] = {'points': self._plan(key, route, limits) if key else []}
            self._sent_plan = key
        file = self.view.file() if self.view is not None else None
        view_key = json.dumps(file, sort_keys=True, ensure_ascii=False)
        if view_key != self._sent_view:
            out['view'] = {'file': None if file is None else dict(
                name=file['name'], origin=file['origin'], kind=file['route']['kind'],
                closed=file['route']['closed'], count=len(file['route']['points']),
                points=drawn_points(file['route']))}
            if file is not None and 'control_points' in file:
                out['view']['file']['control_points'] = file['control_points']
            self._sent_view = view_key
        return out


def _send(handler, body, kind, status=200, headers=None):
    handler.send_response(status)
    handler.send_header('Content-Type', kind)
    handler.send_header('Content-Length', str(len(body)))
    handler.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
    for k, v in (headers or {}).items():
        handler.send_header(k, v)
    handler.end_headers()
    handler.wfile.write(body)


_vehicle = None


def _vehicle_bytes():
    """模型文件本身就是 gzip：先确认能解开、格式对，再原样缓存。"""
    global _vehicle
    if _vehicle is None:
        with open(VEHICLE_ASSET, 'rb') as fh:
            raw = fh.read()
        if not gzip.decompress(raw).startswith(b'WRIG0001'):
            raise OSError('vehicle asset format')
        _vehicle = raw
    return _vehicle


def handle(handler, *, snapshot, name, profile, sessions, network, view=None):
    """处理 /display 开头的 GET；不是就返回 False 交给原来的路由。snapshot() 返回 radar_web_server 的状态快照。"""
    path = handler.path.split('?')[0]
    if path in ('/display', '/display/'):
        with open(os.path.join(STATIC, 'index.html'), 'rb') as fh:
            _send(handler, fh.read(), 'text/html; charset=utf-8')
        return True
    if path == '/display/vehicle.bin':
        try:
            _send(handler, _vehicle_bytes(), 'application/octet-stream', headers={'Content-Encoding': 'gzip'})
        except (OSError, EOFError):
            handler.send_error(404)
        return True
    if path == '/display/stream':
        _stream(handler, Feed(name=name, profile=profile, sessions=sessions, network=network, view=view), snapshot)
        return True
    name_only = path[len('/display/'):] if path.startswith('/display/') else None
    if name_only in PAGE_FILES:
        with open(os.path.join(STATIC, name_only), 'rb') as fh:
            _send(handler, fh.read(), PAGE_FILES[name_only])
        return True
    return False


def _stream(handler, feed, snapshot):
    # 事件流没有长度，靠断开结束；不让这条连接被当作可复用的长连接
    handler.close_connection = True
    handler.send_response(200)
    handler.send_header('Content-Type', 'text/event-stream; charset=utf-8')
    handler.send_header('Cache-Control', 'no-cache')
    handler.send_header('Connection', 'close')
    handler.end_headers()
    try:
        while True:
            try:
                payload = json.dumps(feed.message(snapshot(), time.monotonic()), ensure_ascii=False, allow_nan=False)
            except ValueError:
                payload = None          # 某个字段不是有限数：这一帧不发，下一帧再来
            if payload is not None:
                handler.wfile.write(('data: ' + payload + '\n\n').encode('utf-8'))
                handler.wfile.flush()
            time.sleep(STREAM_PERIOD_S)
    except OSError:
        pass
