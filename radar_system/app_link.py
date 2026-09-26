#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手机 app 与车之间的连接层(纯 Python 标准库 + numpy,不依赖 ROS)。

radar_web_server.py 用它给安卓 app 提供三样东西:

1. ``/ws`` WebSocket 长连接(RFC 6455,标准库实现,板端不用装新包)
   - 车 -> 手机:``hello``(车体尺寸、相机参数)一次,之后 ``tel`` 遥测 10Hz;
   - 手机 -> 车:摇杆、跟随开关、急停/复位、点选目标、ping;
   - 连接断开或摇杆心跳超时,车端自己停车,不依赖手机。
2. ``/video.mjpg`` 相机 MJPEG(见 radar_web_server.VideoHub);
3. UDP 广播 ``DISCOVERY_PORT``,app 在路由器 WiFi 或车载热点下都能自动找到车。

协议字段见 docs/APP_PROTOCOL.md。这里只做编码、校验和几何换算,
真正的运动指令仍走 MotionClient / ManualDriveLatch / MotionAuthority 那一套。
"""
import base64
import hashlib
import json
import math
import struct
import threading

import numpy as np

PROTOCOL_VERSION = 1
DISCOVERY_PORT = 48620
DISCOVERY_MAGIC = 'rk3588-follow'
WS_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'

OP_CONT, OP_TEXT, OP_BINARY, OP_CLOSE, OP_PING, OP_PONG = 0x0, 0x1, 0x2, 0x8, 0x9, 0xA


# =============================================================================
# WebSocket(服务端)
# =============================================================================

def accept_key(key):
    """Sec-WebSocket-Key -> Sec-WebSocket-Accept。

    >>> accept_key('dGhlIHNhbXBsZSBub25jZQ==')
    's3pPLMBiTxaQ9kYGzzhZRbK+xOo='
    """
    digest = hashlib.sha1((key.strip() + WS_GUID).encode('ascii')).digest()
    return base64.b64encode(digest).decode('ascii')


class WebSocketClosed(Exception):
    pass


class WebSocket:
    """已完成握手的一条连接。send_* 线程安全;recv 只能在一个线程里调用。"""

    def __init__(self, rfile, wfile, max_message=65536):
        self.rfile, self.wfile = rfile, wfile
        self.max_message = max_message
        self._send_lock = threading.Lock()
        self.closed = False

    # ---- 发送 ----
    def _send(self, opcode, payload):
        if self.closed and opcode != OP_CLOSE:
            raise WebSocketClosed()
        n = len(payload)
        if n < 126:
            header = struct.pack('!BB', 0x80 | opcode, n)
        elif n < 65536:
            header = struct.pack('!BBH', 0x80 | opcode, 126, n)
        else:
            header = struct.pack('!BBQ', 0x80 | opcode, 127, n)
        with self._send_lock:
            try:
                self.wfile.write(header + payload)
                self.wfile.flush()
            except (OSError, ValueError) as exc:
                self.closed = True
                raise WebSocketClosed() from exc

    def send_text(self, text):
        self._send(OP_TEXT, text.encode('utf-8'))

    def send_json(self, obj):
        self.send_text(json.dumps(obj, ensure_ascii=False, separators=(',', ':'),
                                  allow_nan=False))

    def close(self, code=1000):
        if self.closed:
            return
        self.closed = True
        try:
            self._send(OP_CLOSE, struct.pack('!H', code))
        except WebSocketClosed:
            pass

    # ---- 接收 ----
    def _read_exact(self, n):
        data = b''
        while len(data) < n:
            chunk = self.rfile.read(n - len(data))
            if not chunk:
                raise WebSocketClosed()
            data += chunk
        return data

    def _read_frame(self):
        b0, b1 = self._read_exact(2)
        fin, opcode = bool(b0 & 0x80), b0 & 0x0F
        masked, n = bool(b1 & 0x80), b1 & 0x7F
        if n == 126:
            n = struct.unpack('!H', self._read_exact(2))[0]
        elif n == 127:
            n = struct.unpack('!Q', self._read_exact(8))[0]
        if n > self.max_message:
            self.close(1009)
            raise WebSocketClosed()
        mask = self._read_exact(4) if masked else None
        payload = self._read_exact(n) if n else b''
        if mask:
            payload = bytes(b ^ mask[i & 3] for i, b in enumerate(payload))
        return fin, opcode, payload

    def recv(self):
        """返回下一条完整的文本(str)或二进制(bytes)消息;对端关闭时抛 WebSocketClosed。"""
        parts, kind = [], None
        while True:
            fin, opcode, payload = self._read_frame()
            if opcode == OP_PING:
                self._send(OP_PONG, payload)
                continue
            if opcode == OP_PONG:
                continue
            if opcode == OP_CLOSE:
                self.close()
                raise WebSocketClosed()
            if opcode in (OP_TEXT, OP_BINARY):
                parts, kind = [payload], opcode
            elif opcode == OP_CONT and kind is not None:
                parts.append(payload)
            else:
                self.close(1002)
                raise WebSocketClosed()
            if sum(len(p) for p in parts) > self.max_message:
                self.close(1009)
                raise WebSocketClosed()
            if fin:
                data = b''.join(parts)
                return data.decode('utf-8') if kind == OP_TEXT else data


def client_frame(opcode, payload, mask=b'\x01\x02\x03\x04', fin=True):
    """按客户端格式(带掩码)编一帧,给测试和演示客户端用。"""
    n = len(payload)
    b0 = (0x80 if fin else 0) | opcode
    if n < 126:
        header = struct.pack('!BB', b0, 0x80 | n)
    elif n < 65536:
        header = struct.pack('!BBH', b0, 0x80 | 126, n)
    else:
        header = struct.pack('!BBQ', b0, 0x80 | 127, n)
    body = bytes(b ^ mask[i & 3] for i, b in enumerate(payload))
    return header + mask + body


# =============================================================================
# 手机 -> 车:指令校验
# =============================================================================

SPEED_TIERS = ('low', 'med', 'high')
LINE_ACTIONS = ('mark', 'start', 'stop', 'clear')
ROUTE_ACTIONS = ('set', 'start', 'clear')
RTK_STALE_S = 2.0      # /gnss/status 2 Hz
LINE_STALE_S = 1.5     # /line/status 5 Hz


class CommandError(ValueError):
    pass


def _finite(value, name, lo, hi):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CommandError(name + ' 必须是数字')
    value = float(value)
    if not math.isfinite(value):
        raise CommandError(name + ' 必须是有限数值')
    return max(lo, min(hi, value))


def parse_command(text):
    """把一条上行文本解析成已校验的指令 dict;不认识或不合法时抛 CommandError。"""
    try:
        msg = json.loads(text)
    except (TypeError, ValueError) as exc:
        raise CommandError('不是合法 JSON') from exc
    if not isinstance(msg, dict):
        raise CommandError('指令必须是 JSON 对象')
    kind = msg.get('type')
    out = {'type': kind}
    if 'id' in msg:
        out['id'] = str(msg['id'])[:32]
    if kind == 'drive':
        out['throttle'] = _finite(msg.get('throttle', 0.0), 'throttle', -1.0, 1.0)
        out['steer'] = _finite(msg.get('steer', 0.0), 'steer', -1.0, 1.0)
        tier = msg.get('tier', 'med')
        out['tier'] = tier if tier in SPEED_TIERS else 'med'
        seq = msg.get('seq')
        out['seq'] = int(seq) if isinstance(seq, int) and not isinstance(seq, bool) else None
    elif kind == 'follow':
        if not isinstance(msg.get('enable'), bool):
            raise CommandError('follow.enable 必须是 true/false')
        out['enable'] = msg['enable']
    elif kind == 'select_target':
        out['bearing_rad'] = _finite(msg.get('bearing_rad'), 'bearing_rad', -math.pi, math.pi)
    elif kind == 'ping':
        out['t'] = msg.get('t')
    elif kind == 'line':
        action = msg.get('action')
        if action not in LINE_ACTIONS:
            raise CommandError('line.action 必须是 ' + '/'.join(LINE_ACTIONS))
        out['action'] = action
        if action == 'mark':
            point = str(msg.get('point', '')).upper()
            if point not in ('A', 'B'):
                raise CommandError('line.point 必须是 A 或 B')
            out['point'] = point
        if action == 'start' and 'cruise' in msg:
            out['cruise'] = _finite(msg['cruise'], 'cruise', 0.05, 1.0)
    elif kind == 'route':
        action = msg.get('action')
        if action not in ROUTE_ACTIONS:
            raise CommandError('route.action 必须是 ' + '/'.join(ROUTE_ACTIONS))
        out['action'] = action
        if action == 'set' and 'route' not in msg:
            raise CommandError('route.set 需要 route')
        if action in ('set', 'start') and 'route' in msg:
            import route_plan   # 结构和取值范围在这里挡住；画得成画不成由 line_node 判断
            try:
                out['route'] = route_plan.parse_spec(msg['route']).to_dict(digits=4)
            except ValueError as exc:
                raise CommandError('route: ' + str(exc)) from exc
    elif kind == 'view':
        # 平板地图上正在看的图纸（导入文件、还没应用到车）或 null；只给车载屏幕跟着显示，不改车上存的路线
        f = msg.get('file')
        if f is None:
            out['file'] = None
        elif isinstance(f, dict):
            import route_plan
            try:
                route = route_plan.parse_spec(f.get('route')).to_dict(digits=4)
            except ValueError as exc:
                raise CommandError('view.file.route: ' + str(exc)) from exc
            origin = f.get('origin')
            if origin is not None:
                if not isinstance(origin, (list, tuple)) or len(origin) != 2:
                    raise CommandError('view.file.origin 必须是 [纬度, 经度] 或 null')
                origin = [_finite(origin[0], 'origin', -1e9, 1e9), _finite(origin[1], 'origin', -1e9, 1e9)]
                if abs(origin[0]) > 90.0 or abs(origin[1]) > 180.0:
                    raise CommandError('view.file.origin 经纬度超出范围')     # 不夹到范围里，错的原点不显示
            out['file'] = {'name': str(f.get('name') or '')[:64], 'route': route, 'origin': origin}
            if 'control_points' in f:
                controls = f['control_points']
                if not isinstance(controls, list) or not 2 <= len(controls) <= 400:
                    raise CommandError('view.file.control_points 至少 2 个，最多 400 个')
                points = []
                for point in controls:
                    if not isinstance(point, (list, tuple)) or len(point) != 2:
                        raise CommandError('control_points 每个点必须是 [东, 北]')
                    pair = [_finite(v, 'control_points', -math.inf, math.inf) for v in point]
                    if any(abs(v) > 1000 for v in pair):
                        raise CommandError('control_points 坐标超出 ±1000 m')
                    points.append(pair)
                out['file']['control_points'] = points
        else:
            raise CommandError('view.file 必须是对象或 null')
    elif kind not in ('estop', 'reset'):
        raise CommandError('未知指令类型: ' + str(kind)[:32])
    return out


def resolve_stick(throttle, steer, tier, tiers_mps, reverse_scale, geometry):
    """摇杆 (-1..1, -1..1) + 速度档 -> (vx, wz, 实际前轮转角度数)。

    速度上限、倒车折扣、舵角限位全部由车端按共享配置换算,手机只报比例;
    角速度按当前车速从转角反算,换档不改变转弯半径(与网页方向键同一规则)。
    """
    from robot_core.kinematics import yaw_from_steer
    throttle = max(-1.0, min(1.0, float(throttle)))
    steer = max(-1.0, min(1.0, float(steer)))
    if abs(throttle) < 0.05:
        return 0.0, 0.0, 0.0
    base = tiers_mps.get(tier, tiers_mps['med'])
    vx = throttle * base * (reverse_scale if throttle < 0 else 1.0)
    steer_rad = steer * geometry.max_steer_rad
    wz = yaw_from_steer(vx, steer_rad, geometry)
    return round(vx, 4), round(wz, 4), round(math.degrees(steer_rad), 1)


# =============================================================================
# 车 -> 手机:遥测
# =============================================================================

def encode_scan(ranges):
    """距离(米,0/非有限=无效) -> uint16 厘米小端 base64。720 束约 1.9KB。"""
    arr = np.asarray(ranges, dtype=np.float64)
    cm = np.where(np.isfinite(arr) & (arr > 0), np.round(arr * 100.0), 0.0)
    return base64.b64encode(np.clip(cm, 0, 65535).astype('<u2').tobytes()).decode('ascii')


def decode_scan(text):
    return (np.frombuffer(base64.b64decode(text), dtype='<u2').astype(np.float64) / 100.0)


def camera_bearing(vx, vy, sensors):
    """车体系点 -> 相机水平方位(左正),与检测框的 bearing_rad 同一约定。"""
    rel = math.atan2(vy - sensors['camera_y_m'], vx - sensors['camera_x_m']) - sensors['camera_yaw_rad']
    return math.atan2(math.sin(rel), math.cos(rel))


def mark_locked(persons, target, sensors, tolerance_rad=math.radians(6.0)):
    """在检测框里找跟随目标那一个,置 locked=True;目标不在画面里时都不标。"""
    for p in persons:
        p['locked'] = False
    if not target or not target.get('in_camera_view') or target.get('veh_x') is None:
        return persons
    want = camera_bearing(target['veh_x'], target['veh_y'], sensors)
    best, best_err = None, tolerance_rad
    for p in persons:
        b = p.get('bearing')
        if b is None:
            continue
        err = abs(math.atan2(math.sin(b - want), math.cos(b - want)))
        if err <= best_err:
            best, best_err = p, err
    if best is not None:
        best['locked'] = True
    return persons


def _num(value, digits=2):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return None
    return round(float(value), digits)


def build_hello(profile, *, name, camera, video_path='/video.mjpg', session=None):
    g, s = profile['geometry'], profile['sensors']
    tan = math.tan(g['max_steer_rad'])
    return {
        'type': 'hello', 'v': PROTOCOL_VERSION, 'name': name, 'session': session,
        'video': video_path,
        'car': {
            'front_m': g['front_m'], 'rear_m': g['rear_m'], 'half_width_m': g['half_width_m'],
            'wheelbase_m': g['wheelbase_m'], 'track_m': g['track_m'],
            'max_steer_deg': round(math.degrees(g['max_steer_rad']), 1),
            'min_turn_radius_m': round(g['wheelbase_m'] / tan + 0.5 * g['track_m'], 3),
            'lidar_x_m': s['lidar_x_m'], 'lidar_y_m': s['lidar_y_m'],
            'camera_x_m': s['camera_x_m'], 'camera_y_m': s['camera_y_m'],
            'camera_yaw_rad': s['camera_yaw_rad'],
        },
        'camera': camera,
        'speed_tiers': {k: profile['manual'][k + '_mps'] for k in SPEED_TIERS},
    }


def _fresh(section, now, max_age_s):
    rx = (section or {}).get('_rx_monotonic')
    return section if rx is not None and 0 <= now - rx <= max_age_s else None


def rtk_telemetry(gnss, now):
    """/gnss/status -> app 的 rtk 段；RTK 节点 2 s 没更新就只报 stale，不给旧的定位等级。"""
    g = _fresh(gnss, now, RTK_STALE_S)
    if g is None:
        return {'stale': True}
    n = g.get('ntrip') or {}
    return {'stale': False, 'fix': g.get('rtk', 'NONE'), 'sats': g.get('sats'),
            'diff_age': _num(g.get('diff_age'), 1), 'sigma_h': _num(g.get('sigma_h'), 3),
            'heading_ok': bool(g.get('heading_ok')), 'baseline_m': _num(g.get('baseline_m'), 3),
            'receiver_ok': bool(g.get('receiver_ok')),
            'antennas': {name: {
                'available': bool(a.get('available')), 'source': a.get('source'),
                'sol': a.get('sol'), 'fix': a.get('fix', 'NONE'),
                'sats_used': a.get('sats_used') if a.get('available') else None,
                'sats_tracked': a.get('sats_tracked') if a.get('available') else None,
                'age_s': _num(a.get('age_s'), 1),
                # 仅副天线：与主天线同一时刻的距离、是否与安装基线相符（不符时界面不能显示"固定"）
                'offset_m': _num(a.get('offset_m'), 3), 'consistent': a.get('consistent')}
                for name in ('ant1', 'ant2')
                for a in [(g.get('antennas') or {}).get(name) or {}]},
            'heading': None if not g.get('heading') else {
                k: g['heading'].get(k) for k in ('sol', 'fix', 'sats_used', 'sats_tracked', 'length_m', 'pitch_deg',
                                                  'sigma_deg', 'ref_m', 'valid')},
            'ntrip': {'configured': bool(n.get('configured')), 'connected': bool(n.get('connected')),
                      **({'enabled': n['enabled']} if isinstance(n.get('enabled'), bool) else {}),
                      'error': n.get('error'), 'age_s': _num(n.get('age_s'), 1),
                      'config_revision': n.get('config_revision'), 'config_error': n.get('config_error'),
                      'bytes': n.get('bytes', 0)}}


def line_telemetry(line, now):
    """/line/status -> app 的 line 段；走直线节点没在跑时只报 stale。"""
    ln = _fresh(line, now, LINE_STALE_S)
    if ln is None:
        return {'stale': True}
    tr = ln.get('tracker') or {}
    stats = tr.get('stats') or {}
    pose = ln.get('pose')
    return {
        'stale': False, 'phase': ln.get('phase'), 'message': ln.get('message'),
        'cruise': _num(ln.get('cruise')), 'motion_mode': ln.get('motion_mode'),
        'points': ln.get('points') or {},
        'pose': None if not pose else {
            'x': _num(pose.get('x'), 3), 'y': _num(pose.get('y'), 3), 'yaw_deg': _num(pose.get('yaw_deg'), 2),
            'speed': _num(pose.get('speed')), 'quality': pose.get('quality'), 'usable': bool(pose.get('usable')),
            'sigma_xy': _num(pose.get('sigma_xy'), 4), 'sigma_yaw_deg': _num(pose.get('sigma_yaw_deg'), 3),
            'reasons': pose.get('reasons') or []},
        'track': {'state': tr.get('state'), 'reason': tr.get('reason'), 'e': _num(tr.get('e'), 4),
                  'heading_err_deg': _num(tr.get('heading_err_deg'), 2), 's': _num(tr.get('s'), 3),
                  'length': _num(tr.get('length'), 3), 'v_cmd': _num(tr.get('v_cmd')),
                  'rms': _num(stats.get('rms_m'), 4), 'max': _num(stats.get('max_m'), 4),
                  # 连线路线倒车补角：当前档位（forward/reverse）、第几段 / 共几段、是否在换向点等车停稳
                  'gear': tr.get('gear', 'forward'), 'section': tr.get('section', 1),
                  'sections': tr.get('sections', 1), 'switching': bool(tr.get('switching', False))},
        'last_result': ln.get('last_result'),
        # 路线作业：地图原点（经纬度）、天线原始定位（融合位姿没有时也能在地图上画车的位置）、车上存的路线
        'task': ln.get('task', 'line'), 'origin': ln.get('origin'), 'antenna': ln.get('antenna'),
        'route': ln.get('route'), 'route_limits': ln.get('route_limits'),
        # 诊断页：下发 / 实际曲率、RTK 与融合推算之差（米，车体系沿车头 / 向左）、离群与重置次数
        'diag': _line_diag(ln.get('diag')),
        # 定位跳动（地图上标出来）：编号、本地坐标、经纬度、最大差值 m、向左分量、持续 s、距今 s
        'rtk_events': [{k: e.get(k) for k in ('id', 'x', 'y', 'lat', 'lon', 'd', 'left', 'dur', 'age_s')}
                       for e in (ln.get('rtk_events') or []) if isinstance(e, dict)],
    }


def _line_diag(d):
    if not isinstance(d, dict):
        return None
    return {'kappa_cmd': _num(d.get('kappa_cmd'), 4), 'kappa_act': _num(d.get('kappa_act'), 4),
            'yaw_rate': _num(d.get('yaw_rate'), 4), 'rtk_along_m': _num(d.get('rtk_along_m'), 4),
            'rtk_left_m': _num(d.get('rtk_left_m'), 4), 'position_outliers': d.get('position_outliers'),
            'position_resets': d.get('position_resets'), 'jumps': d.get('jumps')}


def build_telemetry(snap, *, now, seq, sensors, include_scan=True):
    """radar_web_server 的 state 快照 -> 紧凑遥测。follower 需已做过陈旧标记。"""
    f = snap.get('follower') or {}
    stale = bool(f.get('stale', True))
    target = None if stale else f.get('target')
    wt = snap.get('wheeltec') or {}
    vel = (wt.get('telemetry') or {}).get('velocity') or [None, None, None]
    motion = snap.get('motion') or {}
    ai = snap.get('ai_status') or {}
    manual = snap.get('manual_override') or {}

    persons = []
    for d in snap.get('ai_targets') or []:
        if not isinstance(d, dict) or str(d.get('label', '')).lower() != 'person':
            continue
        persons.append({'box': [d.get('x1'), d.get('y1'), d.get('x2'), d.get('y2')],
                        'conf': _num(d.get('conf')), 'dist': _num(d.get('distance')),
                        'bearing': _num(d.get('bearing_rad'), 4)})
    mark_locked(persons, target, sensors)

    out = {
        'type': 'tel', 'seq': seq, 't': round(now, 3),
        'link': {'lidar_hz': _num(snap.get('hz'), 1),
                 'cam_fps': _num(ai.get('fps'), 1),
                 'chassis_age_s': _num(snap.get('wheeltec_status_age_s')),
                 'chassis_ok': bool(wt.get('connected')) and not wt.get('holding', False)},
        'battery': {'v': _num(snap.get('voltage')), 'pct': snap.get('battery_pct')},
        'motion': {'mode': motion.get('mode', 'UNKNOWN'), 'healthy': bool(motion.get('healthy')),
                   'fresh': bool(snap.get('motion_fresh')),
                   'reason': motion.get('reason'),
                   'faults': motion.get('health_faults') or [],
                   'waiting_stationary': bool(motion.get('waiting_stationary')),
                   # RTK 路线/直线（NAVIGATION）是否经过雷达防撞；旧版底盘驱动没有这项为 None
                   'nav_lidar': (wt.get('guard') or {}).get('navigation_uses_lidar')},
        'drive': {'speed': _num(vel[0]), 'yaw_rate': _num(vel[2]),
                  'manual': bool(manual.get('active')),
                  'steer_deg': _num(manual.get('steer_deg'), 1)},
        'follow': {
            'running': bool(snap.get('follower_running')), 'stale': stale,
            'state': f.get('state', 'OFFLINE'), 'reason': f.get('limit_reason'),
            'phase': f.get('turnaround_phase'), 'plan': None if stale else f.get('turnaround_plan'),
            'aim': None if stale else f.get('aim_point'),
            'cap': _num(f.get('speed_cap_mps')), 'clearance': _num(f.get('path_clearance_m')),
            'aeb': bool(f.get('aeb_active')),
            'tracks': [] if stale else [
                {'id': t.get('id'), 'x': t.get('x'), 'y': t.get('y'),
                 'confirmed': t.get('confirmed'), 'target': t.get('target')}
                for t in (f.get('tracks') or []) if isinstance(t, dict)],
            'target': None if not target else {
                'id': target.get('track_id'), 'x': target.get('veh_x'), 'y': target.get('veh_y'),
                'gap': _num(target.get('gap_used')), 'bearing_deg': target.get('bearing_deg'),
                'speed': target.get('ground_speed'), 'in_camera': bool(target.get('in_camera_view')),
                'coasting': bool(target.get('coasting'))},
        },
        'persons': persons,
        'rtk': rtk_telemetry(snap.get('gnss'), now),
        'line': line_telemetry(snap.get('line'), now),
    }
    ranges = snap.get('ranges')
    if include_scan and ranges:
        out['scan'] = {'a0': _num(snap.get('angle_min', 0.0), 5),
                       'da': _num(snap.get('angle_increment', 2 * math.pi / len(ranges)), 6),
                       'r': encode_scan(ranges)}
    return out


# =============================================================================
# 视频:只在有人看时才编码,每个观看者拿最新一帧
# =============================================================================

class VideoHub:
    """相机帧 -> 最新 JPEG。ROS 回调先问 wants_frame(),没人看就连解码都省掉。"""

    def __init__(self, max_fps=12.0):
        self.period = 1.0 / max_fps
        self._cond = threading.Condition()
        self._clients = 0
        self._seq = 0
        self._jpeg = None
        self._stamp = None
        self._last = -1e9

    @property
    def clients(self):
        with self._cond:
            return self._clients

    def attach(self):
        with self._cond:
            self._clients += 1

    def detach(self):
        with self._cond:
            self._clients = max(0, self._clients - 1)

    def wants_frame(self, now):
        with self._cond:
            return self._clients > 0 and now - self._last >= self.period

    def publish(self, jpeg, stamp, now):
        with self._cond:
            self._seq += 1
            self._jpeg, self._stamp, self._last = jpeg, stamp, now
            self._cond.notify_all()

    def wait_next(self, after_seq, timeout):
        """等比 after_seq 新的一帧 -> (seq, jpeg, stamp);超时返回 None。"""
        with self._cond:
            if not self._cond.wait_for(lambda: self._seq > after_seq and self._jpeg is not None,
                                       timeout):
                return None
            return self._seq, self._jpeg, self._stamp


def mjpeg_part(jpeg, stamp, boundary=b'frame'):
    head = (b'--' + boundary + b'\r\nContent-Type: image/jpeg\r\nContent-Length: '
            + str(len(jpeg)).encode() + b'\r\n')
    if stamp is not None:
        head += b'X-Stamp: ' + ('%.3f' % stamp).encode() + b'\r\n'
    return head + b'\r\n' + jpeg + b'\r\n'


# =============================================================================
# 发现:路由器 WiFi 与车载热点都要能找到车
# =============================================================================

def broadcast_addresses(ip_output):
    """解析 `ip -o -4 addr show` 输出里各网卡的广播地址(不含回环)。

    >>> broadcast_addresses('2: wlan0    inet 192.168.2.40/24 brd 192.168.2.255 scope global wlan0\\n'
    ...                     '1: lo    inet 127.0.0.1/8 scope host lo\\n'
    ...                     '5: wlan1    inet 10.42.0.1/24 brd 10.42.0.255 scope global wlan1')
    ['192.168.2.255', '10.42.0.255']
    """
    out = []
    for line in ip_output.splitlines():
        parts = line.split()
        if 'brd' in parts:
            addr = parts[parts.index('brd') + 1]
            if not addr.startswith('127.') and addr not in out:
                out.append(addr)
    return out


def beacon(name, port, profile_hash_value):
    """UDP 发现广播内容。"""
    return json.dumps({'magic': DISCOVERY_MAGIC, 'v': PROTOCOL_VERSION, 'name': name,
                       'port': port, 'profile': profile_hash_value},
                      separators=(',', ':')).encode('utf-8')
