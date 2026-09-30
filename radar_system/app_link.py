#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""平板 app 与车之间的连接层(纯 Python 标准库,不依赖 ROS)。

radar_web_server.py 用它给平板 app 提供两样东西:

1. ``/ws`` WebSocket 长连接(RFC 6455,标准库实现,板端不用装新包)
   - 车 -> 平板:``hello``(车体尺寸、手动档位、直行参数)一次,之后 ``tel`` 遥测 10Hz;
   - 平板 -> 车:摇杆、IMU 直行、急停/复位、直线 / 路线 / 场地作业、ping;
   - 连接断开或摇杆心跳超时,车端自己停车,不依赖平板。
2. UDP 广播 ``DISCOVERY_PORT``,app 在路由器 WiFi 或车载热点下都能自动找到车。

协议字段见 docs/APP_PROTOCOL.md。这里只做编码、校验和几何换算,
真正的运动指令仍走 MotionClient / ManualDriveLatch / MotionAuthority 那一套。
"""
import base64
import hashlib
import json
import math
import struct
import threading

from heading_hold import HoldConfig

STRAIGHT = HoldConfig()     # IMU 定向直行的参数（hello 里告诉 app）

PROTOCOL_VERSION = 1
DISCOVERY_PORT = 48620
DISCOVERY_MAGIC = 'rk3588-follow'      # 早期名字，平板 app 按它认车，不改
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
ROUTE_ACTIONS = ('set', 'start', 'clear', 'pause', 'preview', 'resume', 'rework', 'field_start')
STRAIGHT_ACTIONS = ('go', 'stop')
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


def _route_distance(value, name, lo, hi):
    # Unlike joystick normalization, a requested repair interval must never be clamped.
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lo <= value <= hi:
        raise CommandError('%s 必须是 %g–%g 内的有限数值' % (name, lo, hi))
    return float(value)


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
    elif kind == 'straight':
        # IMU 定向直行：app 点开始后每 50 ms 发一次 go（同一个 run），点停止发 stop；车速、方向由车端定（heading_hold）
        action = msg.get('action')
        if action not in STRAIGHT_ACTIONS:
            raise CommandError('straight.action 必须是 ' + '/'.join(STRAIGHT_ACTIONS))
        out['action'] = action
        run = msg.get('run')
        if isinstance(run, bool) or not isinstance(run, int) or not 0 <= run < 2 ** 31:
            raise CommandError('straight.run 必须是作业编号')
        out['run'] = run
        seq = msg.get('seq')
        out['seq'] = int(seq) if isinstance(seq, int) and not isinstance(seq, bool) else None
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
        if action=='field_start':
            token=msg.get('preview_token')
            if not isinstance(token,str) or not 1<=len(token)<=80:raise CommandError('缺少规划标识')
            if not isinstance(msg.get('allow_outside',False),bool):raise CommandError('越界确认格式错误')
            out.update(preview_token=token,allow_outside=msg.get('allow_outside',False))
        if action in ('pause', 'preview', 'resume', 'rework'):
            if 'allow_outside' in msg:
                if type(msg['allow_outside']) is not bool:raise CommandError('越界确认格式错误')
                out['allow_outside']=msg['allow_outside']
            rev = msg.get('route_rev')
            if isinstance(rev, bool) or not isinstance(rev, int) or rev < 0:
                raise CommandError('route_rev 必须是路线版本号')
            out['route_rev'] = rev
            for key in ('job_id', 'preview_token'):
                if key in msg:
                    value = msg[key]
                    if not isinstance(value, str) or not 1 <= len(value) <= 80:
                        raise CommandError(key + ' 格式无效')
                    out[key] = value
            if action == 'pause' and 'job_id' not in out:
                raise CommandError('暂停需要 job_id')
            if action in ('resume', 'rework') and 'preview_token' not in out:
                raise CommandError('请先预览接入路径')
            if action == 'preview':
                operation = msg.get('operation')
                if operation not in ('resume', 'rework'):
                    raise CommandError('operation 必须是 resume 或 rework')
                out['operation'] = operation
                if operation == 'resume':
                    if 'job_id' not in out:
                        raise CommandError('续画需要 job_id')
                    out['overlap_m'] = _route_distance(msg.get('overlap_m', 0), 'overlap_m', 0, 0.5)
                else:
                    element = msg.get('element')
                    if isinstance(element, bool) or not isinstance(element, int) or not 0 <= element < 20000:
                        raise CommandError('element 必须是线段编号')
                    out['element'] = element
                    for key in ('from_m', 'to_m'):
                        out[key] = _route_distance(msg.get(key), key, 0, 1000000)
                    if out['to_m'] - out['from_m'] < 0.2:
                        raise CommandError('补画范围至少 0.2 m')
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


# 摇杆死区：比例小于它当 0。app 自己有死区，固定发 high、行程线性换成指令（刚出死区约 0.04，
# 选的最高速比最快档低时油门按比例缩），这里只挡数值噪声
STICK_DEADBAND = 0.02


def resolve_stick(throttle, steer, tier, tiers_mps, reverse_scale, geometry):
    """摇杆 (-1..1, -1..1) + 速度档 -> (vx, wz, 实际前轮转角度数)。

    速度上限、倒车折扣、舵角限位全部由车端按共享配置换算,手机只报比例;
    角速度按当前车速从转角反算,换档不改变转弯半径(与网页方向键同一规则)。
    """
    from robot_core.kinematics import yaw_from_steer
    throttle = max(-1.0, min(1.0, float(throttle)))
    steer = max(-1.0, min(1.0, float(steer)))
    if getattr(geometry, 'differential', False):
        return _differential_stick(throttle, steer, tier, tiers_mps, reverse_scale, geometry)
    if abs(throttle) < STICK_DEADBAND:
        return 0.0, 0.0, 0.0
    base = tiers_mps.get(tier, tiers_mps['med'])
    vx = throttle * base * (reverse_scale if throttle < 0 else 1.0)
    steer_rad = steer * geometry.max_steer_rad
    wz = yaw_from_steer(vx, steer_rad, geometry)
    return round(vx, 4), round(wz, 4), round(math.degrees(steer_rad), 1)


def _differential_stick(throttle, steer, tier, tiers_mps, reverse_scale, geometry):
    """差速车（两后轮差速、前面万向轮）：摇杆打横就原地转，没有前轮转角（返回 0°）。

    角速度上限 = 原地转向角速度 × 本档车速 / 最快档车速，慢档转得也慢；前进时同一比例的角速度，
    车速越快弯越大。倒车时往左打是车尾往左（与旧车、方向盘一样的手感），所以角速度反号。
    """
    moving = abs(throttle) >= STICK_DEADBAND
    if not moving and abs(steer) < STICK_DEADBAND:
        return 0.0, 0.0, 0.0
    base = tiers_mps.get(tier, tiers_mps['med'])
    vx = throttle * base * (reverse_scale if throttle < 0 else 1.0) if moving else 0.0
    wz = steer * geometry.pivot_rate_rad_s * base / max(tiers_mps.values())
    if vx < 0:
        wz = -wz
    return round(vx, 4), round(wz, 4), 0.0


# =============================================================================
# 车 -> 手机:遥测
# =============================================================================

def _num(value, digits=2):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return None
    return round(float(value), digits)


def build_hello(profile, *, name, session=None):
    g, drive = profile['geometry'], profile['drive']
    tan = math.tan(g['max_steer_rad'])
    differential = drive['type'] == 'differential'
    return {
        'type': 'hello', 'v': PROTOCOL_VERSION, 'name': name, 'session': session,
        'car': {
            'front_m': g['front_m'], 'rear_m': g['rear_m'], 'half_width_m': g['half_width_m'],
            'wheelbase_m': g['wheelbase_m'], 'track_m': g['track_m'],
            'max_steer_deg': round(math.degrees(g['max_steer_rad']), 1),
            'min_turn_radius_m': round(drive['min_radius_m'] if differential else g['wheelbase_m'] / tan + 0.5 * g['track_m'], 3),
            # 差速车能原地转，没有前轮转角；nozzle_x_m 是喷头（差速车连线路线的压线点）在后轴前多少米
            'drive': drive['type'], 'nozzle_x_m': drive['nozzle_x_m'],
        },
        'speed_tiers': {k: profile['manual'][k + '_mps'] for k in SPEED_TIERS},
        # IMU 定向直行（只用 IMU 偏航，不用 RTK）：固定车速、目标精度、偏差超过 abort_deg 停车
        'straight': {'speed_mps': STRAIGHT.speed, 'tolerance_deg': STRAIGHT.tolerance_deg,
                     'abort_deg': STRAIGHT.abort_deg},
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
            # 接收机数据从哪来：'usb'（车上串口）/ 'tablet'（平板串口转发，见 gnss_bridge）；旧 RTK 节点没有
            'source': g.get('source') if g.get('source') in ('usb', 'tablet') else None,
            'tablet_connected': bool((g.get('tablet') or {}).get('connected')),
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
            'heading': None if not g.get('heading') else dict(
                {k: g['heading'].get(k) for k in ('sol', 'fix', 'sats_used', 'sats_tracked', 'length_m', 'pitch_deg',
                                                   'sigma_deg', 'ref_m', 'valid')},
                # 双天线测出的车头方向（ENU 度，东 0 逆时针）：只在测向有效时给
                yaw_deg=_num(g['heading'].get('yaw_deg'), 2) if g['heading'].get('valid') is True else None),
            'ntrip': {'configured': bool(n.get('configured')), 'connected': bool(n.get('connected')),
                      **({'enabled': n['enabled']} if isinstance(n.get('enabled'), bool) else {}),
                      'error': n.get('error'), 'age_s': _num(n.get('age_s'), 1),
                      'config_revision': n.get('config_revision'), 'config_error': n.get('config_error'),
                      'bytes': n.get('bytes', 0),
                      # 车端 CORS 暂停的原因：'tablet' = 数据来自平板串口，差分由平板自己连；没暂停为 None
                      'paused': n.get('paused') if n.get('paused') in ('tablet',) else None}}


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
        **({'operations': ln['operations']} if ln.get('operations') is not None else {}),
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
                  # 连线路线倒车补角：当前档位（forward/reverse）、第几段 / 共几段、是否在换向点等车停稳；
                  # 差速车换段时是否正在原地转向
                  'gear': tr.get('gear', 'forward'), 'section': tr.get('section', 1),
                  'sections': tr.get('sections', 1), 'switching': bool(tr.get('switching', False)),
                  'pivoting': bool(tr.get('pivoting', False))},
        'last_result': ln.get('last_result'),
        # 路线作业：地图原点（经纬度）、天线原始定位（融合位姿没有时也能在地图上画车的位置）、车上存的路线
        'task': ln.get('task', 'line'), 'origin': ln.get('origin'), 'antenna': ln.get('antenna'),
        'route': ln.get('route'), 'route_limits': ln.get('route_limits'),
        # 诊断页：下发 / 实际曲率、RTK 与融合推算之差（米，车体系沿车头 / 向左）、离群与重置次数
        'diag': _line_diag(ln.get('diag')),
        # 车身姿态（IMU，imu_filter_madgwick）：横滚左侧抬起为正、俯仰车头压低为正（REP-103），度；
        # yaw_deg 是 IMU 自己的偏航（没有磁力计：相对 imu 组件启动时，左转为正，会漂）；没有时为 None
        'attitude': _line_attitude(ln.get('attitude')),
        # 倾斜补偿（融合按 IMU 横滚俯仰把天线三维杠杆臂投到水平面）：天线高度 m、在不在补、没补的原因
        # （no_height / attitude_stale / too_steep / imu_mismatch）、用的横滚俯仰°、补偿量（车体系前、左，m）；旧车端没有为 None
        'tilt': _line_tilt(ln.get('tilt')),
        # 定位跳动（地图上标出来）：编号、本地坐标、经纬度、最大差值 m、向左分量、持续 s、距今 s
        'rtk_events': [{k: e.get(k) for k in ('id', 'x', 'y', 'lat', 'lon', 'd', 'left', 'dur', 'age_s')}
                       for e in (ln.get('rtk_events') or []) if isinstance(e, dict)],
    }


def _line_attitude(a):
    if not isinstance(a, dict):
        return None
    roll, pitch = _num(a.get('roll_deg'), 2), _num(a.get('pitch_deg'), 2)
    if roll is None or pitch is None:
        return None
    return {'roll_deg': roll, 'pitch_deg': pitch, 'yaw_deg': _num(a.get('yaw_deg'), 2), 'source': a.get('source')}


def _line_tilt(t):
    if not isinstance(t, dict) or not isinstance(t.get('active'), bool):
        return None
    reason = t.get('reason')
    return {'height_m': _num(t.get('height_m'), 3), 'active': t['active'],
            'reason': reason if isinstance(reason, str) else None,
            'roll_deg': _num(t.get('roll_deg'), 2), 'pitch_deg': _num(t.get('pitch_deg'), 2),
            'forward_m': _num(t.get('forward_m'), 4), 'left_m': _num(t.get('left_m'), 4)}


def _line_diag(d):
    if not isinstance(d, dict):
        return None
    return {'kappa_cmd': _num(d.get('kappa_cmd'), 4), 'kappa_act': _num(d.get('kappa_act'), 4),
            'yaw_rate': _num(d.get('yaw_rate'), 4), 'rtk_along_m': _num(d.get('rtk_along_m'), 4),
            'rtk_left_m': _num(d.get('rtk_left_m'), 4), 'position_outliers': d.get('position_outliers'),
            'position_resets': d.get('position_resets'), 'jumps': d.get('jumps')}


STRAIGHT_KEYS = ('run', 'state', 'reason', 'target_deg', 'yaw_deg', 'error_deg', 'max_error_deg',
                 'within_pct', 'elapsed_s', 'speed', 'wz')


def straight_telemetry(st):
    """heading_hold.HeadingHold.status() → 遥测 straight；没有时 None（不带这一项）。数字只收有限值。
    车端的作业键是 (连接会话, app 的作业编号)：拆成 session 和 run，app 用 hello 里的 session 认自己的那次。"""
    if not isinstance(st, dict):
        return None
    out = {'imu_ok': bool(st.get('imu_ok')), 'state': st.get('state', 'idle')}
    for key in STRAIGHT_KEYS[2:]:
        if key in st:
            v = st[key]
            out[key] = v if isinstance(v, str) or v is None else _num(v, 3)
    run = st.get('run')
    if isinstance(run, tuple) and len(run) == 2:
        out['session'], run = run
    if isinstance(run, int) and not isinstance(run, bool):
        out['run'] = run
    return out


def build_telemetry(snap, *, now, seq):
    """radar_web_server 的 state 快照 -> 紧凑遥测。"""
    wt = snap.get('wheeltec') or {}
    vel = (wt.get('telemetry') or {}).get('velocity') or [None, None, None]
    motion = snap.get('motion') or {}
    manual = snap.get('manual_override') or {}

    out = {
        'type': 'tel', 'seq': seq, 't': round(now, 3),
        'link': {'chassis_age_s': _num(snap.get('wheeltec_status_age_s')),
                 'chassis_ok': bool(wt.get('connected')) and not wt.get('holding', False)},
        'battery': {'v': _num(snap.get('voltage')), 'pct': snap.get('battery_pct')},
        'motion': {'mode': motion.get('mode', 'UNKNOWN'), 'healthy': bool(motion.get('healthy')),
                   'fresh': bool(snap.get('motion_fresh')),
                   'reason': motion.get('reason'),
                   'faults': motion.get('health_faults') or [],
                   'waiting_stationary': bool(motion.get('waiting_stationary')),
                   # RTK 路线/直线（NAVIGATION）是否经过雷达防撞（旧阿克曼车的底盘防撞层）；Gen5 / 旧版驱动为 None
                   'nav_lidar': (wt.get('guard') or {}).get('navigation_uses_lidar')},
        'drive': {'speed': _num(vel[0]), 'yaw_rate': _num(vel[2]),
                  'manual': bool(manual.get('active')),
                  'steer_deg': _num(manual.get('steer_deg'), 1)},
        'rtk': rtk_telemetry(snap.get('gnss'), now),
        'line': line_telemetry(snap.get('line'), now),
    }
    straight = straight_telemetry(snap.get('straight'))
    if straight is not None:
        out['straight'] = straight
    return out


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
