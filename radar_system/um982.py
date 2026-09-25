#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""和芯星通 UM982（雷迅 CUAV 双天线 RTK）输出解析、GNSS 时间换算和 NTRIP 客户端。不依赖 ROS。

接收机 COM2 已配置为 20 Hz 输出 BESTPOSA / HEADINGA / BESTVELA，1 Hz 输出 GGA 等 NMEA
（配置记录与备份在车上 ~/rtk/）。字段顺序对照《Unicore N4 高精度命令手册》。

    msg = parse_unicore(line)     # BestPos / Heading / BestVel；其他日志返回 None；校验失败抛 ParseError
"""
import base64
import math
import os
import socket
import stat
import threading
import time
from collections import deque
from dataclasses import dataclass

GPS_WEEK_S = 604800.0

# 位置解类型分级，与融合模块（gnss_fusion）的取舍一致：只有多频窄巷固定解 NARROW_INT 能用于控制。
# WIDE_INT / L1_INT 虽然整周已固定，但只是宽巷/单频解，融合不拿来更新位置，状态里归到"浮点"一档，
# 免得界面显示"固定"却记不了点。
RTK_FIXED = frozenset({'NARROW_INT'})
RTK_FLOAT = frozenset({'NARROW_FLOAT', 'L1_FLOAT', 'IONOFREE_FLOAT', 'WIDE_INT', 'L1_INT'})
# 双天线航向：整周模糊度固定才可信
HEADING_FIXED = frozenset({'L1_INT', 'NARROW_INT', 'WIDE_INT'})
# 双天线基线不写死长度：两根天线都是 RTK 固定解时，按同一测量时刻两者位置的实时距离作参考
# （最近 BASELINE_REF_WINDOW_S 秒的中位数，至少 BASELINE_REF_MIN_SAMPLES 组）。用户要求：两根天线都固定
# 就信——测向是整周固定解、且有这个实时参考（两根都固定）就算有效，不再比较测向长度。参考距离小于
# MIN_BASELINE_M 说明两根天线收到的是同一个信号（室内转发器），不算。
BASELINE_REF_WINDOW_S = 3.0
BASELINE_REF_MIN_SAMPLES = 5
MIN_BASELINE_M = 0.10
_WGS84_A, _WGS84_E2 = 6378137.0, 6.69437999014e-3
# 日志第一个字段是解算状态；不是 SOL_COMPUTED 时，解类型字段不可信，按无解处理
SOL_OK = 'SOL_COMPUTED'


class ParseError(ValueError):
    pass


def _crc32_table():
    table = []
    for n in range(256):
        c = n
        for _ in range(8):
            c = (c >> 1) ^ 0xEDB88320 if c & 1 else c >> 1
        table.append(c)
    return table


_CRC_TABLE = _crc32_table()


def crc32(data):
    """Unicore/NovAtel ASCII 日志的 CRC32：初值 0、无末尾取反，范围是 '#' 与 '*' 之间的字符。"""
    c = 0
    for b in data:
        c = _CRC_TABLE[(c ^ b) & 0xFF] ^ (c >> 8)
    return c


def nmea_checksum(body):
    c = 0
    for b in body.encode('ascii'):
        c ^= b
    return c


@dataclass(frozen=True)
class BestPos:
    week: int
    tow: float                # GPS 周内秒（解算时刻）
    sol_status: str
    pos_type: str
    lat_deg: float
    lon_deg: float
    height_m: float           # 椭球高 = 海拔 + 大地水准面差距
    sigma_lat: float          # m
    sigma_lon: float
    sigma_hgt: float
    diff_age_s: float         # 无差分时为 inf
    sats_tracked: int
    sats_used: int
    antenna: int = 1


@dataclass(frozen=True)
class Heading:
    week: int
    tow: float
    sol_status: str
    pos_type: str
    length_m: float           # 接收机测得的基线长度
    heading_deg: float        # ANT1→ANT2，从正北顺时针
    pitch_deg: float
    sigma_heading_deg: float
    sigma_pitch_deg: float
    sats_tracked: int
    sats_used: int


@dataclass(frozen=True)
class BestVel:
    week: int
    tow: float
    sol_status: str
    vel_type: str
    latency_s: float          # 速度相对位置时刻的延迟（多普勒测速为 0）
    speed_mps: float          # ANT1 水平速度
    track_deg: float          # 对地航迹，从正北顺时针
    vert_mps: float


def _split(line):
    line = line.strip()
    if not line.startswith('#') or '*' not in line or ';' not in line:
        raise ParseError('not a unicore ascii log')
    body, crc = line[1:].rsplit('*', 1)
    try:
        expected = int(crc[:8], 16)
    except ValueError as exc:
        raise ParseError('bad crc field') from exc
    if crc32(body.encode('ascii', 'replace')) != expected:
        raise ParseError('crc mismatch')
    head, data = body.split(';', 1)
    return head.split(','), _fields(data)


def _fields(data):
    # 字段里可能有带引号的基站号，里面不会有逗号
    return [f.strip('"') for f in data.split(',')]


def _time(head):
    try:
        if head[0] == 'BESTNAVHA':
            # N4 原生日志的周数比兼容日志早一列，周内时间的单位是毫秒。
            return int(head[4]), float(head[5]) / 1000.0
        return int(head[5]), float(head[6])
    except (IndexError, ValueError) as exc:
        raise ParseError('bad header time') from exc


def _age(text):
    value = float(text)
    return value if value > 0 else math.inf


def parse_unicore(line):
    """解析定位、独立副天线定位、航向和速度日志，其余日志返回 None。"""
    if not line.startswith(('#BESTPOSA', '#BESTNAVHA', '#HEADINGA', '#BESTVELA')):
        return None
    head, f = _split(line)
    week, tow = _time(head)
    try:
        if head[0] in ('BESTPOSA', 'BESTNAVHA'):
            return BestPos(week, tow, f[0], f[1], float(f[2]), float(f[3]),
                           float(f[4]) + float(f[5]), float(f[7]), float(f[8]), float(f[9]),
                           _age(f[11]), int(f[13]), int(f[14]), 2 if head[0] == 'BESTNAVHA' else 1)
        if head[0] == 'HEADINGA':
            return Heading(week, tow, f[0], f[1], float(f[2]), float(f[3]), float(f[4]),
                           float(f[6]), float(f[7]), int(f[9]), int(f[10]))
        if head[0] == 'BESTVELA':
            return BestVel(week, tow, f[0], f[1], float(f[2]), float(f[4]), float(f[5]), float(f[6]))
    except (IndexError, ValueError) as exc:
        raise ParseError('bad %s fields' % head[0]) from exc
    return None


def parse_gga(line):
    """校验并拆分 NMEA GGA；返回字段列表，校验失败返回 None。"""
    line = line.strip()
    if not line.startswith(('$GNGGA', '$GPGGA')) or '*' not in line:
        return None
    body, crc = line[1:].rsplit('*', 1)
    try:
        if nmea_checksum(body) != int(crc[:2], 16):
            return None
    except ValueError:
        return None
    return body.split(',')


def compact_gga(fields):
    """把高精度 GGA 精简成标准长度（≤82 字符）上报 VRS；无定位时返回 None。"""
    if len(fields) < 15 or fields[6] in ('', '0') or not fields[2] or not fields[4]:
        return None
    body = 'GPGGA,%s.00,%.5f,%s,%.5f,%s,%s,%s,%s,%.1f,M,%.1f,M,,' % (
        fields[1].split('.')[0], float(fields[2]), fields[3], float(fields[4]), fields[5],
        fields[6], fields[7], fields[8], float(fields[9] or 0), float(fields[11] or 0))
    return '$%s*%02X\r\n' % (body, nmea_checksum(body))


class GnssClock:
    """GNSS 解算时刻 → 本机单调时钟。

    串口传输只会让数据晚到、不会早到，所以窗口内 (收到时刻 - GPS 时间) 的最小值最接近真实钟差；
    再减去接收机解算输出本身的延迟。窗口保持几秒，跟得上两个时钟之间的 ppm 级漂移。
    """

    def __init__(self, window_s=10.0, solution_latency_s=0.02, max_jump_s=1.0):
        self.window_s, self.latency, self.max_jump = window_s, solution_latency_s, max_jump_s
        self.samples = deque()
        self.offset = None

    def measurement_time(self, week, tow, t_rx):
        gps = week * GPS_WEEK_S + tow
        diff = t_rx - gps
        if self.offset is not None and abs(diff - self.offset) > self.max_jump:
            # 接收机重启或时间跳变：丢掉旧窗口重新估计
            self.samples.clear()
        while self.samples and self.samples[0][0] < t_rx - self.window_s:
            self.samples.popleft()
        # 单调队列维护窗口最小值
        while self.samples and self.samples[-1][1] >= diff:
            self.samples.pop()
        self.samples.append((t_rx, diff))
        self.offset = self.samples[0][1]
        return gps + self.offset - self.latency


# =============================================================================
# NTRIP：从 CORS 取 RTCM 改正数写进接收机
# =============================================================================

def load_ntrip_env(path):
    """读账号文件；文件不存在返回 None。权限对组/其他用户开放时拒绝使用。"""
    if not os.path.exists(path):
        return None
    mode = stat.S_IMODE(os.stat(path).st_mode)
    if mode & 0o077:
        raise PermissionError('%s 权限是 %o，先 chmod 600' % (path, mode))
    env = {}
    with open(path, encoding='utf-8') as fh:
        for line in fh:
            line = line.strip()
            if line and not line.startswith('#') and '=' in line:
                k, v = line.split('=', 1)
                env[k.strip()] = v.strip()
    missing = [k for k in ('NTRIP_HOST', 'NTRIP_PORT', 'NTRIP_MOUNT', 'NTRIP_USER', 'NTRIP_PASS')
               if not env.get(k)]
    if missing:
        raise ValueError('ntrip.env 缺少 ' + ', '.join(missing))
    return env


def rtcm_frames(buf):
    """从字节流里切出完整 RTCM3 帧的消息类型（诊断用，不校验 CRC）。返回 (类型列表, 剩余字节)。"""
    types, i = [], 0
    while True:
        i = buf.find(b'\xd3', i)
        if i < 0:
            return types, b''
        if len(buf) - i < 6:
            return types, buf[i:]
        n = ((buf[i + 1] & 0x03) << 8) | buf[i + 2]
        if len(buf) - i < n + 6:
            return types, buf[i:]
        types.append((buf[i + 3] << 4) | (buf[i + 4] >> 4))
        i += n + 6


class NtripClient:
    """后台线程：连 CORS、定时上报 GGA、把改正数交给 sink。账号密码只在内存里，不写日志。"""

    def __init__(self, env, gga_source, sink, gga_period_s=5.0, idle_timeout_s=30.0):
        self.env = env
        self.mounts = [env['NTRIP_MOUNT']] + ([env['NTRIP_MOUNT_FALLBACK']]
                                              if env.get('NTRIP_MOUNT_FALLBACK') else [])
        self.gga_source, self.sink = gga_source, sink
        self.gga_period, self.idle_timeout = gga_period_s, idle_timeout_s
        self.stop_event = threading.Event()
        self.session_lock = threading.Lock()
        self.sock = None
        self.lock = threading.Lock()
        self.state = dict(connected=False, mount=None, bytes=0, last_rx=None, rtcm_types=[],
                          error=None, rejected=[])
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def stop(self):
        self.stop_event.set()
        with self.session_lock:
            if self.sock is not None:
                try:
                    self.sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                self.sock.close()
        if self.thread.is_alive() and self.thread is not threading.current_thread():
            self.thread.join(timeout=1.0)
        self._set(connected=False)

    def _deliver(self, data):
        # stop 返回后旧连接不能继续往接收机写改正数。
        with self.session_lock:
            if not self.stop_event.is_set():
                self.sink(data)

    def status(self):
        with self.lock:
            return dict(self.state)

    def _set(self, **kw):
        with self.lock:
            self.state.update(kw)

    def _run(self):
        idx = 0
        while not self.stop_event.is_set():
            mount = self.mounts[idx]
            try:
                result = self._session(mount)
            except OSError as exc:
                if self.stop_event.is_set():
                    break
                self._set(connected=False, error='%s: %s' % (mount, exc))
                result = 'dropped'
            if result == 'rejected':
                with self.lock:
                    self.state['rejected'] = self.state['rejected'] + [mount]
                if idx + 1 < len(self.mounts):
                    idx += 1
                    continue
                # 所有接入点都被拒：账号问题，慢速重试，不刷屏
                self.stop_event.wait(60.0)
                idx = 0
                continue
            self.stop_event.wait(3.0)

    def _session(self, mount):
        auth = base64.b64encode(('%s:%s' % (self.env['NTRIP_USER'],
                                            self.env['NTRIP_PASS'])).encode()).decode()
        request = ('GET /%s HTTP/1.0\r\nUser-Agent: NTRIP rk3588-rtk/1.0\r\nAccept: */*\r\n'
                   'Authorization: Basic %s\r\nConnection: close\r\n\r\n' % (mount, auth))
        sock = socket.create_connection((self.env['NTRIP_HOST'], int(self.env['NTRIP_PORT'])),
                                        timeout=10)
        try:
            with self.session_lock:
                if self.stop_event.is_set():
                    return 'dropped'
                self.sock = sock
            sock.sendall(request.encode())
            head = b''
            while b'\r\n' not in head and len(head) < 2048:
                chunk = sock.recv(1024)
                if not chunk:
                    break
                head += chunk
            first = head.split(b'\r\n')[0].decode('ascii', 'replace')
            status = first.split()
            if len(status) < 2 or status[0] not in ('ICY', 'HTTP/1.0', 'HTTP/1.1') or status[1] != '200':
                code = status[1] if len(status) >= 2 and status[1].isdigit() else 'invalid_reply'
                self._set(connected=False, error='%s: %s' % (mount, code))
                return 'rejected'
            # HTTP 头可能分包；必须读完，不能把后半段头写进接收机。
            if status[0].startswith('HTTP/'):
                while b'\r\n\r\n' not in head:
                    if len(head) >= 8192 or self.stop_event.is_set():
                        raise OSError('invalid caster header')
                    chunk = sock.recv(1024)
                    if not chunk:
                        raise OSError('incomplete caster header')
                    head += chunk
                rest = head.split(b'\r\n\r\n', 1)[1]
            else:
                rest = head.split(b'\r\n', 1)[1].lstrip(b'\r\n')
            self._set(connected=True, mount=mount, error=None)
            sock.settimeout(1.0)
            # 连上后立即上报第一条 GGA，VRS 要先知道大概位置才开始发改正数
            pending, last_gga, last_rx = b'', None, time.monotonic()
            if rest:
                pending = self._corrections(rest, pending, last_rx)
            while not self.stop_event.is_set():
                now = time.monotonic()
                if last_gga is None or now - last_gga >= self.gga_period:
                    gga = self.gga_source()
                    if gga:
                        sock.sendall(gga.encode('ascii'))
                        last_gga = now
                try:
                    data = sock.recv(4096)
                except socket.timeout:
                    if now - last_rx > self.idle_timeout:
                        self._set(connected=False, error='%s: no corrections' % mount)
                        return 'dropped'
                    continue
                if not data:
                    if self.stop_event.is_set():
                        return 'dropped'
                    self._set(connected=False, error='%s: closed by caster' % mount)
                    return 'dropped'
                last_rx = time.monotonic()
                pending = self._corrections(data, pending, last_rx)
            return 'dropped'
        finally:
            sock.close()
            with self.session_lock:
                if self.sock is sock:
                    self.sock = None
            self._set(connected=False)

    def _corrections(self, data, pending, now):
        self._deliver(data)
        types, pending = rtcm_frames(pending + data)
        with self.lock:
            self.state['bytes'] += len(data)
            self.state['last_rx'] = now
            if types:
                self.state['rtcm_types'] = sorted(set(self.state['rtcm_types']) | set(types))
        return pending


# =============================================================================
# 串口字节流 → 带测量时刻的消息（RTK 节点的纯逻辑部分）
# =============================================================================

def _finite(v):
    return v if math.isfinite(v) else None


class Um982Stream:
    """把串口字节流切成行、解析、换算测量时刻，输出可直接 JSON 化的消息。

    消息 kind：fix（BESTPOS）、heading（HEADING）、vel（BESTVEL）。t 是本机单调时钟的测量时刻，
    t_rx 是收到时刻。无差分龄期等无穷值写成 None。status 是接收机报的解算状态；它不是 SOL_COMPUTED
    时 sol 一律写成 NONE，下游（融合、界面）只看 sol 就不会把无效解当真。
    """

    RATE_WINDOW_S = 2.0

    def __init__(self, clock=None):
        self.clock = clock or GnssClock()
        self.secondary_clock = GnssClock()
        self.buf = b''
        self.gga = None                 # 最新一条通过校验的 GGA 字段
        self.last = {}                  # kind -> 最新消息
        self.recent_fix = deque(maxlen=40)  # 最近 2 s 的主天线定位，按测量时刻和副天线配对
        self.baseline_pairs = deque()       # (测量时刻, 两天线固定解的实时距离)
        self.times = {k: deque() for k in ('fix', 'fix2', 'heading', 'vel')}
        self.crc_errors = 0
        self.last_rx = None

    def feed(self, data, t_rx):
        self.buf += data
        *lines, self.buf = self.buf.split(b'\n')
        if len(self.buf) > 4096:        # 没有换行的垃圾数据，别让缓冲无限长
            self.buf = b''
        out = []
        for raw in lines:
            line = raw.decode('ascii', 'replace').strip()
            if line.startswith(('$GNGGA', '$GPGGA')):
                fields = parse_gga(line)
                if fields:
                    self.gga = fields
                continue
            try:
                m = parse_unicore(line)
            except ParseError:
                self.crc_errors += 1
                continue
            if m is None:
                continue
            msg = self._message(m, t_rx)
            self.last[msg['kind']] = msg
            if msg['kind'] == 'fix':
                self.recent_fix.append(msg)
            elif msg['kind'] == 'fix2':
                d = self._antenna_offset()
                if d is not None:
                    self.baseline_pairs.append((msg['t'], d))
            elif msg['kind'] == 'heading':
                # 下游（rtk_node 发布、走线融合）都按这里的 valid 取舍，不再各自按固定长度判断
                ref = self._baseline_ref(msg['t'])
                msg['baseline_ref'] = None if ref is None else round(ref, 3)
                msg['valid'] = bool(msg['sol'] in HEADING_FIXED and ref is not None)
            q = self.times[msg['kind']]
            q.append(t_rx)
            while q and q[0] < t_rx - self.RATE_WINDOW_S:
                q.popleft()
            self.last_rx = t_rx
            out.append(msg)
        return out

    def _message(self, m, t_rx):
        secondary = isinstance(m, BestPos) and m.antenna == 2
        clock = self.secondary_clock if secondary else self.clock
        t = clock.measurement_time(m.week, m.tow, t_rx)
        ok = m.sol_status == SOL_OK
        if isinstance(m, BestPos):
            return dict(kind='fix2' if secondary else 'fix', t=t, t_rx=t_rx,
                        antenna=m.antenna, source='BESTNAVH' if secondary else 'BESTPOS',
                        lat=m.lat_deg, lon=m.lon_deg, height=m.height_m,
                        sol=m.pos_type if ok else 'NONE', status=m.sol_status,
                        sigma_n=m.sigma_lat, sigma_e=m.sigma_lon, sigma_u=m.sigma_hgt,
                        diff_age=_finite(m.diff_age_s), sats=m.sats_used, sats_tracked=m.sats_tracked)
        if isinstance(m, Heading):
            return dict(kind='heading', t=t, t_rx=t_rx, heading=m.heading_deg, pitch=m.pitch_deg,
                        length=m.length_m, sol=m.pos_type if ok else 'NONE', status=m.sol_status,
                        sigma_heading=m.sigma_heading_deg, sats=m.sats_used, sats_tracked=m.sats_tracked)
        return dict(kind='vel', t=t - m.latency_s, t_rx=t_rx, speed=m.speed_mps, track=m.track_deg,
                    sol=m.vel_type if ok else 'NONE', status=m.sol_status)

    def gga_report(self):
        return compact_gga(self.gga) if self.gga else None

    def status(self, now):
        fix, hd = self.last.get('fix'), self.last.get('heading')
        def antenna(kind, source):
            m = self.last.get(kind)
            age = max(0.0, now - m['t_rx']) if m else None
            fresh = age is not None and age < 2.5
            sol = m['sol'] if fresh else None
            return dict(available=fresh, source=source, age_s=round(age, 1) if age is not None else None,
                        sol=sol, fix=fix_grade(sol), sats_used=m['sats'] if fresh else None,
                        sats_tracked=m['sats_tracked'] if fresh else None)
        antennas = dict(ant1=antenna('fix', 'BESTPOS'), ant2=antenna('fix2', 'BESTNAVH'))
        fix = fix if antennas['ant1']['available'] else None
        hd = hd if hd and now - hd['t_rx'] < 1.0 else None
        # 副天线离主天线的实时距离照实报（两根都固定时）；两根都固定就信，不再做一致性判断
        antennas['ant2'].update(offset_m=None, consistent=None)
        if antennas['ant1']['available'] and antennas['ant2']['available']:
            offset = self._antenna_offset()
            if offset is not None:
                antennas['ant2']['offset_m'] = round(offset, 3)
        rates = {k: round(len(q) / self.RATE_WINDOW_S, 1) if q and now - q[-1] < self.RATE_WINDOW_S else 0.0
                 for k, q in self.times.items()}
        return dict(
            receiver_ok=self.last_rx is not None and now - self.last_rx < 1.0,
            antennas=antennas,
            rates=rates, crc_errors=self.crc_errors,
            sol=fix['sol'] if fix else None,
            rtk=('FIXED' if fix and fix['sol'] in RTK_FIXED else
                 'FLOAT' if fix and fix['sol'] in RTK_FLOAT else
                 'SINGLE' if fix and fix['sol'] in ('SINGLE', 'PSRDIFF') else 'NONE'),
            sats=fix['sats'] if fix else 0,
            diff_age=fix['diff_age'] if fix else None,
            sigma_h=round(max(fix['sigma_n'], fix['sigma_e']), 4) if fix else None,
            heading_sol=hd['sol'] if hd else None,
            heading_ok=bool(hd and hd.get('valid')),
            baseline_m=hd['length'] if hd else None,
            # 双天线基线（测向）解：两根天线的联合解，卫星数是两根天线共同参与解算的，单独上报，不当作右天线的
            heading=None if not hd else dict(
                sol=hd['sol'], fix=heading_grade(hd['sol']), sats_used=hd['sats'],
                sats_tracked=hd.get('sats_tracked'), length_m=round(hd['length'], 3),
                pitch_deg=round(hd['pitch'], 1), sigma_deg=round(hd['sigma_heading'], 2),
                ref_m=hd.get('baseline_ref'), valid=bool(hd.get('valid'))))

    def _baseline_ref(self, t):
        """t 时刻的基线参考：最近窗口内两天线固定解距离的中位数；样本不够或短于 MIN_BASELINE_M 返回 None。"""
        while self.baseline_pairs and self.baseline_pairs[0][0] < t - BASELINE_REF_WINDOW_S:
            self.baseline_pairs.popleft()
        if len(self.baseline_pairs) < BASELINE_REF_MIN_SAMPLES:
            return None
        d = sorted(v for _, v in self.baseline_pairs)[len(self.baseline_pairs) // 2]
        return d if d >= MIN_BASELINE_M else None

    def _antenna_offset(self):
        """副天线最新定位与同一测量时刻（±30 ms）主天线定位之间的空间距离（米）；配不上返回 None。"""
        b = self.last.get('fix2')
        a = min(self.recent_fix, key=lambda f: abs(f['t'] - b['t']), default=None) if b else None
        # 两边都是 RTK 固定解才比：单点、浮点本身就有分米到米级误差，比出来的距离说明不了副天线对不对
        if a is None or abs(a['t'] - b['t']) > 0.03 or not {a['sol'], b['sol']} <= RTK_FIXED:
            return None
        phi = math.radians(a['lat'])
        w = 1.0 - _WGS84_E2 * math.sin(phi) ** 2
        dn = math.radians(b['lat'] - a['lat']) * _WGS84_A * (1.0 - _WGS84_E2) / w ** 1.5
        de = math.radians(b['lon'] - a['lon']) * _WGS84_A / math.sqrt(w) * math.cos(phi)
        return math.sqrt(dn * dn + de * de + (b['height'] - a['height']) ** 2)


def heading_grade(sol):
    """测向解分级：整周固定（含 L1/宽巷）为 FIXED，浮点为 FLOAT，其余为 NONE。"""
    return 'FIXED' if sol in HEADING_FIXED else 'FLOAT' if sol and 'FLOAT' in sol else 'NONE'


def fix_grade(sol):
    return ('FIXED' if sol in RTK_FIXED else 'FLOAT' if sol in RTK_FLOAT else
            'SINGLE' if sol in ('SINGLE', 'PSRDIFF') else 'NONE')
