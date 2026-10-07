#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""和芯星通 UM982（雷迅 CUAV 双天线 RTK）输出解析、GNSS 时间换算和 NTRIP 客户端。不依赖 ROS。

接收机 COM2 已配置为 20 Hz 输出 BESTPOSA / HEADINGA / BESTVELA，1 Hz 输出 GGA 等 NMEA
（配置记录与备份在车上 ~/rtk/）。字段顺序对照《Unicore N4 高精度命令手册》。
同样的四条日志也可以是二进制（BESTPOSB / HEADINGB / BESTVELB / BESTNAVHB，长度约为 ASCII 的一半，
平板经 CAN 转发时用），Um982Stream 两种都认，解出来是同样的对象。

    msg = parse_unicore(line)     # BestPos / Heading / BestVel；其他日志返回 None；校验失败抛 ParseError
    msg = parse_binary(frame)     # 同上，frame 是 CRC 校验过的整帧二进制
"""
import base64
import math
import os
import socket
import stat
import struct
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
# 用户规则（2026-09-29）：测向是整周固定解、且接收机报的基线长度在这个范围内就算有效，
# 不要求两根天线的位置也是 RTK 固定解。Gen5 两天线相距约 0.415 m；范围外（室内转发器约几毫米、
# 模糊度固定错了的长度）不算。
HEADING_BASELINE_MIN_M = 0.30
HEADING_BASELINE_MAX_M = 0.50
# 两根天线都是 RTK 固定解时，同一测量时刻两者平面坐标（东、北，不算高差）的实时距离照实上报
# （最近 BASELINE_REF_WINDOW_S 秒的中位数，至少 BASELINE_REF_MIN_SAMPLES 组），只作显示，不参与判定。
# 小于 MIN_BASELINE_M 说明两根天线收到的是同一个信号（室内转发器），不报。
BASELINE_REF_WINDOW_S = 3.0
BASELINE_REF_MIN_SAMPLES = 5
MIN_BASELINE_M = 0.10
POSITION_BASELINE_NOMINAL_M = .41  # 用户提供的已知天线间距；只用于位置观测权重，不否决固定解。
POSITION_BASELINE_SETTLE_S = .3
_WGS84_A, _WGS84_E2 = 6378137.0, 6.69437999014e-3
# 日志第一个字段是解算状态；不是 SOL_COMPUTED 时，解类型字段不可信，按无解处理
SOL_OK = 'SOL_COMPUTED'

# 二进制日志（2026-10-06 在接收机空闲口 COM1 上同一历元各抓 A、B 两种对照，字段逐一相同）：
# BESTPOS / HEADING / BESTVEL 是 NovAtel 兼容帧（同步字 AA 44 12，帧头长度在第 3 字节，28），
# BESTNAVH 是 N4 原生帧（同步字 AA 44 B5，帧头固定 24 字节）。CRC32 算法同 ASCII，覆盖帧头和数据，跟在数据后面。
SYNC_NOVATEL = b'\xaa\x44\x12'
SYNC_N4 = b'\xaa\x44\xb5'
# 消息 ID → 数据长度（字节）。长度不对就是帧头坏了（经 CAN 丢了一块），不等后面的数据。
BINARY_LOGS = {42: 72, 971: 44, 99: 44, 2119: 120}
BESTNAVH_ID = 2119
# 《N4 高精度命令手册》R1.4 表「Solution Status」「Position or Velocity Type」
SOL_STATUS = {0: 'SOL_COMPUTED', 1: 'INSUFFICIENT_OBS', 2: 'NO_CONVERGENCE', 4: 'COV_TRACE'}
POS_TYPE = {0: 'NONE', 1: 'FIXEDPOS', 2: 'FIXEDHEIGHT', 8: 'DOPPLER_VELOCITY', 16: 'SINGLE', 17: 'PSRDIFF',
            18: 'SBAS', 32: 'L1_FLOAT', 33: 'IONOFREE_FLOAT', 34: 'NARROW_FLOAT', 48: 'L1_INT', 49: 'WIDE_INT',
            50: 'NARROW_INT', 52: 'INS', 53: 'INS_PSRSP', 54: 'INS_PSRDIFF', 55: 'INS_RTKFLOAT', 56: 'INS_RTKFIXED',
            68: 'PPP_CONVERGING', 69: 'PPP'}


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


def _enum(table, value):
    return table.get(value, 'UNKNOWN_%d' % value)


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


def binary_frame(buf, i):
    """buf[i:] 以二进制同步字开头时：返回 ('frame', 结束位置)；数据还没到齐返回 ('need', None)；
    不是本模块认的四条日志或帧头对不上返回 ('skip', None)；长度够了但 CRC 不对返回 ('crc', None)。"""
    if len(buf) - i < 3:
        return 'need', None             # 同步字在缓冲末尾只到了一半
    sync = buf[i:i + 3]
    if sync == SYNC_NOVATEL:
        if len(buf) - i < 10:
            return 'need', None
        head = buf[i + 3]
        msg_id, length = struct.unpack_from('<H', buf, i + 4)[0], struct.unpack_from('<H', buf, i + 8)[0]
    elif sync == SYNC_N4:
        if len(buf) - i < 8:
            return 'need', None
        head = 24
        msg_id, length = struct.unpack_from('<HH', buf, i + 4)
    else:
        return 'skip', None
    if BINARY_LOGS.get(msg_id) != length or head != (28 if sync == SYNC_NOVATEL else 24):
        return 'skip', None
    end = i + head + length + 4
    if len(buf) < end:
        return 'need', None
    if crc32(buf[i:end - 4]) != struct.unpack_from('<I', buf, end - 4)[0]:
        return 'crc', None
    return 'frame', end


def parse_binary(frame):
    """CRC 校验过的整帧（见 binary_frame）→ BestPos / Heading / BestVel，字段与 parse_unicore 相同；别的日志返回 None。"""
    if frame[:3] == SYNC_NOVATEL:
        head, (msg_id,), (week, ms) = frame[3], struct.unpack_from('<H', frame, 4), struct.unpack_from('<HI', frame, 14)
    else:
        head, (msg_id,), (week, ms) = 24, struct.unpack_from('<H', frame, 4), struct.unpack_from('<HI', frame, 10)
    b, tow = frame[head:-4], ms / 1000.0
    if msg_id in (42, BESTNAVH_ID):
        sol, typ, lat, lon, hgt, und, _datum, s_lat, s_lon, s_hgt = struct.unpack_from('<IIdddfIfff', b, 0)
        (diff_age,) = struct.unpack_from('<f', b, 56)
        return BestPos(week, tow, _enum(SOL_STATUS, sol), _enum(POS_TYPE, typ), lat, lon, hgt + und,
                       s_lat, s_lon, s_hgt, diff_age if diff_age > 0 else math.inf, b[64], b[65],
                       2 if msg_id == BESTNAVH_ID else 1)
    if msg_id == 971:
        sol, typ, length, heading, pitch, _reserved, s_heading, s_pitch = struct.unpack_from('<IIffffff', b, 0)
        return Heading(week, tow, _enum(SOL_STATUS, sol), _enum(POS_TYPE, typ), length, heading, pitch,
                       s_heading, s_pitch, b[36], b[37])
    if msg_id == 99:
        sol, typ, latency, _age_s, speed, track, vert = struct.unpack_from('<IIffddd', b, 0)
        return BestVel(week, tow, _enum(SOL_STATUS, sol), _enum(POS_TYPE, typ), latency, speed, track, vert)
    return None


def _next_sync(buf, i):
    """i 起第一个二进制同步字（AA 44 12 / AA 44 B5）的位置；没有返回 -1。末尾不完整的同步字也算（等后续字节）。"""
    while True:
        j = buf.find(b'\xaa', i)
        if j < 0 or j + 1 >= len(buf) or (buf[j + 1] == 0x44 and (j + 2 >= len(buf) or buf[j + 2] in (0x12, 0xb5))):
            return j
        i = j + 1


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


def _hms_seconds(text):
    """hhmmss.sss → 当天 UTC 秒。$INS.NAVI 省掉了前导 0（4552.400 = 00:45:52.4），GGA 是完整的 004553.00。"""
    whole, _, frac = text.strip().partition('.')
    whole = whole.zfill(6)
    if len(whole) != 6 or not whole.isdigit() or (frac and not frac.isdigit()):
        raise ValueError('bad time ' + text)
    return int(whole[:2]) * 3600 + int(whole[2:4]) * 60 + int(whole[4:]) + (float('0.' + frac) if frac else 0.0)


def parse_ins_navi(line):
    """单天线流动站（HCYTJ S20241460，2026-10-07 实抓）的 $INS.NAVI：接收机自己的组合导航解，5 Hz。

    字段（和同一秒的 GGA 逐项对过）：1 UTC 时分秒（省前导 0）、2 纬度（度）、3 经度（度）、4 椭球高（m，= GGA 海拔 +
    大地水准面差距）、8/9/10 横滚/俯仰/航向（度，没对准时航向停在 10.0）；其余几项含义不明，不用。这句不带定位质量，
    是不是 RTK 固定看同一台接收机的 GGA（gga_quality）。
    不是这一句返回 None；校验失败、字段坏了抛 ParseError。返回 dict(tod, lat, lon, height)。
    """
    line = line.strip()
    if not line.startswith('$INS.NAVI,'):
        return None
    if '*' not in line:
        raise ParseError('INS.NAVI without checksum')
    body, crc = line[1:].rsplit('*', 1)
    try:
        if nmea_checksum(body) != int(crc[:2], 16):
            raise ParseError('bad INS.NAVI checksum')
        f = body.split(',')
        tod, lat, lon, height = _hms_seconds(f[1]), float(f[2]), float(f[3]), float(f[4])
    except (ValueError, IndexError) as exc:
        raise ParseError('bad INS.NAVI') from exc
    if not (0 <= tod < 86401 and -90 <= lat <= 90 and -180 <= lon <= 180 and math.isfinite(height)):
        raise ParseError('bad INS.NAVI values')
    return dict(tod=tod, lat=lat, lon=lon, height=height)


# GGA 定位质量 → 和 BESTPOS 同一套解类型名，下游（融合、状态）不用分来源。GGA 的 4（RTK 固定）按 NARROW_INT：
# 融合只认它为固定解；5（RTK 浮点）按 NARROW_FLOAT。6（推算）等不是卫星解，按 NONE
GGA_SOL = {'1': 'SINGLE', '2': 'PSRDIFF', '4': 'NARROW_INT', '5': 'NARROW_FLOAT'}
# 单天线：INS.NAVI 的这一帧按最近一条 GGA 的质量标（GGA 1 Hz）；GGA 比它早这么多秒以上就不知道质量，按 NONE
INS_QUALITY_MAX_AGE_S = 1.5
# INS.NAVI 不带位置精度：取同一秒或稍早（INS_QUALITY_MAX_AGE_S 内）那条 $GNGST 报的纬度、经度 1σ（2026-10-07 起接收机
# 开了 GST，固定时水平约 0.7 cm；高程精度用户说不用，不取，sigma_u 照旧按水平的 2 倍填）。没有这样的 GST 时融合要一个
# 观测 σ：固定解按 UM982 BESTPOS 固定时报的量级（约 1 cm），浮点 0.3 m。这是假定值，状态里不当成接收机报的精度显示（sigma_h 报 None）
INS_SIGMA_M = {'NARROW_INT': 0.01, 'NARROW_FLOAT': 0.3}
INS_SIGMA_OTHER_M = 3.0


# 用户规则（2026-10-07）：单天线 PDOP 小于 3 才可信，3 及以上一律不用（先定 2，同日改 3）。PDOP 取 $GNGSA
# （1 Hz，几条 GSA 是同一个组合解的 DOP），收到后 PDOP_FRESH_S 内有效；没有 PDOP（接收机没出 GSA、断了）也算不可信
INS_MAX_PDOP = 3.0
PDOP_FRESH_S = 2.0


def gsa_pdop(line):
    """$GxGSA 的 PDOP（第 15 个字段，12 个卫星号之后）；不是 GSA、校验不过、没有数返回 None。"""
    line = line.strip()
    if not line.startswith('$G') or line[3:7] != 'GSA,' or '*' not in line:
        return None
    body, crc = line[1:].rsplit('*', 1)
    try:
        if nmea_checksum(body) != int(crc[:2], 16):
            return None
        pdop = float(body.split(',')[15])
    except (ValueError, IndexError):
        return None
    return pdop if math.isfinite(pdop) and pdop > 0 else None


def gst_sigma(line):
    """$GxGST 的水平定位精度（接收机估的 1σ，米）：字段 1 UTC 时分秒、6 纬度、7 经度（2 伪距残差 RMS、3–5 误差椭圆、
    8 高程不用）。不是 GST、校验不过、纬经度精度不是正数返回 None。返回 dict(tod, sigma_n, sigma_e)。"""
    line = line.strip()
    if not line.startswith('$G') or line[3:7] != 'GST,' or '*' not in line:
        return None
    body, crc = line[1:].rsplit('*', 1)
    try:
        if nmea_checksum(body) != int(crc[:2], 16):
            return None
        f = body.split(',')
        tod, sn, se = _hms_seconds(f[1]), float(f[6]), float(f[7])
    except (ValueError, IndexError):
        return None
    if not all(math.isfinite(v) and v > 0 for v in (sn, se)):
        return None
    return dict(tod=tod, sigma_n=sn, sigma_e=se)


def _same_epoch(tod, ref_tod):
    """质量语句（GGA、GST，1 Hz）的时刻 ref_tod 是不是 INS.NAVI 这一帧（tod）的同一秒或稍早 INS_QUALITY_MAX_AGE_S 内（跨 UTC 零点也算）。"""
    d = (tod - ref_tod) % 86400.0
    d = d - 86400.0 if d > 43200 else d
    return -0.05 <= d <= INS_QUALITY_MAX_AGE_S


def gga_quality(fields):
    """parse_gga 拆好的字段 → dict(tod 当天 UTC 秒, sol, sats, diff_age)；时间坏了返回 None。GGA 的经纬度不用。"""
    try:
        tod = _hms_seconds(fields[1])
        sats = int(fields[7] or 0)
        diff_age = float(fields[13]) if len(fields) > 13 and fields[13] else None
    except (ValueError, IndexError):
        return None
    return dict(tod=tod, sol=GGA_SOL.get(fields[6], 'NONE'), sats=sats, diff_age=diff_age)


class GnssClock:
    """GNSS 解算时刻 → 本机单调时钟。

    串口传输只会让数据晚到、不会早到，所以窗口内 (收到时刻 - GPS 时间) 的最小值最接近真实钟差；
    再减去接收机解算输出本身的延迟。窗口保持几秒，跟得上两个时钟之间的 ppm 级漂移。

    晚到超过 max_jump_s 的（平板中转卡了几秒、积压的数据一下子到）按原钟差算，如实标成旧数据，也不进窗口：
    2026-10-01 实车中转断流 3.85 s，旧做法把它当成接收机重启、钟差重估成「刚测到」，融合按 RTK 对齐到
    3.85 s 前的位置（往回跳 2.5 m），车按错的位置猛打方向跑偏。比钟差早到 max_jump_s 以上（时钟跳了）、
    或晚到 max_late_s 以上（接收机重启、GPS 时间变了）才丢掉旧窗口重新估计。
    """

    def __init__(self, window_s=10.0, solution_latency_s=0.02, max_jump_s=1.0, max_late_s=60.0, max_drift=2e-4):
        self.window_s, self.latency, self.max_jump = window_s, solution_latency_s, max_jump_s
        self.max_late, self.max_drift = max_late_s, max_drift
        self.samples = deque()
        self.offset = self.offset_t = None

    def measurement_time(self, week, tow, t_rx):
        gps = week * GPS_WEEK_S + tow
        diff = t_rx - gps
        if self.offset is not None and (diff < self.offset - self.max_jump or diff > self.offset + self.max_late):
            # 时钟跳变或接收机重启：丢掉旧窗口重新估计
            self.samples.clear()
            self.offset = None
        elif self.offset is not None and diff > self.offset + self.max_jump:
            # 迟到的数据：测量时刻照旧按原钟差算（融合按时刻丢掉太旧的、回放还在历史里的）
            return gps + self.offset - self.latency
        while self.samples and self.samples[0][0] < t_rx - self.window_s:
            self.samples.popleft()
        # 单调队列维护窗口最小值
        while self.samples and self.samples[-1][1] >= diff:
            self.samples.pop()
        self.samples.append((t_rx, diff))
        low = self.samples[0][1]
        if self.offset is None or low <= self.offset:
            self.offset = low
        else:
            # 窗口里早到的样本过期、最小值变大：只按时钟漂移的速度往上跟（max_drift 秒/秒 = 200 ppm，是晶振实际漂移的几倍），
            # 断流比窗口还长时，积压数据里最先进窗口的那条（晚到近 max_jump_s）不会把钟差抬高
            self.offset = min(low, self.offset + self.max_drift * max(0.0, t_rx - self.offset_t))
        self.offset_t = t_rx
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

    单天线流动站（平板转来的 HCYTJ 接收机，2026-10-07）没有 Unicore 日志：收到通过校验的 $INS.NAVI 就把
    antenna_mode 标成 single，它的 5 Hz 组合导航解当定位（fix，source='INS.NAVI'）。GGA 不当定位（1 Hz），
    只拿它的定位质量给 INS.NAVI 标固定 / 浮点，和上报 CORS；PDOP（GSA）不小于 3 的帧标 pdop_ok False（用户规则）；
    位置精度取 GST（没有 GST 用假定值，标 sigma_assumed）。
    收到主天线 BESTPOS 标回 dual。
    fix 消息都带 antenna_mode，走直线节点按它换天线杠杆臂。
    """

    RATE_WINDOW_S = 2.0

    def __init__(self, clock=None):
        self.clock = clock or GnssClock()
        self.secondary_clock = GnssClock()
        self.ins_clock = GnssClock()    # INS.NAVI 只有当天 UTC 时刻，和 GPS 周内秒不是一个钟，单独估钟差
        self._ins_day = 0               # INS.NAVI 跨过 UTC 零点的次数（时刻往回跳半天以上算过了零点）
        self._ins_tod = None
        # dual：UM982 双天线（包括缺副天线日志的时候）。收到通过校验的 $INS.NAVI 才改成 single
        self.antenna_mode = 'dual'
        self.buf = b''
        self.gga = None                 # 最新一条通过校验的 GGA 字段
        self.pdop = self.pdop_rx = None  # 最新的 PDOP（GSA）和收到时刻
        self.gst = None                 # 最新一条通过校验的 GST 精度（gst_sigma）
        self.last = {}                  # kind -> 最新消息
        self.recent_fix = deque(maxlen=40)  # 最近 2 s 的主天线定位，按测量时刻和副天线配对
        self.baseline_pairs = deque()       # (测量时刻, 两天线固定解的实时距离)
        self.position_baseline = None
        self._baseline_bad_since = None
        self._baseline_epoch = None
        self.times = {k: deque() for k in ('fix', 'fix2', 'heading', 'vel')}
        self.crc_errors = 0
        self.last_rx = None

    def feed(self, data, t_rx):
        """ASCII 行（以换行结尾）和二进制帧（同步字开头、按长度取）混在一起也行。二进制帧到齐前停下等后续数据；
        帧不全（经 CAN 丢了一块）CRC 对不上就跳过这个同步字往后找，不会拼错。同步字之前没有换行的残片直接丢掉。"""
        buf = self.buf + data
        out = []
        i = 0
        while i < len(buf):
            s = _next_sync(buf, i)
            n = buf.find(b'\n', i)
            if n >= 0 and (s < 0 or n < s):
                self._line(buf[i:n], t_rx, out)
                i = n + 1
                continue
            if s < 0:
                break                   # 半行：等换行
            kind, end = binary_frame(buf, s)
            if kind == 'need':
                i = s
                break
            if kind != 'frame':
                if kind == 'crc':
                    self.crc_errors += 1
                i = s + 1
                continue
            m = parse_binary(buf[s:end])
            if m is not None:
                self._emit(m, t_rx, out)
            i = end
        self.buf = buf[i:]
        if len(self.buf) > 4096:        # 没有换行、也凑不成帧的垃圾数据，别让缓冲无限长
            self.buf = b''
        return out

    def _line(self, raw, t_rx, out):
        line = raw.decode('ascii', 'replace').strip()
        if line.startswith(('$GNGGA', '$GPGGA')):
            fields = parse_gga(line)
            if fields:
                self.gga = fields
            return
        if line.startswith('$G') and line[3:7] == 'GSA,':
            pdop = gsa_pdop(line)
            if pdop is not None:
                self.pdop, self.pdop_rx = pdop, t_rx
            return
        if line.startswith('$G') and line[3:7] == 'GST,':
            gst = gst_sigma(line)
            if gst is not None:
                self.gst = gst
            return
        if line.startswith('$INS.NAVI'):
            try:
                p = parse_ins_navi(line)
            except ParseError:
                self.crc_errors += 1
                return
            if p is not None:
                self.antenna_mode = 'single'
                self._publish(self._ins_message(p, t_rx), t_rx, out)
            return
        try:
            m = parse_unicore(line)
        except ParseError:
            self.crc_errors += 1
            return
        if m is not None:
            self._emit(m, t_rx, out)

    def _ins_message(self, p, t_rx):
        """单天线的 INS.NAVI → 和 BESTPOS 同样字段的 fix 消息。解类型、卫星数、差分龄期取同一秒或稍早那条 GGA
        （INS_QUALITY_MAX_AGE_S 以内）；没有这样的 GGA 就不知道是不是固定解，按 NONE（融合不用）。位置精度取同样时间窗的
        GST；没有就按解类型给假定值（sigma_assumed，状态里不报）。"""
        if self._ins_tod is not None and p['tod'] < self._ins_tod - 43200:
            self._ins_day += 1
        elif self._ins_tod is not None and p['tod'] > self._ins_tod + 43200:
            self._ins_day -= 1
        self._ins_tod = p['tod']
        t = self.ins_clock.measurement_time(0, self._ins_day * 86400.0 + p['tod'], t_rx)
        q = gga_quality(self.gga) if self.gga else None
        q = q if q is not None and _same_epoch(p['tod'], q['tod']) else None
        sol = q['sol'] if q else 'NONE'
        g = self.gst if self.gst is not None and _same_epoch(p['tod'], self.gst['tod']) else None
        sn, se = (g['sigma_n'], g['sigma_e']) if g is not None else (INS_SIGMA_M.get(sol, INS_SIGMA_OTHER_M),) * 2
        pdop = self._fresh_pdop(t_rx)
        return dict(kind='fix', t=t, t_rx=t_rx, antenna=1, source='INS.NAVI', sigma_assumed=g is None,
                    # PDOP 不小于 INS_MAX_PDOP（或没有）：sol 照实报，pdop_ok 为 False，走直线节点不交给融合
                    pdop=pdop, pdop_ok=pdop is not None and pdop < INS_MAX_PDOP,
                    lat=p['lat'], lon=p['lon'], height=p['height'], sol=sol,
                    status=SOL_OK if sol != 'NONE' else 'INSUFFICIENT_OBS',
                    sigma_n=sn, sigma_e=se, sigma_u=2 * max(sn, se),
                    diff_age=q['diff_age'] if q else None, sats=q['sats'] if q else 0, sats_tracked=None)

    def _fresh_pdop(self, now):
        return self.pdop if self.pdop_rx is not None and 0 <= now - self.pdop_rx <= PDOP_FRESH_S else None

    def _emit(self, m, t_rx, out):
        msg = self._message(m, t_rx)
        if msg['kind'] == 'fix':
            self.antenna_mode = 'dual'      # 主天线 BESTPOS 只有双天线 UM982 出；流动站那路没有这句
        self._publish(msg, t_rx, out)

    def _publish(self, msg, t_rx, out):
        if msg['kind'] == 'fix':
            msg['antenna_mode'] = self.antenna_mode
        self.last[msg['kind']] = msg
        if msg['kind'] == 'fix':
            self.recent_fix.append(msg)
        if msg['kind'] in ('fix','fix2'):
            d = self._antenna_offset()
            if d is not None:
                pair_t = self.last['fix2']['t']
                if not self.baseline_pairs or pair_t != self.baseline_pairs[-1][0]:
                    self.baseline_pairs.append((pair_t, d))
            self._update_position_baseline(msg['t_rx'])
            if self.position_baseline is not None:
                msg['position_baseline'] = dict(self.position_baseline)
                if msg['kind']=='fix':
                    msg['position_variance_scale'] = self.position_baseline['variance_scale']
        elif msg['kind'] == 'heading':
            # 下游（rtk_node 发布、走线融合）都按这里的 valid 取舍
            ref = self._baseline_ref(msg['t'])
            msg['baseline_ref'] = None if ref is None else round(ref, 3)
            msg['valid'] = heading_usable(msg['sol'], msg['length'])
        q = self.times[msg['kind']]
        q.append(t_rx)
        while q and q[0] < t_rx - self.RATE_WINDOW_S:
            q.popleft()
        self.last_rx = t_rx
        out.append(msg)

    def _message(self, m, t_rx):
        secondary = isinstance(m, BestPos) and m.antenna == 2
        clock = self.secondary_clock if secondary else self.clock
        t = clock.measurement_time(m.week, m.tow, t_rx)
        ok = m.sol_status == SOL_OK
        if isinstance(m, BestPos):
            return dict(kind='fix2' if secondary else 'fix', t=t, t_rx=t_rx,
                        gps_week=m.week, gps_tow_s=m.tow,
                        antenna=m.antenna, source='BESTNAVH' if secondary else 'BESTPOS',
                        lat=m.lat_deg, lon=m.lon_deg, height=m.height_m,
                        sol=m.pos_type if ok else 'NONE', status=m.sol_status,
                        sigma_n=m.sigma_lat, sigma_e=m.sigma_lon, sigma_u=m.sigma_hgt,
                        diff_age=_finite(m.diff_age_s), sats=m.sats_used, sats_tracked=m.sats_tracked)
        if isinstance(m, Heading):
            return dict(kind='heading', t=t, t_rx=t_rx, heading=m.heading_deg, pitch=m.pitch_deg,
                        gps_week=m.week, gps_tow_s=m.tow,
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
        antennas = dict(ant1=antenna('fix', fix.get('source', 'BESTPOS') if fix else 'BESTPOS'),
                        ant2=antenna('fix2', 'BESTNAVH'))
        fix = fix if antennas['ant1']['available'] else None
        hd = hd if hd and now - hd['t_rx'] < 1.0 else None
        # 副天线离主天线的实时距离照实报（两根都固定时），只作显示，不做一致性判断
        antennas['ant2'].update(offset_m=None, consistent=None)
        if antennas['ant1']['available'] and antennas['ant2']['available']:
            offset = self._antenna_offset()
            if offset is not None:
                antennas['ant2']['offset_m'] = round(offset, 3)
        rates = {k: round(len(q) / self.RATE_WINDOW_S, 1) if q and now - q[-1] < self.RATE_WINDOW_S else 0.0
                 for k, q in self.times.items()}
        return dict(
            receiver_ok=self.last_rx is not None and now - self.last_rx < 1.0,
            antenna_mode=self.antenna_mode,
            pdop=self._fresh_pdop(now),
            antennas=antennas,
            rates=rates, crc_errors=self.crc_errors,
            sol=fix['sol'] if fix else None,
            rtk=('FIXED' if fix and fix['sol'] in RTK_FIXED else
                 'FLOAT' if fix and fix['sol'] in RTK_FLOAT else
                 'SINGLE' if fix and fix['sol'] in ('SINGLE', 'PSRDIFF') else 'NONE'),
            sats=fix['sats'] if fix else 0,
            diff_age=fix['diff_age'] if fix else None,
            # 单天线 INS.NAVI 不带精度：精度取 GST，没有 GST 时融合用的是假定值，不当成接收机报的精度显示
            sigma_h=round(max(fix['sigma_n'], fix['sigma_e']), 4) if fix and not fix.get('sigma_assumed') else None,
            heading_sol=hd['sol'] if hd else None,
            heading_ok=bool(hd and hd.get('valid')),
            position_baseline=None if self.position_baseline is None or now-self.position_baseline['t_rx']>.4 else dict(self.position_baseline),
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

    def _paired_positions(self):
        """有原始 GPS 历元时严格配同一历元；旧记录只保留估计时刻配对用于显示。"""
        b = self.last.get('fix2')
        if b is None:return None
        aligned = 'gps_week' in b and 'gps_tow_s' in b
        def delta(a):
            if aligned:
                if 'gps_week' not in a or 'gps_tow_s' not in a:return math.inf
                return abs((a['gps_week']-b['gps_week'])*GPS_WEEK_S+a['gps_tow_s']-b['gps_tow_s'])
            return abs(a['t']-b['t'])
        a = min(self.recent_fix, key=delta, default=None)
        # 两边都是 RTK 固定解才比：单点、浮点本身就有分米到米级误差，比出来的距离说明不了副天线对不对
        if a is None or delta(a) > (.001 if aligned else .03) or not {a['sol'], b['sol']} <= RTK_FIXED:
            return None
        return a,b,aligned

    @staticmethod
    def _baseline_vector(a,b):
        phi = math.radians(a['lat'])
        w = 1.0 - _WGS84_E2 * math.sin(phi) ** 2
        dn = math.radians(b['lat'] - a['lat']) * _WGS84_A * (1.0 - _WGS84_E2) / w ** 1.5
        de = math.radians(b['lon'] - a['lon']) * _WGS84_A / math.sqrt(w) * math.cos(phi)
        return dn,de

    def _antenna_offset(self):
        pair=self._paired_positions()
        return None if pair is None else math.hypot(*self._baseline_vector(*pair[:2]))

    def _update_position_baseline(self, now):
        pair=self._paired_positions()
        if pair is None:
            # 同一历元另一帧还在传输中，不把短暂等待当成恢复或持续性中断。
            if self.position_baseline is not None and now-self.position_baseline['t_rx']<=.4:
                return
            self.position_baseline=None;self._baseline_bad_since=self._baseline_epoch=None
            return
        if not pair[2] or now-pair[1]['t_rx']>.4:
            self.position_baseline=None;self._baseline_bad_since=self._baseline_epoch=None
            return
        a,b,_=pair;epoch=(b['gps_week'],b['gps_tow_s'])
        if epoch==self._baseline_epoch:return
        previous=self.position_baseline
        self._baseline_epoch=epoch
        dn,de=self._baseline_vector(a,b);length=math.hypot(dn,de)
        if not all(math.isfinite(v) for v in (dn,de,a['sigma_n'],a['sigma_e'],b['sigma_n'],b['sigma_e'])):
            self.position_baseline=None;self._baseline_bad_since=None
            return
        sigma=math.sqrt((dn*dn*(a['sigma_n']**2+b['sigma_n']**2)+de*de*(a['sigma_e']**2+b['sigma_e']**2))/max(length*length,1e-12))
        hd=self.last.get('heading') or {}
        pitch=math.radians(hd.get('pitch',0.)) if hd.get('valid') else 0.
        reference=POSITION_BASELINE_NOMINAL_M*math.cos(pitch)
        error=abs(length-reference);threshold=max(.02,2.*sigma)
        if error>threshold:
            if self._baseline_bad_since is None or previous is None or now-previous['t_rx']>.4:
                self._baseline_bad_since=now
        else:self._baseline_bad_since=None
        suspect=self._baseline_bad_since is not None and now-self._baseline_bad_since>=POSITION_BASELINE_SETTLE_S-1e-9
        # 只增加位置观测方差，IMU/轮速预测权重相应提高；不改变解类型，不引入停车/禁喷。
        self.position_baseline=dict(t_rx=now,gps_week=b['gps_week'],gps_tow_s=b['gps_tow_s'],
            epoch_aligned=True,nominal_m=POSITION_BASELINE_NOMINAL_M,horizontal_reference_m=reference,
            instant_m=length,sigma_m=sigma,threshold_m=threshold,error_m=error,suspect=suspect,
            variance_scale=min(25.,max(1.,(error/threshold)**2)) if suspect else 1.)


def heading_usable(sol, length_m):
    """测向是否可用于定车头：整周固定，且基线长度在 HEADING_BASELINE_MIN_M–MAX_M 之内。"""
    return bool(sol in HEADING_FIXED and length_m is not None and math.isfinite(length_m)
                and HEADING_BASELINE_MIN_M <= length_m <= HEADING_BASELINE_MAX_M)


def heading_grade(sol):
    """测向解分级：整周固定（含 L1/宽巷）为 FIXED，浮点为 FLOAT，其余为 NONE。"""
    return 'FIXED' if sol in HEADING_FIXED else 'FLOAT' if sol and 'FLOAT' in sol else 'NONE'


def fix_grade(sol):
    return ('FIXED' if sol in RTK_FIXED else 'FLOAT' if sol in RTK_FLOAT else
            'SINGLE' if sol in ('SINGLE', 'PSRDIFF') else 'NONE')
