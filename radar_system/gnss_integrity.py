#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RTK 完好性监测：载波相位历元差分（TDCP）查接收机 RTK 有没有跳（11-27 用户「这些先加上」，user_rules.RTK_INTEGRITY_*）。

c30c07fd 22:18:28 那条：喷线 14.7 m 处主天线 RTK 突然往左跳 10 cm、副天线跟着跳，IMU 车头没动、车没被带偏；
原始观测离线做 TDCP，载波相位算出的天线轨迹在那里是平的，跳的是实时 RTK。原来只能按 σ、卫星数猜，这里直接查。

做法：每来一个 OBSVMB 历元 k，拿 window_s 前的历元 k0 当参考，两拍都有 RTK 固定解：
    λ(L_k − L_k0) = |s(k) − r_k| − |s(k0) − r_k0| + cΔdtr − cΔdts + Δ对流层
在两拍的 RTK 位置 r̂ 处线性化，解出 δ（3）+ 钟差变化（1）：δ = (r_k − r̂_k) − (r_k0 − r̂_k0)，就是这 window_s 里
RTK 自己的误差变了多少（卫星位置几米的误差、参考历元 RTK 的绝对误差都消掉）。水平 |δ| > jump_m：RTK 跳了，
参考历元（跳之前最后好的那拍）冻成锚点，之后都对它比；|δ| 回到 recover_m 以内连续 recover_s 才算回来；
跳了以后查不了（星少了）照旧当跳了；锚点超过 hold_max_s、或开始转（换线）才回到「不确定」。
没跳过时 TDCP 解不出（星少、残差大、在转）就是「不确定」，不拦 RTK（照原来用）。

用的观测：GPS L1C/A、Galileo E1C、北斗 B1I（GEO 不用），高度角 ≥ elev_mask_deg、载噪比 ≥ cn0_min；星历用接收机自己发的
GPSEPHB / BDSEPHB / GALEPHB（有变化才发，开机后要等一会儿才齐）。周跳：锁定时间没跟上、或相位变化和多普勒积分对不上。
坏星（星历不对、多路径大）按残差一颗颗剔。只用标准库。离线回放见 tools/rtk_integrity_replay.py。
"""
import math
import struct
import zlib
from collections import deque

import user_rules as rules

C = 299792458.0
OMGE = 7.2921151467e-5
SIGNAL = {'G': (0, 1575.42e6), 'E': (2, 1575.42e6), 'C': (0, 1561.098e6)}   # 系统 → (信号类型, 频率)
GM = {'G': (3.986005e14, 7.2921151467e-5), 'E': (3.986004418e14, 7.2921151467e-5),
      'C': (3.986004418e14, 7.292115e-5)}
SYSTEM = {0: 'G', 1: 'R', 2: 'S', 3: 'E', 4: 'C', 5: 'J', 6: 'I'}
ID_OBSVM, ID_GPSEPH, ID_BDSEPH, ID_GALEPH = 12, 106, 108, 109
EPH_MAX_AGE_S = {'G': 4 * 3600., 'E': 4 * 3600., 'C': 2 * 3600.}


# ───────────── Unicore 二进制 ─────────────

def _crc_ok(frame):
    """Unicore 的 CRC32：初值 0、不取反（RTKLIB rtk_crc32）。"""
    return (zlib.crc32(frame[:-4], 0xFFFFFFFF) ^ 0xFFFFFFFF) == struct.unpack_from('<I', frame, len(frame) - 4)[0]


class FrameSplitter:
    """字节流 → Unicore 二进制帧 (消息号, 周, 周内秒, 内容)。跨包的半帧留到下次。"""

    def __init__(self, max_buffer=262144):
        self.buf = bytearray()
        self.max_buffer = max_buffer

    def feed(self, data):
        self.buf += data
        out, i, n = [], 0, len(self.buf)
        while True:
            i = self.buf.find(b'\xaa\x44\xb5', i)
            if i < 0:
                i = max(0, n - 2)
                break
            if i + 24 > n:
                break
            mid, mlen = struct.unpack_from('<HH', self.buf, i + 4)
            end = i + 24 + mlen + 4
            if end > n:
                if mlen > 65000:
                    i += 3
                    continue
                break
            frame = bytes(self.buf[i:end])
            if _crc_ok(frame):
                week = struct.unpack_from('<H', frame, 10)[0]
                tow = struct.unpack_from('<I', frame, 12)[0] * 1e-3
                out.append((mid, week, tow, frame[24:24 + mlen]))
                i = end
            else:
                i += 3
        del self.buf[:i]
        if len(self.buf) > self.max_buffer:
            del self.buf[:len(self.buf) - self.max_buffer]
        return out


def decode_obs(body):
    """OBSVMB → {(系统, prn): (L 周, 伪距 m, 锁定时间 s, 载噪比, 多普勒 Hz, 频率)}：只留 SIGNAL 里的信号、相位和码都锁着的。"""
    out = {}
    if len(body) < 4:
        return out
    nobs = struct.unpack_from('<I', body, 0)[0]
    for k in range(nobs):
        p = 4 + 40 * k
        if p + 40 > len(body):
            break
        st = struct.unpack_from('<I', body, p + 36)[0]
        sysc = SYSTEM.get((st >> 16) & 7)
        sig = (st >> 21) & 0x1F
        if sysc not in SIGNAL or sig != SIGNAL[sysc][0] or not (st >> 10) & 1 or not (st >> 12) & 1:
            continue
        prn = struct.unpack_from('<H', body, p + 2)[0]
        if sysc == 'C' and (prn <= 5 or prn >= 59):          # 北斗 GEO：轨道算法不一样，不用
            continue
        if sysc == 'G' and prn > 32:
            continue
        psr, adr = struct.unpack_from('<dd', body, p + 4)
        if psr <= 0 or adr == 0:
            continue
        dop = struct.unpack_from('<f', body, p + 24)[0]
        cn0 = struct.unpack_from('<H', body, p + 28)[0] / 100.
        lockt = struct.unpack_from('<f', body, p + 32)[0]
        out[(sysc, prn)] = (-adr, psr, lockt, cn0, dop, SIGNAL[sysc][1])
    return out


def _unpack(body, fields, p=0):
    vals = {}
    for name, fmt in fields:
        vals[name] = struct.unpack_from('<' + fmt, body, p)[0]
        p += struct.calcsize(fmt)
    return vals


_KEPLER = [('toes', 'd'), ('A', 'd'), ('deln', 'd'), ('M0', 'd'), ('e', 'd'), ('omg', 'd'), ('cuc', 'd'), ('cus', 'd'),
           ('crc', 'd'), ('crs', 'd'), ('cic', 'd'), ('cis', 'd'), ('i0', 'd'), ('idot', 'd'), ('OMG0', 'd'), ('OMGd', 'd')]


def decode_eph(mid, body):
    """GPSEPHB / BDSEPHB / GALEPHB → ((系统, prn), 星历) 或 None。toes / toc 是各系统自己的周内秒。"""
    try:
        if mid in (ID_GPSEPH, ID_BDSEPH):
            if len(body) < 224:
                return None
            head = _unpack(body, [('prn', 'I'), ('tow', 'd'), ('health', 'I'), ('iode', 'I'), ('iode2', 'I'),
                                  ('week', 'I'), ('zweek', 'I')])
            e = _unpack(body, _KEPLER, 32)                  # 头 4+8+4+4+4+4+4 = 32 字节
            tail = _unpack(body, [('iodc', 'I'), ('toc', 'd')], 32 + 16 * 8)
            p = 32 + 16 * 8 + 12
            if mid == ID_GPSEPH:
                e.update(f0=struct.unpack_from('<d', body, p + 8)[0], f1=struct.unpack_from('<d', body, p + 16)[0],
                         f2=struct.unpack_from('<d', body, p + 24)[0])
                key = ('G', head['prn'])
            else:
                e.update(f0=struct.unpack_from('<d', body, p + 16)[0], f1=struct.unpack_from('<d', body, p + 24)[0],
                         f2=struct.unpack_from('<d', body, p + 32)[0])
                key = ('C', head['prn'])
            e.update(toc=tail['toc'], health=head['health'])
            return key, e
        if mid == ID_GALEPH:
            if len(body) < 220:
                return None
            prn, fnav = struct.unpack_from('<II', body, 0)
            names = ['sqrtA', 'deln', 'M0', 'e', 'omg', 'cuc', 'cus', 'crc', 'crs', 'cic', 'cis', 'i0', 'idot', 'OMG0', 'OMGd']
            e = dict(zip(names, struct.unpack_from('<15d', body, 28)))
            e['toes'] = float(struct.unpack_from('<I', body, 24)[0])
            p = 28 + 120
            toc_f, af_f = struct.unpack_from('<I', body, p)[0], struct.unpack_from('<3d', body, p + 4)
            toc_i, af_i = struct.unpack_from('<I', body, p + 28)[0], struct.unpack_from('<3d', body, p + 32)
            use_f = bool(fnav & 1)
            af = af_f if use_f else af_i
            e.update(A=e.pop('sqrtA') ** 2, toc=float(toc_f if use_f else toc_i), f0=af[0], f1=af[1], f2=af[2], health=0)
            return ('E', prn), e
    except struct.error:
        return None
    return None


# ───────────── 广播星历 ─────────────

def _wrap_week(t):
    return t - 604800. if t > 302400. else t + 604800. if t < -302400. else t


def sat_pos_clk(sysc, e, t):
    """卫星地固系位置（m）和钟差（s，含相对论项）。t：该系统时间的周内秒（北斗 BDT）。"""
    mu, omge = GM[sysc]
    A = e['A']
    tk = _wrap_week(t - e['toes'])
    M = e['M0'] + (math.sqrt(mu / A ** 3) + e['deln']) * tk
    E = M
    for _ in range(30):
        E1 = M + e['e'] * math.sin(E)
        if abs(E1 - E) < 1e-13:
            E = E1
            break
        E = E1
    sinE, cosE = math.sin(E), math.cos(E)
    phi = math.atan2(math.sqrt(1 - e['e'] ** 2) * sinE, cosE - e['e']) + e['omg']
    s2, c2 = math.sin(2 * phi), math.cos(2 * phi)
    u = phi + e['cus'] * s2 + e['cuc'] * c2
    r = A * (1 - e['e'] * cosE) + e['crs'] * s2 + e['crc'] * c2
    i = e['i0'] + e['idot'] * tk + e['cis'] * s2 + e['cic'] * c2
    x, y = r * math.cos(u), r * math.sin(u)
    O = e['OMG0'] + (e['OMGd'] - omge) * tk - omge * e['toes']
    cO, sO, ci = math.cos(O), math.sin(O), math.cos(i)
    tc = _wrap_week(t - e['toc'])
    dts = e['f0'] + e['f1'] * tc + e['f2'] * tc * tc - 2 * math.sqrt(mu * A) * e['e'] * sinE / C ** 2
    return (x * cO - y * ci * sO, x * sO + y * ci * cO, y * math.sin(i)), dts


def sat_at(sysc, e, tow_gps, psr, r_rx):
    """接收时刻（GPST 周内秒）、伪距：发射时刻的卫星位置转到接收时刻地固系（Sagnac），和卫星钟差。"""
    t = tow_gps - (14.0 if sysc == 'C' else 0.0)
    tx = t - psr / C
    for _ in range(2):
        _, dts = sat_pos_clk(sysc, e, tx)
        tx = t - psr / C - dts
    (X, Y, Z), dts = sat_pos_clk(sysc, e, tx)
    a = OMGE * math.dist((X, Y, Z), r_rx) / C
    return (X * math.cos(a) + Y * math.sin(a), -X * math.sin(a) + Y * math.cos(a), Z), dts


def tropo(el):
    """对流层延迟（m），只用它在两拍之间的变化：天顶 2.4 m，低高度角按 Niell 近似。"""
    s = math.sin(el)
    return 2.4 / (s + 0.00143 / (math.tan(el) + 0.0445))


def geodetic_to_ecef(lat, lon, h):
    a, f = 6378137.0, 1 / 298.257223563
    e2 = f * (2 - f)
    p, l = math.radians(lat), math.radians(lon)
    N = a / math.sqrt(1 - e2 * math.sin(p) ** 2)
    return ((N + h) * math.cos(p) * math.cos(l), (N + h) * math.cos(p) * math.sin(l), (N * (1 - e2) + h) * math.sin(p))


def enu_basis(lat, lon):
    p, l = math.radians(lat), math.radians(lon)
    return ((-math.sin(l), math.cos(l), 0.), (-math.sin(p) * math.cos(l), -math.sin(p) * math.sin(l), math.cos(p)),
            (math.cos(p) * math.cos(l), math.cos(p) * math.sin(l), math.sin(p)))


def _solve4(rows):
    """加权最小二乘 y = h·x（x 四维）：rows [(h(4), y, w)] → (x, 残差列表, 水平精度因子) 或 None（几何太差）。"""
    N = [[0.0] * 4 for _ in range(4)]
    b = [0.0] * 4
    for h, y, w in rows:
        for i in range(4):
            b[i] += w * h[i] * y
            for j in range(4):
                N[i][j] += w * h[i] * h[j]
    # 高斯消元求逆（4×4）
    M = [N[i][:] + [1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
    for c in range(4):
        piv = max(range(c, 4), key=lambda r: abs(M[r][c]))
        if abs(M[piv][c]) < 1e-12:
            return None
        M[c], M[piv] = M[piv], M[c]
        f = M[c][c]
        M[c] = [v / f for v in M[c]]
        for r in range(4):
            if r != c and M[r][c] != 0.0:
                g = M[r][c]
                M[r] = [a - g * bb for a, bb in zip(M[r], M[c])]
    Q = [row[4:] for row in M]
    x = [sum(Q[i][j] * b[j] for j in range(4)) for i in range(4)]
    res = [y - sum(hh * xx for hh, xx in zip(h, x)) for h, y, _ in rows]
    return x, res, Q


# ───────────── 监测 ─────────────

class RtkIntegrity:
    """RTK 完好性监测（见文件头）。线程不安全：rtk_node 在 stream_lock 里调。

    feed_bytes(data)：接收机原始字节（取 OBSVMB、星历）；add_fix(tow, lat, lon, h, fixed)：主天线 RTK（BESTPOS，
    同一台接收机的 GPS 周内秒）；add_heading(tow, heading_deg, valid)：双天线测向（只用来判断在不在转）。
    status()：state = ok / jump / unknown，delta_m 最近一次 RTK 与载波相位的水平差，等等。
    """

    def __init__(self, enabled=rules.RTK_INTEGRITY_ENABLED, window_s=rules.RTK_JUMP_WINDOW_S, jump_m=rules.RTK_JUMP_M,
                 persist=rules.RTK_JUMP_PERSIST, recover_m=rules.RTK_RECOVER_M, recover_s=rules.RTK_RECOVER_S, hold_max_s=rules.RTK_JUMP_HOLD_MAX_S,
                 min_sats=rules.TDCP_MIN_SATS, res_max_m=rules.TDCP_RES_MAX_M, elev_mask_deg=rules.TDCP_ELEV_MASK_DEG,
                 cn0_min=rules.TDCP_CN0_MIN, turn_deg_s=rules.TDCP_TURN_DEG_S, slip_cyc=0.5, hdop_max=3.0):
        if type(enabled) is not bool:
            raise ValueError('enabled 必须是 True 或 False')
        for name, v in (('window_s', window_s), ('jump_m', jump_m), ('recover_m', recover_m), ('recover_s', recover_s),
                        ('hold_max_s', hold_max_s), ('res_max_m', res_max_m), ('turn_deg_s', turn_deg_s),
                        ('slip_cyc', slip_cyc), ('hdop_max', hdop_max)):
            if not math.isfinite(v) or v <= 0:
                raise ValueError(name + ' 必须是有限正数')
        if recover_m >= jump_m:
            raise ValueError('recover_m 必须小于 jump_m')
        if not (isinstance(min_sats, int) and min_sats >= 5):
            raise ValueError('min_sats 至少 5')
        if not (isinstance(persist, int) and persist >= 1):
            raise ValueError('persist 至少 1')
        self.persist = persist
        self.over = 0                    # 连续几次超过 jump_m
        self.recent = deque()            # jump 里最近对锚点的 (tow, 水平差)
        self.enabled = enabled
        self.window_s, self.jump_m, self.recover_m, self.recover_s = window_s, jump_m, recover_m, recover_s
        self.hold_max_s, self.min_sats, self.res_max_m = hold_max_s, min_sats, res_max_m
        self.elev_mask, self.cn0_min = math.radians(elev_mask_deg), cn0_min
        self.turn_rate, self.slip_cyc, self.hdop_max = math.radians(turn_deg_s), slip_cyc, hdop_max
        self.split = FrameSplitter()
        self.eph = {}
        self.epochs = deque()            # (tow, obs)
        self.fixes = {}                  # 周内秒（取到 0.01 s）→ 主天线 ECEF（只存固定解）
        self.headings = deque()          # (tow, 航向 rad)
        self.turn_tow = -math.inf        # 最近一次在转的时刻
        self.pending = deque()           # 还没等到同一拍 RTK 的观测历元 tow
        self.state, self.anchor = 'unknown', None
        self.since = None                # jump 起始
        self.err = None                  # jump 里最近一次算出的 (tow, RTK 相对锚点的误差 (东, 北))：对不上锚点时接力
        self.delta_m = self.sats = self.rms_m = self.age_s = None
        self.reason = 'starting'
        self.last_tow = None
        self.jumps = 0
        self.checks = 0
        self.basis = None

    # ---- 输入 ----
    def feed_bytes(self, data):
        if not self.enabled or not data:
            return
        for mid, week, tow, body in self.split.feed(data):
            if mid == ID_OBSVM:
                obs = decode_obs(body)
                if obs:
                    self.epochs.append((tow, obs))
                    self.pending.append(tow)
                    while self.epochs and _wrap_week(tow - self.epochs[0][0]) > self.hold_max_s + self.window_s + 2.:
                        self.epochs.popleft()
            elif mid in (ID_GPSEPH, ID_BDSEPH, ID_GALEPH):
                got = decode_eph(mid, body)
                if got is not None and got[1].get('health', 0) == 0 and got[1]['A'] > 1e6:
                    self.eph[got[0]] = got[1]
        self._evaluate()

    def add_fix(self, tow, lat, lon, height, fixed):
        if not self.enabled or tow is None:
            return
        key = round(tow, 2)
        if fixed and all(math.isfinite(v) for v in (lat, lon, height)):
            self.fixes[key] = geodetic_to_ecef(lat, lon, height)
            if self.basis is None:
                self.basis = enu_basis(lat, lon)
        else:
            self.fixes.pop(key, None)
        if len(self.fixes) > 4000:
            for k in sorted(self.fixes)[:1000]:
                del self.fixes[k]
        self._evaluate()

    def add_heading(self, tow, heading_deg, valid):
        if not self.enabled or tow is None or not valid or heading_deg is None:
            return
        h = math.radians(heading_deg)
        # 转速按 0.5–1.5 s 前那帧算：测向每帧本身抖 0.3° 上下，按相邻两帧（0.1 s）算就有 3°/s
        for t0, h0 in self.headings:
            dt = _wrap_week(tow - t0)
            if 0.5 <= dt <= 1.5:
                if abs(math.atan2(math.sin(h - h0), math.cos(h - h0))) / dt > self.turn_rate:
                    self.turn_tow = tow
                break
        self.headings.append((tow, h))
        while len(self.headings) > 50:
            self.headings.popleft()

    # ---- 判 ----
    def _fix_at(self, tow):
        return self.fixes.get(round(tow, 2))

    def _epoch_at(self, tow):
        for t, obs in self.epochs:
            if abs(_wrap_week(t - tow)) < 1e-3:
                return t, obs
        return None

    def _evaluate(self):
        while self.pending:
            tow = self.pending[0]
            if self._fix_at(tow) is None:
                if self.last_tow is not None and _wrap_week(self.last_tow - tow) > 1.0 or len(self.pending) > 20:
                    self.pending.popleft()          # 这拍等不到 RTK 固定解了
                    continue
                if self.epochs and _wrap_week(self.epochs[-1][0] - tow) > 1.0:
                    self.pending.popleft()
                    continue
                return
            self.pending.popleft()
            self._check(tow)

    def _unknown(self, tow, reason):
        if self.state != 'unknown':
            self.anchor, self.since, self.err = None, None, None
            self.recent.clear()
        self.over = 0
        self.state, self.reason = 'unknown', reason
        self.last_tow = tow

    def _check(self, tow):
        self.last_tow = tow
        self.checks += 1
        if _wrap_week(tow - self.turn_tow) <= self.window_s + 1.0:
            self._unknown(tow, 'turning')
            return
        if self.state == 'jump' and self.anchor is not None:
            ref = self.anchor
            if _wrap_week(tow - ref) > self.hold_max_s:
                self._unknown(tow, 'hold_expired')
                return
        else:
            ref = None
            for t, _ in self.epochs:
                if _wrap_week(tow - t) >= self.window_s - 0.05 and self._fix_at(t) is not None:
                    ref = t                          # 最近的那个够 window_s 的
                if _wrap_week(tow - t) < self.window_s - 0.05:
                    break
            if ref is None:
                self._unknown(tow, 'no_reference')
                return
        got = self._tdcp(ref, tow)
        if got is None and self.state == 'jump' and self.err is not None and 0 < _wrap_week(tow - self.err[0]) <= 1.0:
            # 对不上锚点（这几秒里周跳的星多了、剩下不够）：从上一拍接力，误差 = 上一拍相对锚点的 + 这一步的
            step = self._tdcp(self.err[0], tow)
            if step is not None:
                (de, dn), n, rms = step
                got = ((de + self.err[1][0], dn + self.err[1][1]), n, rms)
        if got is None:
            if self.state != 'jump':
                self._unknown(tow, self.reason if self.reason.startswith('tdcp') else 'tdcp')
            else:
                # 跳了以后查不了（星少了）：不回头信刚跳过的 RTK，照旧当跳了，最多到锚点 hold_max_s（上面）或开始转
                self.reason = 'jump_unverified'
            return
        (de, dn), n, rms = got
        dh = math.hypot(de, dn)
        self.delta_m, self.sats, self.rms_m, self.age_s = dh, n, rms, _wrap_week(tow - ref)
        if self.state == 'jump':
            self.err = (tow, (de, dn))
            # 回来：最近 recover_s 对锚点的差平均在 recover_m 以内（单次判 RTK 本身 1–2 cm 的抖动常过不了）
            self.recent.append((tow, dh))
            while self.recent and _wrap_week(tow - self.recent[0][0]) > self.recover_s + 1e-6:
                self.recent.popleft()
            span = _wrap_week(tow - self.recent[0][0])
            if (span >= self.recover_s * 0.75 and len(self.recent) >= 3
                    and sum(d for _, d in self.recent) / len(self.recent) <= self.recover_m):
                self.state, self.anchor, self.since, self.err, self.reason = 'ok', None, None, None, 'recovered'
                self.recent.clear()
            return
        self.over = self.over + 1 if dh > self.jump_m else 0
        if self.over >= self.persist:
            # 锚点取第一次超的那拍的参考（跳之前最后好的那拍）
            self.state, self.anchor, self.since, self.reason = 'jump', self._first_ref, tow, 'jump'
            self.err = (tow, (de, dn)) if self._first_ref == ref else None
            self.jumps += 1
            self.over = 0
            self.recent.clear()
        else:
            if self.over == 1:
                self._first_ref = ref
            self.state, self.reason = 'ok', 'ok' if self.over == 0 else 'over'

    def _tdcp(self, t0, t1):
        """t0 → t1：((东, 北) 的 δ m, 用了几颗星, 残差 rms m) 或 None（并在 reason 里写原因）。"""
        e0, e1 = self._epoch_at(t0), self._epoch_at(t1)
        r0, r1 = self._fix_at(t0), self._fix_at(t1)
        if e0 is None or e1 is None or r0 is None or r1 is None or self.basis is None:
            self.reason = 'tdcp_missing'
            return None
        up = self.basis[2]
        rows, keys = [], []
        obs0 = e0[1]
        for sat, (L1, P1, lock1, cn1, D1, f) in e1[1].items():
            if sat not in obs0 or sat not in self.eph or cn1 < self.cn0_min:
                continue
            e = self.eph[sat]
            L0, P0, lock0, cn0, D0, _ = obs0[sat]
            dt = _wrap_week(t1 - t0)
            if lock1 + 0.05 < lock0 + dt:
                continue                                  # 锁定时间没跟上：周跳
            # 多普勒积分（梯形）和相位变化比：中间每拍都要有这颗星
            integ, prev_t, prev_d, ok = 0.0, t0, D0, True
            for t, obs in [(t, o) for t, o in self.epochs if 0 < _wrap_week(t - t0) <= dt + 1e-6]:
                v = obs.get(sat)
                if v is None:
                    ok = False
                    break
                integ += 0.5 * (v[4] + prev_d) * _wrap_week(t - prev_t)
                prev_t, prev_d = t, v[4]
            if not ok or abs((L1 - L0) + integ) > self.slip_cyc * max(1.0, dt / 0.2) ** 0.5:
                continue
            if abs(_wrap_week(t1 - (14.0 if sat[0] == 'C' else 0.0) - e['toes'])) > EPH_MAX_AGE_S[sat[0]]:
                continue
            s1, dts1 = sat_at(sat[0], e, t1, P1, r1)
            s0, dts0 = sat_at(sat[0], e, t0, P0, r0)
            rho1, rho0 = math.dist(s1, r1), math.dist(s0, r0)
            ev = [(a - b) / rho1 for a, b in zip(s1, r1)]
            el1 = math.asin(sum(a * b for a, b in zip(ev, up)))
            el0 = math.asin(sum((a - b) / rho0 * c for a, b, c in zip(s0, r0, up)))
            if el1 < self.elev_mask or el0 < self.elev_mask:
                continue
            y = (C / f) * (L1 - L0) - (rho1 - rho0) + C * (dts1 - dts0) - (tropo(el1) - tropo(el0))
            rows.append(([-ev[0], -ev[1], -ev[2], 1.0], y, math.sin(el1) ** 2))
            keys.append(sat)
        while len(rows) >= self.min_sats:
            got = _solve4(rows)
            if got is None:
                self.reason = 'tdcp_geometry'
                return None
            x, res, Q = got
            med = sorted(abs(r) for r in res)[len(res) // 2]
            worst = max(range(len(res)), key=lambda i: abs(res[i]))
            if abs(res[worst]) <= max(self.res_max_m, 4 * med):
                E, N, _ = self.basis
                d = x[:3]
                de, dn = sum(a * b for a, b in zip(d, E)), sum(a * b for a, b in zip(d, N))
                # 水平精度因子：Q 投到东、北
                qe = sum(E[i] * E[j] * Q[i][j] for i in range(3) for j in range(3))
                qn = sum(N[i] * N[j] * Q[i][j] for i in range(3) for j in range(3))
                wsum = sum(r[2] for r in rows) / len(rows)
                hdop = math.sqrt(max(0.0, (qe + qn) * wsum))
                if hdop > self.hdop_max:
                    self.reason = 'tdcp_geometry'
                    return None
                rms = math.sqrt(sum(r * r for r in res) / len(res))
                return (de, dn), len(rows), rms
            rows.pop(worst)
            keys.pop(worst)
        self.reason = 'tdcp_sats'
        return None

    # ---- 输出 ----
    def suspect(self):
        """现在要不要把 RTK 当跳了（不采用）。"""
        return self.enabled and self.state == 'jump'

    def status(self):
        r = lambda v, k=4: None if v is None else round(v, k)  # noqa: E731
        return dict(enabled=self.enabled, state=self.state, reason=self.reason, delta_m=r(self.delta_m),
                    sats=self.sats, rms_m=r(self.rms_m), age_s=r(self.age_s, 2), jumps=self.jumps, checks=self.checks,
                    ephemerides=len(self.eph))
