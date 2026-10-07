#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""卫星定位的固定延迟在线估计：定位比它标的时间晚多少（接收机内部、平板转发路上那段固定延迟），估出来由任务层扣掉。

为什么要：车端按接收机的时间对齐卫星数据，转发路上的抖动没关系，但固定的那段延迟看不出来（接收机和车没有共用的时钟，
um982.GnssClock 只能假设最快到的那帧晚 20 ms）。多出来的延迟 τ 让车以为自己还在「车速 × τ」之后：横向（IMU 管方向）
不受影响，开喷、停喷、拐弯点沿线挪（0.8 m/s 每 10 ms 0.8 cm；全链路仿真单天线晚 50 ms 线头线尾各差约 4 cm）。
2026-10-07 用户：「好的修复」。

怎么估：车停着时卫星位置晚多少都一样。从停着到起步走开的这一段里，卫星测到的天线位置和按轮程 + IMU 航向角推算的
天线位移（加上杠杆臂随车头转的那点）只差一个起点、一个整体转角（推算用的 IMU 航向角和地面差一个偏移）、一个比例
（轮径）和时间差 τ：起点、转角、比例按最小二乘直接解，τ 找让两条曲线对得最好的那个。停着的那几帧定起点，起步加速
那段车速在变，τ 和起点、比例分得开。

新估出来的值等车停稳了才换上（车在动时把卫星时间一下挪几百毫秒，相邻两帧像是位置跳了「车速 × 差值」，融合会当成卫星
抖动变大、十几秒不信卫星；仿真单天线晚 200 ms 时因此停车）。任务层按天线模式存盘，重启后直接用上次的值。一个作业有很多次起步（倒车接入、助跑、拐角原地转
后），取最近几段的中位数。纯逻辑，不依赖 ROS、不依赖融合内部状态。
"""
import cmath
import math
from collections import deque

LATENCY_MIN_S = -0.1        # 搜索范围：晚 −0.1–0.4 s（负的是时间戳反而晚了）
LATENCY_MAX_S = 0.4
LATENCY_STEP_S = 0.005
STILL_SPEED_MPS = 0.005     # 轮速和 IMU 转速都低于这些算停着
STILL_RATE_RAD_S = 0.01
MOVE_SPEED_MPS = 0.03       # 轮速或 IMU 转速（原地转）超过这些才算动了；介于两者之间（起步前万向轮一拧）不打断停稳计时
MOVE_RATE_RAD_S = 0.05
STILL_MIN_S = 0.3           # 停够这么久才算一次起步（换向、原地转后的短停也算）
STOP_S = 0.2                # 动起来以后又停够这么久：这一段结束
WINDOW_M = 4.0              # 一段最多看这么远、这么久，或者车又停下就结束
WINDOW_S = 6.0
MIN_SHIFT_M = 0.8           # 一段至少走这么远、卫星至少这么多帧在动、车速至少到这么快才算
MIN_MOVING_FIXES = 6
MIN_PEAK_SPEED_MPS = 0.25
MAX_SCALE_ERROR = 0.1       # 轮径比例差超过 10 %、对上后残差超过 2 cm 均方根：这段数据有问题（打滑、卫星跳），不用
MAX_RESIDUAL_M = 0.02
# 最多补这么多：单天线 5 Hz 时卫星按 0.2 s 往前挪，两帧之间融合的位置不确定度（编码器轮速噪声约 4 cm/s 推出来的）
# 涨过 3 cm 控制门限，车停着也在可用 / 不可用之间跳、停车（全链路仿真）；0.1 s 正常。超出的部分照报（estimate_s、capped）
LATENCY_APPLY_MAX_S = 0.12
KEEP_WINDOWS = 7            # 取最近这么多段的中位数；第一段合格就先用（一个作业的第一条线线头也补上）
MIN_WINDOWS = 1
BUFFER_S = 10.0


class GnssLatencyEstimator:
    """喂：on_fix（卫星固定解，本地米，接收机标的时间）、on_wheel（轮速，可带区间）、on_imu（IMU 航向角）。
    value：现在用的延迟（秒，还没估出来时是 initial_s）；任务层把卫星数据的时间减去它再交给融合。"""

    def __init__(self, initial_s=0.0):
        self.initial = self.value = float(initial_s)
        self.estimates = deque(maxlen=KEEP_WINDOWS)
        self.last = None                    # 最近一段的结果 dict(latency_s, scale, residual_m, shift_m)，没过的带 reason
        self.pending = None                 # 估出来、等车停稳再换上的值
        self.estimate = None                # 最近几段的中位数（没封顶的）
        self.reset()

    def reset(self, initial_s=None):
        """换了接收机 / 转发路径（单天线 ↔ 双天线）：之前估的不算了，从 initial_s（这一路上次存的值）重来。"""
        if initial_s is not None:
            self.initial = float(initial_s)
        self.value = min(self.initial, LATENCY_APPLY_MAX_S)
        self.pending = self.estimate = None
        self.estimates.clear()
        self.wheel = {}                     # 起点时刻 -> (终点时刻或 None, 车速)；同一起点的新样本替换旧的（区间补全）
        self.imu = deque()                  # (t, 航向角，连续展开)
        self.fixes = deque()                # (t, x, y)
        self.still_since = None
        self.window = None                  # dict(t_still 停下的时刻, t0 起步时刻, start 起步时的卫星位置)
        self._last_speed = 0.0
        self._last_rate = 0.0

    # ---- 输入 ----
    def on_wheel(self, t, speed, end_t=None, end_speed=None):
        if not (math.isfinite(t) and math.isfinite(speed)):
            return
        self.wheel[t] = (end_t if end_t is not None and math.isfinite(end_t) and end_t > t else None, speed)
        now = end_t if end_t is not None and math.isfinite(end_t) else t
        self._last_speed = end_speed if end_speed is not None and math.isfinite(end_speed) else speed
        self._update_motion(now)

    def on_imu(self, t, yaw, rate):
        if not (math.isfinite(t) and math.isfinite(yaw)):
            return
        if self.imu and t <= self.imu[-1][0]:
            return
        if self.imu:
            yaw = self.imu[-1][1] + math.remainder(yaw - self.imu[-1][1], 2 * math.pi)
        self.imu.append((t, yaw))
        self._last_rate = rate if math.isfinite(rate) else 0.0
        self._trim(t)

    def on_fix(self, t, x, y, lever):
        """卫星固定解（天线位置，本地米）。lever：天线在车体系的 (前, 左)，推算天线位移用。"""
        if not all(math.isfinite(v) for v in (t, x, y)):
            return
        self.fixes.append((t, x, y))
        self._lever = lever
        w = self.window
        if w is not None:
            if w['start'] is None:
                w['start'] = complex(x, y)
            elif t - w['t0'] > WINDOW_S or abs(complex(x, y) - w['start']) > WINDOW_M:
                self._finish(t)

    # ---- 一段的起止 ----
    def _update_motion(self, now):
        v, rate = abs(self._last_speed), abs(self._last_rate)
        if v < STILL_SPEED_MPS and rate < STILL_RATE_RAD_S:
            if self.still_since is None:
                self.still_since = now
            if now - self.still_since >= STOP_S:
                if self.window is not None:
                    self._finish(now)
                if self.pending is not None:      # 停稳了：换上新值
                    self.value, self.pending = self.pending, None
            return
        if v <= MOVE_SPEED_MPS and rate <= MOVE_RATE_RAD_S:
            return                          # 停和动之间的小抖动：不算停、也不打断停稳计时
        if self.window is None and self.still_since is not None and now - self.still_since >= STILL_MIN_S:
            self.window = dict(t_still=self.still_since, t0=now, start=None)
        self.still_since = None
        w = self.window
        if w is not None and now - w['t0'] > WINDOW_S:
            self._finish(now)

    def _trim(self, now):
        while self.imu and self.imu[0][0] < now - BUFFER_S:
            self.imu.popleft()
        while self.fixes and self.fixes[0][0] < now - BUFFER_S:
            self.fixes.popleft()
        for t in [t for t in self.wheel if t < now - BUFFER_S]:
            del self.wheel[t]

    # ---- 推算位移 ----
    def _yaw_at(self, t):
        imu = self.imu
        if not imu:
            return None
        if t <= imu[0][0]:
            return imu[0][1]
        if t >= imu[-1][0]:
            return imu[-1][1]
        lo, hi = 0, len(imu) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if imu[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        (t0, y0), (t1, y1) = imu[lo], imu[hi]
        return y0 + (y1 - y0) * (t - t0) / (t1 - t0)

    def _dead_reckoning(self, t0, t1):
        """t0 起（这时还停着）按轮程 + IMU 航向角推算的后轴位移轨迹：[(t, 复数位移, 航向角)]，按轮速样本的时刻。"""
        keys = sorted(k for k in self.wheel if k < t1)
        out = [(t0, 0j, self._yaw_at(t0))]
        pos = 0j
        for i, k in enumerate(keys):
            end, v = self.wheel[k]
            if end is None:
                end = keys[i + 1] if i + 1 < len(keys) else k
                end = min(end, k + 0.2)
            a, b = max(k, t0), min(end, t1)
            if b <= a:
                continue
            steps = max(1, int(math.ceil((b - a) / 0.02)))
            for j in range(steps):
                ta, tb = a + (b - a) * j / steps, a + (b - a) * (j + 1) / steps
                yaw = self._yaw_at((ta + tb) / 2)
                if yaw is None:
                    return None
                pos += v * (tb - ta) * cmath.exp(1j * yaw)
                out.append((tb, pos, self._yaw_at(tb)))
        return out

    def _finish(self, now):
        w, self.window = self.window, None
        lever = getattr(self, '_lever', None)
        if w is None or lever is None:
            return
        t0 = w['t_still']                   # 从停下那一刻起推算（起步前轮速约为 0）
        fixes = [(t, complex(x, y)) for t, x, y in self.fixes if t0 <= t <= now]
        track = self._dead_reckoning(t0, now + LATENCY_MAX_S)
        result = self._fit(t0, fixes, track, complex(*lever))
        self.last = result
        if 'reason' in result:
            return
        self.estimates.append(result['latency_s'])
        if len(self.estimates) >= MIN_WINDOWS:
            ordered = sorted(self.estimates)
            n = len(ordered)
            self.estimate = ordered[n // 2] if n % 2 else (ordered[n // 2 - 1] + ordered[n // 2]) / 2
            self.pending = min(self.estimate, LATENCY_APPLY_MAX_S)

    @staticmethod
    def _interp(track, t):
        if t <= track[0][0]:
            return track[0][1], track[0][2]
        if t > track[-1][0]:
            return None
        lo, hi = 0, len(track) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if track[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        (ta, pa, ya), (tb, pb, yb) = track[lo], track[hi]
        f = (t - ta) / (tb - ta) if tb > ta else 0.0
        return pa + (pb - pa) * f, ya + (yb - ya) * f

    def _fit(self, t0, fixes, track, lever):
        """找 τ：卫星位置 g(t) ≈ 起点 + z · d(t − τ)，d 是推算的天线位移（起步前为 0），z = 比例 × 转角（复数，闭式最小二乘）。"""
        if track is None or len(track) < 2 or track[0][2] is None or not fixes:
            return dict(reason='no_motion_data')
        first = fixes[0][1]
        moving = [g for _, g in fixes if abs(g - first) > 0.05]
        shift, peak = 0.0, 0.0                       # 推算走过的路程（倒车、换向都算）、最高车速
        for (ta, pa, _), (tb, pb, _) in zip(track, track[1:]):
            shift += abs(pb - pa)
            if tb > ta:
                peak = max(peak, abs(pb - pa) / (tb - ta))
        if shift < MIN_SHIFT_M or len(moving) < MIN_MOVING_FIXES or peak < MIN_PEAK_SPEED_MPS:
            return dict(reason='too_short', shift_m=round(shift, 3))
        lever0 = lever * cmath.exp(1j * track[0][2])

        def cost(tau):
            pairs = []
            for t, g in fixes:
                at = self._interp(track, t - tau)
                if at is None:
                    continue
                p, yaw = at
                pairs.append((g, p + lever * cmath.exp(1j * yaw) - lever0))
            if len(pairs) < MIN_MOVING_FIXES:
                return math.inf, None, len(pairs)
            # g ≈ 起点 + z·d：复数带截距的最小二乘
            gm = sum(g for g, _ in pairs) / len(pairs)
            dm = sum(d for _, d in pairs) / len(pairs)
            den = sum(abs(d - dm) ** 2 for _, d in pairs)
            if den <= 0:
                return math.inf, None, len(pairs)
            z = sum((d - dm).conjugate() * (g - gm) for g, d in pairs) / den
            c0 = gm - z * dm
            return sum(abs(g - c0 - z * d) ** 2 for g, d in pairs), z, len(pairs)

        n_steps = int(round((LATENCY_MAX_S - LATENCY_MIN_S) / LATENCY_STEP_S))
        taus = [LATENCY_MIN_S + i * LATENCY_STEP_S for i in range(n_steps + 1)]
        costs = [cost(tau) for tau in taus]
        i = min(range(len(taus)), key=lambda k: costs[k][0])
        c, z, n = costs[i]
        if not math.isfinite(c) or i in (0, len(taus) - 1):
            return dict(reason='no_minimum', shift_m=round(shift, 3))
        # 抛物线插值到网格之间
        c0, c2 = costs[i - 1][0], costs[i + 1][0]
        den = c0 - 2 * c + c2
        tau = taus[i] + (0.5 * (c0 - c2) / den * LATENCY_STEP_S if den > 0 and math.isfinite(den) else 0.0)
        rms = math.sqrt(c / n)
        scale = abs(z)
        if abs(scale - 1.0) > MAX_SCALE_ERROR or rms > MAX_RESIDUAL_M:
            return dict(reason='inconsistent', scale=round(scale, 4), residual_m=round(rms, 4), shift_m=round(shift, 3))
        return dict(latency_s=round(tau, 4), scale=round(scale, 4), residual_m=round(rms, 4), shift_m=round(shift, 3))

    def status(self):
        return dict(value_s=round(self.value, 4), pending_s=None if self.pending is None else round(self.pending, 4),
                    estimate_s=None if self.estimate is None else round(self.estimate, 4),
                    capped=self.estimate is not None and self.estimate > LATENCY_APPLY_MAX_S,
                    windows=len(self.estimates),
                    recent_s=[round(v, 4) for v in self.estimates], last=self.last)
