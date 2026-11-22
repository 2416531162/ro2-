"""低速转速修正：用驱动器的位置读数算真实轮速，修正下发给驱动器的目标转速。

为什么要（2026-11-19 架空实测，两台驱动器上位机重设后）：驱动器自己报的转速一直等于指令，但按位置读数
（和用户数圈对过）算，0.1 m/s 实际 1.83×、0.2 m/s 1.6–1.7×，0.3 m/s 右轮跑一半从对的跳到 1.67×，
0.5 m/s 两轮都对。驱动器低速那套测速少算了近一半，速度环就一直往上加 → 低速打颤、起步两轮不一致。

做法：每台驱动器一张「增益表」，按目标轮速（m/s）分几个点，点之间线性插值；下发转速 = 目标 × 增益。
  - 起始值按 11-19 架空实测（见 DEFAULT_TABLE），所以刚开机第一下就基本对。
  - 目标转速稳住一段时间（settle_s）后，用最近 window_s 的位置增量算真实转速，和目标比，慢慢（tau_s）
    调这一速度附近两个点的增益。只在稳速时学，起步、加减速、换向不学，免得追着暂态跑。
  - 学到的表存在车上（file），重启接着用；表坏了 / 读不了就用默认值。
  - 增益夹在 [GAIN_MIN, GAIN_MAX]，每次学习的步子也有上限：驱动器犯病时最多少给/多给这么多，不会失控。
目标 0 永远发 0（停车、急停不受影响）；不改轮程（轮程本来就按位置算）。
"""
import json
import math
import os
import tempfile
from collections import deque

# 轮速节点（m/s，按目标轮速的绝对值）→ 增益。11-19 架空：0.1 → 1/1.84、0.2 → 1/1.65、0.3 左对右半程 1.67×、0.5 都对。
# 0.3–0.4 之间是驱动器两套测速的切换处，增益在这里从约 0.55 跳到约 1.0（11-19 学到 0.3 → 0.5–0.56、0.35 左 0.98）：
# 节点只放 0.3 和 0.4 时线性插值表示不了这个台阶，0.33 m/s（v 0.5、R1 圆弧的内轮）实际只有 92%，所以在 0.32、0.35 加点。
DEFAULT_TABLE = ((0.05, 0.55), (0.1, 0.55), (0.2, 0.55), (0.25, 0.55), (0.3, 0.55), (0.32, 0.8), (0.35, 1.0),
                 (0.4, 1.0), (0.5, 1.0), (0.8, 1.0), (1.2, 1.0))
GAIN_MIN, GAIN_MAX = 0.35, 1.3
MIN_LEARN_MPS = 0.04        # 太慢的位置分辨率不够（霍尔一格就是好几个百分点），不学
MAX_STEP = 0.05             # 每次学习增益最多改 5%（对数）
# 两轮同一查表速度（2026-11-19 用户「改成同一个增益」）：04:28 自动倒车 0.35 m/s 两轮目标 0.33 / 0.37 各查各的，
# 正好跨在 0.30→0.35 的台阶两边，一点点转向被放大成两轮差约 1.9 倍（实际车头转速约指令 3 倍），车头 ±5–10° 来回摆。
# 改成两轮都按两轮平均速度查表：差速在 ±COMMON_BAND_MPS 以内（跟线、直行纠偏）两轮用平均速度这一处的增益，转向不再被放大；
# 差得多（圆弧内外轮、原地转）各自离平均速度超出的部分照旧按自己的速度查，速度一直连续变化不跳。
COMMON_BAND_MPS = 0.05


def _interp(nodes, gains, x):
    if x <= nodes[0]:
        return gains[0]
    if x >= nodes[-1]:
        return gains[-1]
    i = max(k for k in range(len(nodes) - 1) if nodes[k] <= x)
    f = (x - nodes[i]) / (nodes[i + 1] - nodes[i])
    return gains[i] * (1 - f) + gains[i + 1] * f


class SpeedTrim:
    def __init__(self, wheels, *, path='', enabled=True, window_s=0.5, settle_s=1.0, tau_s=1.5, max_age_s=0.3,
                 table=DEFAULT_TABLE, save_every_s=5.0):
        self.wheels, self.path, self.enabled = wheels, path, enabled
        self.window_s, self.settle_s, self.tau_s, self.max_age_s = window_s, settle_s, tau_s, max_age_s
        self.save_every_s = save_every_s
        self.nodes = [n for n, _ in table]
        default = [g for _, g in table]
        self.gains = {1: list(default), 2: list(default)}
        self.load_error = None
        self._dirty, self._saved_at = False, None
        self._load()
        self.pos = {1: deque(maxlen=64), 2: deque(maxlen=64)}
        self.target = {1: 0, 2: 0}          # 上一周期的目标 erpm（修正前）
        self.anchor = {1: 0, 2: 0}          # 这段稳速开始时的目标：慢慢爬坡也算在变
        self.steady_since = {1: None, 2: None}
        self.measured = {1: None, 2: None}  # 位置算的真实转速 erpm（驱动器符号）
        self.learned = 0
        self._last = None

    # ---- 增益表 ----
    def _load(self):
        if not self.path or not os.path.exists(self.path):
            return
        try:
            data = json.load(open(self.path))
            nodes = [float(n) for n in data['nodes']]
            if len(nodes) < 2 or any(b <= a for a, b in zip(nodes, nodes[1:])):
                raise ValueError('轮速节点不合理')
            loaded = {}
            for m, key in ((1, 'left'), (2, 'right')):
                g = [float(x) for x in data[key]]
                if len(g) != len(nodes) or not all(math.isfinite(x) and GAIN_MIN <= x <= GAIN_MAX for x in g):
                    raise ValueError('%s 增益不合理' % key)
                loaded[m] = g
            # 节点和代码不一样（升级加了点）：按旧表插值到新节点，学过的不丢
            for m, g in loaded.items():
                self.gains[m] = g if nodes == self.nodes else [_interp(nodes, g, n) for n in self.nodes]
            if nodes != self.nodes:
                self._dirty = True
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.load_error = '%s: %s（用默认值）' % (type(exc).__name__, exc)

    def save(self, now=None):
        if not self.path or not self._dirty:
            return
        try:
            d = os.path.dirname(self.path) or '.'
            os.makedirs(d, exist_ok=True)
            fd, tmp = tempfile.mkstemp(dir=d, prefix='.speed_trim')
            with os.fdopen(fd, 'w') as f:
                json.dump(dict(nodes=self.nodes, left=[round(g, 4) for g in self.gains[1]],
                               right=[round(g, 4) for g in self.gains[2]]), f)
            os.replace(tmp, self.path)
            self._dirty = False
        except OSError:
            pass
        self._saved_at = now

    def _locate(self, mps):
        """目标轮速 → (左节点下标, 右节点权重)；两头之外按端点。"""
        n = self.nodes
        if mps <= n[0]:
            return 0, 0.0
        if mps >= n[-1]:
            return len(n) - 2, 1.0
        i = max(k for k in range(len(n) - 1) if n[k] <= mps)
        return i, (mps - n[i]) / (n[i + 1] - n[i])

    def gain(self, motor, mps):
        i, f = self._locate(abs(mps))
        g = self.gains[motor]
        return g[i] * (1 - f) + g[i + 1] * f

    @staticmethod
    def lookup_mps(target_mps):
        """两轮目标轮速（m/s，带符号）→ 各轮查增益表用的速度：两轮都在转时以两轮平均速度为准，
        自己偏离平均超出 COMMON_BAND_MPS 的部分才按自己的速度（见 COMMON_BAND_MPS）。"""
        a = {m: abs(v) for m, v in target_mps.items()}
        if any(v == 0 for v in a.values()):
            return a
        mean = sum(a.values()) / len(a)
        return {m: v - max(-COMMON_BAND_MPS, min(COMMON_BAND_MPS, v - mean)) for m, v in a.items()}

    # ---- 每周期 ----
    def _measure(self, now, motor, state):
        if state.t_position is None or state.position_cdeg is None:
            return None
        q = self.pos[motor]
        if not q or q[-1][0] != state.t_position:
            q.append((state.t_position, int(state.position_cdeg)))
        if now - q[-1][0] > self.max_age_s:
            return None
        t1, p1 = q[-1]
        older = [r for r in q if t1 - r[0] >= self.window_s]
        if not older:
            return None
        t0, p0 = older[-1]
        if t1 - t0 > 2 * self.window_s:
            return None
        delta = (p1 - p0 + 2 ** 31) % 2 ** 32 - 2 ** 31
        rpm = delta / 36000.0 / (t1 - t0) * 60.0          # 电机轴每分钟转数
        w = self.wheels
        return rpm * (w.erpm_per_rpm or w.pole_pairs), t0

    def apply(self, now, target, states):
        """target：{1: 左 erpm, 2: 右 erpm}（按 WheelConfig 换算好的目标）→ 修正后要发的 erpm。"""
        dt = 0.0 if self._last is None else min(0.2, max(0.0, now - self._last))
        self._last = now
        per = self.wheels.erpm_per_mps()
        limit = int(round(self.wheels.max_wheel_mps * per))
        out = {}
        look = self.lookup_mps({m: int(target[m]) / per for m in (1, 2)})
        for m in (1, 2):
            e = int(target[m])
            got = self._measure(now, m, states[m]) if m in states else None
            self.measured[m] = got[0] if got else None
            a = self.anchor[m]
            if e == 0 or a == 0 or (e > 0) != (a > 0) or abs(e - a) > 0.02 * abs(a) + 1:
                self.steady_since[m], self.anchor[m] = (now if e else None), e
            self.target[m] = e
            if e == 0 or not self.enabled:
                out[m] = e
                continue
            mps = e / per
            if (got and dt > 0 and self.steady_since[m] is not None and abs(mps) >= MIN_LEARN_MPS
                    and got[1] >= self.steady_since[m] + self.settle_s - self.window_s
                    and now - self.steady_since[m] >= self.settle_s and got[0] * e > 0):
                err = math.log(abs(got[0]) / abs(e))           # >0 = 实际比目标快
                step = max(-MAX_STEP, min(MAX_STEP, -err * dt / self.tau_s))
                i, f = self._locate(look[m])
                g = self.gains[m]
                for k, wk in ((i, 1 - f), (i + 1, f)):
                    if wk > 0:
                        g[k] = min(GAIN_MAX, max(GAIN_MIN, g[k] * math.exp(step * wk)))
                self.learned += 1
                self._dirty = True
            cmd = int(round(e * self.gain(m, look[m])))
            if cmd == 0:
                cmd = 1 if e > 0 else -1
            out[m] = max(-limit, min(limit, cmd))
        if self._dirty and all(v == 0 for v in target.values()) and (
                self._saved_at is None or now - self._saved_at >= self.save_every_s):
            self.save(now)                                     # 停着的时候才写盘
        return out

    def status(self):
        return {'enabled': self.enabled, 'nodes_mps': self.nodes,
                'gain': {'左': [round(g, 3) for g in self.gains[1]], '右': [round(g, 3) for g in self.gains[2]]},
                'measured_erpm': [None if v is None else round(v, 1) for v in (self.measured[1], self.measured[2])],
                'learned': self.learned, 'load_error': self.load_error}
