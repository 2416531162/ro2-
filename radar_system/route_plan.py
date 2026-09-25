#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""路线几何：app 画的路线（连线路线 / 路点折线 / 圆）→ path_tracker.Path（一串等曲率段）。纯计算，不依赖 ROS。

连线路线  （app 默认，kind='path'）操作员在地图上点出点、自己把点一段段连起来，points 就是按连线顺序的点，
          相邻两点之间走直线：不自动连线；没连回起点就不闭合、不绕圈；连回起点（closed）才按 laps 跑圈。
          车从当前位姿出发，先满舵转到对准第 1 个点再直线开过去，然后纯追踪（前瞻 PATH_LOOKAHEAD_M）
          沿连线模拟一遍，曲率限制在车能稳定跟踪的范围内，每 PATH_STEP_M 记一段，得到"车实际能走的线"
          ——跟踪器跟它，app 也把它画出来。点距不限；比车的最小转弯半径还急的拐角，参考线尽量贴着转过去。
          第一个点在车后方或紧贴车身侧面（不绕圈到不了）时拒绝。
路点折线  相邻路点之间走直线，拐角用半径 turn_radius 的圆弧过渡（圆弧与两边直线相切，方向连续）。
          闭合路线最后一段回到第 1 个路点；多圈时第 1 个路点的拐角同样按圆弧过渡，最后一圈停在第 1 个路点。
圆        从圆的最南点出发（逆时针时车头朝东、顺时针朝西），转 laps 圈回到这里停车。
接入      从车当前位姿到路线起点（第 1 个路点、车头朝第 1 段方向；圆为最南点）用 Dubins 最短路径：
          半径 turn_radius 的圆弧 + 直线 + 圆弧，或三段圆弧。只前进，不倒车。

坐标：本地 ENU 平面（x 东 y 北），偏航东为 0、逆时针为正；曲率左转为正。
app 端 RouteGeometry.kt 按同样的规则画预览，两边的一致性由 export_app_fixtures 导出的样例检查。
"""
import math
from dataclasses import dataclass, replace

KINDS = ('path', 'points', 'circle')
MAX_POINTS = 400
PATH_STEP_M = 0.1            # 连线路线参考线的步长
PATH_LOOKAHEAD_M = 1.0       # 纯追踪前瞻距离：直角弯冲出最少（1.0 m 时约 1 m），平缓线仍贴线（几厘米）
ENTRY_MAX_TURN_RAD = math.pi # 到第一个点之前累计转向超过半圈：要绕圈才能到，不做
START_REACH_M = 0.2          # 离第一个点这么近算到了（接入段是直线对准它开过去的）
MAX_LAPS = 10
MAX_COORD_M = 1000.0         # 本地坐标范围（离原点）
MIN_LEG_M = 0.2              # 相邻路点最小间距
MAX_TURN_DEG = 170.0         # 拐角转向超过这个角度就是折返，圆弧过渡需要的直线太长
MAX_ENTRY_M = 30.0           # 接入路线最长：车离起点太远时先开近，不替人规划长距离穿越
TURN_RADIUS_RANGE = (1.0, 20.0)
CIRCLE_RADIUS_RANGE = (0.5, 100.0)
CRUISE_RANGE = (0.1, 1.0)


@dataclass(frozen=True)
class RouteSpec:
    kind: str = 'points'
    points: tuple = ()           # ((x, y), …)；存盘时是 ((lat, lon), …)
    closed: bool = True
    center: tuple = None         # 圆心 (x, y)
    radius: float = 3.0          # 圆半径
    clockwise: bool = True
    laps: int = 1
    turn_radius: float = 2.5     # 拐角圆弧、接入路线的转弯半径
    cruise: float = 0.5

    def map_points(self, fn):
        """把路点和圆心逐个换算（本地 ↔ 经纬度），其余参数不变。"""
        return replace(self, points=tuple(tuple(fn(*p)) for p in self.points),
                       center=None if self.center is None else tuple(fn(*self.center)))

    def to_dict(self, digits=3):
        def r(p):
            return [round(p[0], digits), round(p[1], digits)]
        return dict(kind=self.kind, points=[r(p) for p in self.points], closed=self.closed,
                    center=None if self.center is None else r(self.center), radius=self.radius,
                    clockwise=self.clockwise, laps=self.laps, turn_radius=self.turn_radius, cruise=self.cruise)


def _number(value, name, lo, hi):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('%s 必须是有限数值' % name)
    if not lo <= value <= hi:
        raise ValueError('%s 应在 %g–%g' % (name, lo, hi))
    return float(value)


def _pair(value, name, lo, hi):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError('%s 必须是两个数' % name)
    return _number(value[0], name, lo[0], hi[0]), _number(value[1], name, lo[1], hi[1])


def parse_spec(d, *, geodetic=False):
    """app 发来的路线（本地米）或存盘的路线（geodetic=True，经纬度）→ RouteSpec。

    只检查结构和取值范围：路点不够、拐角太急这类"还没画完"的问题允许保存，开始前由 loop_pieces 报告。
    """
    if not isinstance(d, dict):
        raise ValueError('路线必须是 JSON 对象')
    kind = d.get('kind')
    if kind not in KINDS:
        raise ValueError('路线类型必须是 ' + '/'.join(KINDS))
    lo, hi = ((-90.0, -180.0), (90.0, 180.0)) if geodetic else ((-MAX_COORD_M,) * 2, (MAX_COORD_M,) * 2)
    raw = d.get('points', [])
    if not isinstance(raw, (list, tuple)) or len(raw) > MAX_POINTS:
        raise ValueError('路点最多 %d 个' % MAX_POINTS)
    points = tuple(_pair(p, '路点坐标', lo, hi) for p in raw)
    center = d.get('center')
    center = None if center is None else _pair(center, '圆心坐标', lo, hi)
    flags = {k: d.get(k, default) for k, default in (('closed', True), ('clockwise', True))}
    for k, v in flags.items():
        if not isinstance(v, bool):
            raise ValueError('%s 必须是 true/false' % k)
    laps = d.get('laps', 1)
    if isinstance(laps, bool) or not isinstance(laps, int) or not 1 <= laps <= MAX_LAPS:
        raise ValueError('圈数应为 1–%d' % MAX_LAPS)
    if kind == 'path' and not flags['closed']:
        laps = 1            # 没连回起点：走一遍，不绕圈
    return RouteSpec(kind=kind, points=points, closed=flags['closed'], center=center,
                     radius=_number(d.get('radius', 3.0), '圆半径', *CIRCLE_RADIUS_RANGE),
                     clockwise=flags['clockwise'], laps=laps if flags['closed'] or kind == 'circle' else 1,
                     turn_radius=_number(d.get('turn_radius', 2.5), '转弯半径', *TURN_RADIUS_RANGE),
                     cruise=_number(d.get('cruise', 0.5), '速度', *CRUISE_RANGE))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def advance(x, y, yaw, length, kappa):
    """从 (x, y, yaw) 沿等曲率段走 length 米后的位姿（与 path_tracker.Piece.pose 同一公式）。"""
    if abs(kappa) < 1e-12:
        return x + length * math.cos(yaw), y + length * math.sin(yaw), yaw
    yaw2 = yaw + kappa * length
    return x + (math.sin(yaw2) - math.sin(yaw)) / kappa, y - (math.cos(yaw2) - math.cos(yaw)) / kappa, yaw2


# ---- 连线路线 ----

def _polyline(points):
    """去掉重合点（< 1 mm），返回 (点, 累计长度)。"""
    pts = []
    for p in points:
        if not pts or math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) >= 1e-3:
            pts.append((float(p[0]), float(p[1])))
    cum = [0.0]
    for a, b in zip(pts, pts[1:]):
        cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    return pts, cum


def _point_at(pts, cum, s):
    if s <= 0.0 or len(pts) == 1:
        return pts[0]
    for i in range(1, len(pts)):
        if s <= cum[i]:
            u = (s - cum[i - 1]) / (cum[i] - cum[i - 1])
            return (pts[i - 1][0] + u * (pts[i][0] - pts[i - 1][0]), pts[i - 1][1] + u * (pts[i][1] - pts[i - 1][1]))
    return pts[-1]


def _project(pts, cum, x, y, lo, hi):
    """折线上路程在 [lo, hi] 内离 (x, y) 最近的点的路程（同样近取路程小的）。"""
    best_d, best_s = math.inf, lo
    for i in range(1, len(pts)):
        a0, a1 = cum[i - 1], cum[i]
        if a1 < lo or a0 > hi:
            continue
        (ax, ay), (bx, by) = pts[i - 1], pts[i]
        seg = a1 - a0
        u = ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / (seg * seg)
        u = max((lo - a0) / seg, min((hi - a0) / seg, max(0.0, min(1.0, u))))
        d = math.hypot(x - ax - u * (bx - ax), y - ay - u * (by - ay))
        if d < best_d - 1e-12:
            best_d, best_s = d, a0 + u * seg
    return best_s


def path_points(spec):
    """连线路线实际要走的折线：闭合时按圈数重复并回到起点。"""
    pts = list(spec.points)
    if spec.closed and len(pts) >= 3:
        pts = pts * spec.laps + [pts[0]]
    return pts


def path_length(points):
    return _polyline(points)[1][-1] if points else 0.0


def drawn_reference(points, car, min_radius, *, step=PATH_STEP_M, lookahead=PATH_LOOKAHEAD_M):
    """连线折线 + 车当前位姿 → (参考线分段 [(长度, 曲率), …], 接入长度米, 参考线离连线的最大距离米)。

    还没到第一个点时：满舵转到车头对准它，再直线开过去。到了（START_REACH_M 以内）以后纯追踪：瞄折线上当前投影
    再往前 lookahead 的点，投影只往前找（不会跳到画线交叉的另一段）；曲率限制在 1/min_radius。终点在前瞻内后
    直接瞄终点，距离不足一步时补一段直线停在它旁边。到第一个点前累计转向超过半圈、或总里程超过合理上限
    （车转不过去、越走越偏）时抛 ValueError。
    """
    if not points:
        raise ValueError('先在地图上点点、连线')
    pts, cum = _polyline(points)
    total, kmax = cum[-1], 1.0 / min_radius
    x, y, yaw = car
    entry_d = math.hypot(pts[0][0] - x, pts[0][1] - y)
    limit = 3.0 * (entry_d + total) + 10.0
    reached = entry_d <= START_REACH_M
    s = travelled = turned = 0.0
    entry_m = 0.0 if reached else None
    pieces, dev = [], 0.0
    while True:
        if reached:
            s = _project(pts, cum, x, y, s, s + 2.0 * lookahead)
            if s > 0.0:     # 只算沿线走的偏离，开往起点那一段不算
                px, py = _point_at(pts, cum, s)
                dev = max(dev, math.hypot(x - px, y - py))
            final = s + lookahead >= total
            tx, ty = pts[-1] if final else _point_at(pts, cum, s + lookahead)
        else:
            final = total == 0.0
            tx, ty = pts[0]
        dx, dy = tx - x, ty - y
        d = math.hypot(dx, dy)
        alpha = wrap(math.atan2(dy, dx) - yaw) if d > 1e-9 else 0.0
        if final and (reached or total == 0.0) and (d <= step or abs(alpha) > math.pi / 2):
            ahead = d * math.cos(alpha)
            if ahead > 1e-3:
                pieces.append((ahead, 0.0))
            break
        if d <= 1e-9:
            k = 0.0
        elif not reached:
            # 开往第一个点：先满舵转到车头对准它，再直线开过去（转过的角度最小，不多绕）。
            # 纯追踪走圆弧到侧面的点要转两倍方位角，60–90° 的点就得转大半圈
            # 偏角小于 1e-6 rad 就当已对准走直线：残差除以步长会把浮点误差放大（30 m 只差 0.03 mm）
            k = math.copysign(min(kmax, abs(alpha) / step), alpha) if abs(alpha) > 1e-6 else 0.0
            if abs(alpha) >= math.pi - 1e-9:
                k = kmax                       # 正后方：往左转
        elif abs(alpha) > math.pi / 2:
            k = kmax if alpha >= 0.0 else -kmax    # 线在车后半边（急弯没跟上）：满舵往回转
        else:
            k = max(-kmax, min(kmax, 2.0 * math.sin(alpha) / d))
        x, y, yaw = advance(x, y, yaw, step, k)
        pieces.append((step, k))
        travelled += step
        if not reached:
            turned += abs(k) * step
            if turned > ENTRY_MAX_TURN_RAD:
                raise ValueError('第一个点在车后方或太靠近车身侧面：车不倒车也不绕圈，到不了。请把第一个点画在车头前方')
            if math.hypot(pts[0][0] - x, pts[0][1] - y) <= START_REACH_M:
                reached, entry_m = True, travelled
        if travelled > limit:
            raise ValueError('路线里有车转不过去的地方（车最小转弯半径 %.2f m），参考线越走越偏' % min_radius)
    merged = []
    for length, k in pieces:
        if merged and merged[-1][1] == k:
            merged[-1] = (merged[-1][0] + length, k)
        else:
            merged.append((length, k))
    return merged, (entry_m if entry_m is not None else travelled), dev


# ---- 路线本身（不含接入段）----

def loop_pieces(spec, min_radius):
    """返回 (起点位姿, [(长度, 曲率), …])；画不成（路点不够、拐角太急、半径太小）时抛 ValueError（中文说明）。"""
    if spec.turn_radius < min_radius - 1e-9:
        raise ValueError('转弯半径 %.2f m 小于车能稳定跟踪的 %.2f m' % (spec.turn_radius, min_radius))
    if spec.kind == 'path':
        raise ValueError('连线路线要从车当前位置算，用 drawn_reference')
    if spec.kind == 'circle':
        if spec.center is None:
            raise ValueError('还没有设置圆心')
        if spec.radius < min_radius - 1e-9:
            raise ValueError('圆半径 %.2f m 小于车能稳定跟踪的 %.2f m' % (spec.radius, min_radius))
        cx, cy = spec.center
        sign = -1.0 if spec.clockwise else 1.0
        start = (cx, cy - spec.radius, math.pi if spec.clockwise else 0.0)
        return start, [(2.0 * math.pi * spec.radius * spec.laps, sign / spec.radius)]
    return _points_loop(spec)


def _points_loop(spec):
    pts, closed, r = spec.points, spec.closed, spec.turn_radius
    n = len(pts)
    if n < (3 if closed else 2):
        raise ValueError('%s路线至少要 %d 个路点' % ('闭合' if closed else '', 3 if closed else 2))
    legs = []                                      # (长度, 方向)
    for i in range(n if closed else n - 1):
        (x0, y0), (x1, y1) = pts[i], pts[(i + 1) % n]
        length = math.hypot(x1 - x0, y1 - y0)
        if length < MIN_LEG_M:
            raise ValueError('第 %d、%d 个路点离得太近' % (i + 1, (i + 1) % n + 1))
        legs.append((length, math.atan2(y1 - y0, x1 - x0)))
    laps = spec.laps if closed else 1
    turn, tangent = [0.0] * n, [0.0] * n           # 每个路点处的转向角、圆弧切点到路点的距离
    # 闭合路线只跑 1 圈时第 1 个路点不拐弯（从这里出发、在这里停）
    corners = range(n) if closed and laps > 1 else range(1, n) if closed else range(1, n - 1)
    for j in corners:
        d = wrap(legs[j % len(legs)][1] - legs[j - 1][1])
        if abs(d) > math.radians(MAX_TURN_DEG):
            raise ValueError('第 %d 个路点处几乎原路折返，车不能原地掉头' % (j + 1))
        turn[j], tangent[j] = d, r * math.tan(abs(d) / 2.0)
    for i, (length, _) in enumerate(legs):
        need = tangent[i] + tangent[(i + 1) % n]
        if need > length + 1e-9:
            raise ValueError('第 %d→%d 个路点太近：按 %.1f m 半径拐弯至少要 %.1f m，现在只有 %.1f m'
                             % (i + 1, (i + 1) % n + 1, r, need, length))
    pieces = []
    for lap in range(laps):
        for i, (length, _) in enumerate(legs):
            first, last = lap == 0 and i == 0, lap == laps - 1 and i == len(legs) - 1
            end = (i + 1) % n
            straight = length - (0.0 if first else tangent[i]) - (0.0 if last else tangent[end])
            if straight > 1e-6:
                pieces.append((straight, 0.0))
            if not last and r * abs(turn[end]) > 1e-6:
                pieces.append((r * abs(turn[end]), math.copysign(1.0 / r, turn[end])))
    return (pts[0][0], pts[0][1], legs[0][1]), pieces


# ---- 接入段：Dubins 最短路径 ----

def _mod2pi(a):
    return a % (2.0 * math.pi)


def _dubins_words(alpha, beta, d):
    """六种 Dubins 组合在单位半径下的 (名字, t, p, q)；不成立的组合不返回。公式见 Shkel & Lumelsky 2001。"""
    sa, sb, ca, cb = math.sin(alpha), math.sin(beta), math.cos(alpha), math.cos(beta)
    cab = math.cos(alpha - beta)
    out = []
    p2 = 2.0 + d * d - 2.0 * cab + 2.0 * d * (sa - sb)
    if p2 >= 0.0:
        tmp = math.atan2(cb - ca, d + sa - sb)
        out.append(('LSL', _mod2pi(-alpha + tmp), math.sqrt(p2), _mod2pi(beta - tmp)))
    p2 = 2.0 + d * d - 2.0 * cab + 2.0 * d * (sb - sa)
    if p2 >= 0.0:
        tmp = math.atan2(ca - cb, d - sa + sb)
        out.append(('RSR', _mod2pi(alpha - tmp), math.sqrt(p2), _mod2pi(-beta + tmp)))
    p2 = -2.0 + d * d + 2.0 * cab + 2.0 * d * (sa + sb)
    if p2 >= 0.0:
        p = math.sqrt(p2)
        tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
        out.append(('LSR', _mod2pi(-alpha + tmp), p, _mod2pi(-_mod2pi(beta) + tmp)))
    p2 = d * d - 2.0 + 2.0 * cab - 2.0 * d * (sa + sb)
    if p2 >= 0.0:
        p = math.sqrt(p2)
        tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
        out.append(('RSL', _mod2pi(alpha - tmp), p, _mod2pi(beta - tmp)))
    c = (6.0 - d * d + 2.0 * cab + 2.0 * d * (sa - sb)) / 8.0
    if abs(c) <= 1.0:
        p = _mod2pi(2.0 * math.pi - math.acos(c))
        t = _mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + p / 2.0)
        out.append(('RLR', t, p, _mod2pi(alpha - beta - t + p)))
    c = (6.0 - d * d + 2.0 * cab + 2.0 * d * (sb - sa)) / 8.0
    if abs(c) <= 1.0:
        p = _mod2pi(2.0 * math.pi - math.acos(c))
        t = _mod2pi(-alpha - math.atan2(ca - cb, d + sa - sb) + p / 2.0)
        out.append(('LRL', t, p, _mod2pi(_mod2pi(beta) - alpha - t + p)))
    return out


def dubins(start, goal, radius):
    """start、goal 为 (x, y, yaw)。返回最短的 [(长度, 曲率), …]（去掉零长度段；已在终点时为空）。

    每个候选都积分验算终点，公式或数值出错的候选直接丢掉，不会把车带到别处。
    """
    x0, y0, yaw0 = start
    dx, dy = goal[0] - x0, goal[1] - y0
    dist = math.hypot(dx, dy)
    if dist < 1e-9 and abs(wrap(goal[2] - yaw0)) < 1e-9:
        return []
    theta = math.atan2(dy, dx)
    alpha, beta = _mod2pi(yaw0 - theta), _mod2pi(goal[2] - theta)
    k = 1.0 / radius
    curv = {'L': k, 'R': -k, 'S': 0.0}
    best = None
    for word, *lengths in _dubins_words(alpha, beta, dist / radius):
        pieces = [(u * radius, curv[c]) for c, u in zip(word, lengths) if u * radius > 1e-6]
        x, y, yaw = start
        for length, kappa in pieces:
            x, y, yaw = advance(x, y, yaw, length, kappa)
        if math.hypot(x - goal[0], y - goal[1]) > 1e-6 * max(1.0, dist) or abs(wrap(yaw - goal[2])) > 1e-6:
            continue
        total = sum(length for length, _ in pieces)
        if best is None or total < best[0] - 1e-9:
            best = (total, pieces)
    if best is None:
        raise ValueError('无法规划接入路线')
    return best[1]


def route_length(spec, min_radius):
    """(路线长度米, 画不成的原因)，不含接入段；状态里报给 app。"""
    if spec.kind == 'path':
        if len(spec.points) < 2:
            return None, '先在地图上点点，再把点连起来'
        return path_length(path_points(spec)), None
    try:
        _, pieces = loop_pieces(spec, min_radius)
    except ValueError as exc:
        return None, str(exc)
    return sum(p[0] for p in pieces), None


def plan(spec, car_pose, min_radius, *, max_entry_m=MAX_ENTRY_M):
    """整条要跑的路径：从车当前位姿接入 + 路线本身。返回 (Path, 接入长度米)。"""
    from path_tracker import Path   # path_tracker 依赖 robot_core；只在真要建路径时导入
    if spec.kind == 'path':
        if len(spec.points) < 2:
            raise ValueError('先在地图上点点，再把点连起来')
        pieces, entry_m, _ = drawn_reference(path_points(spec), car_pose, min_radius)
        if entry_m > max_entry_m:
            raise ValueError('车离路线起点太远（%.0f m，最多 %.0f m），请先把车开近些' % (entry_m, max_entry_m))
        if not pieces:
            raise ValueError('车已经在终点上')
        return Path(car_pose, pieces, name='route_path'), entry_m
    start, loop = loop_pieces(spec, min_radius)
    entry = dubins(car_pose, start, spec.turn_radius)
    entry_m = sum(length for length, _ in entry)
    if entry_m > max_entry_m:
        raise ValueError('车离路线起点太远（接入路线 %.0f m，最多 %.0f m），请先把车开近些' % (entry_m, max_entry_m))
    return Path(car_pose, entry + loop, name='route_' + spec.kind), entry_m
