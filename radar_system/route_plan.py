#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""路线几何：app 画的路线（连线路线 / 路点折线 / 圆）→ 分段路径（每段一个 path_tracker.Path，前进或倒车）。
纯计算，不依赖 ROS。

连线路线  （app 默认，kind='path'）操作员在地图上点出点、自己把点一段段连起来，points 就是按连线顺序的点，
          相邻两点之间走直线：不自动连线；没连回起点就不闭合、不绕圈；连回起点（closed）才按 laps 跑圈。
          车身中心（车长中点，在后轴前 (front−rear)/2）要压过全部连线，拐角点也压到，不能漏：
            缓弯  后轴按最小半径的圆弧直接转过去，车身中心离拐角点不超过 COVER_TOL_M，不停车；
            急弯  掉头补角：车身中心开过拐角 VERTEX_PASS_M 后继续往前，等整车都开出拐角（车尾也过了
                  CORNER_CLEAR_M）→ 倒车一次，车头摆向下一条边 → 往前开上下一条边，车身中心从拐角后面开始压。
                  三段都可以打方向，在满足上面顺序的走法里取最短的（Reeds-Shepp 最短路径，Nav2 / OMPL 同一套），
                  90° 多开约 3.0 m、换向 2 次，是这台车掉 90° 头的最短距离。
          起步用 Reeds-Shepp 最短路径（可以倒车）开到第一段延长线上、车身中心到第 1 个点前 ENTRY_LEAD_M 处对正，
          车已经压在第一段延长线上时直接开过去。终点车身中心多走 VERTEX_PASS_M。
          差速车（Gen5，能原地转）改按喷头压线、急弯停车原地转向，见 pivot_cover / pivot_entry；
          路线选「圆弧掉头」（turn='arc'）时急弯不停车：喷头开过拐角后按转弯半径绕圆弧接到下一条边上，见 _arc_turn。
路点折线  相邻路点之间走直线，拐角用半径 turn_radius 的圆弧过渡（圆弧与两边直线相切，方向连续）。
          闭合路线最后一段回到第 1 个路点；多圈时第 1 个路点的拐角同样按圆弧过渡，最后一圈停在第 1 个路点。
圆        从圆的最南点出发（逆时针时车头朝东、顺时针朝西），转 laps 圈回到这里停车。
          差速车（Gen5）按喷头画圆：喷头在后轴中点前 f、左 l，后轴中点走半径 √(R²−f²) ± l（逆时针 +）的同心圆，
          喷头正好在半径 R 上；最后多走 VERTEX_PASS_M 让喷头盖过起点。接入同连线路线（pivot_entry，原地转向）。
          独立画圆时喷头画过起点（交点）再多喷 CIRCLE_OVERLAP_M 才停喷，沿圆继续 SOLO_CIRCLE_COAST_M，随后沿后轴切线前进 SOLO_CIRCLE_EXIT_M
          再停车（_circle_run_out）。直线助跑之后先沿圆走 CIRCLE_RUN_IN_M 不喷，转弯稳住了再起喷（_circle_run_in）。
喷头位置  nozzle_x 是数时喷头在车中线上、后轴中点前这么多米（共享配置 drive.nozzle_x_m）；是 (前, 左) 二元组时喷头
          可以在侧面（app 的「对准点」，2026-10-02 用户：喷头在右天线右 25 cm），连线路线见 _side_cover。
开始作业  （app 的 start_current，plan_current）路线不分起点终点：从离车最近的地方开始、跑完整条（圆、闭合路线绕完整圈
          回到起步处，开放路线从近的那一头画到另一头），见 start_from_nearest；差速车喷头离起点近就倒车助跑进线（runup_entry）。
接入      路点折线和圆：从车当前位姿到路线起点（第 1 个路点、车头朝第 1 段方向；圆为最南点）用 Dubins
          最短路径：半径 turn_radius 的圆弧 + 直线 + 圆弧，或三段圆弧。只前进，不倒车。

坐标：本地 ENU 平面（x 东 y 北），偏航东为 0、逆时针为正；曲率左转为正。
app 端 RouteGeometry.kt 按同样的规则画预览，两边的一致性由 export_app_fixtures 导出的样例检查。
"""
import math
from motion_speeds import DEFAULT_PAINT_SPEED
from dataclasses import dataclass, replace

KINDS = ('path', 'points', 'circle')
# 差速车连线路线的急弯怎么转（app 可选，存在路线里）：
#   pivot  停车原地转向，倒车助跑回拐角（默认，旧路线没有这个字段也是它）
#   arc    圆弧掉头：像阿克曼车那样只往前开、不停车，按路线的 turn_radius 绕过去接到下一条边上（见 _arc_turn）
TURNS = ('pivot', 'arc')
MAX_POINTS = 400
COVER_TOL_M = 0.05           # 缓弯直接圆弧转过去时，车身中心离拐角点最多这么远；更急的弯掉头补角
VERTEX_PASS_M = 0.05         # 车身中心开过拐角 / 终点这么远才换向、停车：差一两厘米也压得过去
# 差速车直线线尾：图纸终点停喷，然后沿原方向开出 40 cm 缓停（2026-10-05）。
# END_PASS_M 保留旧接口；画圆和阿克曼车仍使用 VERTEX_PASS_M。
END_PASS_M = 0.0
LINE_RUN_OUT_M = 0.40
# 独立画圆（2026-10-09）：过起点多喷 10 cm，停喷后沿同一圆再走 30 cm，均按喷头沿图纸圆的弧长计。
CIRCLE_OVERLAP_M = 0.10
SOLO_CIRCLE_COAST_M = 0.30
CIRCLE_RUN_OUT_M = 0.50
# 仅独立画圆：完成停喷后的沿圆段，再沿当前车头方向离开圆，前进 1 m 再停车。
SOLO_CIRCLE_EXIT_M = 1.00
# 差速车画圆起喷（2026-10-06 用户：交界处两条线没重合，「车在圆上转了 1 m，再开始喷」）：直线助跑后先沿圆走
# 1 m（喷头沿图纸上的圆量）不喷，再从原来的起点开始喷。原来直线助跑到起点才突然转弯，转向延迟约 0.3 s，
# 开头 0.7–1 m 喷头偏内 1–1.6 cm、航向差 1–2°，一圈回来多喷的 0.2 m 压在线上，交界处错开 1–1.8 cm。
# 2026-10-07 1 m → 2 m（用户：接入改聪明，但「线头不能差，要么对给助跑」）：仿真（Gen5 拟合底盘，R 1.5 / 3 m 顺逆 × 7 种摆位 ×
# 16 种子）起喷后 0.5 m 内最大偏差中位 原来 8.2 mm → 就近接入 + 沿圆 1.5 m 6.7、+ 2 m 5.0；入弧那下的偏差要 1–2 m 才收回
# 2026-10-09 2 m → 2.5 m（用户：「圆上多走 0.5 m 再喷」）：实录 9 个圆进圆的摆动要 3–5 m 才收完，起喷点平均偏 1.1 cm
CIRCLE_RUN_IN_M = 2.50
# 进圆过渡曲线（2026-10-09 用户：「进圆加一段过渡曲线」）：直线助跑和圆之间接一段 CIRCLE_TRANSITION_M 长、曲率从 0
# 均匀增到圆曲率的过渡（CIRCLE_TRANSITION_STEPS 段等长圆弧，每段取段中点的曲率），接上圆后再沿圆走 CIRCLE_RUN_IN_M
# 才喷。原来直线在切点一步跳到圆曲率，两轮转速一下变 ±7 %，驱动器速度环（实录：约 1 Hz、阻尼比 0.3–0.4，阶跃冲过头
# 20–30 %）先少转后多转；过渡 1 m（0.5 m/s 约 2 s）比速度环晃一下慢，不再激起摆动。路线和前馈一起变缓，和 10-04
# 试过的「只平滑前馈」不同。圆本身、起喷点、多喷和收尾都不变；直线助跑整体往圆外挪约 r·(L/r)²/24（R 2 m 约 1.7 cm）
CIRCLE_TRANSITION_M = 1.00
CIRCLE_TRANSITION_STEPS = 10
CORNER_CLEAR_M = 0.05        # 急弯先把整车开出拐角（车尾也过了拐角这么远）再倒车
CORNER_EXTRA_M = tuple(0.05 * i for i in range(21))     # 急弯倒车前可以再多往前开这么远（候选，找最短）
CORNER_BACK_M = (0.0, 0.1, 0.2, 0.3)                    # 倒车后可以停得离拐角再远这么多（候选）
MIN_RUN_M = 0.05             # 两次换向之间至少开这么远，太短的段控制器跟不上
CUSP_TIE_M = 0.01            # 一样长的走法里挑换向少的
ENTRY_LEAD_M = 1.0           # 接入段在车身中心到第 1 个点前这么远对正第一段，控制器到点前已稳住。
                             # 2026-09-30 实车：原地转向后起步万向轮甩尾，横偏 1.1 m 左右才收回来，0.5 m 不够
ON_LINE_M = 0.05             # 车已压在第一段延长线上（横偏、车头误差都在这以内）就直接开过去
ON_LINE_DEG = 5.0
REVERSE_ENTRY_MAX_M = 3.0    # 接入时最多倒这么远到助跑起点（倒车能少原地转很多才倒，见 pivot_entry）
REVERSE_ENTRY_TURN_SAVE_RAD = math.radians(90.0)   # 倒过去比开过去少原地转这么多以上才倒；差不多时照旧往前开
# 接入走法估时（entry_seconds，只用来挑走法、挑圆的起点）：每段起停多花 1.3 / 0.7 s、原地转（>5°）0.8 rad/s 另加 1.7 s
# 停稳（field_planner.Settings 的 Gen5 实车标定）；助跑最后 RUN_UP_M 按 0.3 m/s（path_tracker runup_speed_mps）
ENTRY_FORWARD_MPS = 0.8
ENTRY_REVERSE_MPS = 0.5
ENTRY_RUNUP_MPS = 0.3
ENTRY_SECTION_S = 1.3
ENTRY_REVERSE_SECTION_S = 0.7
ENTRY_PIVOT_RATE = 0.8
ENTRY_PIVOT_S = 1.7
ENTRY_PIVOT_MIN_RAD = math.radians(5.0)
MAX_LAPS = 10
MAX_COORD_M = 1000.0         # 本地坐标范围（离原点）
MIN_LEG_M = 0.2              # 相邻路点最小间距
MAX_TURN_DEG = 170.0         # 路点折线：拐角转向超过这个角度就是折返，圆弧过渡需要的直线太长
MAX_ENTRY_M = 30.0           # 接入路线最长：车离起点太远时先开近，不替人规划长距离穿越
NEAR_START_M = 0.5           # 「开始作业」：差速车喷头离起点（路线上离车最近处）这么近就原地转向、倒车助跑进线
                             # （runup_entry），更远按 pivot_entry 开到起点前 ENTRY_LEAD_M 对正
START_TIE_M = 0.05           # 离车差不多一样近（差这么多以内）的起点里挑接入最省事的（如喷头停在圆心）
START_SNAP_M = 0.3           # 闭合连线离车最近处离拐角不到这么远就从拐角开始，不留一小截再拐
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
    turn_radius: float = 2.5     # 拐角圆弧、接入路线的转弯半径；差速车连线路线圆弧掉头的半径
    cruise: float = DEFAULT_PAINT_SPEED
    turn: str = 'pivot'          # 差速车连线路线急弯的转法，见 TURNS；阿克曼车、圆、路点折线不用
    # 圆：后轴从圆上哪个方位角出发（rad，东 0、逆时针为正）；None 为最南点。开始作业时按车的位置临时换
    # （见 start_from_nearest），不存盘、不发给 app
    start_angle: float = None

    def map_points(self, fn):
        """把路点和圆心逐个换算（本地 ↔ 经纬度），其余参数不变。"""
        return replace(self, points=tuple(tuple(fn(*p)) for p in self.points),
                       center=None if self.center is None else tuple(fn(*self.center)))

    def to_dict(self, digits=3):
        def r(p):
            return [round(p[0], digits), round(p[1], digits)]
        return dict(kind=self.kind, points=[r(p) for p in self.points], closed=self.closed,
                    center=None if self.center is None else r(self.center), radius=self.radius,
                    clockwise=self.clockwise, laps=self.laps, turn_radius=self.turn_radius, cruise=self.cruise,
                    turn=self.turn)


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
    turn = d.get('turn', 'pivot')       # 旧 app、旧存盘路线没有：原地转向
    if turn not in TURNS:
        raise ValueError('急弯转法必须是 ' + '/'.join(TURNS))
    return RouteSpec(kind=kind, points=points, closed=flags['closed'], center=center,
                     radius=_number(d.get('radius', 3.0), '圆半径', *CIRCLE_RADIUS_RANGE),
                     clockwise=flags['clockwise'], laps=laps if flags['closed'] or kind == 'circle' else 1,
                     turn_radius=_number(d.get('turn_radius', 2.5), '转弯半径', *TURN_RADIUS_RANGE),
                     cruise=_number(d.get('cruise', DEFAULT_PAINT_SPEED), '速度', *CRUISE_RANGE), turn=turn)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def advance(x, y, yaw, length, kappa):
    """从 (x, y, yaw) 沿等曲率段走 length 米后的位姿（与 path_tracker.Piece.pose 同一公式）。"""
    if abs(kappa) < 1e-12:
        return x + length * math.cos(yaw), y + length * math.sin(yaw), yaw
    yaw2 = yaw + kappa * length
    return x + (math.sin(yaw2) - math.sin(yaw)) / kappa, y - (math.cos(yaw2) - math.cos(yaw)) / kappa, yaw2


# ---- 连线路线：车压过全部连线 ----

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


def path_points(spec):
    """连线路线实际要走的折线：闭合时按圈数重复并回到起点。"""
    pts = list(spec.points)
    if spec.closed and len(pts) >= 3:
        pts = pts * spec.laps + [pts[0]]
    return pts


def path_length(points):
    return _polyline(points)[1][-1] if points else 0.0


def _append(pieces, length, kappa):
    """加一段；零长度不加，与上一段曲率相同就接上。"""
    if length <= 1e-9:
        return
    if pieces and pieces[-1][1] == kappa:
        pieces[-1] = (pieces[-1][0] + length, kappa)
    else:
        pieces.append((length, kappa))


def _offset(p, yaw, d):
    """从点 p 沿 yaw 方向走 d 米（d 为负往回）。"""
    return p[0] + d * math.cos(yaw), p[1] + d * math.sin(yaw)


def nozzle_point(nozzle):
    """压线点（喷头）在车体系的 (前, 左)，m：nozzle 是数时在中线上（旧接口），是二元组时是 (前, 左)。"""
    f, l = (nozzle[0], nozzle[1]) if isinstance(nozzle, (tuple, list)) else (nozzle, 0.0)
    f, l = float(f), float(l)
    if not (math.isfinite(f) and math.isfinite(l)):
        raise ValueError('喷头位置无效')
    return f, l


def axle_for(q, yaw, f, l):
    """喷头（车体系前 f、左 l）正好压在 q、车头 yaw 时后轴中点的位置。"""
    c, s = math.cos(yaw), math.sin(yaw)
    return q[0] - f * c + l * s, q[1] - f * s - l * c


def nozzle_at(x, y, yaw, f, l):
    """后轴中点在 (x, y)、车头 yaw 时喷头（前 f、左 l）的位置。"""
    c, s = math.cos(yaw), math.sin(yaw)
    return x + f * c - l * s, y + f * s + l * c


class _Sections:
    """按车头坐标的 (方向, 长度, 曲率) 一段段接起来；方向变了就断开成新的一段。

    输出每段 (方向, 起点位姿, [(长度, 曲率), …])，位姿和曲率都按行进方向（倒车段 yaw = 车头 + π、曲率反号），
    即 path_tracker.Path 的画法。
    """

    def __init__(self, pose):
        self.pose, self.out = pose, []

    def add(self, direction, length, kappa):
        if length <= 1e-9:
            return
        x, y, yaw = self.pose
        if not self.out or self.out[-1][0] != direction:
            self.out.append((direction, (x, y, wrap(yaw if direction > 0 else yaw + math.pi)), []))
        _append(self.out[-1][2], length, direction * kappa)
        self.pose = advance(x, y, yaw, direction * length, kappa)


# ---- Reeds-Shepp 最短路径（可以倒车）：公式与 OMPL ReedsSheppStateSpace 相同（Reeds & Shepp 1990）----

_RS_ZERO = 1e-10
_HALF_PI = 0.5 * math.pi


def _rs_mod(x):
    """取到 [−π, π]。"""
    v = math.fmod(x, 2.0 * math.pi)
    if v < -math.pi:
        v += 2.0 * math.pi
    elif v > math.pi:
        v -= 2.0 * math.pi
    return v


def _rs_polar(x, y):
    return math.hypot(x, y), math.atan2(y, x)


def _rs_tau_omega(u, v, xi, eta, phi):
    delta = _rs_mod(u - v)
    a = math.sin(u) - math.sin(delta)
    b = math.cos(u) - math.cos(delta) - 1.0
    t1 = math.atan2(eta * a - xi * b, xi * a + eta * b)
    t2 = 2.0 * (math.cos(delta) - math.cos(v) - math.cos(u)) + 3.0
    tau = _rs_mod(t1 + math.pi) if t2 < 0.0 else _rs_mod(t1)
    return tau, _rs_mod(tau - u + v - phi)


def _rs_lsl(x, y, phi):          # 8.1  L+S+L+
    u, t = _rs_polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if t >= -_RS_ZERO:
        v = _rs_mod(phi - t)
        if v >= -_RS_ZERO:
            return t, u, v
    return None


def _rs_lsr(x, y, phi):          # 8.2  L+S+R+
    u1, t1 = _rs_polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    u1 = u1 * u1
    if u1 >= 4.0:
        u = math.sqrt(u1 - 4.0)
        t = _rs_mod(t1 + math.atan2(2.0, u))
        v = _rs_mod(t - phi)
        if t >= -_RS_ZERO and v >= -_RS_ZERO:
            return t, u, v
    return None


def _rs_lrl(x, y, phi):          # 8.3  L+R−L
    u1, theta = _rs_polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if u1 <= 4.0:
        u = -2.0 * math.asin(0.25 * u1)
        t = _rs_mod(theta + 0.5 * u + math.pi)
        v = _rs_mod(phi - t + u)
        if t >= -_RS_ZERO and u <= _RS_ZERO:
            return t, u, v
    return None


def _rs_lrlr_a(x, y, phi):       # 8.7  L+R+L−R−
    xi, eta = x + math.sin(phi), y - 1.0 - math.cos(phi)
    rho = 0.25 * (2.0 + math.sqrt(xi * xi + eta * eta))
    if rho <= 1.0:
        u = math.acos(rho)
        t, v = _rs_tau_omega(u, -u, xi, eta, phi)
        if t >= -_RS_ZERO and v <= _RS_ZERO:
            return t, u, v
    return None


def _rs_lrlr_b(x, y, phi):       # 8.8  L+R−L−R+
    xi, eta = x + math.sin(phi), y - 1.0 - math.cos(phi)
    rho = (20.0 - xi * xi - eta * eta) / 16.0
    if 0.0 <= rho <= 1.0:
        u = -math.acos(rho)
        if u >= -_HALF_PI:
            t, v = _rs_tau_omega(u, u, xi, eta, phi)
            if t >= -_RS_ZERO and v >= -_RS_ZERO:
                return t, u, v
    return None


def _rs_lrsl(x, y, phi):         # 8.9  L+R−S−L−
    rho, theta = _rs_polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if rho >= 2.0:
        r = math.sqrt(rho * rho - 4.0)
        u = 2.0 - r
        t = _rs_mod(theta + math.atan2(r, -2.0))
        v = _rs_mod(phi - _HALF_PI - t)
        if t >= -_RS_ZERO and u <= _RS_ZERO and v <= _RS_ZERO:
            return t, u, v
    return None


def _rs_lrsr(x, y, phi):         # 8.10 L+R−S−R−
    xi, eta = x + math.sin(phi), y - 1.0 - math.cos(phi)
    rho, theta = _rs_polar(-eta, xi)
    if rho >= 2.0:
        u = 2.0 - rho
        v = _rs_mod(theta + _HALF_PI - phi)
        if theta >= -_RS_ZERO and u <= _RS_ZERO and v <= _RS_ZERO:
            return theta, u, v
    return None


def _rs_lrslr(x, y, phi):        # 8.11 L+R−S−L−R+
    xi, eta = x + math.sin(phi), y - 1.0 - math.cos(phi)
    rho, _ = _rs_polar(xi, eta)
    if rho >= 2.0:
        u = 4.0 - math.sqrt(rho * rho - 4.0)
        if u <= _RS_ZERO:
            t = _rs_mod(math.atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta))
            v = _rs_mod(t - phi)
            if t >= -_RS_ZERO and v >= -_RS_ZERO:
                return t, u, v
    return None


# (公式, 字, 反射后的字, 段长排法, 是否用倒序坐标)：每个公式再做时间反转、左右反射、两者都做，共 48 种
_RS_FAMILIES = (
    (_rs_lsl, 'LSL', 'RSR', lambda t, u, v: (t, u, v), False),
    (_rs_lsr, 'LSR', 'RSL', lambda t, u, v: (t, u, v), False),
    (_rs_lrl, 'LRL', 'RLR', lambda t, u, v: (t, u, v), False),
    (_rs_lrl, 'LRL', 'RLR', lambda t, u, v: (v, u, t), True),
    (_rs_lrlr_a, 'LRLR', 'RLRL', lambda t, u, v: (t, u, -u, v), False),
    (_rs_lrlr_b, 'LRLR', 'RLRL', lambda t, u, v: (t, u, u, v), False),
    (_rs_lrsl, 'LRSL', 'RLSR', lambda t, u, v: (t, -_HALF_PI, u, v), False),
    (_rs_lrsr, 'LRSR', 'RLSL', lambda t, u, v: (t, -_HALF_PI, u, v), False),
    (_rs_lrsl, 'LSRL', 'RSLR', lambda t, u, v: (v, u, -_HALF_PI, t), True),
    (_rs_lrsr, 'RSRL', 'LSLR', lambda t, u, v: (v, u, -_HALF_PI, t), True),
    (_rs_lrslr, 'LRSLR', 'RLSRL', lambda t, u, v: (t, -_HALF_PI, u, -_HALF_PI, v), False),
)


def reeds_shepp_paths(start, goal, radius):
    """start → goal（(x, y, 车头)）的全部 Reeds-Shepp 候选，每个都积分验算过终点（公式或数值出错的直接丢掉）。

    每个候选是 [(方向, 长度, 曲率), …]：方向 +1 前进、−1 倒车，曲率按车头（左转为正），零长度段已去掉。
    """
    x0, y0, yaw0 = start
    dx, dy = goal[0] - x0, goal[1] - y0
    c, s = math.cos(yaw0), math.sin(yaw0)
    x, y, phi = (c * dx + s * dy) / radius, (-s * dx + c * dy) / radius, goal[2] - yaw0
    xb, yb = x * math.cos(phi) + y * math.sin(phi), x * math.sin(phi) - y * math.cos(phi)
    kappa = {'L': 1.0 / radius, 'R': -1.0 / radius, 'S': 0.0}
    dist = math.hypot(dx, dy)
    out = []
    for fn, word, reflected, order, back in _RS_FAMILIES:
        px, py = (xb, yb) if back else (x, y)
        for sx, sy, sphi, flip, w in ((1, 1, 1, 1, word), (-1, 1, -1, -1, word),
                                      (1, -1, -1, 1, reflected), (-1, -1, 1, -1, reflected)):
            r = fn(sx * px, sy * py, sphi * phi)
            if r is None:
                continue
            segs = [(1 if flip * a > 0 else -1, abs(a) * radius, kappa[ch])
                    for ch, a in zip(w, order(*r)) if abs(a) * radius > 1e-6]
            px_, py_, pyaw = start
            for d, length, k in segs:
                px_, py_, pyaw = advance(px_, py_, pyaw, d * length, k)
            if math.hypot(px_ - goal[0], py_ - goal[1]) > 1e-6 * max(1.0, dist) or abs(wrap(pyaw - goal[2])) > 1e-6:
                continue
            out.append(segs)
    return out


def _runs(segs):
    """连续同方向的段合成一次行驶：[(方向, 路程), …]。"""
    runs = []
    for d, length, _ in segs:
        if runs and runs[-1][0] == d:
            runs[-1] = (d, runs[-1][1] + length)
        else:
            runs.append((d, length))
    return runs


def reeds_shepp(start, goal, radius, *, end_forward=False):
    """最短的 Reeds-Shepp 路径（一样长时挑换向少的；end_forward 时最后接着前进也算一次换向）。

    两次换向之间太短（< MIN_RUN_M）的不要；都不行返回 None。
    """
    best = None
    for segs in reeds_shepp_paths(start, goal, radius):
        runs = _runs(segs)
        if any(length < MIN_RUN_M for _, length in runs):
            continue
        cusps = len(runs) - 1 + (1 if end_forward and runs and runs[-1][0] < 0 else 0)
        cost = sum(length for _, length, _ in segs) + CUSP_TIE_M * cusps
        if best is None or cost < best[0] - 1e-9:
            best = (cost, segs)
    return None if best is None else best[1]


_CORNER_CACHE = {}


def _corner_move(theta, radius, body, start_x, back_q):
    """急弯掉头补角（拐角在原点，第一条边沿 +x 过来，左转 θ > 0）：(后轴起点 x, 后轴终点离拐角, [(方向, 长度, 曲率), …])。

    后轴从第一条边上 x = start_x（再多往前 e1）出发 → 前进一段，此时整车开出拐角 CORNER_CLEAR_M → 倒车一段 →
    前进开到下一条边上、后轴在拐角后 back_q（再多往后 e2）处对正。只取"前进-倒车-前进"的走法（最后的前进可以
    就是下一条边本身），在里面挑最短的；右转按左右反射。找不到返回 None。
    """
    key = (theta, radius, body, start_x, back_q)
    if key in _CORNER_CACHE:
        return _CORNER_CACHE[key]
    front, rear, half = body
    best = None
    for e2 in CORNER_BACK_M:
        q = back_q + e2
        b = (-q * math.cos(theta), -q * math.sin(theta), theta)
        for e1 in CORNER_EXTRA_M:
            a = (start_x + e1, 0.0, 0.0)
            for segs in reeds_shepp_paths(a, b, radius):
                runs = _runs(segs)
                if [r[0] for r in runs] not in ([1, -1], [1, -1, 1]) or any(r[1] < MIN_RUN_M for r in runs):
                    continue
                x, y, yaw = a
                for dr, length, k in segs:
                    if dr < 0:
                        break
                    x, y, yaw = advance(x, y, yaw, length, k)
                clear = min(x + ax * math.cos(yaw) - ay * math.sin(yaw) for ax in (-rear, front) for ay in (-half, half))
                if clear < CORNER_CLEAR_M - 1e-9:
                    continue
                cost = e1 + e2 + sum(length for _, length, _ in segs)
                if best is None or cost < best[0] - 1e-9:
                    best = (cost, a[0], q, segs)
    _CORNER_CACHE[key] = None if best is None else best[1:]
    return _CORNER_CACHE[key]


def _cover(points, min_radius, body, labels=None):
    """连线折线 → (分段, 掉头补角的拐角数)，不含接入段。第一段从车身中心在第 1 个点处（或更靠后）出发、前进。"""
    pts, lab = [], []
    for p, n in zip(points, labels or range(1, len(points) + 1)):
        if not pts or math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) >= 1e-3:
            pts.append((float(p[0]), float(p[1])))
            lab.append(n)
    if len(pts) < 2:
        raise ValueError('先在地图上点点，再把点连起来')
    r, n, pas = min_radius, len(pts), VERTEX_PASS_M
    d = 0.5 * (body[0] - body[1])
    legs = [(math.hypot(b[0] - a[0], b[1] - a[1]), math.atan2(b[1] - a[1], b[0] - a[0])) for a, b in zip(pts, pts[1:])]
    turn = [0.0] * n
    for j in range(1, n - 1):
        turn[j] = wrap(legs[j][1] - legs[j - 1][1])
    tangent = [r * math.tan(abs(a) / 2.0) for a in turn]
    smooth = {j for j in range(1, n - 1) if r * (1.0 / math.cos(turn[j] / 2.0) - 1.0) <= COVER_TOL_M + 1e-12}
    gentle = set(smooth)
    # 相邻两个缓弯的圆弧切点要在同一条边上放得下，放不下时拐得急的那头改掉头补角
    changed = True
    while changed:
        changed = False
        for i, (length, _) in enumerate(legs):
            a, b = i, i + 1
            if a in smooth and b in smooth and tangent[a] + tangent[b] > length + 1e-9:
                smooth.discard(a if abs(turn[a]) >= abs(turn[b]) else b)
                changed = True
    # 后轴在每条边上直线的起点、终点（相对这条边两头的点，沿边方向）。车身中心在后轴前 d：
    # 起点后轴在第 1 个点后 d；掉头前车身中心过拐角 pas、掉头后从拐角后 pas 开始；终点车身中心过终点 pas。
    # 边太短、挨着的缓弯圆弧占了地方时，直线就从圆弧切点开始 / 到圆弧切点为止，车身中心多开一点，照样压得到。
    first = max(d, tangent[1] - legs[0][0]) if 1 in smooth else d
    moves = {}
    for j in range(1, n - 1):
        if j in smooth:
            continue
        start_x = pas - d
        if j - 1 in smooth:
            start_x = max(start_x, tangent[j - 1] - legs[j - 1][0])
        back_q = d + pas
        if j + 1 in smooth:
            back_q = max(back_q, tangent[j + 1] - legs[j][0])
        move = _corner_move(abs(turn[j]), r, body, start_x, back_q)
        if move is None:
            if j in gentle:
                raise ValueError('第 %d 个点附近的弯比车的最小转弯半径 %.2f m 还急，点又太密，车压不全：'
                                 '把这段画缓些，或把点距拉开' % (lab[j], r))
            raise ValueError('第 %d 个点拐 %.0f°，算不出掉头走法' % (lab[j], math.degrees(abs(turn[j]))))
        moves[j] = move
    h0 = legs[0][1]
    out = _Sections(_offset(pts[0], h0, -first) + (h0,))
    for i, (length, h) in enumerate(legs):
        j = i + 1
        a = -first if i == 0 else (tangent[i] if i in smooth else -moves[i][1])
        if j == n - 1:
            out.add(1, max(0.0, length - a + pas - d), 0.0)
            break
        out.add(1, length - a + (-tangent[j] if j in smooth else moves[j][0]), 0.0)
        if j in smooth:
            out.add(1, r * abs(turn[j]), math.copysign(1.0 / r, turn[j]))
            continue
        side = 1.0 if turn[j] > 0 else -1.0
        for dr, seg, k in moves[j][2]:
            out.add(dr, seg, side * k)
    return out.out, len(moves)


def cover_route(points, min_radius, body, *, labels=None):
    """连线折线 → 车身中心压过全部连线的分段路径（不含接入段）：[(方向, (x, y, yaw), [(长度, 曲率), …]), …]。

    方向 +1 前进、−1 倒车；yaw 和曲率都按行进方向（倒车段 yaw = 车头 + π）。第一段从后轴在第 1 个点后方
    (front−rear)/2 处出发（车身中心正好在第 1 个点）。body = (front_m, rear_m, half_width_m)，后轴中心到车头、
    车尾、半宽。labels 是每个点在 app 里的编号（闭合多圈时点会重复），只用于报错。画不成抛 ValueError。
    """
    return _cover(points, min_radius, body, labels)[0]


def _route_sections(spec, min_radius, body):
    n = len(spec.points)
    pts = path_points(spec)
    return _cover(pts, min_radius, body, [i % n + 1 for i in range(len(pts))])


def cover_entry(first, car, min_radius):
    """车当前位姿（后轴中心）→ 路线第一段（first，前进）的接入：(接入后的分段, 接入路程米)。

    车已压在第一段延长线上、在起点后方：从车在线上的垂足直线开过去。否则用 Reeds-Shepp 最短路径（可以倒车）
    开到第一段起点后方 ENTRY_LEAD_M 处对正，再直线开进来。
    """
    _, (x0, y0, h0), pieces = first
    x, y, yaw = car
    along = (x - x0) * math.cos(h0) + (y - y0) * math.sin(h0)
    side = -(x - x0) * math.sin(h0) + (y - y0) * math.cos(h0)
    if along <= 1e-9 and abs(side) <= ON_LINE_M and abs(wrap(yaw - h0)) <= math.radians(ON_LINE_DEG):
        out = _Sections(_offset((x0, y0), h0, along) + (h0,))
        out.add(1, -along, 0.0)
        for length, k in pieces:
            out.add(1, length, k)
        return out.out, -along
    goal = _offset((x0, y0), h0, -ENTRY_LEAD_M) + (h0,)
    way = reeds_shepp(car, goal, min_radius, end_forward=True)
    if way is None:
        way = [(1, length, k) for length, k in dubins(car, goal, min_radius)]
    out = _Sections(car)
    for d, length, k in way:
        out.add(d, length, k)
    out.add(1, ENTRY_LEAD_M, 0.0)
    for length, k in pieces:
        out.add(1, length, k)
    return out.out, sum(length for _, length, _ in way) + ENTRY_LEAD_M


def cover_summary(spec, min_radius, body, *, nozzle_x=None):
    """连线路线（不含接入段）要开的总路程、换向次数、掉头补角的拐角数；画不成返回 None。

    nozzle_x 不为 None 是差速车（见 pivot_cover）：corners 是急弯数，另报 pivots（原地转向次数；路线选了圆弧掉头时
    急弯不原地转，为 0）。
    """
    try:
        if nozzle_x is not None:
            sections, corners = _pivot_route_sections(spec, min_radius, nozzle_x)
        else:
            sections, corners = _route_sections(spec, min_radius, body)
    except ValueError:
        return None
    out = dict(drive_m=round(sum(length for _, _, pieces in sections for length, _ in pieces), 3),
               cusps=sum(1 for a, b in zip(sections, sections[1:]) if a[0] != b[0]) if nozzle_x is not None
               else len(sections) - 1, corners=corners)
    if nozzle_x is not None:
        out['pivots'] = pivot_count(sections)
    return out


# ---- 差速车（Gen5：两后轮差速、前面自由万向轮）：急弯停车原地转向 ----
# 与 app 的 PivotRoute.kt 同一套规则（app 本机仿真按它画），两边的一致性由 app 端 PivotParityTest 检查。

PIVOT_ARC_TOL_M = 0.01       # 喷头在两后轮中间时：缓弯按缓弯半径圆弧转过去离拐角不超过这么远才直接转，否则停车原地转
PIVOT_TOL_RAD = math.radians(1.0)   # 前后两段车头方向差这么多以上才算原地转向（与 SectionTracker 的 pivot_tol 相同）
# 急弯原地转向后、起步倒车助跑时，后轴至少退到拐角 / 起点后方这么远，再往前回到拐角 / 起点（喷头在它后方
# 0.9 m 开始助跑：先慢走 0.5 m 让万向轮翻回来，见 path_tracker reverse_creep_m，再加速）；喷头位置仍按实际杆臂算。
# 2026-10-01 用户：倒退 0.5 m 改 0.8 m，倒完慢速前进让万向轮回正（实车倒 0.5 m 后喷头只剩 0.2 m 助跑，
# 倒车留下的 2–4 cm 横偏带进线头，开画后 0.2–0.3 m 才收回）。
# 2026-10-01 下午 0.8 → 1.2 m：0.8 m 时慢走正好占满助跑，喷头到线头时车速中位才 0.25 m/s，0.15 → 0.8 m/s 的加速
# 全在线上（加速时两轮读回转速差来回 ±50–150 erpm，车头甩 2–3°），原地转留下的 ~1 cm 横偏也没纠完；
# 14 趟起步喷头过线头 0.5 m 内最大偏差中位 1.8 cm、5 趟超过 2 cm。1.2 m 时慢走 0.5 m 后还有 0.4 m 加速，
# 喷头到线头约 0.65 m/s；每次起步 / 急弯多约 2 s（用户选 1.2 m；1.5 m 能在线外加到 0.8 m/s，多约 3 s）
PIVOT_REVERSE_MIN_M = 1.20  # 旧后轴距离，兼容历史配置；新助跑按喷头到线头计。
# 2026-10-10 用户：助跑 2.5 → 2.8（先说加 0.5 m，后定 2.8 m）。10-10 实录 10 条直线起步摆动要走 1.3–2 m 才收住，
# 7 条到线头还在纠，开喷时车头差 0.6–2.4°；起步头 1 m 先稳再纠偏见 path_tracker launch_settle_*
RUN_UP_M = 2.80
# 喷头在后轴前时后轴被喷头拖着走（见 _pulled_cover）：每片喷头走不超过 nozzle_x × PULL_STEP，片内按等曲率接
# （片端车头方向准确，片内误差约 (PULL_STEP·d)²·φ/(12d)，d 0.30 m、拐 28° 约 0.8 mm）；车头与边的夹角
# 收敛到 PULL_SETTLE_RAD 后用一段圆弧把后轴正好收回到边上（收回时喷头最多偏 PULL_SETTLE_RAD·d/4，约 0.3 mm）
PULL_STEP = 0.25
PULL_SETTLE_RAD = math.radians(0.25)
PULL_STRAIGHT_RAD = 1e-6     # 夹角小于这么多就当已经对齐，直接走直线
# 圆弧掉头（turn='arc'，2026-10-02 用户：前面要掉头时不原地转，像阿克曼底盘一样拐弯掉头、丝滑接入；原地转保留，app 可选）：
# 喷头开过拐角 ARC_RUN_OUT_M 后按 turn_radius 绕 Dubins 最短路径（只往前），接到下一条边的延长线上、喷头在拐角前
# ARC_RUN_IN_M 处对正，再直线开到拐角接着压线。整条路线一段到底，不停车、不倒车。
# 助跑与原地转的倒车助跑同一个起点（后轴在拐角后 nozzle_x + 0.9 = 1.2 m）：出圆弧时曲率从 1/R 跳到 0，
# 执行延迟带来的车头误差在喷头到拐角前收掉
ARC_RUN_OUT_M = 0.2
ARC_RUN_IN_M = PIVOT_REVERSE_MIN_M - 0.30


class _PaintPieces(list):
    """保持几何三元组接口，同时记录段内喷线区间。"""
    def __init__(self):
        super().__init__()
        self.paint_spans = []


class _PivotSections:
    """按车头坐标接起来；方向变了或原地转过向就断开成新的一段（行进方向坐标，与 _Sections 相同）。

    原地转向不单独成段：前后两段起点车头方向不同，跟踪器在两段之间原地转过去。
    """

    def __init__(self, pose):
        self.pose, self.out, self._split = pose, [], True

    def pivot(self, turn):
        x, y, yaw = self.pose
        self.pose, self._split = (x, y, wrap(yaw + turn)), True

    def add(self, direction, length, kappa, *, paint=False):
        if length <= 1e-9:
            return
        x, y, yaw = self.pose
        if self._split or self.out[-1][0] != direction:
            self.out.append((direction, (x, y, wrap(yaw if direction > 0 else yaw + math.pi)), _PaintPieces()))
            self._split = False
        pieces = self.out[-1][2]
        start = sum(v for v, _ in pieces)
        if paint:
            if pieces.paint_spans and abs(pieces.paint_spans[-1][1]-start)<1e-8:
                pieces.paint_spans[-1] = (pieces.paint_spans[-1][0], start+length)
            else:
                pieces.paint_spans.append((start, start+length))
        _append(pieces, length, direction * kappa)
        self.pose = advance(x, y, yaw, direction * length, kappa)


def _paint_run_out(out):
    """停喷后缓停：直线开出 40 cm，曲线沿末端曲率继续 50 cm；不延长喷涂范围。"""
    if not out.out:
        return
    kappa = out.out[-1][2][-1][1]
    straight = abs(kappa) < 1e-9
    out.add(1, LINE_RUN_OUT_M if straight else CIRCLE_RUN_OUT_M,
            0.0 if straight else kappa)


def _heading(section, end=False):
    """一段起点（end=True 为终点）的车头方向。"""
    d, start, pieces = section
    x, y, yaw = start
    if end:
        for length, k in pieces:
            x, y, yaw = advance(x, y, yaw, length, k)
    return wrap(yaw if d > 0 else yaw + math.pi)


def pivot_count(sections):
    """相邻两段之间要原地转向的次数。"""
    return sum(1 for a, b in zip(sections, sections[1:]) if abs(wrap(_heading(b) - _heading(a, True))) > PIVOT_TOL_RAD)


def _arc_turn(out, vertex, heading, d, arc_radius):
    """圆弧掉头（接在 out 后面，后轴中点此时在喷头正好压到拐角 vertex 的位置）：喷头再往前开 ARC_RUN_OUT_M →
    半径 arc_radius 的 Dubins 最短路径（只往前）→ 后轴在下一条边（方向 heading）延长线上、喷头在拐角前 ARC_RUN_IN_M
    处对正 → 直线开到喷头正好压到拐角。"""
    out.add(1, ARC_RUN_OUT_M, 0.0)
    goal = _offset(vertex, heading, -(d + ARC_RUN_IN_M)) + (heading,)
    for length, kappa in dubins(out.pose, goal, arc_radius):
        out.add(1, length, kappa)
    # 终点按 Dubins 验算过（1e-6 以内）；对齐到准确的助跑起点，喷头正好开到拐角，下一条边从这里量
    out.pose = goal
    out.add(1, ARC_RUN_IN_M, 0.0)


def _pivot_cover(points, radius, nozzle_x, arc_radius=None):
    """连线折线 → 差速车分段（不含接入段）和急弯数。压线点（喷头）在后轴中点前 nozzle_x。

    喷头在后轴前（nozzle_x > 0）：喷头沿连线走、后轴被拖着走，缓弯不停车，急弯原地转向，见 _pulled_cover。
    喷头在两后轮中间（nozzle_x = 0）：缓弯（转向小于 90°、按 radius 圆弧转过去离拐角不超过 PIVOT_ARC_TOL_M）
    直接圆弧转过去；急弯后轴中点开到拐角点停车，原地转到下一条边的方向。
    arc_radius 不为 None（圆弧掉头）：急弯不停车、不原地转，按 arc_radius 绕过去（_arc_turn），缓弯照旧。
    第一段从喷头在第 1 个点处出发、前进，线尾停喷后缓停：直线 40 cm、曲线 50 cm。
    """
    if not (math.isfinite(radius) and radius > 0):
        raise ValueError('缓弯半径无效')
    nozzle_x, side = nozzle_point(nozzle_x)
    if not (nozzle_x >= 0 or side):
        raise ValueError('喷头位置无效')
    if arc_radius is not None and not (math.isfinite(arc_radius) and arc_radius >= radius - 1e-9):
        raise ValueError('圆弧掉头半径 %.2f m 小于车能稳定跟踪的 %.2f m' % (arc_radius, radius))
    pts = []
    for p in points:
        if not pts or math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) >= 1e-3:
            pts.append((float(p[0]), float(p[1])))
    if len(pts) < 2:
        raise ValueError('先在地图上点点，再把点连起来')
    if abs(side) > 1e-9:
        return _side_cover(pts, nozzle_x, side, arc_radius, radius)
    if nozzle_x > 0:
        return _pulled_cover(pts, radius, nozzle_x, arc_radius)
    n, d, pas = len(pts), nozzle_x, VERTEX_PASS_M
    legs = [(math.hypot(b[0] - a[0], b[1] - a[1]), math.atan2(b[1] - a[1], b[0] - a[0])) for a, b in zip(pts, pts[1:])]
    turn = [0.0] * n
    for j in range(1, n - 1):
        turn[j] = wrap(legs[j][1] - legs[j - 1][1])
    tangent = [radius * math.tan(abs(a) / 2.0) for a in turn]
    smooth = {j for j in range(1, n - 1)
              if abs(turn[j]) < math.pi / 2 and radius * (1.0 / math.cos(turn[j] / 2.0) - 1.0) <= PIVOT_ARC_TOL_M + 1e-12}

    def start(i):           # 后轴中点在第 i 条边上直线的起点（从这条边起点量）
        return tangent[i] if i > 0 and i in smooth else -d

    def end(i):
        if i + 1 == n - 1:
            return legs[i][0] - d + END_PASS_M
        return legs[i][0] - tangent[i + 1] if i + 1 in smooth else legs[i][0]
    # 边太短放不下圆弧的切点：拐得急的那头改原地转向
    changed = True
    while changed:
        changed = False
        for i in range(len(legs)):
            if end(i) < start(i) - 1e-9:
                both = [j for j in (i, i + 1) if j in smooth]
                smooth.discard(max(both, key=lambda j: abs(turn[j])))
                changed = True
    h0 = legs[0][1]
    out = _PivotSections(_offset(pts[0], h0, -d) + (h0,))
    for i in range(len(legs)):
        out.add(1, end(i) - start(i), 0.0, paint=True)
        j = i + 1
        if j == n - 1:
            _paint_run_out(out)
            break
        if j in smooth:
            out.add(1, radius * abs(turn[j]), math.copysign(1.0 / radius, turn[j]), paint=True)
            continue
        if arc_radius is not None:
            _arc_turn(out, pts[j], legs[j][1], d, arc_radius)
            continue
        _paint_run_out(out)
        run = max(d, PIVOT_REVERSE_MIN_M)-d
        _transit(out, _offset(pts[j], legs[j][1], -(d+run)), legs[j][1])
        out.add(1, run, 0.0)
    return out.out, n - 2 - len(smooth)


def _pulled_cover(pts, radius, d, arc_radius=None):
    """喷头在后轴前 d（> 0）：喷头沿连线走，后轴像拖车一样被喷头拖着走（曳物线），喷头始终压在线上。

    车头与下一条边的夹角 φ 决定后轴要转多急：后轴转弯半径 d / tan|φ|。|φ| ≤ atan(d / radius)（Gen5 d 0.30、
    radius 0.556 约 28°）的拐角不停车拖过去；更急的停车原地转向：后轴中点开到拐角点（喷头开过拐角 d），
    原地转到下一条边的方向，倒车 max(d, PIVOT_REVERSE_MIN_M)，再往前回到拐角点继续压线。
    arc_radius 不为 None（圆弧掉头）：更急的拐角不停车，喷头开过拐角后按 arc_radius 绕过去（_arc_turn）。
    每条边都从已接好的路径上的喷头位置量起（朝这条边的终点），片内近似的误差不往后面的边累积。
    """
    n, limit = len(pts), math.atan(d / radius)
    h0 = math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0])
    out = _PivotSections(_offset(pts[0], h0, -d) + (h0,))
    corners = 0
    for i in range(1, n):
        x, y, yaw = out.pose
        nx, ny = x + d * math.cos(yaw), y + d * math.sin(yaw)
        head = math.atan2(pts[i][1] - ny, pts[i][0] - nx)
        if i > 1 and abs(wrap(yaw - head)) > limit + 1e-12:
            corners += 1
            if arc_radius is not None:
                # 圆弧掉头：喷头从拐角点（图纸上的点，不是拖出来的近似位置）沿这条边对正开过来，接着直线压这条边
                v = pts[i - 1]
                head = math.atan2(pts[i][1] - v[1], pts[i][0] - v[0])
                _arc_turn(out, v, head, d, arc_radius)
            else:
                # 急弯：停喷开出后，从开出终点转场到下一边助跑起点。
                _paint_run_out(out)
                vertex = pts[i-1]
                head = math.atan2(pts[i][1]-vertex[1], pts[i][0]-vertex[0])
                run = max(d, PIVOT_REVERSE_MIN_M)-d
                _transit(out, _offset(vertex, head, -(d+run)), head)
                out.add(1, run, 0.0)
            x, y, yaw = out.pose
            nx, ny = x + d * math.cos(yaw), y + d * math.sin(yaw)
        dist = math.hypot(pts[i][0] - nx, pts[i][1] - ny) + (END_PASS_M if i == n - 1 else 0.0)
        _pull(out, wrap(yaw - head), dist, d)
        if i == n-1:
            _paint_run_out(out)
    return out.out, corners


class _Pieces:
    """只记 (长度, 曲率)，给 _pull 当输出用（_side_pull 再换算到真实后轴）。"""

    def __init__(self):
        self.pieces = []

    def add(self, direction, length, kappa, *, paint=False):
        if length > 1e-12:
            self.pieces.append((length, kappa))


def _side_gentle(phi, f, l, radius):
    """喷头在侧面（前 f、左 l）、车头比下一条边偏左 phi 时能不能不停车拖过去：拖的起点后轴曲率最大
    （虚拟后轴 tan|φ|/f，换到真实后轴 κ / (1 + lκ)），不急过车能稳定跟踪的 radius。往喷头那侧拐时真实后轴在外圈、
    更缓；往另一侧拐时在内圈、更急（Gen5 喷头在右 0.41 m：往右可拖约 58°，往左约 14°）。"""
    if abs(phi) >= math.pi / 2:
        return False
    k = -math.copysign(math.tan(abs(phi)) / f, phi)
    factor = 1.0 + l * k
    return factor > 0.05 and abs(k) / factor <= 1.0 / radius + 1e-9


def _side_pull(out, phi, dist, f, l):
    """喷头在侧面（前 f、左 l）时沿直线走 dist、车头比这条线偏左 phi 时后轴被拖出的路径，接在 out 后面。

    后轴往左平移 l 得到虚拟后轴，喷头正好在它正前方 f，按中线喷头拖（_pull）；真实后轴 = 虚拟后轴 − l·n，
    车头相同，路程 ×(1 + lκ)、曲率 κ / (1 + lκ)（κ 为虚拟后轴曲率），是精确换算。"""
    rec = _Pieces()
    _pull(rec, phi, dist, f)
    for length, kappa in rec.pieces:
        factor = 1.0 + l * kappa
        out.add(1, length * factor, kappa / factor, paint=True)


def _side_cover(pts, f, l, arc_radius=None, radius=None):
    """喷头在侧面（车体系前 f、左 l，l ≠ 0）：喷头沿连线走，喷头正好压在边上。

    缓弯（_side_gentle：后轴拖着走不急过 radius）不停车：后轴像拖车一样被喷头拖着走（_side_pull）。
    2026-10-03 实车：app 的两点弧线（R 5.7 m）拆成 25 个点、每点拐 4.4°，原来每个点都当急弯，倒车 + 原地转 24 次。
    急弯不按缓弯圆弧转（喷头不在中线上，原地转时喷头绕后轴画圆离开拐角）：喷头压到拐角后停车，
    原地转 → 直线挪（前进或倒车，转得少的那个）→ 原地转到下一条边的助跑起点（后轴在「喷头压拐角」处后方
    max(f, PIVOT_REVERSE_MIN_M) − f，与中线喷头的倒车助跑一样长），往前助跑，喷头到拐角接着压下一条边。
    arc_radius 不为 None（圆弧掉头）：急弯时喷头开过拐角 ARC_RUN_OUT_M 后按 arc_radius 绕 Dubins（只往前）到下一条边，
    喷头在拐角前 ARC_RUN_IN_M 对正（同 _arc_turn）。共线的路点直接接着走。线尾停喷后缓停：直线 40 cm、曲线 50 cm。
    每条边都从已接好的路径上的喷头位置量起（朝这条边的终点），拖出来的近似误差不往后面的边累积。
    返回 (分段, 急弯数)。
    """
    n = len(pts)
    run = max(f, PIVOT_REVERSE_MIN_M) - f
    h0 = math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0])
    out = _PivotSections(axle_for(pts[0], h0, f, l) + (h0,))
    corners = 0
    for i in range(1, n):
        x, y, yaw = out.pose
        nx, ny = nozzle_at(x, y, yaw, f, l)
        head = math.atan2(pts[i][1] - ny, pts[i][0] - nx)
        phi = wrap(yaw - head)
        if i > 1 and abs(phi) > PULL_STRAIGHT_RAD and (radius is None or not _side_gentle(phi, f, l, radius)):
            corners += 1
            # 急弯：喷头此时正压在拐角（图纸上的点）上；按图纸上这条边的方向接
            vertex = pts[i - 1]
            nxt = math.atan2(pts[i][1] - vertex[1], pts[i][0] - vertex[0])
            if arc_radius is not None:
                out.add(1, ARC_RUN_OUT_M, 0.0)
                goal = _offset(axle_for(vertex, nxt, f, l), nxt, -ARC_RUN_IN_M) + (nxt,)
                for piece, kappa in dubins(out.pose, goal, arc_radius):
                    out.add(1, piece, kappa)
                out.pose = goal
                out.add(1, ARC_RUN_IN_M, 0.0)
            else:
                _paint_run_out(out)
                _transit(out, _offset(axle_for(vertex, nxt, f, l), nxt, -run), nxt)
                out.add(1, run, 0.0)
            x, y, yaw = out.pose
            nx, ny = nozzle_at(x, y, yaw, f, l)
            head = math.atan2(pts[i][1] - ny, pts[i][0] - nx)
            phi = wrap(yaw - head)
        dist = math.hypot(pts[i][0] - nx, pts[i][1] - ny) + (END_PASS_M if i == n - 1 else 0.0)
        _side_pull(out, phi, dist, f, l)
        if i == n-1:
            _paint_run_out(out)
    return out.out, corners


def _transit(out, goal, heading):
    """后轴从 out.pose 挪到 goal、车头转到 heading：原地转 → 直线（前进或倒车，两次原地转加起来转得少的那个）
    → 原地转。离 goal 不到 ON_LINE_M 就不开直线（由跟踪器当横偏纠正，路线本身仍从 goal 量）。"""
    x, y, yaw = out.pose
    dist = math.hypot(goal[0] - x, goal[1] - y)
    if dist >= ON_LINE_M:
        to = math.atan2(goal[1] - y, goal[0] - x)
        forward = abs(wrap(to - yaw)) + abs(wrap(heading - to))
        backward = abs(wrap(to + math.pi - yaw)) + abs(wrap(heading - to - math.pi))
        if backward < forward - 1e-9:
            out.pivot(wrap(to + math.pi - yaw))
            out.add(-1, dist, 0.0)
        else:
            out.pivot(wrap(to - yaw))
            out.add(1, dist, 0.0)
    out.pose = (goal[0], goal[1], out.pose[2])
    out.pivot(wrap(heading - out.pose[2]))


def _pull(out, phi, dist, d):
    """喷头沿直线走 dist 米、车头比这条线偏左 phi 时后轴被拖出的路径，接在 out 后面。

    曳物线：tan(φ/2) = tan(φ0/2)·e^(−s/d)（s 是喷头走过的路程），后轴路程 s − d·ln((1+t0²)/(1+t²))，
    后轴曲率 −tan φ / d。夹角收敛到 PULL_SETTLE_RAD 后，用一段圆弧把后轴正好收回到线上、车头对齐，再走直线；
    边太短来不及收回就一直拖到这条边的终点。
    """
    if abs(phi) < PULL_STRAIGHT_RAD:
        out.add(1, dist, 0.0, paint=True)
        return
    settle = min(abs(phi), PULL_SETTLE_RAD)
    half = math.tan(settle / 2.0)
    s_c = d * math.log(math.tan(abs(phi) / 2.0) / half)
    # 收回圆弧：后轴离线 d·sin(settle)、车头偏 settle，半径 d / tan(settle/2) 的圆弧转 settle 正好回到线上，
    # 这期间喷头沿线正好走 2d
    if s_c + 2.0 * d > dist:
        _tractrix(out, phi, dist, d)
        return
    _tractrix(out, phi, s_c, d)
    out.add(1, d * settle / half, -math.copysign(half / d, phi), paint=True)
    out.add(1, dist - s_c - 2.0 * d, 0.0, paint=True)


def _tractrix(out, phi0, dist, d):
    """喷头沿直线走 dist 米，后轴被拖着走的那段曳物线，按每片喷头走不超过 PULL_STEP·d 分成等曲率片。"""
    if dist <= 0.0:
        return
    t0 = math.tan(phi0 / 2.0)
    k = max(1, math.ceil(dist / (PULL_STEP * d) - 1e-9))
    prev_phi, prev_sa = phi0, 0.0
    for j in range(1, k + 1):
        s = dist * j / k
        t = t0 * math.exp(-s / d)
        phi = 2.0 * math.atan(t)
        sa = s - d * math.log((1.0 + t0 * t0) / (1.0 + t * t))
        if sa - prev_sa > 1e-12:
            out.add(1, sa - prev_sa, (phi - prev_phi) / (sa - prev_sa), paint=True)
            prev_phi, prev_sa = phi, sa


def pivot_cover(points, radius, nozzle_x, arc_radius=None):
    """差速车连线折线 → 喷头压过全部连线的分段路径（不含接入段），格式同 cover_route。
    arc_radius 不为 None 时急弯圆弧掉头（见 _arc_turn），否则原地转向。"""
    return _pivot_cover(points, radius, nozzle_x, arc_radius)[0]


def _pivot_route_sections(spec, radius, nozzle_x):
    """差速车连线路线的分段和急弯数；路线选了圆弧掉头（turn='arc'）按它的 turn_radius 绕。"""
    return _pivot_cover(path_points(spec), radius, nozzle_x, spec.turn_radius if spec.turn == 'arc' else None)


def _via_runup(first, car, direction):
    """车开到助跑起点 G（第一段起点沿反方向退 RUN_UP_M）：direction 1 车头对着 G 开过去，-1 车尾对着 G 倒过去；
    到 G 后原地转到第一段方向，往前助跑 RUN_UP_M 接第一段（倒车后的助跑起步锁车头更长，见 path_tracker.REVERSE_HOLD_M）。
    车离 G 不到 ON_LINE_M 就不开这段，由跟踪器当横偏纠正（路线本身不跟着车偏）。返回 (分段, 接入路程米)。"""
    _, (x0, y0, h0), pieces = first
    x, y, yaw = car
    run = RUN_UP_M
    gx, gy = _offset((x0, y0), h0, -run)
    dist = math.hypot(gx - x, gy - y)
    out = _PivotSections((x, y, yaw))
    moved = 0.0
    if dist >= ON_LINE_M:
        to_g = math.atan2(gy - y, gx - x)
        out.pivot(wrap(to_g + (0.0 if direction > 0 else math.pi) - yaw))
        out.add(direction, dist, 0.0)
        moved = dist
    out.pose = (gx, gy, out.pose[2])
    out.pivot(wrap(h0 - out.pose[2]))
    out.add(1, run, 0.0)
    for length, k in pieces:
        out.add(1, length, k)
    return out.out, moved + run


def entry_seconds(sections, car_yaw):
    """接入分段（_PivotSections 的输出，最后一段是助跑、不含路线本身）从车头 car_yaw 起大概要多久（秒），
    只用来在几种走法里挑；不是速度指令。"""
    t, yaw = 0.0, car_yaw
    for i, (d, (_, _, h), pieces) in enumerate(sections):
        head = h if d > 0 else wrap(h + math.pi)
        turn = abs(wrap(head - yaw))
        if turn > ENTRY_PIVOT_MIN_RAD:
            t += turn / ENTRY_PIVOT_RATE + ENTRY_PIVOT_S
        length = sum(v for v, _ in pieces)
        if d < 0:
            t += length / ENTRY_REVERSE_MPS + ENTRY_REVERSE_SECTION_S
        else:
            slow = min(length, RUN_UP_M) if i == len(sections) - 1 else 0.0
            t += slow / ENTRY_RUNUP_MPS + (length - slow) / ENTRY_FORWARD_MPS + ENTRY_SECTION_S
        yaw = wrap(head + sum(v * k for v, k in pieces))
    return t


def _turned(sections, car_yaw):
    """接入分段从车头 car_yaw 起一共要原地转多少（弧度，只算 >5° 的，跟踪器 pivot_min 以内不转）。"""
    total, yaw = 0.0, car_yaw
    for d, (_, _, h), pieces in sections:
        head = h if d > 0 else wrap(h + math.pi)
        turn = abs(wrap(head - yaw))
        total += turn if turn > ENTRY_PIVOT_MIN_RAD else 0.0
        yaw = wrap(head + sum(v * k for v, k in pieces))
    return total


def pivot_entry(first, car, nozzle_x=0.0):
    """差速车当前位姿（后轴中点）→ 路线第一段的接入：(接入后的分段, 接入路程米)。

    车已在第一段延长线上（横偏 ON_LINE_M 以内）、在助跑起点后方：车头不对先原地转正，直线开过去，中间不停。
    喷头离起点不到 NEAR_START_M：倒车助跑（runup_entry）。否则开到第一段起点后方 RUN_UP_M 处（助跑起点 G）、
    原地转到第一段方向，再直线开进来：车尾对着 G 倒过去比车头对着 G 开过去少原地转 REVERSE_ENTRY_TURN_SAVE_RAD 以上、
    且倒不超过 REVERSE_ENTRY_MAX_M 就倒（2026-10-07 用户：车在点后面时直接往后倒一点就行，不要原地转一圈再转回来）。
    """
    _, (x0, y0, h0), pieces = first
    x, y, yaw = car
    along = (x - x0) * math.cos(h0) + (y - y0) * math.sin(h0)
    side = -(x - x0) * math.sin(h0) + (y - y0) * math.cos(h0)
    if along <= -RUN_UP_M + 1e-9 and abs(side) <= ON_LINE_M:
        out = _PivotSections(_offset((x0, y0), h0, along) + (h0,))
        out.add(1, -along, 0.0)
        for length, k in pieces:
            out.add(1, length, k)
        return out.out, -along
    if along > -RUN_UP_M and math.hypot(along,side) <= NEAR_START_M:
        return runup_entry(first, car, nozzle_x)
    bare = (first[0], first[1], [])
    back = _via_runup(bare, car, -1)[0]
    if sum(sum(v for v, _ in p) for d, _, p in back if d < 0) <= REVERSE_ENTRY_MAX_M + 1e-9 and \
            _turned(back, yaw) + REVERSE_ENTRY_TURN_SAVE_RAD <= _turned(_via_runup(bare, car, 1)[0], yaw) + 1e-9:
        return _via_runup(first, car, -1)
    return _via_runup(first, car, 1)


def runup_entry(first, car, nozzle_x):
    """差速车喷头已在路线起点附近（NEAR_START_M 以内）→ 第一段的接入：(接入后的分段, 接入路程米)。

    助跑起点 G = 第一段后轴起点沿反方向退 RUN_UP_M；此时喷头距实际线头正好 2.5 m，车头朝
    第一段方向。车在 G 前方（顺第一段方向量）就原地转到车尾对着 G、倒过去；
    在 G 后方就转过去开过去。到 G 后转正，往前按助跑速度对正，控制器在线头前预留加速距离；喷头到起点开始画。
    G 定在线上：车离线一点（标点、停车对不准）由倒车这段带回线上，不留给画线时纠正。
    车已压在第一段延长线上、离起点至少助跑那么远：直接开过去（同 pivot_entry）。
    """
    _, (x0, y0, h0), pieces = first
    run = RUN_UP_M  # 喷头到线头的真实助跑距离，不再减去喷头前伸量。
    x, y, yaw = car
    along = (x - x0) * math.cos(h0) + (y - y0) * math.sin(h0)
    side = -(x - x0) * math.sin(h0) + (y - y0) * math.cos(h0)
    if along <= -run + 1e-9 and abs(side) <= ON_LINE_M and abs(wrap(yaw - h0)) <= math.radians(ON_LINE_DEG):
        return pivot_entry(first, car, nozzle_x)
    return _via_runup(first, car, -1 if along + run > 0.0 else 1)

# ---- 路线本身（不含接入段）----

def loop_pieces(spec, min_radius, nozzle_x=None):
    """返回 (起点位姿, [(长度, 曲率), …])；画不成（路点不够、拐角太急、半径太小）时抛 ValueError（中文说明）。

    nozzle_x 不为 None（差速车）时圆按喷头画：返回后轴中点要走的圆（见模块说明）；不给时是图纸上的圆本身
    （车载屏幕、续画按图纸画用）。"""
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

        def start(r):
            if spec.start_angle is None:            # 最南点
                return cx, cy - r, math.pi if spec.clockwise else 0.0
            a = spec.start_angle
            return cx + r * math.cos(a), cy + r * math.sin(a), wrap(a + sign * math.pi / 2)
        if nozzle_x is not None and _offset_nozzle(nozzle_x):
            r = circle_axle_radius(spec.radius, nozzle_x, min_radius, sign)
            return start(r), [(2.0 * math.pi * r * spec.laps + VERTEX_PASS_M, sign / r)]
        return start(spec.radius), [(2.0 * math.pi * spec.radius * spec.laps, sign / spec.radius)]
    return _points_loop(spec)


def _offset_nozzle(nozzle):
    """喷头不在后轴中点（前向 > 0 或在侧面）：圆要按喷头算后轴的圆。"""
    f, l = nozzle_point(nozzle)
    return f > 0 or abs(l) > 1e-9


def circle_axle_radius(radius, nozzle_x, min_radius, sign=1.0):
    """喷头（后轴中点前 f、左 l）画半径 radius 的圆时，后轴中点要走的圆半径 √(R²−f²) + sign·l（sign 逆时针 +1、
    顺时针 −1：圆心在车左边时喷头在左就离圆心近，后轴圆要大）；太小时抛 ValueError。"""
    f, l = nozzle_point(nozzle_x)
    if radius <= abs(f) + 1e-9:
        raise ValueError('圆半径 %.2f m 要大于喷头到后轴的 %.2f m' % (radius, abs(f)))
    r = math.sqrt(radius * radius - f * f) + sign * l
    if r < min_radius - 1e-9:
        raise ValueError('圆半径 %.2f m 太小：喷头画这个圆时后轴要转 %.2f m 的圆，小于车能稳定跟踪的 %.2f m'
                         % (radius, r, min_radius))
    return r


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


def route_length(spec, min_radius, nozzle_x=None):
    """(路线长度米, 画不成的原因)，不含接入段；状态里报给 app。连线路线的长度是连线本身（不含倒车补角）；
    圆和路点折线是车要开的路程（差速车的圆是后轴中点的圆，见 loop_pieces）。"""
    if spec.kind == 'path':
        if len(spec.points) < 2:
            return None, '先在地图上点点，再把点连起来'
        return path_length(path_points(spec)), None
    try:
        _, pieces = loop_pieces(spec, min_radius, nozzle_x if spec.kind == 'circle' else None)
    except ValueError as exc:
        return None, str(exc)
    return sum(p[0] for p in pieces), None


def _marker(spec, body, nozzle_x):
    """车上要压在路线上的点 (前, 左)：差速车是喷头；阿克曼车连线路线是车身中心，圆和路点折线是后轴中心。"""
    if nozzle_x is not None:
        return nozzle_point(nozzle_x)
    return ((body[0] - body[1]) / 2 if spec.kind == 'path' and body is not None else 0.0), 0.0


def _start_cost(start, car, min_radius, nozzle_x):
    """车从当前位姿接入到后轴起点 start 大概要多久（秒，只用来在差不多一样近的起点里挑）。"""
    x, y, yaw = car
    sx, sy, h0 = start
    if nozzle_x is None:
        try:
            return sum(length for length, _ in dubins(car, start, min_radius)) / 0.5
        except ValueError:
            return math.inf
    gx, gy = _offset((sx, sy), h0, -RUN_UP_M)
    dist = math.hypot(gx - x, gy - y)
    to_g = math.atan2(gy - y, gx - x) if dist >= ON_LINE_M else yaw
    return dist / 0.5 + (abs(wrap(to_g - yaw)) + abs(wrap(h0 - to_g))) / 0.8


def start_from_nearest(spec, car_pose, min_radius, *, body=None, nozzle_x=None):
    """开始作业跑的路线：路线不分起点终点，从离车最近的地方开始、跑完整条（2026-10-01 用户：「点击开始作业，
    就直接在车最近的路线开始走，跑完所有轨迹」）。返回换过起点的 RouteSpec，原来的图纸不变：
      圆            从离压线点（差速车喷头）最近的那一点开始（start_angle），绕完 laps 圈回到这一点；
      闭合连线      从离压线点最近的那条边上的垂足开始（离拐角不到 START_SNAP_M 就从拐角），绕完 laps 圈回到这里；
      闭合路点折线  从最近的路点开始（拐角有圆弧，不在边中间插点）；
      开放连线、路点折线  从离压线点近的那一头开始画完整条线：近的是最后一点就把点倒过来（反着画）。
    差不多一样近（START_TIE_M 以内，比如喷头停在圆心）时挑接入最省事的。"""
    mx, my = _marker(spec, body, nozzle_x)
    x, y, yaw = car_pose
    px, py = nozzle_at(x, y, yaw, mx, my)
    cands = []          # (压线点到起点的距离, 换过起点的路线, 后轴起点位姿)

    def leg_start(points):
        (ax, ay), (bx, by) = points[0], points[1]
        h = math.atan2(by - ay, bx - ax)
        return axle_for((ax, ay), h, mx, my) + (h,)

    if spec.kind == 'circle':
        if spec.center is None:
            raise ValueError('还没有设置圆心')
        cx, cy = spec.center
        sign = -1.0 if spec.clockwise else 1.0
        if nozzle_x is not None and _offset_nozzle(nozzle_x):
            d, side = nozzle_point(nozzle_x)
            r = circle_axle_radius(spec.radius, (d, side), min_radius, sign)
        else:
            d = side = 0.0
            r = spec.radius
        # 喷头在后轴方位角前面（顺着画的方向）这么多
        lx, ly = nozzle_at(r, 0.0, sign * math.pi / 2, d, side)
        lag = sign * math.atan2(ly, lx)
        near = math.atan2(py - cy, px - cx) - sign * lag
        angles = [near] + [near + 2 * math.pi * i / 72 for i in range(1, 72)]
        if nozzle_x is not None:
            # 差速车：后轴顺着切线直接开上圆（接圆前沿圆走的 CIRCLE_RUN_IN_M 从切点算）的两个起点也算候选；
            # 有进圆过渡时直线切的是外挪 p 的圆，过渡接上圆那一点比切点靠前 φ
            dc = math.hypot(x - cx, y - cy)
            p, phi = _transition_offset(r, sign)
            if dc > r + p + 1e-6:
                for s in (1.0, -1.0):
                    angles.append(math.atan2(y - cy, x - cx) + s * math.acos((r + p) / dc)
                                  + sign * (phi + CIRCLE_RUN_IN_M / spec.radius))
        for a in angles:
            a = wrap(a)
            sx, sy = cx + r * math.cos(a), cy + r * math.sin(a)
            h = wrap(a + sign * math.pi / 2)
            nx, ny = nozzle_at(sx, sy, h, d, side)
            cands.append((math.hypot(px - nx, py - ny), replace(spec, start_angle=a), (sx, sy, h)))
        if nozzle_x is not None:
            # 整圆没有起点终点：差速车挑接入最省事的起点（2026-10-07 用户：画圆直接走最近的接入口开始），
            # 不是离喷头最近的那一点——那一点要往回让出沿圆走和直线助跑，车得绕到后面去
            def cost(c):
                route, entry_fn, _ = _circle_route(c[1], car_pose, min_radius, nozzle_x)
                return entry_seconds(entry_fn((1, route[0][1], []), car_pose, nozzle_x)[0], yaw)
            return min(cands, key=lambda c: (round(cost(c), 6), c[0]))[1]
    elif spec.closed and len(spec.points) >= 3:
        pts = [tuple(map(float, p)) for p in spec.points]
        n = len(pts)
        for k in range(n):
            (ax, ay), (bx, by) = pts[k], pts[(k + 1) % n]
            length = math.hypot(bx - ax, by - ay)
            if length < 1e-9:
                continue
            ux, uy = (bx - ax) / length, (by - ay) / length
            u = max(0.0, min(length, (px - ax) * ux + (py - ay) * uy))
            if spec.kind == 'points':           # 路点折线拐角有圆弧：只从路点起步
                corner = k if u < length / 2 else (k + 1) % n
            elif u < START_SNAP_M:
                corner = k                      # 离这条边的起点（拐角）近：从拐角开始
            elif u > length - START_SNAP_M:
                corner = (k + 1) % n
            else:
                corner = None
            order = pts[corner:] + pts[:corner] if corner is not None else \
                [(ax + u * ux, ay + u * uy)] + pts[k + 1:] + pts[:k + 1]
            sx, sy = order[0]
            cands.append((math.hypot(px - sx, py - sy), replace(spec, points=tuple(order)), leg_start(order)))
    elif len(spec.points) >= 2:
        pts = tuple(tuple(map(float, p)) for p in spec.points)
        for order in (pts, pts[::-1]):
            cands.append((math.hypot(px - order[0][0], py - order[0][1]), replace(spec, points=order),
                          leg_start(order)))
    else:
        return spec
    best = min(c[0] for c in cands)
    near = [c for c in cands if c[0] <= best + START_TIE_M]
    if len(near) == 1:
        return near[0][1]
    return min(near, key=lambda c: (_start_cost(c[2], car_pose, min_radius, nozzle_x), c[0]))[1]


def _circle_run_out(spec, loop):
    """差速车的圆（loop_pieces 的后轴圆）：整圈后喷头再多喷 CIRCLE_OVERLAP_M（沿图纸上半径 R 的圆量，
    后轴圆半径 r 上乘 r/R），停喷后沿圆继续 SOLO_CIRCLE_COAST_M，再切线开出 SOLO_CIRCLE_EXIT_M。仅独立圆。"""
    k = loop[-1][1]
    r = 1.0 / abs(k)
    arc = 2.0 * math.pi * r * spec.laps + (CIRCLE_OVERLAP_M + SOLO_CIRCLE_COAST_M) * r / spec.radius
    return loop[:-1] + [(arc, k), (SOLO_CIRCLE_EXIT_M, 0.0)]


def _circle_transition(k):
    """进圆过渡（见 CIRCLE_TRANSITION_M）：[(长度, 曲率), …]，曲率从 0 均匀增到 k；CIRCLE_TRANSITION_M 为 0 时为空。"""
    n = CIRCLE_TRANSITION_STEPS
    if CIRCLE_TRANSITION_M <= 0 or n <= 0:
        return []
    step = CIRCLE_TRANSITION_M / n
    return [(step, k * (i + 0.5) / n) for i in range(n)]


def _transition_start(x, y, yaw, ramp):
    """过渡终点（接上圆那一点）位姿 (x, y, yaw) 往回推过渡 ramp，返回过渡起点位姿。"""
    for step, kk in reversed(ramp):
        x, y, yaw = advance(x, y, yaw, -step, kk)
    return x, y, yaw


def _transition_offset(r, sign):
    """后轴圆半径 r（sign 逆时针 +1、顺时针 −1）接进圆过渡时，直线助跑那条线：(离圆心多远 − r, 这条线上离圆心最近的
    那一点比过渡接上圆那一点在圆上靠后多少弧度)。没有过渡为 (0, 0)。"""
    ramp = _circle_transition(sign / r)
    if not ramp:
        return 0.0, 0.0
    # 接上圆那一点放在圆心 (0, 0) 正下方往右开（逆时针）/ 往左开（顺时针）
    jx, jy, jyaw = 0.0, -r, (0.0 if sign > 0 else math.pi)
    sx, sy, syaw = _transition_start(jx, jy, jyaw, ramp)
    # 圆心到直线的距离、垂足
    ux, uy = math.cos(syaw), math.sin(syaw)
    t = -(sx * ux + sy * uy)
    fx, fy = sx + t * ux, sy + t * uy
    return math.hypot(fx, fy) - r, abs(wrap(math.atan2(fy, fx) - math.atan2(jy, jx)))


def _circle_run_in(spec, route):
    """差速车的圆：起点沿后轴圆往回挪 CIRCLE_RUN_IN_M（沿图纸上半径 R 的圆量，后轴圆半径 r 上是
    CIRCLE_RUN_IN_M·r/R），这段圆弧并进第一段，算接入、不喷；再往前接进圆过渡（_circle_transition），也算接入、不喷；
    起喷点还是原来的起点。返回 (新 route, 后轴多走的米数)。"""
    (d, (x, y, yaw), pieces), rest = route[0], route[1:]
    length, k = pieces[0]
    arc = CIRCLE_RUN_IN_M / (abs(k) * spec.radius)
    ramp = _circle_transition(k)
    start = _transition_start(*advance(x, y, yaw, -arc, k), ramp)
    return ([(d, start, ramp + [(arc + length, k)] + list(pieces[1:]))] + list(rest),
            arc + sum(v for v, _ in ramp))


def _entry_fn(first, car, nozzle_x):
    """倒车助跑还是原地转向接入：按喷头离起喷点（圆为挪回沿圆走之前的起点）的距离选。"""
    _, (x0, y0, h0), _ = first
    x, y, yaw = car
    f, l = nozzle_point(nozzle_x)
    gap = math.dist(nozzle_at(x, y, yaw, f, l), nozzle_at(x0, y0, h0, f, l))
    return runup_entry if gap <= NEAR_START_M else pivot_entry


def _circle_route(run, car, min_radius, nozzle_x):
    """差速车的圆（run 已定好起点）：(路线 [整圈 + 多喷 + 多走，起点挪回 CIRCLE_RUN_IN_M], 接入函数, 沿圆多走的米数)。"""
    start, loop = loop_pieces(run, min_radius, nozzle_x)
    route = [(1, start, _circle_run_out(run, loop))]
    entry_fn = _entry_fn(route[0], car, nozzle_x)
    route, run_in = _circle_run_in(run, route)
    return route, entry_fn, run_in


def _circle_paint_end(sections, spec):
    """整圈加 10 cm 停喷；后续 30 cm 喷头圆弧与 1 m 后轴切线均不喷。"""
    section = sections[-1]
    curve = section.path.pieces[-2]
    coast = SOLO_CIRCLE_COAST_M / (abs(curve.kappa) * spec.radius)
    section.paint_to_m = section.path.length - SOLO_CIRCLE_EXIT_M - coast
    return sections


def _planned_sections(entry, route, name):
    from path_tracker import Path, Section
    raw = entry + route[1:]
    result = [Section(d, Path(start, pieces, name=name+str(i)))
              for i, (d, start, pieces) in enumerate(raw)]
    if not hasattr(route[0][2], 'paint_spans'):
        return result
    for i, sec in enumerate(result):
        source = route[0] if i == len(entry)-1 else raw[i]
        spans = getattr(source[2], 'paint_spans', []) if i >= len(entry)-1 else []
        shift = sec.path.length-sum(v for v, _ in source[2]) if i == len(entry)-1 else 0.
        sec.paint_spans = [(a+shift,b+shift) for a,b in spans]
        sec.field_role = 'paint' if spans else 'transit'
        sec.paint_from_m = sec.paint_spans[0][0] if spans else 0.
        sec.paint_to_m = sec.paint_spans[-1][1] if spans else None
    return result


def plan_current(spec, car_pose, min_radius, *, body=None, max_entry_m=MAX_ENTRY_M, nozzle_x=None, info=None):
    """开始作业：从离车最近的地方开始、跑完整条路线（见 start_from_nearest）。返回 ([path_tracker.Section, …], 接入路程米)。

    差速车（nozzle_x 不为 None）：喷头离起点不到 NEAR_START_M 就倒车助跑进线（runup_entry），更远按 pivot_entry
    开到起点前 ENTRY_LEAD_M 对正；阿克曼车同 plan（Reeds-Shepp / Dubins 接入）。原始图纸不变。
    info 给一个 dict 时填进 arc_turns：差速车连线路线圆弧掉头的急弯数（其余为 0）。
    """
    from path_tracker import Path, Section
    run = start_from_nearest(spec, car_pose, min_radius, body=body, nozzle_x=nozzle_x)
    if nozzle_x is None:
        return plan(run, car_pose, min_radius, body=body, max_entry_m=max_entry_m, info=info)
    if info is not None:
        info['arc_turns'] = 0
    if run.kind == 'path':
        if len(run.points) < 2:
            raise ValueError('先在地图上点点，再把点连起来')
        route, corners = _pivot_route_sections(run, min_radius, nozzle_x)
        if info is not None and run.turn == 'arc':
            info['arc_turns'] = corners
    elif run.kind == 'circle':
        route, entry_fn, run_in = _circle_route(run, car_pose, min_radius, nozzle_x)
    else:
        start, loop = loop_pieces(run, min_radius)
        route = [(1, start, loop)]
    if run.kind != 'circle':
        entry_fn, run_in = _entry_fn(route[0], car_pose, nozzle_x), 0.0
    entry, entry_m = entry_fn(route[0], car_pose, nozzle_x)
    if entry_m > max_entry_m:
        raise ValueError('车离路线太远（接入路线 %.0f m，最多 %.0f m），请先把车开近些' % (entry_m, max_entry_m))
    sections = _planned_sections(entry, route, 'route_'+run.kind+'_')
    return (_circle_paint_end(sections, run) if run.kind == 'circle' else sections), entry_m + run_in


def plan(spec, car_pose, min_radius, *, body=None, max_entry_m=MAX_ENTRY_M, nozzle_x=None, info=None):
    """整条要跑的路径：从车当前位姿接入 + 路线本身。返回 ([path_tracker.Section, …], 接入路程米)。

    路点折线和圆只有一段（前进）；连线路线按 cover_route 分成前进、倒车交替的若干段，body 见 cover_route。
    nozzle_x 不为 None 是差速车：连线路线按 pivot_cover 喷头压线、急弯原地转向（路线选了圆弧掉头时圆弧绕过去），
    接入用 pivot_entry；段与段之间车头方向不同的地方由 SectionTracker 原地转过去。info 同 plan_current。
    """
    from path_tracker import Path, Section   # path_tracker 依赖 robot_core；只在真要建路径时导入
    if info is not None:
        info['arc_turns'] = 0
    if spec.kind == 'path':
        if len(spec.points) < 2:
            raise ValueError('先在地图上点点，再把点连起来')
        if nozzle_x is not None:
            route, corners = _pivot_route_sections(spec, min_radius, nozzle_x)
            if info is not None and spec.turn == 'arc':
                info['arc_turns'] = corners
            entry, entry_m = pivot_entry(route[0], car_pose, nozzle_x)
        elif body is None:
            raise ValueError('连线路线要知道车身尺寸（body）才能算车身中心压线')
        else:
            route, _ = _route_sections(spec, min_radius, body)
            entry, entry_m = cover_entry(route[0], car_pose, min_radius)
        if entry_m > max_entry_m:
            raise ValueError('车离路线起点太远（接入路线 %.0f m，最多 %.0f m），请先把车开近些' % (entry_m, max_entry_m))
        return _planned_sections(entry, route, 'route_path_'), entry_m
    if spec.kind == 'circle' and nozzle_x is not None:
        # 差速车：喷头画圆，接入原地转向对准、直线开过去（与连线路线同一套）
        start, loop = loop_pieces(spec, min_radius, nozzle_x)
        route, run_in = _circle_run_in(spec, [(1, start, _circle_run_out(spec, loop))])
        entry, entry_m = pivot_entry(route[0], car_pose, nozzle_x)
        if entry_m > max_entry_m:
            raise ValueError('车离路线起点太远（接入路线 %.0f m，最多 %.0f m），请先把车开近些' % (entry_m, max_entry_m))
        return _circle_paint_end([Section(d, Path(s, pieces, name='route_circle_%d' % i))
                                  for i, (d, s, pieces) in enumerate(entry)], spec), entry_m + run_in
    start, loop = loop_pieces(spec, min_radius)
    entry = dubins(car_pose, start, spec.turn_radius)
    entry_m = sum(length for length, _ in entry)
    if entry_m > max_entry_m:
        raise ValueError('车离路线起点太远（接入路线 %.0f m，最多 %.0f m），请先把车开近些' % (entry_m, max_entry_m))
    return [Section(1, Path(car_pose, entry + loop, name='route_' + spec.kind))], entry_m
