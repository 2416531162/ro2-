#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车体足迹与扫掠路径碰撞检查 (Swept-Path Collision Check)。

为什么需要这个
--------------
改造前的避障是「前向 ±30° 锥形里取最近距离」,这等于**把车当成一个点**。
两个后果:

1. **忽略车宽**。障碍物在正前方偏一点、不在锥形里,但在车体宽度之内,
   照样撞上 —— 过门时轮子刮墙就是这么来的。
   而且锥形的形状本身就不对:±30° 在 0.5m 处只覆盖 ±0.29m,
   比半车宽还窄;到 3m 处又张到 ±1.7m,把根本撞不到的东西也算进来。
   直行时真正该检查的是一条**等宽走廊**,不是一个扇形。

2. **忽略转弯扫掠**。阿克曼车转弯时车体扫过的是一个圆环:
   - **内侧后轮**切内弯,半径最小 (R - 半车宽)
   - **外侧前角**甩出去,半径最大 (hypot(前悬, R + 半车宽))

   车头中线能过去,不代表四个角都能过去。门框恰恰是被这两处刮到的。

坐标约定
--------
车体坐标系原点在**后轴中心**(阿克曼转弯时车体绕后轴延长线上一点旋转),
x 向前为正,y 向左为正。雷达装在别处,用 SensorMount 换算过来。

所有尺寸都必须**实测**,照着 docs/TUNING.md 第 9 节量。
估错了比不做还危险:估小了照样撞,估大了车哪儿都过不去。
"""

import math
from dataclasses import dataclass

__all__ = ["VehicleFootprint", "SensorMount", "scan_to_vehicle_frame",
           "optical_to_vehicle", "is_self_hit", "drop_self_hits", "in_blind_sector",
           "swept_path_clearance", "corridor_clearance", "arc_clearance",
           "widest_passable_steer", "limit_steer_for_clearance",
           "lidar_target_gap"]

EPS = 1e-9


@dataclass
class VehicleFootprint:
    """车体外轮廓,原点在后轴中心。

    front_m       后轴中心 -> 车体最前端。注意要量到**最外伸的那个东西**:
                  保险杠、相机支架、雷达立柱,哪个伸得最远算哪个。
    rear_m        后轴中心 -> 车体最后端。
    half_width_m  车体中线 -> 最宽处的一半。通常是**轮胎外沿**,不是底盘板宽。
    margin_m      侧向安全余量。建议至少 0.05m:雷达有角分辨率误差,
                  车也不会绝对笔直地走。
    """
    front_m: float = 0.67          # 实测 2026-09-16
    rear_m: float = 0.18
    half_width_m: float = 0.335     # 全宽 0.67 的一半,按轮胎外沿
    margin_m: float = 0.06

    def __post_init__(self):
        if self.front_m <= 0 or self.rear_m < 0:
            raise ValueError("前后悬尺寸不合理")
        if self.half_width_m <= 0:
            raise ValueError("half_width_m 必须为正")
        if self.margin_m < 0:
            raise ValueError("margin_m 不能为负")

    @property
    def effective_half_width(self):
        return self.half_width_m + self.margin_m

    @property
    def width_m(self):
        return 2.0 * self.half_width_m

    def corners(self):
        """四个角,顺序:前左、前右、后左、后右。"""
        hw = self.half_width_m
        return ((self.front_m, hw), (self.front_m, -hw),
                (-self.rear_m, hw), (-self.rear_m, -hw))

    def min_gap_needed(self):
        """直行通过一个开口所需的最小净宽(含余量)。

        >>> VehicleFootprint(half_width_m=0.335, margin_m=0.06).min_gap_needed()
        0.79
        """
        return round(2.0 * self.effective_half_width, 6)


@dataclass
class SensorMount:
    """传感器在车体坐标系里的安装位置。

    x_m    后轴中心 -> 传感器,向前为正
    y_m    车体中线 -> 传感器,向左为正
    yaw_rad 传感器 0 度方向相对车头方向的偏转,逆时针为正
    """
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0


def in_blind_sector(bearing_rad, sectors_deg):
    """方位角是否落在某个屏蔽扇区内。

    扇区用度数表示,可以跨 0 度(例如 (-175, 175) 表示车尾那一小块)。
    比距离门限更精准:只屏蔽真正有车体结构的方向,不牺牲其他方向的探测距离。
    """
    if not sectors_deg:
        return False
    deg = math.degrees(bearing_rad)
    deg = (deg + 180.0) % 360.0 - 180.0      # 归一到 [-180, 180)
    for lo, hi in sectors_deg:
        if lo <= hi:
            if lo <= deg <= hi:
                return True
        elif deg >= lo or deg <= hi:         # 跨越 ±180
            return True
    return False


def scan_to_vehicle_frame(bearings_ranges, mount, max_range=8.0,
                          blind_sectors_deg=None):
    """把 (方位角, 距离) 列表换算成车体坐标系的 (x, y) 点。

    方位角以传感器自身 0 度为基准。返回的点已剔除超量程、非有限值,
    以及落在 blind_sectors_deg 里的方向(车体结构所在的角度)。
    """
    points = []
    cos_y, sin_y = math.cos(mount.yaw_rad), math.sin(mount.yaw_rad)
    for bearing, r in bearings_ranges:
        if not (math.isfinite(bearing) and math.isfinite(r)):
            continue
        if r <= 0.0 or r > max_range:
            continue
        if in_blind_sector(bearing, blind_sectors_deg):
            continue
        sx, sy = r * math.cos(bearing), r * math.sin(bearing)
        points.append((mount.x_m + sx * cos_y - sy * sin_y,
                       mount.y_m + sx * sin_y + sy * cos_y))
    return points


def lidar_target_gap(bearing_rad, range_m, mount, front_m):
    """雷达 (方位, 距离) -> (车头到目标的纵向间距, 横向偏移 **右为正**)。

    雷达方位角是 ROS 约定(左为正),跟随节点里目标横向偏移沿用相机光学系(右为正)。
    直接用 tan(bearing) * gap 会把符号弄反:人在左边,车往右打舵。
    这里先按安装位置投影到车体系,再统一零点与符号。
    """
    px, py = scan_to_vehicle_frame([(bearing_rad, range_m)], mount,
                                   max_range=float("inf"))[0]
    return px - front_m, -py


def is_self_hit(x, y, footprint, skin_m=0.05):
    """这个扫描点是不是雷达扫到了车自己。

    雷达装在车上,周围有相机支架、天线杆、传感器盒、车架立柱,这些会被扫成
    距离恒定、永不消失的"障碍物"。**必须丢弃,绝不能当成障碍物**:

      - 当成障碍物 -> AEB 一直触发,人眼看前方明明空无一物
      - 走廊检查里更糟 —— 车体轮廓内的点会让 corridor_clearance 直接返回 0,
        车永久停住

    判据:点落在车体轮廓(外扩 skin_m)之内。skin_m 是给雷达测距噪声和
    安装位置测量误差留的余量。

    >>> fp = VehicleFootprint(front_m=0.67, rear_m=0.18, half_width_m=0.335)
    >>> is_self_hit(0.55, 0.20, fp)      # 车头里侧的支架
    True
    >>> is_self_hit(0.90, 0.0, fp)       # 车头前方 0.23m,真障碍物
    False
    >>> is_self_hit(0.60, 0.55, fp)      # 侧面伸出去很远,不是车
    False
    """
    return (-footprint.rear_m - skin_m <= x <= footprint.front_m + skin_m
            and abs(y) <= footprint.half_width_m + skin_m)


def drop_self_hits(points, footprint, skin_m=0.05):
    """滤掉雷达扫到车自己的点,返回 (保留的点, 丢弃数)。"""
    kept = [pt for pt in points if not is_self_hit(pt[0], pt[1], footprint, skin_m)]
    return kept, len(points) - len(kept)


def optical_to_vehicle(x_opt, y_opt, z_opt, mount, pitch_rad):
    """相机光学坐标系 -> 车体坐标系。

    光学系约定 (OpenCV/ROS 标准): X 向右, Y 向下, Z 沿光轴向前。
    车体系约定: x 向前(水平), y 向左, z 向上。

    **为什么必须做这个变换**:深度相机给的 z 是沿**光轴**的距离。相机俯装时
    光轴斜向下,z 不等于水平距离。而且误差不是固定的 —— 它随目标相对相机的
    高度变化:

        俯 15°、目标在相机上方 0.6m(站立的人躯干)时,
        真实水平距离 1.13m,相机只读到 0.94m,差 19cm。

    直接拿 z 当水平距离,车会停在比设定值远 20cm 的地方。
    因为误差随高度变化,也不能用一个常数去补。

    换算(θ 为俯角,向下为正):

        x = Z·cosθ − Y·sinθ
        y = −X
        z = −Z·sinθ − Y·cosθ

    >>> import math
    >>> m = SensorMount(x_m=0.54)
    >>> x, y, z = optical_to_vehicle(0.0, -0.79, 0.824, m, math.radians(15))
    >>> round(x - 0.54, 2)        # 相机到目标的水平距离
    1.0
    """
    c, s = math.cos(pitch_rad), math.sin(pitch_rad)
    x_v = z_opt * c - y_opt * s
    y_v = -x_opt
    z_v = -z_opt * s - y_opt * c
    # 只做纵向/横向平移;相机高度不参与水平距离计算
    return mount.x_m + x_v, mount.y_m + y_v, z_v


def corridor_clearance(points, footprint, max_range=8.0):
    """直行:到第一个挡在车宽走廊里的障碍物的距离(从车头最前端算起)。

    只有横向落在 ±(半车宽 + 余量) 之内的点才算数 —— 这正是锥形检查漏掉的。

    >>> fp = VehicleFootprint(front_m=0.4, rear_m=0.2, half_width_m=0.3, margin_m=0.0)
    >>> round(corridor_clearance([(1.4, 0.0)], fp), 3)     # 正前方 1.4m
    1.0
    >>> round(corridor_clearance([(1.4, 0.25)], fp), 3)    # 偏左 0.25m,仍在车宽内
    1.0
    >>> round(corridor_clearance([(1.4, 0.8)], fp), 3)     # 偏左 0.8m,让得开
    8.0
    >>> round(corridor_clearance([(0.3, 0.0)], fp), 3)     # 已经贴在车头上
    0.0
    """
    half = footprint.effective_half_width
    best = max_range
    for px, py in points:
        if abs(py) > half:
            continue
        if px < -footprint.rear_m:
            continue            # 在车尾之后,前进时撞不到
        if px <= footprint.front_m:
            if abs(py) <= footprint.half_width_m:
                return 0.0          # 已经在车体物理轮廓之内,贴上了
            continue                # 在车身侧向余量内,但位于车头后方,不挡直行
        gap = px - footprint.front_m
        if gap < best:
            best = gap
    return max(0.0, min(best, max_range))


def _swept_radii(footprint, radius):
    """左转半径 radius 时,车体扫过的圆环内外半径。

    内半径来自**内侧后轮一带**(转弯中心正对着的车体侧面),
    外半径来自**外侧前角** —— 这两处就是刮门框的元凶。
    """
    hw = footprint.half_width_m + footprint.margin_m
    inner = radius - hw
    outer = math.hypot(footprint.front_m + footprint.margin_m, radius + hw)
    return max(0.0, inner), outer


def arc_clearance(points, footprint, radius, left, max_range=8.0):
    """转弯:沿圆弧行驶到第一个会撞上的障碍物的路程。

    radius 是**后轴中心**的转弯半径(与固件 TurnR = Vx/Vz 同一个量)。
    left=True 左转(转弯中心在车体左侧),False 右转。
    """
    if radius <= EPS:
        return 0.0

    inner, outer = _swept_radii(footprint, radius)
    best = max_range

    for px, py in points:
        # 右转时把 y 镜像过去,后面只按左转算一套逻辑,避免符号写错
        y = py if left else -py
        dx, dy = px, y - radius
        d = math.hypot(dx, dy)
        if d < inner or d > outer:
            continue

        # 车体绕中心逆时针前进;起点(后轴中心)相对中心的方位是 -90°
        delta = math.atan2(dy, dx) + math.pi / 2.0
        while delta < 0.0:
            delta += 2.0 * math.pi
        while delta >= 2.0 * math.pi:
            delta -= 2.0 * math.pi

        travel = radius * delta - footprint.front_m
        if travel < best:
            best = max(0.0, travel)

    return max(0.0, min(best, max_range))


def swept_path_clearance(points, footprint, geometry, steer_rad, max_range=8.0):
    """统一入口:按当前前轮转角选走廊或圆弧检查。

    geometry 是 motion_safety.ChassisGeometry,提供轴距与轮距。
    返回沿实际行驶路径到第一个障碍物的距离(从车头最前端算起,米)。
    """
    steer = max(-geometry.max_steer_rad, min(geometry.max_steer_rad, steer_rad))
    if abs(steer) < math.radians(1.0):
        return corridor_clearance(points, footprint, max_range)

    # 与固件一致:TurnR = 轴距/tan(前轮转角) + 轮距/2 (见 PROTOCOL.md 8.1)
    radius = geometry.wheelbase_m / math.tan(abs(steer)) + 0.5 * geometry.track_m
    return arc_clearance(points, footprint, radius, left=(steer > 0),
                         max_range=max_range)


def limit_steer_for_clearance(points, footprint, geometry, desired_steer,
                              min_clearance, max_range=8.0, tries=(0.75, 0.5, 0.25, 0.0)):
    """净空不够时把转角往中间收,直到路走得通;收到笔直还不行才是真过不去。

    这是过门刮轮子的**行为层**对策。几何上转弯需要的通道比直行更宽
    (见 _swept_radii),所以在窄门里跟着人修方向,必然刮。

    人往旁边偏一点,车本能地打舵去追,恰好在门框里扫出最宽的轨迹。
    正确做法是:先把车摆正穿过去,出了门再修方向。

    返回 (收敛后的转角, 该转角下的净空)。
    """
    clear = swept_path_clearance(points, footprint, geometry, desired_steer, max_range)
    if clear >= min_clearance or abs(desired_steer) < EPS:
        return desired_steer, clear

    best_steer, best_clear = desired_steer, clear
    for frac in tries:
        steer = desired_steer * frac
        c = swept_path_clearance(points, footprint, geometry, steer, max_range)
        if c > best_clear:
            best_steer, best_clear = steer, c
        if c >= min_clearance:
            return steer, c          # 够用就停手,保留尽可能多的转向能力
    return best_steer, best_clear


def widest_passable_steer(points, footprint, geometry, candidates_deg=None,
                          max_range=8.0):
    """在候选转角里挑一个前方最空的,用于「车头能过但角过不去」时找条活路。

    返回 (转角弧度, 该转角下的可行距离)。
    """
    if candidates_deg is None:
        limit = math.degrees(geometry.max_steer_rad)
        candidates_deg = [-limit, -limit / 2, 0.0, limit / 2, limit]
    best_steer, best_clear = 0.0, -1.0
    for deg in candidates_deg:
        steer = math.radians(deg)
        clear = swept_path_clearance(points, footprint, geometry, steer, max_range)
        if clear > best_clear:
            best_steer, best_clear = steer, clear
    return best_steer, best_clear


if __name__ == "__main__":
    import doctest
    import sys
    import os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from motion_safety import ChassisGeometry

    failures, _ = doctest.testmod()
    fp = VehicleFootprint()
    geo = ChassisGeometry()

    print("车体足迹: 全宽 %.2fm  前悬 %.2fm  后悬 %.2fm  余量 %.2fm"
          % (fp.width_m, fp.front_m, fp.rear_m, fp.margin_m))
    print("直行通过最小净宽: %.2f m\n" % fp.min_gap_needed())

    print("转角   转弯半径   内侧扫掠   外侧扫掠   扫掠带宽")
    print("-" * 52)
    for deg in (5, 10, 14, 20):
        steer = math.radians(deg)
        radius = geo.wheelbase_m / math.tan(steer) + 0.5 * geo.track_m
        inner, outer = _swept_radii(fp, radius)
        print("%3d°  %7.2f m  %7.2f m  %7.2f m  %7.2f m"
              % (deg, radius, inner, outer, outer - inner))
    print("\n注意扫掠带宽 > 车宽 %.2fm —— 转弯时需要的通道比直行更宽。" % fp.width_m)
    raise SystemExit(1 if failures else 0)
