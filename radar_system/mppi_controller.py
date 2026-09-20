#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MPPI (Model Predictive Path Integral) 跟随控制器。

它替换的是**参考量的生成**:纯追踪的转角、距离 P 律加前馈的速度、以及
`limit_steer_for_clearance` 那个收舵覆盖。它**不替换**刹车包络、AEB、
车体扫掠检查和脱困状态机 —— 那些仍然串在它下游,原样保留。

为什么这条边界不能挪
--------------------
MPPI 是**软约束**采样器:它给撞车的 rollout 加一个很大的代价,但最终输出是
按 exp(-S/λ) 加权的平均控制,这个平均值可能落在没有任何一条样本占据的区域。
代价权重调错一个数量级,碰撞惩罚就会被目标项淹没,而且**不会报错** ——
车照跑,只是开始蹭东西。所以硬安全必须由一个独立的、确定性的监护层保证。

本控制器自己也留了一道:算完加权平均之后再用这条输出做一次**确定性 rollout**,
真撞上就把 feasible 置 False、速度归零交回给调用方,而不是把一条自己都验不过
的指令发下去。

MPPI 在这里真正买到的东西
------------------------
1. 消掉内部矛盾。纯追踪说「往左打去跟人」,收舵逻辑说「摆正过门」,这两条
   过去是靠优先级和一个保持窗口硬压的。现在它们是同一个代价函数里的两项。
2. 真正的前瞻。单条圆弧的净空检查是短视的:它只知道「沿这条弧多远会撞」,
   不知道「先往右让一下再切回来就能过去」。MPPI 在 T 步的控制序列上采样,
   绕开椅子是自然解,不是特例代码。
3. 用得上人的速度。目标不是人**现在**在哪,而是 t 秒后的**预测跟随点**。
   这把「追尾巴」变成「拦截」,跟随看起来自不自然基本就取决于它。
4. 视野保持进了代价函数。Astra S 的水平视野有限,MPPI 抄近道会把人甩出画面,
   跟丢之后再好的控制也没用 —— 所以「人要留在相机里」必须是显式代价,
   教科书上的 MPPI 没有这一项,但这台车上少了它就会自己把自己跟丢。

坐标系:全部在**规划时刻的后轴中心**车体系里,x 向前,y 向左。
注意与跟随层的换算:那边的 z 是「车头到人」的间距,这里要加回 front_m。
"""

import math
from dataclasses import dataclass, field
import numpy as np

from mppi_backend import get_backend, torch_available, cuda_available
from runtime_config import PROFILE

EPS = 1e-6


# =============================================================================
# 局部距离场
# =============================================================================

@dataclass
class FieldConfig:
    resolution_m: float = 0.05
    x_min_m: float = -1.50      # 车后留一点,倒车不归 MPPI 管但代价要看得见
    x_max_m: float = 5.00
    y_half_m: float = 3.00
    max_distance_m: float = 3.00    # 距离场上限,再远对代价没有区别
    max_points: int = 600           # 栅格去重后的点数上限


class DistanceField:
    """车体系下的局部欧氏距离场:每个格子到最近障碍物的距离。

    为什么不直接拿点云给 rollout 打分:那是 O(K·T·P·N)。K=2048、T=30、
    三圆、N=400 就是每周期 7400 万次距离计算。距离场把它拆成「建场一次
    O(格子·N)」+「查表 O(K·T·3)」,后者是 18 万次访存,差两个数量级。

    建场用的是对障碍点的暴力最小值,不是近似的两遍扫描 —— 精确、十行、
    可以直接对拍验证,而在 GPU 上 (16k 格 × 400 点) 只是一次广播。
    """

    def __init__(self, backend, config=None):
        self.b = backend
        self.cfg = config or FieldConfig()
        res = self.cfg.resolution_m
        self.nx = max(2, int(round((self.cfg.x_max_m - self.cfg.x_min_m) / res)))
        self.ny = max(2, int(round(2.0 * self.cfg.y_half_m / res)))
        self.x0 = self.cfg.x_min_m
        self.y0 = -self.cfg.y_half_m
        self.res = res
        # 格子中心坐标,建一次就不变
        b = self.b
        ix = b.array([(i + 0.5) for i in range(self.nx)])
        iy = b.array([(j + 0.5) for j in range(self.ny)])
        self._cx = self.x0 + ix * res                      # (nx,)
        self._cy = self.y0 + iy * res                      # (ny,)
        self.field = b.full((self.ny * self.nx,), self.cfg.max_distance_m)
        self.obstacle_count = 0
        self.last_points = []
        self.verification_points = np.empty((0, 2), dtype=np.float64)

    def _dedup(self, points):
        """按栅格去重并裁掉视野外的点,控制建场成本。"""
        cell = self.res
        seen = {}
        for x, y in points:
            if not (self.cfg.x_min_m - 1.0 <= x <= self.cfg.x_max_m + 1.0):
                continue
            if abs(y) > self.cfg.y_half_m + 1.0:
                continue
            seen[(round(x / cell), round(y / cell))] = (x, y)
        out = list(seen.values())
        if len(out) > self.cfg.max_points:
            step = len(out) / float(self.cfg.max_points)
            out = [out[int(i * step)] for i in range(self.cfg.max_points)]
        return out

    def build(self, points):
        b = self.b
        original = np.asarray(points, dtype=np.float64).reshape(-1, 2)
        if not np.isfinite(original).all():
            raise ValueError('MPPI obstacle points must be finite')
        # Exact verification must retain thin obstacles removed by grid/cap sampling.
        self.verification_points = original
        pts = self._dedup(original)
        self.obstacle_count = len(pts)
        self.last_points = pts
        if not pts:
            # 一个障碍物都没有 != 前面一定是空的。上游必须先确认这一帧雷达
            # 本身可用 (ScanEvidence.usable),否则这里会把瞎眼当畅通。
            self.field[:] = self.cfg.max_distance_m
            return self
        px = b.array([p[0] for p in pts]).reshape(1, 1, -1)
        py = b.array([p[1] for p in pts]).reshape(1, 1, -1)
        gx = self._cx.reshape(1, -1, 1)          # (1, nx, 1)
        gy = self._cy.reshape(-1, 1, 1)          # (ny, 1, 1)
        # Minimize squared distances first: one sqrt per cell, not per point/cell.
        d2 = (gx - px) ** 2 + (gy - py) ** 2
        d = b.sqrt(b.amin(d2, axis=2)).reshape(-1)
        # Keep the allocation stable: the CUDA graph holds this table's address.
        self.field[:] = b.clip(d, 0.0, self.cfg.max_distance_m)
        return self

    def lookup(self, x, y):
        """最近邻查表。分辨率 5cm 下不做双线性够用,少一半算子。

        落到格子外的点钳到边界格:侧向出界意味着车拐到了局部地图之外,
        那里没有证据,用边界值是保守但不悲观的折中 —— 真正的硬保证在
        下游的扫掠检查,不在这里。
        """
        b = self.b
        fx = b.clip((x - self.x0) / self.res, 0.0, self.nx - 1.0)
        fy = b.clip((y - self.y0) / self.res, 0.0, self.ny - 1.0)
        idx = b.to_long(fy) * self.nx + b.to_long(fx)
        return b.take(self.field, idx)

    def probe(self, x, y):
        """单点查询,给测试和遥测用。"""
        b = self.b
        return b.item(self.lookup(b.array([x]), b.array([y])).reshape(-1)[0])


def rectangle_clearance(poses, points, front_m, rear_m, half_width_m):
    """一串位姿下,车体**矩形**到最近障碍点的净空。负数 = 已经压上去了。

    为什么复验不能用栅格距离场:5cm 分辨率的最近邻查表误差最大是半个对角线,
    也就是 3.5cm。而这台车过 80cm 门两边各只剩 3cm —— 用一个误差比余量还大的
    近似值去判"能不能过",结论是随机的。

    所以分工是:栅格距离场负责给 K 条样本**排序**(近似完全够用),最终选出来
    的那一条用这里的精确几何**复验**。单条 40 步 x 600 点 = 2.4 万次,不值一提。
    """
    points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    poses = np.asarray(poses, dtype=np.float64).reshape(-1, 3)
    if points.size == 0 or poses.size == 0:
        return 9.0
    if not np.isfinite(points).all() or not np.isfinite(poses).all():
        return -float('inf')
    cx = 0.5 * (front_m - rear_m)          # 矩形中心相对后轴中心
    hx = 0.5 * (front_m + rear_m)
    hy = half_width_m
    c, s = np.cos(poses[:, 2, None]), np.sin(poses[:, 2, None])
    dx = points[None, :, 0] - poses[:, None, 0]
    dy = points[None, :, 1] - poses[:, None, 1]
    ex = np.abs(dx*c + dy*s - cx) - hx
    ey = np.abs(-dx*s + dy*c) - hy
    distance = np.hypot(np.maximum(ex, 0.0), np.maximum(ey, 0.0))
    distance += np.minimum(np.maximum(ex, ey), 0.0)
    return min(9.0, float(distance.min()))


# =============================================================================
# 配置
# =============================================================================

@dataclass
class MPPIConfig:
    """★ 标的需要实车确认。其余可以先照抄。"""

    # ---- 采样 ----
    samples: int = 1024          # K；增加前在板端 benchmark_mppi.py 实测预算
    # T*dt = 6.0s；跟随节点派生的 0.45m/s 上限下最多约 2.7m。
    # 前瞻长度与转弯半径、现场净空和算力共同决定可规划的局部机动。
    horizon: int = 40            # T 步
    dt_s: float = 0.15           # 每步时长
    control_dt_s: float | None = None  # 未指定时等于预测步；跟随节点传入实际 50 ms 周期
    cuda_graph: bool = True     # 固定形状 rollout 在启动阶段捕获，周期内只 replay
    temperature: float = 0.35    # λ。调小 -> 更接近最优单条样本,也更抖
    sigma_v: float = 0.12        # 速度噪声 (m/s)
    sigma_steer: float = 0.10    # 转角噪声 (rad)
    # 噪声沿时间轴的相关系数 (AR(1))。这是能不能绕开障碍物的关键,不是调味料:
    # 逐步独立的高斯噪声只会产生零均值的抖动,几乎采不出"连续打舵一秒半"
    # 这种样本 —— 而绕过正前方的柱子恰恰需要先**远离**目标点再切回来,
    # 是个局部极小。相关噪声让样本敢于持续偏航,才探得到绕行解。
    # 0.85 对应约 6 步 (0.9s) 的相关时间；0 表示逐步独立噪声。
    noise_correlation: float = 0.85

    # ---- 车辆与执行 ----
    max_speed_mps: float = 0.55
    min_speed_mps: float = 0.0   # 本轮 MPPI 不负责倒车,下界钉死在 0
    accel_limit_mps2: float = 0.90
    decel_limit_mps2: float = 2.50
    max_steer_rad: float = PROFILE['geometry']['max_steer_rad']
    steer_rate_radps: float = 1.20
    wheelbase_m: float = PROFILE['geometry']['wheelbase_m']
    track_m: float = PROFILE['geometry']['track_m']
    latency_s: float = 0.35      # ★ 死时间。不建模它,MPPI 会系统性打过头

    # ---- 车体 ----
    footprint_front_m: float = PROFILE['geometry']['front_m']
    footprint_rear_m: float = PROFILE['geometry']['rear_m']
    footprint_half_width_m: float = PROFILE['geometry']['half_width_m']
    safety_margin_m: float = 0.035   # 已经算进车体圆半径里,不要再加一次
    # 圆覆盖必然比矩形鼓出来一点,鼓出量 = r - 半宽 = hypot(hw,seg) - hw。
    # 3 个圆时鼓出 2.6cm —— 而这台车过 80cm 门本来两边各只剩 3cm,
    # 2.6cm 直接把门堵死了。5 个圆鼓出 1.0cm,窄门才过得去。
    body_circles: int = 5
    # 软代价希望额外留出的余量。它**不是**碰撞判据,碰撞判据只看圆半径本身;
    # 把两者混为一谈会让安全余量被重复计入,车就再也过不了窄门。
    standoff_m: float = 0.10

    # ---- 跟随几何 ----
    follow_distance_m: float = 1.00   # 车头到人的期望间距
    deadband_m: float = 0.12
    person_speed_moving_mps: float = 0.15   # 高于此值认为人在走,跟随点转到人身后
    person_predict_cap_s: float = 2.0       # 人的匀速外推最多信这么久

    # ---- 代价权重 ----
    w_goal: float = 6.0          # 到预测跟随点的距离
    w_terminal: float = 18.0     # 终端额外权重,让它真的奔着目标去
    w_obstacle: float = 140.0    # 安全裕度被侵蚀
    w_collision: float = 4000.0  # 真撞上。要大到任何目标收益都换不来
    w_fov: float = 30.0          # 人跑出相机视野 (铰链,超出 fov_keep 才罚)。
                                 # 拐直角弯时这一项不够硬,人就会被甩出画面 ——
                                 # 跟丢之后再好的控制律也没有意义
    w_heading: float = 2.5       # 车头没对着人 (软项,始终生效)
    # 只罚转角**变化率**是不够的:到位之后任何恒定舵角代价都为零,随机游走
    # 会把舵停在满位 —— 人一动车就横着窜出去。必须罚转角本身。
    w_steer: float = 18.0        # 转角幅值
    w_speed: float = 0.6         # 无谓的高速
    w_dsteer: float = 12.0       # 转角抖动。调小了车会画龙
    w_dspeed: float = 2.0        # 速度抖动
    # MPPI 标准的名义控制偏离项 gamma * u^T Sigma^-1 eps。它诱导的收缩量是
    # -(gamma/lambda) * u_nom,所以有意义的单位是"每周期把名义序列往零拉几成"。
    # 直接写 gamma 极易写错量级:gamma=0.2、lambda=0.35 就是每周期拉掉 57%,
    # 和收敛正面打架,表现为转角来回摆。
    nominal_shrink: float = 0.02

    # ---- 视野 ----
    camera_hfov_rad: float = 0.98    # ★ Astra S 水平视野 ~56°,半角 0.49
    fov_keep_rad: float = 0.38       # 希望把人保持在这个半角以内

    # ---- 可行性 ----
    feasible_clearance_m: float = 0.015  # 复验允许的最小净空 (精确几何,与 AEB 余量同量级)
    # 只检查**近期**那一段。整条 3 秒轨迹都要求无碰撞会让车被远处的东西吓瘫:
    # 它还没走到那儿就会重新规划十几次。真正的硬保证在下游的扫掠检查,
    # 这里要保证的是"接下来这一秒确实安全"。
    feasible_steps: int = 10
    # 加权平均不可行时,退到代价最低的那条**单样本**。这是 MPPI 的标准兜底:
    # 平均值可能落在没有样本占据的坏区域,而最优样本至少是真实采过的一条。
    allow_best_sample: bool = True
    field: FieldConfig = field(default_factory=FieldConfig)

    def body_offsets(self):
        """用等半径圆覆盖车体矩形,返回 [(沿轴偏移, 半径), ...]。

        矩形 x ∈ [-rear, front],半宽 hw+margin。N 个圆等距排布,
        半径 r = hypot(hw, 段半长) —— 圆会略微鼓出矩形,是保守方向。
        """
        n = max(1, int(self.body_circles))
        hw = self.footprint_half_width_m + self.safety_margin_m
        length = self.footprint_front_m + self.footprint_rear_m
        seg = length / (2.0 * n)
        radius = math.hypot(hw, seg)
        start = -self.footprint_rear_m + seg
        return [(start + 2.0 * seg * k, radius) for k in range(n)]

    @classmethod
    def from_follower(cls, cfg):
        """从 FollowerConfig 派生,避免同一个物理量在两处各写一遍。"""
        out = cls()
        out.max_speed_mps = cfg.max_speed_mps
        out.accel_limit_mps2 = cfg.accel_limit_mps2
        out.decel_limit_mps2 = cfg.decel_limit_mps2
        out.max_steer_rad = cfg.max_steer_rad
        out.steer_rate_radps = cfg.steer_rate_radps
        out.wheelbase_m = cfg.geometry.wheelbase_m
        out.track_m = cfg.geometry.track_m
        out.latency_s = cfg.control_latency_s
        out.footprint_front_m = cfg.footprint_front_m
        out.footprint_rear_m = cfg.footprint_rear_m
        out.footprint_half_width_m = cfg.footprint_half_width_m
        out.safety_margin_m = cfg.footprint_margin_m
        out.follow_distance_m = cfg.follow_distance_m
        out.deadband_m = cfg.deadband_m
        out.control_dt_s = 1.0 / cfg.control_hz
        out.camera_hfov_rad = math.radians(cfg.camera_hfov_deg)
        out.fov_keep_rad = min(out.fov_keep_rad, out.camera_hfov_rad * 0.45)
        return out


@dataclass
class MPPISolution:
    speed: float
    steer: float
    feasible: bool
    cost: float
    min_clearance: float
    reason: str
    solve_ms: float = 0.0
    obstacles: int = 0


# =============================================================================
# 控制器
# =============================================================================

class MPPIController:

    def __init__(self, config=None, backend=None, prefer='auto'):
        self.cfg = config or MPPIConfig()
        cfg = self.cfg
        if cfg.control_dt_s is None:
            cfg.control_dt_s = cfg.dt_s
        if (not all(math.isfinite(value) for value in
                    (cfg.dt_s, cfg.control_dt_s, cfg.latency_s, cfg.temperature))
                or cfg.samples < 2 or cfg.horizon < 2 or cfg.dt_s <= 0
                or cfg.control_dt_s <= 0 or cfg.latency_s < 0
                or cfg.feasible_steps < 1 or cfg.temperature <= 0):
            raise ValueError('Invalid MPPI sampling/timing configuration')
        self.b = backend or get_backend(prefer)
        self.field = DistanceField(self.b, self.cfg.field)
        T = self.cfg.horizon
        self._nominal = self.b.zeros((T, 2))     # [v_ref, steer_ref]
        self._tick = 0
        self.last = None
        circles = self.cfg.body_offsets()
        self._circle_offsets = self.b.array([c[0] for c in circles]).reshape(1, -1)
        self._circle_radii = self.b.array([c[1] for c in circles]).reshape(1, -1)
        self._graph = None
        self._warmup_done = False
        self._sigma = self.b.array([cfg.sigma_v, cfg.sigma_steer]).reshape(1, 1, 2)
        self._inv_var = self.b.array([1/max(cfg.sigma_v**2, EPS),
                                      1/max(cfg.sigma_steer**2, EPS)]).reshape(1, 1, 2)
        beta = min(max(float(cfg.noise_correlation), 0.), .99)
        # AR(1) as a fixed matrix: avoids two small GPU kernels per horizon step.
        kernel = np.zeros((T, T), dtype=np.float32)
        for row in range(T):
            kernel[row, 0] = beta**row
            for col in range(1, row+1):
                kernel[row, col] = math.sqrt(1-beta*beta)*beta**(row-col)
        self._noise_kernel = self.b.array(kernel)
        self._shift_key = None

    # ------------------------------------------------------------------
    # 跟随点
    # ------------------------------------------------------------------

    def follow_points(self, person_xy, person_vel, robot_speed):
        """每一步的预测跟随点,后轴中心坐标系。

        人在 t 秒后的位置按匀速外推 (外推时长有上限,人不会一直匀速)。
        跟随点取在人**身后** follow_distance 处;人站着不动时退化为
        「在人与车的连线上、离人 follow_distance 的那个点」,两者按人的
        速度平滑过渡,避免人刚起步时目标点瞬移。

        注意零点:跟随层的 follow_distance_m 是**车头**到人的间距,而这里
        算的是后轴中心的目标位置,所以要把 front_m 加回去。
        """
        cfg = self.cfg
        px, py = person_xy
        vx, vy = person_vel
        speed = math.hypot(vx, vy)
        standoff = cfg.follow_distance_m + cfg.footprint_front_m

        if speed > EPS:
            move_dir = (-vx / speed, -vy / speed)        # 人身后的方向
        else:
            move_dir = (0.0, 0.0)
        los = math.hypot(px, py)
        if los > EPS:
            los_dir = (-px / los, -py / los)             # 从人指回车
        else:
            los_dir = (-1.0, 0.0)

        alpha = min(1.0, max(0.0, (speed - cfg.person_speed_moving_mps) / 0.30))
        ux = alpha * move_dir[0] + (1.0 - alpha) * los_dir[0]
        uy = alpha * move_dir[1] + (1.0 - alpha) * los_dir[1]
        norm = math.hypot(ux, uy)
        if norm < EPS:
            ux, uy = los_dir
            norm = 1.0
        ux, uy = ux / norm, uy / norm

        pts_x, pts_y, per_x, per_y = [], [], [], []
        for t in range(cfg.horizon):
            tau = min(cfg.latency_s + (t + 1) * cfg.dt_s, cfg.person_predict_cap_s)
            fx, fy = px + vx * tau, py + vy * tau
            per_x.append(fx)
            per_y.append(fy)
            pts_x.append(fx + standoff * ux)
            pts_y.append(fy + standoff * uy)
        return (self.b.array(pts_x), self.b.array(pts_y),
                self.b.array(per_x), self.b.array(per_y))

    # ------------------------------------------------------------------
    # 运动学
    # ------------------------------------------------------------------

    def yaw_rate(self, speed, steer):
        """批量版的 motion_safety.yaw_from_steer,必须逐位对得上。

        固件算的是左前轮转角,TurnR = 轴距/tan(转角) + 轮距/2。上位机这边
        用的是同一个定义,否则 MPPI 规划的弧和车实际走的弧不是一条。
        """
        b = self.b
        cfg = self.cfg
        a = b.abs(steer)
        safe = b.clip(a, 1e-4, cfg.max_steer_rad)
        radius = cfg.wheelbase_m / b.tan(safe) + 0.5 * cfg.track_m
        yaw = b.abs(speed) / radius
        yaw = yaw * b.sign(steer) * b.sign(speed + 1e-12)
        return b.where(a < 1e-4, yaw * 0.0, yaw)

    def _rollout(self, controls, state0, goals, persons, collect_clearance=False,
                 clearance_steps=None, trace=None):
        """批量前向仿真。controls: (K, steps, 2);返回 (代价, 最小净空)。

        trace 非 None 时,把每一步的位姿 (x, y, theta) 追加进去 —— 只在单条
        确定性复验里用,批量采样时不要传,拷贝代价白给。
        """
        b, cfg = self.b, self.cfg
        K = controls.shape[0]
        steps = controls.shape[1]
        n_delay = steps - cfg.horizon

        v = b.full((K,), state0['speed'])
        d = b.full((K,), state0['steer'])
        th = b.zeros((K,))
        x = b.zeros((K,))
        y = b.zeros((K,))
        cost = b.zeros((K,))
        min_clear = b.full((K,), cfg.field.max_distance_m)

        prev_v = controls[:, 0, 0] * 0.0 + state0['speed']
        prev_d = controls[:, 0, 1] * 0.0 + state0['steer']

        for s in range(steps):
            dt = cfg.latency_s / n_delay if s < n_delay else cfg.dt_s
            v_ref = controls[:, s, 0]
            d_ref = controls[:, s, 1]
            # 执行器限幅:MPPI 不准提出车做不到的动作
            v = v + b.clip(v_ref - v, -cfg.decel_limit_mps2 * dt,
                           cfg.accel_limit_mps2 * dt)
            v = b.clip(v, cfg.min_speed_mps, cfg.max_speed_mps)
            d = d + b.clip(d_ref - d, -cfg.steer_rate_radps * dt,
                           cfg.steer_rate_radps * dt)
            d = b.clip(d, -cfg.max_steer_rad, cfg.max_steer_rad)

            omega = self.yaw_rate(v, d)
            th_mid = th + 0.5 * omega * dt          # 中点法,圆弧积分更准
            x = x + v * b.cos(th_mid) * dt
            y = y + v * b.sin(th_mid) * dt
            th = th + omega * dt
            if trace is not None:
                trace.append(b.stack([x, y, th], axis=1))

            if s < n_delay:
                # 死时间段:这些指令已经在路上了,改不了,只推状态不计代价
                prev_v, prev_d = v_ref, d_ref
                continue

            t = s - n_delay
            # --- 障碍 ---
            cos_t, sin_t = b.cos(th), b.sin(th)
            # Evaluate all body circles in one gather/reduction per time step.
            cx = x.reshape(-1, 1) + self._circle_offsets * cos_t.reshape(-1, 1)
            cy = y.reshape(-1, 1) + self._circle_offsets * sin_t.reshape(-1, 1)
            dist = self.field.lookup(cx, cy)
            slack = b.relu(self._circle_radii + cfg.standoff_m - dist)
            hit = dist - self._circle_radii
            obstacle_cost = cfg.w_obstacle * slack * slack
            obstacle_cost += cfg.w_collision * b.relu(-hit) / self._circle_radii
            cost = cost + b.sum(obstacle_cost, axis=1)
            if collect_clearance and (clearance_steps is None or t < clearance_steps):
                min_clear = b.amin(b.stack([min_clear, b.amin(hit, axis=1)], axis=0), axis=0)

            # --- 目标 ---
            gx, gy = goals[0][t], goals[1][t]
            ex, ey = x - gx, y - gy
            weight = cfg.w_goal + (cfg.w_terminal if t == cfg.horizon - 1 else 0.0)
            cost = cost + weight * (ex * ex + ey * ey)

            # --- 视野保持 ---
            ppx, ppy = persons[0][t], persons[1][t]
            rel = b.atan2(ppy - y, ppx - x) - th
            rel = b.atan2(b.sin(rel), b.cos(rel))
            over = b.relu(b.abs(rel) - cfg.fov_keep_rad)
            cost = cost + cfg.w_fov * over * over + cfg.w_heading * rel * rel

            # --- 平滑与速度 ---
            dv = v_ref - prev_v
            dd = d_ref - prev_d
            cost = cost + cfg.w_dspeed * dv * dv + cfg.w_dsteer * dd * dd
            cost = cost + cfg.w_steer * d * d + cfg.w_speed * v * v
            prev_v, prev_d = v_ref, d_ref

        return cost, min_clear

    def warmup(self):
        """Capture the GPU rollout before subscribing to motion commands.

        Only fixed-shape arithmetic is captured; sensor uploads, RNG and exact
        geometry verification stay outside the graph. Capture failures abort
        startup instead of silently changing the backend or consuming a control tick.
        """
        if self._warmup_done:
            return
        if self.b.is_gpu and self.cfg.cuda_graph:
            import torch
            cfg, b = self.cfg, self.b
            delay = int(math.ceil(cfg.latency_s / cfg.dt_s))
            self._graph_controls = b.zeros((cfg.samples, cfg.horizon + delay, 2))
            self._graph_targets = b.zeros((4, cfg.horizon))
            self._graph_state = b.zeros((2,))

            def evaluate():
                return self._rollout(self._graph_controls,
                    {'speed': self._graph_state[0], 'steer': self._graph_state[1]},
                    (self._graph_targets[0], self._graph_targets[1]),
                    (self._graph_targets[2], self._graph_targets[3]))[0]

            with torch.cuda.device(b.device):
                stream = torch.cuda.Stream(device=b.device)
                stream.wait_stream(torch.cuda.current_stream())
                with torch.cuda.stream(stream):
                    for _ in range(3):
                        self._correlated_noise(cfg.samples, cfg.horizon)
                        evaluate()
                torch.cuda.current_stream().wait_stream(stream)
                torch.cuda.synchronize(b.device)
                graph = torch.cuda.CUDAGraph()
                with torch.cuda.graph(graph):
                    self._graph_cost = evaluate()
                self._graph = graph
        self._warmup_done = True

    def _score(self, controls, state0, goals, persons):
        if self._graph is None:
            return self._rollout(controls, state0, goals, persons)[0]
        self._graph_controls.copy_(controls)
        self._graph_targets.copy_(self.b.stack([*goals, *persons]))
        self._graph_state.copy_(self.b.array([state0['speed'], state0['steer']]))
        self._graph.replay()
        return self._graph_cost

    # ------------------------------------------------------------------
    # 求解
    # ------------------------------------------------------------------

    def solve(self, obstacles, person_xy, person_vel, speed, steer, usable=True):
        """算一步。

        obstacles   车体系下的障碍点 [(x, y), ...],已剔除车体自反射
        person_xy   人在后轴中心坐标系下的位置 (不是车头间距!)
        person_vel  人在车体系下的速度,来自 TargetLock.velocity_fl
        speed/steer 底盘当前实测车速与当前舵角
        usable      这一帧雷达是否可信。False 时直接判不可行 ——
                    没有证据不等于前面是空的
        """
        import time as _time
        t0 = _time.perf_counter()
        b, cfg = self.b, self.cfg

        if not all(math.isfinite(v) for v in (*person_xy, *person_vel, speed, steer)):
            self.reset()
            self.last = MPPISolution(0.0, 0.0, False, float('inf'), 0.0,
                                     'invalid_state')
            return self.last
        if not usable:
            self.reset()
            self.last = MPPISolution(0.0, steer, False, float('inf'), 0.0,
                                     'scan_unusable')
            return self.last

        if self.b.is_gpu and self.cfg.cuda_graph and not self._warmup_done:
            raise RuntimeError('MPPI CUDA graph requires warmup() before control starts')
        self.field.build(obstacles)
        # 逐 tick 播种:采样式控制器如果不可复现,现场根本没法复盘一次异常
        self._tick += 1
        b.seed(self._tick)

        n_delay = int(math.ceil(cfg.latency_s / cfg.dt_s))
        T, K = cfg.horizon, cfg.samples
        goals_x, goals_y, per_x, per_y = self.follow_points(person_xy, person_vel, speed)
        goals, persons = (goals_x, goals_y), (per_x, per_y)
        state0 = {'speed': speed, 'steer': steer}

        noise = self._correlated_noise(K, T)
        noise = noise * self._sigma
        nominal = self._nominal.reshape(1, T, 2)
        sampled = nominal + noise
        sampled = b.stack([
            b.clip(sampled[:, :, 0], cfg.min_speed_mps, cfg.max_speed_mps),
            b.clip(sampled[:, :, 1], -cfg.max_steer_rad, cfg.max_steer_rad),
        ], axis=2)

        if n_delay > 0:
            # 已经发出去、还没生效的指令:保持当前指令,状态照推但不计代价。
            # 不建模这段死时间的话,MPPI 会以为自己下一刻就能改变车的状态,
            # 于是系统性地打过头 —— 0.35s 在 dt=0.1 下是整整 3.5 步。
            held = b.stack([b.full((K, n_delay), speed),
                            b.full((K, n_delay), steer)], axis=2)
            controls = self._concat_time(held, sampled)
        else:
            controls = sampled

        cost = self._score(controls, state0, goals, persons)
        # 名义控制偏离项 (MPPI 标准的 gamma * u_nom^T Sigma^-1 eps),
        # 让解收敛回低噪声控制而不是在噪声里乱走。两个通道各按自己的方差归一。
        deviation = b.sum(b.sum(nominal * noise * self._inv_var, axis=2), axis=1)
        cost = cost + (cfg.nominal_shrink * cfg.temperature) * deviation

        best = b.amin(cost)
        weights = b.exp(-(cost - best) / max(cfg.temperature, EPS))
        total = b.sum(weights)
        weights = weights / (total + EPS)
        # Retain signed perturbations at the zero-speed boundary: averaging only
        # clipped, nonnegative speeds creates a forward-creep bias when blocked.
        update = b.sum(noise * weights.reshape(K, 1, 1), axis=0)
        nominal_new = self._nominal + update
        nominal_new = b.stack([
            b.clip(nominal_new[:, 0], cfg.min_speed_mps, cfg.max_speed_mps),
            b.clip(nominal_new[:, 1], -cfg.max_steer_rad, cfg.max_steer_rad),
        ], axis=1)

        # --- 确定性复验 ---
        # 加权平均出来的控制可能落在没有任何样本占据的区域。不自己再跑一遍
        # 就发下去,等于把「代价很高」当成了「不会发生」。
        obs_pts = self.field.verification_points
        plan = b.to_numpy(nominal_new)
        clearance = self._verify(plan, speed, steer, n_delay, obs_pts)
        vcost = b.item(best)  # Lowest sampled cost, including nominal regularization.
        feasible = clearance >= cfg.feasible_clearance_m
        reason = 'ok'

        if not feasible and cfg.allow_best_sample:
            # 退到代价最低的那条真实样本
            k = b.argmin(cost)
            cand = sampled[k]
            cplan = b.to_numpy(cand)
            cclear = self._verify(cplan, speed, steer, n_delay, obs_pts)
            if cclear >= cfg.feasible_clearance_m:
                plan, clearance = cplan, cclear
                nominal_new = cand
                feasible, reason = True, 'best_sample'

        out_v, out_d = map(float, plan[0])
        if not (np.isfinite(plan).all() and math.isfinite(vcost)):
            feasible = False
        if not feasible:
            # 没有一条可行:输出停车,并把名义序列拉回零。不拉的话下个周期
            # 还会拿同一条撞墙的计划去复验,车就永久瘫在这儿了。
            out_v, reason = 0.0, 'mppi_infeasible'
            out_d = steer
            nominal_new = b.zeros((T, 2))
        # 热启动按实际控制周期推进，末尾保持。
        self._nominal = self._advance_nominal(nominal_new)
        b.synchronize()  # solve_ms includes completion, not just queued GPU work.

        solution = MPPISolution(
            speed=out_v,
            steer=out_d,
            feasible=feasible,
            cost=vcost,
            min_clearance=clearance,
            reason=reason,
            solve_ms=(_time.perf_counter() - t0) * 1000.0,
            obstacles=self.field.obstacle_count,
        )
        self.last = solution
        return solution

    def _verification_poses(self, sequence, speed, steer, n_delay):
        """Single CPU trace: one device transfer per plan, with the same actuator model.

        Include current pose and the command latency interval. Checking only after
        the delay can miss an obstacle crossed before a new command takes effect.
        """
        cfg = self.cfg
        x = y = theta = 0.0
        v, d = speed, steer
        poses = [(x, y, theta)]
        for step in range(n_delay + min(cfg.feasible_steps, cfg.horizon)):
            dt = cfg.latency_s / n_delay if step < n_delay else cfg.dt_s
            vr, dr = (speed, steer) if step < n_delay else sequence[step-n_delay]
            v += max(-cfg.decel_limit_mps2*dt, min(cfg.accel_limit_mps2*dt, float(vr)-v))
            v = max(cfg.min_speed_mps, min(cfg.max_speed_mps, v))
            d += max(-cfg.steer_rate_radps*dt, min(cfg.steer_rate_radps*dt, float(dr)-d))
            d = max(-cfg.max_steer_rad, min(cfg.max_steer_rad, d))
            omega = (v / (cfg.wheelbase_m/math.tan(abs(d)) + .5*cfg.track_m)
                     * math.copysign(1.0, d)) if abs(d) >= 1e-4 else 0.0
            mid = theta + .5*omega*dt
            x, y, theta = x + v*math.cos(mid)*dt, y + v*math.sin(mid)*dt, theta + omega*dt
            poses.append((x, y, theta))
        return poses

    def _verify(self, sequence, speed, steer, n_delay, obstacles):
        cfg = self.cfg
        if not np.isfinite(sequence).all():
            return -float('inf')
        return rectangle_clearance(self._verification_poses(sequence, speed, steer, n_delay),
            obstacles, cfg.footprint_front_m, cfg.footprint_rear_m, cfg.footprint_half_width_m)

    def _advance_nominal(self, sequence):
        # At 20 Hz with a 150 ms prediction step, shift by 1/3 step, not 1 step.
        key = (self.cfg.horizon, self.cfg.control_dt_s, self.cfg.dt_s)
        if key != self._shift_key:
            positions = np.minimum(np.arange(self.cfg.horizon, dtype=np.float32)
                                   + self.cfg.control_dt_s/self.cfg.dt_s, self.cfg.horizon-1)
            self._shift_lo = self.b.to_long(self.b.array(np.floor(positions)))
            self._shift_hi = self.b.to_long(self.b.array(np.ceil(positions)))
            self._shift_alpha = self.b.array(positions-np.floor(positions)).reshape(-1, 1)
            self._shift_key = key
        return sequence[self._shift_lo]*(1-self._shift_alpha) + sequence[self._shift_hi]*self._shift_alpha

    def _correlated_noise(self, K, T):
        """沿时间轴相关的单位方差噪声。

        AR(1): e_t = beta*e_{t-1} + sqrt(1-beta^2)*w_t,稳态方差仍为 1,
        所以 sigma 的含义不变,只是样本变得"有主见"。
        """
        b, beta = self.b, float(self.cfg.noise_correlation)
        raw = b.randn((K, T, 2))
        if beta <= 0.0:
            return raw
        if b.kind == 'torch':
            return (raw.transpose(1, 2) @ self._noise_kernel.T).transpose(1, 2)
        beta = min(beta, 0.99)
        scale = math.sqrt(1.0 - beta * beta)
        cols = []
        prev = raw[:, 0, :]
        cols.append(prev)
        for t in range(1, T):
            prev = beta * prev + scale * raw[:, t, :]
            cols.append(prev)
        return b.stack(cols, axis=1)

    def _concat_time(self, head, tail):
        b = self.b
        if b.kind == 'torch':
            import torch as _t
            return _t.cat([head, tail], dim=1)
        import numpy as _np
        return _np.concatenate([head, tail], axis=1)

    def reset(self):
        self._nominal = self.b.zeros((self.cfg.horizon, 2))

    def describe(self):
        return (f"MPPI {self.b.describe()} K={self.cfg.samples} "
                f"T={self.cfg.horizon} dt={self.cfg.dt_s:.2f}s "
                f"前瞻={self.cfg.horizon * self.cfg.dt_s:.1f}s")
