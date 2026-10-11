#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""差速车（Gen5）定点画线（A→B 直线）：IMU 航向保持做内环 + RTK 横偏修正做外环。纯计算，不依赖 ROS。

为什么不用 LineTracker：它按曲率控制、只往前外推 0.1 s；Gen5 底盘转向晚约 0.44 s（2026-09-30 实测：陀螺 ≈
0.44 s 前的角速度指令 × 1.02–1.06），0.8 m/s 时来回大摆（仿真喷头横偏 95% 约 25 cm）。IMU 直行
（heading_hold.HeadingHold，带 Smith 预估）实车 0.8 m/s 车头稳在目标 ±0.8° 内，这里直接拿它当内环。

外环（每个控制周期，用 RTK 融合位姿）：
    e  = 喷头 / 水泵（车体系前 nozzle_x_m、左 nozzle_y_m）相对 A→B 线的横偏，线左为正
    δ  = −atan(k_e·e) − k_i·∫e ds，限幅 ±max_bias_deg        目标航向偏置：偏左就朝右偏一点开回来
    ∫e ds 只在前进、|e| < integ_band_m 时累积，并限幅（积分项贡献不超过 integ_max_deg）
坐标换算（内环在 IMU 自己的偏航系里跑：HDA536T 上电为 0，会慢慢漂）：
    偏移 = 融合航向 − IMU 偏航，低通（时间常数 offset_tau_s；刚开始按样本平均，攒够一个时间常数后按低通）
    目标 = 线方向 − 偏移 + δ（IMU 系）；偏移的残差（IMU 漂移跟不上、双天线安装残差）由外环积分吃掉
    起步 / 继续时目标从当时的 IMU 偏航出发，按 turn_rate_deg_s 限速转到上式：内环偏离目标超过 5° 就停车，
    不能一步跳过去。还没转到位（差 align_deg 以上）时车速不超过 align_radius_m × turn_rate，边慢慢走边转。
车速（规则同 LineTracker）：起步斜坡、横偏超过 slow_offset_m 降速、离 B 按减速度刹停。底盘车速也晚约 0.4 s，
离 B 的剩余距离扣掉已发出、还没执行的车速指令走的路（speed_delay_s），免得冲过 B。

s（沿线距离）、起止和起步检查按记 A、B 用的同一个点（track_x_m、track_y_m），横偏 e 按喷头量。差速车 LineMission
把两者都设成水泵（可以在侧面，2026-10-02 用户：作业都要让水泵走直线），A、B 也按水泵记。
参考航向：
    参考航向 = IMU 偏航 + 偏移，偏移 = 融合航向 − IMU 偏航，一直照学（低通）。喷头位置、车头误差都按参考航向算：
    融合用自己的航向从天线倒推后轴中心，所以先按融合航向还原出天线位置，再用参考航向推回后轴和喷头。
    2026-10-05 起不再判「航向跳变」：融合航向（以卫星测向为准）和 IMU 推的参考差得再多也不停、不冻结偏移，
    IMU 只是辅助（用户：「原来的检查必须删除，这种检查是不合理的」）。每次开始 / 继续时参考差超过 jump_deg 就直接重定偏移。
    位置质量退化或正在接回绝对位置时，外环积分冻结，原有积分与横偏比例纠正继续工作。
    起步转完那一刻偏移从头重新平均：起步转弯时 IMU 的刻度误差（转 30°、误差 2% 就差 0.6°）不带进来。
停车 / 中止与 LineTracker 相同：定位不可用 → PAUSED；|e| > 0.5 m 或车头偏 > 45° → ABORTED(off_line)；
内环自己停车映射成中止：IMU 偏离目标超过 5°（imu_deviation：被推歪、IMU 重启归零、方向装反）、
IMU 中断（imu_stale）；开始时没有 IMU 航向不能起步（no_imu）。

参数来源：内环现参数（kp 2.2 / ki 1.0 / kd 0.4，延迟 0.44 s）与 ±3° 是在 09-30 实测底盘模型上仿真挑的
（tests/test_line_cascade.py）；内环换成更温和的参数反而变差。外环 2026-10-02 由 k_e 0.6 / k_i 0.05 改为 0.8 / 0.3
（用户要水泵横偏 1.5 cm 以内）：旧外环纠不回每趟 −0.5～+1.2° 的测向偏差，按 10-02 实车数据校准的四种底盘仿真
95% 约 1.25 cm、整条线平均偏 0.8 cm；新外环 95% 约 0.45–0.55 cm、最大约 1 cm，测向偏 2° 时最大 1.4 cm。
k_e 1.3 / k_i 0.4（路线长直线外环那组）精度只好 0.05 cm，但底盘延迟到 0.5 s 就会因内环偏离 5° 中止，0.8 / 0.3
到 0.5 s 仍不中止（实测延迟 0.28–0.44 s）。停车瞄准 B 本身（end_aim_m），车速延迟按 10-01 实测 0.33 s 预估。
"""
import dataclasses
import math
from collections import deque
from dataclasses import dataclass, field

from heading_hold import HeadingHold, HoldConfig
from line_tracker import LineState, lateral_integral_allowed, wrap
from robot_core.kinematics import ChassisGeometry

DEG = math.pi / 180.0

# 中止 / 起步不了的原因（mission 的 message 用；off_line 等与 LineTracker 相同的代码由 app 翻译）
REASON_TEXT = {
    'imu_deviation': 'IMU 航向偏离目标超过 5°（被推歪、IMU 重启归零或装反）',
    'imu_stale': 'IMU 数据中断',
    'no_imu': '没有 IMU 航向（/imu/data）',
    # 路线 / 场地作业（path_tracker.SectionTracker）的中止原因；原来直接把英文代码显示给用户（2026-10-04）
    'pivot_timeout': '原地转向没转到位（转了 20 s，或补转 2 次仍差 5° 以上）',
    'not_stopped_at_cusp': '换向前车没停稳',
    'off_line': '偏离路线太远',
    'paint_start_unready': '起步尚未收稳，已停止起喷；请继续作业预览重新接入原线头',
    'paint_start_off_line': '到线头时喷头离线超过 5 cm',
}


@dataclass(frozen=True)
class CascadeConfig:
    cruise_mps: float = 0.8
    # 以下几项与 LineConfig 同名同义，LineMission 从 LineConfig 抄过来（SHARED_FIELDS）
    accel_mps2: float = 0.5
    decel_mps2: float = 0.5
    min_mps: float = 0.06            # 终点前的蠕行速度，保证真的走到 B
    track_x_m: float = 0.0           # 沿线距离 s、起止按这个点（车体系 x），与记 A、B 的点相同
    track_y_m: float = 0.0           # 同上，车体系 y（左为正）
    start_max_offset_m: float = 0.5
    start_max_heading_rad: float = math.radians(30.0)
    start_before_a_m: float = 3.0    # 允许从 A 点后方最多这么远起步（助跑）
    abort_offset_m: float = 0.5
    abort_heading_rad: float = math.radians(45.0)
    slow_offset_m: float = 0.10      # 横偏超过它开始降速
    stop_tol_m: float = 0.02         # 与 LineConfig 同名；这里不再用来判到达（按 end_aim_m 瞄准 B 停）
    min_line_m: float = 1.0
    # 外环
    nozzle_x_m: float = 0.30         # 横偏量喷头：后轴前多少米
    nozzle_y_m: float = 0.0          # 喷头在车中线左多少米（右为负）
    # 2026-10-02 用户要水泵横偏 1.5 cm 以内：0.6 / 0.05 太软，每趟 −0.5～+1.2° 的测向偏差纠不回来（按 10-02 实车数据
    # 校准的四种底盘仿真：95% 1.25 cm、整条线平均偏 0.8 cm）；改成与路线作业长直线外环同一组（path_tracker
    # differential_config straight_outer_k_e / k_i）后 95% 约 0.5 cm、最差一趟 0.65 cm
    k_e: float = 0.8                 # 横偏 → 目标航向偏置，1/m
    k_i: float = 0.3                 # 横偏积分，1/m²
    max_bias_deg: float = 3.0        # 目标航向偏置限幅
    integ_band_m: float = 0.10       # |e| 小于它才积分（切入阶段不积）
    integ_max_deg: float = 2.5       # 积分项贡献的航向偏置上限（要盖得住测向偏差，实车见过 1.2°）
    offset_tau_s: float = 10.0       # 融合航向 − IMU 偏航的低通时间常数
    # 航向冲突期间不学习偏移；能否继续由融合质量和任务层喷头误差预算判断
    jump_deg: float = 0.5            # 开始 / 继续时参考航向离融合航向超过它就直接重定偏移
    # 目标限速转向（起步 / 继续时从当时的车头转到线方向）
    turn_rate_deg_s: float = 3.0     # 慢走时内环不积分，跟 r 的斜坡落后约 r/kp + r × 延迟 ≈ 2.7°（6°/s 会过 5° 停车）
    align_deg: float = 1.0           # 目标还差这么多没转到位就算「在对准」
    align_radius_m: float = 1.0      # 对准时车速上限 = 这个半径 × turn_rate（约 0.1 m/s）
    speed_delay_s: float = 0.33      # 底盘车速响应延迟，到 B 的刹停按它预估（2026-10-01 实车 26 次段尾停车等效延迟中位 0.33 s，
                                     # 四分位 0.29–0.37；09-30 估的 0.40 s 偏大，加上按 B 前 stop_tol 停，水泵平均停在 B 前约 4 cm）
    end_aim_m: float = 0.0           # 水泵停车瞄准 B 后多少米（0 = 正好停在 B；以前按 B 前 stop_tol_m 停，系统性少画 2 cm）
    hold: HoldConfig = field(default_factory=HoldConfig)    # 内环参数（与 IMU 直行相同）


# 与 LineConfig 同名、按同一规则用的字段：LineMission 建差速车定点画线时从当前 LineConfig 抄
SHARED_FIELDS = ('accel_mps2', 'decel_mps2', 'min_mps', 'track_x_m', 'start_max_offset_m', 'start_max_heading_rad',
                 'start_before_a_m', 'abort_offset_m', 'abort_heading_rad', 'slow_offset_m', 'stop_tol_m',
                 'min_line_m')


def learn_heading_offset(offset, count, difference, dt, tau):
    """A/B 与路线直线共用的 IMU/绝对航向偏移学习；先平均，再低通。"""
    count += 1
    alpha = max(min(1.0, dt / tau), 1.0 / count)
    correction = alpha * difference
    return wrap(offset + correction), count, correction


def lateral_heading_step(cfg, e, distance, integral, target, line_yaw, offset, dt, frozen):
    """A/B 的横偏积分、目标偏角和目标限速；distance 必须是沿线位置增量。"""
    if not frozen and cfg.k_i > 0.0 and abs(e) < cfg.integ_band_m:
        lim = cfg.integ_max_deg * DEG / cfg.k_i
        integral = max(-lim, min(lim, integral + e * distance))
    bias = -math.atan(cfg.k_e * e) - cfg.k_i * integral
    lim = cfg.max_bias_deg * DEG
    bias = max(-lim, min(lim, bias))
    desired = wrap(line_yaw - offset + bias)
    step = cfg.turn_rate_deg_s * DEG * dt
    target = wrap(target + max(-step, min(step, wrap(desired - target))))
    aligning = abs(wrap(desired - target)) > cfg.align_deg * DEG
    return integral, bias, target, aligning


# ---- 直线 IMU 为主、卫星辅助（路线 / 场地作业的前进长直线） ----
# 2026-10-06 用户：IMU 直行（只按 IMU 航向开）中途车头稳在 ±1° 内，比卫星为主的直线走得好；「只要和 IMU 推算对不上，
# 直线上也听 IMU 的，靠 imu 走」，IMU 默认不漂。上面的 A/B 控制律里卫星从三处拧车头：IMU 与融合航向的偏移一直按
# 10 s 低通跟着卫星测向学、横偏 1 cm 车头就偏约 0.5°（最多 3°、每秒最快 3°）、横偏积分。这里改成：
#   1. 偏移 = 融合航向 − IMU 偏航，只在接管前学（车头几乎不转时取平均，离平均超过 offset_reject_deg 的样本丢掉），
#      攒够 lock_samples 个就锁住，这条线上不再跟卫星测向变；
#   2. 喷头横偏按「IMU 航向 × 走过的路」推算，卫星横偏只按距离尺度慢慢融进来（互补滤波），同时从卫星轨迹估出 IMU
#      参考航向还差多少（β），车头目标把 β 减掉。每条线都要靠卫星轨迹重新对：10-06 上午 18 趟路线 27 段直线实测，
#      双天线测向比实际走出来的方向偏 −0.66～+0.63°（往东 −0.2°、往西 +0.2°，跟方向有关，平均不掉），IMU 与实际
#      方向的差同一条线上变化 < 0.1°，但每次原地掉头 180° 后变 0.3–0.5°（转得多，刻度差一点点就出来）。所以助跑
#      和推算开始 fast_m 内按 fast_observer_m 快对准，上线以后按 observer_m；
#   3. 卫星横偏和推算差超过 jump_m 就当卫星跳了：不再用卫星，按 IMU 推算接着开、接着喷，不报错、不停；
#      差回到 rejoin_m 以内并持续 rejoin_s 才重新用；一直对不上就这条线按 IMU 走完，下一条线重新对；
#   4. 往回拉按推算横偏（deadband_m 以内不拉），车头最多偏 max_bias_deg，上线以后目标车头每秒最多改 rate_deg_s；
#      开喷后 head_m 内按 head_k_e / head_max_bias_deg（见 paint_pull）。
# 卫星位置不可信（推算中、降权、正在接回）时同 3，修正冻结。融合与地图显示不变，仍以卫星为准。
# 参数按 10-02 实车校准的仿真挑（会话 scratchpad sim/：A/C/E 三种底盘、0.8/0.5 m/s、1.2 m 助跑、测向每趟偏 ±0.6°）：
# 正常时真喷头 p95 与原 A/B 持平（0.57–0.68 vs 0.63–0.71 cm）、最差一趟 0.74–1.01 vs 1.09–1.27 cm、线头最差
# 1.03–1.26 vs 1.26–1.46 cm，车身小摆略小；测向偏到 −1～+1.2° 时 p95 0.63–0.70 vs 1.00–1.09 cm；卫星横跳 5 cm
# 3 s 或一直不回来，原 A/B 跟着拐、真喷头偏到 7 cm，这里 ≤ 1.6 cm；测向跳 3° 2 s 1.42 → 0.98 cm。
# 卫星噪声加倍（4 mm）时中位略差（0.8 m/s 1.04–1.07 vs 0.95 cm）、最差一趟仍好。


@dataclass(frozen=True)
class ImuFirstConfig:
    observer_m: float = 1.0          # 卫星横偏融进 IMU 推算的距离尺度，m：越大越信 IMU
    fast_observer_m: float = 0.4     # 对准阶段（助跑、推算开始 fast_m 内）的距离尺度：卫星快速把 IMU 方向对到线上
    fast_m: float = 2.0              # 推算开始后这么远内算对准阶段（另：助跑还没到线头也算）
    max_heading_fix_deg: float = 3.0  # β（卫星轨迹看出来的参考航向误差）上限
    deadband_m: float = 0.002        # 喷头横偏（推算）在这以内不往回拉
    # 画线时（线头之后）往回拉：横偏 → 车头偏置 k_e（1/m），车头最多偏 max_bias_deg。10-08 用户「慢慢修」：原来 0.8 / 1°，
    # 10-06 23:47 那条 37.8 m 线 22–24 m 推算偏右 1 cm 多，车头目标 2 m 内转 +0.55°，3 m 横挪约 2.5 cm，地上看得见一个弯；
    # 卫星那几米读大了约 1 cm，又冲到左边。改成 1.5–2 cm 的偏差 4–6 m 内平缓收回
    k_e: float = 0.4
    max_bias_deg: float = 0.3
    rate_deg_s: float = 1.0          # 上线以后目标车头每秒最多改这么多（原 A/B 3°/s）
    target_accel_deg_s2: float = 4.0  # RTK 拟合模式的目标角加速度；及时响应，同时避免拟合点更新时突变
    # 线头段：这条直线开喷处（线头 / 弯道出口 / 暂停后继续处）后 head_m 内仍按原来的 head_k_e / head_max_bias_deg
    # （线头精度不能变差，用户规矩），再 head_blend_m 内线性过渡到上面的 k_e / max_bias_deg；不在喷（转场段、停喷后的收尾）一直按原来的
    head_k_e: float = 0.8
    head_max_bias_deg: float = 1.0
    head_m: float = 3.0
    head_blend_m: float = 2.0
    jump_m: float = 0.02             # 卫星横偏和推算差超过它：卫星跳了，不用
    rejoin_m: float = 0.01           # 差回到这以内……
    rejoin_s: float = 1.0            # ……持续这么久才重新用卫星
    trust_from_m: float = 0.5        # 推算开始后这么远内卫星都用（先贴上卫星，不判跳）
    offset_reject_deg: float = 0.5   # 学偏移时离当前平均超过它的样本丢掉（卫星测向跳了）
    offset_min_samples: int = 10     # 攒够这么多才开始丢
    lock_samples: int = 50           # 攒够这么多（50 Hz 约 1 s）锁住偏移，开始推算
    # 车头转得比它快的样本不学偏移（挡原地转）。原来 2°/s（怕卫星测向比 IMU 晚）：10-06 晚 9 趟实车融合航向和 IMU 对
    # 陀螺回归，滞后 −4～+2 ms、高频差 0.02–0.03°（融合本来就用同一个陀螺推），不用挡；而助跑起步车头来回甩 3–5°/s、
    # 峰值 10°/s，2°/s 把样本大多丢掉，偏移喷线后 0.4–1.4 m 才锁住，线头 1 m 多由原路径跟踪开，跟着来回摆
    # （线头 2 m 弯 0.6–1.5 cm，线中 0.1–0.4 cm）
    gyro_max_deg_s: float = 15.0
    # 2026-10-06 下午用户：「基线变成 0.4 以下还在用卫星校正导致画出蛇形」。15:57 / 16:01 两趟双天线基线掉到
    # 0.32–0.38 m（标称 0.41），测向和 IMU 来回差 7–12°，跑的是 8411c884（还没有基线门槛）：卫星修正跟着纠，横偏来回 7.6–9.3 cm。用户又定：高于 0.42 m
    # 同样处理。基线不在 [min_baseline_m, max_baseline_m]（或刚出过、baseline_hold_s 内）：卫星修正全部冻结、偏移不学，
    # 只按 IMU 走（照常喷，同卫星跳点）
    min_baseline_m: float = 0.40
    max_baseline_m: float = 0.42
    baseline_hold_s: float = 1.0
    # 连续这么多帧测向出界才冻结（10-06 傍晚实车：基线中位 0.412 m，零星一两帧 0.395–0.399 / 0.420–0.432，每帧冻 1 s
    # 冻掉直线上中位 37%、开头 57–72%；零星出界那段卫星横偏去趋势 std 0.46 cm，和基线正常 0.41 cm 差不多，连续出界尾巴
    # 1.44 cm（95%）明显差）。3：冻结中位 37% → 14%，下午那种连续掉到 0.32–0.38 m 照样第三帧（约 0.4 s）就冻
    baseline_bad_frames: int = 3
    # 一条线开头基线就不对、偏移学不到：这段已攒够 offset_min_samples 个就用它，否则借上一条线对好的角度（偏移 + β）
    # 先开，基线正常了再按卫星轨迹纠（用户 10-06 定的；原地掉头 180° 后 IMU 与地面方向的差会变 0.3–0.5°）。
    # 借的不能超过 borrow_max_age_s，和此刻融合航向 − IMU 偏航差不能超过 borrow_max_gap_deg（挡 IMU 重启归零）
    borrow_max_age_s: float = 600.0
    borrow_max_gap_deg: float = 10.0
    # 2026-10-06 晚用户：「直接靠 imu 画线，卫星只看啥时候停水泵，不考虑 imu 偏移」。False：IMU 接管以后卫星不再修正
    # 横偏、不估 β，喷头横偏只按 IMU 航向 × 走过的路推算；IMU → 线方向的换算只用接管前双天线测向学的偏移
    # （学不到时同样借上一条线的）。接管前（助跑、弯道出来那段本来就由卫星开）卫星照常定横偏，β 不估。
    # 沿线走到哪、什么时候停水泵、什么时候停车仍按卫星位置
    rtk_trim: bool = True
    # 2026-10-06 晚用户：「中途卫星好的时候接入修正，让数据在两者更好」。rtk_trim 关着、rtk_fit 开：卫星不逐拍拉横偏，
    # 把这条线到目前为止「卫星横偏 − IMU 推算」按走过的路攒起来（每 fit_bin_m 平均成一个点），拟合 起点横偏 a +
    # 角度差 b × 路程（卡尔曼，先验 a ~ fit_prior_m、b ~ fit_prior_deg）；喷头横偏 = IMU 推算 + a + b × 路程，β = b。
    # 数据越多越稳，角度纠准以后基本不动，线保持 IMU 那样直。卫星不可信 / 基线不对 / 跳点（同上）时不攒，只按 IMU
    rtk_fit: bool = False
    fit_bin_m: float = 0.25
    fit_noise_m: float = 0.004       # 一个点（fit_bin_m 平均）的卫星横偏误差；卫星噪声前后相关，取得比单拍大
    fit_prior_m: float = 0.005
    fit_prior_deg: float = 0.5       # 双天线测向比实际方向偏 −0.66～+0.63°（10-06 上午实测）
    # rtk_fit 的跳点只认「短距离内突然跳开」：和 jump_window_m 以前（中间断过就取断之前）最近用过的点比，扣掉已拟合的角度，
    # 差超过 jump_m + tan(jump_slope_deg) × 两点间距才算跳。10-06 傍晚实车 B→A 三趟：掉头后基线冻结前 3–5 m，换算错
    # 0.4–0.9°，卫星和推算按距离均匀拉开，基线一好差已 3.5–4.5 cm，原来按「差 > 2 cm」判成跳点，整条线没再用卫星，偏到 18–38 cm
    jump_window_m: float = 0.4
    jump_slope_deg: float = 1.5
    # 开头换算的误差（双天线和上一条线带过来的加权后，见 AlignmentConfig）比双天线学满时小多少，拟合角度的先验就收多少，
    # 夹在 [fit_prior_min_deg, fit_prior_max_deg]；这条线拟合到误差 ≤ carry_fit_sigma_deg 才记下 c 给下一条线带
    # 下限 0.15 → 0.5°（10-07）：起步换算实测会差 0.3–0.9°（10-07 上午同一方向三趟），比它自己算的误差（0.2–0.3°）大
    fit_prior_min_deg: float = 0.5
    fit_prior_max_deg: float = 1.5
    carry_fit_sigma_deg: float = 0.10
    # 2026-10-06 傍晚 eaf2d772 两趟 101 m 线：App 上喷线（卫星）已明显偏离路线，车却不纠——一条线攒 150+ 点后拟合误差
    # 只剩 0.0007°，不再改；本场 IMU 换算漂约 −0.37°/min。用户：「喷的轨迹偏移了卫星轨迹却没有修正」要解决，同时保持 IMU
    # 的直线度。拟合改成跟踪：修正量 c（真实喷头横偏 − IMU 推算）和角度 b 都允许按距离随机游走（fit_walk_*），角度跟得上
    # 慢漂；用到推算上的修正和角度再按距离限速（fit_rate_m / fit_beta_rate_deg），一个坏点拉不动车，真实偏移一直在纠。
    # 卫星用不用：固定解 + 双天线基线在 [fit_min_baseline_m, fit_max_baseline_m]（用户定的可信规则 0.30–0.50 m，连续
    # baseline_bad_frames 帧出界才停），加上和 IMU 对不上的突然跳点不用；0.40–0.42 只管学开头换算
    # 修正量、角度本身的随机游走收小（漂移交给下面的漂移速度 r）：仿真今天的 101 m 线 IMU 不漂 / 漂 −0.4°/min 都是
    # p95 0.65 cm（游走 2 mm / 0.03° 时不漂 1.4 cm——卫星噪声被当成真变化）
    fit_walk_c_m: float = 0.0005      # 修正量随机游走，m/√m
    fit_walk_b_deg: float = 0.005     # 角度随机游走，°/√m
    # 角度本身每米变多少（IMU 漂移速度）也一起估：卫星好的时候学出来，进了卫星差的地方按它往前推（傍晚仿真今天的 101 m 线：
    # 坏区 60–97 m、IMU 漂 −0.4°/min 时，角度停在进坏区那一刻，到 97 m 又漂 0.3°、横偏 11 cm；IMU 不漂时学出来≈0）
    # 仿真今天的 101 m 线（坏区 60–97 m）：先验 0.01 / 游走 0.001 → IMU 漂 −0.4°/min p95 12.9 → 2.6 cm，但不漂时
    # 1.0 → 1.7 cm（把卫星噪声当漂移）；0.005 / 0.0003 → 5.9 / 1.4 cm（折中）；6 cm 鼓包 30 m 后都回 0
    fit_prior_rate_deg: float = 0.005  # 漂移速度先验，°/m（0.4°/min ≈ 0.008°/m）
    fit_walk_r_deg: float = 0.0003     # 漂移速度随机游走，°/m/√m
    fit_max_rate_deg: float = 0.05    # 漂移速度上限，°/m
    # 推算上的修正每走 1 m 最多变这么多，m。10-08 用户：每 0.8 m 最多 1.5 cm（原来每 1 m 1 cm）
    fit_rate_m: float = 0.015 / 0.8
    fit_beta_rate_deg: float = 0.15   # 用到目标车头上的角度每走 1 m 最多变这么多，°
    # 前 fit_start_m（推算开始算，含助跑）角度每米最多变 fit_start_beta_rate_deg：10-07 上午起步换算差 0.9°，拟合早就看出来了，
    # 按 0.15°/m 用了 6 m 才用上，线头后 2 m 偏到 2.8 cm。拟合点攒够 fit_start_points_m 以后才放快（一两个点定不了角度）
    fit_start_m: float = 6.0
    fit_start_beta_rate_deg: float = 0.5
    fit_start_points_m: float = 1.0
    # 拟合起点（第一拍的卫星横偏）那一拍卫星不可信、第一个可信点和「起点 + 角度先验」的预测差超过 fit_anchor_sigma 倍标准差：
    # 起点错了，这一点重新当起点（见 ImuFirstGuide._fit_anchor）。10-07 仿真（遮挡较多 100 m）：暂停后重开那拍在多路径区、
    # 起点错 10 cm，第一个可信点把它算成角度错 2.3°，车斜着开出去。不一概重定：倒车助跑里第一拍常被判不可信但其实是准的，
    # 它和后面几米一起推起步角度（坏区前段·第二条线仿真：一概重定 3 m 后中位 3.5 → 5.5 cm）
    fit_anchor_sigma: float = 3.0
    # 10-07 用户：「app 地图显示已经偏离轨迹，但是没有及时纠正」。游走小（一个坏点拉不动车），持续的真偏差也被当噪声慢慢消化
    # （10-07 上午 14–17 m 卫星和拟合差 1.1–1.6 cm 持续 3 m 不动）：卫星点和拟合连续 persist_m 都在同一边差超过 persist_e_m，
    # 增加位置修正量的不确定度让拟合跟上；不把一个位置台阶直接当成角度误差。
    # 角度仍由连续沿程观测学习，推算上按 fit_rate_m / 角度限速拉回。
    persist_m: float = 1.0
    persist_e_m: float = 0.01
    fit_min_baseline_m: float = 0.30
    fit_max_baseline_m: float = 0.50
    # 跳点只挡短暂的：跳开后 jump_accept_s 内没回来、但新位置自己稳定（扣掉直线趋势后散布 ≤ rejoin_m），就认卫星，照常慢慢纠
    # （傍晚回放：原来跳一次就一直只靠 IMU，推算和卫星越拉越远）
    # 固定解一下跳出很远（> jump_accept_max_m）多半是整周解错了，不认，按 IMU 走
    jump_accept_s: float = 3.0
    jump_accept_max_m: float = 0.10
    # 卫星位置可靠才拿来纠（用户 10-06 晚定「四道关」：走直靠 IMU，卫星只在可靠时慢慢纠）。傍晚 100 m 线中间那块和同一块的
    # 纯 IMU 直行：卫星横偏几米内来回 15–22 cm，车头（IMU、双天线）稳在零点几度——卫星在跳。那块 vs 正常路段：接收机
    # σ = hypot(σn, σe) > 2.5 cm 98% vs 0%；两天线位置距离误差 > 3 cm 48% vs 0%；卫星 < 18 颗 46% vs 0%。连续
    # fit_quality_frames 帧不合格就停用，之后连续合格 fit_settle_s 才再用
    fit_max_sigma_m: float = 0.025
    # 两天线距离误差 3 → 5 cm、超限后恢复 2 → 0.5 s（10-07 用户定）：10-07 上午 25 m 线 17.7–22.7 m 误差零星 3.3–3.9 cm，
    # σ 1.8–1.9 cm、22–23 颗、基线 0.41 都正常，3 cm 一超就停、再等 2 s，5 m 没用卫星，偏了 3 cm 没纠。
    # 10-06 那块坏区靠 σ > 2.5 cm（98%）照样挡住
    fit_max_pair_m: float = 0.05
    fit_pair_settle_s: float = 0.5
    # 卫星数（主天线）：用户 10-06 晚定 20；10-08 用户改：低于 17 才不信。17–19 颗只在 σ ≤ fit_low_sats_max_sigma_m、
    # 两天线距离误差 < fit_low_sats_max_pair_m、固定解时照常信（10-06 16:32 那趟靠楼一侧主天线掉到 17–19 颗，σ ≤ 2.0 cm、
    # 两天线误差 ≤ 2.9 cm、一直固定解，卫星本来是好的，被 20 颗挡掉 18–26 m 没纠，按 IMU 偏了 2.5 cm）
    fit_min_sats: int = 17
    fit_full_sats: int = 20
    fit_low_sats_max_sigma_m: float = 0.027    # 10-08 下午用户 2.0 → 2.7 cm：10-06 22:08 那条线 8–14 m 17–19 颗、σ 2.2–2.4 cm，
                                                # 卫星报的往右 2.7 cm 和地上实际一致，被 2.0 挡掉只按 IMU 走偏了
    fit_low_sats_max_pair_m: float = 0.03
    # 两根天线对不上（位置距离误差 ≥ antenna_pick_pair_m）时，用和 IMU 推算对得上的那根（10-08 用户）。10-06 22:08 那条线
    # 15–20 m 主天线被干扰往左 5–9 cm、两天线差到 9–10 cm，副天线和 IMU 都说车没偏（地上实际也没偏）。
    # 副天线横偏 − 主天线横偏 在两根对得上时慢慢学成固定差（antenna_offset_samples 帧以上才算学好），用副天线时扣掉；
    # 一根和推算差 ≤ antenna_agree_m、另一根差超过 antenna_agree_m + antenna_margin_m 才选，不然照旧（≥ 20 颗且误差
    # ≤ fit_max_pair_m 用主天线，否则不用卫星）
    antenna_pick_pair_m: float = 0.03
    antenna_agree_m: float = 0.015
    antenna_margin_m: float = 0.01
    antenna_offset_samples: int = 10
    antenna_fresh_s: float = 0.5
    # 助跑段（线头之前，还没喷，10-08 用户）：线头偏 0.5–2.1 cm（助跑起点就偏 1–3 cm，锁车头 1 m 后只剩 ~1.7 m 往回拉，
    # 按 k_e 0.8 / 1° 每米只拉回 ~55%；推算还在「慢慢修」，16:29 线头车以为 0.8 cm、卫星 2.1 cm）。助跑段推算直接往卫星靠
    # （每走 runup_rtk_tau_m 靠拢 63%，卫星可用且没判跳才靠），往回拉按 runup_k_e / runup_max_bias_deg，目标车头最快
    # runup_rate_deg_s、角加速度 runup_target_accel_deg_s2；横偏收小车头偏置跟着收，线头处短距离交接后恢复喷线策略。
    runup_rtk_tau_m: float = 0.3
    runup_k_e: float = 2.0
    runup_max_bias_deg: float = 2.0
    runup_rate_deg_s: float = 3.0
    runup_target_accel_deg_s2: float = 8.0
    # 到线头不能同时撤掉位置对齐和纠偏力度；在最初半米连续交给喷线控制。
    # 不改变路径、助跑长度或开关喷边界。
    runup_blend_m: float = 0.5
    fit_quality_frames: int = 2
    fit_settle_s: float = 2.0

    def __post_init__(self):
        for name in ('observer_m', 'fast_observer_m', 'max_heading_fix_deg', 'k_e', 'max_bias_deg', 'rate_deg_s', 'jump_m',
                     'rejoin_m', 'rejoin_s', 'offset_reject_deg', 'gyro_max_deg_s', 'fit_bin_m', 'fit_noise_m',
                     'fit_prior_m', 'fit_prior_deg', 'jump_window_m', 'jump_slope_deg', 'fit_prior_min_deg',
                     'fit_prior_max_deg', 'carry_fit_sigma_deg', 'fit_rate_m', 'fit_beta_rate_deg', 'jump_accept_s', 'jump_accept_max_m',
                     'fit_max_sigma_m', 'fit_max_pair_m', 'fit_settle_s', 'fit_pair_settle_s',
                     'fit_low_sats_max_sigma_m', 'fit_low_sats_max_pair_m',
                     'antenna_pick_pair_m', 'antenna_agree_m', 'antenna_margin_m', 'antenna_fresh_s',
                     'runup_rtk_tau_m', 'runup_k_e', 'runup_max_bias_deg', 'runup_rate_deg_s', 'runup_target_accel_deg_s2', 'runup_blend_m',
                     'fit_prior_rate_deg', 'fit_walk_r_deg', 'fit_max_rate_deg', 'fit_start_beta_rate_deg',
                     'persist_m', 'persist_e_m', 'fit_anchor_sigma', 'target_accel_deg_s2',
                     'head_k_e', 'head_max_bias_deg'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' 必须是有限正数')
        for name in ('deadband_m', 'trust_from_m', 'fast_m', 'min_baseline_m', 'max_baseline_m', 'baseline_hold_s',
                     'fit_walk_c_m', 'fit_walk_b_deg', 'fit_min_baseline_m', 'fit_max_baseline_m', 'fit_start_m',
                     'fit_start_points_m', 'head_m', 'head_blend_m',
                     'borrow_max_age_s', 'borrow_max_gap_deg'):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(name + ' 必须是有限非负数')
        if self.min_baseline_m > self.max_baseline_m or self.fit_min_baseline_m > self.fit_max_baseline_m:
            raise ValueError('基线下限不能大于上限')
        if self.rejoin_m > self.jump_m:
            raise ValueError('rejoin_m 不能大于 jump_m')
        if type(self.rtk_trim) is not bool or type(self.rtk_fit) is not bool:
            raise ValueError('rtk_trim、rtk_fit 必须是 True 或 False')
        if self.rtk_trim and self.rtk_fit:
            raise ValueError('rtk_trim 和 rtk_fit 只能开一个')
        for name in ('offset_min_samples', 'lock_samples', 'baseline_bad_frames', 'fit_min_sats', 'fit_full_sats',
                     'fit_quality_frames', 'antenna_offset_samples'):
            if type(getattr(self, name)) is not int or getattr(self, name) < 1:
                raise ValueError(name + ' 必须是正整数')
        if self.fit_min_sats > self.fit_full_sats:
            raise ValueError('fit_min_sats 不能大于 fit_full_sats')


class ImuFirstGuide:
    """一条直线（一个直线小段）的 IMU 偏移、喷头横偏推算和卫星跳点判断；纯计算。换线 / 暂停后 reset。"""

    def __init__(self, config=ImuFirstConfig()):
        self.cfg = config
        self.reset()

    def reset(self):
        self.offset = None           # 融合航向 − IMU 偏航，rad
        self.offset_n = 0
        self.offset_rejected = 0
        self.locked = False
        self.source = None           # learned 这段学的 / partial 基线坏了用已攒的 / borrowed 借上一条线的
        self.e = None                # 喷头横偏估计，m（线左为正）
        self.beta = 0.0              # 真实车头 − IMU 参考航向（偏航 + 偏移），rad
        self.mode = 'rtk'            # rtk：卫星在修正；imu：卫星和推算对不上，只按 IMU
        self.innovation = None       # 卫星横偏 − 推算，m
        self.jumps = 0
        self.imu_only_s = 0.0
        self._agree_s = 0.0
        self.fast = True
        self._rel = None
        self._travel = 0.0
        self.fit_points = 0          # rtk_fit：已拟合的点数
        self._fit_hist = deque()     # rtk_fit：最近用过的 (路程, 卫星横偏 − IMU 推算横移)，判跳点的参照
        self.token = None            # ImuAlignment.begin_line 给的这条线的号
        self.start = None            # 开头换算怎么定的：dual 双天线 / carried 带过来的 / sigma 误差 / weight 带过来的权重
        self._prior_scale = 1.0      # 拟合角度先验相对 fit_prior_deg 的倍数（开头换算越准越小）
        self._jump_ref = None        # rtk_fit：跳开前的参照 (路程, 值, 当时拟合的角度)
        self._jump_after = []        # rtk_fit：跳开以后的 (路程, 值)，看新位置稳不稳
        self._jump_s = 0.0           # rtk_fit：跳开以后卫星可用的时间
        self._jump_step = 0.0        # rtk_fit：跳开那一下差了多少
        self.jumps_accepted = 0
        self._persist = None         # rtk_fit：最近同一边差超过 persist_e_m / 2 的连续点 (路程, 差)
        self.persists = 0            # 认过几次真偏
        self.rtk_accepted = False    # 当前观测实际通过质量及跳点门，供跨线标定使用
        self.target_rate = 0.0       # 平滑目标的连续角速度，rad/s
        self._fit = None             # rtk_fit：(起点横偏, IMU 推算横移, [a, b], P, 当前这一格 [Σ路程, Σ残差, 个数, 起点])

    @property
    def travel(self):
        """推算开始后往前走了多远，m。"""
        return self._travel

    def learn_offset(self, difference, gyro):
        """接管前每拍一个样本：difference = 融合航向 − IMU 偏航（同一时刻），gyro 陀螺 z（rad/s）。返回是否用上。"""
        cfg = self.cfg
        if self.locked or not math.isfinite(difference) or not math.isfinite(gyro):
            return False
        if abs(gyro) > cfg.gyro_max_deg_s * DEG:
            return False
        if self.offset is None:
            self.offset, self.offset_n = wrap(difference), 1
        else:
            d = wrap(difference - self.offset)
            if self.offset_n >= cfg.offset_min_samples and abs(d) > cfg.offset_reject_deg * DEG:
                self.offset_rejected += 1
                return False
            self.offset_n += 1
            self.offset = wrap(self.offset + d / self.offset_n)
        if self.offset_n >= cfg.lock_samples:
            self.locked, self.source = True, 'learned'
        return True

    def lock_without_baseline(self, alignment, now, difference):
        """基线不对、偏移还没锁时调用：已攒够 offset_min_samples 个就用已攒的，否则借 alignment（ImuAlignment）里
        上一条线对好的角度。difference = 此刻融合航向 − IMU 偏航。返回是否锁上。"""
        cfg = self.cfg
        if self.locked:
            return True
        if self.offset is not None and self.offset_n >= cfg.offset_min_samples:
            self.locked, self.source = True, 'partial'
            return True
        value = None if alignment is None else alignment.recall(now, difference, cfg.borrow_max_age_s,
                                                                 cfg.borrow_max_gap_deg)
        if value is None:
            return False
        self.offset, self.beta, self.locked, self.source = value, 0.0, True, 'borrowed'
        return True

    def blend_start(self, predicted, align_cfg, rate=None):
        """偏移刚锁住时调用：predicted = ImuAlignment.predict(此刻)，(c, σ) rad 或 None。learned / partial 时和双天线学的
        按误差加权（双天线误差 = dual_sigma_deg × √(lock_samples / 样本数)）；两者差 > max_gap_deg 只用双天线。"""
        cfg = self.cfg
        if not self.locked or self.start is not None:
            return
        full = align_cfg.dual_sigma_deg * DEG
        if self.source == 'borrowed':
            sd = predicted[1] if predicted is not None else 2.0 * full
            self.start = dict(dual=None, carried=self.offset, sigma=sd, weight=1.0)
        else:
            sd = full * math.sqrt(cfg.lock_samples / max(1, self.offset_n))
            self.start = dict(dual=self.offset, carried=None, sigma=sd, weight=0.0)
            if predicted is not None:
                c, sc = predicted
                gap = wrap(c - self.offset)
                self.start['carried'] = c
                if abs(gap) <= align_cfg.max_gap_deg * DEG:
                    w = sd * sd / (sd * sd + sc * sc)
                    self.offset = wrap(self.offset + w * gap)
                    self.start.update(sigma=math.sqrt(sd * sd * sc * sc / (sd * sd + sc * sc)), weight=w)
        self._prior_scale = self.start['sigma'] / full
        if rate is not None and all(math.isfinite(v) for v in rate):
            self.start['rate'] = rate

    def fit_rate(self):
        """rtk_fit：拟合出的漂移速度 (rad/m, 误差 rad/m)，还没开始拟合为 None。"""
        return None if self._fit is None else (self._fit['x'][2], math.sqrt(max(0.0, self._fit['P'][2][2])))

    def fit_sigma(self):
        """rtk_fit：拟合出的角度差的误差（rad），还没开始拟合为 None。"""
        return None if self._fit is None else math.sqrt(max(0.0, self._fit['P'][1][1]))

    def update(self, measured, rel, distance, dt, nozzle, rtk_ok, fast=False, steering=False):
        """锁住偏移以后每拍一次。measured：卫星（融合位置，天线杠杆臂按 rel + β 改算）量的喷头横偏，m；
        rel：IMU 参考航向 − 线方向，rad；distance：这一拍往前走的路，m；nozzle：喷头（车体前, 左），m；
        rtk_ok：这一拍卫星位置可以用来修正；fast：对准阶段（调用方另判，推算开始 fast_m 内也算）；
        steering：IMU 内环已经在开（rtk_trim、rtk_fit 都关时，这以后卫星不再修正）。返回喷头横偏估计。"""
        cfg = self.cfg
        self.fast = fast or self._travel < cfg.fast_m
        if cfg.rtk_fit:
            return self._fit_update(measured, rel, distance, dt, nozzle, rtk_ok)
        if not cfg.rtk_trim and steering:
            rtk_ok = False
        if self.e is None:
            self.e, self._rel, self._travel = measured, rel, 0.0
            self.innovation = 0.0
            return self.e
        nx, ny = nozzle
        h, hp = rel + self.beta, self._rel + self.beta
        # 后轴横移（按两拍的平均航向）+ 车头转动带着喷头横移；β 是认知修正、不是真转，不进杠杆项
        self.e += distance * math.sin(0.5 * (h + hp)) + nx * (math.sin(h) - math.sin(hp)) + ny * (math.cos(h) - math.cos(hp))
        self._rel = rel
        self._travel += abs(distance)
        r = measured - self.e
        self.innovation = r
        use = rtk_ok
        if rtk_ok and self._travel > cfg.trust_from_m:
            if self.mode == 'rtk' and abs(r) > cfg.jump_m:
                self.mode, self._agree_s = 'imu', 0.0
                self.jumps += 1
            elif self.mode == 'imu':
                self._agree_s = self._agree_s + dt if abs(r) <= cfg.rejoin_m else 0.0
                if self._agree_s >= cfg.rejoin_s:
                    self.mode = 'rtk'
            use = self.mode == 'rtk'
        if self.mode == 'imu':
            self.imu_only_s += dt
        if use:
            scale = cfg.fast_observer_m if self.fast else cfg.observer_m
            k = min(0.5, abs(distance) / scale)
            self.e += 2.0 * k * r
            if cfg.rtk_trim:
                lim = cfg.max_heading_fix_deg * DEG
                self.beta = max(-lim, min(lim, self.beta + k / scale * r))
        return self.e

    def _jump_check(self, r, dt, rtk_ok):
        """卫星横偏和推算差 r：判跳点 / 回来（同 update）。返回这拍卫星能不能用。"""
        cfg = self.cfg
        use = rtk_ok
        if rtk_ok and self._travel > cfg.trust_from_m:
            if self.mode == 'rtk' and abs(r) > cfg.jump_m:
                self.mode, self._agree_s = 'imu', 0.0
                self.jumps += 1
            elif self.mode == 'imu':
                self._agree_s = self._agree_s + dt if abs(r) <= cfg.rejoin_m else 0.0
                if self._agree_s >= cfg.rejoin_s:
                    self.mode = 'rtk'
            use = self.mode == 'rtk'
        if self.mode == 'imu':
            self.imu_only_s += dt
        return use

    def _fit_jump_check(self, z, u, dt, rtk_ok):
        """rtk_fit 的跳点（见 ImuFirstConfig.jump_window_m）。z = 卫星横偏 − IMU 推算横移，u = 路程。返回这拍卫星能不能用。
        跳开以后，回到跳开前那条趋势（参照点 + 当时的角度 × 距离）rejoin_m 内持续 rejoin_s 才再用；一直不回来就这条线按 IMU 走完。"""
        cfg, hist = self.cfg, self._fit_hist
        if not rtk_ok:
            # 恢复证据必须连续；质量中断前的稳定帧不能在恢复后接着计时。
            self._agree_s = self._jump_s = 0.0
            self._jump_after.clear()
            if self.mode == 'imu':
                self.imu_only_s += dt
            return False
        b = self._fit_now(u)[1]
        if self.mode == 'rtk':
            while len(hist) >= 2 and hist[1][0] <= u - cfg.jump_window_m:
                hist.popleft()
            if hist and self._travel > cfg.trust_from_m:
                ru, rz = hist[0]
                du = u - ru
                if abs(z - rz - b * du) > cfg.jump_m + math.tan(cfg.jump_slope_deg * DEG) * du:
                    self.mode, self._agree_s, self._jump_ref = 'imu', 0.0, (ru, rz, b)
                    self._jump_after, self._jump_s, self._jump_step = [], 0.0, z - rz - b * du
                    self.jumps += 1
        else:
            ru, rz, rb = self._jump_ref
            self._agree_s = self._agree_s + dt if abs(z - rz - rb * (u - ru)) <= cfg.rejoin_m else 0.0
            self._jump_s += dt
            self._jump_after.append((u, z))
            if self._agree_s >= cfg.rejoin_s:
                self.mode = 'rtk'
            elif self._jump_s >= cfg.jump_accept_s and abs(self._jump_step) <= cfg.jump_accept_max_m and self._settled():
                step = z - self._fit_now(u)[0]
                # 没回来但新位置稳定、跳开那一下不远：认卫星，让卡尔曼的修正量能挪过去（推算上仍按 fit_rate_m 慢慢挪）。
                # 只按 IMU 走的这段推算和卫星又差出去多少都一起认（傍晚回放：按累积差判就一直认不上）
                self._fit['P'][0][0] += step * step
                self.mode = 'rtk'
                self.jumps_accepted += 1
                hist.clear()
        if self.mode == 'imu':
            self.imu_only_s += dt
            return False
        hist.append((u, z))
        return True

    def pull_to(self, measured, fraction):
        """助跑段（还没喷）：推算横偏按 fraction（0–1）直接往卫星横偏 measured 靠；拟合的修正量一起挪，开喷时接着用。"""
        if self.e is None or self._fit is None or not math.isfinite(measured):
            return
        d = max(0.0, min(1.0, fraction)) * (measured - self.e)
        self._fit['c'] += d
        self._fit['x'][0] += d
        self.e += d

    def source_changed(self):
        """卫星横偏换了来源（主 / 副天线）：跳点参照从这一拍重新攒，两根天线之间的差不算跳。"""
        self._fit_hist.clear()

    def _settled(self):
        """跳开以后最近 jump_accept_s 的卫星点自己稳定：扣掉直线趋势后散布 ≤ rejoin_m。"""
        pts = self._jump_after[-int(max(2, self.cfg.jump_accept_s * 50)):]
        if len(pts) < 10 or pts[-1][0] - pts[0][0] < 0.2:
            return False
        n = len(pts)
        mu = sum(p[0] for p in pts) / n
        mz = sum(p[1] for p in pts) / n
        suu = sum((p[0] - mu) ** 2 for p in pts)
        k = sum((p[0] - mu) * (p[1] - mz) for p in pts) / suu if suu > 0 else 0.0
        return max(abs(p[1] - mz - k * (p[0] - mu)) for p in pts) <= self.cfg.rejoin_m

    def _fit_update(self, measured, rel, distance, dt, nozzle, rtk_ok):
        """rtk_fit（见 ImuFirstConfig）：喷头横偏 = IMU 推算 + 修正 c；卫星可用时每 fit_bin_m 平均成一个点，卡尔曼跟踪
        [c, 角度 b]（c 随路程按 b 增长，两者都按距离随机游走）；用到推算和目标车头上的 c、b 再按距离限速。"""
        cfg = self.cfg
        if self.e is None:
            self.rtk_accepted = rtk_ok
            prior = max(cfg.fit_prior_min_deg, min(cfg.fit_prior_max_deg, cfg.fit_prior_deg * self._prior_scale))
            sc, sb, sr = cfg.fit_prior_m, math.sin(prior * DEG), cfg.fit_prior_rate_deg * DEG
            r0 = 0.0
            carried = (self.start or {}).get('rate')
            if carried is not None:                  # 上一条线学到的漂移速度（线头就是卫星差的地方也能按它推）
                r0, sr = carried[0], min(sr, carried[1])
            self._fit = dict(e0=measured, imu=0.0, x=[0.0, 0.0, r0], P=[[sc * sc, 0.0, 0.0], [0.0, sb * sb, 0.0], [0.0, 0.0, sr * sr]],
                             u=0.0, cell=[0.0, 0.0, 0, 0.0], c=0.0, b=0.0, anchor=not rtk_ok)
            self.e, self._rel, self._travel, self.innovation = measured, rel, 0.0, 0.0
            return self.e
        nx, ny = nozzle
        h, hp = rel, self._rel
        f = self._fit
        f['imu'] += distance * math.sin(0.5 * (h + hp)) + nx * (math.sin(h) - math.sin(hp)) + ny * (math.cos(h) - math.cos(hp))
        self._rel = rel
        self._travel += abs(distance)
        u, base = self._travel, f['e0'] + f['imu']
        x, cell = f['x'], f['cell']
        c_now, b_now = self._fit_now(u)
        self.innovation = measured - (base + c_now)
        self.rtk_accepted = self._fit_jump_check(measured - base, u, dt, rtk_ok)
        if self.rtk_accepted:
            if cell[2] == 0:
                cell[3] = u
            cell[0] += u
            cell[1] += measured - base
            cell[2] += 1
            if u - cell[3] >= cfg.fit_bin_m:
                hu, z = cell[0] / cell[2], cell[1] / cell[2]
                if f['anchor'] and self._fit_start_wrong(hu, z):
                    self._fit_anchor(hu, z)
                else:
                    f['anchor'] = False
                    self._persist_check(hu, z)
                    self._fit_step(hu, z)
                cell[:] = [0.0, 0.0, 0, 0.0]
                self.fit_points += 1
                c_now, b_now = self._fit_now(u)
        else:
            cell[:] = [0.0, 0.0, 0, 0.0]
            self._persist = None     # 没有可信观测的里程不能算成“持续偏差”
        lim = math.sin(cfg.max_heading_fix_deg * DEG)
        x[1] = max(-lim, min(lim, x[1]))
        b_now = max(-lim, min(lim, b_now))
        step = abs(distance)
        # 用到车头上的角度带来的横移照常补（IMU 推算不含 β）；只有额外的纠正按 fit_rate_m 限速
        f['c'] += f['b'] * step
        f['c'] += max(-cfg.fit_rate_m * step, min(cfg.fit_rate_m * step, c_now - f['c']))
        start = u < cfg.fit_start_m and self.fit_points * cfg.fit_bin_m >= cfg.fit_start_points_m - 1e-9
        db = (cfg.fit_start_beta_rate_deg if start else cfg.fit_beta_rate_deg) * DEG * step
        f['b'] += max(-db, min(db, b_now - f['b']))
        self.beta = math.asin(max(-lim, min(lim, f['b'])))
        self.e = base + f['c']
        return self.e

    def _fit_start_wrong(self, hu, z):
        """拟合起点那拍不可信时，第一个可信点（路程 hu、值 z）和预测差多少个标准差，超过 fit_anchor_sigma 为 True。"""
        cfg, f = self.cfg, self._fit
        x, P = f['x'], f['P']
        d = max(0.0, hu - f['u'])
        F0 = (1.0, d, d * d / 2)
        var = sum(F0[i] * P[i][j] * F0[j] for i in range(3) for j in range(3))
        var += cfg.fit_walk_c_m ** 2 * d + cfg.fit_noise_m ** 2
        r = z - sum(F0[i] * x[i] for i in range(3))
        return r * r > cfg.fit_anchor_sigma ** 2 * var

    def _fit_anchor(self, hu, z):
        """rtk_fit：拟合起点那拍卫星不可信，这是第一个可信点：修正量就取这一点、按 fit_prior_m 信（同正常起点）；
        角度、漂移速度的估计和不确定度照旧推到这一点，不拿起点的错去改角度。"""
        cfg, f = self.cfg, self._fit
        x, P = f['x'], f['P']
        d = max(0.0, hu - f['u'])
        wb, wr = (cfg.fit_walk_b_deg * DEG) ** 2 * d, (cfg.fit_walk_r_deg * DEG) ** 2 * d
        pbb = P[1][1] + 2.0 * d * P[1][2] + d * d * P[2][2] + wb
        pbr = P[1][2] + d * P[2][2]
        prr = P[2][2] + wr
        x[:] = [z, x[1] + x[2] * d, x[2]]
        f['P'] = [[cfg.fit_prior_m ** 2, 0.0, 0.0], [0.0, pbb, pbr], [0.0, pbr, prr]]
        f['u'], f['anchor'] = hu, False

    def _persist_check(self, hu, z):
        """rtk_fit：这一点（路程 hu，卫星横偏 − IMU 推算 z）和拟合差多少。最近 persist_m 的点都在同一边差超过
        persist_e_m 的一半、平均超过 persist_e_m，放开位置修正量（差²）的不确定度。
        横向位置台阶不证明航向有误，不同时放大角度不确定度；角度由后续连续观测的斜率学习。
        这一点起拟合跟上（见 ImuFirstConfig.persist_m）。10-07 回放：残差在 1.0–1.5 cm 之间晃，逐点都要超 1 cm 会被
        0.99 断开，所以按平均。"""
        cfg = self.cfg
        r = z - self._fit_now(hu)[0]
        p = self._persist
        if abs(r) <= 0.5 * cfg.persist_e_m or p and (r > 0) != (p[-1][1] > 0):
            p = self._persist = None
        if abs(r) <= 0.5 * cfg.persist_e_m:
            return
        if p is None:
            p = self._persist = deque()
        p.append((hu, r))
        while len(p) > 1 and hu - p[1][0] >= cfg.persist_m - 1e-9:
            p.popleft()                                  # 只留最近 persist_m 的点
        mean = sum(v for _, v in p) / len(p)
        if hu - p[0][0] >= cfg.persist_m - 1e-9 and abs(mean) > cfg.persist_e_m:
            P = self._fit['P']
            P[0][0] += mean * mean
            self.persists += 1
            self._persist = None

    def _fit_now(self, u):
        """卡尔曼状态（上一点）推到路程 u：(修正量 c, 角度 b)，含角度变化速度 r。"""
        x, d = self._fit['x'], u - self._fit['u']
        return x[0] + x[1] * d + x[2] * d * d / 2, x[1] + x[2] * d

    def _fit_step(self, hu, z):
        """卡尔曼：状态 [c, b, r] 从上一点预测到路程 hu（c += b·Δ + r·Δ²/2，b += r·Δ，三者按距离随机游走），
        再用这一点 z（卫星横偏 − IMU 推算）更新。"""
        cfg, f = self.cfg, self._fit
        x, P = f['x'], f['P']
        d = max(0.0, hu - f['u'])
        F = [[1.0, d, d * d / 2], [0.0, 1.0, d], [0.0, 0.0, 1.0]]
        q = [cfg.fit_walk_c_m ** 2 * d, (cfg.fit_walk_b_deg * DEG) ** 2 * d, (cfg.fit_walk_r_deg * DEG) ** 2 * d]
        x[:] = [sum(F[i][k] * x[k] for k in range(3)) for i in range(3)]
        FP = [[sum(F[i][k] * P[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        P = [[sum(FP[i][k] * F[j][k] for k in range(3)) + (q[i] if i == j else 0.0) for j in range(3)] for i in range(3)]
        s = P[0][0] + cfg.fit_noise_m ** 2
        k = [P[i][0] / s for i in range(3)]
        r = z - x[0]
        x[:] = [x[i] + k[i] * r for i in range(3)]
        rl = cfg.fit_max_rate_deg * DEG
        x[2] = max(-rl, min(rl, x[2]))
        f['P'] = [[P[i][j] - k[i] * P[0][j] for j in range(3)] for i in range(3)]
        f['u'] = hu

    def status(self):
        deg = lambda a: None if a is None else round(math.degrees(a), 3)  # noqa: E731
        return dict(first_mode=self.mode, first_rtk_trim=self.cfg.rtk_trim, first_fast=self.fast, first_locked=self.locked,
                    first_source=self.source, first_offset_deg=deg(self.offset),
                    first_offset_n=self.offset_n, first_offset_rejected=self.offset_rejected,
                    first_e=self.e, first_beta_deg=deg(self.beta), first_innovation=self.innovation,
                    first_jumps=self.jumps, first_jumps_accepted=self.jumps_accepted, first_imu_only_s=round(self.imu_only_s, 2),
                    first_rtk_fit=self.cfg.rtk_fit,
                    first_fit_points=self.fit_points, first_persists=self.persists,
                    first_rtk_accepted=self.rtk_accepted,
                    first_fit_sigma_deg=None if self._fit is None else round(math.degrees(math.sqrt(max(0.0, self._fit['P'][1][1]))), 4),
                    first_fit_lag=None if self._fit is None else round(self._fit_now(self._travel)[0] - self._fit['c'], 4),
                    first_fit_rate_deg_m=None if self._fit is None else round(math.degrees(self._fit['x'][2]), 5),
                    first_start_dual_deg=deg((self.start or {}).get('dual')),
                    first_start_carried_deg=deg((self.start or {}).get('carried')),
                    first_start_sigma_deg=deg((self.start or {}).get('sigma')),
                    first_start_weight=None if self.start is None else round(self.start['weight'], 3))


@dataclass(frozen=True)
class AlignmentConfig:
    """IMU → 地面方向换算 c（真实车头 − IMU 偏航）跨线带过去的校准（ImuAlignment）。

    2026-10-06 傍晚实车 4094cc78 的 17 趟（卫星拟合充分）：c 随时间变约 −0.07°/min，随 IMU 累计转角变约 −0.17%
    （每转 +180° 约 −0.30°），两项一起拟合残差 0.38°；只按其中一项残差 0.9–1.0°。相邻两条线干净的 7 对：上一条线拟合好
    的 c 按校准带过掉头，留一法误差中位 0.23°、最大 0.40°；双天线开头学的中位 0.14°、最大 0.56°；两者平均 0.15°、0.24°。
    开头双天线只攒到 15–25 个样本的三趟（误差 0.45–0.86°），带过来的误差 0.14–0.39°。默认值取这批数据，车上按每条线
    拟合好的 c 在线修正（卡尔曼，状态 [每分钟变多少, 每转 1 rad 变多少]）。"""
    drift_deg_min: float = -0.07
    drift_sigma_deg_min: float = 0.05
    scale: float = -0.0017
    scale_sigma: float = 0.001
    walk_deg_sqrt_min: float = 0.10   # 模型以外的随机变化，°/√min
    # 每分钟变化量本身也会慢慢变（傍晚最后几分钟比平均快），°/min/√min。按 24 趟顺序回放：0 → 加权后开头误差中位 0.13° 最大
    # 0.30°，0.02 → 0.12° / 0.30°，0.05–0.2 反应过度、最大 0.46–0.67°；校准量从 0 开始学也有 0.17° / 0.37°（双天线 0.20° / 0.86°）
    drift_walk: float = 0.02
    obs_floor_deg: float = 0.05       # 一条线拟合出的 c 至少按这么大的误差算
    max_age_s: float = 600.0          # 上一条线的 c 超过这么久不带
    imu_gap_s: float = 1.0            # IMU 断了这么久：累计转角接不上，以前的 c 作废
    # 带过来的和双天线测的差超过它：不混（IMU 重启、测向坏），只用双天线。2026-10-10 1.5 → 3.0：双天线开头学的比卫星轨迹
    # 方向偏中位 0.5°、最大 1.3°，带过来的中位 0.2°；10-10 16:16:42 两者差 1.56° 刚过 1.5，丢了准的（差 0.2°）用了偏的
    # （差 1.34°），线头偏 2 cm、β 补了 12 m 又反向过头 2.5 cm。今天和 10-09 的 16 条直线回放：开头换算误差中位
    # 0.25° → 0.16°、最大 1.34° → 0.42°，其余各条不变。IMU 重启归零的差远大于 3°，照样只用双天线。
    # （同时试过把 dual_sigma_deg 调到 0.5：中位 0.19°、最大 0.54°，有两条变差，没用）
    max_gap_deg: float = 3.0
    dual_sigma_deg: float = 0.3       # 双天线学满 lock_samples 时开头换算的误差（傍晚实车中位 0.22°、最大 0.66°）
    rate_walk_deg_m: float = 0.003    # 带过去的漂移速度（°/m）每过 1 min 多算这么大的误差（漂移速度本身会变）

    def __post_init__(self):
        for name in ('drift_sigma_deg_min', 'scale_sigma', 'walk_deg_sqrt_min', 'drift_walk', 'obs_floor_deg', 'max_age_s', 'imu_gap_s',
                     'rate_walk_deg_m',
                     'max_gap_deg', 'dual_sigma_deg'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' 必须是有限正数')
        for name in ('drift_deg_min', 'scale'):
            if not math.isfinite(getattr(self, name)):
                raise ValueError(name + ' 必须是有限数')


class ImuAlignment:
    """任务层持有、跨线跨作业共用的 IMU → 地面方向换算 c（真实车头 − IMU 偏航，rad）：

    - feed_imu：每帧 IMU 偏航，累出展开的转角 θ（掉头转了几圈都记得）；
    - 每条线开头 begin_line（把上一条线定下来的 c 交给校准），这条线卫星拟合好了 update_line 记下 (时刻, θ, c, 误差)；
    - predict：上一条线的 c + 每分钟变化 × 分钟 + 比例 × 转角，连同误差；开头和双天线测的按误差加权（ImuFirstGuide.blend_start）；
    - 相邻两条线都拟合好：差值 = 每分钟变化 × 分钟 + 比例 × 转角，在线修正两个校准量。"""

    def __init__(self, config=AlignmentConfig()):
        self.cfg = config
        d, k = config.drift_deg_min * DEG, config.scale
        self.x = [d, k]
        self.P = [[(config.drift_sigma_deg_min * DEG) ** 2, 0.0], [0.0, config.scale_sigma ** 2]]
        self.theta = self._yaw = self._imu_t = None
        self.obs = None                  # 上一条线定下来的 (t, θ, c, σ)；σ 为空：没经卫星拟合（只借，不校准）
        self.pending = None              # 这条线正在更新的
        self.token = 0
        self.calibrations = 0
        self.rate = None                 # 最近一条线拟合好的漂移速度 (时刻, rad/m, 误差 rad/m)，下一条线开头带过去
        self._rate_pending = None

    # 旧接口：value / t 是最近一条线的 c 和时刻
    @property
    def value(self):
        o = self.pending or self.obs
        return None if o is None else o[2]

    @property
    def t(self):
        o = self.pending or self.obs
        return None if o is None else o[0]

    def feed_imu(self, t, yaw):
        if not (math.isfinite(t) and math.isfinite(yaw)):
            return
        if self._yaw is None or t - self._imu_t > self.cfg.imu_gap_s:
            if self._yaw is not None:
                self.obs = self.pending = None   # IMU 断过：转角接不上
            self.theta = yaw
        else:
            self.theta += wrap(yaw - self._yaw)
        self._yaw, self._imu_t = yaw, t

    def begin_line(self):
        """新的一条线开头（偏移刚锁住）调用：上一条线记下的 c 定下来，能和再上一条配成对就校准。返回这条线的号。"""
        if self._rate_pending is not None:
            self.rate, self._rate_pending = self._rate_pending, None
        p, o = self.pending, self.obs
        if p is not None:
            if (o is not None and p[3] is not None and o[3] is not None and p[1] is not None and o[1] is not None
                    and 0.0 < p[0] - o[0] <= self.cfg.max_age_s):
                self._calibrate(o, p)
            self.obs = p
        self.pending = None
        self.token += 1
        return self.token

    def update_rate(self, token, t, rate, sigma):
        """这条线拟合出的漂移速度（rad/m）和误差，下一条线开头带过去。"""
        if token == self.token and all(math.isfinite(v) for v in (t, rate, sigma)):
            self._rate_pending = (t, rate, sigma)

    def predict_rate(self, now):
        """上一条线的漂移速度 (rad/m, 误差)；太旧（> max_age_s）或 IMU 断过返回 None。误差按隔的时间放大。"""
        r = self.rate
        if r is None or self.obs is None or not math.isfinite(now) or not 0.0 <= now - r[0] <= self.cfg.max_age_s:
            return None
        return r[1], math.hypot(r[2], self.cfg.rate_walk_deg_m * DEG * math.sqrt((now - r[0]) / 60.0))

    def update_line(self, token, t, c, sigma=None):
        """这条线的 c（偏移 + β）；sigma（rad）为空表示没经卫星拟合，只给下一条借、不参与校准和加权。"""
        if token == self.token and math.isfinite(t) and math.isfinite(c):
            if sigma is None and self.pending is not None and self.pending[3] is not None:
                return
            self.pending = (t, self.theta, wrap(c), None if sigma is None else max(sigma, self.cfg.obs_floor_deg * DEG))

    def remember(self, t, value):
        self.update_line(self.token, t, value)

    def _calibrate(self, o, p):
        dt, dth = (p[0] - o[0]) / 60.0, p[1] - o[1]
        z = wrap(p[2] - o[2]) - (self.x[0] * dt + self.x[1] * dth)
        self.P[0][0] += (self.cfg.drift_walk * DEG) ** 2 * dt
        P = self.P
        ph = [P[0][0] * dt + P[0][1] * dth, P[1][0] * dt + P[1][1] * dth]
        r = o[3] ** 2 + p[3] ** 2 + (self.cfg.walk_deg_sqrt_min * DEG) ** 2 * dt
        gain = 1.0 / (dt * ph[0] + dth * ph[1] + r)
        k = [ph[0] * gain, ph[1] * gain]
        self.x = [self.x[0] + k[0] * z, self.x[1] + k[1] * z]
        self.P = [[P[0][0] - k[0] * ph[0], P[0][1] - k[0] * ph[1]], [P[1][0] - k[1] * ph[0], P[1][1] - k[1] * ph[1]]]
        self.calibrations += 1

    def predict(self, now):
        """上一条线（这条线开头时就是 obs）的 c 带到现在：(c, σ) rad；没有、太旧、转角接不上返回 None。"""
        o = self.obs
        if o is None or not math.isfinite(now) or not 0.0 <= now - o[0] <= self.cfg.max_age_s:
            return None
        dt = (now - o[0]) / 60.0
        if o[1] is None or self.theta is None:
            return None
        dth = self.theta - o[1]
        P = self.P
        pdd = P[0][0] + (self.cfg.drift_walk * DEG) ** 2 * dt
        var = ((o[3] if o[3] is not None else self.cfg.dual_sigma_deg * DEG) ** 2 + dt * dt * pdd + dth * dth * P[1][1]
               + 2 * dt * dth * P[0][1] + (self.cfg.walk_deg_sqrt_min * DEG) ** 2 * dt)
        return wrap(o[2] + self.x[0] * dt + self.x[1] * dth), math.sqrt(max(var, 0.0))

    def recall(self, now, difference, max_age_s, max_gap_deg):
        """开头基线不对时借：带到现在的 c；和此刻 融合航向 − IMU 偏航 差超过 max_gap_deg（IMU 重启归零）不借。"""
        o = self.pending or self.obs
        if o is None or not math.isfinite(now) or not math.isfinite(difference) or not 0.0 <= now - o[0] <= max_age_s:
            return None
        pred = self.predict(now) if o is self.obs else None
        value = pred[0] if pred is not None else o[2]
        if abs(wrap(difference - value)) > max_gap_deg * DEG:
            return None
        return value

    def status(self):
        deg = lambda a: None if a is None else round(math.degrees(a), 4)  # noqa: E731
        return dict(align_drift_deg_min=deg(self.x[0]), align_scale_pct=round(self.x[1] * 100, 4),
                    align_drift_sigma=deg(math.sqrt(max(self.P[0][0], 0.0))),
                    align_scale_sigma_pct=round(math.sqrt(max(self.P[1][1], 0.0)) * 100, 4),
                    align_calibrations=self.calibrations, align_theta_deg=deg(self.theta),
                    align_c_deg=deg(None if self.obs is None else self.obs[2]))


def paint_pull(cfg, along_m):
    """画线时往回拉的 (k_e, 车头最多偏几度)：开喷处后 along_m 米。head_m 内按 head_k_e / head_max_bias_deg，
    再 head_blend_m 内线性过渡到 k_e / max_bias_deg（ImuFirstConfig）。"""
    if cfg.head_blend_m > 0:
        w = max(0.0, min(1.0, (along_m - cfg.head_m) / cfg.head_blend_m))
    else:
        w = 1.0 if along_m >= cfg.head_m else 0.0
    return (1.0 - w) * cfg.head_k_e + w * cfg.k_e, (1.0 - w) * cfg.head_max_bias_deg + w * cfg.max_bias_deg


def runup_weight(cfg, along_m):
    """线头前保持助跑控制，线头后按距离连续退出；两端斜率为零。"""
    u=max(0.,min(1.,along_m/cfg.runup_blend_m))
    return 1.-u*u*(3.-2.*u)


def imu_first_target(cfg, guide, target, line_yaw, dt, align_rate_deg_s, align_deg, aligned, pull=None, entry_weight=0.):
    """目标车头（IMU 系）：线方向 − 偏移 − β + 横偏往回拉（推算横偏 deadband_m 内不拉）。还没对准过、或在对准
    阶段（guide.fast）按 align_rate_deg_s 转，上线以后每秒最多改 cfg.rate_deg_s。pull：(k_e, 车头最多偏几度)，
    None 用 cfg.k_e / cfg.max_bias_deg（画线时见 paint_pull）。entry_weight 是助跑控制权重 0..1。
    返回 (目标, 往回拉的偏置, 在对准, 对准过)。"""
    k_e, max_bias_deg = (cfg.k_e, cfg.max_bias_deg) if pull is None else pull
    k_e += entry_weight*(cfg.runup_k_e-k_e)
    max_bias_deg += entry_weight*(cfg.runup_max_bias_deg-max_bias_deg)
    e = guide.e
    excess = max(0.0, abs(e) - cfg.deadband_m)
    lim = max_bias_deg * DEG
    bias = max(-lim, min(lim, -math.atan(k_e * math.copysign(excess, e))))
    desired = wrap(line_yaw - guide.offset - guide.beta + bias)
    rate = cfg.rate_deg_s if aligned and not guide.fast else align_rate_deg_s
    rate += entry_weight*(max(align_rate_deg_s,cfg.runup_rate_deg_s)-rate)
    gap = wrap(desired - target)
    if cfg.rtk_fit and dt > 0:
        # 到目标前先减速；目标本身突变时允许有界过渡，不瞬间翻转角速度。
        accel = (cfg.target_accel_deg_s2+entry_weight*(cfg.runup_target_accel_deg_s2-cfg.target_accel_deg_s2))*DEG
        braking = max(0.0, (math.sqrt((accel * dt) ** 2 + 8.0 * accel * abs(gap)) - accel * dt) / 2.0)
        want = math.copysign(min(rate * DEG, braking), gap) if gap else 0.0
        guide.target_rate += max(-accel * dt, min(accel * dt, want - guide.target_rate))
        target = wrap(target + guide.target_rate * dt)
    else:
        step = rate * DEG * max(0.0, dt)
        target = wrap(target + max(-step, min(step, gap)))
    left = abs(wrap(desired - target))
    aligned = aligned or left <= 1e-6
    return target, bias, not aligned and left > align_deg * DEG, aligned


class ImuLineTracker:
    """接口与 LineTracker 相同（set_line / check_start / start / stop / update / status、st），另加 sync：
    每个控制周期（和起步检查前）由 LineMission 给此刻的单调时钟和最近一帧 IMU (t, 偏航 rad, 陀螺 z rad/s)。"""

    def __init__(self, config=CascadeConfig(), geometry=None):
        self.cfg = config
        self.speed_limit_mps = None
        self.geo = geometry or ChassisGeometry()
        # 内环的 speed 只用来判断「车走起来了没有」（指令车速不到一半不积分），按巡航速度算
        self.hold_cfg = dataclasses.replace(config.hold, speed=config.cruise_mps)
        self.hold = HeadingHold(self.hold_cfg)
        self.a = self.b = None
        self.st = LineState()
        self._now = None
        self._imu = None
        self._lever = None           # 融合用的主天线杠杆臂（车体系前、左，m），mission 每周期给
        self._diff = None            # 融合航向 − 参考航向，rad
        self._settled_since = None   # 这次起步什么时候对准、加速完（那一刻偏移从头重新平均）
        self._run = 0                # 每次起步 / 继续换一个，内环按新的一次作业重新开始
        self._offset = None          # 融合航向 − IMU 偏航（低通），rad
        self._offset_n = 0
        self._target = None          # 发给内环的目标（IMU 系，限速后），rad
        self._bias = 0.0             # δ，rad
        self._aligning = False
        self._wz = 0.0
        self._last_s = None
        self._v_prev = 0.0
        self._v_sent = deque()       # 发出的 (时刻, 车速)，每个值一直作用到下一条；只留预估要用的
        self._sum2 = self._n = 0.0
        self._max = 0.0

    # ---- 输入 ----
    def sync(self, now, imu, lever=None):
        """此刻的时钟、最近一帧 IMU（没有时 None）、融合正在用的主天线杠杆臂（没有时 None：位置不改算）。"""
        self._now = float(now)
        self._lever = lever
        if imu is not None and imu != self._imu:
            self._imu = imu
            self.hold.on_imu(imu[1], imu[2], now=imu[0])

    def _imu_now(self):
        """(IMU 偏航按陀螺推到此刻, 陀螺)；IMU 太旧或没有时 None。"""
        imu, now = self._imu, self._now
        if imu is None or now is None:
            return None
        age = now - imu[0]
        if not 0.0 <= age <= self.hold_cfg.imu_stale_s:
            return None
        return wrap(imu[1] + imu[2] * age), imu[2]

    def _reference(self, pose):
        """参考航向（ENU）= IMU 偏航 + 偏移，顺手记下融合航向与它的差；还没有偏移或 IMU 太旧时 None。"""
        imu = self._imu_now()
        if imu is None or self._offset is None:
            self._diff = None
            return None
        ref = wrap(imu[0] + self._offset)
        self._diff = wrap(pose.yaw - ref)
        return ref

    def _position(self, pose, heading):
        """后轴中心按 heading 改算：融合位置 + R(融合航向)·杠杆臂 = 天线位置，再减 R(heading)·杠杆臂。"""
        if self._lever is None or heading == pose.yaw:
            return pose.x, pose.y
        ax, ay = self._lever
        cf, sf, cr, sr = math.cos(pose.yaw), math.sin(pose.yaw), math.cos(heading), math.sin(heading)
        return (pose.x + (cf - cr) * ax - (sf - sr) * ay, pose.y + (sf - sr) * ax + (cf - cr) * ay)

    # ---- 任务 ----
    def set_line(self, a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length < self.cfg.min_line_m:
            raise ValueError('A、B 两点太近（%.2f m）' % length)
        self.a, self.b = tuple(a), tuple(b)
        self.u = (dx / length, dy / length)
        self.n = (-self.u[1], self.u[0])
        self.line_yaw = math.atan2(dy, dx)
        self.st = LineState(length=length)

    def _along_across(self, x, y, yaw, forward, left=0.0):
        c, si = math.cos(yaw), math.sin(yaw)
        px, py = x + forward * c - left * si, y + forward * si + left * c
        rx, ry = px - self.a[0], py - self.a[1]
        return rx * self.u[0] + ry * self.u[1], rx * self.n[0] + ry * self.n[1]

    def geometry_of(self, x, y, yaw):
        """(喷头横偏, 车头误差, 记点那个点的沿线距离)。"""
        _, e = self._along_across(x, y, yaw, self.cfg.nozzle_x_m, self.cfg.nozzle_y_m)
        s, _ = self._along_across(x, y, yaw, self.cfg.track_x_m, self.cfg.track_y_m)
        return e, wrap(yaw - self.line_yaw), s

    def check_start(self, pose):
        """能否从当前位姿开始；返回 (可以, 原因)。条件同 LineTracker，另要有新鲜的 IMU 航向。"""
        if self.a is None:
            return False, 'no_line'
        if not pose.usable_for_control:
            return False, 'pose_unusable'
        e, he, s = self.geometry_of(pose.x, pose.y, pose.yaw)
        if abs(e) > self.cfg.start_max_offset_m:
            return False, 'too_far_from_line'
        if abs(he) > self.cfg.start_max_heading_rad:
            return False, 'heading_not_aligned'
        if s < -self.cfg.start_before_a_m:
            return False, 'too_far_behind_a'
        if s > self.st.length - 0.5:
            return False, 'already_past_b'
        if self._imu_now() is None:
            return False, 'no_imu'
        return True, ''

    def start(self, pose, integral=None):
        """开始或在暂停后继续；integral 不给时沿用当前值。内环重新开始，目标从此刻的 IMU 偏航出发。"""
        ok, why = self.check_start(pose)
        if not ok:
            if self.st.state != 'PAUSED':
                self.st.state = 'IDLE'
            self.st.reason = why
            return False
        self.st.state, self.st.reason = 'RUNNING', ''
        if integral is not None:
            self.st.integral = integral
        imu_yaw, _ = self._imu_now()
        # 参考航向（IMU + 偏移）离融合航向超过 jump_deg：IMU 只是辅助，按融合航向（卫星）重定偏移
        if self._offset is None or abs(wrap(pose.yaw - imu_yaw - self._offset)) > self.cfg.jump_deg * DEG:
            self._offset, self._offset_n = wrap(pose.yaw - imu_yaw), 1
        self._target = imu_yaw
        self._settled_since = None
        self._run += 1
        self._last_s = None
        self._v_prev = max(0.0, pose.speed)
        self._v_sent.clear()
        self._sum2 = self._n = self._max = 0.0
        return True

    def stop(self, reason='operator'):
        if self.st.state in ('RUNNING', 'PAUSED'):
            self.st.state, self.st.reason = 'ABORTED', reason
        self.hold.end('user')
        return self._halt()

    def _halt(self, state=None, reason=None):
        if state is not None:
            self.st.state, self.st.reason = state, reason
        self.st.v_cmd, self._v_prev, self._wz = 0.0, 0.0, 0.0
        if self._now is not None:
            self._v_sent.append((self._now, 0.0))
        return 0.0, 0.0

    def _pending_m(self, now):
        """已经发出、底盘还没执行（speed_delay_s 内）的车速指令还要走的路，m。"""
        t0, sent = now - self.cfg.speed_delay_s, self._v_sent
        while len(sent) > 1 and sent[1][0] <= t0:
            sent.popleft()
        total = 0.0
        for i, (ts, v) in enumerate(sent):
            te = sent[i + 1][0] if i + 1 < len(sent) else now
            if te > t0:
                total += v * (te - max(ts, t0))
        return total

    # ---- 每个控制周期 ----
    def update(self, pose, dt):
        """返回 (vx, wz)。pose 需有 x, y, yaw, speed, usable_for_control；调用前先 sync。"""
        st, cfg = self.st, self.cfg
        if st.state not in ('RUNNING', 'PAUSED'):
            st.v_cmd = 0.0
            return 0.0, 0.0
        st.integral_frozen = not lateral_integral_allowed(pose)
        if not pose.usable_for_control:
            # 定位不可用：立即停，恢复后由操作员（或 LineMission 的自动继续）重新开始
            self.hold.end('pose')
            return self._halt('PAUSED', 'pose_unusable')
        ref = self._reference(pose)
        if st.state == 'PAUSED':
            st.reason = 'pose_recovered_waiting_operator'
            return self._halt()
        now = self._now

        # 偏移 = 融合航向 − IMU 偏航（同一时刻）：一直照学，先按样本平均，攒够一个时间常数后按低通
        if ref is not None:
            self._offset, self._offset_n, correction = learn_heading_offset(
                self._offset, self._offset_n, self._diff, dt, cfg.offset_tau_s)
            ref = wrap(ref + correction)
        heading = pose.yaw if ref is None else ref
        x, y = self._position(pose, heading)
        e, he, s = self.geometry_of(x, y, heading)
        st.e, st.heading_err, st.s = e, he, s
        if abs(e) > cfg.abort_offset_m or abs(he) > cfg.abort_heading_rad:
            self.hold.end('off_line')
            return self._halt('ABORTED', 'off_line')
        remaining = st.length + cfg.end_aim_m - s - self._pending_m(now)
        if remaining <= 0.0:
            self.hold.end('user')
            return self._halt('DONE', '')

        # 积分只在前进且贴近直线时累积，并限幅
        ds = 0.0 if self._last_s is None else max(0.0, s - self._last_s)
        self._last_s = s
        st.integral, self._bias, self._target, self._aligning = lateral_heading_step(
            cfg, e, ds, st.integral, self._target, self.line_yaw, self._offset, dt, st.integral_frozen)

        # 车速：巡航、偏差降速、终点前按减速度刹停、对准时慢走、起步斜坡
        v = cfg.cruise_mps
        if abs(e) > cfg.slow_offset_m:
            v *= max(0.4, 1.0 - (abs(e) - cfg.slow_offset_m) / 0.3)
        v = min(v, max(cfg.min_mps, math.sqrt(2.0 * cfg.decel_mps2 * max(0.0, remaining))))
        if self._aligning:
            v = min(v, max(cfg.min_mps, cfg.align_radius_m * cfg.turn_rate_deg_s * DEG))
        ramping = v > self._v_prev + cfg.accel_mps2 * dt
        v = min(v, self._v_prev + cfg.accel_mps2 * dt)
        if self.speed_limit_mps is not None:
            v = min(v, max(0.0, self.speed_limit_mps))
        if self._settled_since is None and not self._aligning and not ramping:
            # 起步转完、加速完：偏移从这里重新平均（起步转弯里 IMU 刻度误差攒下的差不带进来）
            self._settled_since = now
            self._offset_n = 0

        self.hold.on_odom(pose.speed, now=now)
        _, wz = self.hold.step(self._run, now, target=self._target, vx=v)
        if self.hold.state != 'running':
            reason = 'imu_deviation' if self.hold.reason == 'deviation' else (self.hold.reason or 'imu_stale')
            return self._halt('ABORTED', reason)
        self._v_prev = st.v_cmd = v
        self._wz = wz
        self._v_sent.append((now, v))
        st.kappa = wz / v if v >= 0.05 else 0.0

        if s >= 0.0:
            self._n += 1
            self._sum2 += e * e
            self._max = max(self._max, abs(e))
            st.stats = dict(rms_m=math.sqrt(self._sum2 / self._n), max_m=self._max, samples=int(self._n))
        return v, wz

    def status(self):
        st = self.st
        deg = lambda a: None if a is None else round(math.degrees(a), 3)  # noqa: E731
        imu = self._imu_now()
        hold_err = None if imu is None or self._target is None else wrap(imu[0] - self._target)
        # 键与 LineTracker 相同（app 不用改）；hold 是内外环的中间量，作业记录里用来查问题
        return dict(state=st.state, reason=st.reason, e=st.e, heading_err_deg=math.degrees(st.heading_err),
                    s=st.s, length=st.length, v_cmd=st.v_cmd, kappa=st.kappa, integral=st.integral,
                    trim_kappa=None, stats=dict(st.stats), integral_frozen=st.integral_frozen,
                    hold=dict(target_deg=deg(self._target), offset_deg=deg(self._offset), bias_deg=deg(self._bias),
                              integ_bias_deg=deg(-self.cfg.k_i * st.integral), error_deg=deg(hold_err),
                              aligning=self._aligning, wz=round(self._wz, 4), state=self.hold.state,
                              reason=self.hold.reason, heading_diff_deg=deg(self._diff)))
