#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""差速车（Gen5）直线控制用的配置和算法块，路线 / 场地作业的直线、圆弧（path_tracker.PathTracker）在用。纯计算，不依赖 ROS。

    CascadeConfig       直线内外环参数（IMU 航向保持内环 + 喷头横偏外环），differential_config 从这里取
    ImuFirstConfig      直线 IMU 为主、卫星拟合慢慢修（rtk_fit）的全部参数；用户定的门槛在 user_rules
    ImuFirstGuide       上面那套的实现：开头换算、拟合、跳点、天线挑选、侧滑、草地
    ImuAlignment        线与线之间带着 IMU → 地面方向的换算
    learn_heading_offset / lateral_heading_step / imu_first_target 等   路线直线每拍调用的计算

2026-11-22 删掉了 A→B 定点画线的控制器 ImuLineTracker（App 早就不发记点、开始；路线、场地作业不用它）。
为什么用 IMU 航向内环而不是按曲率跟线：Gen5 底盘转向晚约 0.44 s（2026-09-30 实测），按曲率跟 0.8 m/s 来回大摆；
IMU 直行（heading_hold.HeadingHold，带 Smith 预估）实车 0.8 m/s 车头稳在目标 ±0.8° 内。

参数来源：内环现参数（kp 2.2 / ki 1.0 / kd 0.4，延迟 0.44 s）与 ±3° 是在 09-30 实测底盘模型上仿真挑的
（tests/test_line_cascade.py）；内环换成更温和的参数反而变差。外环 2026-10-02 由 k_e 0.6 / k_i 0.05 改为 0.8 / 0.3
（用户要水泵横偏 1.5 cm 以内）：旧外环纠不回每趟 −0.5～+1.2° 的测向偏差，按 10-02 实车数据校准的四种底盘仿真
95% 约 1.25 cm、整条线平均偏 0.8 cm；新外环 95% 约 0.45–0.55 cm、最大约 1 cm，测向偏 2° 时最大 1.4 cm。
k_e 1.3 / k_i 0.4（路线长直线外环那组）精度只好 0.05 cm，但底盘延迟到 0.5 s 就会因内环偏离 5° 中止，0.8 / 0.3
到 0.5 s 仍不中止（实测延迟 0.28–0.44 s）。停车瞄准 B 本身（end_aim_m），车速延迟按 10-01 实测 0.33 s 预估。
"""
import math
from collections import deque
from dataclasses import dataclass, field

from heading_hold import HoldConfig
from line_tracker import wrap
import user_rules as rules

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
    # 以下几项与 LineConfig 同名同义（原来 A→B 定点画线从 LineConfig 抄，2026-11-22 删了）
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
    # 喷头横偏（推算）在这以内不往回拉：助跑、线头段、不在喷时用 deadband_m（照旧 2 mm），画线时线头段之后按 paint_deadband_m
    # （和 k_e 一样在 head_blend_m 内过渡，见 paint_deadband）。11-21 用户「去死区」：2 mm 让平地整条线停在 +0.2–0.3 cm
    # 不回 0。线头段不去：起步模型（test_launch_settle）里线头也去掉死区时线头中位 0.37 → 0.50 cm（线头不能变差，用户规矩）
    deadband_m: float = rules.DEADBAND_M
    paint_deadband_m: float = rules.PAINT_DEADBAND_M
    # 画线时（线头之后）往回拉：横偏 → 车头偏置 k_e（1/m），车头最多偏 max_bias_deg。10-08 用户「慢慢修」：原来 0.8 / 1°，
    # 10-06 23:47 那条 37.8 m 线 22–24 m 推算偏右 1 cm 多，车头目标 2 m 内转 +0.55°，3 m 横挪约 2.5 cm，地上看得见一个弯；
    # 卫星那几米读大了约 1 cm，又冲到左边。改成 0.4 / 0.3°，1.5–2 cm 的偏差 4–6 m 内平缓收回。
    # 11-21 用户「去死区 + k_e 0.8」：草地中途侧滑拉不回（偏差 0.5–1 cm、最大 2–2.5 cm），改 0.8 / 0.6°。
    # 仿真（0.36 s 延迟车模型 + 卫星噪声 + 草地侧滑）：草地平均 0.89→0.70 cm、p95 1.9→1.5，平地 0.24→0.19；
    # 代价是卫星读偏 1 cm 持续 3 m 时线跟着多偏（最大 0.76→0.91 cm）
    k_e: float = rules.K_E
    max_bias_deg: float = rules.MAX_BIAS_DEG
    rate_deg_s: float = 1.0          # 上线以后目标车头每秒最多改这么多（原 A/B 3°/s）
    target_accel_deg_s2: float = 4.0  # RTK 拟合模式的目标角加速度；及时响应，同时避免拟合点更新时突变
    # 线头段：这条直线开喷处（线头 / 弯道出口 / 暂停后继续处）后 head_m 内仍按原来的 head_k_e / head_max_bias_deg
    # （线头精度不能变差，用户规矩），再 head_blend_m 内线性过渡到上面的 k_e / max_bias_deg；不在喷（转场段、停喷后的收尾）一直按原来的
    head_k_e: float = rules.HEAD_K_E
    head_max_bias_deg: float = rules.HEAD_MAX_BIAS_DEG
    head_m: float = rules.HEAD_M
    head_blend_m: float = rules.HEAD_BLEND_M
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
    # 2026-11-22 用户：「学开头换算还是按 0.40–0.44」（原 0.42：这台车基线中位 0.418、σ 0.0035，23–51 % 的帧 > 0.42，
    # 开头换算常学得晚）。rtk_fit 开着时卫星纠偏另按 fit_min/max_baseline_m，这一对只管学开头换算（直线、圆弧）
    min_baseline_m: float = rules.START_LEARN_BASELINE_M[0]
    max_baseline_m: float = rules.START_LEARN_BASELINE_M[1]
    baseline_hold_s: float = rules.START_LEARN_HOLD_S
    # 连续这么多帧测向出界才冻结（10-06 傍晚实车：基线中位 0.412 m，零星一两帧 0.395–0.399 / 0.420–0.432，每帧冻 1 s
    # 冻掉直线上中位 37%、开头 57–72%；零星出界那段卫星横偏去趋势 std 0.46 cm，和基线正常 0.41 cm 差不多，连续出界尾巴
    # 1.44 cm（95%）明显差）。3：冻结中位 37% → 14%，下午那种连续掉到 0.32–0.38 m 照样第三帧（约 0.4 s）就冻
    baseline_bad_frames: int = rules.BASELINE_BAD_FRAMES
    # 一条线开头基线就不对、偏移学不到：这段已攒够 offset_min_samples 个就用它，否则借上一条线对好的角度（偏移 + β）
    # 先开，基线正常了再按卫星轨迹纠（用户 10-06 定的；原地掉头 180° 后 IMU 与地面方向的差会变 0.3–0.5°）。
    # 借的不能超过 borrow_max_age_s，和此刻融合航向 − IMU 偏航差不能超过 borrow_max_gap_deg（挡 IMU 重启归零）
    borrow_max_age_s: float = rules.BORROW_MAX_AGE_S
    borrow_max_gap_deg: float = rules.BORROW_MAX_GAP_DEG
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
    # 卫星用不用：固定解 + 双天线基线在 [fit_min_baseline_m, fit_max_baseline_m]（用户定的可信规则 0.35–0.45 m，11-21 由 0.30–0.50 收紧，连续
    # baseline_bad_frames 帧出界才停），加上和 IMU 对不上的突然跳点不用；0.40–0.44 只管学开头换算
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
    fit_rate_m: float = rules.FIT_RATE_M
    fit_beta_rate_deg: float = 0.15   # 用到目标车头上的角度每走 1 m 最多变这么多，°
    # 前 fit_start_m（推算开始算，含助跑）角度每米最多变 fit_start_beta_rate_deg：10-07 上午起步换算差 0.9°，拟合早就看出来了，
    # 按 0.15°/m 用了 6 m 才用上，线头后 2 m 偏到 2.8 cm。拟合点攒够 fit_start_points_m 以后才放快（一两个点定不了角度）
    fit_start_m: float = 6.0
    fit_start_beta_rate_deg: float = 0.5
    fit_start_points_m: float = 1.0
    # 没有记忆的线（ImuFirstGuide.no_memory：路线作业重启后第一条）助跑里（还没开喷，不管直线度）拟合点攒够 fit_start_points_m 以后，
    # 角度每米最多变 fit_runup_beta_rate_deg：2026-11-22 仿真，这时开头按双天线定、测向误差 ±1.5°，拟合在助跑里已经看出角度，
    # 按 0.5°/m 到线头才用上 0.1–0.6°；放到 2°/m 线头 0.3 m 内最差 1.16 → 0.81 cm（测向偏 +1.5°）。
    # 有记忆的线不放（仿真：放了后面几条线头中位 0.29 → 0.37 cm，开头本来就准，助跑那点卫星噪声反倒用进去）
    fit_runup_beta_rate_deg: float = 2.0
    # 拟合起点（第一拍的卫星横偏）那一拍卫星不可信、第一个可信点和「起点 + 角度先验」的预测差超过 fit_anchor_sigma 倍标准差：
    # 起点错了，这一点重新当起点（见 ImuFirstGuide._fit_anchor）。10-07 仿真（遮挡较多 100 m）：暂停后重开那拍在多路径区、
    # 起点错 10 cm，第一个可信点把它算成角度错 2.3°，车斜着开出去。不一概重定：倒车助跑里第一拍常被判不可信但其实是准的，
    # 它和后面几米一起推起步角度（坏区前段·第二条线仿真：一概重定 3 m 后中位 3.5 → 5.5 cm）
    fit_anchor_sigma: float = 3.0
    # 10-07 用户：「app 地图显示已经偏离轨迹，但是没有及时纠正」。游走小（一个坏点拉不动车），持续的真偏差也被当噪声慢慢消化
    # （10-07 上午 14–17 m 卫星和拟合差 1.1–1.6 cm 持续 3 m 不动）：卫星点和拟合连续 persist_m 都在同一边差超过 persist_e_m，
    # 增加位置修正量的不确定度让拟合跟上；不把一个位置台阶直接当成角度误差。
    # 角度仍由连续沿程观测学习，推算上按 fit_rate_m / 角度限速拉回。
    persist_m: float = rules.PERSIST_M
    persist_e_m: float = rules.PERSIST_E_M
    # 2a018383：角度拟合很稳，位置却落后可信卫星 1.6 cm；只放开一次位置协方差仍会再次落后。
    # 喷线后，可信观测连续同侧超过 1 cm 达 1 m，单独启用横偏跟随：IMU + 已学角度预测，卫星按
    # 距离平滑校正位置。角度/侧滑继续由原拟合学习，不把位置差当成转角；修正仍限 fit_rate_m。
    # 回到线附近后放慢位置跟随，避免追逐毫米级卫星往复噪声；卫星不可信时只预测，不撤销已确认的位置。
    position_follow_min_e_m: float = rules.PERSIST_E_M
    position_follow_confirm_m: float = rules.PERSIST_M
    position_follow_tau_m: float = 0.35
    position_follow_near_tau_m: float = 1.5
    fit_min_baseline_m: float = rules.HEADING_BASELINE_M[0]
    fit_max_baseline_m: float = rules.HEADING_BASELINE_M[1]
    # 跳点只挡短暂的：跳开后 jump_accept_s 内没回来、但新位置自己稳定（扣掉直线趋势后散布 ≤ rejoin_m），就认卫星，照常慢慢纠
    # （傍晚回放：原来跳一次就一直只靠 IMU，推算和卫星越拉越远）
    # 固定解一下跳出很远（> jump_accept_max_m）多半是整周解错了，不认，按 IMU 走
    jump_accept_s: float = 3.0
    jump_accept_max_m: float = 0.10
    # 卫星位置可靠才拿来纠（用户 10-06 晚定「四道关」：走直靠 IMU，卫星只在可靠时慢慢纠）。傍晚 100 m 线中间那块和同一块的
    # 纯 IMU 直行：卫星横偏几米内来回 15–22 cm，车头（IMU、双天线）稳在零点几度——卫星在跳。那块 vs 正常路段：接收机
    # σ = hypot(σn, σe) > 2.5 cm 98% vs 0%；两天线位置距离误差 > 3 cm 48% vs 0%；卫星 < 18 颗 46% vs 0%。连续
    # fit_quality_frames 帧不合格就停用，之后连续合格 fit_settle_s 才再用
    fit_max_sigma_m: float = rules.FIT_MAX_SIGMA_M
    # 两天线距离误差 3 → 5 cm、超限后恢复 2 → 0.5 s（10-07 用户定）：10-07 上午 25 m 线 17.7–22.7 m 误差零星 3.3–3.9 cm，
    # σ 1.8–1.9 cm、22–23 颗、基线 0.41 都正常，3 cm 一超就停、再等 2 s，5 m 没用卫星，偏了 3 cm 没纠。
    # 10-06 那块坏区靠 σ > 2.5 cm（98%）照样挡住
    fit_max_pair_m: float = rules.FIT_MAX_PAIR_M
    fit_pair_settle_s: float = rules.FIT_PAIR_SETTLE_S
    # 精度 ≤ 这个值的固定解一定采用（user_rules.RTK_ACCEPT_SIGMA_M，PathTracker._rtk_precise）。11-22 185721：σ ≤ 1.24 cm、
    # 35 颗以上、两天线差 ≤ 1.9 cm，只因基线读 0.424（> 0.42）起步放行一直不过，整条喷线卫星一拍没用，草地上偏到 5.9 cm
    fit_accept_sigma_m: float = rules.RTK_ACCEPT_SIGMA_M
    # 卫星数（主天线）：用户 10-06 晚定 20；10-08 用户改：低于 17 才不信。17–19 颗只在 σ ≤ fit_low_sats_max_sigma_m、
    # 两天线距离误差 < fit_low_sats_max_pair_m、固定解时照常信（10-06 16:32 那趟靠楼一侧主天线掉到 17–19 颗，σ ≤ 2.0 cm、
    # 两天线误差 ≤ 2.9 cm、一直固定解，卫星本来是好的，被 20 颗挡掉 18–26 m 没纠，按 IMU 偏了 2.5 cm）
    fit_min_sats: int = rules.SATS_MIN
    fit_full_sats: int = rules.SATS_FULL
    fit_low_sats_max_sigma_m: float = rules.LOW_SATS_MAX_SIGMA_M  # 10-08 下午用户 2.0 → 2.7 cm：10-06 22:08 那条线 8–14 m 17–19 颗、σ 2.2–2.4 cm，
                                                # 卫星报的往右 2.7 cm 和地上实际一致，被 2.0 挡掉只按 IMU 走偏了；
                                                # 11-22 用户收到 2.5（星少不能比 ≥20 颗松），那条线 2.2–2.4 cm 照样能用；
                                                # 11-26 用户放宽到 3.0，和 fit_max_sigma_m 一起
    fit_low_sats_max_pair_m: float = rules.LOW_SATS_MAX_PAIR_M
    # 两根天线对不上（位置距离误差 ≥ antenna_pick_pair_m）时，用和 IMU 推算对得上的那根（10-08 用户）。10-06 22:08 那条线
    # 15–20 m 主天线被干扰往左 5–9 cm、两天线差到 9–10 cm，副天线和 IMU 都说车没偏（地上实际也没偏）。
    # 副天线横偏 − 主天线横偏 在两根对得上时慢慢学成固定差（antenna_offset_samples 帧以上才算学好），用副天线时扣掉；
    # 一根和推算差 ≤ antenna_agree_m、另一根差超过 antenna_agree_m + antenna_margin_m 才选，不然照旧（≥ 20 颗且误差
    # ≤ fit_max_pair_m 用主天线，否则不用卫星）
    antenna_pick_pair_m: float = rules.ANTENNA_PICK_PAIR_M
    antenna_agree_m: float = rules.ANTENNA_AGREE_M
    antenna_margin_m: float = rules.ANTENNA_MARGIN_M
    antenna_offset_samples: int = 10
    antenna_fresh_s: float = 0.5
    # 助跑段（线头之前，还没喷，10-08 用户）：线头偏 0.5–2.1 cm（助跑起点就偏 1–3 cm，锁车头 1 m 后只剩 ~1.7 m 往回拉，
    # 按 k_e 0.8 / 1° 每米只拉回 ~55%；推算还在「慢慢修」，16:29 线头车以为 0.8 cm、卫星 2.1 cm）。助跑段推算直接往卫星靠
    # （每走 runup_rtk_tau_m 靠拢 63%，卫星可用且没判跳才靠），往回拉按 runup_k_e / runup_max_bias_deg，目标车头最快
    # runup_rate_deg_s、角加速度 runup_target_accel_deg_s2；横偏收小车头偏置跟着收，线头处短距离交接后恢复喷线策略。
    runup_rtk_tau_m: float = rules.RUNUP_RTK_TAU_M
    runup_k_e: float = rules.RUNUP_K_E
    runup_max_bias_deg: float = rules.RUNUP_MAX_BIAS_DEG
    runup_rate_deg_s: float = rules.RUNUP_RATE_DEG_S
    runup_target_accel_deg_s2: float = 8.0
    # 到线头不能同时撤掉位置对齐和纠偏力度；在最初半米连续交给喷线控制。
    # 不改变路径、助跑长度或开关喷边界。
    runup_blend_m: float = 0.5
    fit_quality_frames: int = rules.FIT_QUALITY_FRAMES
    fit_settle_s: float = rules.FIT_SETTLE_S
    # 侧坡侧滑（2026-10-11 用户「加 2、3」）：车在侧坡上往坡下溜，轨迹方向 − 车头 ≈ crab × 横滚（10-11 下午 2° 侧坡 12 条
    # 线全部按方向翻号，西→东 +0.10..+0.57°、东→西 −0.14..−0.60°，拟合 −0.16）；IMU 管车头、看不到。原来全靠拟合的角度 b
    # 慢慢学（还把上一条线反方向的侧滑带到下一条线开头）。现在拟合多一个量 k（侧滑 / 横滚），卫星轨迹和 IMU 推算的差由 b 和
    # k × ∫横滚 一起解释（卫星实时量侧滑），目标车头再减 k × 此刻横滚（坡一变就补）；k 带到下一条线。
    # 横滚用扣掉水平零点的（平地 ≈ 0），低通 crab_roll_tau_s（颠簸不进转向），超过 crab_max_roll_deg 或断了不补
    crab_enabled: bool = True
    crab_prior: float = rules.CRAB_PRIOR
    crab_prior_sigma: float = 0.10
    crab_walk: float = 0.002           # k 每米随机游走，/√m
    crab_max: float = 0.4
    crab_roll_tau_s: float = 1.0
    crab_max_roll_deg: float = 8.0
    crab_roll_fresh_s: float = 0.3
    # 画线时往回拉加强（2026-10-11 用户）：卫星可信时喷头横偏同一边持续超过 boost_e_m 走满 boost_persist_m，车头最多偏
    # max_bias_deg 放到 boost_max_bias_deg；回到 boost_release_e_m 以内或卫星不可信恢复
    boost_e_m: float = rules.PERSIST_E_M
    boost_release_e_m: float = 0.005
    boost_persist_m: float = rules.PERSIST_M
    boost_max_bias_deg: float = rules.BOOST_MAX_BIAS_DEG
    # 草地模式（2026-11-21 用户「做草地模式」）：草地上车会侧滑，「IMU 车头 → 实际走向」的角度几米就变 0.2° 左右；
    # 拟合按 fit_walk_b_deg（0.005°/√m，平地这个角几乎不变）学得太慢。直线上按走过的路取最近 grass_window_m 的两轮
    # 电机电流（|A| 平均）中位数，≥ grass_on_a 算草地、≤ grass_off_a 才回平地（11-21 实录喷线 2 m 中位：平地最高 4.3 A、
    # 一般 2.2–2.4 A；草地一般 8–12 A）；草地时线头段之后（同 paint_pull 过渡）角度游走用 grass_fit_walk_b_deg。
    # 仿真（车头扰动按 11-21 草地实录校准）：在 k_e 0.8 / 0.6° 之上草地平均 0.75 → 0.61 cm（0.5 m/s）、0.86 → 0.63 cm
    # （0.8 m/s），草地卫星读偏 1 cm 也更好；平地开它卫星读偏时最大 0.87 → 1.2–1.4 cm，所以只在草地开。
    # 只记喷线中、车速 ≥ grass_min_speed_mps 的电流（停车、原地转、倒车助跑和慢速起步不进窗口：11-21 11:17 平地那趟
    # 倒车助跑、0.35 m/s 起步时两轮电流一个 10 A 一个 1 A 来回跳，中位数到 6 A，按「直线在开就记」会把平地认成草地）；
    # 电流读数断了 grass_current_fresh_s 清空回平地。回放 11-21 六趟：草地线线头后 78–100% 认成草地，平地 0%
    grass_enabled: bool = True
    grass_window_m: float = 2.0
    grass_on_a: float = rules.GRASS_ON_A
    grass_off_a: float = rules.GRASS_OFF_A
    grass_min_speed_mps: float = rules.GRASS_MIN_SPEED_MPS
    grass_current_fresh_s: float = 0.5
    grass_fit_walk_b_deg: float = 0.1
    # 楼下模式（2026-11-24 用户「写补丁先仿真」，user_rules.POOR_SKY_*）：楼下卫星被楼面反射，用上的星从 40 掉到 24、σ 1.25 →
    # 1.6–1.9 cm，卫星位置连着几米偏 1–1.5 cm；车跟着拉，平地直线鼓 1.6–4.2 cm（IMU 本身是直的）。σ（poor_sky_sigma_tau_s
    # 低通）> poor_sky_sigma_m、不是草地：这条直线进楼下模式（一条线里进了就不退，除非下面的出口）——线头段之后不往回拉
    # （paint_pull 的线头后部分 × poor_sky_pull_scale，加强往回拉不开），开头 fit_start_m 以后拟合角度用到车头上每米最多改
    # poor_sky_beta_rate_deg（开头照常 fit_start_beta_rate_deg：带过来的角度有错在这几米里按卫星对掉），真偏 / 位置跟随门槛
    # 放到 poor_sky_follow_e_m。线头段照旧。
    # 出口（这条线退回平常、不再进）：拟合角度和正在用的差 > max(poor_sky_exit_deg, poor_sky_exit_sigma × 拟合误差)；
    # 可信卫星说喷头同一边偏 > poor_sky_exit_e_m 走满 poor_sky_exit_m（车真偏了）；认出草地。
    # 回放 11-24 楼下 6 条 × 4（车端真实代码，带过来角度误差 ±0.08°，user_rules.POOR_SKY_*）：鼓包 3.28 → 1.72 cm，
    # 最大 2.43 → 2.50 cm，线头不变。开头也放慢（0.1°/m）鼓包 1.9、最差 6.6；出口 4 cm 楼下卫星自己偏就误退，鼓包 2.1–2.4。
    # 6 m 后的角度限速 0.02 → 0.08°/m（11-24 晚）：0.02 时记忆错、侧滑时线斜着走，回放见 user_rules.POOR_SKY_BETA_RATE_DEG
    poor_sky_enabled: bool = True
    poor_sky_sigma_m: float = rules.POOR_SKY_SIGMA_M
    poor_sky_sigma_tau_s: float = 2.0
    poor_sky_pull_scale: float = rules.POOR_SKY_PULL_SCALE
    poor_sky_beta_rate_deg: float = rules.POOR_SKY_BETA_RATE_DEG
    poor_sky_follow_e_m: float = rules.POOR_SKY_EXIT_E_M
    poor_sky_exit_deg: float = rules.POOR_SKY_EXIT_DEG
    poor_sky_exit_sigma: float = 3.0
    poor_sky_exit_e_m: float = rules.POOR_SKY_EXIT_E_M
    poor_sky_exit_m: float = rules.PERSIST_M

    def __post_init__(self):
        for name in ('observer_m', 'fast_observer_m', 'max_heading_fix_deg', 'k_e', 'max_bias_deg', 'rate_deg_s', 'jump_m',
                     'rejoin_m', 'rejoin_s', 'offset_reject_deg', 'gyro_max_deg_s', 'fit_bin_m', 'fit_noise_m',
                     'fit_prior_m', 'fit_prior_deg', 'jump_window_m', 'jump_slope_deg', 'fit_prior_min_deg',
                     'fit_prior_max_deg', 'carry_fit_sigma_deg', 'fit_rate_m', 'fit_beta_rate_deg', 'jump_accept_s', 'jump_accept_max_m',
                     'fit_max_sigma_m', 'fit_max_pair_m', 'fit_accept_sigma_m', 'fit_settle_s', 'fit_pair_settle_s',
                     'fit_low_sats_max_sigma_m', 'fit_low_sats_max_pair_m',
                     'antenna_pick_pair_m', 'antenna_agree_m', 'antenna_margin_m', 'antenna_fresh_s',
                     'runup_rtk_tau_m', 'runup_k_e', 'runup_max_bias_deg', 'runup_rate_deg_s', 'runup_target_accel_deg_s2', 'runup_blend_m',
                     'fit_prior_rate_deg', 'fit_walk_r_deg', 'fit_max_rate_deg', 'fit_start_beta_rate_deg', 'fit_runup_beta_rate_deg',
                     'persist_m', 'persist_e_m', 'position_follow_min_e_m', 'position_follow_confirm_m',
                     'position_follow_tau_m', 'position_follow_near_tau_m', 'fit_anchor_sigma', 'target_accel_deg_s2',
                     'head_k_e', 'head_max_bias_deg', 'crab_prior_sigma', 'crab_max', 'crab_roll_tau_s',
                     'crab_max_roll_deg', 'crab_roll_fresh_s', 'boost_e_m', 'boost_release_e_m', 'boost_persist_m',
                     'boost_max_bias_deg', 'grass_window_m', 'grass_on_a', 'grass_off_a', 'grass_min_speed_mps',
                     'grass_current_fresh_s', 'grass_fit_walk_b_deg', 'poor_sky_sigma_m', 'poor_sky_sigma_tau_s',
                     'poor_sky_beta_rate_deg', 'poor_sky_follow_e_m', 'poor_sky_exit_deg',
                     'poor_sky_exit_sigma', 'poor_sky_exit_e_m', 'poor_sky_exit_m'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' 必须是有限正数')
        if type(self.grass_enabled) is not bool:
            raise ValueError('grass_enabled 必须是 True 或 False')
        if type(self.poor_sky_enabled) is not bool:
            raise ValueError('poor_sky_enabled 必须是 True 或 False')
        if not (math.isfinite(self.poor_sky_pull_scale) and 0.0 <= self.poor_sky_pull_scale <= 1.0):
            raise ValueError('poor_sky_pull_scale 必须在 0..1')
        if self.grass_off_a > self.grass_on_a:
            raise ValueError('grass_off_a 不能大于 grass_on_a')
        if not math.isfinite(self.crab_prior) or abs(self.crab_prior) > self.crab_max:
            raise ValueError('crab_prior 必须在 ±crab_max 内')
        if not math.isfinite(self.crab_walk) or self.crab_walk < 0:
            raise ValueError('crab_walk 必须是有限非负数')
        if type(self.crab_enabled) is not bool:
            raise ValueError('crab_enabled 必须是 True 或 False')
        if self.boost_release_e_m > self.boost_e_m:
            raise ValueError('boost_release_e_m 不能大于 boost_e_m')
        for name in ('deadband_m', 'paint_deadband_m', 'trust_from_m', 'fast_m', 'min_baseline_m', 'max_baseline_m', 'baseline_hold_s',
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
        self._runup = False          # 这一拍在助跑里（调用方给的 fast，不含推算开始 fast_m）
        self.no_memory = False       # 这条线开头没有上一条线拟合好的换算（重启后第一条，调用方按规划设，见 route_plan first_line）
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
        self._fit = None             # rtk_fit：(起点横偏, IMU 推算横移, [c, b, r, k], P, 当前这一格 [Σ路程, Σ残差, 个数, 起点, Σ∫横滚])
        self.crab = 0.0              # 此刻补的侧滑角（k × 横滚），rad；目标车头再减它（见 ImuFirstConfig.crab_*）
        self.roll = None             # 此刻用的横滚（低通、扣掉水平零点），rad；没有为空
        self.fit_e = None            # 慢速角度拟合给出的位置，和实际控制横偏分开记录
        self._position_e = None      # 持续可信横偏确认后的独立位置估计
        self._position_rel = None
        self._position_heading = 0.0
        self._position_sign = 0
        self._position_confirmed_m = 0.0
        self._position_correction = 0.0
        self.grass_weight = 0.0      # 草地模式权重 0..1（跟踪器每拍给，见 ImuFirstConfig.grass_*）：拟合角度游走往 grass_fit_walk_b_deg 放
        self.poor_sky = False        # 楼下模式（ImuFirstConfig.poor_sky_*）：这条线进了就不退，除非出口
        self.poor_sky_exit = None    # 退出的原因：angle 角度对不上 / offset 车真偏了 / grass 草地；退了这条线不再进
        self._poor_far = (0, 0.0)    # 楼下模式里可信卫星说喷头同一边偏超过 poor_sky_exit_e_m 的 (边, 走了多远)

    def set_poor_sky(self, poor, grass):
        """跟踪器每拍给：poor 卫星差（σ 低通超过 poor_sky_sigma_m），grass 认出草地。见 ImuFirstConfig.poor_sky_*。"""
        if not self.cfg.poor_sky_enabled or self.poor_sky_exit is not None:
            return
        if grass:
            if self.poor_sky:
                self._leave_poor_sky('grass')
            return
        if poor:
            self.poor_sky = True

    def _leave_poor_sky(self, reason):
        self.poor_sky = False
        self.poor_sky_exit = reason

    def _persist_e_m(self):
        """真偏（_persist_check）门槛：楼下模式放到 poor_sky_follow_e_m。"""
        cfg = self.cfg
        return max(cfg.persist_e_m, cfg.poor_sky_follow_e_m) if self.poor_sky else cfg.persist_e_m

    def _walk_b(self):
        """拟合角度 b 的随机游走，rad/√m：平地 fit_walk_b_deg，草地按 grass_weight 放到 grass_fit_walk_b_deg。"""
        cfg = self.cfg
        w = max(0.0, min(1.0, self.grass_weight))
        return (cfg.fit_walk_b_deg + w * (cfg.grass_fit_walk_b_deg - cfg.fit_walk_b_deg)) * DEG

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

    def blend_start(self, predicted, align_cfg, rate=None, crab=None):
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
        if crab is not None and all(math.isfinite(v) for v in crab):
            self.start['crab'] = crab

    def fit_rate(self):
        """rtk_fit：拟合出的漂移速度 (rad/m, 误差 rad/m)，还没开始拟合为 None。"""
        return None if self._fit is None else (self._fit['x'][2], math.sqrt(max(0.0, self._fit['P'][2][2])))

    def fit_sigma(self):
        """rtk_fit：拟合出的角度差的误差（rad），还没开始拟合为 None。"""
        return None if self._fit is None else math.sqrt(max(0.0, self._fit['P'][1][1]))

    def fit_angle(self):
        """rtk_fit：拟合出的此刻角度差（rad，不按 fit_beta_rate 限速跟），还没开始拟合为 None。"""
        if self._fit is None:
            return None
        lim = math.sin(self.cfg.max_heading_fix_deg * DEG)
        return math.asin(max(-lim, min(lim, self._fit_now(self._travel)[1])))

    def update(self, measured, rel, distance, dt, nozzle, rtk_ok, fast=False, steering=False, roll=None):
        """锁住偏移以后每拍一次。measured：卫星（融合位置，天线杠杆臂按 rel + β 改算）量的喷头横偏，m；
        rel：IMU 参考航向 − 线方向，rad；distance：这一拍往前走的路，m；nozzle：喷头（车体前, 左），m；
        rtk_ok：这一拍卫星位置可以用来修正；fast：对准阶段（调用方另判，推算开始 fast_m 内也算）；
        steering：IMU 内环已经在开（rtk_trim、rtk_fit 都关时，这以后卫星不再修正）；roll：此刻横滚（低通、扣零点，rad），
        没有为空（不补侧滑）。返回喷头横偏估计。"""
        cfg = self.cfg
        self.fast = fast or self._travel < cfg.fast_m
        self._runup = bool(fast)
        if cfg.rtk_fit:
            return self._fit_update(measured, rel, distance, dt, nozzle, rtk_ok, roll)
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
        """助跑段（还没喷）：推算横偏按 fraction（0–1）直接往卫星横偏 measured 靠；拟合的修正量一起挪，开喷时接着用。

        没有记忆的线（no_memory）卡尔曼状态 x 不挪，只按卫星点更新：2026-11-22 仿真，每拍把 x 的修正量 c 也往卫星拉，
        起步换算角度错 1.8° 时卫星和推算按距离拉开的差全被这里当位置吃掉，助跑 2 m 拟合角度只学到 0.6°（拉开 5.6 cm），
        线头 −3.3 cm 的那种第一条就是这样；不挪以后走 1.2 m 就学到 1.6°（真值 1.8°）。有记忆的线开头换算本来就准，
        照旧一起挪（仿真：也不挪时后面几条线头中位 0.29 → 0.37 cm）。"""
        if self.e is None or self._fit is None or not math.isfinite(measured):
            return
        d = max(0.0, min(1.0, fraction)) * (measured - self.e)
        self._fit['c'] += d
        if not self.no_memory:
            self._fit['x'][0] += d
        self.e += d
        if self._position_e is not None:
            self._position_e += d
        if self.fit_e is not None:
            self.fit_e += d

    def source_changed(self):
        """卫星横偏换了来源（主 / 副天线）：跳点参照从这一拍重新攒，两根天线之间的差不算跳。"""
        self._fit_hist.clear()
        self._position_sign = 0
        self._position_confirmed_m = 0.0

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

    def _fit_update(self, measured, rel, distance, dt, nozzle, rtk_ok, roll=None):
        """rtk_fit（见 ImuFirstConfig）：喷头横偏 = IMU 推算 + 修正 c；卫星可用时每 fit_bin_m 平均成一个点，卡尔曼跟踪
        [c, 角度 b, 角度变化 r, 侧滑 k]（c 随路程按 b + k × 横滚 增长，都按距离随机游走）；用到推算和目标车头上的 c、b
        再按距离限速，侧滑按 k × 此刻横滚直接补（crab）。"""
        cfg = self.cfg
        use_roll = (cfg.crab_enabled and roll is not None and math.isfinite(roll)
                    and abs(roll) <= cfg.crab_max_roll_deg * DEG)
        phi = roll if use_roll else 0.0
        self.roll = roll if use_roll else None
        if self.e is None:
            self.rtk_accepted = rtk_ok
            prior = max(cfg.fit_prior_min_deg, min(cfg.fit_prior_max_deg, cfg.fit_prior_deg * self._prior_scale))
            sc, sb, sr = cfg.fit_prior_m, math.sin(prior * DEG), cfg.fit_prior_rate_deg * DEG
            r0 = 0.0
            carried = (self.start or {}).get('rate')
            if carried is not None:                  # 上一条线学到的漂移速度（线头就是卫星差的地方也能按它推）
                r0, sr = carried[0], min(sr, carried[1])
            k0, sk = (cfg.crab_prior, cfg.crab_prior_sigma) if cfg.crab_enabled else (0.0, 0.0)
            crab = (self.start or {}).get('crab')
            if cfg.crab_enabled and crab is not None:  # 上一条线学到的侧滑（同一块场地基本不变）
                k0, sk = crab[0], min(sk, crab[1])
            self._fit = dict(e0=measured, imu=0.0, x=[0.0, 0.0, r0, k0],
                             P=[[sc * sc, 0.0, 0.0, 0.0], [0.0, sb * sb, 0.0, 0.0], [0.0, 0.0, sr * sr, 0.0],
                                [0.0, 0.0, 0.0, sk * sk]],
                             u=0.0, Phi=0.0, phi_total=0.0, cell=[0.0, 0.0, 0, 0.0, 0.0], c=0.0, b=0.0, anchor=not rtk_ok)
            self.e, self._rel, self._travel, self.innovation = measured, rel, 0.0, 0.0
            self.fit_e = measured
            self.crab = k0 * phi
            return self.e
        nx, ny = nozzle
        h, hp = rel, self._rel
        f = self._fit
        f['imu'] += distance * math.sin(0.5 * (h + hp)) + nx * (math.sin(h) - math.sin(hp)) + ny * (math.cos(h) - math.cos(hp))
        self._rel = rel
        self._travel += abs(distance)
        f['phi_total'] += phi * abs(distance)
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
            cell[4] += f['phi_total']
            if u - cell[3] >= cfg.fit_bin_m:
                hu, z, ph = cell[0] / cell[2], cell[1] / cell[2], cell[4] / cell[2]
                if f['anchor'] and self._fit_start_wrong(hu, z, ph):
                    self._fit_anchor(hu, z, ph)
                else:
                    f['anchor'] = False
                    self._persist_check(hu, z, ph)
                    self._fit_step(hu, z, ph)
                cell[:] = [0.0, 0.0, 0, 0.0, 0.0]
                self.fit_points += 1
                c_now, b_now = self._fit_now(u)
        else:
            cell[:] = [0.0, 0.0, 0, 0.0, 0.0]
            self._persist = None     # 没有可信观测的里程不能算成“持续偏差”
        lim = math.sin(cfg.max_heading_fix_deg * DEG)
        x[1] = max(-lim, min(lim, x[1]))
        b_now = max(-lim, min(lim, b_now))
        step = abs(distance)
        km = cfg.crab_max
        self.crab = max(-km, min(km, x[3])) * phi
        # 用到车头上的角度（β 和补的侧滑）带来的横移照常补（IMU 推算不含）；只有额外的纠正按 fit_rate_m 限速
        f['c'] += (f['b'] + self.crab) * step
        f['c'] += max(-cfg.fit_rate_m * step, min(cfg.fit_rate_m * step, c_now - f['c']))
        enough = self.fit_points * cfg.fit_bin_m >= cfg.fit_start_points_m - 1e-9
        start = u < cfg.fit_start_m and enough
        if self.poor_sky and enough:
            # 楼下模式出口：拟合出的角度和正在用的差太多（带过来的角度错了），退回平常按 fit_*_beta_rate 跟
            sb = math.sqrt(max(0.0, f['P'][1][1]))
            if abs(b_now - f['b']) > max(math.sin(cfg.poor_sky_exit_deg * DEG), cfg.poor_sky_exit_sigma * sb):
                self._leave_poor_sky('angle')
        rate = cfg.fit_start_beta_rate_deg if start else cfg.fit_beta_rate_deg
        if self.poor_sky and not start and u >= cfg.fit_start_m:
            rate = cfg.poor_sky_beta_rate_deg
        if self._runup and enough and self.no_memory:
            rate = max(rate, cfg.fit_runup_beta_rate_deg)   # 没有记忆：助跑里不喷，角度按卫星轨迹尽快用上
        db = rate * DEG * step
        f['b'] += max(-db, min(db, b_now - f['b']))
        self.beta = math.asin(max(-lim, min(lim, f['b'])))
        self.fit_e = base + f['c']
        self.e = self._follow_position(measured, rel, distance, nozzle)
        if self.poor_sky and self.rtk_accepted and not self.fast:
            # 楼下模式出口：可信卫星说喷头同一边偏超过 poor_sky_exit_e_m 走满 poor_sky_exit_m（车真偏了，要往回拉）
            side = 0 if abs(measured) <= cfg.poor_sky_exit_e_m else (1 if measured > 0 else -1)
            far = self._poor_far[1] + step if side and side == self._poor_far[0] else (step if side else 0.0)
            self._poor_far = (side, far)
            if side and far >= cfg.poor_sky_exit_m - 1e-9:
                self._leave_poor_sky('offset')
        return self.e

    def _follow_position(self, measured, rel, distance, nozzle):
        """可信的持续同侧偏差单独跟随位置；航向参数保留原来的慢拟合和跳点判断。

        确认必须连续且有实际轮程，助跑不累计；丢定位后保留已确认的位置，只按惯性预测。
        β/侧滑是模型修正，不是车头真的转动，不能进入喷头转动杠杆项。
        """
        cfg, step = self.cfg, abs(distance)
        sign = 1 if measured > 0.0 else -1
        min_e = max(cfg.position_follow_min_e_m, cfg.poor_sky_follow_e_m) if self.poor_sky else cfg.position_follow_min_e_m
        qualified = self.rtk_accepted and not self.fast and abs(measured) > min_e
        if qualified:
            if sign != self._position_sign:
                self._position_confirmed_m = 0.0
            self._position_sign = sign
            self._position_confirmed_m += step
        else:
            self._position_sign = 0
            self._position_confirmed_m = 0.0
        self._position_correction = 0.0
        heading = self.beta + self.crab
        if self._position_e is None:
            if self._position_confirmed_m >= cfg.position_follow_confirm_m:
                self._position_e = self.fit_e
                self._position_rel = rel
                self._position_heading = heading
            return self.fit_e
        previous = self._position_rel
        # 两拍间实际转动用 IMU 相对角；参考角修正只参与平移方向。
        nx, ny = nozzle
        h = 0.5 * (rel + previous + heading + self._position_heading)
        self._position_e += (distance * math.sin(h)
                             + nx * (math.sin(rel + self.beta) - math.sin(previous + self.beta))
                             + ny * (math.cos(rel + self.beta) - math.cos(previous + self.beta)))
        self._position_rel, self._position_heading = rel, heading
        if self.rtk_accepted:
            tau = cfg.position_follow_tau_m if qualified else cfg.position_follow_near_tau_m
            correction = (1.0 - math.exp(-step / tau)) * (measured - self._position_e)
            limit = cfg.fit_rate_m * step
            self._position_correction = max(-limit, min(limit, correction))
            self._position_e += self._position_correction
        return self._position_e

    def _fit_start_wrong(self, hu, z, ph=None):
        """拟合起点那拍不可信时，第一个可信点（路程 hu、值 z、∫横滚 ph）和预测差多少个标准差，超过 fit_anchor_sigma 为 True。"""
        cfg, f = self.cfg, self._fit
        x, P = f['x'], f['P']
        d = max(0.0, hu - f['u'])
        F0 = (1.0, d, d * d / 2, (f['Phi'] if ph is None else ph) - f['Phi'])
        var = sum(F0[i] * P[i][j] * F0[j] for i in range(4) for j in range(4))
        var += cfg.fit_walk_c_m ** 2 * d + cfg.fit_noise_m ** 2
        r = z - sum(F0[i] * x[i] for i in range(4))
        return r * r > cfg.fit_anchor_sigma ** 2 * var

    def _fit_anchor(self, hu, z, ph=None):
        """rtk_fit：拟合起点那拍卫星不可信，这是第一个可信点：修正量就取这一点、按 fit_prior_m 信（同正常起点）；
        角度、漂移速度、侧滑的估计和不确定度照旧推到这一点，不拿起点的错去改角度。"""
        cfg, f = self.cfg, self._fit
        x, P = f['x'], f['P']
        d = max(0.0, hu - f['u'])
        wb, wr = self._walk_b() ** 2 * d, (cfg.fit_walk_r_deg * DEG) ** 2 * d
        wk = (cfg.crab_walk ** 2 * d) if cfg.crab_enabled else 0.0
        pbb = P[1][1] + 2.0 * d * P[1][2] + d * d * P[2][2] + wb
        pbr = P[1][2] + d * P[2][2]
        prr = P[2][2] + wr
        pkk = P[3][3] + wk
        x[:] = [z, x[1] + x[2] * d, x[2], x[3]]
        f['P'] = [[cfg.fit_prior_m ** 2, 0.0, 0.0, 0.0], [0.0, pbb, pbr, 0.0], [0.0, pbr, prr, 0.0], [0.0, 0.0, 0.0, pkk]]
        f['u'], f['anchor'] = hu, False
        if ph is not None:
            f['Phi'] = ph

    def _persist_check(self, hu, z, ph=None):
        """rtk_fit：这一点（路程 hu，卫星横偏 − IMU 推算 z）和拟合差多少。最近 persist_m 的点都在同一边差超过
        persist_e_m 的一半、平均超过 persist_e_m，放开位置修正量（差²）的不确定度。
        横向位置台阶不证明航向有误，不同时放大角度不确定度；角度由后续连续观测的斜率学习。
        这一点起拟合跟上（见 ImuFirstConfig.persist_m）。10-07 回放：残差在 1.0–1.5 cm 之间晃，逐点都要超 1 cm 会被
        0.99 断开，所以按平均。"""
        cfg, pe = self.cfg, self._persist_e_m()
        r = z - self._fit_now(hu, ph)[0]
        p = self._persist
        if abs(r) <= 0.5 * pe or p and (r > 0) != (p[-1][1] > 0):
            p = self._persist = None
        if abs(r) <= 0.5 * pe:
            return
        if p is None:
            p = self._persist = deque()
        p.append((hu, r))
        while len(p) > 1 and hu - p[1][0] >= cfg.persist_m - 1e-9:
            p.popleft()                                  # 只留最近 persist_m 的点
        mean = sum(v for _, v in p) / len(p)
        if hu - p[0][0] >= cfg.persist_m - 1e-9 and abs(mean) > pe:
            P = self._fit['P']
            P[0][0] += mean * mean
            self.persists += 1
            self._persist = None

    def _fit_now(self, u, ph=None):
        """卡尔曼状态（上一点）推到路程 u（这时 ∫横滚 ph，空 = 现在的）：(修正量 c, 角度 b)，含角度变化速度 r、侧滑 k。"""
        f = self._fit
        x, d = f['x'], u - f['u']
        dphi = (f['phi_total'] if ph is None else ph) - f['Phi']
        return x[0] + x[1] * d + x[2] * d * d / 2 + x[3] * dphi, x[1] + x[2] * d

    def _fit_step(self, hu, z, ph=None):
        """卡尔曼：状态 [c, b, r, k] 从上一点预测到路程 hu（c += b·Δ + r·Δ²/2 + k·Δ∫横滚，b += r·Δ，都按距离随机游走），
        再用这一点 z（卫星横偏 − IMU 推算）更新。"""
        cfg, f = self.cfg, self._fit
        x, P = f['x'], f['P']
        d = max(0.0, hu - f['u'])
        dphi = (f['Phi'] if ph is None else ph) - f['Phi']
        F = [[1.0, d, d * d / 2, dphi], [0.0, 1.0, d, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
        q = [cfg.fit_walk_c_m ** 2 * d, self._walk_b() ** 2 * d, (cfg.fit_walk_r_deg * DEG) ** 2 * d,
             cfg.crab_walk ** 2 * d if cfg.crab_enabled else 0.0]
        n = 4
        x[:] = [sum(F[i][k] * x[k] for k in range(n)) for i in range(n)]
        FP = [[sum(F[i][k] * P[k][j] for k in range(n)) for j in range(n)] for i in range(n)]
        P = [[sum(FP[i][k] * F[j][k] for k in range(n)) + (q[i] if i == j else 0.0) for j in range(n)] for i in range(n)]
        s = P[0][0] + cfg.fit_noise_m ** 2
        k = [P[i][0] / s for i in range(n)]
        r = z - x[0]
        x[:] = [x[i] + k[i] * r for i in range(n)]
        rl = cfg.fit_max_rate_deg * DEG
        x[2] = max(-rl, min(rl, x[2]))
        x[3] = max(-cfg.crab_max, min(cfg.crab_max, x[3]))
        f['P'] = [[P[i][j] - k[i] * P[0][j] for j in range(n)] for i in range(n)]
        f['u'] = hu
        f['Phi'] = f['Phi'] if ph is None else ph

    def fit_crab(self):
        """rtk_fit：拟合出的侧滑 k（侧滑 / 横滚）和误差；没开或还没拟合为 None。"""
        if self._fit is None or not self.cfg.crab_enabled:
            return None
        return self._fit['x'][3], math.sqrt(max(0.0, self._fit['P'][3][3]))

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
                    first_position_follow=self._position_e is not None,
                    first_position_fit_e=self.fit_e,
                    first_position_confirmed_m=round(self._position_confirmed_m, 4),
                    first_position_correction_m=self._position_correction,
                    first_fit_sigma_deg=None if self._fit is None else round(math.degrees(math.sqrt(max(0.0, self._fit['P'][1][1]))), 4),
                    first_fit_lag=None if self._fit is None else round(self._fit_now(self._travel)[0] - self._fit['c'], 4),
                    first_fit_rate_deg_m=None if self._fit is None else round(math.degrees(self._fit['x'][2]), 5),
                    first_crab_k=None if self._fit is None else round(self._fit['x'][3], 4),
                    first_crab_deg=round(math.degrees(self.crab), 4),
                    first_roll_deg=None if self.roll is None else round(math.degrees(self.roll), 3),
                    first_start_dual_deg=deg((self.start or {}).get('dual')),
                    first_start_carried_deg=deg((self.start or {}).get('carried')),
                    first_start_sigma_deg=deg((self.start or {}).get('sigma')),
                    first_start_weight=None if self.start is None else round(self.start['weight'], 3),
                    first_poor_sky=self.poor_sky, first_poor_sky_exit=self.poor_sky_exit)


@dataclass(frozen=True)
class AlignmentConfig:
    """IMU → 地面方向换算 c（真实车头 − IMU 偏航）跨线带过去（ImuAlignment）：只带上一条线卫星拟合好的 c，不学、不推算。

    2026-11-24 用户「删掉学习」：原来相邻两条线拟合好就在线学「c 每分钟变多少、每转一圈变多少」（卡尔曼），是给 10 月
    旧 IMU（HDA536T，c 随时间约 −0.07～−0.4°/min、转角刻度 −0.17%）用的。SCH16T 实测 c 随时间 +0.005 ± 0.034°/min、
    刻度已标定在 IMU 固件里（0x2012，user_rules.GEN5_IMU_YAW_SCALE_PPM；车上不做刻度，用户规矩），没什么可学。11-24 固件
    静止修正一下拨了 +4.12°，学习把它学成漂移 +0.35°/min、刻度 −0.44%，后面带过去的换算错 1.7–3.4°；按 11-24 的线顺序
    重放，不学（只带上一条线的 c）06:58 以后各条开头误差 −0.18～+0.07°。

    下面几项只用来放大带过去的 c 的误差（隔得越久、转得越多越不信，开头和双天线按误差加权），不推算 c：
    drift_sigma_deg_min、drift_walk（随时间）、scale_sigma（随转角）、walk_deg_sqrt_min（其余随机变化）。"""
    drift_sigma_deg_min: float = 0.05
    scale_sigma: float = 0.001
    walk_deg_sqrt_min: float = 0.10   # 模型以外的随机变化，°/√min
    drift_walk: float = 0.02          # °/min/√min
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
    # 2026-11-24：06:43 写过 IMU 参数后 SCH16T 固件偏航零偏不对（静止时偏航 −0.5～−0.95°/min），06:58 第一次安静停车，
    # 固件静止修正把偏航一下拨了 +4.12°（陀螺读数一直 0）。车停着（调用方给 still：两轮都停、陀螺 < still_max_gyro_deg_s）
    # 地面车头不变，IMU 偏航的变化全是 IMU 自己的：带过去的 c 跟着挪（c −= 变化），不算转角
    still_max_gyro_deg_s: float = 0.5

    def __post_init__(self):
        for name in ('drift_sigma_deg_min', 'scale_sigma', 'walk_deg_sqrt_min', 'drift_walk', 'obs_floor_deg', 'max_age_s', 'imu_gap_s',
                     'rate_walk_deg_m', 'still_max_gyro_deg_s', 'max_gap_deg', 'dual_sigma_deg'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' 必须是有限正数')


@dataclass(frozen=True)
class DriveLearnConfig:
    """开机后作业前的行驶里顺带学换算 c（DriveLearner）。门槛在 user_rules.DRIVE_LEARN_*。"""
    min_m: float = rules.DRIVE_LEARN_MIN_M
    max_sigma_deg: float = rules.DRIVE_LEARN_MAX_SIGMA_DEG
    keep_s: float = rules.DRIVE_LEARN_KEEP_S
    span_s: float = 0.8              # 车头方向用前后这么久的两个固定解连线，对中点时刻的 IMU 偏航
    min_speed_mps: float = 0.2
    max_rate_deg_s: float = 10.0     # 转得比这快不用（原地转、急弯：杆臂修正、时间对齐都不准）
    max_fix_sigma_m: float = 0.02    # 固定解 σ（水平合成）超过这个不用
    wheel_fresh_s: float = 0.3       # 轮速离那一刻这么近才拿来分前进、倒车
    corr_m: float = 0.5              # 误差按每这么多米一个独立的点算
    floor_deg: float = 0.15          # 学出来的误差至少按这么大（卫星慢变的偏差平均不掉）
    outlier_deg: float = 3.0         # 离中位这么远的点不用
    max_step_mps: float = 3.0        # 相邻两帧位置隔得比这个速度还远：当跳点，断开重连

    def __post_init__(self):
        for name in ('min_m', 'max_sigma_deg', 'keep_s', 'span_s', 'min_speed_mps', 'max_rate_deg_s', 'max_fix_sigma_m',
                     'wheel_fresh_s', 'corr_m', 'floor_deg', 'outlier_deg', 'max_step_mps'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' 必须是有限正数')


def _interp(seq, t):
    """seq：按时刻排好的 (t, v)；t 在范围里返回线性插值，不在返回 None。"""
    if not seq or t < seq[0][0] or t > seq[-1][0]:
        return None
    lo, hi = 0, len(seq) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if seq[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    (t0, v0), (t1, v1) = seq[lo], seq[hi]
    return v0 if t1 <= t0 else v0 + (v1 - v0) * (t - t0) / (t1 - t0)


class DriveLearner:
    """开机后作业前的行驶里顺带学 c（真实车头 − IMU 偏航）：开着（前进、倒车按轮速分）、RTK 固定、σ 小、转得不快时，
    span_s 前后两个固定解连线的方向减掉主天线杆臂随转弯的修正就是车头，c = 车头 − 中点时刻的 IMU 偏航。
    按路程加权、去掉离群，只算最近 keep_s 内的。停车时 IMU 偏航自己变（ImuAlignment.feed_imu still）跟着挪，IMU 断过清空。
    11-25 用户「开机多跑一会学习能不能省去」：学够了开机第一条就不空跑（ImuAlignment.adopt_drive）。"""

    def __init__(self, config=DriveLearnConfig()):
        self.cfg = config
        self.fixes = deque()             # (t, x, y) 连续的固定解
        self.yaws = deque()              # (t, 展开的 IMU 偏航)
        self.wheels = deque()            # (t, 轮速 m/s，倒车为负)
        self.samples = deque()           # (t, c − 当时的 _off, 这一步的路程 m)
        self._off = 0.0                  # 停车时 IMU 偏航自己变的累计（shift）：读的时候加回去，不用逐个改

    def reset(self):
        self.fixes.clear()
        self.yaws.clear()
        self.wheels.clear()
        self.samples.clear()
        self._off = 0.0

    def shift(self, dc):
        """车停着 IMU 偏航自己变了 −dc：学到的 c 跟着挪。"""
        self._off = wrap(self._off + dc)

    def on_imu(self, t, yaw):
        self.yaws.append((t, yaw))
        while self.yaws and self.yaws[0][0] < t - 3.0 * self.cfg.span_s - 1.0:
            self.yaws.popleft()

    def on_wheel(self, t, speed):
        if not (math.isfinite(t) and math.isfinite(speed)):
            return
        if self.wheels and t < self.wheels[-1][0]:
            return
        self.wheels.append((t, speed))
        while self.wheels and self.wheels[0][0] < t - 3.0 * self.cfg.span_s - 1.0:
            self.wheels.popleft()

    def _wheel_at(self, t):
        best = None
        for tw, v in self.wheels:
            if abs(tw - t) <= self.cfg.wheel_fresh_s and (best is None or abs(tw - t) < abs(best[0] - t)):
                best = (tw, v)
        return None if best is None else best[1]

    def on_fix(self, t, x, y, ok, lever):
        """一帧主天线位置（本地米）。ok：固定解且 σ ≤ max_fix_sigma_m；不 ok 断开（不跨着连）。lever：主天线（前、左）m。"""
        cfg = self.cfg
        if not ok or not all(math.isfinite(v) for v in (t, x, y)):
            self.fixes.clear()
            return
        if self.fixes and t <= self.fixes[-1][0]:
            return
        if self.fixes and math.hypot(x - self.fixes[-1][1], y - self.fixes[-1][2]) > cfg.max_step_mps * (t - self.fixes[-1][0]):
            self.fixes.clear()           # 位置跳了（车不可能这么快）：从这一帧重新连
        self.fixes.append((t, x, y))
        while len(self.fixes) > 2 and self.fixes[1][0] <= t - cfg.span_s:
            self.fixes.popleft()
        ta, xa, ya = self.fixes[0]
        dt = t - ta
        if dt < 0.6 * cfg.span_s or dt > 1.5 * cfg.span_s:
            return
        dx, dy = x - xa, y - ya
        v = math.hypot(dx, dy) / dt
        if v < cfg.min_speed_mps:
            return
        tm = 0.5 * (t + ta)
        ya0, ya1, ym = _interp(self.yaws, ta), _interp(self.yaws, t), _interp(self.yaws, tm)
        wheel = self._wheel_at(tm)
        if ya0 is None or ya1 is None or ym is None or wheel is None or abs(wheel) < 0.5 * cfg.min_speed_mps:
            return
        om = (ya1 - ya0) / dt
        if abs(om) > math.radians(cfg.max_rate_deg_s):
            return
        sv = 1.0 if wheel > 0 else -1.0
        ax, ay = lever
        # 天线在车体系的速度 = (车速 − ω·ay, ω·ax)，模长是卫星量的 v：侧向分量已知，纵向分量按前进 / 倒车开方
        side = ax * om
        head = math.atan2(dy, dx) - math.atan2(side, sv * math.sqrt(max(v * v - side * side, 0.0)))
        tp, xp, yp = self.fixes[-2]
        self.samples.append((tm, wrap(head - ym - self._off), math.hypot(x - xp, y - yp)))
        while self.samples and self.samples[0][0] < t - cfg.keep_s:
            self.samples.popleft()

    def estimate(self, now):
        """最近 keep_s 内学到的 (c, σ, 路程 m)；不够 min_m 返回 None（σ 由调用方和 max_sigma_deg 比）。"""
        cfg = self.cfg
        while self.samples and self.samples[0][0] < now - cfg.keep_s:
            self.samples.popleft()
        if sum(w for _, _, w in self.samples) < cfg.min_m:
            return None
        sw = sum(w * math.sin(c) for _, c, w in self.samples)
        cw = sum(w * math.cos(c) for _, c, w in self.samples)
        c0 = math.atan2(sw, cw)
        d = sorted(wrap(c - c0) for _, c, _ in self.samples)
        c0 = wrap(c0 + d[len(d) // 2])
        kept = [(wrap(c - c0), w) for _, c, w in self.samples if abs(wrap(c - c0)) <= math.radians(cfg.outlier_deg)]
        c0 = wrap(c0 + self._off)
        dist = sum(w for _, w in kept)
        if dist < cfg.min_m:
            return None
        m = sum(e * w for e, w in kept) / dist
        var = sum(w * (e - m) ** 2 for e, w in kept) / dist
        sigma = max(math.radians(cfg.floor_deg), math.sqrt(var / max(1.0, dist / cfg.corr_m)))
        return wrap(c0 + m), sigma, dist


class ImuAlignment:
    """任务层持有、跨线跨作业共用的 IMU → 地面方向换算 c（真实车头 − IMU 偏航，rad）：

    - feed_imu：每帧 IMU 偏航，累出展开的转角 θ（掉头转了几圈都记得）；车停着时偏航的变化是 IMU 自己的，c 跟着挪；
    - 每条线开头 begin_line（上一条线记下的 c 定下来），这条线卫星拟合好了 update_line 记下 (时刻, θ, c, 误差)；
    - predict：上一条线的 c（不学、不推算，见 AlignmentConfig），误差按隔的时间、转角放大；开头和双天线测的按误差加权
      （ImuFirstGuide.blend_start）。"""

    def __init__(self, config=AlignmentConfig()):
        self.cfg = config
        self.theta = self._yaw = self._imu_t = None
        self.obs = None                  # 上一条线定下来的 (t, θ, c, σ)；σ 为空：没经卫星拟合（只借，不加权）
        self.pending = None              # 这条线正在更新的
        self.token = 0
        self.rate = None                 # 最近一条线拟合好的漂移速度 (时刻, rad/m, 误差 rad/m)，下一条线开头带过去
        self._rate_pending = None
        self.crab = None                 # 最近一条线拟合好的侧滑 k (时刻, k, 误差)，下一条线开头带过去（同一块场地）
        self._crab_pending = None
        self.still_shift = 0.0           # 车停着时 IMU 偏航自己变了多少（累计，rad），换算已跟着挪
        self.drive = DriveLearner()      # 开机后作业前的行驶里顺带学的 c（adopt_drive）
        self.adopted = None              # 最近一次把行驶里学的当记忆用：(时刻, c, σ, 路程 m)
        self._raw = None                 # 展开的 IMU 偏航（停车时的变化也算，DriveLearner 用）
        self._drive_status = None

    # 旧接口：value / t 是最近一条线的 c 和时刻
    @property
    def value(self):
        o = self.pending or self.obs
        return None if o is None else o[2]

    @property
    def t(self):
        o = self.pending or self.obs
        return None if o is None else o[0]

    def remembered(self):
        """有没有能带到下一条线开头的换算：最近一条线经卫星拟合好（不是只借来的）、离最近一帧 IMU 不超过 max_age_s、
        IMU 没断过。重启后第一条没有（LineMission.first_line_no_memory）。"""
        o = self.pending or self.obs
        t = self._imu_t
        return (o is not None and o[3] is not None and t is not None and math.isfinite(t)
                and 0.0 <= t - o[0] <= self.cfg.max_age_s)

    def feed_imu(self, t, yaw, still=False):
        """每帧 IMU 偏航。still：车停着（调用方判：两轮都停够久、陀螺 < still_max_gyro_deg_s）——地面车头没变，偏航的变化
        是 IMU 自己的（固件静止修正、零偏漂），换算 c 跟着挪、不算转角（见 AlignmentConfig.still_max_gyro_deg_s）。"""
        if not (math.isfinite(t) and math.isfinite(yaw)):
            return
        if self._yaw is None or t - self._imu_t > self.cfg.imu_gap_s:
            if self._yaw is not None:
                self.obs = self.pending = None   # IMU 断过：转角接不上
            self.theta = self._raw = yaw
            self.drive.reset()
        elif still:
            d = wrap(yaw - self._yaw)
            self.still_shift += d
            self._raw += d
            self.obs = self._shifted(self.obs, -d)
            self.pending = self._shifted(self.pending, -d)
            self.drive.shift(-d)
        else:
            d = wrap(yaw - self._yaw)
            self.theta += d
            self._raw += d
        self._yaw, self._imu_t = yaw, t
        self.drive.on_imu(t, self._raw)

    def adopt_drive(self):
        """没有能带的换算（remembered() 为假）时，开机后行驶里学够的 c（DriveLearner，误差 ≤ max_sigma_deg）当作上一条线
        定下来的（obs）用：开机第一条就不空跑、按有记忆走（LineMission._plan_route）。返回 (c, σ, 路程 m)，没用上返回 None。"""
        if self.remembered() or self._imu_t is None or self.theta is None:
            return None
        est = self.drive.estimate(self._imu_t)
        if est is None or est[1] > math.radians(self.drive.cfg.max_sigma_deg):
            return None
        c, sigma, dist = est
        self.obs, self.pending = (self._imu_t, self.theta, c, sigma), None
        self.adopted = (self._imu_t, c, sigma, dist)
        return est

    @staticmethod
    def _shifted(o, dc):
        return None if o is None else (o[0], o[1], wrap(o[2] + dc), o[3])

    def begin_line(self):
        """新的一条线开头（偏移刚锁住）调用：上一条线记下的 c 定下来。返回这条线的号。"""
        if self._rate_pending is not None:
            self.rate, self._rate_pending = self._rate_pending, None
        if self._crab_pending is not None:
            self.crab, self._crab_pending = self._crab_pending, None
        if self.pending is not None:
            self.obs = self.pending
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

    def update_crab(self, token, t, k, sigma):
        """这条线拟合出的侧滑 k（侧滑 / 横滚）和误差，下一条线开头带过去。"""
        if token == self.token and all(math.isfinite(v) for v in (t, k, sigma)):
            self._crab_pending = (t, k, sigma)

    def predict_crab(self, now):
        """上一条线的侧滑 (k, 误差)；太旧（> max_age_s）返回 None。场地不变，k 不随转角、时间漂，误差不放大。"""
        c = self.crab
        if c is None or not math.isfinite(now) or not 0.0 <= now - c[0] <= self.cfg.max_age_s:
            return None
        return c[1], c[2]

    def better_than_carried(self, now, sigma):
        """误差 sigma（rad）比现在能带过去的 c 准：这条线还没记下同样准或更准的，且上一条线带到现在的更不准或没有。"""
        if self.pending is not None and self.pending[3] is not None and self.pending[3] <= sigma:
            return False
        p = self.predict(now)
        return p is None or sigma < p[1]

    def update_line(self, token, t, c, sigma=None):
        """这条线的 c（偏移 + β）；sigma（rad）为空表示没经卫星拟合，只给下一条借、不参与加权。"""
        if token == self.token and math.isfinite(t) and math.isfinite(c):
            if sigma is None and self.pending is not None and self.pending[3] is not None:
                return
            self.pending = (t, self.theta, wrap(c), None if sigma is None else max(sigma, self.cfg.obs_floor_deg * DEG))

    def remember(self, t, value):
        self.update_line(self.token, t, value)

    def predict(self, now):
        """上一条线（这条线开头时就是 obs）的 c 带到现在：(c, σ) rad；没有、太旧、转角接不上返回 None。"""
        o = self.obs
        if o is None or not math.isfinite(now) or not 0.0 <= now - o[0] <= self.cfg.max_age_s:
            return None
        dt = (now - o[0]) / 60.0
        if o[1] is None or self.theta is None:
            return None
        dth = self.theta - o[1]
        cfg = self.cfg
        pdd = (cfg.drift_sigma_deg_min * DEG) ** 2 + (cfg.drift_walk * DEG) ** 2 * dt
        var = ((o[3] if o[3] is not None else cfg.dual_sigma_deg * DEG) ** 2 + dt * dt * pdd + dth * dth * cfg.scale_sigma ** 2
               + (cfg.walk_deg_sqrt_min * DEG) ** 2 * dt)
        return o[2], math.sqrt(max(var, 0.0))

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
        now = self._imu_t
        if now is not None and (self._drive_status is None or not 0.0 <= now - self._drive_status[0] < 1.0):
            self._drive_status = (now, self.drive.estimate(now))      # 每拍都要 status：学到的每秒算一次
        drive = None if self._drive_status is None else self._drive_status[1]
        return dict(align_still_shift_deg=deg(self.still_shift), align_theta_deg=deg(self.theta),
                    align_c_deg=deg(None if self.obs is None else self.obs[2]),
                    align_drive_m=None if drive is None else round(drive[2], 2),
                    align_drive_sigma_deg=None if drive is None else deg(drive[1]),
                    align_adopted_m=None if self.adopted is None else round(self.adopted[3], 2))


def paint_weight(cfg, along_m):
    """开喷处后 along_m 米时线头段之后那组参数（往回拉、死区、草地模式）的权重：head_m 内 0，再 head_blend_m 内线性到 1。"""
    if cfg.head_blend_m > 0:
        return max(0.0, min(1.0, (along_m - cfg.head_m) / cfg.head_blend_m))
    return 1.0 if along_m >= cfg.head_m else 0.0


def paint_pull(cfg, along_m, scale=1.0):
    """画线时往回拉的 (k_e, 车头最多偏几度)：开喷处后 along_m 米。head_m 内按 head_k_e / head_max_bias_deg，
    再 head_blend_m 内线性过渡到 k_e / max_bias_deg（ImuFirstConfig）。scale 只乘线头段之后那部分（楼下模式
    poor_sky_pull_scale），线头段照旧。"""
    w = paint_weight(cfg, along_m)
    return ((1.0 - w) * cfg.head_k_e + w * cfg.k_e * scale,
            (1.0 - w) * cfg.head_max_bias_deg + w * cfg.max_bias_deg * scale)


def paint_deadband(cfg, along_m):
    """画线时往回拉的死区，m：线头段（和不在喷时 along_m = 0）按 deadband_m，之后同 paint_pull 过渡到 paint_deadband_m。"""
    w = paint_weight(cfg, along_m)
    return (1.0 - w) * cfg.deadband_m + w * cfg.paint_deadband_m


class GroundLoad:
    """草地识别（ImuFirstConfig.grass_*，2026-11-21 用户）：按走过的路（喷线中、车速够才记）记两轮电机电流（|A| 平均），取最近 grass_window_m
    的中位数，≥ grass_on_a 算草地，≤ grass_off_a 回平地（中间保持）。窗口没走满不算草地；读数断了清空。纯计算。"""

    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.grass = False
        self.median = None               # 最近 grass_window_m 电流中位数，A；窗口没走满为空
        self._dist = 0.0
        self._samples = deque()          # (走过的路 m, 电流 A)

    def update(self, step_m, current_a):
        """这一拍走了 step_m（只在车速够时给，停车、原地转给 0 不记）、两轮电流 |A| 平均（读数断了为 None：清空、
        回平地）。返回是不是草地。"""
        cfg = self.cfg
        if current_a is None or not math.isfinite(current_a):
            self.reset()
            return False
        if not step_m > 0.0:
            return self.grass
        self._dist += step_m
        self._samples.append((self._dist, abs(current_a)))
        while len(self._samples) > 1 and self._dist - self._samples[1][0] >= cfg.grass_window_m:
            self._samples.popleft()
        if self._dist - self._samples[0][0] < 0.95 * cfg.grass_window_m:
            self.median = None                           # 窗口还没走满
            return self.grass
        values = sorted(v for _, v in self._samples)
        n = len(values)
        self.median = values[n // 2] if n % 2 else 0.5 * (values[n // 2 - 1] + values[n // 2])
        if self.grass and self.median <= cfg.grass_off_a:
            self.grass = False
        elif not self.grass and self.median >= cfg.grass_on_a:
            self.grass = True
        return self.grass


class PullBoost:
    """画线时往回拉加强（ImuFirstConfig.boost_*，2026-10-11 用户）：卫星可信、喷头横偏同一边持续超过 boost_e_m 走满
    boost_persist_m 就打开，回到 boost_release_e_m 以内或卫星不可信就关。每条直线 reset。"""

    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.on, self.sign, self.dist = False, 0, 0.0

    def update(self, e, rtk_good, step):
        cfg = self.cfg
        if e is None or not math.isfinite(e) or not rtk_good:
            self.reset()
            return False
        if abs(e) <= cfg.boost_release_e_m:
            self.reset()
        elif abs(e) > cfg.boost_e_m:
            sign = 1 if e > 0 else -1
            if sign != self.sign:
                self.on, self.sign, self.dist = False, sign, 0.0
            self.dist += max(0.0, step)
            if self.dist >= cfg.boost_persist_m - 1e-9:
                self.on = True
        return self.on

    def limit(self, pull):
        """paint_pull 给的 (k_e, 最多偏几度)：打开时上限放到 boost_max_bias_deg（线头段本来更大的照旧）。"""
        return (pull[0], max(pull[1], self.cfg.boost_max_bias_deg)) if self.on else pull


def runup_weight(cfg, along_m):
    """线头前保持助跑控制，线头后按距离连续退出；两端斜率为零。"""
    u=max(0.,min(1.,along_m/cfg.runup_blend_m))
    return 1.-u*u*(3.-2.*u)


def imu_first_target(cfg, guide, target, line_yaw, dt, align_rate_deg_s, align_deg, aligned, pull=None, entry_weight=0.,
                     deadband=None):
    """目标车头（IMU 系）：线方向 − 偏移 − β + 横偏往回拉（推算横偏 deadband 内不拉，None 用 cfg.deadband_m；画线时见
    paint_deadband）。还没对准过、或在对准阶段（guide.fast）按 align_rate_deg_s 转，上线以后每秒最多改 cfg.rate_deg_s。
    pull：(k_e, 车头最多偏几度)，None 用 cfg.k_e / cfg.max_bias_deg（画线时见 paint_pull）。entry_weight 是助跑控制权重 0..1。
    返回 (目标, 往回拉的偏置, 在对准, 对准过)。"""
    k_e, max_bias_deg = (cfg.k_e, cfg.max_bias_deg) if pull is None else pull
    k_e += entry_weight*(cfg.runup_k_e-k_e)
    max_bias_deg += entry_weight*(cfg.runup_max_bias_deg-max_bias_deg)
    e = guide.e
    excess = max(0.0, abs(e) - (cfg.deadband_m if deadband is None else deadband))
    lim = max_bias_deg * DEG
    bias = max(-lim, min(lim, -math.atan(k_e * math.copysign(excess, e))))
    desired = wrap(line_yaw - guide.offset - guide.beta - getattr(guide, 'crab', 0.0) + bias)
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
