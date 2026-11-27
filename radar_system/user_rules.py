"""车端规则表：用户定过的门槛都在这一个文件里（2026-11-22 整理，数值和整理前一样）。

改一条规则只改这里；各模块原来的名字（um982.HEADING_BASELINE_MIN_M、ImuFirstConfig.fit_min_baseline_m 等）
都还在，只是取这里的值，所以旧代码、测试里 mock 原名字照样有效。每条后面写了用在哪、谁哪天定的；
为什么取这个值、试过什么，看用的地方原来的注释。

不在这里的（写在配置文件里，车上按文件生效）：
- robot_core/robot_gen5.json：喷头位置 nozzle_x_m 0.235 / nozzle_right_m 0.445（11-19）、
  控制指令超时 safety.command_timeout_s 1.5 s（10-08）、驱动器不回复 feedback_timeout_s 2.0 s（10-06）、
  作业原地转 1.5 rad/s / 刹车 2.0 rad/s²（11-21）、driver.acceleration_m_s2 1.0（定位暂停渐停用它，10-04「只有急停硬刹」）
- oid_protocol/oid.yaml：CAN 500k、左轮 ID 126、右轮 ID 116、帧间隔 3 ms、40 Hz（11-16 / 11-19）、
  闭环电流最多 40 A（11-21）、手动松手渐停 1.0 m/s²（11-19）
- 车上 ~/rtk/fusion.json：天线高、安装偏差等标定值（文件里写了就按文件）
"""

# ───────────── 一、卫星、测向什么时候算数 ─────────────

# 测向基线带：双天线测向整周固定、接收机报的基线在这范围内才算有效。同一个带用在：测向有效（um982）、
# 卫星可信（gnss_fusion.TRUSTED_BASELINE_M）、控制航向直接对齐双天线（OUTPUT_SNAP_BASELINE_M）、
# 直线卫星纠偏（ImuFirstConfig.fit_min/max_baseline_m）。09-29 定 0.30–0.50，11-21 收紧到 0.35–0.45
HEADING_BASELINE_M = (0.35, 0.45)
# 基线连续这么多帧出界才算出界（同一帧重复传只数一次）：卫星纠偏停用、学开头换算冻结都按它（10-06 傍晚定 3；11-21 用于卫星纠偏）
BASELINE_BAD_FRAMES = 3
# 学开头换算（IMU 航向 ↔ 双天线测向的差）只用这个窄带里的帧，出界后 START_LEARN_HOLD_S 内也不学。
# 10-06「基线变成 0.4 以下还在用卫星校正导致画出蛇形」定 0.40–0.42；11-22「学开头换算还是按 0.40–0.44」；
# 11-23 用户「0.40–0.42 需要放宽到 0.4 到 0.45」（11-22 傍晚基线读 0.416–0.435，93% 的帧 > 0.42）
START_LEARN_BASELINE_M = (0.40, 0.45)
START_LEARN_HOLD_S = 1.0
# 卫星可信 = 主天线固定 + 副天线固定 + 测向固定且基线在带内，三样都在这么久内来过；可信时不报任何定位错误（10-05）
RTK_TRUSTED_FRESH_S = 1.0
# 卫星数（10-06 定 20，10-08 改 17）：少于 SATS_MIN 不用；SATS_MIN–(SATS_FULL−1) 颗要同时满足
# 接收机 σ ≤ LOW_SATS_MAX_SIGMA_M（10-08 2.0 → 2.7 cm；11-22 用户「17–19 颗也收到 2.5」，和 ≥ SATS_FULL 的 FIT_MAX_SIGMA_M 一样；
# 11-26 用户「定位误差 2.5 cm 放宽到 3cm」，两个一起 3.0）、两天线距离误差 < LOW_SATS_MAX_PAIR_M、固定解；≥ SATS_FULL 照常
SATS_MIN = 17
SATS_FULL = 20
LOW_SATS_MAX_SIGMA_M = 0.030
LOW_SATS_MAX_PAIR_M = 0.03
# 卫星位置可靠（直线拟合的四道关，10-06；10-07 两天线距离 3 → 5 cm）：接收机 σ ≤ 3.0 cm；两天线距离误差 ≤ 5 cm，
# 超了以后 0.5 s 再用；连续 2 帧不合格停用，连续合格 2 s 再用
# σ 2.5 → 3.0 cm（11-26 用户「定位误差 2.5 cm 放宽到 3cm」）：11-26 楼下那条 22 m 线主天线 σ 2.05–2.75 cm（中位 2.30），
# 2.5 时前 11.5 m 一拍没用上卫星，车头差 0.87° 按 IMU 走到偏 16 cm 才往回拉；3.0 时整条都能用
FIT_MAX_SIGMA_M = 0.030
# 圆上两条「卫星好时跟得更快」的快修（整圆好天慢修 arc_good_*、作业圆弧双天线径向快修 arc_rtk_*）原来借用 FIT_MAX_SIGMA_M；
# 11-26 用户只说放宽定位误差门槛，这两条快修照旧按 2.5 cm（楼下 2.5–3 cm 不算「卫星好」）
ARC_FAST_MAX_SIGMA_M = 0.025
FIT_MAX_PAIR_M = 0.05
FIT_PAIR_SETTLE_S = 0.5
# 精度够就一定用（11-23 用户「如果精度 ≤2cm，应该都被采用」）：主天线固定解、接收机 σ = hypot(σn, σe) ≤ 这个值、
# 卫星数 ≥ SATS_MIN、两天线距离误差 ≤ FIT_MAX_PAIR_M（没对齐不看）时，直线和圆弧的卫星纠偏一定采用——测向基线带、
# 起步放行、积分冻结、不合格后的等待都不挡。照旧挡的：跳点门（和 IMU 推算突然对不上，多路径）、定位本身不可用（推算维持等）
RTK_ACCEPT_SIGMA_M = 0.02
FIT_QUALITY_FRAMES = 2
FIT_SETTLE_S = 2.0
# 两根天线差 ≥ 3 cm 算对不上：直线用和 IMU 推算对得上（差 ≤ 1.5 cm、比另一根近 1 cm 以上）的那根（10-08），
# 融合也切到同一根（11-21，FusionConfig.secondary_*）
ANTENNA_PICK_PAIR_M = 0.03
ANTENNA_AGREE_M = 0.015
ANTENNA_MARGIN_M = 0.01
# 单天线：PDOP 小于它才可信（10-07，先定 2，同日改 3）
INS_MAX_PDOP = 3.0
# 双天线测向安装偏差（11-22 实测 +1.7°；车上文件写了别的非 0 值照用）
GEN5_MOUNT_OFFSET_DEG = 1.7
# SCH16T 航向角刻度（11-23 实测真转一整圈多读约 0.46°，多读 0.127%）标定在 IMU 固件参数里，车上程序不再乘：
# 用户「标定在imu的固件里面」→ 模块 0x2012 陀螺 Z 刻度 = −1270 ppm（发 0x401: 40 12 20 00 0A FB 00 00，存模块 flash）
GEN5_IMU_YAW_SCALE_PPM = -1270   # 只作记录 / 核对用（读回 0x401: 40 12 20 01 → 0x402 应答 d4..d7 = 0A FB FF FF）
# 天线位置按图纸（11-19），车体系 (前, 左) 米：主天线；单天线模式用右天线
GEN5_ANTENNA_XY = (0.235, 0.205)
SINGLE_ANTENNA_XY = (0.235, -0.205)
# 卫星可信时控制用的车头直接对齐双天线：车停着、不在喷、或差超过这么多（10-10）
OUTPUT_SNAP_DEG = 2.0
# 测向变浮点但主、副天线都固定：航向不确定度放宽到这么多照样走（09-27「测向变浮点不能停，靠 IMU 维持」）。
# 融合（gnss_fusion.HEADING_HOLD_SIGMA_DEG）和任务层（line_mission.GYRO_HOLD_SIGMA_YAW_DEG）用同一个值
HEADING_HOLD_SIGMA_DEG = 2.0

# ───────────── 二、卫星断了、传感器不好时 ─────────────

# 固定解断了不降速、照常喷，靠 IMU + 轮速推算；满 8 s 停喷，再走 1 s 停车，恢复后自动继续（10-06 / 10-07）
RTK_LOST_PAINT_STOP_S = 8.0
RTK_LOST_STOP_DELAY_S = 1.0
# 固定解更新间隔到这么久才进中断处理（用户定 500 ms）
RTK_HOLD_WATCH_S = 0.5
# 轮速这么久没更新才算过期（10-05）
WHEEL_TIMEOUT_S = 1.0
# 还没开喷就偏离路线不中止（10-10）：停稳 0.5 s，控制航向和融合差 ≤ 1°、位置差 ≤ 2 cm，再从最近处重新接入；
# 同一任务最多 3 次
REENTRY_MAX = 3
REENTRY_ALIGN_DEG = 1.0
REENTRY_ALIGN_M = 0.02
REENTRY_STILL_S = 0.5
# 第一个线头没过起喷门槛：线头后这么远以内才停下退回去重新助跑，次数算进重新接入（10-11）
REDO_RUNUP_PAST_M = 0.30

# ───────────── 三、直线：IMU 为主，卫星慢慢修 ─────────────

# 开头学不到换算就借上一条线的：10 分钟以内、和此刻差 ≤ 10°（10-06）
BORROW_MAX_AGE_S = 600.0
BORROW_MAX_GAP_DEG = 10.0
# 拟合位置修正每 0.8 m 最多 1.5 cm（10-08）
FIT_RATE_M = 0.015 / 0.8
# 往回拉（11-21，「线头不能变差」）：线头后 HEAD_M 内每米横偏给 HEAD_K_E 的车头偏、最多 HEAD_MAX_BIAS_DEG；
# 再 HEAD_BLEND_M 内过渡到 K_E、最多 MAX_BIAS_DEG
HEAD_K_E = 0.8
HEAD_MAX_BIAS_DEG = 1.0
HEAD_M = 3.0
HEAD_BLEND_M = 2.0
K_E = 0.8
MAX_BIAS_DEG = 0.6
# 死区：喷线时不要（11-21 去死区）；线头段和助跑 2 mm
PAINT_DEADBAND_M = 0.0
DEADBAND_M = 0.002
# 一直偏在一边：可信卫星在同一边超过 1 cm、持续走 1 m 就当真偏（10-07）；位置跟上去、往回拉加强（10-11），
# 三处（persist_*、position_follow_*、boost_*）用同一对门槛
PERSIST_E_M = 0.01
PERSIST_M = 1.0
# 加强往回拉时车头最多偏这么多（10-11）
BOOST_MAX_BIAS_DEG = 0.6
# 侧坡侧滑：按 横滚 × CRAB_PRIOR 补（10-11）
CRAB_PRIOR = -0.16
# 草地模式：两轮电流（最近 2 m 中位数）≥ 5 A 进、≤ 4 A 退，车速 ≥ 0.4 m/s 才判（11-21）
GRASS_ON_A = 5.0
GRASS_OFF_A = 4.0
GRASS_MIN_SPEED_MPS = 0.4
# 楼下模式（11-24 用户「写补丁先仿真」）：楼下（遮挡、楼面反射）卫星位置会连着几米偏 1–1.5 cm，车跟着它来回拉就画弯。
# 定位精度 σ（2 s 低通）> POOR_SKY_SIGMA_M 且不是草地：这条直线开头 6 m 照常按卫星对角度，之后靠 IMU 和对好的角度走——
# 线头段之后不往回拉，拟合的角度每米最多改 POOR_SKY_BETA_RATE_DEG。卫星拟合出的角度和正在用的差超过 POOR_SKY_EXIT_DEG
# （且超过拟合误差 3 倍），或卫星说喷头同一边偏 > POOR_SKY_EXIT_E_M 走满 1 m，这条线退回平常做法（车真偏了）。
# 11-24 楼下 6 条 × 4 回放（带过来的角度误差 ±0.08°）：鼓包 3.28 → 1.72 cm（最差 5.4 → 2.7），最大偏差 2.43 → 2.50，线头不变；
# 代价：带过来的角度错 0.2–1.8°（今天没修记忆时）最大偏差 2.46 → 3.37；看不见的侧滑 0.1° 时 2.66 → 3.36。
# 出口 4 cm 时楼下卫星自己偏（两天线差 2–4 cm）就会误退，退了照样拉弯，所以 6 cm
# POOR_SKY_BETA_RATE_DEG 0.02 → 0.08（11-24 用户「修补丁楼下弱点」）：0.02 时开头 6 m 从多路径学进去的 0.1–0.2° 错角、
# 带过来的角度错 0.3–0.6° 要走 10–30 m 才改过来，又不往回拉，线一直斜着走（073918 偏到 3.9 cm）。闭环回放 11-24 楼下
# 6 条 × 4（cm，全线最大平均 / 鼓包平均；线头都不变）：
#   记忆 ±0.08°        现在 2.43 / 3.28，0.02 2.65 / 1.81，0.05 2.38 / 2.26，0.08 2.31 / 2.32，0.10 2.32 / 2.34
#   当时实际带的记忆    现在 2.46，0.02 3.40，0.05 2.75，0.08 2.65，0.10 2.65（这批记忆是删掉学习前的，错 0.2–1.8°）
#   看不见的侧滑 0.1°   现在 2.66，0.02 3.31，0.05 2.94，0.08 2.79，0.10 2.77
# 试过没用的：线头后按 3 m 平均的卫星横偏超 1–1.5 cm 才往回拉（最大 2.75–2.92，楼下卫星连着偏 3–4 cm 时反倒拉错）；
# 楼下拟合把卫星当更不准（助跑就在楼下时线头变差 0.3 → 3.2 cm）；0.05 再加往回拉 ×0.15 / ×0.25（记忆 ±0.08° 时最大、鼓包都变大）
POOR_SKY_SIGMA_M = 0.015
POOR_SKY_BETA_RATE_DEG = 0.08
POOR_SKY_EXIT_DEG = 0.6
POOR_SKY_EXIT_E_M = 0.06
# 楼下模式线头段之后往回拉留几成（0 = 不拉）。11-24 楼下 6 条 × 4 回放（cm，最大偏差 / 鼓包）：
#   现在 2.43 / 3.28；×0 2.65 / 1.81；×0.25 2.48 / 2.61；×0.5 2.45 / 2.94（线头都不变）
#   带过来的角度错（今天没修记忆时）最大偏差：现在 2.46，×0 3.40，×0.25 2.55，×0.5 2.39；看不见的侧滑 0.1°：2.66 / 3.31 / 2.95 / 2.83
POOR_SKY_PULL_SCALE = 0.0

# ───────────── 四、起步、助跑、线头 ─────────────

# 助跑长度 2 m（11-19）；重启后第一条也一样（11-22 加的第一条 4 m，11-23 用户「不能靠加长助跑解决问题」删掉）
RUN_UP_M = 2.00
# 开机第一条（没有学好的 IMU 换算）先在这条线上空跑学：过线头再往前这么多米不喷，然后原地掉头、直线开回助跑起点、
# 原地转正，再助跑画整条（11-23 用户「允许开机第一段在两个点中间直走学习但不喷」「空跑后，不要倒回来」「必须在内学习」；
# 11-25 用户「空跑不要圆弧了，直线原地掉头去起点」，原来是圆弧绕回、不停车接着助跑）
FIRST_LINE_LEARN_M = 3.0
# 开机后作业前的行驶（手动、IMU 直行、上一个作业的接入）里顺带学换算（line_cascade.DriveLearner）：往前或倒车开、RTK 固定、
# σ ≤ 2 cm、转得不快时，卫星轨迹方向（减主天线杆臂随转弯的修正）就是车头。最近 DRIVE_LEARN_KEEP_S 内学够
# DRIVE_LEARN_MIN_M、误差 ≤ DRIVE_LEARN_MAX_SIGMA_DEG 就当有记忆，开机第一条不空跑；不够照旧空跑
# （11-25 用户「开机多跑一会学习能不能省去」→「改，先仿真验证」）。11-24 实录：开喷前行驶 ≥ 3 m 时学的和那条线拟合的
# 差 ≤ 0.4°（多数 ≤ 0.2°）；只有 1–2 m 又在楼下时错到 1.6°；开头双天线测的均方根 0.9°。
DRIVE_LEARN_MIN_M = 3.0
DRIVE_LEARN_MAX_SIGMA_DEG = 0.3
DRIVE_LEARN_KEEP_S = 300.0
# 助跑速度：直线 0.3 m/s、起步 0.35、从 0.5 m/s 起开始喷（10-05 / 10-06）；圆前切线助跑 0.35 m/s（10-08）
RUNUP_SPEED_MPS = 0.3
RUNUP_START_MPS = 0.35
ENTRY_STABILIZE_MPS = 0.5
CIRCLE_RUNUP_MPS = 0.35
# 助跑里推算往卫星靠（每 0.3 m 靠拢 63%）；往回拉每米 2.0、车头最多偏 2°、目标每秒最多改 3°（10-08）
RUNUP_RTK_TAU_M = 0.3
RUNUP_K_E = 2.0
RUNUP_MAX_BIAS_DEG = 2.0
RUNUP_RATE_DEG_S = 3.0
# 起喷门槛：喷头离线 ≤ 5 cm 才开喷，没到车照走先不喷；开喷后中途偏差只报警不停喷（10-10；10-08「起点必须喷」）
PAINT_START_MAX_OFFSET_M = 0.05
# 起步锁车头 0.6 m（倒车接入后 1.0 m）；交给直线控制要横偏 ≤ 2 cm、车头差 ≤ 3°（10-07）
START_HOLD_M = 0.6
REVERSE_HOLD_M = 1.0
RUNUP_RELEASE_E_M = 0.02
RUNUP_RELEASE_HEADING_DEG = 3.0
# 起步稳轮：0.3–1.0 m 内车头转速最多 2°/s，偏差变大放到 4°/s；车头偏 ≥ 5° 或喷头横偏 ≥ 5 cm 才放到 8°/s
# （11-22「只在大偏差时放宽」）
LAUNCH_SETTLE_MIN_M = 0.3
LAUNCH_SETTLE_MAX_M = 1.0
LAUNCH_SETTLE_DEG_S = 2.0
LAUNCH_RECOVER_DEG_S = 4.0
LAUNCH_BIG_HEADING_DEG = 5.0
LAUNCH_BIG_E_M = 0.05
LAUNCH_BIG_DEG_S = 8.0
# 进线方式可选原地转或圆弧；圆弧要后方空地 ≥ 2.5 m，半径按 3 / 2.5 / 2 / 1.5 m 依次试（11-19）
ENTRY_TURNS = ('pivot', 'arc')
ENTRY_ARC_MIN_SPACE_M = 2.5
ENTRY_ARC_RADII = (3.0, 2.5, 2.0, 1.5)
# 原地转：车头差超过这么多才转（09-30）
PIVOT_MIN_DEG = 5.0
# 离正式路线 3 m 以内不用转场速度，降到画线速度准备接入（10-07）
APPROACH_ZONE_M = 3.0

# ───────────── 五、圆、弧 ─────────────

# 画圆起喷：先走 1 m 进圆过渡曲线（10-06），再沿圆走 2.5 m 才开始喷（10-09）
CIRCLE_TRANSITION_M = 1.00
CIRCLE_RUN_IN_M = 2.50
# 弧线（不是整圆）在终点前 5 cm 停喷（10-08）
ARC_END_EARLY_M = 0.05
# 圆弧掉头：喷头过拐角 0.2 m 后开始绕（10-02；到拐角前对正的距离按 route_plan.ARC_RUN_IN_M 算）
ARC_RUN_OUT_M = 0.2

# ───────────── 六、速度 ─────────────

# 画线速度默认 0.8 m/s（line_mission.LINE_CRUISE_MPS、motion_speeds.DEFAULT_PAINT_SPEED 同一个值），可设 0.1–1.0；
# 作业中改车速按 S 形曲线，最大 0.15 m/s²（11-22）
PAINT_SPEED_MPS = 0.8
CRUISE_RANGE = (0.1, 1.0)
LIVE_CRUISE_ACCEL_MPS2 = .15
# 转场 1.5 m/s；转场弯 0.45 m/s（11-22 由 0.35 提）；画圆、画弧 0.5 m/s；差速车倒车 0.5 m/s
TRANSIT_SPEED_MPS = 1.5
TURN_SPEED_MPS = .45
PAINT_ARC_SPEED_MPS = .5
DIFF_REVERSE_MPS = 0.5
# 轮子最低速度最好不低于 0.3 m/s（10-08）
WHEEL_MIN_MPS = .3
# 手动页「IMU 直行」：车速 0.8 m/s（09-30 由 0.4 提）、起步斜坡 0.25 m/s²（11-21 由 0.5 降）、偏差超过 5° 停
STRAIGHT_HOLD_SPEED_MPS = 0.8
STRAIGHT_HOLD_ACCEL_MPS2 = 0.25
STRAIGHT_HOLD_ABORT_DEG = 5.0
