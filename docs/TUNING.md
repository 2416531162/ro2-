# 实车标定手册

本次重写把「跟多快、什么时候减速、方向盘打多少」这三件事的计算全部收进
`radar_system/motion_safety.py`。代码里所有默认值都按**保守**取,目的是先保证
不撞人;真实性能要靠下面两个参数标定出来。

标有 ★ 的两个参数没量过之前,不要把 `max_speed_mps` 往上调。

---

## 0. 先跑无硬件测试

```bash
cd <仓库根目录>
python3 tests/test_motion_safety.py      # 33 条,应当全绿
python3 radar_system/motion_safety.py    # 打印刹车包络与旧档位换算,自检用
```

---

## 1. ★ 标定 `decel_capability_mps2`(实测减速度)

阿克曼车没有主动刹车,PWM 归零后靠滚动阻力和齿轮箱反拖减速。这个值直接决定
刹车包络的陡峭程度,**估高了就会撞**。

在实际跑的地面上(地砖、水泥、地毯阻力差很多)量:

```bash
# 1. 关掉跟随,只起驱动
python3 wheeltec_protocol/wheeltec_driver.py --ros-args --params-file wheeltec.yaml

# 2. 另开一个终端,给 1 秒定速,记录从松油门到停住的距离
python3 wheeltec_protocol/control.py drive --speed 0.5 --seconds 1.0
```

在地面贴胶带,量出**松开指令那一刻的车头位置**到**完全静止**的距离 `d`:

```
a = v² / (2 × d)
```

例:0.5 m/s 滑行 0.20 m → a = 0.25 / 0.40 = **0.63 m/s²**

多做 3 次取**最小值**(最保守的那次),填进去:

```bash
python3 radar_system/person_follower.py --decel-mps2 0.63
```

> 默认值 1.00 对多数硬地板偏乐观。如果量出来低于 1.0,**一定要改**。

---

## 2. ★ 标定 `control_latency_s`(感知到执行的总死时间)

这是从「相机那一帧曝光」到「轮子真的开始减速」的全链路延迟:

| 环节 | 典型值 |
|---|---|
| 相机曝光 + 传输 | 30~60 ms |
| YOLOv8n-pose 推理 (RK3588 NPU) | 待设备实测 |
| alpha-beta 滤波剩余滞后 | 20~50 ms |
| 控制周期 (20 Hz) | 50 ms |
| 串口 + 固件响应 | 20~40 ms |

**测法**:开 `--dry-run`,人拿一块板子在相机前突然遮挡,用手机慢动作录屏,
对比「板子遮住镜头」与「终端 state 变成 SEARCHING_LOST」的帧差。

```bash
python3 radar_system/person_follower.py --dry-run
```

默认 0.35 s。测出来更大就往上填,**宁大勿小**。

---

## 3. 首次实车:用 safe-mode

```bash
python3 radar_system/person_follower.py --safe-mode --decel-mps2 <你测的值>
```

`--safe-mode` 会强制:极速 ≤ 0.30 m/s、保持距离 ≥ 1.20 m、包络归零点 ≥ 0.90 m、
减速度按 0.70 m/s² 保守估计。先确认逻辑对,再逐步放开。

终端仪表盘会实时打印 `上限 X.XX (限制原因)`。**盯住这个数**:

| 限制原因 | 含义 |
|---|---|
| `follow_envelope` | 正常,离目标近了在收油 |
| `obstacle_envelope` | 雷达前方有东西在限速 |
| `aeb_hard` | 已越过硬急停线 —— **正常跟随时不该出现** |
| `scan_stale` | 雷达数据超过 0.5 s 没更新,已降级到 0.15 m/s |
| `lost` / `blink` | 目标丢失或短暂遮挡 |

如果 `aeb_hard` 经常出现,说明 `decel_capability_mps2` 还是估高了,调小它。

---

## 4. 逐步放开

确认 safe-mode 下十次接近都稳稳停在设定距离上之后:

```bash
python3 radar_system/person_follower.py \
  --decel-mps2 0.63 --latency-s 0.40 \
  --follow-distance-m 0.90 --follow-stop-m 0.70 \
  --max-speed-mps 0.45          # 一次加 0.1,不要一步到位
```

每提一档,重复十次「人快步走 → 突然停步」,确认车停在 `follow_stop_m` 附近。

---

## 5. 网页遥控档位

速度档和转角档现在是两个独立维度,在 `radar_system/radar_web_server.py` 顶部:

```python
SPEED_TIERS_MPS  = {'low': 0.30, 'med': 0.55, 'high': 0.85}
STEER_TIERS_DEG  = {'gentle': 8.0, 'normal': 14.0, 'full': 20.0}
```

改完重启 `radar_web_server.py` 即可,前端会自动跟随(前端不再硬编码速度值)。

`STEER_TIERS_DEG` 的上限受 `ChassisGeometry.max_steer_rad`(默认 0.35 rad ≈ 20°)
约束。如果你的车舵机实际能打更大角度,改 `motion_safety.py` 里的
`ChassisGeometry.max_steer_rad`,不要改 `wheeltec.yaml` —— 那个是驱动层的独立限幅。

### 转弯半径速查

满舵 20° 时最小转弯半径 **0.77 m**。也就是说:

- 想在 1 米宽的走廊里掉头,做不到,必须倒车调头
- `gentle` 档 8° 对应半径约 1.85 m,适合长走廊微调
- `normal` 档 14° 对应半径约 1.09 m

---

## 6. 静止预打舵(可选,需实车验证)

固件在 `Vx == 0` 时会强制舵机归中(见 `wheeltec_protocol/PROTOCOL.md` 8.2),
所以车停着的时候没法提前把轮子摆好。`PROTOCOL.md` 8.3 记录了绕过办法:
下发 5 mm/s 的微速度,固件会照常解算转角,但 PWM 太小驱动不了车身。

```bash
python3 radar_system/person_follower.py --pre-steer
```

**默认关闭**,因为电机静摩擦门限因车而异。开启前先手动确认车真的不动:

```bash
# 期望:舵机向左打满,车身纹丝不动,遥测速度恒为 0
python3 wheeltec_protocol/control.py drive --speed 0.005 --steering-deg 20 --seconds 2
```

如果车会爬行,把 `pre_steer_creep_mps` 调小,或者干脆不用这个功能。

---

## 附:参数速查表

| 参数 | 默认 | 含义 | 调大的后果 | 调小的后果 |
|---|---|---|---|---|
| `decel_capability_mps2` ★ | 1.00 | 实测减速度 | 刹车晚,**可能撞** | 刹车早,跟得远 |
| `control_latency_s` ★ | 0.35 | 全链路死时间 | 更保守 | **可能撞** |
| `follow_distance_m` | 0.90 | 期望保持距离 | 跟得远 | 跟得紧 |
| `follow_stop_m` | 0.70 | 包络归零点 | 停得远 | 停得近 |
| `max_speed_mps` | 0.55 | 极速上限 | 跟得快 | 跟不上快走的人 |
| `obstacle_stop_m` | 0.50 | 雷达包络归零 | 早避障 | 晚避障 |
| `aeb_hard_stop_m` | 0.40 | 硬急停线 | 更早急停 | 更晚急停 |
| `kd_feedforward` | 0.90 | 目标速度前馈 | 跟速更贴 | 退化为纯追距离 |
| `kp_distance` | 0.60 | 距离误差增益 | 响应快、易过冲 | 迟钝 |
| `kp_steer` | 1.10 | 视线角→转角增益 | 转向积极、易画龙 | 转不过弯 |

---

## 7. 感知层参数(第二轮加固新增)

控制层再稳,感知层给出错误距离时一样会撞。这一组参数决定「什么样的观测才配拿来开车」。

### 深度可信度 — `radar_system/person_pose_node.py / depth_measurement.py`

```python
DEPTH_MIN_MM = 150.0            # 下界
DEPTH_MAX_MM = 6000.0           # 上界,Astra S 超过这个距离返回的是垃圾值
DEPTH_INSET = 0.20              # bbox 四边各内缩 20%,避开边缘穿透到背景
DEPTH_PERCENTILE = 20.0         # 取第 20 百分位而非中位数,宁近勿远
DEPTH_MIN_VALID_RATIO = 0.30    # 有效像素占比门限
DEPTH_MIN_PIXELS = 60           # 绝对像素数下限
DEPTH_MAX_PAIR_AGE_S = 0.15     # RGB 与深度帧的最大允许时间差
```

**怎么判断要不要调**:跑 `--dry-run`,看终端的 `野值NNN` 计数。

- 计数长期为 0,但车经常判错距离 → `DEPTH_MIN_VALID_RATIO` 调高到 0.4
- 计数飙升、目标频繁丢失 → 你的场景深度图本来就稀疏(逆光/大玻璃),
  把 `DEPTH_MIN_VALID_RATIO` 降到 0.20,但同时把 `max_speed_mps` 也压低

> `DEPTH_PERCENTILE` 不建议超过 50。取中位数在「一半前景一半背景」的
> 分布上会被拉向远处,把人判得比实际远 —— 这正是要避免的方向。

### 统一多人跟踪 — `radar_system/person_tracker.py`

相机检测和雷达腿部点簇是同一个跟踪器的两路观测(参考 SPENCER / sobits_follower):

- 每个人一条轨迹,匀速模型卡尔曼滤波,状态在里程计系(车动不影响人的速度估计)
- 观测按**采集时刻**换算:相机用检测消息里的图像时间戳,雷达用扫描时间戳,
  车身位姿取那一刻的,补偿推理延迟
- 马氏距离门控(99%)+ 匈牙利算法全局关联
- ByteTrack 两级关联:高分框可新建轨迹,低分框只延续已确认的轨迹
- 雷达点簇只更新已确认的轨迹,不会凭空造出一个人
- 跟随锁定的是**轨迹编号**,相机看不到时雷达照常更新同一条轨迹

```python
track_high_conf       = 0.45  # 高分框门限(可新建轨迹)
track_low_conf        = 0.15  # 低分框门限(只延续);ai_3d_detector.PERSON_LOW_CONFIDENCE 同步
confirm_frames        = 3     # 相机命中几次才确认为人
target_timeout_s      = 0.30  # 目标多久没有任何观测算丢失
max_camera_latency_s  = 0.60  # 相机时间戳比现在早这么多以上,按 0 延迟处理并计数
camera_hfov_deg       = 58    # 反向证据用的相机视场
PersonTracker(gate_max_m=1.2, accel_sigma=1.5, confirmed_timeout_s=1.5,
              lidar_ambiguity_m=0.8, unseen_in_view_max_s=1.5)
```

- **反向证据**:轨迹明明在相机视野里(留 6° 余量、0.9~3.5m),却 1.5 秒没被相机看到
  → 不是人(雷达把柱子当成人了),删除。面板「排除的非人轨迹」计数。
- 旁边有人经过时目标编号不应变化;面板「目标编号」的切换次数持续增长说明关联门限太宽
  → 调小 `gate_max_m`
- 「相机时间戳异常」计数增长 → 相机节点和跟随节点的时钟不一致(检查 `use_sim_time`、NTP)
- 人快走时频繁跟丢 → 调大 `accel_sigma`(允许更大的机动)

### 相机 / 雷达交叉校验

```python
los_half_width_m = 0.25   # 只看「车 -> 人」视线两侧这么宽的窄带
range_conflict_m = 1.00   # 视线上雷达比相机近这么多,计入「冲突」遥测
```

规则:视线窄带里雷达更近就采信雷达(可能是人本身,也可能是挡在中间的东西,
两种情况都不该往前冲);**不再整帧丢弃**,转向照常跟人。
旧版取目标方位 ±10° 扇形里最近的任何东西,2m 外扇形近 1m 宽,
旁边的椅子/门框会被当成人,表现为「屏幕上有人,车却不动」。

**冲突计数持续增长**,按顺序排查:

1. 相机与雷达的**外参**对不上(两者不在同一个原点/朝向),先量安装位置
2. 雷达扫到了车身自己的结构件 → 检查 `scan_blind_sectors_deg` / `self_hit_skin_m`
3. 人和车之间确实常有遮挡物

### 「前方无路」且挡路是「雷达看不到的区域」

网页「正在执行」会写出挡路方向和附近光束的原因:

| 原因 | 含义 | 处理 |
|---|---|---|
| 太近/车身遮挡 (`near`) | 有回波但近于 0.15m,几乎总是车上的结构件 | ≤4° 的窄缝自动用两侧回波桥接 |
| 打在车身上 (`self`) | 回波落在车体轮廓 + `self_hit_skin_m` 内 | 同上 |
| 无回波 (`none`) | 完全没回波:可能是车身结构,也可能是吸光物体 | **不会自动放行**,需确认 |
| 屏蔽扇区 (`masked`) | 落在 `scan_blind_sectors_deg` 内 | ≤4° 桥接,更宽的保持未知 |

车停在空旷处运行 `python3 radar_system/scan_doctor.py --seconds 10`,输出开头会列出
「车头 ±90° 内持续没有有效回波的方向」。确认是车身结构后,把它加进
`scan_blind_sectors_deg`。宽度超过 4° 的前方遮挡无法安全桥接,
只能挪开遮挡物或调整雷达安装高度。

### 卡住后倒车(沿来路)

雷达装在车头且被车上设备挡住,车尾约 75° 看不见(`scan_blind_sectors_deg`)。
看不见的地方不能随便倒,所以只允许**沿车刚开过的路**倒:车身扫过的每一点
都必须落在车身不久前实际占据过的位置上。

```python
RecoveryConfig.blind_reverse_m  = 0.30  # 每次脱困沿来路最多倒多少
RecoveryConfig.trail_m          = 1.5   # 记住最近多少米来路
RecoveryConfig.trail_max_age_s  = 20.0  # 来路超过这么久就不再相信(可能有人走进去)
```

- 刚启动、还没开过就卡住 → 不会倒车(没有来路可循)
- 底盘里程反馈中断 → 来路作废
- 前进时车轮顶住雷达看不到的矮物体(推车底板等)被判为「堵转」,
  脱困会记住当时的舵角,倒车后往另一侧打舵离开,前进满 30cm 才交回跟随
- 面板「运行详情 → 自动脱困」显示来路长度、剩余倒车额度、卡住时舵角

**根本解决办法是让雷达看得见车尾**:把雷达抬高到车上所有设备之上,
或在车尾加一个测距传感器。

### 雷达接力(相机看不到时)

```python
lidar_handoff          = True  # False = 雷达不更新人的轨迹
lidar_handoff_max_s    = 8.0   # 身份「不确定」状态最多维持多久
lidar_track_speed_cap  = 0.35  # 只靠雷达时限速
```

人走出相机视野(Astra S 水平约 58°)后,同一条轨迹由雷达腿部点簇继续更新。
雷达点簇周围 0.8m 内没有别的候选时,这次更新算「身份可信」,计时清零;
旁边一直有椅子腿之类的干扰时,超过 `lidar_handoff_max_s` 就放弃。

- 接力时跟到墙边/柱子上 → 调小 `lidar_handoff_max_s`,或调大 `lidar_ambiguity_m`(更容易判为有干扰)
- 人出画后车很快就停 → 看面板「雷达轨迹」:显示「未认领到腿」说明雷达没把腿关联到这个人,
  多半是相机/雷达外参不准(用 `calib_check.py` 检查)或屏蔽扇区挡住了人所在方向

### 沿人走过的路跟随(纯追踪)

```python
follow_breadcrumbs   = True
pp_lookahead_min_m   = 1.20  # 预瞄距离(后轴起算)
pp_lookahead_gain_s  = 0.60  # 每 1 m/s 车速增加
pp_lookahead_max_m   = 1.80
```

目标轨迹每走 10cm 记一个路径点,车朝「路径上第一个超过预瞄距离的点」打舵
(转角按固件公式 TurnR = L/tan(δ) + 轮距/2 反算),「还差多远」按沿路径剩下的长度算。
人绕过门框、柜子拐弯时车不会斜着切过去。L 型路线仿真(满舵半径 1.77m):

| 预瞄 | 拐角前内切 | 冲出拐角 |
|---|---|---|
| 0.6 m | 0.00 m | 1.01 m |
| 1.2 m(默认) | 0.04 m | 0.40 m |
| 1.5 m | 0.11 m | 0.11 m |

- 过窄门时蹭内侧 → 调小 `pp_lookahead_min_m`
- 拐弯冲得太出去 → 调大(外侧墙会被避障检查挡住)
- 想恢复旧的「直接朝人打舵」→ `follow_breadcrumbs = False`

### 底盘驱动内的独立防撞层 — `wheeltec_protocol/scan_guard.py`

跟随程序、网页遥控、以后的导航都经过底盘驱动,驱动里按雷达实测再兜一层底
(思路同 nav2_collision_monitor)。规则比跟随程序宽松,正常跟随不会触发;
跟随程序崩溃/有 bug,或网页手动遥控时才起作用。

- 只看行驶方向、车宽 +5cm(转弯再 +10cm)走廊内的点
- 允许车速满足:0.2s 延迟距离 + 刹车距离(1.0 m/s²)+ 4cm 余量 ≤ 到障碍距离;
  限速立即生效,不走加减速斜坡;不会解除底盘使能
- 从没收到雷达 → 直通(雷达没开时仍可手动遥控);收到过但断流 → 限速 0.15 m/s
- 车身自反射余量 2cm(比跟随程序小,否则车头 5cm 内的障碍会被忽略)

驱动参数都以 `guard_` 开头,例如关掉:`ros2 run ... --ros-args -p guard_enabled:=false`。
`/wheeltec/status` 里的 `guard` 字段、网页「运行详情 → 底盘防撞层」和手动遥控提示
会显示「防撞减速 / 防撞停车」。

### 录包与离线回放 — `record_follow_bag.sh` / `bag_replay.py`

```bash
bash radar_system/record_follow_bag.sh                     # 现场录制,Ctrl+C 结束
python3 radar_system/bag_replay.py ~/bags/follow_XXXX      # 回放,打印状态占比与切换
python3 radar_system/bag_replay.py BAG --set follow_breadcrumbs=False   # A/B 对比参数
python3 radar_system/bag_replay.py BAG --out timeline.jsonl             # 逐周期状态
```

回放用包里的时间做时钟,相机/雷达时间戳与现场一致;节点以演练模式运行,不发指令。
注意这是**开环**回放:底盘速度是现场录下的,改了参数后车的实际轨迹不会跟着变,
适合查「为什么判丢、为什么判无路」,不适合评估路径。

### 相机 / 雷达外参检查 — `calib_check.py`

```bash
python3 radar_system/calib_check.py --seconds 40
```

一个人依次站到车前 左/中/右 × 近/远,每处 3 秒。工具用二维刚体最小二乘对齐
「相机算出的人」与「雷达上的腿」,给出 `camera_offset_x_m / camera_offset_y_m /
camera_yaw_rad` 建议值。修正前误差 < 8cm 且角度 < 1° 时不需要改。
雷达安装位置 `lidar_offset_x_m` 是基准,先用卷尺量准。

---

## 附二:终端仪表盘字段速查

```
[RUN] [ 跟踪追随 ] 锁定 person X:+0.05 Z:1.20m v:-0.30m/s | 雷达  1.15m | 上限 0.55 (follow_envelope) | vx=+0.42 舵= +2.1° | 野值  3 冲突  0 | 24.1V
```

| 字段 | 含义 | 该盯什么 |
|---|---|---|
| `锁定` / `未锁` | 目标锁定状态 | 跟随中反复变「未锁」= 关联参数太严 |
| `v:` | 目标对地速度估计 | 人站着不动时应接近 0 |
| `上限` | 刹车包络给出的速度上限 | 这是安全性的核心指标 |
| `(原因)` | 谁在限速 | `aeb_hard` 不该出现;`range_conflict` 要查标定 |
| `野值` | 被门控拒绝的观测数 | 缓慢增长正常,飙升要查深度图 |
| `冲突` | 相机雷达矛盾次数 | 持续增长 = 外参或安装有问题 |

---

## 8. 建图开关与网页遥控延迟

### 症状与病根

**「跟随时车速很快,但网页点前进后退很慢」**——这两条路走的是同一套驱动、
同一个 `/cmd_vel`,所以问题不在底盘,在 `radar_web_server.py` 进程内部。

`map_cb` 是遍历整张占据栅格的纯 Python 双重循环。它**持有 GIL**,循环期间
整个进程的其他线程全部冻住,包括处理网页遥控 POST 的 HTTP 线程。
跟随节点是 `subprocess` 起的独立进程,有自己的 GIL,完全不受影响——
这正是两者表现差这么多的原因。

实测(x86,RK3588 上还要再慢 3~5 倍):

| 地图尺寸 | 改造前 | 改造后 | 提速 |
|---|---|---|---|
| 20m×20m | 6.0 ms / 32,060 点 | 2.3 ms / 14,370 点 | 2.6x |
| 40m×40m | 34.7 ms / 128,053 点 | 2.7 ms / 14,415 点 | **13x** |
| 60m×60m | 179.9 ms / 288,101 点 | 3.0 ms / 18,130 点 | **60x** |

### 三项修改

1. **默认不订阅地图**。只要雷达实时显示就够了,`map_cb` 根本不跑。
2. **限流 + 自适应抽样**。开启建图时,每秒最多算一次,并按地图尺寸自动放大
   抽样步长把输出点数钳在 `MAP_MAX_POINTS` 内。
3. **多线程执行器**。`manual_loop`(20Hz 补发 `/cmd_vel` 的定时器)放进独立
   回调组,任何慢回调都饿不死它。

### 怎么用

```bash
# 默认:仅雷达实时显示,网页遥控最跟手
python3 radar_system/radar_web_server.py

# 需要建图时
ENABLE_MAP=1 python3 radar_system/radar_web_server.py
```

启动日志会打印当前状态:

```
🚀 SLAM Web 服务已就绪: http://192.168.0.170:8088
   建图订阅: 关闭 (仅雷达实时显示)   numpy 加速: 可用
   需要建图时用: ENABLE_MAP=1 python3 radar_web_server.py
```

### 相关参数

```python
MAP_MIN_INTERVAL_S = 1.0     # 地图重算最小间隔
MAP_MAX_POINTS = 12000       # 输出点数上限,超了自动加大抽样步长
```

> ⚠️ 如果开了建图之后网页遥控又变迟钝,先把 `MAP_MAX_POINTS` 降到 6000 试试。
> 这个值同时影响 GIL 占用和网页传输量,前端渲染也会跟着变流畅。

---

## 9. 车体足迹(已按实测值配好)

### 2026-09-16 实测

| 参数 | 值 | 位置 |
|---|---|---|
| 全宽(轮胎外沿) | 0.67 m | `footprint_half_width_m = 0.335` |
| 前长(后轴中心→最前端) | 0.67 m | `footprint_front_m` |
| 后长(后轴中心→最后端) | 0.18 m | `footprint_rear_m` |
| 轴距 | 0.54 m | `wheeltec.yaml` + `ChassisGeometry` |
| 轮距 | 0.59 m | `ChassisGeometry.track_m` |
| 雷达前后位置 | 0.53 m | `lidar_offset_x_m`(基本在前轴线上) |
| 雷达左右位置 | 0 | `lidar_offset_y_m`(在中线上) |
| 侧向余量 | 0.06 m | `footprint_margin_m` |

> ⚠️ **`max_steer_rad` 还是标称的 0.35 rad(20°),尚未实测。**
> 轴距改成 0.54 之后,转弯半径对这个角度非常敏感。把车架起来把舵机打到底量一下。

### 修正前后

代码里原来写的是轴距 0.25 / 轮距 0.17,比实车小 **2.2 倍**和 **3.5 倍**:

| | 修正前 | 实际 |
|---|---|---|
| 满舵最小转弯半径 | 0.77 m | **1.77 m** |

这不只影响避障。`yaw_from_steer()` 用的就是这两个数,所以**网页遥控的转角档与
跟随时的转向一直算错**:代码以为打 14° 对应半径 1.09m,实际 2.46m。

### 这台车的通过性(几何硬约束)

| 转角 | 转弯半径 | 所需净宽 |
|---|---|---|
| 直行 | — | **0.79 m** |
| 5° | 6.47 m | 0.83 m |
| 10° | 3.36 m | 0.86 m |
| 14° | 2.46 m | 0.88 m |
| 20° | 1.78 m | **0.91 m** |

标准室内门净宽 0.80~0.90 m。**直行勉强过得去,满舵过不去。**

实测 0.85m 的门、车距门 0.80m 时:

```
转角      扫掠净空
15.00°     0.26 m   会撞门框
 7.50°     0.20 m   会撞门框
 3.75°     0.17 m   会撞门框
 0.00°     8.00 m   畅通
```

连 3.75° 都过不去 —— 0.85 减 0.67 每侧只剩 9cm,扣掉 6cm 余量只剩 3cm。

### 行为:门口自动收舵

跟随时人往旁边偏一点,车本能地打舵去追,恰好在门框里扫出最宽的轨迹。
`limit_steer_for_clearance()` 会在净空低于 `min_path_clearance_m`(默认 0.35m)
时把转角往中间收,直到路走得通;收到笔直还不行才是真过不去,那时净空为 0,
刹车包络自会把车停住。

终端仪表盘上转角后面出现黄色 `收` 字,就是正在收舵。

### 验证

摆两个纸箱模拟门框,净宽从宽往窄收:

```bash
python3 radar_system/person_follower.py --dry-run
python3 radar_system/footprint.py          # 打印当前尺寸下的扫掠表
```

| 门净宽 | 期望 |
|---|---|
| ≥ 1.0 m | `净空` 保持大值,可以边走边修方向 |
| 0.80~0.90 m | 接近时出现 `收`,车摆正穿过 |
| < 0.79 m | `净空` 归零,限制原因 `swept_path`,车停住 |

如果实际能过的门被判成过不去 → `footprint_margin_m` 调小;
还是刮 → 调大,或检查 `half_width_m` 是不是按底盘板量的而非轮胎。

> 现在的行为是**过不去就停,不会自己绕**。`footprint.widest_passable_steer()`
> 已写好选最空转角的逻辑,但没接进控制律 —— 那属于局部路径规划,
> 等这套尺寸在实车上验证过再上。

---

## 12. 雷达自反射:AEB 一直误触发

### 现象

前方 2 米空无一物,网页却一直报「雷达 AEB 防撞触发:前方障碍物 < 0.40m」。
而且「雷达正前测距」显示 0.55m —— 和 AEB 判据(< 0.40m)自相矛盾。

### 原因

雷达装在车上,周围有**相机横条、天线杆、传感器盒、车架立柱**。这些会被扫成
距离恒定、永不消失的「障碍物」。

两个数对不上是因为**用的扇区不同**:网页显示用 ±15°,AEB 用 ±30°。
偏出 15° 的自反射只触发 AEB,不进显示。

**加了车体足迹检查之后这个问题更严重**:自反射点必然落在车体轮廓之内,
`corridor_clearance()` 遇到轮廓内的点直接返回 0 —— 车会**永久停住**。

### 诊断

把车停在空旷处(周围 2 米没东西),跑:

```bash
python3 radar_system/scan_doctor.py --seconds 5
```

它会按 5° 分箱打印全周剖面,把「距离恒定 + 几乎每帧都有」的扇区标红,
并直接给出可以粘进配置的屏蔽参数。判据是**空旷处仍然稳定报近距离回波
= 只可能是车自己**。

### 两道过滤

```python
self_hit_skin_m = 0.05          # 车体轮廓外扩多少算自反射
scan_blind_sectors_deg = ()     # 例: ((-35, -20), (150, 180))
```

1. **轮廓过滤**(默认开启,不用配):落在车体轮廓 + 5cm 之内的点一律丢弃。
   雷达测距有噪声、安装位置也有测量误差,所以要外扩一点。
2. **角度屏蔽**(按需配):明确知道哪些方位有车体结构时用这个更精准 ——
   不会白白牺牲这些方向的探测距离。

调大 `scan_min_valid_m` 也能解决,但那是**全向**牺牲近距离探测能力,
所有方向都看不见 0.5m 以内的东西。能用角度屏蔽就别用它。

### 验证

终端仪表盘和 `/follower/status` 里有 `self_hits` 计数 —— 每帧丢掉了多少个
自反射点。

| 现象 | 说明 |
|---|---|
| `self_hits` 稳定在某个数(比如 30~50) | 正常,就是车自己那几个结构件 |
| `self_hits` 为 0 但 AEB 仍误触发 | 结构件在轮廓之外,需要配 `scan_blind_sectors_deg` |
| `self_hits` 忽大忽小 | 可能把真障碍物也滤掉了,调小 `self_hit_skin_m` |

> ⚠️ 不要为了让 AEB 不响就无脑调大 `self_hit_skin_m`。它外扩的是**车体轮廓**,
> 调到 0.2m 就意味着车头前 20cm 内的真实障碍物也会被当成自反射丢掉。
> 先用 scan_doctor 看清楚结构件到底在哪,再对症下药。

---

## 13. AEB 一直误报前方障碍物:界面显示的是陈旧数据

### 实车诊断记录(2026-09-16)

`scan_doctor.py` 在空旷处采集:

```
angle_min = +0.0°
```

| 方位 | 最近距离 | 判定 |
|---|---|---|
| 330°~30°(**正前方**) | 1.32 ~ 5.03 m | 干净 |
| 155°~230°(**车后**) | 0.17 ~ 0.31 m | 自反射 |

自反射全在车后 —— 雷达装在车头往后扫到自己的车身,正常。
`angle_min = 0`,所以按序号当角度用的老写法在这台雷达上碰巧是对的。
**前向视野是干净的,AEB 没有任何理由触发。**

### 真正的原因:界面在显示已经死掉的节点的最后一帧

截图里「一键启动」按钮还在 —— **跟随节点根本没在跑**。
人体纵距 0.00、置信度 `--` 也是佐证。

但 AEB 横幅照样亮,「雷达正前测距」照样显示 0.55m。因为这两个值来自
**完全不同的数据源**:

| 界面元素 | 数据源 | 状态 |
|---|---|---|
| 实时雷达图(显示 2 米) | `/scan` | 活的 |
| 正前测距 0.55m | `/follower/status` 的 `aeb_min_scan_m` | **冻住的** |
| AEB 横幅 | `/follower/status` 的 `aeb_active` | **冻住的** |

`state['follower']` 写进去之后永不过期。跟随节点被 Ctrl-C 或崩溃时不会
发一条「我停了」,于是它最后那一刻的状态**永远留在界面上**。

通过网页的「停止」按钮关闭是没问题的 —— `stop_follower()` 会主动把
`state['follower']` 置成 OFFLINE。只有非正常退出才会留下这个幽灵。

### 修法

后端给跟随状态打时间戳,超过 `FOLLOWER_STALE_S`(1.5 秒,节点是 20Hz 发)
就标记 `stale` 并清掉会误导人的字段:

```python
payload['aeb_active'] = False
payload['aeb_min_scan_m'] = None
payload['target'] = None
payload['state'] = 'OFFLINE'
```

前端统一用 `fLive = (f && !f.stale) ? f : null`,陈旧时:

- 正前测距退回实时的 `state.front_dist`
- AEB 横幅不显示
- 速度/角速度显示 0

### 怎么确认节点是不是真的在跑

```bash
ros2 node list | grep person_follower
ros2 topic hz /follower/status        # 应当约 20 Hz
```

没有输出就是没在跑,界面上那些数字全是历史遗迹。

---

## 14. angle_min:另一台雷达上会踩的坑(本机不受影响)

本机 N10P 发布 `angle_min = 0`,所以按序号当角度用碰巧是对的。
但这是**运气**,不是正确性 —— 换一台发布 `angle_min = -π` 的雷达就会
前后颠倒 180°,车尾扫到的自己会被当成正前方的障碍物。

LaserScan 的第 0 个光束指向 `msg.angle_min`,不是 0°。正确写法:

```python
angle = msg.angle_min + i * msg.angle_increment
```

查扇区时:

```python
i = round((目标方位 - angle_min) / 角分辨率)
```

已加固的位置:

- `person_follower.on_scan` —— 用 `angle_min + i * angle_inc`
- `scan_utils.sector_min` —— 新增 `angle_min_deg` 参数
- `radar_web_server.scan_cb` —— 传入 `degrees(msg.angle_min)`

`scan_doctor` 会打印 `angle_min` 并在其非 0 时告警,差 180° 时点名
「前后完全颠倒」。本机跑出来是 `+0.0°`,不会告警。

> ⚠️ **不要采纳 scan_doctor 早期版本建议的 `scan_min_valid_m = 0.22`。**
> 那是全向门限,会让所有方向都看不见 0.22m 以内的东西,包括真正挡在
> 车头前的障碍物。自反射全在侧后方时不需要任何配置 —— `drop_self_hits()`
> 的车体轮廓过滤会自动处理。新版 scan_doctor 已经会这么提示。
