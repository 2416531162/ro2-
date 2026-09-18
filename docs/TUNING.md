# 统一配置与现场标定

物理参数唯一来源是 `robot_core/robot.json`，现场可通过 `RK3588_ROBOT_CONFIG` 指向 `/etc/rk3588/robot.json`。所有服务必须读取同一份配置，修改后整套重启。驱动和行为的 `profile_hash` 不一致时不执行运动请求。

不要再分别修改跟随常量、网页速度、驱动 YAML 中的几何或旧 `lidar_calib.json`。跟随 CLI 的物理标定参数只接受与共享配置相同的值；保持距离、任务限速等行为参数仍可用 CLI 调整。发布目录应保持不变，现场标定写入外部配置。

## 参数归属

| 配置键 | 当前值 | 含义 |
|---|---|---|
| `geometry.wheelbase_m` / `track_m` | 0.54 / 0.59 m | 前后轴距、左右轮距 |
| `geometry.front_m` / `rear_m` | 0.67 / 0.18 m | 后轴中心至最前/最后端 |
| `geometry.half_width_m` | 0.335 m | 轮胎外沿半宽 |
| `geometry.max_steer_rad` | 0.35 rad | 标称舵角限位，需实车确认 |
| `sensors.lidar_x_m` | 0.53 m | 雷达至后轴中心前向距离 |
| `sensors.camera_x_m` / `camera_pitch_rad` | 0.54 m / 0.2618 rad | 相机位置、向下俯角 |
| `sensors.raw_lidar_yaw_deg` | 0° | 标准协议原始扫描零点；现场标定后才改为实测偏移 |
| `sensors.lidar_yaw_rad` | 0 | 已发布扫描的残余安装偏角 |
| `safety.decel_mps2` | 1.0 m/s² | 制动包络使用的减速度，需实测 |
| `safety.control_latency_s` / `guard_latency_s` | 0.35 / 0.20 s | 跟随链路与驱动雷达防撞链路延迟 |
| `manual.low_mps` / `med_mps` / `high_mps` | 0.50 / 0.85 / 1.20 m/s | 网页手动档位，驱动仍最终限幅 |

这些值沿用仓库已有配置，不表示本次在设备上重新测量。机器人转向换算在 `robot_core/kinematics.py`，跟随、网页和驱动共用；按当前对称转向模型，0.35 rad 的最小半径约 1.77 m。实际左右转向和倒车还需验证固件约定。车后快速掉头使用跟随上限 0.45 m/s，但仍受扫掠净空、雷达门控和驱动限幅约束。

## 制动与延迟

在对应地面记录停车前实测速度 `v` 和开始减速至停稳距离 `d`，估算 `a = v²/(2d)`。重复测量，并考虑载荷、电池及地面变化，使用保守值写入 `safety.decel_mps2`。例如 0.5 m/s、减速段 0.20 m，对应约 0.63 m/s²。

从相机曝光或障碍出现，到实际轮速开始下降的时间用于估算总链路延迟。仅观察终端状态变化只能测到算法响应，不能代表执行器开始减速。跟随与驱动防撞链路不同，分别填写两个延迟字段。不要为了缩短跟车距离而乐观提高减速度或缩短延迟。

新控制入口要求雷达、底盘、电池健康。`control.py drive` 是有界手动命令，结束会锁存停车；再次使用前执行 reset，不会自动恢复任务。初次验证保持实际停车条件可控，不能用软件测试替代实车刹停检查。

```bash
python3 -m pytest tests -q
# 避免与已有 follower 服务重复运行
bash radar_system/run_follower.sh --dry-run
bash radar_system/run_follower.sh --safe-mode
```

`--dry-run` 仍消费实际 `/odom`，只计算不发请求；不自动用速度积分掩盖定位缺失。`--safe-mode` 限制跟随速度及距离包络，具体值见 `follower_config.build_config`；它不改变共享驱动物理标定。

## 跟随行为参数

`radar_system/follower_config.py` 保留任务参数。当前常用默认值：期望车头至人 1.00 m、包络归零 0.80 m、巡航上限 0.45 m/s、障碍停车净空 0.15 m、硬急停净空 0.08 m。这些是算法门限，不是经过本次实车验收的性能承诺。

```bash
bash radar_system/run_follower.sh --follow-distance-m 1.2 --max-speed-mps 0.3
bash radar_system/run_follower.sh --no-recovery --no-turnaround
```

目标身份由统一人体轨迹维护；相机与雷达关联默认门限 1.2 m；车后快速穿越时，已确认目标可在 4.0 m 的无歧义扩展门内被雷达重捕，车前到车后的跳变会直接重定位，避免目标估计穿过车体；多候选点簇不会触发扩展重捕。高置信相机观测可新建轨迹，低置信观测和雷达只延续匹配目标。相机观测按采集时间对齐共享本地位姿，超时或没有可用历史时丢弃，不补成“刚刚收到的新观测”。相机暂时看不到时可有限雷达接力；质量不够、时间到期则停车或按已有恢复策略处理。

脱困及 K-turn 使用同一位姿来源，依实测位移/转角计算机动预算。具体路径、盲区和换向限制见 [脱困行为](FOLLOWER_RECOVERY.md)。它们不是全局导航规划器。

## 相机与雷达对齐

```bash
python3 radar_system/calib_check.py --seconds 40
python3 radar_system/scan_doctor.py --seconds 10
```

让目标在左/中/右及不同距离站定，检查光学坐标变换后的目标与雷达腿部点簇是否重合。`calib_check.py` 输出的外参建议应写入共享 `sensors` 字段。先确认雷达安装位置，再调整相机偏移和偏角。

`calibrate_lidar.py` 现在原子更新共享配置中的 `raw_lidar_yaw_deg`，不再自动 pkill 雷达进程。默认采用标准协议方向（前 0°、左 90°、后 180°、右 270°）；只有完成现场标定后才填写偏移。使用前设置外部配置环境变量；完成后在停车状态重启整套服务，跟随层不可重复叠加同一个角度。

自反射与盲区从实测剖面判断。跟随 `self_hit_skin_m` 默认 5 cm，驱动防撞层为 2 cm；增加它会同时扩大真实近障碍被滤掉的范围。宽缺口保持未知，不应仅为消除“无路”提示扩大过滤或屏蔽范围。

## 状态诊断

| 现象 | 重点检查 |
|---|---|
| PAUSED / motion_authority | 是否明确选择 FOLLOW，配置摘要是否一致 |
| FAULT / ESTOP | 驱动状态中的原因；修复后停稳 reset，再选择任务 |
| 定位不健康 | 本地话题、frame、时间戳、跳变及驱动 epoch |
| 目标频繁切换 | 相机深度质量、相机/雷达外参、关联门限 |
| swept_path / aeb_hard | 障碍、完整车身尺寸、未知区域、自反射 |
| 手动与跟随都无输出 | 雷达全帧时效、底盘反馈、电压、租约龄期 |

正常新入口中雷达缺失或过期会锁存故障停车。`ScanGuard` 单独类保留旧调试降级行为，但不能把它当作新控制入口允许无雷达驾驶的依据。碰撞走廊兜底也不等于完整局部规划。

## 录包与复现

```bash
bash radar_system/record_follow_bag.sh
python3 radar_system/bag_replay.py BAG --out timeline.jsonl
python3 radar_system/bag_replay.py BAG --set follow_breadcrumbs=False
# 仅用于没有里程计的历史包，明确选择旧式合成积分
python3 radar_system/bag_replay.py OLD_BAG --legacy-velocity-odometry
```

录包包括感知、原始/配置的本地位姿、IMU/TF、驱动状态和三种控制请求。回放只计算，不下发运动。使用与录制时匹配的配置；这是开环回放，改算法不会改变录下的真实车身轨迹，不能据此宣称新的路线已通过实车验证。

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

### 深度可信度 — `radar_system/ai_3d_detector.py`

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

### 野值门控 — `motion_safety.AlphaBetaTracker`

```python
gate_base_m  = 0.35    # 允许的新息幅度下限
gate_rate_mps = 2.5    # 目标动得快时按 dt 放宽
max_rejects  = 3       # 连续拒绝几帧后认定目标真的跳变并重置
```

超出门限的观测会被丢弃,滤波器靠预测外推滑行,此时速度被
`coasting_speed_cap`(默认 0.15 m/s)压住。终端会显示限制原因 `coasting`。

- 正常快走被误拒(`野值` 计数随人走动增长)→ 调大 `gate_rate_mps`
- 野值穿透(车偶尔无故加速)→ 调小 `gate_base_m`

### 目标锁定 — `motion_safety.TargetLock`

```python
lock_radius_m  = 0.55   # 帧间关联半径,超出即认为不是同一个人
lock_timeout_s = 1.50   # 关联不上多久后解锁、允许重选
confirm_frames = 3      # 连续几帧位置一致才锁定
```

终端仪表盘会显示 `锁定` / `未锁`。

- 人快速横向移动时频繁解锁 → 调大 `lock_radius_m`(但太大会容易被旁人抢走)
- 想更快重新捕获 → 调小 `lock_timeout_s`

### 相机 / 雷达交叉证伪

```python
cross_check_cone_deg = 10.0   # 在目标方位 ±N° 内查雷达
range_conflict_m     = 1.00   # 相机比雷达远这么多即判为冲突
```

规则:雷达更近就采信雷达;差超过 `range_conflict_m` 则整帧作废并停车,
状态变为 `SENSOR_CONFLICT`,终端 `冲突NNN` 计数递增。

**冲突计数持续增长说明标定有问题**,按顺序排查:

1. 相机与雷达的**外参**对不上(两者不在同一个原点/朝向),先量安装位置
2. 雷达扫到了车身自己的结构件 → 调大 `scan_min_valid_m`
3. `cross_check_cone_deg` 太宽,把旁边的墙也算进来了 → 调到 6°

> 正常跟随时偶发几次冲突是可以接受的(人的腿和躯干本来就不同距离),
> 但**每分钟超过几次就必须停下来查**,不要靠调大 `range_conflict_m` 掩盖。

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
- `grid_utils.sector_min` —— 新增 `angle_min_deg` 参数
- `radar_web_server.scan_cb` —— 传入 `degrees(msg.angle_min)`

`scan_doctor` 会打印 `angle_min` 并在其非 0 时告警,差 180° 时点名
「前后完全颠倒」。本机跑出来是 `+0.0°`,不会告警。

> ⚠️ **不要采纳 scan_doctor 早期版本建议的 `scan_min_valid_m = 0.22`。**
> 那是全向门限,会让所有方向都看不见 0.22m 以内的东西,包括真正挡在
> 车头前的障碍物。自反射全在侧后方时不需要任何配置 —— `drop_self_hits()`
> 的车体轮廓过滤会自动处理。新版 scan_doctor 已经会这么提示。

---

## 10. 跟踪人:纯追踪转向、自车运动补偿、轻量重识别

这一轮改的是「跟人」本身,不是避障。三件事各自对应一个实车上能看见的毛病。

### 10.1 转向律 — `motion_safety.pure_pursuit_steer`

```python
pursuit_gain    = 1.00   # 1.0 = 几何正解,想更稳就往下调
min_lookahead_m = 0.45   # 前视距离下限,防止贴脸时曲率爆掉
```

改造前是 `steer = kp_steer * bearing`,一个**与距离无关**的比例增益。
轴距 0.54 m 下的实际数字:

| 人在哪 | 视线角 | 几何正解 | 老式子 (kp=1.1) |
|---|---|---|---|
| 1.6 m | 0.30 rad | **0.22 rad** | 0.33 rad(超打 50%) |
| 2.5 m | 0.30 rad | **0.14 rad** | 0.33 rad |
| 3.5 m | 0.30 rad | **0.10 rad** | 0.33 rad(超打 240%) |

远处那一档就是「跟人画龙 / 左右摇摆」的来源:打过头 → 冲过中线 → 反打。
纯追踪按 `κ = 2·sin α / L_d` 算曲率,再按固件的 `TurnR` 定义折成前轮转角,
`yaw_from_steer` 换回角速度时能精确还原同一个转弯半径,整条链路自洽。

- 还是觉得跟得太急 → `pursuit_gain` 调到 0.8,**不要**去改轴距或轮距
- 贴近时原地摆头 → 调大 `min_lookahead_m`
- `kp_steer` 已删除。它和纯追踪是两套东西,留着只会让人以为还能调

### 10.2 自车运动补偿 — `TargetLock.advance`

锚点存在**车体坐标系**里,而车自己在动。改造前拿上一帧的观测位置直接和这一帧
比,等于假设车是静止的。实测量级:车 1.2 rad/s 转向、人在 2 m 处,

```
自车旋转  2.67 m × sin(1.2 × 0.1) ≈ 0.24 m
自车前进  0.55 × 0.1              ≈ 0.055 m
人自己走  1.5 × 0.1               ≈ 0.15 m
                                  合计 ≈ 0.45 m   (关联半径 0.55 m)
```

单帧勉强够,**检测掉一帧就是 0.9 m,必然掉锁** —— 所以转弯比直行更容易跟丢,
而转弯恰恰是最不能跟丢的时候。现在每个周期用底盘实测的 `(speed, yaw_rate)`
把锚点搬到当前车体系,再叠目标自己的速度做外推。

遥测新增 `target_speed_fl`(目标在车体系下的前向/左向速度,m/s)。
**这两个数持续为零而人在走,说明 `/wheeltec/status` 的速度反馈没进来**,
补偿等于没开 —— 先查底盘反馈,不要去调 `lock_radius_m`。

### 10.3 轻量重识别 — `appearance_similarity`

```python
appearance_floor    = 0.45   # 相似度门限,低于此不认为是同一个人
appearance_weight_m = 0.60   # 外观不像在关联评分里折算成多少"米"
height_tolerance_m  = 0.25   # 可见身高差多少算完全不像
signature_ttl_s     = 20.0   # 签名保鲜期,过期后允许重新认人
```

改造前解锁后重选目标的评分是 `abs(x)*1.5 + abs(z - 期望距离)` ——
**谁最正对车头就跟谁**。人转过拐角、被柱子挡两秒、或者迎面来个人,回来就跟错了。

检测器现在顺手输出两个不要钱的身份线索(都在已有 RGB/深度上算,不占 NPU):

- `height_m` —— bbox 像素高 × 深度 / fy,即**可见部分**的物理高度
- `color` —— 上半身 HSV 色调直方图(12 bin,L1 归一化)

锁定期间它们参与关联评分;解锁后重选时,只要签名还新鲜就**必须**长得像
才允许锁定,宁可继续搜索也不跟陌生人走。

遥测新增 `signature_ready` 与 `appearance_rejects`。

- `signature_ready` 一直是 `false` → 检测器版本旧,或者人穿得太暗/太灰
  (`COLOR_MIN_SAT/VAL` 滤掉了全部像素)。此时系统自动退化成纯几何关联,
  行为与改造前一致,**不会更差**
- `appearance_rejects` 持续增长但人就在眼前 → `appearance_floor` 太高,
  先调到 0.35 试;逆光环境下色调直方图本来就不稳
- 中途换外套会掉锁 → 这是设计如此。等 `signature_ttl_s` 过期会自动重新认人

### 10.4 检测器发布频率

```python
ANNOTATED_PERIOD_S = 0.20    # 标注图 5Hz;JSON 目标数据不受影响,仍是满帧
```

标注图是整个推理循环里最贵的一步(色彩转换 + 全分辨率 `tobytes()` + ROS 序列化,
640×480 下约 1 MB/帧)。满帧发布会直接压低检测帧率,而刹车包络的
`control_latency_s = 0.35 s` 正是建立在检测帧率之上的 —— 画面好看一点,
换来的是刹车距离变长。要看流畅画面就调小这个值,但先确认帧率没掉。

