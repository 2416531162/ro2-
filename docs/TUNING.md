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
| YOLO 推理 (RK3588 NPU) | 30~120 ms |
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
