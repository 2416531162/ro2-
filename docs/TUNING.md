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
