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
