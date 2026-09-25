# 雷达与摄像头跟踪架构

## 运行模块

| 模块 | 职责 |
|---|---|
| `real_lidar_node.py` / `n10p_pipeline.py` | N10P 扫描解码与 `/scan` |
| `person_pose_node.py` | RGB-D 最新帧缓冲、人物框与测距发布（名称沿用现有入口） |
| `person_detection.py` | Jetson CUDA FP16 / TensorRT 人物检测；只取 COCO person |
| `pose_inference.py` | 显式选用旧 ONNX/RKNN 时的姿态适配 |
| `depth_measurement.py` | 深度有效比例与近百分位测距，图像 stride/大小端处理 |
| `robot_core` / `follower_config.py` | 共享物理配置、运动学和位姿历史；跟随行为参数 |
| `motion_client.py` / `wheeltec_protocol/motion_authority.py` | 带时间戳、控制租约的运动请求与控制权仲裁 |
| `person_follower.py` | ROS 消息/时钟/控制租约适配 |
| `follower_engine.py` / `follower_perception.py` / `follower_controller.py` | 纯跟随引擎，感知关联与控制策略 |
| `person_tracker.py` / `lidar_track.py` | 使用共享本地位姿的人体轨迹、相机/雷达关联及接力 |
| `follower_recovery.py` / `footprint.py` / `motion_safety.py` | 局部扫掠路径、脱困和速度约束 |
| `turnaround.py` | 人在车后/侧后时的多段前进/倒车掉头搜索（纯几何） |
| `tools/follow_sim.py` | 跟随闭环仿真：真实引擎 + 按实车时序合成的相机/雷达/底盘 |
| `radar_web_server.py` / `board_radar_gui.py` | 网页遥测/手动接管、板载相机与雷达画面 |

移除 SLAM、地图保存、AMCL、三维重建、点云累积、RTK/CORS、Foxglove/RViz 入口、通用物体及人脸模型。
车体矩形足迹、相机深度反投影和短期倒车历史用于安全控制，仍然保留。
不自动启动跟随运动；通过主页按钮或 `run_follower.sh` 启动。

## 人物检测模型

Jetson 默认加载官方 [YOLO26s 检测模型](https://docs.ultralytics.com/models/yolo26/)
`radar_system/models/yolo26s.pt`，只取 COCO `person` 类，不计算人体姿态。
来源、体积和 SHA-256 见 `radar_system/models/detection_manifest.json`。
推理输入设为 512，CUDA PyTorch 使用 FP16；只保留最新相机帧并以最多 30 FPS 处理，
标注图最多发布 5 FPS，人物目标与状态逐个处理帧发布。FPS 上限不保证实际能达到 30；
在 MPPI 同时占用 GPU 的板端需检查 `/camera/ai_detection/status` 中的 `fps`、
`inference_ms`、`pipeline_ms` 和目标丢失情况，不能用官方 T4 延迟推断本机性能。

保留历史配置键 `RK3588_POSE_MODEL`，可显式选择检测用 `.pt`（CUDA PyTorch）或在目标
Jetson 用同一权重导出的 `.engine`（TensorRT）。两者都由 Ultralytics 加载，不需要
`libtrt_engine_wrapper.so`；缺少 CUDA、Ultralytics、模型文件或运行库时明确报错，
不自动换后端。相对模型路径以 `radar_system` 为基准，环境文件推荐绝对路径。
Ultralytics 在 `.pt`/`.engine` 路径负责 RGB、归一化 NCHW 浮点输入与输出还原；
旧 ONNX 姿态路径使用归一化 NCHW 浮点输入，旧 RKNN 姿态路径使用 NHWC RGB uint8。
先安装与 JetPack 匹配的 CUDA PyTorch/torchvision；其余依赖已齐备后，用对应的
`RK3588_PYTHON -m pip install --no-deps 'ultralytics>=8.4,<9'` 安装 Ultralytics，
避免 pip 用普通 PyTorch wheel 覆盖 CUDA 构建。
不要在未核对依赖前直接对板端执行 `pip install -r requirements-jetson.txt`。
可选 TensorRT 引擎须在目标板上导出，输入尺寸保持一致：

```bash
/path/to/board/python -c "from ultralytics import YOLO; YOLO('/opt/rk3588/current/radar_system/models/yolo26s.pt').export(format='engine', imgsz=512, quantize=16, device=0)"
```

成功导出后才将 `RK3588_POSE_MODEL` 指向板端 `yolo26s.engine`。默认 `.pt` 不需要这一步。
如果现场 `/etc/rk3588/runtime.env` 已显式配置旧权重路径，须删除赋值或改为新 `.pt`；
进程和文件显式值始终优先于默认值，不自动迁移。
旧 `.onnx`（ONNX Runtime CPU）和 `.rknn`（RKNN Lite）仅为显式选择的历史兼容路径，
要求 Rockchip 四输出姿态模型，权重已不随 Jetson 发布目录提供；它们不适用于普通检测 ONNX。
`RK3588_YOLO_MODEL` 不再使用；RK3588 构建记录见 `wheeltec_protocol/交接文档.md`。
将官方权重随发布包再分发前应核对 Ultralytics 的模型和代码许可要求。

## 跟踪话题兼容

`/camera/ai_detection/targets` 仍是 JSON 数组，保留 `label=person`、`conf`、框坐标、
`bearing_rad`、`stamp`、`depth_ratio`、`range_valid`，有效深度时提供 `x/y/z/distance`。
普通检测不包含 `keypoints`，状态中的 `keypoint_names` 为空；仅显式选旧姿态模型时
目标消息保留 17 个关键点。跟随锁定只使用位置和运动一致性，不依赖骨架。

深度帧过期、RGB/深度尺寸不一致或深度空洞过多时，仅发布人体方位，交给原雷达测距兜底。
`/camera/ai_detection/image` 最多以 5 FPS 输出人体框和测距图（仅有订阅者时）。
`/camera/ai_detection/status` 提供模型名、FPS、`inference_ms`、预处理/推理/后处理的 `pipeline_ms`。
后者不包含 ROS 图像解码、深度测距、图像绘制、ROS 发布和等待时间。
网页 `/api/stream` 包含这份 `ai_status`。

## 路径性能与验证

`LocalRecovery.clearance()` 将全部候选车姿 × 障碍点、车姿 × 车身边界点批量计算，
一次扫描只准备一份 NumPy 点数组和光束邻居索引。历史观测按批次坐标变换，最近可覆盖观测优先。
仍保留 2 cm 采样、转向过渡、碰撞补偿、完整车体、未知区域阻挡、有限历史倒车。
同时修复重新捕获目标时 BRAKE 状态可能跳过停车等待的原有缺口。

本地新旧实现对照 1,200 组扫描/转角/前后档组合，净空与首次阻挡诊断一致。
本机初次对比（非 RK3588，Python 3.12.13 / NumPy 2.5.3）：五条候选路径中位耗时 **17.731 → 1.804 ms**，约 **9.8×**；
P95 为 18.164 → 4.386 ms。这是路径几何计算加速，不代表 YOLO26s 推理加速。

在目标开发板复测：

```bash
python3 radar_system/tools/benchmark_tracking.py --iterations 200
python3 -m pytest tests -q
bash radar_system/run_follower.sh --dry-run
ros2 topic echo /camera/ai_detection/status
```

当前完整测试入口为 `python3 -m pytest tests -q`，涵盖视觉、跟随、控制仲裁、共享定位和部署回滚。历史性能及显示验证记录不表示本次在设备上重新验收。
真实传感器 QoS、Jetson GPU 上的 30 FPS 目标、相机外参与深度配准、运动延迟和实车避障仍需设备验收。

## 部署迁移

使用 `deployment/manage.py` 成套打包并部署 `robot_core`、`radar_system`、`wheeltec_protocol`、`deployment`，见 [升级与回滚](ARCHITECTURE_ROADMAP.md)。
`start_all.sh headless` 启动已安装服务及被动跟随进程，等待网页明确选择 FOLLOW。它不替代部署器，也不会修改其他服务的启用状态。
`ai` 改为 `run_ai.sh → person_pose_node.py`；如设备另有直接指向旧检测器的自定义服务，需修改其 `ExecStart`。
相机启动只保留 RGB-D 驱动，已移除点云生成和仅供建图使用的静态 TF 发布。
网页主入口回到 `/`，旧 `/map`、`/map2d`、地图管理与 CORS 接口均不再提供。

## 车后目标误关联修正（2026-09-18）

现场反馈“雷达判人的位置更乱”后重新核查板端版本和关联链路。
板端运行文件与本地上一版哈希一致；暂停期间录制约 20 秒 `/scan`、`/odom`、
相机检测与跟随状态，共 1,214 条消息。原运行实例在这段时间切换目标 8 次。
同一数据从空轨迹离线回放：原版切换 8 次，修正版保持一个编号、切换 0 次。
录制没有真人位置真值，不能据此宣称定位精度或身份识别准确率。

修正内容：

- 删除“相机失联 0.15 秒后优先把 4 m 范围后方点簇认作目标”的跨门重定位。
  雷达延续必须通过原有运动预测、马氏距离和距离门；没有连续证据时允许丢失。
- 雷达只延续当前锁定的视觉确认目标，不再把所有旧视觉轨迹挂在环境点簇上。
- 多个点簇都能通过关联门时，拒绝这一帧的位置/速度更新，避免先任选一个，
  再用收紧后的协方差把该选择“证明”为正确。保留观测超时退出。
- 当前目标仍存在时，不因附近有另一个相机目标就改锁。丢失后仅允许 3 秒内、
  最后位置 0.6 m 内唯一的已确认相机目标局部重捕；否则等待再次显式启动跟随重置。
  旧配置名 `lidar_reacquire_gate_m` 为兼容保留，现在仅用于该相机局部重捕（0.2–0.6 m）。
- 雷达收到迟到回波前先清理失效轨迹，不能用新回波复活已过期身份。
- 保留移出相机视野后清除旧反向证据的修复。附近点簇若不能通过关联门，
  不阻止连续、唯一的雷达观测维持目标；站立本身不触发丢失。
- 遥测增加 `lidar_ambiguous_frames`；`target_locked` 反映当前目标可用状态，
  不再把保留用于重捕的旧编号显示为已锁定。

回归覆盖连续 0.9 m/s 绕车、车后站定 30 秒和附近杂波、不同里程计原点/朝向、
车后物体不能将前方目标瞬间拉走、多人候选拒绝、旧轨迹过期和相机旁观者不抢目标。
旧的“0.2 秒从车前 1.8 m 瞬移到车后 1 m 必须接回”测试已替换，
这种合成数据不能作为同一个人的运动连续性证据。

现场记录、离线对比与部署备份位于 `artifacts/rear-association-20260918/`。
合成测试与暂停期间回放不等于实车跟随验收；密集人群、遮挡后出现的相似物体，
当前平面点簇仍无法提供可靠的身份识别。

## 室内灵敏跟随（2026-09-24）

目标：室内摄像头＋二维雷达配合跟人——离近减速、离远加速；人走出相机后由雷达实时接力；
人绕到车后能尽快算出掉头动作。以下结论来自 `tools/follow_sim.py` 闭环仿真，**尚未实车验证**。

### 仿真工具

真实 `FollowerEngine` 加按实车时序合成的输入：相机 30 FPS、发布延迟 70 ms、58° 视场；
N10P 10 Hz、每圈约 450 个采样；行人两条腿随步态摆动；`/odom` 50 Hz；控制 20 Hz；
指令约 60 ms 后生效，底盘按驱动斜坡加速、1.5 m/s² 刹车、舵机 3 rad/s。

```bash
python3 radar_system/tools/follow_sim.py                     # 全部场景指标
python3 radar_system/tools/follow_sim.py walk_behind_left --trace
python3 radar_system/tools/follow_sim.py --set controller='"mppi"' --set mppi_device='"numpy"' \
    --set mppi_samples=512 --set mppi_solve_budget_ms=1e6       # 本机无 CUDA 时比较 MPPI
```

指标：`tracked` 有目标的比例、`lost_events` 丢失次数、`same_id` 是否始终同一轨迹、
`max_track_err` 轨迹估计与真实人位置的最大偏差、`max_cam_blind_s` 相机连续看不到人的最长时间、
`max_gap`/`end_gap` 车头到人的间距、`min_person_clear` 车身离人最近距离、`steer_rate_rms` 画龙程度。
验收门限在 `tests/test_follow_response.py`。仿真只能比较算法改动，替代不了实车验收。

### 改动与效果（纯追踪，5 个随机种子最差值）

| 场景 | 改动前 | 改动后 |
|---|---|---|
| 人以 1.3 m/s 走开 | 0.45 m/s 限速追不上，间距拉到 5.5 m 后丢失 | 不丢，最大间距 3.25 m，站定后 1.26 m |
| 人突然站住 | — | 车身离人最近 0.99 m |
| 人横着走出视野 | 相机看不到 3.35 s | 2.15 s |
| 人绕到车后站住 | 满舵一直前进绕圈，10 s 后仍偏 59°，离墙 0.11 m | 3.65 s 内重新进入相机视野 |
| 人绕到车后继续走远 | 丢失 | 不丢，掉头后跟上，站定 1.18 m |
| 视野外贴着椅子腿走并拐弯 | 轨迹被吸到椅子上，偏 1.9–2.9 m（编号不变） | 偏差 ≤ 0.30 m |

1. **速度按间距决定**：`max_speed_mps` 取底盘上限 1.3 m/s，雷达接力不再另设 0.45 m/s。
   实际速度由 `brake_envelope(间距, decel_mps2, control_latency_s)` 限制——
   按当前 1.0 m/s²、0.35 s，离人车头 1.65 m 时才允许 1.0 m/s。这两个值**必须实车测**（见 TUNING.md）：
   测得减速度更大、延迟更小，跟得更紧；测得更差则必须改小，否则刹不住。
   加速斜坡保持 0.9 m/s²：仿真中调到 1.5 对跟上步速没有改善（瓶颈在刹车包络），却让绕柱子的余量变小。
2. **掉头规划**（`turnaround.py`）：人在车后，或从相机看偏出 60° 时进入。每周期从当前位姿搜索
   最多三段满舵弧线（前进向人一侧打舵、倒车反打舵，两者都让车头朝人转），代价是相机重新看到人
   （相机位置量 ±20°）的预计用时，换向计停稳与舵机摆动时间。第一段不超过雷达认证净空，所有段
   与障碍点、与人（0.45 m）保持余量；已经比余量更近的点只禁止再靠近。每段按剩余行程收油，
   最高 1.0 m/s；扫掠净空、刹车包络、AEB 仍逐周期复核。人贴在车侧时倒车反打舵一段即可转正
   （前进满舵反而让人越来越偏后）；正后方 2 m 需约 6 s（1.72 m 转弯半径下的物理下限附近）。
   进入角 75°/60°/50° 的仿真对比：60° 最均衡，50° 在人横着走出视野时会多余地倒车。
3. **雷达接力关联**：行人卡尔曼加速度噪声 `track_accel_sigma` 1.5 → 3.0 m/s²。
   人拐直角时 1.5 的预测跟不上，关联门收得太紧，轨迹沿旧方向滑走或被旁边点簇吸走。
4. **静止杂物剔除**（`lidar_track.StaticClutter`）：椅子腿、桌腿与腿一样粗，两根还会被配成「一个人」。
   小点簇在里程计系原地持续出现 1.2 s 以上即判为静止，只从人体点簇里剔除，避障仍用全部点。
   判为静止之前就出现在已确认的人身边的条目永久豁免，所以站着不动的人不会被当成椅子。
   `/follower/status.static_clutter_segments` 为本帧剔除的段数。
5. **MPPI**：`w_heading` 2.5 → 10。人走出视野后相机重新看到人的时间由 4.0 s/9.0 s 降到 1.3 s/1.5 s。
   MPPI 在直线跟随时仍有明显转角抖动（`steer_rate_rms` 约 0.8，纯追踪为 0），站定后常斜 10° 左右；
   仿真中纯追踪在大多数指标上不差于 MPPI，MPPI 的优势是绕障。室内以跟人为主时，可在
   `/etc/rk3588/runtime.env` 设 `ROBOT_FOLLOW_CONTROLLER=pure-pursuit` 对比实车效果。

`/follower/status.turnaround_plan` 给出当前掉头方案：方向、本段行程、预计用时、各段长度。

### 实车验证建议

1. 先按 TUNING.md 测 `decel_mps2`、`control_latency_s` 并写入 `/etc/rk3588/robot.json`；
   第一次用 `--safe-mode`，确认刹车距离后再放开。
2. 空旷处依次试：人走开再站住、人横着走出视野、人绕到车后站住、人绕到车后继续走；
   录包（`record_follow_bag.sh`）并对照 `turnaround_plan`、`lidar_track`、`static_clutter_segments`。
3. 在椅子、桌子多的地方重复第 2 步，观察雷达接力是否贴在人身上。

## 相机补充通路确认

已接入独立于人体检测的深度图通路查询，详见 [深度通路确认](DEPTH_PATH.md)。
只有标定、像素有效性、采集位姿和时效检查通过后才补充雷达未知空间；不默认同高。
