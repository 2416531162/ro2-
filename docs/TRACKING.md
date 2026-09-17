# 雷达与摄像头跟踪架构

## 运行模块

| 模块 | 职责 |
|---|---|
| `real_lidar_node.py` / `n10p_pipeline.py` | N10P 扫描解码与 `/scan` |
| `person_pose_node.py` | RGB-D 最新帧缓冲、人体姿态与测距发布 |
| `pose_inference.py` | RKNN NPU、NumPy DFL 解码、NMS、17 点骨架 |
| `depth_measurement.py` | 深度有效比例与近百分位测距，图像 stride/大小端处理 |
| `person_follower.py` / `lidar_track.py` | 人体锁定、相机与雷达关联及接力 |
| `follower_recovery.py` / `footprint.py` / `motion_safety.py` | 局部扫掠路径、脱困和速度约束 |
| `radar_web_server.py` / `board_radar_gui.py` | 网页遥测/手动接管、板载相机与雷达画面 |

移除 SLAM、地图保存、AMCL、三维重建、点云累积、RTK/CORS、Foxglove/RViz 入口、通用物体及人脸模型。
车体矩形足迹、相机深度反投影和短期倒车历史用于安全控制，仍然保留。
不自动启动跟随运动；通过主页按钮或 `run_follower.sh` 启动。

## 姿态模型

默认路径：`radar_system/models/yolov8n_pose_rk3588_fp16.rknn`。
可用环境变量 `RK3588_POSE_MODEL` 指定同格式模型。旧的 `RK3588_YOLO_MODEL` 不再使用。
不存在的权重或错误输出格式会在节点启动时明确报错，没有旧检测模型回退。

采用 Rockchip model-zoo 的 **YOLOv8n-pose 原始输出导出**，输入固定 640×640 RGB uint8，
转换配置在模型中完成 `/255` 归一化。输出为三个 `(1,65,H,W)` 检测头（H/W=80、40、20）
与已解码的 `(1,17,3,8400)` 关键点；同时兼容 `(1,51,8400)` 的等价布局。
普通 Ultralytics 单输出 ONNX、旧六头 YOLOv8 检测模型不能直接使用。
已生成并随项目提供 RK3588 FP16 权重（约 8.2 MiB），不是旧权重改名。

`models/yolov8n-pose.onnx` 来自 Rockchip 官方示例的下载地址；来源及校验和见同目录模型清单。
FP16 转换不使用随意拼凑的 INT8 标定集。需要进一步做 INT8 时，应使用实车光照、距离、遮挡场景标定并复核精度。

可重现的转换（Docker，Linux amd64；Apple Silicon 通过容器模拟运行）：

```bash
docker build --platform linux/amd64 -f radar_system/tools/Dockerfile.pose \
  -t rk3588-pose-builder:2.3.2 radar_system/tools
docker run --rm --platform linux/amd64 -v "$PWD/radar_system:/work" \
  rk3588-pose-builder:2.3.2
```

也可在安装相同依赖的 Linux 环境直接执行 `python radar_system/tools/convert_pose.py`。
构建先验证 ONNX 维度与结构，成功导出后原子替换目标文件。工具链固定 ONNX 1.16.1，
因为 toolkit2 2.3.2 使用了新版 ONNX 已移除的 `onnx.mapping`。
ONNX CPU 示例图推理检测出 3 人，关键点显示通过检查；RKNN 构建成功。
x86 模拟器在 Apple Silicon/QEMU 下停滞于 SessionPreparing，已停止，未宣称转换后推理通过。
开发板 `rknn-toolkit-lite2`、`librknnrt` 与 NPU 驱动需配套；本地转换没有替代设备验证。

## 跟踪话题兼容

`/camera/ai_detection/targets` 仍是 JSON 数组，保留 `label=person`、`conf`、框坐标、
`bearing_rad`、`stamp`、`depth_ratio`、`range_valid`，有效深度时提供 `x/y/z/distance`。
新增 `keypoints`，按 COCO 17 点顺序输出原图像素坐标 `[x,y,confidence]`。
越界或非有限关键点置为 `[0,0,0]`，低置信度关键点不绘制。
跟随锁定仍使用位置和运动一致性，姿态关键点不作为身份识别或独立放宽避障的依据。

深度帧过期、RGB/深度尺寸不一致或深度空洞过多时，仅发布人体方位，交给原雷达测距兜底。
`/camera/ai_detection/image` 输出人体框、骨架和测距图。
`/camera/ai_detection/status` 提供模型名、FPS、`inference_ms`、预处理/推理/后处理的 `pipeline_ms`。
后者不包含图像绘制、ROS 发布和等待时间。网页 `/api/stream` 包含这份 `ai_status`。

## 路径性能与验证

`LocalRecovery.clearance()` 将全部候选车姿 × 障碍点、车姿 × 车身边界点批量计算，
一次扫描只准备一份 NumPy 点数组和光束邻居索引。历史观测按批次坐标变换，最近可覆盖观测优先。
仍保留 2 cm 采样、转向过渡、碰撞补偿、完整车体、未知区域阻挡、有限历史倒车。
同时修复重新捕获目标时 BRAKE 状态可能跳过停车等待的原有缺口。

本地新旧实现对照 1,200 组扫描/转角/前后档组合，净空与首次阻挡诊断一致。
本机初次对比（非 RK3588，Python 3.12.13 / NumPy 2.5.3）：五条候选路径中位耗时 **17.731 → 1.804 ms**，约 **9.8×**；
P95 为 18.164 → 4.386 ms。这是路径几何计算加速，不代表姿态 NPU 推理加速。

在目标开发板复测：

```bash
python3 radar_system/tools/benchmark_tracking.py --iterations 200
python3 -m pytest tests -q
bash radar_system/run_follower.sh --dry-run
ros2 topic echo /camera/ai_detection/status
```

260 项硬件无关测试通过（Python 3.11 / NumPy 1.26.4），覆盖姿态头/关键点对应关系、NMS、大小端/行填充、消息发布、深度失效回退、
光束批量查询对照、狭窄门、盲区、雷达接力、换向停车、手动控制及已删除接口。
浏览器验证桌面/手机页面、遥测、暂停、缩放；Qt 离屏验证窗口与相机模式。
真实传感器 QoS、板端 NPU 性能、相机外参与深度配准、运动延迟和实车避障仍需设备验收。

## 部署迁移

更新实际 systemd 工作目录中的文件后执行 `bash radar_system/start_all.sh`。
该脚本先停止并禁用旧 `mapping/mapping3d/rtk/foxglove` 服务，再启动 `camera/lidar/ai/web/gui`。
`ai` 改为 `run_ai.sh → person_pose_node.py`；如设备另有直接指向旧检测器的自定义服务，需修改其 `ExecStart`。
相机启动只保留 RGB-D 驱动，已移除点云生成和仅供建图使用的静态 TF 发布。
网页主入口回到 `/`，旧 `/map`、`/map2d`、地图管理与 CORS 接口均不再提供。
