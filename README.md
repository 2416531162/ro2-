# Jetson 摄像头与雷达人体跟随

当前链路：**Astra RGB-D → YOLO26s 人物检测＋N10P 雷达 → 人体锁定与局部跟随 → 控制权仲裁 → 驱动防撞 → 底盘**。

正式跟随入口默认使用 **MPPI＋PyTorch CUDA**，1024 条采样、40 步预测、20 Hz 控制。
CUDA 不可用时启动报错；服务启动后等待显式选择 FOLLOW。启用、性能检查和纯追踪回退配置见
[Jetson MPPI](docs/MPPI_JETSON.md)。服务名和配置路径中的 `rk3588` 保留兼容旧部署。
ROS 启动入口统一读取 `/etc/rk3588/runtime.env`，可用 `ROBOT_RUNTIME_ENV=/绝对路径/其他.env`
选择文件；当前进程环境变量优先于文件，文件优先于默认值。默认文件缺失时用默认值，显式
指定的文件缺失、不可读或格式错误则退出。文件遵循 `EnvironmentFile` 赋值/引号规则，不是
Shell 脚本；服务单元不再单独解析它。启动日志列出最终 ROS、Python、模型和标定路径。
完整键名和板端路径示例见 [环境模板](deployment/runtime-jetson.env.example)。
视觉默认加载已随项目提供的官方 `radar_system/models/yolo26s.pt`，只检测 COCO `person`，
以 512 像素输入和 CUDA FP16 推理；处理上限 30 FPS，实际帧率须在板端测量。
可在目标 Jetson 上导出 TensorRT `.engine` 后显式设置 `RK3588_POSE_MODEL`，不再需要自定义
`libtrt_engine_wrapper.so`。运行环境还须安装 Ultralytics，不能因此替换板端匹配的 CUDA PyTorch。

- `robot_core`：统一机器人标定、运动学、观测契约和本地位姿历史。
- `radar_system`：ROS 适配与纯跟随引擎分离；相机/雷达关联、扫掠检查、有限脱困、网页及板载显示。
- `wheeltec_protocol`：唯一串口出口，统一手动/跟随/导航控制权、时效检查与故障停车。
- `deployment`：整套版本打包、内容校验、服务切换及回滚；服务启动后等待显式选择跟随。

当前跟随使用统一 `/odom`，GNSS 融合定位输入和导航控制入口已留好。卫星驱动、融合定位、航点规划尚未实现。之前移除的 SLAM、点云累积、地图管理、RTK/CORS 服务不在当前运行链路中。

```bash
# 本机：测试、生成发布包
python3 -m pytest tests -q
python3 deployment/manage.py stage artifacts/follow-release
python3 deployment/manage.py verify artifacts/follow-release

# 开发板：安装发布包后启动已有服务，仍需网页点击跟随
bash radar_system/start_all.sh headless
```

网页默认端口 `8088`，主页 `/`。完整部署命令、控制接口和 GNSS 接入约束见 [架构说明](docs/ARCHITECTURE_ROADMAP.md)。不能只替换一个目录；新栈需要整套发布。

- [视觉模型与跟踪说明](docs/TRACKING.md)
- [统一配置与标定](docs/TUNING.md)
- [跟随脱困行为](docs/FOLLOWER_RECOVERY.md)
- [驱动接口](wheeltec_protocol/README.md)

本轮视觉模型变更未在 Jetson 上验证，不能将 30 FPS 目标视为已达成或实车验收通过。
