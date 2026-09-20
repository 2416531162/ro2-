# RK3588 轮趣底盘驱动

`wheeltec_driver.py` 是唯一串口拥有者，发布原始里程计、IMU、电压及驱动/控制权状态。当前仓库 YAML 选择 `twist` 协议、115200 波特率、串口 0002；这描述仓库配置，不表示本次已部署或重新验证固件。

物理尺寸、速度上限、超时和防撞物理参数统一读取 `robot_core/robot.json`；协议、设备端口和 TF 发布开关保留在 `wheeltec.yaml`。`ros_env.sh` 从 `/etc/rk3588/runtime.env` 加载 `RK3588_ROBOT_CONFIG` 等值（可用 `ROBOT_RUNTIME_ENV` 指向其他文件），进程环境优先于文件；服务单元不再重复读取。正常模式拒绝孤立覆盖共享参数。默认速度上限是共享配置中的 1.3 m/s，跟随行为另有限速，不应把驱动上限视为推荐行驶速度。

正常运行只接收 `/manual/command`、`/follow/command`、`/navigation/command`，请求携带 `vx/wz/stamp/epoch/profile_hash`。`motion_authority.py` 在驱动锁内完成仲裁，`ControlPolicy` 和 `ScanGuard` 再执行限幅、看门狗及防撞。旧 Twist/Ackermann 入口仅在显式 `legacy_commands: true` 的互斥调试模式中开放。

```bash
bash wheeltec_protocol/run_control.sh status
bash wheeltec_protocol/run_control.sh stop
bash wheeltec_protocol/run_control.sh reset
```

`bash wheeltec_protocol/run_control.sh drive --speed ... --steering-deg ... --seconds ...` 是有界手动运动请求，结束后锁存停车。reset 只回到 IDLE，不恢复旧任务。不要在驱动运行时用其他程序打开相同串口。
`estop.py` 在服务可能运行时调用当前发布版本的 `run_control.sh stop`；即使 ROS 请求失败也不会转而争抢串口。只有 systemd 确认驱动已停止、MainPID 为 0 且没有独立驱动进程，才加载 pyserial 并使用原有持续零帧停车路径。无法确认运行状态则拒绝直接打开串口。该急停入口不检查视觉模型或 CUDA；纯串口诊断工具保持独立。

默认底盘发布 `/odom` 及 `odom → base_link`，base_link 为后轴中心。融合定位接管该 TF 时需关闭 `publish_tf`，并配置行为消费的本地位姿话题；原始与融合话题可分开。

完整服务安装、回滚、模式契约与测试说明见 [架构说明](../docs/ARCHITECTURE_ROADMAP.md)。历史协议资料见 [PROTOCOL.md](PROTOCOL.md)，其中参考固件公式不代替当前实车左右转向和倒车验证。本次没有发实车运动指令。
