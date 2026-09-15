# RK3588 × 轮趣阿克曼底盘

当前部署：`rk3588-wheeltec.service` 已启用，开机采集串口 0002 的真实遥测。默认 `receive_only: true`，发送计数为 0。阿克曼的原地打方向是停车时改变前轮转角。

## 已完成

- 固定 `/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00`，115200 8N1；不按 ACM 编号猜设备。
- 发布 `/odom`、`/imu`、`/voltage`、`/wheeltec/status`，实车反馈约 20 Hz。
- `/ackermann_cmd` 使用 `ackermann_msgs/AckermannDriveStamped`：速度 m/s、前轮转角 rad；消息必须带当前时间戳。
- `/cmd_vel` 使用标准 `geometry_msgs/Twist`：纵向速度与车体角速度。停车打方向使用 `/ackermann_cmd`。
- 50 Hz 单线程串口发送、最新指令覆盖、300 ms 指令/反馈超时、连续零帧停车、断连清除旧指令、重新启用后才接受新动作。
- 默认限速 0.15 m/s、转角限制 0.35 rad。参数在启动时读取，运行中只读。

## 板子上的命令

```bash
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 /root/wheeltec/control.py status
systemctl status rk3588-wheeltec.service
ros2 topic echo /wheeltec/status std_msgs/msg/String --once
```

`control.py stop` 请求驱动锁存停车；仅在已配置发送模式时发送停车帧。遥测模式不会接管遥控器或发送控制字节，服务响应成功也不代表机械急停。驱动运行时不要直接打开同一串口；`wheeltec_monitor.py` 已改为独占打开。

## 尚待实车确认

具体控制板型号、固件版本、轴距、转向字段定义与方向/倍率尚未确认。仓库中发现的两种协议并不相同：通用 11 字节速度帧采用 mode=0，阿克曼参考分支采用 mode=1、转角乘 0.5；它们均不是当前控制板固件身份证明。因此 `/root/wheeltec/wheeltec.yaml` 保留 `protocol: unconfigured`、`protocol_confirmed: false`、`wheelbase_m: 0.0`。ROS arm 服务会明确拒绝启用运动。

确认固件后配置 `protocol: steering_angle`（或只支持速度的 `twist`）、`mode_byte`、`steering_scale`、`wheelbase_m`，再完成架空验证。`twist` 固件没有可验证的静止转角接口时，适配器会拒绝通过虚构行驶速度实现打方向。

此前实车存在命令延迟与意外前冲记录。新的发送和停车逻辑已通过模拟串口测试；这并不证明下位机内部延迟已经消除。实际运动验证仍需按 [交接记录](交接文档.md#7--安全红线血泪教训务必遵守) 先架空驱动轮，并确认物理断电开关可随时操作。主机端软件也不能代替下位机通信超时保护。

## 验证与回滚

适配包在 `ackermann_adaptation_20260915/`：包含原版、修改版、差异、20 项核心测试、ROS 模拟串口测试、实车遥测验收与回滚记录。见 `VERIFICATION.txt`。

本地指定副本回滚：`ackermann_adaptation_20260915/ROLLBACK.sh /absolute/path/to/driver.py`。

在 Mac 上回滚板端部署：`ackermann_adaptation_20260915/ROLLBACK.sh --board`。这会停用新增服务并恢复原驱动及脚本；保留审计目录和新增的 ackermann_msgs 依赖，不自动启动原版运动驱动。
