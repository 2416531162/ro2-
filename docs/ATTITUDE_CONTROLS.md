
## 坡度开关与永久记忆 IMU 清零（App 1.11.25）

App「车辆状态 → 定位与姿态」提供「开启 / 关闭坡度补偿」和「IMU 清零」。

- 上行 `{"type":"tilt","enabled":false,"id":"t1"}`：停稳确认后原子保存 `~/rtk/fusion.json` 的 `tilt_compensation_enabled`，保留原标定与天线高度。
- 上行 `{"type":"imu_zero","id":"z1"}`：把当前横滚、俯仰作为水平零点，先原子保存 `~/rtk/imu_zero.json`，成功后才应用并回报成功。文件格式为 `{"version":1,"roll_deg":2.0,"pitch_deg":-1.0}`，单位为度。文件与目录均 fsync；读回严格校验版本、数值及角度范围。零点永久保存，重启 line 或整车自动恢复。再次主动清零覆盖上次零点；同一 id 的重复帧不重新选择零点。清零前先放平车辆。
- 保存的零点同时修正 `line.attitude.roll_deg/pitch_deg` 和融合坡度补偿。IMU 航向、陀螺、双天线航向及原始 `/imu/data` 保持原接口。
- 配置写入失败时不改变当前零点，回报失败；保存文件损坏时服务仍可启动，通过 `zero_save_error` 提示操作员重新清零，显式清零可修复文件。
- WebSocket ack `forwarded` 仅表示转发，以匹配 id 的 `line.last_result` 为最终结果。断连或 5 s 无结果会结束等待，不自动重发。
- 仅 IDLE / MANUAL、运动状态新鲜健康、无控制指令、实测底盘轮速连续停稳 0.3 s、无运行 / 暂停待续任务时接受。清零另需 0.2 s 内的新鲜 IMU 姿态。停稳判断不依赖 RTK 固定解或融合位姿初始化。
- `line.tilt` 的 `zero_persistent=true` 明确声明永久保存；`zeroed` 表示已应用零点，`zero_roll_deg/zero_pitch_deg` 为已应用的原始零点角度，`zero_save_error` 为保存文件读取错误或 null。缺少新字段的旧车仍按临时清零显示。开关能力及 `blocked/zero_blocked` 保持原接口。
