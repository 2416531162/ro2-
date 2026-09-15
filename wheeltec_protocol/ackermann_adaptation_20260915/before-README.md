# 轮趣底盘 × RK3588 集成

用正点原子 **ATK-DLRK3588** 替代原「ROS2 控制板」，直接驱动轮趣科技（WHEELTEC）底盘。

---

## 📖 先读哪个

| 你是 | 先读 |
|---|---|
| **接手这个项目的 AI / 开发者** | 👉 **[`交接文档.md`](交接文档.md)** —— 环境、坑、未解问题、安全红线全在里面 |
| 想查串口协议 | [`PROTOCOL.md`](PROTOCOL.md) —— 帧格式、BCC、换算、实机验证记录 |
| 出事了要停车 | [`estop.py`](estop.py) —— 立刻跑 `python3 -u estop.py 10` |

---

## 🚨 安全警告（一句话）

**该底盘存在 1~2 秒命令延迟，且停止发指令后仍会继续执行旧指令。**
**任何运动测试前：先架空车轮 + 确认物理断电开关在手边。**
（详情见 `交接文档.md` 第 7 节 —— 上一轮测试中小车曾意外前冲。）

---

## 📁 文件说明

| 文件 | 用途 | 状态 |
|---|---|---|
| `PROTOCOL.md` | 轮趣串口协议完整文档 | ✅ 已实机验证 |
| `交接文档.md` | **项目交接 / 经验总结 / 未解问题** | ✅ |
| `wheeltec_driver.py` | ROS2 驱动节点（发布 odom/imu/voltage，订阅 cmd_vel） | ⚠️ 读已验证，写有延迟问题 |
| `wheeltec_monitor.py` | 独立串口监视器，**不依赖 ROS2** | ✅ 实测可用 |
| `estop.py` | **紧急停车**（持续发零帧） | ✅ |
| `latency_test.py` | 命令延迟量化测试（查未解问题用） | 🆕 待测 |

---

## 🚀 快速上手

### 看底盘遥测（零风险）

```bash
# 在板子上
python3 -u wheeltec_monitor.py
```

输出：
```
     #      X速度      Y速度     Z角速度    AccX    AccY    AccZ    GyrX    GyrY    GyrZ    电压V   BCC
     1    0.000    0.000    0.000   0.059   0.032   9.805  0.0003  0.0003 -0.0011  25.024  197
--- 统计: 有效 46  校验失败 1  帧率 22.32 Hz ---
```

### 跑 ROS2 驱动（只读安全）

```bash
# 在板子上
cd /root/wheeltec && ./run_driver.sh
```

发布的话题：
```
/odom             nav_msgs/Odometry      20 Hz
/imu              sensor_msgs/Imu        20 Hz
/voltage          std_msgs/Float32       20 Hz
/wheeltec/status  std_msgs/String         1 Hz   (JSON 诊断)
```

订阅：
```
/cmd_vel          geometry_msgs/Twist    ⚠️ 会让车动，先架空！
```

### 紧急停车

```bash
python3 -u estop.py 10        # 持续发 10 秒零帧
```

---

## 🔌 串口识别（重要）

两个底盘外设的 USB 转串口芯片**型号完全相同**（WCH `1a86:55d4`），只能靠序列号区分：

| 设备 | by-id 路径 | 波特率 |
|---|---|---|
| **轮趣底盘** | `/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00` | 115200 |
| N10P 激光雷达 | `/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0001-if00` | 460800 |

> ⚠️ **不要用 `/dev/ttyACM0` / `ttyACM1` 硬编码** —— 重启后编号可能互换。

---

## ❓ 当前未解决问题

1. **写方向命令延迟 1~2 秒，原地旋转无响应，停车不即时** —— 详见 `交接文档.md` 第 6 节
2. **用户说接了两块板，但 USB 只枚举出一块** —— 同上 6.4

---

## 🔬 协议速查

**接收（底盘 → RK3588，24 字节，20 Hz，大端）**
```
7B | inhibit | vx(i16) | vy(i16) | wz(i16) | acc×3(i16) | gyro×3(i16) | 电压(u16) | BCC | 7D
 0  |    1    |  2..3   |  4..5   |  6..7    |   8..13     |   14..19    |  20..21   | 22  | 23
```

**发送（RK3588 → 底盘，11 字节，大端）**
```
7B | 模式 | 0 | vx(i16) | vy(i16) | wz(i16) | BCC | 7D
 0 |  1   | 2 |  3..4   |  5..6   |  7..8    |  9  | 10
```

**换算**
| 量 | 公式 |
|---|---|
| 速度 | `raw / 1000.0` → m/s（raw 为 i16）|
| 加速度 | `raw / 1671.84` → m/s² |
| 角速度 | `raw × 0.00026644` → rad/s |
| 电压 | `raw / 1000.0` → V（raw 为 u16）|
| BCC | `XOR(前面所有字节)` |

**停车帧固定值**：`7b 00 00 00 00 00 00 00 00 7b 7d`

---

## 📚 参考资料

- 轮趣官方 ROS2 驱动源码：[`CarlDegio/turn_on_wheeltec_robot`](https://github.com/CarlDegio/turn_on_wheeltec_robot)（本次协议来源）
- 第三方安全封装库（含 inhibit 位说明）：[`niuma-phd/wheeltec_vcu_serial`](https://github.com/niuma-phd/wheeltec_vcu_serial)
- 正点原子硬件参考手册：`10、用户手册/硬件参考手册/`（第 3.16~3.19 节讲 Type-C）
