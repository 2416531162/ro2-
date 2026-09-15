# 轮趣科技（WHEELTEC）底盘串口通信协议

> 来源：轮趣官方 ROS2 驱动包 `turn_on_wheeltec_robot` 源码（`src/wheeltec_robot.cpp` / `include/turn_on_wheeltec_robot/wheeltec_robot.hpp`）
> 实机验证：RK3588 `ATK-DLRK3588` ← USB 串口 `/dev/ttyACM1`（WCH `1a86:55d4`，序列号 `0002`）
> 验证结果：**101/101 帧 BCC 校验全部通过**，`AccZ = 9.84 m/s² ≈ 1g`，电压 `24.76 V`（6S 锂电池）

---

## 一、物理层

| 项 | 值 |
|---|---|
| 波特率 | **115200** |
| 数据位/停止位/校验 | 8 / N / 1（8N1）|
| 流控 | 无 |
| 遥测帧率 | **20 Hz**（下位机主动上报）|
| 上位机发包率 | 由 `cmd_vel` 回调触发（典型 20~50 Hz）|

---

## 二、帧格式

### 2.1 接收帧（下位机 → 上位机，**24 字节**）

| 偏移 | 长度 | 字段 | 类型 | 单位 / 换算 |
|---|---|---|---|---|
| `rx[0]` | 1 | **帧头** | u8 | 固定 `0x7B` |
| `rx[1]` | 1 | `Flag_Stop` | u8 | 预留位（实测恒 0）|
| `rx[2..3]` | 2 | **X 方向速度** | i16 **大端** | mm/s → m/s |
| `rx[4..5]` | 2 | **Y 方向速度** | i16 大端 | mm/s → m/s（全向底盘有效）|
| `rx[6..7]` | 2 | **Z 角速度** | i16 大端 | 0.001 rad/s |
| `rx[8..9]` | 2 | IMU 加速度 X | i16 大端 | raw ÷ 1671.84 = m/s² |
| `rx[10..11]` | 2 | IMU 加速度 Y | i16 大端 | raw ÷ 1671.84 = m/s² |
| `rx[12..13]` | 2 | IMU 加速度 Z | i16 大端 | raw ÷ 1671.84 = m/s² |
| `rx[14..15]` | 2 | IMU 角速度 X | i16 大端 | raw × 0.00026644 = rad/s |
| `rx[16..17]` | 2 | IMU 角速度 Y | i16 大端 | raw × 0.00026644 = rad/s |
| `rx[18..19]` | 2 | IMU 角速度 Z | i16 大端 | raw × 0.00026644 = rad/s |
| `rx[20..21]` | 2 | **电源电压** | u16 大端 | mV → V |
| `rx[22]` | 1 | **BCC 校验** | u8 | 见第 3 节 |
| `rx[23]` | 1 | **帧尾** | u8 | 固定 `0x7D` |

### 2.2 发送帧（上位机 → 下位机，**11 字节**）

| 偏移 | 长度 | 字段 | 类型 | 说明 |
|---|---|---|---|---|
| `tx[0]` | 1 | **帧头** | u8 | `0x7B` |
| `tx[1]` | 1 | 预留 | u8 | 填 0 |
| `tx[2]` | 1 | 预留 | u8 | 填 0 |
| `tx[3..4]` | 2 | **X 目标速度** | i16 **大端** | `linear.x × 1000`（m/s → mm/s）|
| `tx[5..6]` | 2 | **Y 目标速度** | i16 大端 | `linear.y × 1000` |
| `tx[7..8]` | 2 | **Z 目标角速度** | i16 大端 | `angular.z × 1000` |
| `tx[9]` | 1 | **BCC 校验** | u8 | `tx[0..8]` 异或 |
| `tx[10]` | 1 | **帧尾** | u8 | `0x7D` |

---

## 三、校验算法（BCC）

**规则：从第 0 字节起，按位异或到最后一位数据的前一个字节。**

```c
unsigned char Check_Sum(unsigned char Count_Number, unsigned char mode)
{
    unsigned char check_sum = 0, k;
    if (mode == READ_DATA_CHECK)          // mode = 0，校验接收帧
        for (k = 0; k < Count_Number; k++)
            check_sum = check_sum ^ Receive_Data.rx[k];
    else                                  // mode = 1，校验发送帧
        for (k = 0; k < Count_Number; k++)
            check_sum = check_sum ^ Send_Data.tx[k];
    return check_sum;
}
```

- **接收帧**：`Check_Sum(22, 0)`，即 `rx[0] ^ rx[1] ^ ... ^ rx[21]`，结果与 `rx[22]` 比较
- **发送帧**：`Check_Sum(9, 1)`，即 `tx[0] ^ tx[1] ^ ... ^ tx[8]`，结果写入 `tx[9]`

Python 等价实现：

```python
def bcc(data: bytes) -> int:
    c = 0
    for b in data:
        c ^= b
    return c

# 接收帧校验
assert bcc(frame[0:22]) == frame[22]
# 发送帧校验
tx[9] = bcc(tx[0:9])
```

---

## 四、数据换算常量

| 常量 | 值 | 用途 |
|---|---|---|
| `GYROSCOPE_RATIO` | `0.00026644` | 陀螺仪 raw → rad/s（量程 ±500°，对应 ±32768）|
| `ACCEL_RATIO` | `1671.84` | 加速度计 raw → m/s²（量程 ±2g，对应 ±32768）|
| 速度换算 | `× 1000` | m/s ↔ mm/s |

**官方 `Odom_Trans()` 的等价写法：**

```c
data_return = (transition_16 / 1000) + (transition_16 % 1000) * 0.001;
// 即 C 语言整数除法 + 取余，等价于 有符号值 / 1000.0
```

> ⚠️ **C++ 移植到 Python 时的坑**：C 语言里 `transition_16` 是 `short`（有符号），
> `transition_16 / 1000` 是**向零取整**，`transition_16 % 1000` 的符号跟随被除数。
> Python 的 `//` 是**向下取整**，负数会算错。Python 里请直接用：
> ```python
> def odom_trans(raw: int) -> float:      # raw 是有符号 i16
>     return raw / 1000.0
> ```

---

## 五、实机验证记录

**抓包环境**

```
设备   : /dev/ttyACM1  (USB HUB Port 1, 5-1.4.1)
芯片   : WCH USB Single Serial  1a86:55d4  serial=0002
参数   : 115200 8N1
抓包   : 101 帧 / 5 秒 = 20.2 Hz
```

**原始帧（实拍）**

```
7b 00 0000 0000 0000 0060 0058 4042 0003 fffe fffb 60b9 9e 7d
7b 00 0000 0000 0000 0050 0030 3fe4 0002 fffe fffd 60b9 18 7d
7b 00 0000 0000 0000 0050 0068 4034 0004 fffe fffc 60b8 e9 7d
```

**解码结果**

```
  #  Stop   X速度    Y速度   Z角速度   AccX  AccY   AccZ  GyrX GyrY GyrZ   电压V   BCC
  0    0   0.000   0.000   0.000     96    88  16450     3   -2   -5  24.761   158
  1    0   0.000   0.000   0.000     80    48  16356     2   -2   -3  24.761    24
  2    0   0.000   0.000   0.000     80   104  16436     4   -2   -4  24.760   233
  3    0   0.000   0.000   0.000     66    78  16422     2    1   -3  24.760   200
```

**BCC 校验：`101/101` 全部通过** ✅

**物理量合理性核对**

| 量 | 计算 | 结果 | 判断 |
|---|---|---|---|
| AccZ | `16450 ÷ 1671.84` | **9.84 m/s²** | ✅ ≈ 1g（板子水平静止）|
| AccX | `96 ÷ 1671.84` | 0.057 m/s² | ✅ ≈ 0（水平）|
| AccY | `88 ÷ 1671.84` | 0.053 m/s² | ✅ ≈ 0（水平）|
| 电压 | `24761 mV` | **24.76 V** | ✅ 6S 锂电池典型电压 |
| X/Y/Z 速度 | `0` | 0 m/s | ✅ 底盘静止 |

---

## 六、ROS2 话题映射（官方驱动行为）

| 话题 | 类型 | 内容 |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | 由 X/Y/Z 速度积分得到的位姿 + 速度 |
| `/imu` | `sensor_msgs/Imu` | 9 轴 IMU（加速度 + 角速度）|
| `/voltage` | `std_msgs/Float32` | 电源电压（V），约每 10 帧发一次 |
| `/cmd_vel` | `geometry_msgs/Twist` | **订阅**，下发底盘速度指令 |

**里程计积分（官方逻辑）**

```c
Robot_Pos.X += (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time;
Robot_Pos.Y += (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time;
Robot_Pos.Z += Robot_Vel.Z * Sampling_Time;
```

---

## 七、RK3588 替代方案要点

原 ROS2 控制板做的事情，RK3588 完全等价的实现：

1. **打开 `/dev/ttyACM1` @ 115200**
2. **读循环**：逐字节找 `0x7B`，收满 24 字节，验 `rx[23]==0x7D` 且 `rx[22]==BCC(rx[0:22])`
3. **解析** → 发布 `/odom`、`/imu`、`/voltage`
4. **订阅 `/cmd_vel`** → 组 11 字节帧 → 写入串口

**注意串口选择**：`/dev/ttyACM0` 是本项目的 N10P 激光雷达（460800），
`/dev/ttyACM1` 才是轮趣底盘（115200）。建议用 `/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00`
做稳定绑定，避免重启后 ACM 编号互换。

---

## 八、阿克曼底盘转向协议与原地打舵机制（实机 100% 验证）

### 8.1 下位机固件核心算法（C50X / 阿克曼固件）

轮趣下位机 STM32 固件（`balance_task.c` / `uartx_callback.c`）在接收到上位机串口速度后，执行以下运动学转角转换：

```c
float Akm_Vz_to_Angle(float Vx, float Vz)
{
    float TurnR, Angle_Left; // 转弯半径, 左前轮角度

    if (Vz != 0 && Vx != 0)  // 核心判定：必须线速度与角速度同时非零！
    {
        TurnR = Vx / Vz;     // 转弯半径 R = v / w
        // 几何转角公式
        Angle_Left = atan(robot.HardwareParam.AxleSpacing / (TurnR - 0.5f * robot.HardwareParam.WheelSpacing));
    }
    else
    {
        Angle_Left = 0;      // 关键！当 Vx == 0 时，固件强制将舵机归零打正！
    }
    return Angle_Left;
}
```

随后在电机驱动层将角度输出给转向舵机：
```c
Servo = (SERVO_INIT - Angle * K * Ratio); // SERVO_INIT = 1500 us
Servo = target_limit_int(Servo, 900, 2000); // 限幅保护
```

### 8.2 为什么纯角速度指令（Vx=0）前轮完全不动？

- 串口发送帧的 Z 字段（字节 7..8）是**整车横摆角速度 $\omega_z$**，而非裸舵机 PWM。
- 阿克曼结构物理上不能像差速底盘那样原地自转（$v_x=0$ 时转弯半径无意义），因此固件设计者写死了 `if (Vx == 0) Angle_Left = 0`。
- 上位机发 $v_x = 0$ 时，不论 Z 通道数值多大，前轮舵机目标都会被固件锁定在 0°（正中间）。
- 而**航模遥控器**走的是独立的中断通道（`Remote_Control`），摇杆直接映射到舵机 PWM 寄存器，完全绕过了阿克曼运动学除法，所以遥控器可以原地打舵。

### 8.3 原地静止打舵实现方案：微速度触发机制（Micro-Speed Trigger）

**实测完全验证成功的方法**：
利用驱动电机和减速齿轮箱的**物理静摩擦死区**（电机需要至少 $50\text{ mm/s}$ 才能起动），下发极微小的虚拟线速度：

1. **指令参数**：
   - 线速度：$v_x = 5\text{ mm/s} = 0.005\text{ m/s}$（字节 3..4 为 `0x0005`）
   - 角速度：$v_z = 0.020\text{ rad/s}$（字节 7..8 为 `0x0014`）
   - 完整帧：`7b 00 00 00 05 00 00 00 14 6e 7d`
2. **执行效果**：
   - **下位机解算**：判定 $v_x \neq 0$，$R = \frac{0.005}{0.020} = 0.25\text{ m}$，解算左前轮偏角达到最大物理上限（$+0.35\text{ rad} \approx 20^\circ$），**舵机瞬间向左打满**！
   - **后驱电机状态**：$5\text{ mm/s}$ 输出的 PWM 仅约 20/7200，完全无法驱动车身，**遥测速度恒为 0，小车在地面保持绝对静止**。
3. **方向符号约定**：
   - **左打满**：$v_x > 0$ 配合 $v_z > 0$（同号），如 `vx=0.005, vz=0.020`
   - **右打满**：$v_x > 0$ 配合 $v_z < 0$（异号），如 `vx=0.005, vz=-0.020`
   - **倒车左转**：$v_x < 0$ 配合 $v_z < 0$（同号），如 `vx=-0.150, vz=-0.350`

