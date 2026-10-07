# Gen5 RTK 划线车：现在的架构（2026-10-02）

本仓库现在的主业是 **Gen5 差速划线车**：正点原子 ATK-DLRK3588（Ubuntu 24.04、ROS 2 Jazzy）+ 两台欧艾迪轮毂电机驱动器
+ 前万向轮 + UM982 双天线 RTK + RION HDA536T IMU，平板 app（`~/Desktop/app/follow-cockpit`，独立仓库）操作。

本文描述当前本地代码。2026-10-02 的融合重规划只做本地测试与已有录包回放，未部署或操作车辆；
后续实车直线效果仍需独立喷头轨迹与闭环验证，见 [离线验证记录](../artifacts/imu-gnss-replan-20261002/README.md)。

## 1. 进程与数据流

车上部署配置 `rtk`（`deployment/manage.py` 的 `PROFILES`，只有这一套）起这些服务（`deployment/run_component.sh`）：

| 服务 | 程序 | 干什么 |
| --- | --- | --- |
| `rk3588-wheeltec`（chassis） | `oid_protocol/oid_driver.py`（差速车）；旧车 `wheeltec_protocol/wheeltec_driver.py` | 唯一的电机出口：控制权仲裁（`wheeltec_protocol/motion_authority.py`）、指令超时停车、驱动器心跳、轮速里程 `/odom` |
| `rk3588-perception@rtk` | `radar_system/rtk_node.py` | UM982 串口或平板串口转发（WiFi TCP 48622 / 蓝牙 RFCOMM 通道 22，`gnss_bridge.py`）→ `/gnss/raw`（JSON）、`/gnss/fix`、`/gnss/heading`；车端 CORS（`ntrip_config.py`） |
| `rk3588-perception@imu` | `rion_protocol/hda536t_imu.py` | CAN 上的 HDA536T → `/imu`、`/imu/data`（航向角、横滚俯仰） |
| `rk3588-perception@line` | `radar_system/line_node.py` | 融合（`gnss_fusion.py`）+ 作业状态机（`line_mission.py`）→ `/navigation/command`、`/line/status` |
| `rk3588-perception@web` | `radar_system/radar_web_server.py` | 平板 app 的 WebSocket（`/ws`，协议 `app_link.py`、[APP_PROTOCOL.md](APP_PROTOCOL.md)）；手动、IMU 定向直行（`heading_hold.py`）；转发 `/line/command`；车载屏幕 `/display`；网页手动遥控兜底页 `/`（`templates/index.html`：方向键、急停、状态） |
| `rk3588-perception@jobrec` | `radar_system/job_recorder.py` | 点开始作业就把原始话题录成 mcap（`~/rtk/jobs`，见 [JOB_RECORDS.md](JOB_RECORDS.md)） |

```
UM982 ─串口/平板转发─► rtk_node ─/gnss/raw─┐
HDA536T ─CAN─► hda536t_imu ─/imu/data────────┼─► line_node: gnss_fusion → LineMission ─/navigation/command─┐
欧艾迪驱动器 ◄─RS485/CAN─ oid_driver ─/odom──┘        (路线 / 定点画线 / 场地作业)                          │
      ▲                  ▲ motion_authority（手动 / 导航 / 急停 仲裁） ◄───────────────────────────────────┘
      └──────────────────┴─ /manual/command ◄─ radar_web_server ◄─ app（WebSocket）
```

话题大多是 `std_msgs/String` 里放 JSON（`/gnss/raw`、`/line/status`、`/line/command`、各 `…/status`）。

## 2. 控制：哪种任务走哪套跟踪

| 任务 | 入口 | 控制器 | 说明 |
| --- | --- | --- | --- |
| 路线（连线、圆）、场地作业（球场） | `line_mission` 的 route、`route_operations.py` | `path_tracker.SectionTracker` / `PathTracker` | 规划 `route_plan.py`（差速车：缓弯喷头拖后轴、急弯原地转 + 倒 1.2 m、往前 0.15 m/s 慢走 0.5 m 再加速 0.4 m 助跑）、场地 `field_planner.py`。曲率前馈 + 执行延迟 0.30 s 预估（段尾停车同；配驱动器速度环 Kp 2.7 / Ki 45）；原地转向差 5° 以上才转；起步慢走 0.3 m（倒车段 0.15 m、倒车 0.5 m/s）、起步 0.6 m 内锁车头 |
| 定点画线 A→B（差速车） | `line_mission` 的 start | `line_cascade.ImuLineTracker` | IMU 航向内环（`heading_hold.HeadingHold`）+ 连续融合喷嘴横偏外环；定位差时冻结横偏积分，暂停由融合许可与喷头停车预算决定 |
| 定点画线 A→B（旧阿克曼车） | 同上 | `line_tracker.LineTracker` | 旧车用 |
| 手动页「IMU 直行」 | `radar_web_server` | `heading_hold.HeadingHold` | 只用 IMU，不用 RTK |

差速车参数集中在 `path_tracker.differential_config()`（执行延迟 0.30 s，与驱动器速度环参数配套、弯道横向加速度 0.20 m/s²、起步慢走 / 锁车头）；原地转向 0.8 rad/s、1.0 rad/s²（`robot_core/robot_gen5.json` 的 `drive`。2026-10-01 试过 1.0、2.0：驱动器速度环滞后约 0.4 s 后冲过头到下发的 2–3 倍，车实际转到 1.94 rad/s、顿住回弹，转完离线 5 cm，退回）。

**已知结构问题（待整理，见第 5 节）**：沿线控制有 4 套；路线直线段没用定点画线那套 IMU 内环，起步、甩头这类处理要分别写。

## 3. 定位融合（`gnss_fusion.py`）

全局四状态 `[x, y, yaw, bias]` 由轮速和 IMU 航向角持续预测，可信 RTK 修位置、可信双天线修航向。
Gen5 默认 `continuous_output=True`：另维护连续后轴控制输出，控制器读取 `FusedPose.x/y/yaw`；
`correction_xy/yaw` 报告全局估计减控制输出的待应用差。`sigma` 与协方差仍是全局随机不确定度，
待应用差是实际校准残差，不能藏进协方差或因固定解恢复而忽略。
Gen5 默认值由 `line_node.load_fusion_config()` 补齐，车上 `fusion.json` 可覆盖，详见 [GNSS_FUSION.md](GNSS_FUSION.md)。

- **航向校准**：报告精度下限 0.1°，计同步误差与 0.25 s 观测相关性，通过增大观测方差限制 KF 增益。
  直行增益时间常数 2 s，明显转弯、转后约 1 s、有证据的恢复或 IMU 失效时为 0.15 s；转弯判据独立为 3°/s。
  Gen5 不再因测向稳定 0.5 s 就硬重定；突变必须结合 IMU 转角、合格位移航迹或 IMU 失效证据。
  老阿克曼仍保留原 0.5 s 稳定硬重定路径。转角刻度余量按整段相关转角累计，不能因采样更密而消失。
- **有效 IMU 与漂移**：只有通过尖峰检查的运动输入刷新 `imu_valid_t`；更新角度参考不等于输入有效。
  IMU 失效时仍可用双天线校准和差速轮速预测。`angle` bias 是内算航向漂移率，`rate` bias 才是原始陀螺零偏；
  转弯、角度重接、连续输出尚未对齐及恢复冻结窗口不学习漂移，Gen5 航迹/BESTVEL 也不把角度偏差学成 bias。
- **位置、测向分别判质量**：双天线固定位置与测向基线不一致时先不信位置；互检偏差超过 2 cm 给位置连续降权，
  确认超过 10 cm 的异常进入推算维持（转动时门限另放宽）。测向长度不在 0.35–0.45 m 时只停用该帧测向。
  几何偏差当前只对位置加权，未承诺航向连续几何加权；位置不可信并不必然意味着航向不可信，互检通过也不能排除共模漂移。
- **连续校正**：常态位置时间常数 0.25 s；输出航向时间常数 0.20 s、校正限速 1°/s。
  掉线恢复时沿车头/横向校正各限 4 cm/s，待应用位置差小于 1 cm 后返回常态平滑。
  大于 0.3 m 的恢复位置矛盾先标记重定位、停车，轮速确认停稳后才直接对齐输出；行驶中不一步追过去。
- **失联与停车预算**：可信固定解中断 0.15 s 后开始完整预算，0.5 s 后报告 `rtk_hold`，5 s 只是兜底上限，
  不保证能走满。`rtk_hold_budget.py` 将全局协方差按全局航向投影到实际喷头，另加精确杆臂旋转后的待应用校正；
  停车距离还计未消除航向差的横向影响，覆盖已发指令、执行延迟及制动。预算达 65% 限速，预计停稳越界或将超时则提前暂停。
  固定解恢复但仍有校正时继续记账，只预测当前随机误差与校正运动余量，不延续旧 5 s 倒计时或假设整个制动过程失联。
- **开始、继续作业须重新确认**：固定解新鲜、无维持/不可信/接回/重置状态，0.5 s 内固定解与控制位姿的天线推算差不超过 5 cm，
  校准差及喷头预算安全。实际大残差阻止起步；小校准差或 `gnss_alignment` 提示本身不会锁住自动恢复。
  运行中未判不可信的固定解与控制位姿差超过 0.3 m，仍用 `position_mismatch` 暂停。
- **测量时间与回放**：积压 GNSS 按真实测量时刻处理；1 s 历史窗内回放，太旧直接丢弃。
  连续输出、冲突、累计统计与质量证据参与快照；质量证据随事件保存，不能用未来互检给旧测量背书。

`/line/status.hold_budget` 扩展校正预算字段，`calibration()` 增加 `heading_filter` 与 `alignment`；
原消息类型和预算原因代码保留，App 添加中文解释。离线录包只有原始控制指令与融合数值，
没有独立喷头真值，也未验证新控制闭环，不能将数值更平顺写成实车走线精度已改善。

## 4. 配置都在哪

| 内容 | 位置 | 进发布包 |
| --- | --- | --- |
| 车型、几何、手动档、驱动类型 | `robot_core/robot_gen5.json`（`RK3588_ROBOT_CONFIG` 指定），旧车 `robot.json`；读取校验 `robot_core/config.py` | 是 |
| 电机驱动器（总线、极对数 20、轮半径 0.194 m、加减速） | `oid_protocol/oid.yaml` | 是 |
| IMU | `rion_protocol/hda536t.yaml` | 是 |
| 跟踪参数（差速车） | `radar_system/path_tracker.py` 的 `differential_config()` | 是（代码） |
| 融合标定（天线杆臂、天线高度） | 车上 `~/rtk/fusion.json`；Gen5 专用默认值在 `line_node.load_fusion_config()` | 否（车上文件） |
| CORS 账号 | 车上 `~/rtk/ntrip.env`（app 可写） | 否 |
| 运行环境（ROS、配置路径） | 车上 `/etc/rk3588/runtime.env`，模板 `deployment/runtime-rk3588.env.example` | 否 |

平板 app 里抄了一部分常数（如 `RouteGeometry.ENTRY_LEAD_M`、`PIVOT_TOL_RAD`），靠协议样本（`tests/fixtures/app_protocol/`，
`radar_system/tools/export_app_fixtures.py` 生成）和 app 的 `RouteGeometryTest`、`PivotParityTest` 对照。改了规划常数要同步两边。

## 5. 待整理（按收益排序）

1. **统一 Gen5 沿线控制**：已实现共享航向内核与前进长直线双环候选；闭环仿真尚未同时改善横偏和小摆，路线候选默认关闭。详见 [长直线控制试验](STRAIGHT_CONTROL_EXPERIMENT.md)。
2. **配置收拢**：跟踪、融合的 Gen5 参数进车型配置；app 的常数改为从车上读（hello / route_limits），不再各抄一份。
3. **拆大文件**：`gnss_fusion.py`（约 1200 行、55 个参数）、`line_mission.py`（约 750 行：状态机、规划调用、诊断、状态上报）。
4. **标准消息**：`/line/status` 等 JSON 字符串话题换成自定义 ROS 2 消息。

开发与部署规则见 [DEVELOPMENT.md](DEVELOPMENT.md)。
