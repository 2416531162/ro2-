# Gen5 RTK 划线车：现在的架构（2026-09-30）

本仓库现在的主业是 **Gen5 差速划线车**：正点原子 ATK-DLRK3588（Ubuntu 24.04、ROS 2 Jazzy）+ 两台欧艾迪轮毂电机驱动器
+ 前万向轮 + UM982 双天线 RTK + RION HDA536T IMU，平板 app（`~/Desktop/app/follow-cockpit`，独立仓库）操作。

## 1. 进程与数据流

车上部署配置 `rtk`（`deployment/manage.py` 的 `PROFILES`，只有这一套）起这些服务（`deployment/run_component.sh`）：

| 服务 | 程序 | 干什么 |
| --- | --- | --- |
| `rk3588-wheeltec`（chassis） | `oid_protocol/oid_driver.py`（差速车）；旧车 `wheeltec_protocol/wheeltec_driver.py` | 唯一的电机出口：控制权仲裁（`wheeltec_protocol/motion_authority.py`）、指令超时停车、驱动器心跳、轮速里程 `/odom` |
| `rk3588-perception@rtk` | `radar_system/rtk_node.py` | UM982 串口或平板串口转发（TCP 48622，`gnss_bridge.py`）→ `/gnss/raw`（JSON）、`/gnss/fix`、`/gnss/heading`；车端 CORS（`ntrip_config.py`） |
| `rk3588-perception@imu` | `rion_protocol/hda536t_imu.py` | CAN 上的 HDA536T → `/imu`、`/imu/data`（航向角、横滚俯仰） |
| `rk3588-perception@line` | `radar_system/line_node.py` | 融合（`gnss_fusion.py`）+ 作业状态机（`line_mission.py`）→ `/navigation/command`、`/line/status` |
| `rk3588-perception@web` | `radar_system/radar_web_server.py` | 平板 app 的 WebSocket（`/ws`，协议 `app_link.py`、[APP_PROTOCOL.md](APP_PROTOCOL.md)）；手动、IMU 定向直行（`heading_hold.py`）；转发 `/line/command`；车载屏幕 `/display`；网页手动遥控兜底页 `/`（`templates/index.html`：方向键、急停、状态） |
| `rk3588-perception@jobrec` | `radar_system/job_recorder.py` | 点开始作业就把原始话题录成 mcap（`~/rtk/jobs`，见 [JOB_RECORDS.md](JOB_RECORDS.md)） |

```
UM982 ─串口/平板转发─► rtk_node ─/gnss/raw─┐
HDA536T ─CAN─► hda536t_imu ─/imu─────────────┼─► line_node: gnss_fusion → LineMission ─/navigation/command─┐
欧艾迪驱动器 ◄─RS485/CAN─ oid_driver ─/odom──┘        (路线 / 定点画线 / 场地作业)                          │
      ▲                  ▲ motion_authority（手动 / 导航 / 急停 仲裁） ◄───────────────────────────────────┘
      └──────────────────┴─ /manual/command ◄─ radar_web_server ◄─ app（WebSocket）
```

话题大多是 `std_msgs/String` 里放 JSON（`/gnss/raw`、`/line/status`、`/line/command`、各 `…/status`）。

## 2. 控制：哪种任务走哪套跟踪

| 任务 | 入口 | 控制器 | 说明 |
| --- | --- | --- | --- |
| 路线（连线、圆）、场地作业（球场） | `line_mission` 的 route、`route_operations.py` | `path_tracker.SectionTracker` / `PathTracker` | 规划 `route_plan.py`（差速车：缓弯喷头拖后轴、急弯原地转 + 倒 0.5 m）、场地 `field_planner.py`。曲率前馈 + 执行延迟 0.44 s 预估；原地转向差 5° 以上才转；起步慢走 0.3 m、起步 0.6 m 内锁车头 |
| 定点画线 A→B（差速车） | `line_mission` 的 start | `line_cascade.ImuLineTracker` | IMU 航向内环（`heading_hold.HeadingHold`）+ RTK 喷嘴横偏外环；航向跳变按 IMU 维持 5 s |
| 定点画线 A→B（旧阿克曼车） | 同上 | `line_tracker.LineTracker` | 旧车用 |
| 手动页「IMU 直行」 | `radar_web_server` | `heading_hold.HeadingHold` | 只用 IMU，不用 RTK |

差速车参数集中在 `path_tracker.differential_config()`（执行延迟 0.44 s、弯道横向加速度 0.12 m/s²、起步慢走 / 锁车头）。

**已知结构问题（待整理，见第 5 节）**：沿线控制有 4 套；路线直线段没用定点画线那套 IMU 内环，起步、甩头这类处理要分别写。

## 3. 定位融合（`gnss_fusion.py`）

轮速 + IMU 航向角推算，RTK 固定解修位置、双天线测向修航向（航向以双天线为主）。Gen5 的专用设置在
`line_node.load_fusion_config()` 里按车型补（航向角输入、`rtk_hold_s` 5 s 等），详见 [GNSS_FUSION.md](GNSS_FUSION.md)。

- 双天线位置互检（`DualAntennaCheck`）：两根天线的固定解之差对不上测向基线（> 10 cm）就不信 RTK，按推算最多走 5 s 再停车，恢复后自动继续。
- 掉信号（固定解 0.5 s 没来：平板中转断流、浮点、没解）不推算，原地停，恢复后自动继续（2026-10-01 用户规则）。中转断流后积压的数据按真实测量时刻处理（`um982.GnssClock` 不再把迟到的数据当成刚测到），太旧的融合直接丢掉。
- 侧向过程噪声 `cross_walk_m_sqrt_s` 0.0015：融合侧向约 1.5 s 跟上 RTK。调大能少约 0.3–0.6 cm 慢摆，但主天线单独侧漂时车会跟着偏（2026-09-30 回放评估后保持不变）。

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

1. **统一 Gen5 沿线控制**：路线直线段也用 IMU 航向内环 + RTK 外环，起步 / 甩头 / 跳变只写一遍（会改变车的行为，需实车验证）。
2. **配置收拢**：跟踪、融合的 Gen5 参数进车型配置；app 的常数改为从车上读（hello / route_limits），不再各抄一份。
3. **拆大文件**：`gnss_fusion.py`（约 1200 行、55 个参数）、`line_mission.py`（约 750 行：状态机、规划调用、诊断、状态上报）。
4. **标准消息**：`/line/status` 等 JSON 字符串话题换成自定义 ROS 2 消息。

开发与部署规则见 [DEVELOPMENT.md](DEVELOPMENT.md)。
