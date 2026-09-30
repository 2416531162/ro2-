# Gen5 RTK 划线车（车端）

正点原子 ATK-DLRK3588（Ubuntu 24.04、ROS 2 Jazzy）上的车端程序：两后轮差速（欧艾迪轮毂电机驱动器）+ 前万向轮，
UM982 双天线 RTK + RION HDA536T IMU 定位，按平板 app 下发的路线、定点直线、足球场规划画线。
平板 app 是独立工程（本机 `~/Desktop/app/follow-cockpit`），协议见 [APP_PROTOCOL.md](docs/APP_PROTOCOL.md)。

- `radar_system`：RTK 接收（`rtk_node`）、定位融合（`gnss_fusion`）、作业与跟踪（`line_node`、`line_mission`、
  `path_tracker`、`route_plan`、`field_planner`）、网页和 app 链路（`radar_web_server`、`app_link`）、作业记录。
- `oid_protocol`：欧艾迪驱动器（RS485 / CAN）；`rion_protocol`：HDA536T IMU（CAN）。
- `wheeltec_protocol`：控制权仲裁（手动 / 导航 / 急停）和底盘驱动公共部分。
- `robot_core`：车型配置（`robot_gen5.json`）、运动学。
- `deployment`：发布包（从 git 提交打包）、校验、systemd 服务激活与回滚。

```bash
# Mac：跑相关测试、从提交打发布包
python3 -m pytest -q tests/test_path_tracking.py tests/test_line_mission.py
python3 deployment/manage.py stage --git HEAD /tmp/rel
```

架构和数据流见 [ARCHITECTURE.md](docs/ARCHITECTURE.md)，开发、测试、部署规则见 [DEVELOPMENT.md](docs/DEVELOPMENT.md)，
板子与硬件接线见 [RK3588_GEN5.md](docs/RK3588_GEN5.md)，差速底盘见 [DIFFERENTIAL_DRIVE.md](docs/DIFFERENTIAL_DRIVE.md)，
定位融合见 [GNSS_FUSION.md](docs/GNSS_FUSION.md)，作业记录见 [JOB_RECORDS.md](docs/JOB_RECORDS.md)。
