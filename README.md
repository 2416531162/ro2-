# RK3588 Radar / ROS2 Project

RK3588 雷达、底盘协议、Web UI 与设备端控制代码。

## 目录

- `radar_system/`：雷达、SLAM、RTK、目标检测与 Web 服务
- `wheeltec_protocol/`：轮趣底盘串口协议与控制代码
- `rk3588_dashboard/`：RK3588 设备面板主程序

## 三维点云主视图与双端地图

默认主页和开发板屏幕已升级为 **可旋转的三维点云视图**：高度着色、距离环、车位跟随、轨迹、
人体标记、缩放/平移、深浅背景。真实深度观测在采样时刻经 TF 放入 map，屏幕与网页读取同一份二进制数据。
二维占据图仍是导航层，不能将单平面雷达墙线拉高冒充三维重建。

- [三维显示、数据源、部署与验证边界](docs/POINT_CLOUD_VIEW.md)
- [真实 2D SLAM、地图保存、AMCL 和里程计 TF](docs/LIVE_MAPPING.md)

```text
:8088/map      三维主视图
:8088/map2d    原二维地图与建图管理
:8088/control 原相机与遥控
```

```bash
# 现有雷达、相机、底盘和 SLAM/定位保持运行，不要重复启动同名服务。
bash radar_system/run_web.sh
bash radar_system/run_gui.sh
```

默认 Astra 三维累积需要先校准相机真实 TF，再在 web 进程设置 `SENSOR_TF_CALIBRATED=1`。
没有外参就明确等待，不填假高度。可以选择已有 PointCloud2 或 OctoMap 数据源。
默认是最多 60000 点、最近 45 秒的观测窗口，不是永久保留的完整三维模型。

**范围：** 显示和地图接口不直接控制运动。原 `person_follower.py` 与底盘算法保留，
Nav2 动态跟随闭环仍未接通；代码已更新不代表已经部署或通过实车、Qt、GPU 验收。
