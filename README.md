# RK3588 Radar / ROS2 Project

RK3588 雷达、底盘协议、Web UI 与设备端控制代码。

## 目录

- `radar_system/`：雷达、SLAM、RTK、目标检测与 Web 服务
- `wheeltec_protocol/`：轮趣底盘串口协议与控制代码
- `rk3588_dashboard/`：RK3588 设备面板主程序

## 实时建图与双端地图

新增 `live_map_web.py` 与 `live_map_gui.py`：同一份真实 `/map`、TF 位姿、轨迹和人体观测，同时显示在开发板屏幕与网页。默认网页 `:8088/map`，原相机/遥控保留在 `:8088/control`。支持 SLAM 建图、完整地图保存、已有地图 AMCL 定位及可选 OctoMap 实测 3D 显示。

先阅读 **[部署、操作与验收说明](docs/LIVE_MAPPING.md)**。原底盘需要加载更新后的 `wheeltec.yaml` 并重启；不要重复运行底盘、SLAM 或 TF 发布者。已有 systemd 独立 mapping 服务需停止后再让网页管理建图。

```bash
# 已有底盘/雷达/相机保持运行；两个终端分别启动，勿与现有服务重复。
bash radar_system/run_web.sh
bash radar_system/run_gui.sh
```

**范围说明：** 本次完成建图/定位与显示底座，以及地图坐标下的人体/候选跟随点接口；没有接通 Nav2 动态跟随运动闭环，没有用显示候选点直接驱动车辆。原 `person_follower.py` 保留。真实硬件、Qt 运行、外参、定位和制动仍需上车验收，不能把软件测试当作实车验证。
