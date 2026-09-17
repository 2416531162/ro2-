# RK3588 雷达与摄像头人体跟踪

运行链路收敛为 **Astra RGB-D → YOLOv8n-pose → 人体锁定 → N10P 雷达接力 / 路径检查 → 底盘控制**。

- 摄像头：RK3588 NPU 人体姿态检测，人体框、17 个关键点、深度测距与骨架画面。
- 雷达：实时扫描、人体接力跟踪、障碍物检查。
- 控制：车体扫掠、刹车包络、限量脱困、手动接管；NumPy 批量路径检查。
- 显示：网页雷达与跟随遥测，板载屏幕相机骨架 / RGB / 深度与实时雷达。

SLAM、地图保存/定位、三维重建、点云累积、RTK、Foxglove、人脸及通用物体检测已移除。
RGB-D 测距与车体几何仍用于跟踪和碰撞检查。

```bash
# 开发板：沿用现有 rk3588-perception@.service 和 ROS 2 环境
bash radar_system/start_all.sh
# 跟随由网页明确启动；也可先进行无运动输出检查
bash radar_system/run_follower.sh --dry-run
```

网页端口 `8088`，主页 `/`。`start_all.sh` 会停止并禁用旧的建图、RTK 和 Foxglove 服务。
底盘串口驱动继续由现有 wheeltec 服务管理。

- [架构、姿态权重与验证说明](docs/TRACKING.md)
- [运动参数与标定](docs/TUNING.md)
- [跟随脱困行为](docs/FOLLOWER_RECOVERY.md)

本地代码、模型转换及合成场景测试不等于已部署或通过实车验收。
