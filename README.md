# RK3588 摄像头与雷达人体跟随

当前链路：**Astra RGB-D → YOLOv8n-pose＋N10P 雷达 → 人体锁定与局部跟随 → 控制权仲裁 → 驱动防撞 → 底盘**。

- `robot_core`：统一机器人标定、运动学、观测契约和本地位姿历史。
- `radar_system`：ROS 适配与纯跟随引擎分离；相机/雷达关联、扫掠检查、有限脱困、网页及板载显示。
- `wheeltec_protocol`：唯一串口出口，统一手动/跟随/导航控制权、时效检查与故障停车。
- `deployment`：整套版本打包、内容校验、服务切换及回滚；服务启动后等待显式选择跟随。

当前跟随使用统一 `/odom`，GNSS 融合定位输入和导航控制入口已留好。卫星驱动、融合定位、航点规划尚未实现。之前移除的 SLAM、点云累积、地图管理、RTK/CORS 服务不在当前运行链路中。

```bash
# 本机：测试、生成发布包
python3 -m pytest tests -q
python3 deployment/manage.py stage artifacts/follow-release
python3 deployment/manage.py verify artifacts/follow-release

# 开发板：安装发布包后启动已有服务，仍需网页点击跟随
bash radar_system/start_all.sh headless
```

网页默认端口 `8088`，主页 `/`。完整部署命令、控制接口和 GNSS 接入约束见 [架构说明](docs/ARCHITECTURE_ROADMAP.md)。不能只替换一个目录；新栈需要整套发布。

- [视觉模型与跟踪说明](docs/TRACKING.md)
- [统一配置与标定](docs/TUNING.md)
- [跟随脱困行为](docs/FOLLOWER_RECOVERY.md)
- [驱动接口](wheeltec_protocol/README.md)

本地代码和合成测试通过不等于已部署或通过实车验收。
