# 三维点云主视图（参考三维建图导航显示）

## 交付与范围

这次把默认显示从二维占据图/固定斜视稀疏点，改成 **可自由旋转的三维点云视图**。
网页为 WebGL 点渲染并提供软件降级；开发板屏幕为原生 PyQt5 + NumPy 透视/深度缓冲渲染。
两端读取同一个地图坐标、同一个二进制点云版本。保留原二维地图管理、相机和遥控。

这是显示和观测累积升级，不是新增完整三维 SLAM，也没有接通 Nav2 动态跟随的底盘运动闭环。
默认深度模式显示 **最近 45 秒、最多 60000 点的局部观测**，不是永远保留的全屋模型。
人员或家具移动可留下短时残影；不能把这层直接用于导航碰撞判断。需要持续体素占据模型时，
可选择已有 OctoMap 快照作为数据源，而不是把二维墙线拉高。

## 已实现

- 默认三维视角，俯视、低视角、全图、车位跟随；左键旋转、右键/Shift 平移、滚轮缩放。
- 按真实 Z 高度或距机器人距离着色，显示标尺；仅 PointCloud2 确实含 intensity 时提供强度着色。
- 高度范围筛选、点大小、距离环（1/2/3/5 m）、米制辅助网格、深/浅背景。
- 车体实际平面轮廓与朝向、行驶轨迹、规划路径（已有时）、人体标记、未经导航校验的候选点。
- N10P 扫描只作为二维投影叠加，绝不从单平面扫描捏造三维墙面。
- 采样时刻 TF、重复/过期帧拒绝、内参与深度配对检查、真实外参确认。
- 地图会话 epoch + 版本绑定；切图时拒收在途旧帧/旧二进制；显著回环/重定位变化时清掉历史观测重建。
- 来源、点数、年龄、错误、历史状态明确展示。断线隐藏实时车位/目标，保留的点云明确标为历史。

## 新代码入口

```text
cloud_scene.py         点格式解析、深度投影、有限体素历史、二进制包、原生透视/深度缓冲
live_cloud_node.py     只读 ROS 显示桥，继承现有 LiveMapNode
cloud_web.py           8088 新主界面和二进制传输；原 HTTP 控制逻辑复用
cloud_gui.py           原生三维屏幕，独立工作线程；另保留 2D 与相机标签
static/cloud_viewer.js WebGL 1 实测点云 + 软件降级，无 CDN/第三方 JS 下载
static/cloud_app.js    状态/版本同步、页面操作、传输与断线处理
```

`run_web.sh` / `run_gui.sh` 已指向新入口；`start_component.sh` 无需改动。
原 `live_map_*`、底盘和跟随算法保留。本次没有改速度、制动、串口、雷达驱动或跟随安全参数。

## 部署

先停稳车并停止跟随。拉取并部署到服务真正运行的目录，不要只更新另一份 checkout。

```bash
git pull --ff-only
systemctl cat rk3588-perception@web.service
# 更新到实际运行目录后：
sudo systemctl restart rk3588-perception@web.service rk3588-perception@gui.service
```

网页：

```text
http://开发板IP:8088/map       新三维点云主界面
http://开发板IP:8088/map2d     原二维导航地图与建图管理
http://开发板IP:8088/control   原相机/遥控
```

屏幕默认打开“三维建模 / 点云地图”，另有“二维导航 / 建图管理”“相机 / 雷达感知”标签。
网页服务需要先启动，原生屏幕通过本机 8088 获取同源数据，不额外启动 SLAM 或浏览器。
保留原 [建图、AMCL 与 TF 部署说明](LIVE_MAPPING.md)，确保底盘里程计和单一 map→odom 正常。

## Astra 深度模式（默认）

默认自动订阅注册深度和对应 CameraInfo，**无需启动 OctoMap** 就能做有限三维观测累积。
必须先有真实地图定位和校准后的 `map→odom→base_link→相机 optical frame`。

```bash
# 仅在真实传感器外参已经标定并正确发布 TF 后：
SENSOR_TF_CALIBRATED=1 bash radar_system/run_web.sh
bash radar_system/run_gui.sh
```

环境变量设为 1 只是确认，不会创建 TF，也不能替代实测标定。没有标定时界面会等待，不使用猜测高度。
systemd 启动时需要将此变量加到 web 服务的 Environment 配置，终端里的 export 不会自动传给已有服务。
默认相机话题：

```text
DEPTH_TOPIC=/camera/depth_registered/image_raw
DEPTH_INFO_TOPIC=/camera/rgb/camera_info
```

两者需同 frame、同尺寸且为正确注册配对。支持 16UC1/mono16 毫米和 32FC1 米、大小端、行步长。
缺少相机高度/俯角、错误 frame、内参不匹配、过期观测都会提示；不能靠改颜色消除数据问题。
N10P 只能提供扫描平面，参考商品展示与当前传感器覆盖范围不能等同。

## 其他真实三维数据源

选择一个来源，不能把多个独立地图混合累加：

```bash
# 上游已发布、具备正确 frame 与采样时间戳的三维 PointCloud2
RO2_CLOUD_SOURCE=pointcloud RO2_CLOUD_TOPIC=/mapping/depth_points \
  bash radar_system/run_web.sh

# 使用上一版已支持的 OctoMap 完整体素快照
RO2_CLOUD_SOURCE=octomap bash radar_system/run_web.sh
```

PointCloud2 源每帧采样最多 8000 点，再按 5 cm 体素选择真实点；不把体素中心当测量结果。
发布者需保证时间戳、坐标、运动补偿正确；本显示桥不是多线雷达去畸变或 LiDAR-IMU 里程计。
OctoMap 模式替换整份快照（包括清空），保持体素历史，但显示最多 60000 个体素点。
OctoMap 上游仍需按 LIVE_MAPPING.md 启动并标定，不由显示页面自动虚构。

## 传输和计算预算

`/api/live_map` 只带三维元数据，实际点云通过：

```text
GET /api/live_map/scene.bin?epoch=<会话>&v=<版本>
```

负载为 N×4 小端 float32：x/y/z/intensity；无强度为 -1。支持 HTTP gzip，缓存最近三个不可变版本。
错误会话/过期版本返回 409，前端重新取元数据。60k 点原始数据上限为 960000 字节/版本；不是每次 UI 轮询重复发 JSON 点列表。
三维累积处理约 5 Hz，打包最多约 2 Hz；这些是调度参数，不是实测整板吞吐保证。

网页支持 GPU WebGL；GPU 不可用时明确标识软件预览并限 10000 点。原生屏幕无新增浏览器依赖，
用 NumPy 工作线程做透视/深度缓冲，最多约 1280×900 像素栅格化，再由 Qt 绘制图像和标注。
原生后端是 CPU，不应宣传成 Mali GPU 加速。实际 4GB 板负载和流畅度需硬件验收。

## 验证记录

本次在无 ROS、无 PyQt5 的测试主机上：

- 38 项解析、深度单位、TF 数学、体素预算、时间过期、版本/二进制、透视/深度缓冲测试通过。
- 10 项 ROS/TF **替身**逻辑测试通过，包含采样时刻变换、缺外参、过期帧、重定位、切图在途旧帧。
- 2 项 Chromium 实际页面/交互测试通过，使用明确标记的合成数据和内存 API 替身；该环境走 CPU 降级渲染。
- 1 项 WebGL 驱动测试跳过：测试主机无法创建 WebGL 上下文，未宣称 GPU 渲染通过。
- Python 编译、JS 和 shell 语法检查通过；未重跑仓库原有全部测试。

没有 ROS executor 实际联调、PyQt 窗口运行、开发板性能/外参/地图精度或车辆验收。
软件测试截图和离线演示均标注合成数据，不能当作实车重建成果。

```bash
python -m pytest tests/test_cloud_scene.py tests/test_cloud_node.py tests/test_cloud_browser.py -q
# 自己的桌面有 PyQt5 时可单独打开 3D 屏幕，不加载相机/ROS UI：
python radar_system/cloud_gui.py --map-only --windowed
```

上线检查：核对深度 frame 和 CameraInfo，移动机器人时墙面应保持对齐；断开相机应显示历史/过期；
切换地图不能残留旧点云。大回环会清空有限历史重建；它不会假称能对全部历史做非刚性回环重投影。
这次未新增 Nav2 自动运动，候选跟随点仍不可直接交给底盘。

8088 仍按原项目局域网部署，未新增公网认证/TLS，勿直接暴露公网。
