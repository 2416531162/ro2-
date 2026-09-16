# 实时建图、定位与双端地图显示

## 本次交付范围

已实现真实 2D SLAM 启动、完整地图保存、已有地图 AMCL 定位启动、统一地图数据桥、网页与原生 PyQt5 地图视图，以及可选的真实 OctoMap 3D 显示。原相机、雷达、遥控与 `person_follower.py` 保留。

**还没有接通 Nav2 动态跟随的运动闭环。** `/person_tracker/pose` 与 `/follow/candidate_goal` 是接口准备：候选点尚未做地图可通行性、阿克曼转弯半径、人员安全距离的完整校验，不能直接连接到底盘。本次新增建图/显示节点均不发布 `/cmd_vel`。目标丢失不会新增盲目追赶行为。不要把 `YOLO person` 类别检测理解成身份识别。

## 1. 为什么要更改现有代码

旧 `joint_3d_mapping_node.py` 发布了静态 `map → base_link`，把机器人在世界中固定住；旧点云还把不同时间、不同传感器的数据拼接后统一写成当前时间。新版移除该静态 TF，逐个传感器保留真实 frame/stamp，并要求在采样时刻能查到 TF。

`wheeltec.yaml` 只增加/更改定位相关配置：`frame_id: odom`、`base_frame_id: base_link`、`publish_tf: true`。原速度、转向、串口、协议和刹停参数不变。`base_link` 统一为当前项目测量使用的后轴中心。

```text
Wheeltec /odom + odom→base_link
                     ↓
N10P /scan → mapping_scan.py → /mapping/scan → slam_toolbox → /map + map→odom
                     ↓                                  ↓
         仅去除物理车体内部自反射                 live_map_web.py :8088
         原始 /scan 不变                         ├─ 网页 /map
                                                  └─ 原生屏幕 live_map_gui.py

已保存地图 → map_server + AMCL → /map + map→odom
Astra + YOLO → 原始带时间戳人体观测 → 相机安装参数 + TF → map 坐标
已有 follower 确认且无歧义的目标 → /person_tracker/pose → /follow/candidate_goal
可选 OctoMap /occupied_cells_vis_array → 两端 3D 实测点云视图
```

同一时刻只能有一个 `map→odom` 发布者，SLAM 和 AMCL 不同时启动。也只能有一个 `odom→base_link` 发布者；已使用 robot_localization 时，不要同时启用 Wheeltec 的 TF。禁止再运行 `fake_slam_node.py` 作为实车定位来源。

## 2. 开发板依赖（ROS 2 Jazzy）

保留现有相机驱动、OpenCV、RKNN、底盘环境。缺少以下包时安装：

```bash
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-map-server ros-jazzy-sensor-msgs-py \
  python3-numpy python3-pyqt5
# 只有启用可选 3D 时才需要：
sudo apt install ros-jazzy-octomap-server
```

4GB 板默认先运行 2D 模式，不默认开启 OctoMap。显示地图限制至最长边 1024 像素，保守聚合不会抽样漏掉薄墙；真实导航地图不降采样。地图图像约 1 Hz 编码，位姿/扫描处理约 5 Hz；3D 显示最多 6000 个体素点。这是负载限制，不是实测帧率/内存保证。

## 3. 更新与启动

先让底盘停稳、关闭自动跟随。拉取代码；如果 systemd 实际运行 `/root/radar_system`，必须把修改部署到该目录，仅在另一个 checkout 执行 `git pull` 不会改变运行程序。可用下列命令确认部署路径：

```bash
systemctl cat rk3588-perception@web.service
```

现有底盘驱动需要重启并加载更新后的 `wheeltec_protocol/wheeltec.yaml`，才能发布正确 TF。不要启动第二个底盘串口实例。其定位配置等效于：

```text
--ros-args --params-file /实际项目目录/wheeltec_protocol/wheeltec.yaml
```

沿用现有 systemd 部署时，先停止旧的独立建图服务，再让网页管理建图进程；保留雷达、相机和已更新的底盘服务运行：

```bash
sudo systemctl stop rk3588-perception@mapping.service
sudo systemctl restart rk3588-perception@web.service rk3588-perception@gui.service
```

也可在仓库根目录分别运行两个终端（不要与上述同名服务重复启动）：

```bash
bash radar_system/run_web.sh
bash radar_system/run_gui.sh
```

网页地址（替换开发板 IP）：

```text
http://开发板IP:8088/map       实时地图（默认首页）
http://开发板IP:8088/control   原相机/遥控界面
```

屏幕默认打开“实时建模地图”标签，“相机 / 雷达感知”保留原界面。屏幕通过本机 `127.0.0.1:8088` 取地图，因此网页服务必须运行；失联会明确标记，只保留历史地图、不继续显示旧位置为实时位置。

## 4. 实際工作流

**首次建图：** 点击“开始建图”，进入原遥控界面，以低速观察性绕行。不要碰撞测试，不默认自动跟随。地图显示占据区域、已观测空闲、未知区域、机器人轮廓/朝向、轨迹、实时扫描；地图在两个显示端共享。

**保存地图：** 输入如 `office_01`，点击保存。调用 Nav2 `map_saver_cli` 保存完整 `/map`，不是屏幕 PNG。默认保存在 `radar_system/maps/`，可通过 `RO2_MAP_DIR` 指定。已有同名文件不覆盖；地图名拒绝路径穿越和 shell 字符。进程日志保存在同目录 `mapping.log`。

**再次进入：** 停止本界面启动的建图，选择已保存地图，点击“加载地图并定位”。在网页展开“AMCL 初始位置”，填实际地图 X、Y 和朝向。加载地图不等于自动定位成功；“定位 TF 在线”只代表 TF 新鲜度检查通过，**不是 AMCL 精度/收敛验收**。观察扫描是否与墙体对齐，再验证定位。

界面的停止按钮只管理本界面启动的建图/定位进程，不是底盘急停，也不终止外部 systemd 启动的进程。外部 SLAM/AMCL 已在运行时拒绝重复启动，防止两个地图/TF 源冲突。

## 5. 真实 3D（可选）

2D 地图和 3D 模型不是同一个东西。网页/屏幕的“3D 实测点云”仅显示 OctoMap 实际体素；没有体素时显示等待提示，不拉高 2D 墙线伪造建模。

必须先用实测值建立完整传感器 TF：雷达高度、相机高度、俯角、相机机体至 optical frame 的旋转，以及真实的父/子 frame。脚本 2D 默认雷达 `z=0` 只是平面建图约定，不是实测安装高度。相机驱动已有 optical TF 时不要再重复发布。

本项目记录的雷达前向偏移为 0.53 m、相机前向偏移 0.54 m、相机俯角 15°；高度不能猜测。`run_mapping.sh` 支持 `LIDAR_X_M/LIDAR_Y_M/LIDAR_HEIGHT_M/LIDAR_YAW_RAD`。外部 URDF 已发布 `base_link→laser` 时设置 `PUBLISH_LASER_TF=0`。人体显示的相机偏移/俯角也可通过 `live_map_display` ROS 参数调整。

校准后，在真实 SLAM/AMCL 已运行时启动：

```bash
SENSOR_TF_CALIBRATED=1 bash radar_system/run_3d_mapping.sh
```

默认使用 `/camera/depth_registered/image_raw` 与 `/camera/rgb/camera_info`。两者必须同 frame、同图像尺寸；非注册深度必须通过 `DEPTH_TOPIC/DEPTH_INFO_TOPIC` 选择正确配对。支持 16UC1 毫米、32FC1 米以及行步长/大小端。每帧保留原始时间戳，缺少 TF、过期数据、错误内参均不入图。传感器点云在原始传感器坐标系分别送入 OctoMap，保留正确射线起点。

注意：普通 OctoMap 累积不是带全局回环重投影的 3D SLAM。大的 SLAM 回环修正后可能需要重建 3D 图；它不替换 `/map`，也没有在此提交接入导航碰撞控制。2D LiDAR 看不到的悬空/低矮障碍不能只依赖 2D 地图防撞。

## 6. 跟随接口与安全边界

带有效深度、置信度和采集时间戳的人体观测，经相机安装换算与采样时刻 TF 投到 `map`。视觉候选都可显示，但只有与原 follower 已锁定目标无歧义匹配时才发布选定人和候选跟随点。没有凭“最近的一个人”自动切换跟随对象。

候选点距人保留 1.2 m 车头间距，并计入 0.67 m 前伸长度；太近时保持当前车位，不生成额外倒车目标。`/follow/candidate_goal` 明确是未经可通行性校验的候选接口，**尚未发送 `NavigateToPose` action**。下一阶段需要实现阿克曼可行路径、目标更新节流、失目标状态机以及唯一速度仲裁/安全输出后，才能接通 Nav2 自动绕行。不能让旧 Follower 与 Nav2 同时写 `/cmd_vel`，也不能从障碍层删除被跟随的人。

新页面只展示数据时不自动给底盘解锁；只有原遥控动作走原有显式控制流程。原 `person_follower.py` 的控制和安全算法本次没有更改。

## 7. 验证记录与上车检查

本次在无 ROS/无硬件环境执行：35 项纯 Python 回归 + 1 项 Chromium 离线合成数据 UI 测试，共 36 项通过；Python 编译与所有新增/修改 shell 脚本语法检查通过。UI 测试验证地图渲染、视图切换、缩放、保存错误/成功提示与断线提示。它没有真实调用 ROS；没有实测 Qt 窗口、RK3588 内存、SLAM 精度、相机外参和实车制动，也没有重跑仓库原有整套测试。

```bash
python -m pytest tests/test_live_map.py tests/test_live_map_browser.py -q
source /opt/ros/jazzy/setup.bash
ros2 topic echo /odom --once --field child_frame_id
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
ros2 topic info /map --verbose
```

上车确认：`/odom` 子坐标为 `base_link`、每条 TF 只有一个发布者、`/map` 只有一个真实地图源、车移动时扫描正确累积、墙体与扫描重合、停止相机/雷达后界面能标记失效、重启后地图能重新定位。完成这些检查后，再做受控低速跟随验证。

HTTP 8088 沿用原项目的局域网服务模型；新增建图写接口校验浏览器同源，但本次未补全登录认证/TLS，不要直接暴露公网。
