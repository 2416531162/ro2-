# 安卓 app 连接协议 v1

车端实现：`radar_system/app_link.py`（编码/校验）＋ `radar_system/radar_web_server.py`（端口 8088）。
安卓端实现：独立工程，不在本仓库里（本机路径 `~/Desktop/app/follow-cockpit`）。协议版本不兼容时 `hello.v` 加一。

协议样例的基准在 `tests/fixtures/app_protocol/`，`tests/test_app_link.py` 检查它与车端编码器一致。改了字段后，用 `python3 radar_system/tools/export_app_fixtures.py --out <安卓工程>/app/src/test/resources` 重新生成并同步到安卓工程，安卓端的单元测试解析的就是这份样例。

## 找到车

| 方式 | 说明 |
|---|---|
| UDP 广播 | 车每秒向各网卡广播地址发 `{"magic":"rk3588-follow","v":1,"name":…,"port":8088,"profile":…}` 到 UDP **48620** |
| 车载热点 | 手机连车的热点时，手机网关就是车。app 先 `GET http://网关:8088/app/beacon`（内容与 UDP 广播相同），`magic` 对上才列出，家里路由器之类的网关不会误列成车 |
| 手动 | 输入 IP，记住上次成功的地址 |

车名来自环境变量 `RK3588_CAR_NAME`，缺省为主机名。

## 通道

| 通道 | 用途 |
|---|---|
| `ws://车:8088/ws` | 双向 JSON 文本帧：下行 `hello` 一次＋`tel` 10 Hz；上行指令 |
| `http://车:8088/video.mjpg` | `multipart/x-mixed-replace; boundary=frame`，JPEG 最多 12 FPS（`RK3588_APP_VIDEO_FPS`），每帧带 `X-Stamp`（采集 ROS 时间）。没人看时车端不解码 |
| `POST /api/motion/stop` | 急停的第二条路：WebSocket 卡住时 app 同时走 HTTP |
| `GET /app/beacon` | 返回与 UDP 发现广播相同的 JSON，只读；app 探测网关是不是车 |
| `GET /api/rtk/cors` | CORS 配置摘要、连接开关能力及运行状态，不返回密码 |
| `POST /api/rtk/cors` | 保存账号，或以 `action: disconnect / connect` 断开／重新连接车端 CORS |

## 下行

`hello`：`v`、`name`、`session`、`video` 路径、`car`（`front_m`/`rear_m`/`half_width_m`/`wheelbase_m`/`track_m`/`max_steer_deg`/`min_turn_radius_m`/`drive`（`ackermann` 前轮转向，`differential` 两后轮差速、能原地转，此时 `min_turn_radius_m` 是跟踪圆弧的最小半径、没有前轮转角）/`nozzle_x_m` 喷头在后轴前多少米/雷达与相机安装位置，车体系原点后轴中心、x 前 y 左）、`camera`（`width`/`height`/`fx`/`cx`）、`speed_tiers`（手动三档 m/s）。

`tel`（10 Hz）：

| 字段 | 内容 |
|---|---|
| `seq`, `t` | 序号、车端单调时钟 |
| `link` | `lidar_hz`、`cam_fps`、`chassis_age_s`、`chassis_ok` |
| `battery` | `v`、`pct` |
| `motion` | 仲裁 `mode`（IDLE/MANUAL/FOLLOW/NAVIGATION/FAULT/ESTOP）、`healthy`、`fresh`、`reason`、`faults`、`waiting_stationary` |
| `drive` | 实测 `speed`（m/s）、`yaw_rate`、`manual` 是否手动中、`steer_deg` |
| `follow` | `running`、`stale`、`state`/`reason`（跟随状态机）、`phase`（掉头阶段）、`plan`（掉头方案：`direction`、`turn_dir`、`stroke_m`、`eta_s`、`segments` [[方向, 米]…]）、`aim`（预瞄点）、`cap`、`clearance`、`aeb`、`tracks`、`target` |
| `follow.target` | `id`、车体系 `x`/`y`、`gap`（车头到人）、`bearing_deg`、`speed`、`in_camera`、`coasting` |
| `persons` | 画面里的人：`box` [x1,y1,x2,y2]（像素）、`conf`、`dist`、`bearing`（相机水平方位，左正）、`locked`（是否跟随目标） |
| `scan` | `a0`、`da`（弧度）、`r`：uint16 厘米小端 base64，0 表示无效 |
| `rtk` | RTK 接收机（`rtk_node`）：`fix`（FIXED/FLOAT/SINGLE/NONE，与融合的取舍一致：FIXED 只指 `NARROW_INT`，`WIDE_INT`/`L1_INT` 归 FLOAT；解算状态不是 `SOL_COMPUTED` 按 NONE）、`sats`、`diff_age`（秒，无差分为 null）、`sigma_h`（米）、`heading_ok`（双天线测向整周固定，且两根天线的位置都是 RTK 固定解——两根都固定就信，不比较测向长度、不写死也不实测天线间距；两天线平面距离 <0.10 m 视为室内转发器同一路信号，不算）、`baseline_m`（测向长度）、`receiver_ok`、`ntrip`（`configured`/`connected`/`error`/`age_s`）、`antennas`（`ant1` 主天线 BESTPOS、`ant2` 副天线 BESTNAVH：`sol`/`fix`/`sats_used`/`sats_tracked`/`age_s`；`ant2` 另有 `offset_m` 两天线固定解的平面实时距离（东、北，不算高差）、`consistent` 保留为 null）、`heading`（测向解：`sol`/`fix`/`sats_used`/`sats_tracked`/`length_m`/`pitch_deg`/`sigma_deg`/`ref_m` 两天线固定解的平面实时距离/`valid`/`yaw_deg` 双天线测出的车头方向，度，东为 0、逆时针为正，已扣安装偏差，与 `/gnss/heading` 同一换算；只在 `valid` 为 true 时有值，否则 null，app 不得显示无效测向的角度） |
| `line` | RTK 走直线（`line_node`）：`phase`（IDLE/SELECTING/RUNNING）、`message`、`cruise`、`motion_mode`、`points`（`A`/`B` 本地 ENU 米，x 东 y 北）、`pose`（`x`/`y`/`yaw_deg`（东为 0 逆时针）/`speed`/`quality`/`usable`/`sigma_xy`/`sigma_yaw_deg`/`reasons`）、`track`（`state`/`reason`/`e` 横偏米（行进方向的左侧为正）/`heading_err_deg`/`s` 沿线米/`length`/`v_cmd`（倒车为负）/`rms`/`max`/`gear` forward 或 reverse/`section` 第几段/`sections` 共几段/`switching` 在换向点等车停稳/`pivoting` 差速车正在原地转向）、`last_result`（上一条 line/route 指令的执行结果，按 `id` 对应）；走路线另见下一行 |
| `line`（路线） | `task`（line/route，当前或最近一次任务）、`origin`（本地坐标原点的 [纬度, 经度]，融合还没有有效定位时为 null；`line_node` 每次启动取第一次有效定位，原点会变，路线存经纬度所以照样对得上）、`antenna`（主天线最近一次定位：`x`/`y` 本地米、`sol`、`sigma` 米；超过 2 s 没更新为 null。融合位姿不可用时 app 用它画车的位置）、`route`（车上存的路线，换算到当前本地坐标：`kind` points/circle、`points` [[x,y]…]、`closed`、`center`、`radius`、`clockwise`、`laps`、`turn_radius`、`cruise`、`rev` 版本号、`length` 路线长度米（不含接入段；连线路线是连线本身）、`problem` 画不成的原因（null 表示可以跑）、`cover` 连线路线才有：`drive_m` 压过全部连线实际要开的路程（含倒车补角，不含接入段）、`cusps` 换向次数、`corners` 掉头补角的拐角数；画不成为 null）、`route_limits`（`min_radius` 能稳定跟踪的最小转弯半径、`max_points`、`max_laps`、`max_entry_m`、`body` 连线路线压线用的车身 [后轴中心到车头, 到车尾, 半宽] 米、`drive` 同 hello、`nozzle_x` 差速车连线路线按喷头压线时喷头在后轴前多少米（阿克曼车为 null，仍按车身中心）。差速车的 `cover` 另有 `pivots` 原地转向次数，`corners` 是原地转向的拐角数、`cusps` 是倒车换向次数） |
| `line`（诊断） | `diag`：`kappa_cmd` 下发曲率、`kappa_act` 车实际走出的曲率（融合角速度 / 车速，车速 < 0.05 m/s 为 null；车头坐标左转为正，1/m）、`yaw_rate` rad/s、`rtk_along_m` / `rtk_left_m` 最近一次 RTK 固定解与融合在同一时刻推算的主天线位置之差（车体系沿车头 / 向左，米；超过 1 s 没有新的固定解为 null）、`position_outliers` 融合里位置离群（降权或拒绝）次数、`position_resets` 位置重置次数、`jumps` 本次开机以来的定位跳动次数。只读，app 的诊断页画曲线用 |
| `line`（定位跳动） | `rtk_events`：最近 20 次定位跳动（RTK 固定解与融合推算差 3 cm 以上，间隔不到 1 s 的算同一次）：`id` 这次开始的时刻（车上单调时钟，同一次在每帧里重复出现）、`x`/`y` 车当时的本地坐标、`lat`/`lon`、`d` 最大差值米、`left` 向左分量、`dur` 持续秒、`age_s` 距今秒。只在融合本来是固定解、可用于控制时记。app 按经纬度累计，在地图上标出反复跳动的地点 |
| `line`（车身姿态） | `attitude`：`roll_deg` 横滚、`pitch_deg` 俯仰、`yaw_deg` IMU 偏航，度，ROS REP-103 车体系（x 车头、y 左、z 上）：横滚左侧抬起（右侧压低）为正，俯仰车头压低为正，偏航左转为正（−180…180）；`source` 为 `imu_madgwick`（旧车：`imu` 组件是 ROS 2 `imu_filter_madgwick`，gain 0.01，读底盘原始 `/imu`，发 `/imu/data`）或 `hda536t`（Gen5：`imu` 组件是瑞芬 HDA536T 驱动，姿态是传感器自己算的，偏航上电为 0），这一项只给 app 显示（填了天线高度时融合用同一份横滚俯仰做倾斜补偿，见下一行）；0.5 s 没有新姿态或车端没有 `imu` 组件时为 null。没有磁力计，`yaw_deg` 没有绝对参考：0 大致是 `imu` 组件启动时的车头方向，随陀螺零偏漂移（实测静止约 3°/分钟），组件重启归零；旧车端没有这一项。绝对航向：`rtk.heading.yaw_deg`（双天线，只在固定时有）、`pose.yaw_deg`（RTK 融合，控制用）|
| `line`（倾斜补偿） | `tilt`：融合按 IMU 横滚俯仰把 ANT1 的三维杠杆臂投到水平面（`docs/GNSS_FUSION.md`「倾斜补偿」）。`height_m` 天线相位中心离地高度（`~/rtk/fusion.json` 的 `antenna_height_m`，0 表示没填、不补）、`active` 这会儿在不在补、`reason` 没补的原因（`no_height` 没填高度 / `attitude_stale` 0.2 s 没有新姿态 / `too_steep` 横滚或俯仰超过 15° / `imu_mismatch` IMU 已不用），`active` 时再给用的 `roll_deg` / `pitch_deg`（度，REP-103）和补偿量 `forward_m` / `left_m`（天线水平位置比平地时偏了多少，车体系前、左，米），不在补时这四项为 null。没补只降级：`pose.reasons` 加 `tilt_uncompensated`（没填高度时不加），不影响 `usable`。旧车端没有这一项（null） |

跟随节点停止或状态超过 1.5 s 未更新时 `follow.stale=true`，`target`/`tracks` 置空，界面不得显示旧目标。
`rtk` 超过 2 s、`line` 超过 1.5 s 未更新时只发 `{"stale": true}`，界面不得显示旧的定位等级或偏差。
人体识别（默认关闭）超过 3 s 没发状态时 `link.cam_fps` 为 null、`persons` 为空；此时 `follow` 启动的 ack 为 `ok=false`、`message` 为“人体识别没在运行，跟随用不了”。

`ack`：`{"type":"ack","id":…,"cmd":…,"ok":…,"message":…}`；`pong`：原样带回 `t`。

## 上行

| 指令 | 字段 | 车端处理 |
|---|---|---|
| `drive` | `throttle`、`steer`（-1..1）、`tier`（low/med/high）、`seq` | 按档位与共享运动学换算成 vx/wz，走与网页相同的 `apply_manual`；比例小于 2%（`STICK_DEADBAND`）当 0；乱序旧指令丢弃。app 1.9 起不分档：固定发 `high`，油门、方向已按摇杆行程过曲线（刚出死区约 0.025，推一半约 0.2，推满 1） |
| `follow` | `enable` | 启动/停止跟随（后台执行，结果回 ack） |
| `estop` | — | 先清零手动，再锁存 ESTOP，不等其他操作的锁 |
| `reset` | — | 健康且停稳时复位 |
| `select_target` | `bearing_rad` | 转发 `/follower/select_target`，跟随节点选相机刚看到的、方位最接近（≤8°）的已确认轨迹 |
| `ping` | `t` | 回 `pong`，app 用来量往返延迟 |
| `route` | `action`（set/start/clear）、`route`（set 必带，start 可带：带了就先保存再开始）：`kind`（`path` 连线路线，app 默认：操作员点的点按连线顺序走直线，没连回起点就走一遍不绕圈；`circle`；旧版 `points` 自动圆弧过弯）、`points`（≤400 个，本地 ENU 米）、`closed`、`center`、`radius`、`clockwise`、`laps`（1–10，开放路线只跑一次）、`turn_radius`（米）、`cruise`（0.1–1.0 m/s） | 转发 `/line/command`（`route_set`/`route_start`/`route_clear`），结果同样看 `line.last_result`。车上把路点换算成经纬度存进 `~/rtk/route.json`。几何见 `radar_system/route_plan.py`：`path` 要让车身中心（车长中点，后轴前 (front−rear)/2）压过全部连线：缓弯（车身中心离拐角 ≤ 5 cm）直接转过去；急弯掉头补角——车身中心开过拐角 5 cm，整车开出拐角（车尾也过 5 cm）后倒车一次、车头摆向下一条边，再往前开上下一条边，三段都可打方向，在"前进-倒车-前进"里按 Reeds-Shepp 取最短（90° 多开 R·π/2 ≈ 3.0 m、换向 2 次），不要求拐角两边的线有多长；起步用 Reeds-Shepp 最短路径（可以倒车）开到第一段延长线上、车身中心到第 1 个点前 0.5 m 对正，已压在延长线上就直接开过去（app 用同一算法画预览、做仿真）。`points` 拐角按 `turn_radius` 圆弧过渡，圆从最南点出发，二者用 Dubins 接入段（只前进，最长 30 m）。然后申请 NAVIGATION；跑完停在终点。NAVIGATION 默认不经过雷达防撞（底盘驱动参数 `navigation_uses_lidar`，遥测 `motion.nav_lidar`）。运行中停止用 `line` 的 stop；定位丢失暂停后，定位连续可用 1 s 自动从暂停处继续（`line.message`「定位恢复，自动继续」），也可以再发 route start。测向浮点（无效）时航向靠陀螺维持：主、副天线都是固定解、航向 1σ ≤ 2° 就照样走，`line.pose.reasons` 带 `gyro_hold` |
| `line` | `action`（mark/start/stop/clear）、`point`（mark 时 A/B）、`cruise`（start 时可选，0.05–1.0 m/s） | 转发 `/line/command`；ack 只表示已转发，执行结果看遥测 `line.last_result`。记点需要 RTK 固定解；start 先检查起步条件（离线 ≤0.5 m、车头偏 ≤30°），再申请 NAVIGATION。融合位姿不能用于控制时结果是 `pose_unusable：原因代码`（多个用顿号连接，代码见 `docs/GNSS_FUSION.md`），由 app 翻译 |

## 安全约定

### CORS 连接开关

`GET /api/rtk/cors` 的 `actions: ["disconnect", "connect"]` 表示支持连接开关；旧服务没有此字段时 App 禁用开关。`configured` 表示车端保留有效账号，`enabled` 表示持久配置是否启用连接，`revision` 标识配置版本。`ntrip` 是 RTK 节点实际状态，`rtk_stale` 为真时不得用它确认操作已完成。

断开请求体仅为 `{"action":"disconnect"}`，重新连接仅为 `{"action":"connect"}`。这两种请求不带密码，不接受额外配置字段，不替换账号。未配置账号返回 `400 / not_configured`。仍沿用停车检查：底盘在线且数据新鲜、速度绝对值 ≤ 0.02 m/s、运动模式 IDLE／ESTOP、没有自动任务；不满足返回 `409 / vehicle_not_idle`。携带浏览器 `Origin` 的请求仍返回 403。

车端在同一个权限 600 的 `ntrip.env` 中原子写入 `NTRIP_ENABLED=0/1`。省略此项的旧账号默认启用；主动断开后 RTK 热重载停止当前 NTRIP socket 和重试，保留串口读取、GNSS 原始数据与账号，服务重启也不自动重连。重复相同开关操作不改 revision、不重启客户端。重新应用完整账号配置会设为启用。

HTTP 200 的 `status: pending_reload` 只表示请求已保存。App 要等新鲜遥测里的 `rtk.ntrip.config_revision` 与回执一致，且 `enabled=false`、`connected=false`、`config_error=null`，才显示已断开。重新连接要等同版本 `enabled=true` 确认加载，随后继续区分连接中与已连接。旧 RTK 节点没有上报 `enabled` 时编码器省略此字段，不将缺失字段当作主动断开；原有 WebSocket 协议版本保持 v1。

### 车辆操作

- 摇杆按住时 app 以 20 Hz 发 `drive`；松手立即发一次 0。车端 0.75 s 收不到新指令自动停。
- WebSocket 断开时，若该会话开过车，车端立即清零。3 s 收不到任何上行消息（app 每秒 ping）即判断线。
- 急停同时走 WebSocket 与 HTTP。复位必须单独操作，急停不会被任何指令自动解除。
- 所有速度、转角上限由车端共享配置决定，app 只发比例。
- 走直线、走路线运行中：定位不可用立即停车并暂停，定位连续可用 1 秒后自动继续；人工暂停、重启恢复和控制权中断必须明确继续，急停/故障还须先复位。指令超时仍作用于 NAVIGATION，雷达防撞是否参与由 `motion.nav_lidar` 表示。融合在丢失固定解后还会用陀螺和轮速推算 0.5 s 才判定不可用，0.5 m/s 时从失锁到停稳约 0.4 m。


## 路线人工暂停、续画、补画（App 1.7.0）

`line.operations.actions` 为新能力开关。旧节点不发此字段时，App 禁用新增操作。`operations` 还包含 `durable`、`spray_supported`、`error`、`job`、`history`、`elements`、`preview` 和 `execution`（本次实际执行路径）。过期 `line` 不携带这些状态。

新增 `route.action`：

- `pause`：带 `route_rev`、`job_id`。人工暂停独立于定位故障恢复，phase 先为 `PAUSING`，持续零速，停稳后 `PAUSED` 并释放 NAVIGATION。
- `preview`：带 `route_rev` 和 `operation`。resume 需 `job_id`、可选 `overlap_m`（0–0.5）；rework 需 `element`、`from_m`、`to_m`（按所选元素的方向起算，至少 0.2 m）。预览不驱动车。
- `resume` / `rework`：带 `route_rev`、`preview_token`；resume 另带 `job_id`。车端核对原点、路线/任务、车辆参数、停稳/定位和车辆位置；预览有效 60 秒，移动超过 0.10 m 或转动超过 5° 须重算。重复 token 不重复启动。

ack 仍仅表示转发，执行结果用 `line.last_result.id` 关联。预览通过 `operations.preview.request_id` 关联请求，返回 token、目标范围、接入路径和长度；接入最长 30 m。执行时的路径通过 `operations.execution.sections` 给 App 显示。

`job` 包含 ID、kind（route/rework）、state、reason、cursor_m、total_m、can_resume。任务路径及方向按经纬度持久化到 `route_path + '.task.json'`，每秒及状态转换时原子保存；重启恢复待继续状态。人工暂停和中断任务禁止被 set/clear/start 覆盖，需显式继续或结束。`line stop` 结束当前任务；结束补画会恢复原任务的待继续断点。完成与恢复父任务在一次原子写入中提交，最近 20 次结果保留在 history。

当前 `spray_supported=false`，cursor_m 是车辆规划路径进度，含原路线接入/掉头，不是已喷漆长度；重新接入不重复增加原任务进度。新功能没有新增泵阀驱动或自动避障能力。突然掉电可能重走最后一秒未落盘的范围，恢复后不自动开车。写入失败不会阻止停车，确认续画/补画时写入失败不会申请控制权。


## 整场内场优先规划（App 1.8.0）

`POST /api/field/plan` 为只读规划接口。请求例子：

```json
{"origin":[30,120],"nozzle":[0.30,0.24],"paint_speed":0.5,"transit_speed":0.8,"job":{"boundary":[[0,0],[30,0],[30,20],[0,20]],"elements":[{"id":"edge","name":"边线","kind":"polyline","points":[[1,1],[29,1]],"closed":false}]}}
```

`nozzle` 依次为后轴前伸和向右偏移，来自 App 既有设置；边界与图形均为已定位 ENU 米坐标。支持 `polyline`、`circle`（`center`、`radius`）、`arc`（另加 `start_deg`、`sweep_deg`）和人工 `point`（`center`）。当前需要凸形边界、差速底盘、新鲜可用定位和停车状态。

成功返回 `{ok:true,plan:{...}}`。`plan` 包含 token、当前 `start:[x,y,yaw_rad]`、原点和路线版本、划线顺序、车行预览、人工点位、预计时间、`inside` 与 `max_outside_m`。`preview[].outside_spans` 标识有车身外伸的区段；`outside[].body` 是外廓示意。连续曲线及转向的最大外伸通过解析检查求出，预览位置与外场时间为采样估计。

```json
{"type":"route","action":"field_start","preview_token":"服务端返回的 token","allow_outside":true}
```

客户端不提交运动路径，Web 服务从短期缓存取得已算路径后转发 `route_field_start`。token 有效期 60 秒，车端再次核对车位（0.10 m / 5°）、原点、路线版本、尺寸和运动状态；需要外场空间的规划要求 `allow_outside:true`。开始前持久保存完整路径、分段速度及场地边界，然后申请控制权。HTTP 计算不会启动车辆。

任务状态增加 `job.field_plan` 和 `job.current_role`（`paint` / `transit`，表示规划用途，不代表喷头已工作）。暂停与续走沿用版本化断点；整场恢复接入使用同样的内场优先策略，`preview.max_outside_m` 提示接入及剩余路线的外伸，存在外伸时 `route.resume` 也要求 `allow_outside:true`。

此功能不考虑压湿线，不提供障碍物或坑洼地图，也不宣称全局最优。喷头驱动尚未接入，`spray_supported:false`。最小半径、车身外廓、转向速度、加速度和弯道限速来自实际车端配置；允许必要外场，但对外场暴露增加规划代价。


### 选边作业（App 1.8.1）

保持 `job.boundary` 与 `job.elements` 为原始整场图形，使用可选的 `job.selected_strokes` 过滤本次执行范围：

```json
{"selected_strokes":[{"element_id":"boundary","edge_index":2},{"element_id":"centre-circle","edge_index":null}]}
```

折线的边从 0 开始编号；闭合折线最后一边连接末点和首点。圆、圆弧以 `edge_index:null` 选择整个图形，人工点位不可选择为自动作业边。选择在可行性计算前过滤，因此未选中的紧圆弧不会阻止其他边执行。一次可选择 1–80 条；省略 `selected_strokes` 表示整场，显式 null、空列表、未知编号、重复项和错误类型均拒绝，不能回退为整场。

响应增加 `selection_mode:all/selected`，每个 `order` 项增加 `selection:{element_id,edge_index}`。原始图形编号和边编号组成唯一标识，不受重排、反向和圆形起点选择影响。App 核对返回集合与请求一致，旧版忽略选择并返回整场的服务不可用于选边启动。

缓存路径、启动及断点仅包含所选边和必要转场。任务标签为「选边作业 · N 条」，`job.selected_strokes` 保留选择以便检查；原场地边界仍用于接入和续走规划。执行完所选部分即结束，不自动续做其他边。执行中的任务或保留断点不被新选边任务覆盖。
