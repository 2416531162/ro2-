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

`hello`：`v`、`name`、`session`、`video` 路径、`car`（`front_m`/`rear_m`/`half_width_m`/`wheelbase_m`/`track_m`/`max_steer_deg`/`min_turn_radius_m`/雷达与相机安装位置，车体系原点后轴中心、x 前 y 左）、`camera`（`width`/`height`/`fx`/`cx`）、`speed_tiers`（手动三档 m/s）。

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
| `rtk` | RTK 接收机（`rtk_node`）：`fix`（FIXED/FLOAT/SINGLE/NONE，与融合的取舍一致：FIXED 只指 `NARROW_INT`，`WIDE_INT`/`L1_INT` 归 FLOAT；解算状态不是 `SOL_COMPUTED` 按 NONE）、`sats`、`diff_age`（秒，无差分为 null）、`sigma_h`（米）、`heading_ok`（双天线测向整周固定，且两根天线的位置都是 RTK 固定解——两根都固定就信，不比较测向长度、不写死也不实测天线间距；两天线平面距离 <0.10 m 视为室内转发器同一路信号，不算）、`baseline_m`（测向长度）、`receiver_ok`、`ntrip`（`configured`/`connected`/`error`/`age_s`）、`antennas`（`ant1` 主天线 BESTPOS、`ant2` 副天线 BESTNAVH：`sol`/`fix`/`sats_used`/`sats_tracked`/`age_s`；`ant2` 另有 `offset_m` 两天线固定解的平面实时距离（东、北，不算高差）、`consistent` 保留为 null）、`heading`（测向解：`sol`/`fix`/`sats_used`/`sats_tracked`/`length_m`/`pitch_deg`/`sigma_deg`/`ref_m` 两天线固定解的平面实时距离/`valid`） |
| `line` | RTK 走直线（`line_node`）：`phase`（IDLE/SELECTING/RUNNING）、`message`、`cruise`、`motion_mode`、`points`（`A`/`B` 本地 ENU 米，x 东 y 北）、`pose`（`x`/`y`/`yaw_deg`（东为 0 逆时针）/`speed`/`quality`/`usable`/`sigma_xy`/`sigma_yaw_deg`/`reasons`）、`track`（`state`/`reason`/`e` 横偏米（行进方向的左侧为正）/`heading_err_deg`/`s` 沿线米/`length`/`v_cmd`（倒车为负）/`rms`/`max`/`gear` forward 或 reverse/`section` 第几段/`sections` 共几段/`switching` 在换向点等车停稳）、`last_result`（上一条 line/route 指令的执行结果，按 `id` 对应）；走路线另见下一行 |
| `line`（路线） | `task`（line/route，当前或最近一次任务）、`origin`（本地坐标原点的 [纬度, 经度]，融合还没有有效定位时为 null；`line_node` 每次启动取第一次有效定位，原点会变，路线存经纬度所以照样对得上）、`antenna`（主天线最近一次定位：`x`/`y` 本地米、`sol`、`sigma` 米；超过 2 s 没更新为 null。融合位姿不可用时 app 用它画车的位置）、`route`（车上存的路线，换算到当前本地坐标：`kind` points/circle、`points` [[x,y]…]、`closed`、`center`、`radius`、`clockwise`、`laps`、`turn_radius`、`cruise`、`rev` 版本号、`length` 路线长度米（不含接入段；连线路线是连线本身）、`problem` 画不成的原因（null 表示可以跑）、`cover` 连线路线才有：`drive_m` 压过全部连线实际要开的路程（含倒车补角，不含接入段）、`cusps` 换向次数、`corners` 掉头补角的拐角数；画不成为 null）、`route_limits`（`min_radius` 能稳定跟踪的最小转弯半径、`max_points`、`max_laps`、`max_entry_m`、`body` 连线路线压线用的车身 [后轴中心到车头, 到车尾, 半宽] 米） |

跟随节点停止或状态超过 1.5 s 未更新时 `follow.stale=true`，`target`/`tracks` 置空，界面不得显示旧目标。
`rtk` 超过 2 s、`line` 超过 1.5 s 未更新时只发 `{"stale": true}`，界面不得显示旧的定位等级或偏差。
人体识别（默认关闭）超过 3 s 没发状态时 `link.cam_fps` 为 null、`persons` 为空；此时 `follow` 启动的 ack 为 `ok=false`、`message` 为“人体识别没在运行，跟随用不了”。

`ack`：`{"type":"ack","id":…,"cmd":…,"ok":…,"message":…}`；`pong`：原样带回 `t`。

## 上行

| 指令 | 字段 | 车端处理 |
|---|---|---|
| `drive` | `throttle`、`steer`（-1..1）、`tier`（low/med/high）、`seq` | 按档位与共享运动学换算成 vx/wz，走与网页相同的 `apply_manual`；乱序旧指令丢弃 |
| `follow` | `enable` | 启动/停止跟随（后台执行，结果回 ack） |
| `estop` | — | 先清零手动，再锁存 ESTOP，不等其他操作的锁 |
| `reset` | — | 健康且停稳时复位 |
| `select_target` | `bearing_rad` | 转发 `/follower/select_target`，跟随节点选相机刚看到的、方位最接近（≤8°）的已确认轨迹 |
| `ping` | `t` | 回 `pong`，app 用来量往返延迟 |
| `route` | `action`（set/start/clear）、`route`（set 必带，start 可带：带了就先保存再开始）：`kind`（`path` 连线路线，app 默认：操作员点的点按连线顺序走直线，没连回起点就走一遍不绕圈；`circle`；旧版 `points` 自动圆弧过弯）、`points`（≤400 个，本地 ENU 米）、`closed`、`center`、`radius`、`clockwise`、`laps`（1–10，开放路线只跑一次）、`turn_radius`（米）、`cruise`（0.1–1.0 m/s） | 转发 `/line/command`（`route_set`/`route_start`/`route_clear`），结果同样看 `line.last_result`。车上把路点换算成经纬度存进 `~/rtk/route.json`。几何见 `radar_system/route_plan.py`：`path` 要让车身中心（车长中点，后轴前 (front−rear)/2）压过全部连线：缓弯（车身中心离拐角 ≤ 5 cm）直接转过去；急弯掉头补角——车身中心开过拐角 5 cm，整车开出拐角（车尾也过 5 cm）后倒车一次、车头摆向下一条边，再往前开上下一条边，三段都可打方向，在"前进-倒车-前进"里按 Reeds-Shepp 取最短（90° 多开 R·π/2 ≈ 3.0 m、换向 2 次），不要求拐角两边的线有多长；起步用 Reeds-Shepp 最短路径（可以倒车）开到第一段延长线上、车身中心到第 1 个点前 0.5 m 对正，已压在延长线上就直接开过去（app 用同一算法画预览、做仿真）。`points` 拐角按 `turn_radius` 圆弧过渡，圆从最南点出发，二者用 Dubins 接入段（只前进，最长 30 m）。然后申请 NAVIGATION；跑完停在终点。NAVIGATION 默认不经过雷达防撞（底盘驱动参数 `navigation_uses_lidar`，遥测 `motion.nav_lidar`）。运行中停止用 `line` 的 stop；定位丢失暂停后再发 route start 从暂停处继续 |
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
- 走直线、走路线运行中：定位不可用立即停车并暂停，恢复后也要操作员再按开始；手动摇杆、急停、控制权被切走都会结束任务。雷达防撞与指令超时同样作用于 NAVIGATION。融合在丢失固定解后还会用陀螺和轮速推算 0.5 s 才判定不可用，0.5 m/s 时从失锁到停稳约 0.4 m。
