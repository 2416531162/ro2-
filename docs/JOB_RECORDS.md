# 作业自动记录（排查问题用）

每次在 app / 网页上点「开始作业」，车上的 `jobrec` 组件（`radar_system/job_recorder.py`）自动把这次作业的全部原始数据存下来，出了问题回头查。不在 app 上显示，也不参与控制：只订阅话题、写文件，这个进程出错或退出都不影响开车。

## 记什么、什么时候记

- **什么时候**：车进入 `SELECTING` / `RUNNING`（`/line/status` 的 `phase`）就开一段，从点开始前 10 s 记到停下后 5 s。一次「开始」或「继续」是一段；暂停后再继续，接着往同一个作业目录里加段。
- **哪些话题**：`/line/status`、`/line/command`、`/navigation/command`、`/manual/command`、`/motion/status`、`/oid/status`、`/wheeltec/status`、`/gnss/status`、`/gnss/raw`、`/gnss/fix`、`/gnss/heading`、`/imu`、`/imu/data`、`/imu/status`、`/odom`、`/tf`、`/voltage`、`/rosout`。不记相机图像、雷达。
- **大小**：约 15 KB/s（zstd 压缩），一小时约 50 MB。

## 存在哪

```
~/rtk/jobs/<开始时间>_<作业名>_<id 前 8 位>/      （服务用 root 跑：/root/rtk/jobs）
    job.json          作业 id、名称、结果（DONE / CANCELLED / PAUSED…）、每段起止、每段摘要
    context/segNN/    这一段开始时的配置快照：release.txt（版本）、robot 配置、oid.yaml、hda536t.yaml、
                      fusion.json、route.json、line_state.json、route_operations.json、runtime.env
    segNN/            这一段的 ROS 2 bag（mcap）
```

`job.json` 里每段的 `summary`：跟踪横偏（最大、95%、平均）、RTK 各解算状态占比、双天线航向有效占比、最少卫星数、最大差分龄期、定位跳动次数、控制权切换记录、不健康原因、底盘故障 / 不应答、通信错误增量、最低电压、画线节点的提示变化。

没有作业记录的任务（网页上的 A—B 直线）按 `task-<任务名>-<时间>` 单独成一个作业。

## 空间

总量超过 2 GB 或磁盘剩余不到 1 GB 时，自动删最旧的作业目录（正在记的不删）；记录中磁盘剩余不到 512 MB 就停记这一段。

## 查看

```bash
# 在不在记、记到哪
ros2 topic echo --once /jobrec/status
# 列出作业
ls /root/rtk/jobs
# 拉到电脑上
adb pull /root/rtk/jobs/<目录> .
# 回放 / 查看 bag（板子上）
ros2 bag info /root/rtk/jobs/<目录>/seg01
```

另外 `line_mission` 每次行驶还会写一份 CSV（`~/rtk/runs/`，位姿、横偏、指令速度等），和这里的记录互为补充。
