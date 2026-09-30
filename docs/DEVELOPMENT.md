# 开发与部署规则

2026-09-30 整理时发现：车上跑的代码与仓库差 44 个文件（几天的工作部署了没提交），工作区另有几个会话改了一半的东西，
发布包是在车上当前版本的基础上手工换文件拼出来的；一个会话的候选版和另一个会话的改动一起激活，出了问题很难分清是谁的。
下面的规则就是为了不再这样。

## 1. 改代码

- 开始前先 `git status`。别人没提交的改动不要动、不要混进自己的提交；需要改同一个文件时先问用户。
- 一个主题一个提交，提交说明写清楚改了什么、为什么、怎么验证的；**只有用户同意才提交**。
- 改了规划常数、协议字段：同步平板 app（常数、`tests/fixtures/app_protocol/` 样本，用
  `radar_system/tools/export_app_fixtures.py` 生成，app 的 `app/src/test/resources` 也放一份）。
- 回复、注释、文档都用中文。

## 2. 测试

- 只跑相关的测试文件（全套约 10 分钟，用户要求平时不跑全套）。常用：
  - 跟踪 / 原地转向：`test_path_tracking`、`test_differential_drive`、`test_pivot_settle`、`test_pivot_delay`
  - 作业 / 路线 / 场地：`test_line_mission`、`test_route_plan`、`test_route_operations`、`test_field_planner`
  - 定位：`test_gnss_fusion`、`test_rtk_hold`、`test_imu_yaw_fusion`、`test_line_fusion`、`test_line_cascade`
  - 协议：`test_app_link`；驱动：`test_oid_driver`、`test_oid_rs485`、`test_hda536t`
- 仿真：`tests/gen5_line_sim.py`（实测底盘模型）、`tools/gen5_turn_eval.py`（原地转向、圆）、`radar_system/tools/path_sim.py`。
- **不许做会让轮子转的测试**（写驱动器速度、跑车、原地转），除非用户在当次消息里明确同意。

## 3. 部署（只从提交部署）

```bash
# 1. Mac：从提交打包（只含已提交的内容，清单记下来源提交）
python3 deployment/manage.py stage --git HEAD /tmp/rel
python3 deployment/manage.py info /tmp/rel                  # release_id 和来源提交
# 2. 对照车上当前版本：只该差你这次的文件
adb -d shell readlink /opt/rk3588/current
adb -d pull /opt/rk3588/current/manifest.json /tmp/live.json   # 与 /tmp/rel/manifest.json 的 files 比
# 3. 推上去、校验（不影响车上运行）
adb -d push /tmp/rel /opt/rk3588/releases/<release_id>
adb -d shell 'cd /opt/rk3588/releases/<release_id> && python3 deployment/manage.py verify /opt/rk3588/releases/<release_id>'
# 4. 问用户，用户回「部署」才激活（车必须停着、没在作业）
adb -d shell 'cd /opt/rk3588/releases/<release_id> && python3 deployment/manage.py activate --profile rtk --user root /opt/rk3588/releases/<release_id>'
# 5. 检查：systemctl list-units 'rk3588-*'；journalctl 里新进程没有 Traceback（停旧进程时的退出异常可忽略）
```

- 不要在车上的发布目录里手工换文件；不要从工作区打包（`stage` 不带 `--git` 会提示）。
- 激活前再看一次 `/opt/rk3588/current`：别的会话可能刚部署过，要在它上面的提交上做。
- 回滚：用上一个版本目录再 `activate` 一次（`/opt/rk3588/releases/` 下都留着）。
- 车上 `adb`：USB 用 `adb -d`；Wi-Fi 时 `adb connect <IP>:5555`（地址随网络变）。只读操作（看日志、订阅话题、读作业记录）不用问。

## 4. 排查实车问题

作业记录在车上 `~/rtk/jobs/<时间>_<类型>_<id>/`（`job.json` 摘要 + mcap 原始数据），用 `rosbag2_py` 读。
先看数据再下结论：原地转向、起步、定位跳、横偏都能从 `/line/status`、`/navigation/command`、`/oid/status`、`/gnss/raw` 里对出来。
