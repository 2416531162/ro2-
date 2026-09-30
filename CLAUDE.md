# 给开发会话的规则（Gen5 RTK 划线车车端）

先读 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)（进程、控制、配置在哪）和 [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md)（完整规则）。要点：

- 回复、注释、文档都用中文。
- 本仓库只放 Gen5 划线车用得到的东西；和当前无关的（比如早期 Jetson 人体跟随）不要再加回来。
- 开始前 `git status`：别人没提交的改动不要动、不要混进自己的提交。**用户同意才提交。**
- 只跑相关的测试文件，不跑全套（用户要求）。
- **不许做会让轮子转的测试**（写驱动器速度、跑车、原地转），除非用户在当次消息里明确同意。
- 部署只从 git 提交：`python3 deployment/manage.py stage --git HEAD <目录>`，对照车上 `/opt/rk3588/current` 的清单，
  推上去 `verify`，**问用户，用户回「部署」才 `activate --profile rtk --user root`**，激活前车必须停着、没在作业。
- 车上只读操作（看日志、订阅话题、读 `~/rtk/jobs` 作业记录）不用问；判断实车问题先看作业记录的数据。
- 改了规划常数或协议字段，同步平板 app（常数、`tests/fixtures/app_protocol/` 样本）。
