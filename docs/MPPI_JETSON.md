# Jetson MPPI 启用与验证

更新：2026-09-20。正式入口默认 MPPI＋CUDA；本次仅进行本地修改和验证，开发板不在现场，未部署或启动实车服务。

## 启动配置

`run_follower.sh`、systemd 托管跟随和其余 ROS 入口统一由 `ros_env.sh` 加载环境：进程变量 → `/etc/rk3588/runtime.env`（或进程设置的 `ROBOT_RUNTIME_ENV`）→ 默认值。默认文件缺失允许继续，显式文件缺失、不可读或格式错误会阻止启动。服务单元不再单独解释该文件；无需为 systemd 与终端各写一套值。
`person_follower.py` 的 CLI 参数优先于已加载的环境值。
默认 `controller=mppi`、`mppi_device=cuda`、`mppi_samples=1024`。
CUDA 不可用直接报错，避免自动落到 CPU。纯算法 `FollowerConfig()` 保留纯追踪默认值，供测试和离线回放使用。

将 `deployment/runtime-jetson.env.example` 中的配置合并到现有 `/etc/rk3588/runtime.env`，保留现场相机、模型、标定和 Python 路径：

```ini
ROBOT_FOLLOW_CONTROLLER=mppi
ROBOT_MPPI_DEVICE=cuda
ROBOT_MPPI_SAMPLES=1024
```

CLI 参数优先于环境变量。依赖为与 JetPack/CUDA 匹配的 NVIDIA PyTorch；ROS 2 环境支持 Humble/Jazzy。
如果使用 venv，应包含 `--system-site-packages`，并通过 `RK3588_PYTHON` 选择同一个解释器，确保能导入 ROS 与 CUDA PyTorch。
显式 Python 路径无效直接报错；ROS 未显式选时按 Humble、Jazzy 顺序选择。日志列出实际配置文件、ROS setup、Python、模型、机器人配置和深度标定路径，不打印完整环境。
`requirements-jetson.txt` 列出 Ultralytics 但不指定通用 PyTorch wheel；安装依赖时须确认 pip 不会覆盖板端 CUDA 版 PyTorch/torchvision。

在完整发布包安装后，板端运行：

```bash
# 使用服务实际采用的 Python 跑只计算、不连 ROS 的基准
/path/to/python radar_system/tools/benchmark_mppi.py --device cuda --samples 1024 --require-budget

# 托管启动；保持被动，网页选择跟随后才申请运动控制权
bash radar_system/start_all.sh headless

# 独立诊断入口；勿与已有跟随服务重复启动
bash radar_system/run_follower.sh --passive
```

现有服务加载新版本或修改环境变量后，需重启 `rk3588-perception@follower.service`。
独立入口不带 `--passive` 时会显式申请 FOLLOW；`--dry-run` 仅计算，不发送运动请求。
系统服务名与 `/etc/rk3588` 路径保持兼容，名称不表示使用 RKNN。

## 优化内容

- CUDA rollout 在启动时预热并捕获为 CUDA Graph；控制周期复用固定输入缓冲，只更新传感器、目标和车辆状态。
- 车体五圆距离查询合并为批量操作；Torch 的相关噪声用预计算矩阵批量生成；建距离场先对平方距离求最小值再开方。
- 最终轨迹一次拷回 CPU，使用向量化矩形几何复验，消除每个预测步三次 GPU→CPU 同步。
- 复验使用完整原始障碍点，距离场去重和点数上限不会丢掉最终复验中的细小障碍；包含当前位姿与执行延迟期间的位姿。
- 20 Hz 控制每次推进 50 ms 的热启动序列，预测步长仍为 150 ms；目标预测包含 350 ms 执行延迟。
- 新目标、控制权重选、里程计重置和暂停时清理旧计划，避免沿用上一目标或机动的控制序列。
- 超过 25 ms 求解预算的结果当周期停车，不交给下游继续执行；连续 12 次失败后切回纯追踪，下一周期才使用备用控制。异常同样先停车。
- 输出前复查传感器/位姿时效及总控制耗时。GPU 求解计时包含设备实际完成，不能用异步提交耗时冒充执行耗时。

MPPI 仍只生成参考速度和转角，后续车体扫掠、刹车包络、脱困、控制权仲裁和驱动防撞继续执行。
默认跟随速度上限仍为 0.45 m/s，40×0.15 s 提供 6 s 预测，恒定最高速时约 2.7 m 行程。
预测长度不保证存在绕行路线，MPPI 不承担全局规划。

## 性能和运行状态

`benchmark_mppi.py` 测试 400 点扫描、80 cm 门和近墙三种合成场景，输出预热时间、P50/P95/最大耗时和超预算次数。
`--require-budget` 在任一场景 P95 超过预算时返回非零。需在相机 TensorRT 等正常负载运行时再次检查，确认算力竞争下仍有余量。
可用 `--no-cuda-graph` 对照普通 Torch rollout；CPU/NumPy 基准须显式选 `--device cpu` / `--device numpy`。
首次建议保留 1024 条样本，增加到 2048 前先实测，不依据理论 GPU 算力推断耗时。

`/follower/status` 中：

| 字段 | 含义 |
|---|---|
| `controller_requested` / `controller` | 配置请求 / 当前实际控制器 |
| `controller_backend` | 实际数组后端，应为 `torch/cuda` 或 `torch/cuda:0` |
| `mppi_cuda_graph` | 是否已完成 CUDA Graph 捕获 |
| `mppi.solve_ms` / `over_budget` | 最近求解耗时 / 是否超过预算 |
| `mppi.failure_streak` / `fell_back` | 连续失败次数 / 是否已降级 |
| `mppi.cost` | 最低采样代价（含名义控制正则），不是最终平均轨迹的复算代价 |
| `mppi_stop_reason` | 本周期 MPPI 停车原因 |

网页运行详情显示控制器、后端、求解耗时和回退状态。没有目标时没有求解数据属正常。
回退后重新选择 FOLLOW 才重试 MPPI；回退原因未解决时不要依赖频繁重选维持运行。
需固定使用纯追踪时设置 `ROBOT_FOLLOW_CONTROLLER=pure-pursuit` 并重启跟随服务。

## 本地验证与待办

```bash
python3 -m pytest tests/test_mppi.py tests/test_person_follower.py tests/test_mppi_runtime.py -q
python3 radar_system/tools/benchmark_mppi.py --device numpy
```

新增测试覆盖实际启动默认值、环境/CLI 优先级、CUDA 缺失、向量化几何对照、完整障碍复验、
延迟段运动学对照、50/150 ms 序列推进、超时当周期停车、计算期间观测过期、异常回退及目标/控制权切换。
CUDA Graph 与普通 CUDA 路径的对照测试需要 GPU，本机无 CUDA 时明确跳过。

完整工作区在本次修改前已有 23 项失败，涉及目标身份保持、深度/盲区处理、部分底盘反馈及工具测试；
本次保留这些既有改动，不将其标为通过。详细对照结果在 `artifacts/mppi-jetson-20260920/`。
因此本地候选包不等于整车验收完成。板端 CUDA Graph、真实传感器链路、实际控制周期和运动效果均待设备回来后验证。

视觉节点默认引用已随包提供的 `models/yolo26s.pt`，只检测 person，以 CUDA FP16 推理；
仅显式选择 TensorRT 导出时需要板端生成 `.engine`。本轮视觉模型调整未在 Jetson 或实车上验证。
MPPI 自身不依赖视觉模型即可运行合成基准。
