#!/usr/bin/env bash
# ROS setup.bash references optional variables while it is being sourced.
# Enable nounset only after the ROS environment has loaded.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/radar_system/ros_env.sh"
set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export RK3588_ROBOT_CONFIG="${RK3588_ROBOT_CONFIG:-$ROOT/robot_core/robot.json}"
case "${1:-}" in
  # 底盘驱动按共享配置的驱动方式选：differential（Gen5 两后轮差速）用欧艾迪双驱驱动器，其余是 Wheeltec
  chassis)
    DRIVE="$(cd "$ROOT" && "$RK3588_PYTHON" -c 'from robot_core.config import PROFILE; print(PROFILE["drive"]["type"])')" || DRIVE=ackermann
    if [ "$DRIVE" = differential ]; then
      exec "$RK3588_PYTHON" -u "$ROOT/oid_protocol/oid_driver.py" --ros-args --params-file "$ROOT/oid_protocol/oid.yaml"
    fi
    exec "$RK3588_PYTHON" -u "$ROOT/wheeltec_protocol/wheeltec_driver.py" --ros-args --params-file "$ROOT/wheeltec_protocol/wheeltec.yaml" ;;
  rtk) exec "$RK3588_PYTHON" -u "$ROOT/radar_system/rtk_node.py" ;;
  line) exec "$RK3588_PYTHON" -u "$ROOT/radar_system/line_node.py" ;;
  # 作业自动记录（排查问题用）：点「开始作业」就把这次作业的原始数据存到 ~/rtk/jobs，只订阅、不控制
  jobrec) exec "$RK3588_PYTHON" -u "$ROOT/radar_system/job_recorder.py" ;;
  # IMU 跟着底盘选。Gen5（differential）底盘驱动没有陀螺：起瑞芬 HDA536T 驱动（同一条 CAN），它发原始 /imu
  # （融合用）和带传感器自己算的姿态的 /imu/data（app 显示）。
  # 旧车：ROS 2 标准的 imu_filter_madgwick 读底盘驱动的原始 /imu（陀螺 + 加速度，不带姿态），
  # 发带横滚、俯仰的 /imu/data。没有磁力计，偏航只是相对启动时、会漂（app 上写明是相对值）；
  # 控制用的航向是 RTK 融合的。gain 0.01：默认 0.1 在 20 Hz 下每帧修正步长过大，车停着横滚也来回跳 ±0.5°
  # （2026-09-27 实车 60 s 静止：0.1 → 标准差 0.28°，0.01 → 0.05°；IMU 原始数据 1 s 平均只差 0.03°）。
  imu)
    DRIVE="$(cd "$ROOT" && "$RK3588_PYTHON" -c 'from robot_core.config import PROFILE; print(PROFILE["drive"]["type"])')" || DRIVE=ackermann
    if [ "$DRIVE" = differential ]; then
      exec "$RK3588_PYTHON" -u "$ROOT/rion_protocol/hda536t_imu.py" --ros-args --params-file "$ROOT/rion_protocol/hda536t.yaml"
    fi
    exec "/opt/ros/$ROS_DISTRO/lib/imu_filter_madgwick/imu_filter_madgwick_node" --ros-args \
         -p use_mag:=false -p publish_tf:=false -p world_frame:=enu -p gain:=0.01 \
         -r imu/data_raw:=/imu -r imu/data:=/imu/data ;;
  # 网页 + 平板 app 的 WebSocket
  web) exec bash "$ROOT/radar_system/run_web.sh" ;;
  *) echo 'Expected chassis|rtk|line|jobrec|imu|web' >&2; exit 2 ;;
esac
