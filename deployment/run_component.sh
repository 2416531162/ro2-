#!/usr/bin/env bash
# ROS setup.bash references optional variables while it is being sourced.
# Enable nounset only after the ROS environment has loaded.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/radar_system/ros_env.sh"
set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export RK3588_ROBOT_CONFIG="${RK3588_ROBOT_CONFIG:-$ROOT/robot_core/robot_gen5.json}"
case "${1:-}" in
  # 底盘驱动：Gen5 两后轮差速，欧艾迪双驱驱动器（旧阿克曼车的 Wheeltec 驱动 2026-11-23 删了）
  chassis)
    exec "$RK3588_PYTHON" -u "$ROOT/oid_protocol/oid_driver.py" --ros-args --params-file "$ROOT/oid_protocol/oid.yaml" ;;
  rtk) exec "$RK3588_PYTHON" -u "$ROOT/radar_system/rtk_node.py" ;;
  line) exec "$RK3588_PYTHON" -u "$ROOT/radar_system/line_node.py" ;;
  # 作业自动记录（排查问题用）：点「开始作业」就把这次作业的原始数据存到 ~/rtk/jobs，只订阅、不控制
  jobrec) exec "$RK3588_PYTHON" -u "$ROOT/radar_system/job_recorder.py" ;;
  # USB 接入的 IMU762 独立读取入口，必须指定专用串口；不自动切换车辆的 IMU。
  imu762)
    : "${IMU762_PORT:?请设置 IMU762_PORT 为专用 USB 串口路径}"
    IMU762_ARGS=(--port "$IMU762_PORT" --baud "${IMU762_BAUD:-460800}" --ros --print-hz 0)
    if [ -n "${IMU762_LOG:-}" ]; then IMU762_ARGS+=(--log "$IMU762_LOG"); fi
    exec "$RK3588_PYTHON" -u "$ROOT/rion_protocol/imu762_reader.py" "${IMU762_ARGS[@]}" ;;
  # IMU：Gen5 底盘驱动没有陀螺，起 SCH16T 模块驱动（同一条 CAN，2026-11-16 代替 HDA536T），它发原始 /imu
  # （融合用）和带传感器自己算的姿态的 /imu/data（app 显示）
  imu)
    exec "$RK3588_PYTHON" -u "$ROOT/rion_protocol/sch16t_imu.py" --ros-args --params-file "$ROOT/rion_protocol/sch16t.yaml" ;;
  # 网页 + 平板 app 的 WebSocket
  web) exec bash "$ROOT/radar_system/run_web.sh" ;;
  *) echo 'Expected chassis|rtk|line|jobrec|imu|imu762|web' >&2; exit 2 ;;
esac
