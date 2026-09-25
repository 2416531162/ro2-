#!/usr/bin/env bash
# 上车前的冒烟测试：在发布目录里用车上真的 rclpy 把 Python 节点构造一遍（不开串口、不 spin），失败就不要 activate。
# 环境与 run_component.sh 相同（ros_env.sh、RK3588_PYTHON），但换到单独的 ROS 域、只走本机回环，
# 不会和正在运行的服务互相看见。用法：
#   bash deployment/smoke.sh /opt/rk3588/releases/<ID> [rtk line]
set -eo pipefail
RELEASE="$(cd -- "${1:?用法: smoke.sh <发布目录> [组件...]}" && pwd)"
shift
source "$RELEASE/radar_system/ros_env.sh"
set -u
export RK3588_ROBOT_CONFIG="${RK3588_ROBOT_CONFIG:-$RELEASE/robot_core/robot.json}"
export RK3588_MANAGED_FOLLOWER=1
if [ "${ROS_DOMAIN_ID:-0}" = 97 ]; then export ROS_DOMAIN_ID=98; else export ROS_DOMAIN_ID=97; fi
export ROS_LOCALHOST_ONLY=1
exec timeout 60 "$RK3588_PYTHON" -u "$RELEASE/deployment/smoke_nodes.py" "$@"
