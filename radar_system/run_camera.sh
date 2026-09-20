#!/bin/bash
# RGB-D frames are used only for person tracking; no point-cloud generation.
DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
set -e
source "$DIR/ros_env.sh"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"

if [ "$(id -u)" -eq 0 ]; then
    rm -f /dev/shm/sem.astra_device_sem 2>/dev/null || true
    camera_env=(HOME=/home/wheeltec USER=wheeltec
      RK3588_RUNTIME_RESOLVED=1 "ROBOT_RUNTIME_ENV=$ROBOT_RUNTIME_ENV"
      "RK3588_RUNTIME_ENV_SOURCE=$RK3588_RUNTIME_ENV_SOURCE"
      "RK3588_ROS_ROOT=$RK3588_ROS_ROOT" "ROS_DISTRO=$ROS_DISTRO"
      "RK3588_PYTHON=$RK3588_PYTHON" "RK3588_POSE_MODEL=$RK3588_POSE_MODEL"
      "RK3588_ROBOT_CONFIG=$RK3588_ROBOT_CONFIG" "RK3588_DEPTH_PATH_CONFIG=$RK3588_DEPTH_PATH_CONFIG"
      "RK3588_CAMERA_WORKSPACE=${RK3588_CAMERA_WORKSPACE:-/home/wheeltec/install/setup.bash}"
      "ROS_AUTOMATIC_DISCOVERY_RANGE=$ROS_AUTOMATIC_DISCOVERY_RANGE")
    for key in ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE; do
        if [ "${!key+x}" ]; then camera_env+=("$key=${!key}"); fi
    done
    exec sudo -u wheeltec env "${camera_env[@]}" bash "$DIR/run_camera.sh" "$@"
fi

cd "$DIR"
rm -f /dev/shm/sem.astra_device_sem 2>/dev/null || true

CAMERA_WORKSPACE="${RK3588_CAMERA_WORKSPACE:-/home/wheeltec/install/setup.bash}"
if [ -r "$CAMERA_WORKSPACE" ]; then
    selected="$ROS_DISTRO"
    source "$CAMERA_WORKSPACE"
    if [ "$ROS_DISTRO" != "$selected" ]; then
        echo "Camera workspace ROS conflict: selected $selected, $CAMERA_WORKSPACE set $ROS_DISTRO" >&2
        exit 1
    fi
    echo "Camera workspace: $CAMERA_WORKSPACE (ROS_DISTRO=$ROS_DISTRO)" >&2
fi

if ros2 pkg prefix astra_camera >/dev/null 2>&1; then
    exec ros2 launch astra_camera astra_mini.launch.py "$@"
else
    exec ros2 launch openni2_camera camera_only.launch.py "$@"
fi
