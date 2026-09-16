#!/bin/bash
# Optional real 3D layer. Run real SLAM/AMCL + calibrated sensor TF FIRST.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
if [[ "${SENSOR_TF_CALIBRATED:-0}" != 1 ]]; then
  echo '3D mapping requires measured sensor transforms. See docs/LIVE_MAPPING.md.' >&2
  echo 'After calibration: SENSOR_TF_CALIBRATED=1 bash run_3d_mapping.sh' >&2
  exit 2
fi
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/ro2-mapping3d-${UID}.lock"
flock -n 9 || { echo '3D mapping already running' >&2; exit 2; }
PIDS=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  sleep 1
  for pid in "${PIDS[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
/usr/bin/python3 -u "$ROOT/joint_3d_mapping_node.py" --ros-args \
  -p depth_topic:="${DEPTH_TOPIC:-/camera/depth_registered/image_raw}" \
  -p camera_info_topic:="${DEPTH_INFO_TOPIC:-/camera/rgb/camera_info}" &
PIDS+=("$!")
ros2 run octomap_server octomap_server_node --ros-args \
  -p frame_id:=map -p resolution:=0.10 -p sensor_model.max_range:=5.5 \
  -p latch:=true -r cloud_in:=/fused_pointcloud &
PIDS+=("$!")
wait -n "${PIDS[@]}"
