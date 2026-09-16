#!/bin/bash
# Camera driver + Depth-to-Pointcloud (XYZRGB) + Calibrated base_link -> camera_link TF
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"

PIDS=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  sleep 1
  for pid in "${PIDS[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

# Publish calibrated camera extrinsic:
# x=0.54m (forward), y=0m (center), z=0.35m (height above ground), pitch=15° (0.261799 rad downward)
ros2 run tf2_ros static_transform_publisher \
  --x 0.54 --y 0.0 --z 0.35 \
  --yaw 0.0 --pitch 0.261799 --roll 0.0 \
  --frame-id base_link --child-frame-id camera_link &
PIDS+=("$!")

# 1. Launch Astra S camera driver
ros2 launch openni2_camera camera_only.launch.py &
PIDS+=("$!")

# 2. Wait for camera driver to initialize device
sleep 3

# 3. Launch depth_image_proc XYZRGB node with approximate time sync (exact_sync:=false)
ros2 run depth_image_proc point_cloud_xyzrgb_node --ros-args \
  -p exact_sync:=false \
  -p queue_size:=30 \
  -r rgb/camera_info:=/camera/rgb/camera_info \
  -r rgb/image_rect_color:=/camera/rgb/image_raw \
  -r depth_registered/image_rect:=/camera/depth_raw/image \
  -r points:=/camera/depth_registered/points &
PIDS+=("$!")

wait -n "${PIDS[@]}"
