#!/bin/bash
# Manual mapping / saved-map localization. Does not start sensors or move robots.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/ro2-mapping-${UID}.lock"
flock -n 9 || { echo 'Another mapping/localization session is running' >&2; exit 2; }
MODE="${1:-mapping}"
PIDS=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  for i in {1..30}; do
    alive=0
    for pid in "${PIDS[@]}"; do kill -0 "$pid" 2>/dev/null && alive=1 || true; done
    [[ "$alive" == 0 ]] && break
    sleep .1
  done
  for pid in "${PIDS[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
# Existing Wheeltec driver must own odom->base_link (publish_tf:=true).
# base_link is the REAR AXLE centre. x=0.53 is the project's measured N10P offset.
# In 2D only, zero z is a planar convention, not a measured installation height.
# Set PUBLISH_LASER_TF=0 if the URDF already owns this transform.
if [[ "${PUBLISH_LASER_TF:-1}" == 1 ]]; then
  ros2 run tf2_ros static_transform_publisher --x "${LIDAR_X_M:-0.53}" --y "${LIDAR_Y_M:-0.0}" \
    --z "${LIDAR_HEIGHT_M:-0.0}" --yaw "${LIDAR_YAW_RAD:-0.0}" --pitch 0 --roll 0 \
    --frame-id base_link --child-frame-id laser &
  PIDS+=("$!")
fi
/usr/bin/python3 -u "$ROOT/mapping_scan.py" &
PIDS+=("$!")
case "$MODE" in
  mapping)
    ros2 launch slam_toolbox online_async_launch.py slam_params_file:="$ROOT/config/slam.yaml" use_sim_time:=false & ;;
  localization)
    [[ -f "${2:-}" ]] || { echo 'Usage: run_mapping.sh localization /absolute/map.yaml' >&2; exit 2; }
    ros2 launch nav2_bringup localization_launch.py map:="$(realpath -- "$2")" \
      params_file:="$ROOT/config/localization.yaml" use_sim_time:=false autostart:=true use_composition:=false & ;;
  *) echo 'Expected mapping or localization' >&2; exit 2 ;;
esac
PIDS+=("$!")
# Exit (and clean up siblings) when any required child fails.
wait -n "${PIDS[@]}"
