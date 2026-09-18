#!/usr/bin/env bash
# 录制跟随现场,供 bag_replay.py 离线回放。Ctrl+C 结束。
#   bash radar_system/record_follow_bag.sh [输出目录]
# 只录回放需要的小话题(不录图像),10 分钟通常只有几十 MB。
set -e
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ODOM_TOPICS="$(PYTHONPATH="$ROOT${PYTHONPATH:+:$PYTHONPATH}" python3 -c 'from robot_core.config import PROFILE; print(" ".join(dict.fromkeys([PROFILE["localization"]["driver_topic"], PROFILE["localization"]["topic"]])))')"
read -r -a ODOM_ARGS <<< "$ODOM_TOPICS"
OUT_DIR="${1:-$HOME/bags}"
mkdir -p "$OUT_DIR"
NAME="$OUT_DIR/follow_$(date +%Y%m%d_%H%M%S)"
echo "录制到 $NAME (Ctrl+C 结束)"
echo "回放: python3 $(dirname "$0")/bag_replay.py $NAME"
exec ros2 bag record -o "$NAME" \
    /scan \
    /camera/ai_detection/targets \
    /wheeltec/status \
    /voltage \
    /follower/status \
    /motion/status \
    /follow/command \
    /manual/command \
    /navigation/command \
    "${ODOM_ARGS[@]}" \
    /imu \
    /tf \
    /tf_static
