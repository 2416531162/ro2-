#!/bin/bash
set -e
DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$DIR/ros_env.sh"
cd -- "$DIR"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"

# 强制将画面仅显示在外接 HDMI 屏幕，关闭内置屏幕 DSI
if which xrandr >/dev/null 2>&1; then
    HDMI_OUT=$(xrandr | grep -E "^HDMI-[0-9]+ connected" | awk '{print $1}' | head -n 1)
    if [ -n "$HDMI_OUT" ]; then
        echo ">>> [Display] 正在切换至外接 HDMI 屏幕: $HDMI_OUT，关闭内置屏幕 DSI-1, DSI-2..."
        xrandr --output "$HDMI_OUT" --primary --auto --pos 0x0 --output DSI-1 --off --output DSI-2 --off || true
        sleep 1
    fi
fi

# 后台等待窗口就绪并置顶/激活
(
    for i in {1..10}; do
        sleep 1
        if which xdotool >/dev/null 2>&1; then
            WID=$(xdotool search --name "RK3588 激光雷达" 2>/dev/null | tail -n 1)
            if [ -n "$WID" ]; then
                xdotool windowactivate "$WID" windowraise "$WID" 2>/dev/null || true
                break
            fi
        fi
    done
) &

exec "$RK3588_PYTHON" -u board_radar_gui.py "$@"
