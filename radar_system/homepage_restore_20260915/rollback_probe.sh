#!/bin/bash
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

source /opt/ros/jazzy/setup.bash

# 1. 强制旋转板载屏幕为 1920x1080 横屏单屏模式
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority
xhost +local:root 2>/dev/null || xhost + 2>/dev/null || true
xrandr --output DSI-1 --rotate right --output DSI-2 --off 2>/dev/null || true

# 2. 清理旧进程
pkill -f "rtk_node.py" 2>/dev/null
pkill -f "fake_lidar_node.py" 2>/dev/null
pkill -f "fake_slam_node.py" 2>/dev/null
pkill -f "radar_web_server.py" 2>/dev/null
pkill -9 -f "board_radar_gui.py" 2>/dev/null
pkill -f "python3 -u real_lidar_node.py" 2>/dev/null
pkill -f "ai_3d_detector.py" 2>/dev/null
pkill -f "joint_3d_mapping_node.py" 2>/dev/null
pkill -f "octomap_server_node" 2>/dev/null
sleep 0.5

if [ ! -e /dev/ttyACM0 ]; then
    for port in \
        /sys/bus/usb/devices/5-1.4:1.0/5-1.4-port2/disable \
        /sys/bus/usb/devices/3-1.4:1.0/3-1.4-port2/disable
    do
        if [ -e "$port" ]; then
            echo 1 > "$port"
            sleep 1
            echo 0 > "$port"
            sleep 2
        fi
    done
fi
if [ -e /dev/ttyACM0 ]; then
    chmod 666 /dev/ttyACM0
fi

echo "=========================================================="
echo " 正在启动 RK3588 多源多维智能感知系统 (LiDAR + 3D相机)..."
echo "=========================================================="

# 3. 启动相机驱动 (若未启动)
if ! pgrep -f "openni2_camera" > /dev/null; then
    nohup ros2 launch openni2_camera camera_only.launch.py > /tmp/camera.log 2>&1 &
    echo "  [1/6] 奥比中光 Astra S 3D相机驱动已启动"
else
    echo "  [1/6] 奥比中光 Astra S 3D相机驱动已在运行中"
fi

# 3. 启动 CUAV C-RTK 2HP 卫星定位定向节点
python3 -u rtk_node.py > /tmp/rtk_node.log 2>&1 &
PID_RTK=$!
echo "  [RTK] C-RTK 2HP 卫星定位定向节点已启动 (PID: $PID_RTK)"

# 4. 真实雷达 /scan
python3 -u real_lidar_node.py > /tmp/real_lidar.log 2>&1 &
PID_LIDAR=$!
echo "  [2/6] 真实雷达节点已启动 (PID: $PID_LIDAR)"

# 5. AI + 3D 物理空间测距定位引擎
nohup /root/radar_system/run_ai.sh > /tmp/ai.log 2>&1 &
PID_AI=$!
echo "  [3/6] AI + 3D 目标与人脸测距引擎已启动 (PID: $PID_AI)"

# 6. 雷达 + 深度相机 联合 3D 体素建图 (OctoMap)
nohup /root/radar_system/run_3d_mapping.sh > /tmp/run_3d.log 2>&1 &
PID_3D=$!
echo "  [4/6] 雷达+深度相机 联合 3D 体素建图已启动 (PID: $PID_3D)"

# 7. 启动 Web 科技感大屏服务
python3 -u radar_web_server.py > /tmp/radar_web.log 2>&1 &
PID_WEB=$!
echo "  [5/6] Web 建模大屏已就绪 (PID: $PID_WEB)"

# 8. 启动板载横屏全屏 PyQt5 GUI (含雷达盘与实时相机画中画)
nohup /root/radar_system/run_gui.sh > /tmp/radar_gui.log 2>&1 &
PID_GUI=$!
echo "  [6/6] 板载屏幕原生 1920x1080 UI 已启动 (PID: $PID_GUI)"

sleep 1

BOARD_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
echo "=========================================================="
echo " 🎉 系统启动完成！"
echo " 🌐 电脑浏览器打开体验: http://localhost:8088 (Mac USB 直连)"
if [ -n "$BOARD_IP" ]; then
    echo " 📶 局域网访问地址:     http://${BOARD_IP}:8088"
fi
echo " 🖥️ 板载屏幕支持: [AI 3D 测距] / [彩色实景] / [深度热力图] 一键切换"
echo " 🗺️ 3D 体素栅格 (/octomap_full) & 复合投影地图 (/projected_map) 持续构建中"
echo "=========================================================="
