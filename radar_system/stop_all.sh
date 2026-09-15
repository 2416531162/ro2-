#!/bin/bash
echo "正在停止多源感知与建图系统..."
pkill -f "rtk_node.py" 2>/dev/null
pkill -f "fake_lidar_node.py" 2>/dev/null
pkill -f "fake_slam_node.py" 2>/dev/null
pkill -f "python3 -u real_lidar_node.py" 2>/dev/null
pkill -f "radar_web_server.py" 2>/dev/null
pkill -9 -f "board_radar_gui.py" 2>/dev/null
pkill -f "ai_3d_detector.py" 2>/dev/null
pkill -f "joint_3d_mapping_node.py" 2>/dev/null
pkill -9 -f "octomap_server_node" 2>/dev/null
echo "已全部停止。"
