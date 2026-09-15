#!/bin/bash
# RK3588 激光雷达 + 3D 深度相机联合 3D 栅格体素建图启动器

source /opt/ros/jazzy/setup.bash
cd /root/radar_system

echo "=========================================================="
echo " 正在启动 RK3588 雷达+深度相机 联合 3D 体素栅格建图 (OctoMap)..."
echo "=========================================================="

# 1. 启动多源点云空间坐标系对齐与融合节点
/usr/bin/python3 -u joint_3d_mapping_node.py > /tmp/joint_fusion.log 2>&1 &
PID_FUSION=$!
echo "  [1/2] 点云融合与 TF 节点已启动 (PID: $PID_FUSION)"

sleep 2

# 2. 启动 OctoMap 3D 体素建图引擎
ros2 run octomap_server octomap_server_node \
    --ros-args \
    -p frame_id:=map \
    -p resolution:=0.10 \
    -p sensor_model/max_range:=6.0 \
    -p latch:=true \
    --remap cloud_in:=/fused_pointcloud > /tmp/octomap.log 2>&1 &
PID_OCTO=$!
echo "  [2/2] OctoMap 3D 体素建图引擎已启动 (PID: $PID_OCTO)"
echo "=========================================================="
echo " 3D 建图话题就绪:"
echo "   - /octomap_full: 完整 3D 八叉树体素地图"
echo "   - /occupied_cells_vis_array: 3D 带高度着色立方体网格"
echo "   - /projected_map: 融合高度障碍物的 2D 复合栅格地图"
echo "   - /joint_mapping/status: 实时融合性能与障碍物高度诊断"
echo "=========================================================="

trap "kill -9 $PID_FUSION $PID_OCTO 2>/dev/null; exit 0" INT TERM EXIT
wait
