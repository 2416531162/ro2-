#!/bin/bash
source /opt/ros/jazzy/setup.bash
cd /root/radar_system
exec /usr/bin/python3 -u rtk_node.py
