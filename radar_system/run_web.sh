#!/bin/bash
source /opt/ros/jazzy/setup.bash
cd /root/radar_system
exec /usr/bin/python3 -u radar_web_server.py
