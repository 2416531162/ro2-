#!/bin/bash
source /opt/ros/jazzy/setup.bash
cd /root/wheeltec
exec python3 -u wheeltec_driver.py "$@"
