#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
cd /root/wheeltec
exec python3 -u wheeltec_driver.py --ros-args --params-file /root/wheeltec/wheeltec.yaml "$@"
