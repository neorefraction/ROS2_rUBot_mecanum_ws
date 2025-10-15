#!/bin/bash
set -e

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source workspace si cal
# source /root/pc_ws/install/setup.bash

exec "$@"
