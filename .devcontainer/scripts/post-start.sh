#!/bin/bash
# Post-start: Quick setup on container start

# Setup X11
touch ~/.Xauthority 2>/dev/null
xhost +local:root 2>/dev/null || true

# Source workspaces
source /opt/ros/humble/setup.bash 2>/dev/null
[ -f "/ros2_ws/project_tota/ws/controller_ws/install/setup.bash" ] && source /ros2_ws/project_tota/ws/controller_ws/install/setup.bash 2>/dev/null
[ -f "/ros2_ws/project_tota/install/setup.bash" ] && source /ros2_ws/project_tota/install/setup.bash 2>/dev/null