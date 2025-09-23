#!/bin/bash
# Configure essential environment variables in bashrc

set -e

cat >> ~/.bashrc << 'EOF'

# ROS2 Environment
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PROJECT_TOTA_PATH=/ros2_ws/project_tota

# Display
export DISPLAY=${DISPLAY:-:0}
export QT_X11_NO_MITSHM=1
export QT_QPA_PLATFORM=xcb

# Gazebo resource paths
export GZ_SIM_RESOURCE_PATH=/opt/gz_ros2_control_ws/install/share:/ros2_ws/project_tota/install/share:/ros2_ws/project_tota/src:$GZ_SIM_RESOURCE_PATH

# Source workspaces
[ -f /ros2_ws/project_tota/.devcontainer/scripts/environment.sh ] && source /ros2_ws/project_tota/.devcontainer/scripts/environment.sh

EOF

# Add ROCm config if enabled
if [ "${INSTALL_ROCM}" = "true" ]; then
    echo "export HSA_OVERRIDE_GFX_VERSION=11.0.2" >> ~/.bashrc
fi