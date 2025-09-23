#!/bin/bash
# Source ROS2 workspaces in correct order

[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f /opt/gz_ros2_control_ws/install/setup.bash ] && source /opt/gz_ros2_control_ws/install/setup.bash
[ -f /ros2_ws/project_tota/ws/controller_ws/install/setup.bash ] && source /ros2_ws/project_tota/ws/controller_ws/install/setup.bash
[ -f /ros2_ws/project_tota/install/setup.bash ] && source /ros2_ws/project_tota/install/setup.bash