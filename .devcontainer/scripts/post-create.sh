#!/bin/bash
# Post-create: Initialize container

set -e

# Source ROS environment
source /opt/ros/humble/setup.bash

# Run workspace setup
/ros2_ws/project_tota/.devcontainer/scripts/workspace-setup.sh all

# Setup udev rules if present
if [ -f /ros2_ws/project_tota/setup/tota-tx.rules ]; then
    sudo cp /ros2_ws/project_tota/setup/tota-tx.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
fi

# Add user to dialout group for serial access
groups | grep -q dialout || sudo usermod -a -G dialout $(whoami)

echo "DevContainer initialization complete"