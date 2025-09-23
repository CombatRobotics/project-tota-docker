#!/bin/bash

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Exit on error
set -e

# Default to ROS 2 Humble unless already set
ROS_DISTRO="${ROS_DISTRO:-humble}"

# Source ROS environment (needed for rosdep/colcon to resolve ament_cmake, std_msgs, etc.)
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
    echo "Error: /opt/ros/${ROS_DISTRO}/setup.bash not found. Is ROS ${ROS_DISTRO} installed?"
    exit 1
fi

# Check if repos directory exists
if [ ! -d "${repo_root}/repos" ]; then
    echo "Error: repos directory not found"
    exit 1
fi

# Check if ally.repos exists
if [ ! -f "${repo_root}/repos/ally.repos" ]; then
    echo "Error: ally.repos file not found in repos directory" 
    exit 1
fi

# Create gs_ws/src directory if it doesn't exist
# cd ..
mkdir -p ${repo_root}/ws/controller_ws/src

# Import repositories using vcs
echo "Importing repositories from ally.repos..."
vcs import ${repo_root}/ws/controller_ws/src < ${repo_root}/repos/ally.repos

# Build workspace
cd ${repo_root}/ws/controller_ws

# Initialize rosdep (only once; check if not already done)
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi

# Update rosdep database
rosdep update

# Install dependencies from source
rosdep install --from-paths src --ignore-src -r -y --rosdistro "${ROS_DISTRO}"

echo "Building workspace..."

colcon build --symlink-install

echo "Setup complete!"
