#!/bin/bash

# Set error handling
set -e

: > "$HOME/.tota_startup/robot-output.txt"
: > "$HOME/.tota_startup/robot-error.txt"

echo "Starting controller script"
echo "Script path: ${BASH_SOURCE[0]}"
echo "Script realpath: $(realpath ${BASH_SOURCE[0]})"
echo "Script dirname: $(dirname $(realpath ${BASH_SOURCE[0]}))"

# Get the absolute path of the project directory
export PROJECT_TOTA_PATH="$(realpath $(dirname $(realpath ${BASH_SOURCE[0]}))/../../)"

echo "PROJECT_TOTA_PATH: $PROJECT_TOTA_PATH"
echo "User: $USER"
echo "HOME: $HOME"

# Ensure proper environment for systemd
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:$PATH"

# Set necessary environment variables for insightface
export LD_LIBRARY_PATH="$HOME/.local/lib/python3.10/site-packages/cv2/../../lib64:/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:$LD_LIBRARY_PATH"
export PYTHONPATH="$HOME/.local/lib/python3.10/site-packages:$PYTHONPATH"

echo "Activating ROS2 environment"
if [ -d "$PROJECT_TOTA_PATH/rocm_venv" ]; then
    echo "Virtual environment found at: $PROJECT_TOTA_PATH/rocm_venv"
    echo "Virtual environment contents:"
    
    # Activate virtual environment more explicitly
    echo "Activating virtual environment..."
    source "$PROJECT_TOTA_PATH/rocm_venv/bin/activate"
    echo "Virtual environment activated"
    echo "Python path: $(which python)"
    echo "Python version: $(python --version)"
    echo "VIRTUAL_ENV: $VIRTUAL_ENV"
    echo "PATH: $PATH"
    
else
    echo "Error: ROS2 environment not found at $PROJECT_TOTA_PATH/rocm_venv"
    echo "Available directories in project:"
    ls -la "$PROJECT_TOTA_PATH"
    exit 1
fi

# Source ROS2 environment
echo "Sourcing ROS2 environment"
source /opt/ros/humble/setup.bash

# Cyclone DDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "Set RMW_IMPLEMENTATION to: $RMW_IMPLEMENTATION"

echo "Sourcing controller workspace"
if [ -d "$PROJECT_TOTA_PATH/ws/controller_ws/install" ]; then
    source "$PROJECT_TOTA_PATH/ws/controller_ws/install/setup.bash"
    echo "Controller workspace sourced successfully"
else
    echo "Error: Controller workspace not found at $PROJECT_TOTA_PATH/ws/controller_ws/install"
    exit 1
fi

echo ------------------------------------------------------------------------------------------------
echo "Starting controller"
echo ------------------------------------------------------------------------------------------------

echo "Launching controller"
ros2 launch tota_controller_core initiate_tota.launch.py