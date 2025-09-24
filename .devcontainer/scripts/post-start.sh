#!/bin/bash
# Post-start: Quick environment setup with workspace sourcing

set -e

# Constants
WORKSPACE="/ros2_ws/project_tota"
CONTROLLER_WS="${WORKSPACE}/ws/controller_ws"

# Export PROJECT_TOTA_PATH for launch files
export PROJECT_TOTA_PATH="${WORKSPACE}"

# Link persistent bash history if it exists
if [ -f /root/.bash_history_persistent ]; then
    ln -sf /root/.bash_history_persistent /root/.bash_history
fi

# Setup X11 if available
if [ -n "${DISPLAY:-}" ]; then
    touch ~/.Xauthority 2>/dev/null || true
    xhost +local:root 2>/dev/null || true
fi

# Source ROS and workspaces if built
if [ -f /opt/ros/humble/setup.bash ]; then
    set +u  # Disable unbound variable check for ROS sourcing
    source /opt/ros/humble/setup.bash

    # Source controller workspace if it exists
    if [ -f "${CONTROLLER_WS}/install/setup.bash" ]; then
        source "${CONTROLLER_WS}/install/setup.bash"
    fi

    # Source main workspace if it exists
    if [ -f "${WORKSPACE}/install/setup.bash" ]; then
        source "${WORKSPACE}/install/setup.bash"
    fi
    set -u
fi

# Set Cyclone DDS environment if config exists
if [ -f "${WORKSPACE}/cyclonedds.xml" ]; then
    export CYCLONEDDS_URI="file://${WORKSPACE}/cyclonedds.xml"
fi

# Welcome message
echo "╔════════════════════════════════════════════════════════╗"
echo "║         TOTA ROS2 Development Container Ready          ║"
echo "╠════════════════════════════════════════════════════════╣"
echo "║ ROS: Humble     Gazebo: Harmonic    RMW: CycloneDDS    ║"
echo "╚════════════════════════════════════════════════════════╝"

# Show workspace status
if [ -d "${CONTROLLER_WS}/install" ]; then
    echo "✓ Controller workspace built"
fi
if [ -d "${WORKSPACE}/install" ]; then
    echo "✓ Main workspace built"
fi
if [ -f "${WORKSPACE}/cyclonedds.xml" ]; then
    echo "✓ Cyclone DDS configured"
fi

# Check GPU availability
if command -v nvidia-smi &> /dev/null; then
    echo "✓ NVIDIA GPU available"
elif command -v rocm-smi &> /dev/null; then
    echo "✓ AMD ROCm GPU available"
fi

# Check Python environment
if [ -d "${WORKSPACE}/rocm_venv" ]; then
    echo "✓ ROCm Python venv available at: ${WORKSPACE}/rocm_venv"
fi