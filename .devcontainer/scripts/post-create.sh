#!/bin/bash
# Post-create: Initialize container with ROS2 workspace and repositories

set -e

# Constants
WORKSPACE="/ros2_ws/project_tota"
CONTROLLER_WS="${WORKSPACE}/ws/controller_ws"
ROS_DISTRO="humble"

echo "=== Initializing ROS2 Development Environment ==="

# Ensure Claude persistence (will check and restore)
if [ -f "${WORKSPACE}/.devcontainer/scripts/preserve-claude.sh" ]; then
    ${WORKSPACE}/.devcontainer/scripts/preserve-claude.sh
fi

# Initialize rosdep if needed
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "Initializing rosdep..."
    sudo rosdep init || true
fi

# Update rosdep
echo "Updating rosdep..."
rosdep update --rosdistro ${ROS_DISTRO} || true

# Update apt cache
echo "Updating apt cache..."
sudo apt-get update -qq

# Import repositories from ally.repos if it exists
if [ -f "${WORKSPACE}/repos/ally.repos" ]; then
    echo "Importing repositories from ally.repos..."
    mkdir -p "${CONTROLLER_WS}/src"
    cd "${CONTROLLER_WS}/src"

    # Check if vcs is installed
    if ! command -v vcs &> /dev/null; then
        echo "Installing vcstool..."
        sudo apt-get update && sudo apt-get install -y python3-vcstool
    fi

    # Import repositories
    vcs import --recursive < "${WORKSPACE}/repos/ally.repos" || true

    # Install dependencies and build controller workspace
    cd "${CONTROLLER_WS}"
    echo "Installing dependencies for controller workspace..."
    rosdep install --from-paths src --ignore-src -r -y || true

    echo "Building controller workspace..."
    source /opt/ros/humble/setup.bash
    # Ensure correct package versions for colcon build
    python3 -c "import setuptools; exit(0 if int(setuptools.__version__.split('.')[0]) < 76 else 1)" 2>/dev/null || \
        pip3 install "setuptools<76" "packaging==23.2" --quiet

    # Set numpy include path for ROS2 interface compilation
    export NUMPY_INCLUDE_PATH="/usr/local/lib/python3.10/dist-packages/numpy/core/include"
    export CFLAGS="-I${NUMPY_INCLUDE_PATH}"
    export CPPFLAGS="-I${NUMPY_INCLUDE_PATH}"

    colcon build --symlink-install || true
fi

# Install dependencies for main workspace if src exists
if [ -d "${WORKSPACE}/src" ] && [ "$(ls -A ${WORKSPACE}/src 2>/dev/null)" ]; then
    echo "Installing ROS dependencies for main workspace..."
    cd ${WORKSPACE}
    rosdep install --from-paths src --ignore-src -r -y || true

    echo "Building main workspace..."
    source /opt/ros/humble/setup.bash
    [ -f "${CONTROLLER_WS}/install/setup.bash" ] && source "${CONTROLLER_WS}/install/setup.bash"
    # Ensure correct package versions for colcon build
    python3 -c "import setuptools; exit(0 if int(setuptools.__version__.split('.')[0]) < 76 else 1)" 2>/dev/null || \
        pip3 install "setuptools<76" "packaging==23.2" --quiet

    # Set numpy include path for ROS2 interface compilation
    export NUMPY_INCLUDE_PATH="/usr/local/lib/python3.10/dist-packages/numpy/core/include"
    export CFLAGS="-I${NUMPY_INCLUDE_PATH}"
    export CPPFLAGS="-I${NUMPY_INCLUDE_PATH}"

    colcon build --symlink-install || true
fi

# Setup Cyclone DDS configuration
if [ ! -f "${WORKSPACE}/cyclonedds.xml" ]; then
    echo "Creating Cyclone DDS configuration..."
    cat > "${WORKSPACE}/cyclonedds.xml" << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <Interfaces>
                <NetworkInterface name="lo" priority="default" multicast="true" />
            </Interfaces>
        </General>
    </Domain>
</CycloneDDS>
EOF
fi

# Setup udev rules if present
if [ -f "${WORKSPACE}/setup/tota-tx.rules" ]; then
    echo "Setting up udev rules..."
    sudo mkdir -p /etc/udev/rules.d/
    sudo cp "${WORKSPACE}/setup/tota-tx.rules" /etc/udev/rules.d/
    sudo udevadm control --reload-rules || true
fi

# Add user to dialout group for serial access
if ! groups | grep -q dialout; then
    echo "Adding user to dialout group..."
    sudo usermod -a -G dialout $(whoami)
fi

# Create render group if it doesn't exist (for GPU access)
if ! getent group render > /dev/null 2>&1; then
    echo "Creating render group..."
    sudo groupadd -f render
fi

# Add user to render group for GPU access
if ! groups | grep -q render; then
    echo "Adding user to render group..."
    sudo usermod -a -G render $(whoami)
fi

# Python environment already configured in Dockerfile during build (line 37)

echo "DevContainer initialization complete!"