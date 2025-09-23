#!/bin/bash
# Install ROS2 packages from packages.txt
# This layer rebuilds when packages.txt changes

set -e

echo "=== Installing ROS2 Packages ==="

if [ ! -f /tmp/packages.txt ]; then
    echo "Warning: packages.txt not found, skipping ROS2 package installation"
    exit 0
fi

apt-get update

# Install packages from packages.txt, ignoring comments and empty lines
grep -v '^#' /tmp/packages.txt | grep -v '^$' | while read -r package; do
    echo "Installing: $package"
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "$package" || {
        echo "Warning: Failed to install $package, continuing..."
    }
done

# Always install CycloneDDS (core requirement for TOTA)
echo "=== Installing CycloneDDS ==="
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-humble-cyclonedds \
    ros-humble-rmw-cyclonedds-cpp

# Cleanup
rm -rf /var/lib/apt/lists/*

echo "=== ROS2 Packages Installed ==="