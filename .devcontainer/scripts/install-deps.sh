#!/bin/bash
# Install all dependencies in one script
set -e

echo "Installing system dependencies..."

# Update package list
apt-get update

# Core development tools
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    ca-certificates \
    gnupg \
    lsb-release \
    software-properties-common

# Python and pip
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    python3-venv

# Fix setuptools version conflict for ROS2 colcon build compatibility
pip3 install --upgrade "setuptools<76" "packaging==23.2" || true

# Version control tools
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    python3-vcstool \
    python3-colcon-common-extensions

# Development tools
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    gdb \
    valgrind \
    ccache \
    vim \
    nano

# System monitoring
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    htop \
    tmux \
    screen \
    net-tools

# Serial communication
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    minicom \
    picocom

# Network tools
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    socat \
    netcat \
    iputils-ping \
    iproute2

# Libraries for wfb-ng and networking (from install_gs.sh)
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    libpcap-dev \
    libsodium-dev \
    dkms \
    fakeroot \
    debhelper

# Python packages for wfb-ng
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    python3-all-dev \
    python3-twisted \
    python3-pyroute2 \
    python3-future

# GStreamer for video streaming (comprehensive)
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev

# Install ROS packages from packages.txt if it exists
PACKAGES_FILE="/ros2_ws/project_tota/.devcontainer/packages.txt"
if [ -f "$PACKAGES_FILE" ]; then
    echo "Installing ROS packages..."
    # Update package lists before installing ROS packages
    apt-get update
    
    # Install packages with retry logic
    for package in $(grep -v '^#' "$PACKAGES_FILE" | grep -v '^$' | grep '^ros-'); do
        echo "Installing $package..."
        for attempt in 1 2 3; do
            if DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "$package"; then
                echo "Successfully installed $package"
                break
            else
                echo "Attempt $attempt failed for $package, retrying..."
                if [ $attempt -eq 3 ]; then
                    echo "Failed to install $package after 3 attempts, continuing..."
                fi
                sleep 2
            fi
        done
    done
fi

# Claude AI Assistant
if [ "${INSTALL_CLAUDE}" = "true" ]; then
    echo "Installing Claude AI Assistant..."
    curl -fsSL https://deb.nodesource.com/setup_20.x | bash -
    apt-get update
    DEBIAN_FRONTEND=noninteractive apt-get install -y nodejs
    npm install -g @anthropic-ai/claude-code
    echo "Claude AI Assistant installed"
fi

# Clean up
apt-get autoremove -y
apt-get clean
rm -rf /var/lib/apt/lists/*

echo "Dependencies installation complete"