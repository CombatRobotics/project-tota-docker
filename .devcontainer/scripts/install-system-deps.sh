#!/bin/bash
# Install core system dependencies that rarely change
# These are cached and only rebuilt when this script changes

set -e

echo "=== Installing System Dependencies ==="

apt-get update

# Core development tools
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    nano \
    vim \
    htop \
    net-tools \
    iputils-ping \
    usbutils \
    software-properties-common \
    apt-transport-https \
    ca-certificates \
    gnupg \
    lsb-release \
    locales

# Python base
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    python3-all-dev

# Version control and build tools
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    python3-vcstool \
    python3-colcon-common-extensions \
    python3-rosdep

# Serial and USB communication
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    python3-serial \
    python3-pyserial \
    setserial \
    udev \
    libudev-dev

# Archive and compression tools
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    zip \
    unzip \
    tar \
    bzip2 \
    xz-utils

# Cleanup
rm -rf /var/lib/apt/lists/*

echo "=== System Dependencies Installed ==="