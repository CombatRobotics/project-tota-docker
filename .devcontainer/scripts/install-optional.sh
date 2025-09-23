#!/bin/bash
# Install optional components based on build arguments
# This allows selective installation without rebuilding base layers

set -e

echo "=== Installing Optional Components ==="

# Claude AI Assistant
if [ "${INSTALL_CLAUDE}" = "true" ]; then
    echo "Installing Claude AI Assistant..."
    curl -fsSL https://deb.nodesource.com/setup_20.x | bash -
    apt-get update
    DEBIAN_FRONTEND=noninteractive apt-get install -y nodejs
    npm install -g @anthropic-ai/claude-code
    echo "Claude AI Assistant installed"
fi

# ROCm for AMD GPU support
if [ "${INSTALL_ROCM}" = "true" ]; then
    echo "Installing ROCm support..."
    wget -q https://repo.radeon.com/amdgpu-install/6.4.2/ubuntu/jammy/amdgpu-install_6.4.60402-1_all.deb
    apt-get update
    DEBIAN_FRONTEND=noninteractive apt-get install -y ./amdgpu-install_6.4.60402-1_all.deb
    DEBIAN_FRONTEND=noninteractive apt-get install -y rocm
    rm -f amdgpu-install_6.4.60402-1_all.deb
    echo "ROCm support installed"
fi

# WiFi Broadcast dependencies
if [ "${INSTALL_WFB}" = "true" ]; then
    echo "Installing WFB dependencies..."
    apt-get update
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        dkms \
        fakeroot \
        debhelper \
        python3-twisted \
        libpcap-dev \
        python3-pyroute2 \
        python3-configparser \
        libsodium-dev
    echo "WFB dependencies installed"
fi

# Cleanup
rm -rf /var/lib/apt/lists/*

echo "=== Optional Components Installation Complete ==="