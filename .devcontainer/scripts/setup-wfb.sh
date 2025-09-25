#!/bin/bash
# Setup WFB-ng for TOTA project in devcontainer

set -e

echo "═══════════════════════════════════════════════════════════════"
echo "Setting up WFB-ng (WifiBroadcast) for TOTA project..."
echo "═══════════════════════════════════════════════════════════════"

# Constants
WFB_DIR="/opt/wfb-ng"
TOTA_GS_DIR="/root/.tota/tota_gs"
WORKSPACE="/ros2_ws/project_tota"

# WFB Configuration - CHANGE THESE SETTINGS HERE
WFB_CHANNEL=165
WFB_REGION='BO'
WFB_DEFAULT_INTERFACE='wlxfc221c40058c'  # Default WiFi interface for WFB

# Function to install WFB-ng dependencies
install_wfb_dependencies() {
    echo "Installing WFB-ng dependencies..."
    apt-get update
    apt-get install -y \
        python3-all \
        python3-twisted \
        python3-pyroute2 \
        python3-future \
        python3-packaging \
        python3-pip \
        python3-serial \
        python3-numpy \
        python3-prctl \
        python3-pytest \
        python3-stdeb \
        python3-virtualenv \
        libpcap-dev \
        libsodium-dev \
        iw \
        wireless-tools \
        rfkill \
        net-tools \
        build-essential \
        cmake \
        git

    # Install Python packages for WFB
    pip3 install --no-cache-dir \
        pyroute2 \
        msgpack \
        twisted \
        stdeb
}

# Function to build and install WFB-ng
install_wfb_ng() {
    echo "Cloning and building WFB-ng..."

    # Create directories
    mkdir -p "$TOTA_GS_DIR"
    mkdir -p "$WFB_DIR"

    # Clone WFB-ng repository
    if [ ! -d "$TOTA_GS_DIR/wfb-ng" ]; then
        cd "$TOTA_GS_DIR"
        git clone -b stable https://github.com/svpcom/wfb-ng.git
    else
        echo "WFB-ng repository already exists, updating..."
        cd "$TOTA_GS_DIR/wfb-ng"
        git pull
    fi

    cd "$TOTA_GS_DIR/wfb-ng"

    # Build WFB-ng
    echo "Building WFB-ng..."
    make clean || true
    make all

    # Install WFB-ng binaries
    echo "Installing WFB-ng binaries..."
    make install || {
        # Manual installation if make install fails
        cp wfb_rx /usr/local/bin/ || true
        cp wfb_tx /usr/local/bin/ || true
        cp wfb_keygen /usr/local/bin/ || true
        chmod +x /usr/local/bin/wfb_* || true
    }

    # Install Python package for wfb-cli
    echo "Installing WFB-ng Python package..."
    export VERSION="25.1.1"
    export COMMIT=$(git rev-parse HEAD 2>/dev/null || echo "unknown")
    pip3 install --root-user-action=ignore -e . || {
        echo "Warning: Python package installation failed, wfb-cli may not be available"
    }

    # Install Python scripts
    if [ -d "scripts" ]; then
        cp scripts/*.py /usr/local/bin/ || true
        chmod +x /usr/local/bin/*.py || true
    fi
}

# Function to setup WFB configuration
setup_wfb_config() {
    echo "Setting up WFB-ng configuration..."

    # Create configuration directory
    mkdir -p /etc/wifibroadcast

    # Simple configuration matching your actual setup
    cat > /etc/wifibroadcast.cfg <<EOF
[common]
wifi_channel = ${WFB_CHANNEL}
wifi_region = '${WFB_REGION}'

[gs_mavlink]
peer = 'connect://127.0.0.1:14550'

[gs_video]
peer = 'connect://127.0.0.1:5600'
EOF

    echo "WFB-ng configuration created at /etc/wifibroadcast.cfg"
}

# Function to generate WFB keys
generate_wfb_keys() {
    echo "Generating WFB-ng encryption keys..."

    # Check if wfb_keygen exists
    if command -v wfb_keygen >/dev/null 2>&1; then
        # Generate keys if they don't exist
        if [ ! -f /etc/gs.key ]; then
            wfb_keygen | tee /etc/gs.key /etc/drone.key
            echo "✓ WFB-ng keys generated"
        else
            echo "✓ WFB-ng keys already exist"
        fi
    else
        echo "⚠ wfb_keygen not found, creating placeholder keys"
        # Create placeholder keys
        dd if=/dev/urandom bs=32 count=1 2>/dev/null | base64 > /etc/gs.key
        cp /etc/gs.key /etc/drone.key
    fi

    # Set proper permissions
    chmod 600 /etc/gs.key /etc/drone.key
}

# Function to setup WFB service scripts
setup_wfb_services() {
    echo "Setting up WFB-ng service scripts..."

    # Create start script for ground station
    cat > /usr/local/bin/start_wfb_gs.sh <<EOF
#!/bin/bash
# Start WFB-ng ground station services

WIFI_INTERFACE="\${1:-${WFB_DEFAULT_INTERFACE}}"
echo "Starting WFB-ng ground station on $WIFI_INTERFACE..."

# Put interface in monitor mode
ip link set $WIFI_INTERFACE down
iw dev $WIFI_INTERFACE set monitor none
ip link set $WIFI_INTERFACE up
iw dev \$WIFI_INTERFACE set channel ${WFB_CHANNEL} HT20

# Start WFB receivers
wfb_rx -K /etc/gs.key -p 65 -c 127.0.0.1 -u 14550 $WIFI_INTERFACE &
wfb_rx -K /etc/gs.key -p 90 -c 127.0.0.1 -u 5600 $WIFI_INTERFACE &

# Start WFB transmitters
wfb_tx -K /etc/gs.key -p 66 -u 14551 $WIFI_INTERFACE &
wfb_tx -K /etc/gs.key -p 91 -u 5601 $WIFI_INTERFACE &

echo "WFB-ng ground station started"
EOF

    # Create start script for drone
    cat > /usr/local/bin/start_wfb_drone.sh <<EOF
#!/bin/bash
# Start WFB-ng drone services

WIFI_INTERFACE="\${1:-${WFB_DEFAULT_INTERFACE}}"
echo "Starting WFB-ng drone on $WIFI_INTERFACE..."

# Put interface in monitor mode
ip link set $WIFI_INTERFACE down
iw dev $WIFI_INTERFACE set monitor none
ip link set $WIFI_INTERFACE up
iw dev \$WIFI_INTERFACE set channel ${WFB_CHANNEL} HT20

# Start WFB receivers
wfb_rx -K /etc/drone.key -p 66 -c 127.0.0.1 -u 14550 $WIFI_INTERFACE &
wfb_rx -K /etc/drone.key -p 91 -c 127.0.0.1 -u 5601 $WIFI_INTERFACE &

# Start WFB transmitters
wfb_tx -K /etc/drone.key -p 65 -u 14550 $WIFI_INTERFACE &
wfb_tx -K /etc/drone.key -p 90 -u 5602 $WIFI_INTERFACE &

echo "WFB-ng drone started"
EOF

    chmod +x /usr/local/bin/start_wfb_*.sh
    echo "✓ WFB-ng service scripts created"
}

# Function to setup container-specific configurations
setup_container_provisions() {
    echo "Setting up container-specific provisions for WFB-ng..."

    # Create a script to check USB WiFi adapter from container
    cat > /usr/local/bin/check_wfb_adapter.sh <<'EOF'
#!/bin/bash
# Check for WiFi adapters accessible from container

echo "Checking for WiFi adapters accessible from container..."
echo ""
echo "USB devices visible in container:"
if [ -e /dev/bus/usb ]; then
    lsusb 2>/dev/null || echo "lsusb not available, install usbutils"
else
    echo "USB devices not accessible. Container needs --privileged or device mounting"
fi

echo ""
echo "Network interfaces:"
ip link show | grep -E "^[0-9]+: " | cut -d: -f2

echo ""
echo "Wireless interfaces:"
iw dev 2>/dev/null || echo "No wireless interfaces found or iw not available"

echo ""
echo "To use WFB-ng in container, ensure:"
echo "1. Container is run with --privileged flag"
echo "2. WiFi adapter is passed through with --device=/dev/bus/usb"
echo "3. Network capabilities with --cap-add=NET_ADMIN"
echo "4. Host network mode with --network=host (recommended)"
EOF

    chmod +x /usr/local/bin/check_wfb_adapter.sh

    # Create helper script for container WFB usage
    cat > /usr/local/bin/wfb_container_setup.sh <<EOF
#!/bin/bash
# Setup WiFi adapter for WFB in container

INTERFACE="\${1:-${WFB_DEFAULT_INTERFACE}}"

echo "Setting up $INTERFACE for WFB-ng in container..."

# Check if interface exists
if ! ip link show "$INTERFACE" >/dev/null 2>&1; then
    echo "Error: Interface $INTERFACE not found"
    echo "Available interfaces:"
    ip link show | grep -E "^[0-9]+: " | cut -d: -f2
    exit 1
fi

# Try to set monitor mode (may fail without proper permissions)
echo "Attempting to set monitor mode..."
ip link set "$INTERFACE" down 2>/dev/null || {
    echo "Cannot bring interface down. Need NET_ADMIN capability"
    echo "Container must be run with --cap-add=NET_ADMIN"
    exit 1
}

iw dev "$INTERFACE" set monitor none 2>/dev/null || {
    echo "Cannot set monitor mode. Possible issues:"
    echo "1. Driver doesn't support monitor mode"
    echo "2. Need --privileged flag"
    echo "3. Interface not properly passed to container"
    exit 1
}

ip link set "\$INTERFACE" up
iw dev "\$INTERFACE" set channel ${WFB_CHANNEL} HT20

echo "✓ Interface $INTERFACE configured for WFB-ng"
EOF

    chmod +x /usr/local/bin/wfb_container_setup.sh

    # Note about container limitations
    echo "✓ Container provision scripts created"
    echo ""
    echo "IMPORTANT: For WFB-ng to work in Docker container:"
    echo "1. Run container with: --privileged --network=host --cap-add=NET_ADMIN"
    echo "2. Mount USB devices: --device=/dev/bus/usb"
    echo "3. Pass through WiFi adapter device if using specific device"
    echo ""
    echo "Helper scripts created:"
    echo "  - check_wfb_adapter.sh : Check adapter visibility"
    echo "  - wfb_container_setup.sh : Setup adapter for WFB"
}

# Main installation flow
main() {
    echo "Starting WFB-ng installation for TOTA project..."

    # Install dependencies
    install_wfb_dependencies

    # Build and install WFB-ng
    install_wfb_ng

    # Setup configuration
    setup_wfb_config

    # Generate keys
    generate_wfb_keys

    # Setup service scripts
    setup_wfb_services

    # Setup container-specific provisions
    setup_container_provisions

    echo ""
    echo "═══════════════════════════════════════════════════════════════"
    echo "✓ WFB-ng installation complete!"
    echo "═══════════════════════════════════════════════════════════════"
    echo ""
    echo "Current Settings:"
    echo "  Channel: ${WFB_CHANNEL}"
    echo "  Region: ${WFB_REGION}"
    echo "  Default Interface: ${WFB_DEFAULT_INTERFACE}"
    echo ""
    echo "Usage:"
    echo "  Ground Station: start_wfb_gs.sh [interface]"
    echo "  Drone:         start_wfb_drone.sh [interface]"
    echo ""
    echo "Configuration: /etc/wifibroadcast.cfg"
    echo "Keys: /etc/gs.key, /etc/drone.key"
    echo ""
    echo "To change channel or interface, edit variables at top of this script"
    echo "═══════════════════════════════════════════════════════════════"
}

# Run main installation
main "$@"