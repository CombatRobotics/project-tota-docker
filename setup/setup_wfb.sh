#!/bin/bash

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_AS_USER="${SUDO_USER:-$USER}"
USER_HOME=$(eval echo ~"${RUN_AS_USER}")

echo "Installing wfb-ng"
echo "Ensure the Wi-Fi adapter is connected."
read -p "Do you want to continue? (y/n) " -n 1 -r; echo
[[ $REPLY =~ ^[Yy]$ ]] || { echo "Skipping wfb-ng installation"; exit 0; }

ifconfig
echo -n "Enter Wi-Fi interface name (e.g. wlan0): "
read WIFI_INTERFACE

# Validate interface
if ! ip link show "$WIFI_INTERFACE" >/dev/null 2>&1; then
    echo "Error: Interface $WIFI_INTERFACE not found"
    echo "Available interfaces:"
    ip link show | grep -E '^[0-9]+:' | cut -d: -f2
    exit 1
fi

echo "Using interface: $WIFI_INTERFACE"

# Clone and install wfb-ng
mkdir -p "$USER_HOME/.tota/tota_gs"
cd "$USER_HOME/.tota/tota_gs"
git clone -b stable https://github.com/svpcom/wfb-ng.git
cd wfb-ng
sudo ./scripts/install_gs.sh "$WIFI_INTERFACE"

# Configure channel
echo -n "Enter Wi-Fi channel (positive integer): "
read WIFI_CHANNEL
if ! [[ "$WIFI_CHANNEL" =~ ^[1-9][0-9]*$ ]]; then
    echo "Error: Invalid channel"
    exit 1
fi

# Write config
sudo cp /etc/wifibroadcast.cfg "/etc/wifibroadcast.cfg.bak.$(date +%Y%m%d%H%M%S)" 2>/dev/null || true
sudo tee /etc/wifibroadcast.cfg >/dev/null <<EOF
[common]
wifi_channel = $WIFI_CHANNEL
wifi_region = 'BO'

[gs_mavlink]
peer = 'listen://127.0.0.1:14550'
peer = 'connect://127.0.0.1:14551'

[gs_video]
peer = 'connect://127.0.0.1:5600'
EOF

echo "Wi-Fi channel set to: $WIFI_CHANNEL"

# Keys and non-root fix (use SSH as the invoking user)
cd "$USER_HOME/.tota/tota_gs"
sudo -u "$RUN_AS_USER" -H env SSH_AUTH_SOCK="$SSH_AUTH_SOCK" git clone git@github.com:CombatRobotics/wfb_keys.git || true
if [ -d "$USER_HOME/.tota/tota_gs/wfb_keys" ]; then
  sudo cp "$USER_HOME/.tota/tota_gs/wfb_keys/gs.key" /etc/
else
  echo "Warning: wfb_keys repo not available; skipping key copy and sudo fix."
fi

# Enable services
sudo systemctl enable wifibroadcast.service
sudo systemctl enable wifibroadcast@gs.service

# Template-wide restart policy
sudo mkdir -p /etc/systemd/system/wifibroadcast@.service.d
sudo tee /etc/systemd/system/wifibroadcast@.service.d/override.conf >/dev/null <<EOF
[Service]
Restart=on-failure
RestartSec=5s

[Unit]
After=network-online.target
Wants=network-online.target
EOF

# Apply changes and start
sudo systemctl daemon-reload
sudo systemctl restart wifibroadcast@gs
