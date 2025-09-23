#!/bin/bash

# Script to set up udev rules for USB serial devices
# This helps prevent permission issues when devices are disconnected/reconnected

echo "Setting up udev rules for USB serial devices..."

# Create udev rules directory if it doesn't exist
sudo mkdir -p /etc/udev/rules.d

# Use the existing TOTA rules file
sudo cp tota-tx.rules /etc/udev/rules.d/

# Add user to dialout group if not already added
if ! groups $USER | grep -q dialout; then
    echo "Adding user $USER to dialout group..."
    sudo usermod -a -G dialout $USER
    echo "User added to dialout group. You may need to log out and back in for changes to take effect."
fi

# Reload udev rules
echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Udev rules setup complete!"
echo "If you still have permission issues, try:"
echo "1. Log out and log back in (to apply group changes)"
echo "2. Unplug and replug your USB device"