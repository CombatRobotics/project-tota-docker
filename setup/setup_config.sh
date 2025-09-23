#!/bin/bash

# TODO: Add cyclone dds setup to the config setup
# TODO: Check if the device ip is set correctly
# TODO: If not set the device ip as static ip
# TODO: If the check if the domain id is set correctly


if [ -z "$1" ]; then
    echo "Error: Device type not specified"
    echo "Usage: $0 <device_type>"
    exit 1
fi

device_type=$1

echo -e "\nSetting up device configuration..."
read -p "Do you want to continue? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo -e "\nSkipping device configuration setup\n"
    exit 0
fi

sudo apt-get install -y net-tools
ifconfig

# Get configuration values from user
read -p "Enter ${device_type} name: " device_name
read -p "Enter ${device_type} IP address: " device_ip
read -p "Enter ROS domain ID: " domain_id
read -p "Enter ${device_type} type: " device_type_name
read -p "Enter network interface (use ifconfig to find): " network_interface

# Set config file path in main directory
config_file="${PWD}/ws/controller_ws/src/tota_controller_core/config/robot_config.yaml"

# Get current IP address of network interface
current_ip=$(ip addr show $network_interface 2>/dev/null | grep -oP '(?<=inet\s)\d+(\.\d+){3}')

if [ -z "$current_ip" ]; then
    echo -e "\n \e[33m Warning: Could not find IP address for interface $network_interface \e[0m"  
    echo "Please check if the interface name is correct and interface exists"
elif [ "$current_ip" != "$device_ip" ]; then
    echo -e "\n \e[33m Warning: Configured IP ($device_ip) does not match current IP on $network_interface ($current_ip)\e[0m"
    echo "Please configure the network interface with the correct IP address"
fi

# Check if config file already exists
if [ -f "$config_file" ]; then
    echo -e "\n \e[33m Warning: ${config_file} already exists \e[0m"
    read -p "Do you want to overwrite the existing file? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo -e "\nSkipping config file creation\n"
    else
        # Create device config file
        cat > "$config_file" << EOL
device_name: "${device_name}"
device_ip: "${device_ip}"
domain_id: "${domain_id}"
device_type: "${device_type_name}"
device_network_interface: "${network_interface}"
EOL
    fi
else
    # Create device config file
    cat > "$config_file" << EOL
device_name: "${device_name}"
device_ip: "${device_ip}"
domain_id: "${domain_id}"
device_type: "${device_type_name}"
device_network_interface: "${network_interface}"
EOL
fi

# Only add export if not already present in ~/.bashrc
if ! grep -q "^export ROS_DOMAIN_ID=" ~/.bashrc; then
    echo -e "\nexport ROS_DOMAIN_ID=$domain_id" >> ~/.bashrc
else
    echo "ROS_DOMAIN_ID already set in ~/.bashrc, not adding duplicate."
fi