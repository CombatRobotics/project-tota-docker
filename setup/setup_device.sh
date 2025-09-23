#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

setup_device() {
    local device_type=$1
    echo -e "\nSetting up $device_type..."
    
    # Install ROS2 Humble if it does not exist
    if [ ! -d "/opt/ros/humble/" ]; then
        "$SCRIPT_DIR/install_humble.sh" "$device_type"
    else
        echo "ROS 2 Humble is already installed."
    fi

    # Setup repositories using setup_repos.sh
    "$SCRIPT_DIR/setup_repos.sh" "$device_type"

    # Source workspace and export path
    "$SCRIPT_DIR/setup_paths.sh" "$device_type" 

    # Setup cyclonedds setup
    "$SCRIPT_DIR/setup_cyclonedds.sh"

    # Install gs
    "$SCRIPT_DIR/install_gs.sh" "$device_type"

    # Install rocm
    "$SCRIPT_DIR/install_rocm.sh" "$device_type"

    # Setup python venv
    "$SCRIPT_DIR/setup_venv.sh" "$device_type"
   
    # Setup config
    # "$SCRIPT_DIR/setup_config.sh" "$device_type"
    
    # Setup WFB
    sudo "$SCRIPT_DIR/setup_wfb.sh" 

    # UI setup
    "$SCRIPT_DIR/setup_ui.sh"

    # Setup services
    "$SCRIPT_DIR/setup_services.sh" "$device_type"

    # Remove Keyrings
    "$SCRIPT_DIR/cleanup.sh"

    # Install udev rules
    sudo "$SCRIPT_DIR/udev_rules.sh"

    echo "Setup Complete."
}

# Call the function with the first argument passed to the script
setup_device "$1"