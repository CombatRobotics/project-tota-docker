#!/bin/bash

echo -e "\nDo you want to setup Cyclone DDS?"
read -p "Continue? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Skipping Cyclone DDS setup"
    exit 0
fi

echo -e "\nSetting up Cyclone DDS..."

# Install Cyclone DDS
sudo apt-get install -y ros-humble-cyclonedds
sudo apt-get install -y ros-humble-rmw-cyclonedds-cpp

echo -e "\nexport RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
