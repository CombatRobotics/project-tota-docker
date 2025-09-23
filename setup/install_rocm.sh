#!/bin/bash
set -e

DEB_FILE="amdgpu-install_6.4.60402-1_all.deb"
DEB_URL="https://repo.radeon.com/amdgpu-install/6.4.2/ubuntu/jammy/${DEB_FILE}"

if [ ! -f "$DEB_FILE" ]; then
    wget "$DEB_URL"
fi

sudo apt install -y "./$DEB_FILE"
sudo apt update
sudo apt install -y python3-setuptools python3-wheel
sudo usermod -a -G render,video "$LOGNAME" # Add the current user to the render and video groups
sudo apt install -y rocm

sudo apt install -y "linux-headers-$(uname -r)" "linux-modules-extra-$(uname -r)"
sudo apt install -y amdgpu-dkms

amdgpu-install -y --usecase=graphics,rocm

sudo apt update

echo -e "\nexport HSA_OVERRIDE_GFX_VERSION=11.0.2" >> ~/.bashrc

echo "Please reboot the system"