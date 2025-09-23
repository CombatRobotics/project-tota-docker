
# Tota Robot Setup Guide

This repository contains the setup instructions and configuration files for the Tota robot system.

## Setup the project

1. Clone the repository

```bash
git clone git@github.com:CombatRobotics/project-tota.git
```

2. Go to the project directory

```bash
cd project-tota
```

3. Run the setup script

```bash
./setup.sh
```

### Setup Example 

```bash

cri-ally-2@cri-ally-2:~/project-tota$ ./setup.sh 
Device Setup Script
-------------------
1) Setup device
2) Update device
3) Reset device
4) Exit
```

## Installation

### Local Installation

1. [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

2. [CycloneDDS](https://github.com/eclipse-cyclonedds/cyclonedds)
```shell
sudo apt-get install ros-humble-rmw-cyclonedds-cpp
```

3. [Pytorch](https://pytorch.org/get-started/locally/) for ROCm support
```shell
pip3 install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/rocm6.4
```
4. [Onnxruntime-rocm](https://rocm.docs.amd.com/projects/radeon/en/latest/docs/install/native_linux/install-onnx.html) for ROCm support
```shell
pip3 install onnxruntime-rocm -f https://repo.radeon.com/rocm/manylinux/rocm-rel-6.4.2/
```

5. [Gstreamer](https://gstreamer.freedesktop.org/documentation/installing/on-linux.html) for video streaming
```shell
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
```

6. [OpenIPC](https://docs.openipc.org/use-cases/fpv/groundstation-ubuntu/) for video streaming and mavlink data

7. Other python dependencies
```shell
pip3 install -r requirements.txt
```


## Fix for Wifibroadcast
### 1  Enable the units so they start at boot

```bash
# 1A. Enable the wrapper; it pulls in all instances
sudo systemctl enable wifibroadcast.service

# 1B. Enable your specific instance (replace 'gs')
sudo systemctl enable wifibroadcast@gs.service
```

### 2 Add an auto-restart policy (template-wide)
```shell
sudo systemctl edit wifibroadcast@.service
```

```ini
[Service]
# re-run the service when it exits with a non-zero status
Restart=on-failure
RestartSec=5s        # wait 5 s before trying again (optional)

[Unit]
After=network-online.target
Wants=network-online.target

```

### 3 Reload the systemd manager configuration

```bash
sudo systemctl daemon-reload
sudo systemctl start wifibroadcast.service
```

```bash
reboot
```
