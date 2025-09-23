# Usage Guide

## Running Containers

### Basic
```bash
docker run -it --rm ghcr.io/combatrobotics/ros-humble-gz-base:latest
```

### With GUI
```bash
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ghcr.io/combatrobotics/ros-humble-gz-dev:latest
```

### With GPU
```bash
docker run -it --rm \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ghcr.io/combatrobotics/ros-humble-gz-dev:latest
```

## VS Code DevContainers

1. Copy DevContainer files to your project:
```bash
cp -r humble-harmonic-dev/.devcontainer /path/to/your/project/
```

2. Open in VS Code and press F1 → "Reopen in Container"

## Inside the Container

```bash
# Source ROS
source /opt/ros/humble/setup.bash

# Source gz_ros2_control
source /opt/gz_ros2_control_ws/install/setup.bash

# Run Gazebo simulation
gz sim -s -r --headless-rendering empty.sdf

# Launch ROS-Gazebo bridge
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```