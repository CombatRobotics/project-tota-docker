# Available Images

## Base Image: `ros-humble-gz-base`
- **Registry**: `ghcr.io/combatrobotics/ros-humble-gz-base`
- **Size**: ~3.07GB
- **Purpose**: Minimal production image

### Contents
- Ubuntu 22.04 (Jammy)
- ROS 2 Humble (ros-base)
- Gazebo Harmonic
- gz_ros2_control
- OpenGL libraries for headless operation

### Use Cases
- CI/CD pipelines
- Production deployments
- Lightweight development
- Simulation servers

## Dev Image: `ros-humble-gz-dev`
- **Registry**: `ghcr.io/combatrobotics/ros-humble-gz-dev`
- **Size**: ~3.69GB
- **Purpose**: Full development environment

### Additional Contents
- Navigation 2
- MoveIt 2
- SLAM Toolbox
- Robot Localization
- Teleop tools
- RQt and PlotJuggler
- Development tools (vim, nano)

### Use Cases
- Full development environment
- Testing and debugging
- Robot control development
- VS Code DevContainers

## Extending Images

### Custom Dockerfile
```dockerfile
FROM ghcr.io/combatrobotics/ros-humble-gz-base:latest
RUN apt-get update && apt-get install -y \
    your-packages-here
```

### Modifying Dev Image
Edit `humble-harmonic-dev/packages.txt` to add/remove packages