# TOTA DevContainer for ROS 2 Development

This DevContainer provides a complete development environment for TOTA, including ROS2 Humble, Gazebo Harmonic, and all necessary dependencies.

## Features

- **Optimized Caching**: Multi-layer Dockerfile design for fast rebuilds
- **Modular Architecture**: Separate scripts for each component, easy to modify
- **Base Image**: Built on `ghcr.io/combatrobotics/ros-humble-gz-base:latest` with ROS2 Humble and Gazebo Harmonic
- **CycloneDDS**: Pre-configured as the default RMW implementation
- **Python Environment**: Dedicated virtual environment for ML/AI packages
- **TOTA Repositories**: Automatic setup of controller workspace with all TOTA packages
- **GPU Support**: NVIDIA and AMD ROCm support (configurable)
- **Development Tools**: Full suite of debugging, profiling, and development utilities

## Prerequisites

- Docker with GPU support (if using GPU features)
- Visual Studio Code with Remote-Containers extension
- Git with SSH keys configured (for private repositories)

## Quick Start

1. Open the project in VS Code
2. When prompted, click "Reopen in Container" or use Command Palette: "Dev Containers: Reopen in Container"
3. Wait for the container to build (first time may take 10-15 minutes)
4. The post-create script will automatically:
   - Import TOTA repositories
   - Build controller workspace
   - Setup Python virtual environment
   - Configure udev rules

## Configuration

### Build Arguments

Edit `.devcontainer/devcontainer.json` to customize:

- `INSTALL_CLAUDE`: Enable/disable Claude AI assistant (default: true)
- `INSTALL_ROCM`: Enable AMD ROCm support (default: false)
- `INSTALL_WFB`: Enable WiFi broadcast support (default: false)
- `ROS_DOMAIN_ID`: Set ROS2 domain ID (default: 0)
- `ROS_LOCALHOST_ONLY`: Restrict ROS2 to localhost (default: 0)

### Customizing Packages

Edit `.devcontainer/packages.txt` to add/remove Ubuntu packages. Packages are organized by category:
- TOTA-specific ROS2 packages
- Development tools
- Optional hardware support

## Available Commands

After container startup, these aliases are available:

- `tota_build` - Build main TOTA workspace
- `controller_build` - Build controller workspace
- `activate_ml` - Activate ML virtual environment
- `ros2_deps` - Install ROS2 dependencies
- `ros2_build` - Build current ROS2 workspace
- `ros2_clean` - Clean build/install/log directories

## Manual Setup Tasks

Run additional setup using the setup script:

```bash
# Run all setup tasks
/ros2_ws/project_tota/.devcontainer/setup-container.sh --all

# Or run specific tasks
/ros2_ws/project_tota/.devcontainer/setup-container.sh --cyclonedds  # Configure CycloneDDS
/ros2_ws/project_tota/.devcontainer/setup-container.sh --venv        # Setup Python venv
/ros2_ws/project_tota/.devcontainer/setup-container.sh --build-controller  # Build controller workspace
```

## Workspaces

The container manages two ROS2 workspaces:

1. **Main Workspace** (`/ros2_ws/project_tota`)
   - Your main development workspace
   - Source with: `source /ros2_ws/project_tota/install/setup.bash`

2. **Controller Workspace** (`/ros2_ws/project_tota/ws/controller_ws`)
   - TOTA control packages from ally.repos
   - Source with: `source /ros2_ws/project_tota/ws/controller_ws/install/setup.bash`

## GPU Support

### NVIDIA GPUs
- Automatically detected and configured
- Requires NVIDIA Container Toolkit on host

### AMD GPUs (ROCm)
- Set `INSTALL_ROCM=true` in devcontainer.json
- Requires ROCm drivers on host
- HSA_OVERRIDE_GFX_VERSION is pre-configured

## Troubleshooting

### Private Repository Access
If you get errors accessing private repos:
1. Ensure SSH agent forwarding is enabled
2. Add SSH keys to your GitHub account
3. Test with: `ssh -T git@github.com`

### Display Issues
For GUI applications:
1. On Linux: Allow X11 forwarding with `xhost +local:docker`
2. On Windows: Use WSL2 with WSLg
3. On macOS: Install XQuartz and enable connections from network clients

### USB Device Access
The container runs with `--privileged` flag for full USB access. Devices should appear automatically in `/dev/`

## File Structure

```
.devcontainer/
├── devcontainer.json         # Main configuration
├── Dockerfile               # Optimized multi-layer build
├── packages.txt            # ROS2 packages to install
├── scripts/                # Modular installation scripts
│   ├── install-system-deps.sh      # Core system dependencies
│   ├── install-gstreamer.sh        # GStreamer for video
│   ├── install-ros-packages.sh     # ROS2 packages
│   ├── install-optional.sh         # Claude, ROCm, WFB
│   ├── install-python-deps.sh      # Python packages
│   ├── setup-environment.sh        # Environment setup
│   ├── post-create.sh              # Run once after creation
│   ├── post-start.sh               # Run on each start
│   └── setup-container.sh         # Manual setup utilities
├── config/                 # Configuration files
│   ├── bashrc-aliases.sh          # Custom aliases
│   └── ros2-config.yaml           # ROS2 configuration
└── README.md              # This file
```

## Docker Build Layers

The Dockerfile is organized into cached layers for efficient rebuilds:

1. **System Dependencies** - Core tools, rarely changes
2. **GStreamer** - Video streaming support, rarely changes
3. **ROS2 Packages** - From packages.txt, occasional updates
4. **Optional Components** - Claude/ROCm/WFB, conditional
5. **Python Dependencies** - From requirements.txt
6. **Workspace Structure** - Directory creation
7. **Configuration Files** - Scripts and configs, frequent changes
8. **Environment Setup** - Bashrc configuration

When you modify files, only affected layers and those after them rebuild.