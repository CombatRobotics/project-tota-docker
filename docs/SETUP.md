# Setup Guide

## Quick Start

```bash
# Clone repository
git clone https://github.com/CombatRobotics/ros2-docker-dev.git
cd ros2-docker-dev

# Run setup script
chmod +x host_setup.sh
./host_setup.sh
```

The setup script will:
- Install Docker and Docker Buildx
- Add your user to docker group
- Setup X11 display permissions
- Install NVIDIA Container Toolkit (if GPU present)
- Copy DevContainer files to `.devcontainer/`
- Guide you through GitHub registry access

## Manual Setup (if needed)

### GitHub Registry Access
1. Create a Personal Access Token at https://github.com/settings/tokens/new
   - Select scope: `read:packages`
2. Login to Docker:
```bash
export GITHUB_TOKEN='your-token-here'
echo $GITHUB_TOKEN | docker login ghcr.io -u YOUR_USERNAME --password-stdin
```

### Pull Images
```bash
docker pull ghcr.io/combatrobotics/ros-humble-gz-base:latest
docker pull ghcr.io/combatrobotics/ros-humble-gz-dev:latest
```