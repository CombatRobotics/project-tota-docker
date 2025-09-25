# syntax=docker/dockerfile:1.4

ARG BASE_IMAGE=ghcr.io/combatrobotics/ros-humble-gz-base:latest
FROM ${BASE_IMAGE}

# Build arguments
ARG WORKSPACE_PATH=/ros2_ws/project_tota
ARG ROS_DOMAIN_ID=0
ARG ROS_LOCALHOST_ONLY=0
ARG INSTALL_CLAUDE=true

# Environment
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
    ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY} \
    HSA_OVERRIDE_GFX_VERSION=11.0.2 \
    INSTALL_CLAUDE=${INSTALL_CLAUDE} \
    PROJECT_TOTA_PATH=${WORKSPACE_PATH}

# Create workspace structure early
RUN mkdir -p ${WORKSPACE_PATH}/.devcontainer/scripts

# Copy all build-time files to workspace
COPY packages.txt requirements.txt ${WORKSPACE_PATH}/.devcontainer/
COPY scripts/install-deps.sh scripts/setup-python-env.sh scripts/setup-wfb.sh ${WORKSPACE_PATH}/.devcontainer/scripts/

# Run installation scripts
RUN chmod +x ${WORKSPACE_PATH}/.devcontainer/scripts/*.sh && \
    ${WORKSPACE_PATH}/.devcontainer/scripts/install-deps.sh && \
    ${WORKSPACE_PATH}/.devcontainer/scripts/setup-python-env.sh && \
    ${WORKSPACE_PATH}/.devcontainer/scripts/setup-wfb.sh

# Create additional workspace directories
RUN mkdir -p ${WORKSPACE_PATH}/{src,logs,data}

# Copy runtime scripts
COPY scripts/post-create.sh scripts/post-start.sh ${WORKSPACE_PATH}/.devcontainer/scripts/
RUN chmod +x ${WORKSPACE_PATH}/.devcontainer/scripts/*.sh

# Set working directory
WORKDIR ${WORKSPACE_PATH}

# Health check
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
    CMD [ -f /opt/ros/humble/setup.bash ] || exit 1

# Default entrypoint
ENTRYPOINT ["/bin/bash"]
CMD ["-l"]