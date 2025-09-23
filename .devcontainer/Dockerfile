# syntax=docker/dockerfile:1.4

ARG BASE_IMAGE=ghcr.io/combatrobotics/ros-humble-gz-base:latest
FROM ${BASE_IMAGE}

# Build arguments - these rarely change
ARG WORKSPACE_PATH=/ros2_ws/project_tota
ARG GZ_ROS2_CONTROL_WS=/opt/gz_ros2_control_ws
ARG INSTALL_CLAUDE=true
ARG ROS_DOMAIN_ID=0
ARG ROS_LOCALHOST_ONLY=0
ARG INSTALL_ROCM=false
ARG INSTALL_WFB=false

# ============================================
# LAYER 1: System Dependencies (rarely changes)
# ============================================
COPY scripts/install-system-deps.sh /tmp/
RUN --mount=type=cache,target=/var/cache/apt \
    chmod +x /tmp/install-system-deps.sh && \
    /tmp/install-system-deps.sh && \
    rm -f /tmp/install-system-deps.sh

# ============================================
# LAYER 2: GStreamer Dependencies (rarely changes)
# ============================================
COPY scripts/install-gstreamer.sh /tmp/
RUN --mount=type=cache,target=/var/cache/apt \
    chmod +x /tmp/install-gstreamer.sh && \
    /tmp/install-gstreamer.sh && \
    rm -f /tmp/install-gstreamer.sh

# ============================================
# LAYER 3: ROS2 Dependencies (occasionally changes)
# ============================================
COPY packages.txt /tmp/packages.txt
COPY scripts/install-ros-packages.sh /tmp/
RUN --mount=type=cache,target=/var/cache/apt \
    chmod +x /tmp/install-ros-packages.sh && \
    /tmp/install-ros-packages.sh && \
    rm -f /tmp/install-ros-packages.sh /tmp/packages.txt

# ============================================
# LAYER 4: Optional Components (conditional)
# ============================================
COPY scripts/install-optional.sh /tmp/
RUN --mount=type=cache,target=/var/cache/apt \
    chmod +x /tmp/install-optional.sh && \
    INSTALL_CLAUDE="${INSTALL_CLAUDE}" \
    INSTALL_ROCM="${INSTALL_ROCM}" \
    INSTALL_WFB="${INSTALL_WFB}" \
    /tmp/install-optional.sh && \
    rm -f /tmp/install-optional.sh

# ============================================
# LAYER 5: Python Dependencies (changes with requirements.txt)
# ============================================
COPY requirements.txt /tmp/requirements.txt
COPY scripts/install-python-deps.sh /tmp/
RUN chmod +x /tmp/install-python-deps.sh && \
    /tmp/install-python-deps.sh && \
    rm -f /tmp/install-python-deps.sh /tmp/requirements.txt

# ============================================
# LAYER 6: Workspace Structure (rarely changes)
# ============================================
RUN mkdir -p ${WORKSPACE_PATH} \
    && mkdir -p ${WORKSPACE_PATH}/ws/controller_ws/src \
    && mkdir -p ${WORKSPACE_PATH}/rocm_venv \
    && mkdir -p ${WORKSPACE_PATH}/.devcontainer/scripts \
    && mkdir -p ${WORKSPACE_PATH}/.devcontainer/config

# ============================================
# LAYER 7: Environment Setup (rarely changes)
# ============================================
COPY scripts/setup-environment.sh /tmp/
RUN chmod +x /tmp/setup-environment.sh && \
    INSTALL_ROCM="${INSTALL_ROCM}" \
    /tmp/setup-environment.sh && \
    rm -f /tmp/setup-environment.sh

# ============================================
# LAYER 8: Runtime Scripts (changes frequently)
# ============================================
COPY scripts/*.sh ${WORKSPACE_PATH}/.devcontainer/scripts/
RUN chmod +x ${WORKSPACE_PATH}/.devcontainer/scripts/*

WORKDIR ${WORKSPACE_PATH}

SHELL ["/bin/bash", "-c"]
CMD ["/bin/bash"]