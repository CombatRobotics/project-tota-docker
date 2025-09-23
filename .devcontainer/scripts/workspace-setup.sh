#!/bin/bash
# Workspace setup and build script

set -e

WORKSPACE_PATH="${WORKSPACE_PATH:-/ros2_ws/project_tota}"
CONTROLLER_WS="${WORKSPACE_PATH}/ws/controller_ws"

# Initialize ROS dependencies
init_rosdep() {
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        echo "Initializing rosdep..."
        sudo rosdep init
    fi
    rosdep update
}

# Import repositories
import_repos() {
    local repos_file="${WORKSPACE_PATH}/repos/ally.repos"

    if [ ! -f "$repos_file" ]; then
        return 0
    fi

    echo "Importing repositories..."
    mkdir -p "${CONTROLLER_WS}/src"
    cd "${CONTROLLER_WS}/src"
    vcs import < "$repos_file" 2>/dev/null || true
}

# Build workspace
build_workspace() {
    local ws_path="$1"
    local ws_name="$2"

    if [ ! -d "${ws_path}/src" ]; then
        return 0
    fi

    echo "Building ${ws_name} workspace..."
    cd "${ws_path}"

    rosdep install --from-paths src --ignore-src -r -y --rosdistro humble 2>/dev/null || true
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release || true
}

# Setup Python environment
setup_python_env() {
    local venv_path="${WORKSPACE_PATH}/rocm_venv"

    if [ ! -d "$venv_path" ]; then
        echo "Creating Python virtual environment..."
        python3 -m venv --system-site-packages "$venv_path"
    fi

    if [ -f "${WORKSPACE_PATH}/requirements.txt" ]; then
        echo "Installing Python packages..."
        source "$venv_path/bin/activate"
        pip install --quiet --upgrade pip setuptools wheel
        pip install --quiet -r "${WORKSPACE_PATH}/requirements.txt" 2>/dev/null || true
        deactivate
    fi
}

# Main
main() {
    source /opt/ros/humble/setup.bash
    init_rosdep
    import_repos
    build_workspace "${CONTROLLER_WS}" "controller"
    [ -f "${CONTROLLER_WS}/install/setup.bash" ] && source "${CONTROLLER_WS}/install/setup.bash"
    build_workspace "${WORKSPACE_PATH}" "main"
    setup_python_env
}

main "$@"