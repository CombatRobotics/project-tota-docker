#!/bin/bash
set -e

# Paths
SCRIPT_DIR="$(dirname "$(realpath "${BASH_SOURCE[0]}")")"
PROJECT_TOTA_PATH="${PROJECT_TOTA_PATH:-$(realpath "$SCRIPT_DIR/../..")}"
TOTA_UI_PATH="$PROJECT_TOTA_PATH/ws/controller_ws/src/tota_ui"

# ROS env
source /opt/ros/humble/setup.bash
source "$PROJECT_TOTA_PATH/ws/controller_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Ensure rosbridge is available
if ! ros2 pkg list | grep -q '^rosbridge_server$'; then
  echo "Error: rosbridge_server not installed. Install it with:"
  echo "  sudo apt install -y ros-humble-rosbridge-suite"
  exit 1
fi

# Start rosbridge
echo "Starting rosbridge..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /tmp/rosbridge.log 2>&1 &
rosbridge_pid=$!

# Start backend
echo "Starting backend (gstreamer)..."
cd "$TOTA_UI_PATH/backend"
node gstreamer-server.js > /tmp/gstreamer.log 2>&1 &
backend_pid=$!

# Ensure cleanup
trap "kill $rosbridge_pid $backend_pid" EXIT

# Start UI
echo "Starting Next.js app..."
cd "$TOTA_UI_PATH"

# Auto-open browser once Next.js is ready
(
  # wait until the app is reachable
  for i in {1..30}; do
    curl -fsS http://localhost:3000 >/dev/null 2>&1 && break
    sleep 1
  done

  # provide GUI env for user services and launch via wrapper
  export DISPLAY="${DISPLAY:-:0}"
  export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
  export DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-unix:path=${XDG_RUNTIME_DIR}/bus}"

  if [ -x "$HOME/.local/bin/chrome-no-keyring" ]; then
    "$HOME/.local/bin/chrome-no-keyring" "http://localhost:3000" >/dev/null 2>&1 || true
  fi
) &

# production start (assumes npm run build already done in setup)
exec npm run start