#!/bin/bash
set -e

# Paths
SCRIPT_DIR="$(dirname "$(realpath "${BASH_SOURCE[0]}")")"
PROJECT_TOTA_PATH="${PROJECT_TOTA_PATH:-$(realpath "$SCRIPT_DIR/..")}"
TOTA_UI_PATH="$PROJECT_TOTA_PATH/ws/controller_ws/src/tota_ui"

# ROS env
source /opt/ros/humble/setup.bash
source "$PROJECT_TOTA_PATH/ws/controller_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Ensure modern Node.js (>=18). Install Node.js 20.x if needed.
need_node_upgrade=1
if command -v node >/dev/null 2>&1; then
  node_major="$(node -v | sed 's/^v//' | cut -d. -f1)"
  if [ "$node_major" -ge 18 ]; then need_node_upgrade=0; fi
fi

if [ "$need_node_upgrade" -ne 0 ]; then
  echo "Installing/upgrading Node.js to 20.x..."

  # Remove conflicting Ubuntu Node 12 dev headers if present
  if dpkg -s libnode-dev >/dev/null 2>&1; then
    sudo apt-get remove -y libnode-dev
  fi

  # Add NodeSource repo & install Node 20
  curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
  sudo apt-get install -y nodejs build-essential
fi

# Install UI dependencies
if [ -f "$TOTA_UI_PATH/package.json" ]; then
  echo "Installing UI dependencies..."
  (cd "$TOTA_UI_PATH" && rm -rf node_modules package-lock.json && npm install)
fi

# Install backend dependencies
if [ -f "$TOTA_UI_PATH/backend/package.json" ]; then
  echo "Installing backend dependencies..."
  (cd "$TOTA_UI_PATH/backend" && rm -rf node_modules package-lock.json && npm install)
fi

# Build production Next.js app
if [ -f "$TOTA_UI_PATH/package.json" ]; then
  echo "Building Next.js production bundle..."
  (cd "$TOTA_UI_PATH" && npm run build)
fi

# Optional browser on desktop
if [ -n "$DISPLAY" ]; then
  if ! command -v google-chrome >/dev/null 2>&1 && ! command -v chromium-browser >/dev/null 2>&1; then
    wget -q -O - https://dl.google.com/linux/linux_signing_key.pub | sudo apt-key add -
    echo "deb [arch=amd64] http://dl.google.com/linux/chrome/deb/ stable main" | \
      sudo tee /etc/apt/sources.list.d/google-chrome.list >/dev/null
    sudo apt-get update
    sudo apt-get install -y google-chrome-stable
  fi
fi

echo "UI setup complete."