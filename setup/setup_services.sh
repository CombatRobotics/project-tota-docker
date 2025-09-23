#!/bin/bash
set -e

# Absolute repo root (one directory up from this script’s dir)
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo -e "\nSetting up services for Controller"
read -p "Do you want to continue? (y/n) " -n 1 -r; echo
[[ $REPLY =~ ^[Yy]$ ]] || { echo "Skipping service setup"; exit 0; }

if [ -d "${repo_root}/ws/controller_ws" ]; then
    echo "Setting up startup service for controller..."
    
    mkdir -p ~/.tota_startup
    ln -sf "${repo_root}/bootup/startup_scripts/start_controller.sh" ~/.tota_startup/start_controller.sh
    ln -sf "${repo_root}/bootup/startup_scripts/start_ui.sh" ~/.tota_startup/start_ui.sh

    systemctl --user daemon-reload
    systemctl --user enable "${repo_root}/bootup/startup_services/tota_controller.service"
    systemctl --user enable "${repo_root}/bootup/startup_services/tota_ui.service"
    systemctl --user start "tota_controller.service"
    
    echo "Service setup completed successfully"
else
    echo -e "\n\e[33mWarning: Workspace ${repo_root}/ws/controller_ws does not exist. Skipping service setup.\e[0m\n"
fi
