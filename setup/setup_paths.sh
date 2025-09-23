#!/bin/bash

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# TODO: Do not show if the paths are already set


setup_paths() {
    echo "Setting up paths for export and source in .bashrc for controller"
    
    if [ -d "${repo_root}/ws/controller_ws" ]; then
        if ! grep -q "^source ${repo_root}/ws/controller_ws/install/setup.bash" ~/.bashrc; then
            echo -e "\n\n# Source TOTA controller workspace" >> ~/.bashrc
            echo -e "source ${repo_root}/ws/controller_ws/install/setup.bash" >> ~/.bashrc
        else
            echo -e "\nPath already set in .bashrc, not adding duplicate."
        fi
    else
        echo -e "\n \e[33m Warning: Workspace ${repo_root}/ws/controller_ws does not exist. Skipping path setup.\e[0m \n"
    fi
    if ! grep -q "^export PROJECT_TOTA_PATH=${repo_root}" ~/.bashrc; then
        echo -e "\n\n# Export TOTA project path" >> ~/.bashrc
        echo -e "export PROJECT_TOTA_PATH=${repo_root}" >> ~/.bashrc
    else
        echo -e "\nPath already set in .bashrc, not adding duplicate."
    fi
    echo -e "\nPaths added to .bashrc\n"
    export PROJECT_TOTA_PATH=${repo_root}
}

setup_paths