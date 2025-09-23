#!/bin/bash

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${repo_root}/rocm_venv"

echo "This script will create a Python virtual environment for ROCm with access to system site packages."
read -p "Enter the directory for the venv [${VENV_DIR}]: " USER_VENV_DIR
if [[ -n "$USER_VENV_DIR" ]]; then
    VENV_DIR="$USER_VENV_DIR"
fi

# Ensure venv package is available
sudo apt-get update
sudo apt-get install -y python3-venv

echo "Creating venv at: $VENV_DIR"
mkdir -p "$(dirname "$VENV_DIR")"
python3 -m venv --system-site-packages "$VENV_DIR"

source "$VENV_DIR/bin/activate"

pip install --upgrade pip

pip install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/rocm6.4

pip install onnxruntime-rocm -f https://repo.radeon.com/rocm/manylinux/rocm-rel-6.4.2/

pip install -r "${repo_root}/requirements.txt"
