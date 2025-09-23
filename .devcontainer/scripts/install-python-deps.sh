#!/bin/bash
# Install Python dependencies
# This layer rebuilds when requirements.txt changes

set -e

echo "=== Installing Python Dependencies ==="

# Upgrade pip first
pip3 install --no-cache-dir --upgrade pip setuptools wheel

# Install Python requirements if file exists
if [ -f /tmp/requirements.txt ]; then
    echo "Installing from requirements.txt..."
    pip3 install --no-cache-dir -r /tmp/requirements.txt || {
        echo "Note: Some packages may require specific hardware (ROCm/CUDA)"
        echo "These can be installed later in the container"
    }
else
    echo "No requirements.txt found, skipping Python package installation"
fi

# Install common Python tools
pip3 install --no-cache-dir \
    pytest \
    pytest-cov \
    black \
    flake8 \
    mypy \
    ipython || true

echo "=== Python Dependencies Installed ==="