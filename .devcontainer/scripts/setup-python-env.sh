#!/bin/bash
# Setup Python environment with ROCm/CUDA support
set -e

echo "Setting up Python environment with GPU support..."

# Upgrade pip but use compatible setuptools version for ROS2
pip3 install --upgrade pip wheel
pip3 install "setuptools<76"

# Handle sympy conflict by forcing reinstall if needed
echo "Handling potential package conflicts..."
pip3 install --force-reinstall sympy || true

# Detect GPU type and install appropriate PyTorch
if lspci | grep -i nvidia > /dev/null 2>&1; then
    echo "NVIDIA GPU detected, installing CUDA PyTorch..."
    pip3 install --no-deps torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121 || \
    pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
elif lspci | grep -i amd > /dev/null 2>&1; then
    echo "AMD GPU detected, installing ROCm PyTorch..."
    pip3 install --no-deps --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/rocm6.4 || \
    pip3 install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/rocm6.4

    # Install ROCm-specific packages
    pip3 install onnxruntime-rocm || true
else
    echo "No GPU detected, installing CPU-only PyTorch..."
    pip3 install --no-deps torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu || \
    pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
fi

# Install Python requirements if they exist
REQUIREMENTS_FILE="/ros2_ws/project_tota/.devcontainer/requirements.txt"
if [ -f "$REQUIREMENTS_FILE" ]; then
    echo "Installing Python requirements..."
    pip3 install --no-cache-dir --force-reinstall -r "$REQUIREMENTS_FILE" || {
        echo "Some requirements failed to install, trying without force-reinstall..."
        pip3 install --no-cache-dir -r "$REQUIREMENTS_FILE" || true
    }
fi

echo "Python environment setup complete"