#!/bin/bash
# Comprehensive Python environment setup for TOTA project
# This script ensures all dependencies are installed with correct versions

set -e

echo "═══════════════════════════════════════════════════════════════"
echo "Setting up Python environment for TOTA project..."
echo "═══════════════════════════════════════════════════════════════"

# Step 1: System dependencies
echo "Step 1: Installing system dependencies..."
apt-get update -qq
apt-get install -y --no-install-recommends \
    python3-dev \
    python3-numpy \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgomp1 \
    libcairo2-dev \
    pkg-config \
    pciutils \
    2>/dev/null || true

# Step 2: Upgrade pip and install build tools
echo "Step 2: Upgrading pip and installing build tools..."
pip3 install --upgrade pip
pip3 install --no-cache-dir \
    "wheel" \
    "setuptools<76" \
    "packaging==23.2" \
    "cython>=3.0.0"

# Step 3: Core scientific packages with exact versions
echo "Step 3: Installing core scientific packages..."
pip3 install --no-cache-dir \
    "numpy==1.26.4" \
    "scipy==1.13.1" \
    "matplotlib==3.7.5" \
    "pillow==10.4.0"

# Step 4: OpenCV
echo "Step 4: Installing OpenCV..."
pip3 install --no-cache-dir \
    "opencv-python==4.10.0.84" \
    "opencv-python-headless==4.10.0.84"

# Remove distutils-installed sympy manually before PyTorch installation
echo "Removing old sympy installation..."
rm -rf /usr/lib/python3*/dist-packages/sympy* 2>/dev/null || true
rm -rf /usr/local/lib/python3*/dist-packages/sympy* 2>/dev/null || true
rm -rf /usr/lib/python3/dist-packages/sympy.egg-info 2>/dev/null || true
pip3 uninstall -y sympy 2>/dev/null || true

# Step 5: PyTorch (CPU version for compatibility)
echo "Step 5: Installing PyTorch..."
pip3 install --no-cache-dir \
    torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

# Step 6: ONNX stack
echo "Step 6: Installing ONNX stack..."
pip3 install --no-cache-dir \
    "onnx==1.16.2" \
    "onnxruntime==1.18.1" \
    "ml_dtypes==0.4.1"

# Step 7: Machine Learning packages
echo "Step 7: Installing ML packages..."
pip3 install --no-cache-dir \
    "scikit-learn==1.5.1" \
    "scikit-image==0.23.2" \
    "pandas==2.2.2" \
    "seaborn>=0.11.0"

# Step 8: Computer vision dependencies with CORRECT versions
echo "Step 8: Installing computer vision packages..."
# First install dependencies
pip3 install --no-cache-dir \
    "annotated-types==0.7.0" \
    "pydantic-core==2.20.1" \
    "pydantic==2.8.2" \
    "eval-type-backport"

# Install albucore 0.0.16 (compatible version)
pip3 install --no-cache-dir \
    "albucore==0.0.16"

# Install albumentations 1.4.13 (compatible with albucore 0.0.16)
pip3 install --no-cache-dir \
    "albumentations==1.4.13"

# Step 9: InsightFace (install without dependencies first)
echo "Step 9: Installing InsightFace..."
pip3 install --no-cache-dir \
    "easydict" \
    "prettytable" \
    "requests" \
    "charset_normalizer" \
    "certifi"

# Install insightface without dependencies to avoid conflicts
pip3 install --no-cache-dir --no-deps \
    "insightface==0.7.3"

# Step 10: Ultralytics and dependencies
echo "Step 10: Installing Ultralytics..."
pip3 install --no-cache-dir \
    "ultralytics-thop>=2.0.0" \
    "py-cpuinfo" \
    "psutil"

pip3 install --no-cache-dir --no-deps \
    "ultralytics==8.2.103"

# Step 11: ROS2 and robotics packages
echo "Step 11: Installing ROS2 and robotics packages..."
pip3 install --no-cache-dir \
    "pyserial>=3.5" \
    "pymavlink>=2.4.40" \
    "transforms3d>=0.4.1" \
    "catkin-pkg>=0.5.0" \
    "empy>=3.3.4" \
    "lark>=1.1.5" \
    "colcon-common-extensions"

# Step 12: Additional utilities
echo "Step 12: Installing additional utilities..."
pip3 install --no-cache-dir \
    "tqdm>=4.64.0" \
    "PyYAML>=5.3.1" \
    "joblib>=1.2.0" \
    "threadpoolctl>=3.1.0" \
    "imageio>=2.33" \
    "tifffile>=2022.8.12" \
    "networkx>=2.8" \
    "lazy-loader>=0.4"

# Step 13: Fix any remaining dependencies
echo "Step 13: Fixing any remaining dependencies..."
pip3 install --no-cache-dir \
    "filelock>=3.13.1" \
    "fsspec>=2024.6.1" \
    "flatbuffers" \
    "coloredlogs" \
    "humanfriendly" \
    "wcwidth" \
    "mpmath>=1.1.0" \
    "typing-extensions>=4.9.0"

# Step 14: Verification
echo "Step 14: Verifying installation..."
python3 << 'VERIFY'
import warnings
warnings.filterwarnings('ignore')

packages_to_test = [
    ('numpy', 'import numpy'),
    ('scipy', 'import scipy'),
    ('matplotlib', 'import matplotlib'),
    ('cv2', 'import cv2'),
    ('torch', 'import torch'),
    ('onnx', 'import onnx'),
    ('onnxruntime', 'import onnxruntime'),
    ('sklearn', 'import sklearn'),
    ('skimage', 'import skimage'),
    ('PIL', 'from PIL import Image'),
    ('insightface', 'import insightface'),
    ('ultralytics', 'import ultralytics'),
    ('albumentations', 'import albumentations'),
    ('pandas', 'import pandas'),
    ('serial', 'import serial'),
    ('pymavlink', 'import pymavlink'),
    ('transforms3d', 'import transforms3d'),
    ('yaml', 'import yaml'),
    ('catkin_pkg', 'import catkin_pkg')
]

failed = []
for name, import_cmd in packages_to_test:
    try:
        exec(import_cmd)
        print(f"✅ {name}")
    except Exception as e:
        print(f"❌ {name}: {e}")
        failed.append(name)

if failed:
    print(f"\n⚠️ WARNING: {len(failed)} packages failed to import: {', '.join(failed)}")
    print("The script will continue, but these packages may need manual intervention.")
else:
    print("\n✅ All critical packages verified successfully!")
VERIFY

echo "═══════════════════════════════════════════════════════════════"
echo "Python environment setup complete!"
echo "═══════════════════════════════════════════════════════════════"