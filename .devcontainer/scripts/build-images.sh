#!/usr/bin/env bash
set -euo pipefail

python3 <<'PY' > /tmp/devcontainer_env
import json
from pathlib import Path
keys = [
    'UBUNTU_IMAGE',
    'UBUNTU_TAG',
    'BASE_IMAGE',
    'DEV_IMAGE',
    'TARGETARCH',
    'GZ_ROS2_CONTROL_WS',
    'ENABLE_CV',
    'USERNAME',
    'USER_UID',
    'USER_GID',
    'CV_USE_CUDA'
]
args = json.loads(Path('.devcontainer/devcontainer.json').read_text())['build']['args']
for key in keys:
    print(f"{key}={args.get(key, '')}")
PY

readarray -t cfg < /tmp/devcontainer_env
rm -f /tmp/devcontainer_env
for line in "${cfg[@]}"; do
  eval "export ${line}"
done

if [[ ${ENABLE_CV,,} == "true" ]]; then
  CV_TARGET=ubuntu_cv
else
  CV_TARGET=ubuntu_base
fi

docker build -f .devcontainer/Dockerfile.ubuntu \
  --build-arg UBUNTU_IMAGE="$UBUNTU_IMAGE" \
  --build-arg CV_USE_CUDA="$CV_USE_CUDA" \
  --target "$CV_TARGET" \
  -t "$UBUNTU_TAG" .

docker build -f .devcontainer/Dockerfile.base \
  --target base \
  --build-arg UBUNTU_BASE_IMAGE="$UBUNTU_TAG" \
  -t "$BASE_IMAGE" .

docker build -f .devcontainer/Dockerfile.base \
  --target dev \
  --build-arg UBUNTU_BASE_IMAGE="$UBUNTU_TAG" \
  --build-arg TARGETARCH="$TARGETARCH" \
  --build-arg GZ_ROS2_CONTROL_WS="$GZ_ROS2_CONTROL_WS" \
  --build-arg USERNAME="$USERNAME" \
  --build-arg USER_UID="$USER_UID" \
  --build-arg USER_GID="$USER_GID" \
  -t "$DEV_IMAGE" .
