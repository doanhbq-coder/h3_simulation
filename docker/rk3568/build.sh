#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="${SCRIPT_DIR}/../.."

echo "=== Building H3 Robot image for RK3568 (ARM64) ==="
echo "Root: ${ROOT_DIR}"

docker build \
  --build-arg ROS_DISTRO=humble \
  -t h3_robot:rk3568 \
  -f "${SCRIPT_DIR}/Dockerfile" \
  "${ROOT_DIR}"

echo "=== Build complete: h3_robot:rk3568 ==="
