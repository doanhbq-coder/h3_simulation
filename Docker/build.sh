#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="${SCRIPT_DIR}/.."

docker build --build-arg ROS_DISTRO=humble -t h3_simulation:latest -f "${SCRIPT_DIR}/Dockerfile" "${ROOT_DIR}"
