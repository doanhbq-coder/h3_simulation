#!/bin/bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -d "/ws/src" ] && [ ! -f "/ws/install/setup.bash" ]; then
  echo "[entrypoint] Building workspace (this may take a while on ARM64)..."
  cd /ws
  colcon build \
    --symlink-install \
    --parallel-workers 1 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-ignore robot_simulation h3_simulation
fi

if [ -f "/ws/install/setup.bash" ]; then
  source "/ws/install/setup.bash"
fi

exec "$@"
