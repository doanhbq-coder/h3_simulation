#!/bin/bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -d "/ws/src" ] && [ ! -f "/ws/install/setup.bash" ]; then
  echo "[entrypoint] Building workspace because /ws/install/setup.bash is missing..."
  cd /ws
  colcon build --symlink-install --parallel-workers 2
fi

if [ -f "/ws/install/setup.bash" ]; then
  source "/ws/install/setup.bash"
fi

exec "$@"
