# syntax=docker/dockerfile:1
#
# ROS2 Humble for Rockchip RK3568 (ARM64) running Armbian
# Base image: ros-base (no GUI/Gazebo) to keep it lightweight
#
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base AS base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    HOME=/root \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    PYTHONUNBUFFERED=1

# ── System dependencies ──────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    usbutils \
    i2c-tools \
    can-utils \
    iproute2 \
    iputils-ping \
    net-tools \
    wget \
    curl \
    vim-tiny \
    && rm -rf /var/lib/apt/lists/*

# ── ROS2 packages (robot, no simulation) ─────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-laser-geometry \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-rplidar-ros \
    ros-${ROS_DISTRO}-micro-ros-agent \
    ros-${ROS_DISTRO}-rosbridge-server \
    ros-${ROS_DISTRO}-diagnostic-updater \
    && rm -rf /var/lib/apt/lists/*

# ── Python dependencies (for h3_api_server, person_detector, etc.) ───
RUN pip3 install --no-cache-dir \
    fastapi \
    uvicorn[standard] \
    numpy \
    pyserial \
    websockets

# ── Workspace setup ──────────────────────────────────────────────────
WORKDIR /ws
COPY ./src ./src

# Build workspace (limit parallel workers for low-RAM boards)
RUN /bin/bash -lc "\
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build \
        --symlink-install \
        --parallel-workers 1 \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --packages-ignore robot_simulation h3_simulation \
    "

COPY ./docker/rk3568/entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
