# syntax=docker/dockerfile:1
ARG ROS_DISTRO=humble
ARG ROS_BASE_IMAGE=osrf/ros
ARG ROS_IMAGE_TAG=${ROS_DISTRO}-desktop
FROM ${ROS_BASE_IMAGE}:${ROS_IMAGE_TAG}

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    HOME=/root \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-laser-geometry \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY ./src ./src
COPY ./Docker/entrypoint.sh /ros_entrypoint.sh

RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install --parallel-workers 2"

RUN chmod +x /ros_entrypoint.sh

ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
