# Docker setup for h3_simulation

This folder contains a Docker configuration to build and run the `h3_simulation` ROS 2 workspace on other Linux machines.

## Files

- `Dockerfile`: builds a ROS 2 container image with required dependencies and compiles the workspace.
- `docker-compose.yml`: starts a container with X11 support for GUI tools like `rviz2`.
- `entrypoint.sh`: sources ROS 2 and workspace setup files before launching the container command.

## Build

From the repository root:

```bash
cd /path/to/h3_simulation
docker compose -f Docker/docker-compose.yml build
```

If your system still uses legacy `docker-compose`, the old command is:

```bash
docker-compose -f Docker/docker-compose.yml build
```

Or manually:

```bash
docker build --build-arg ROS_DISTRO=humble -t h3_simulation:latest -f Docker/Dockerfile .
```

## Run with local mounts

The Compose file mounts your local `src` directory into the container:

- `../src` → `/ws/src`

This ensures your local code changes are immediately visible inside the container.

It also uses host networking and X11 for GUI apps like `gazebo`.

Start the container:

```bash
xhost +local:root
docker-compose -f Docker/docker-compose.yml up -d --remove-orphans
```

If `install/setup.bash` is missing, the container will build the workspace automatically on first start.

If your workspace code changes later, rebuild inside the container:

```bash
docker exec -it h3_simulation bash
cd /ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 2
source /ws/install/setup.bash
```

## Run

```bash
docker-compose -f Docker/docker-compose.yml up -d
```

## Run Gazebo

From inside the container, source the workspace and launch Gazebo:

```bash
ros2 launch gazebo_ros gazebo.launch.py
```

Or run a custom launch file from your workspace:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch your_package your_launch.launch.py
```

Open a shell inside the running container:

```bash
docker exec -it h3_simulation bash
```

Inside the container, start a ROS 2 command and source the workspace if needed:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 run robot_simulation <node>
```

## Notes

- Use `xhost +local:root` on the host if you need to display GUI apps from the container.
- If your workspace changes, rebuild the image and recreate the container.
- The Docker image currently defaults to `ROS_DISTRO=humble`. Change the build arg if you need another ROS 2 distro.
