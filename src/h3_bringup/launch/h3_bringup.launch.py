from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # a1 controller
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot_simulation'),
            'launch',
            'simulation.launch.py'
        ))
    )

    # a1 laser filter
    laser_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('h3_bringup'),
            'launch',
            'lidar_filter.launch.py'
        ))
    )

    # a1 joystick
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('h3_bringup'),
            'launch',
            'joystick.launch.py'
        ))
    )

    map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('h3_slam'),
            'launch',
            'h3_map_server.launch.py'
        ))
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('h3_slam'),
            'launch',
            'h3_navigation.launch.py'
        ))
    )

    usb_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('cob_fiducials'),
            'launch',
            'usb_cam.launch.py'
        ))
    )


    return LaunchDescription([
        simulation,
        # laser_filter,
        # joystick,
        map_server,
        navigation,
        # usb_cam
    ])

# save map
#ros2 run nav2_map_server map_saver_cli -f map