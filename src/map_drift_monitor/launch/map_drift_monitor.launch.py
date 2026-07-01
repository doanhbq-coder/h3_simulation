import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('map_drift_monitor')
    default_config = os.path.join(pkg, 'config', 'map_drift_monitor.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to map_drift_monitor config yaml',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time',
    )

    monitor_node = Node(
        package='map_drift_monitor',
        executable='map_drift_monitor_node',
        name='map_drift_monitor_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        config_arg,
        use_sim_time_arg,
        monitor_node,
    ])
