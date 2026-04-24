from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('lidar_filter'),
        'config',
        'scan_filter.yaml'
    )

    return LaunchDescription([
        Node(
            package='lidar_filter',
            executable='lidar_merger',
            name='lidar_merger',
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=[
                '0', '0', '0.2',   # x y z (vị trí lidar so với base_link)
                '3.14159', '0', '0',     # roll pitch yaw
                'base_link',
                'laser_frame'
            ],
            output='screen'
        ),
        # Note: TF transforms are now published by robot_state_publisher from URDF
        # laser_frame_front and laser_frame_rear are defined in the URDF
        # This merger node creates a merged scan in laser_frame_front
    ])
