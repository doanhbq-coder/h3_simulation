from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('person_detector'),
        'config',
        'person_detector.yaml'
    )

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/neo_robotics/K1_demo/V0_0_0/scan_front',
        description='LaserScan topic to subscribe to'
    )

    detection_range_arg = DeclareLaunchArgument(
        'detection_range',
        default_value='2.0',
        description='Detection range in meters'
    )

    detector = Node(
        package='person_detector',
        executable='person_detector_node',
        name='person_detector_node',
        parameters=[
            config,
            {
                'scan_topic': LaunchConfiguration('scan_topic'),
                'detection_range': LaunchConfiguration('detection_range'),
            }
        ],
        output='screen',
        emulate_tty=True,
    )

    greeting = Node(
        package='person_detector',
        executable='greeting_node',
        name='greeting_node',
        parameters=[config],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        scan_topic_arg,
        detection_range_arg,
        detector,
        greeting,
    ])
