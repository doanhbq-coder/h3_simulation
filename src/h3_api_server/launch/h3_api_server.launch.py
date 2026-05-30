import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time'
    )
    host_arg = DeclareLaunchArgument(
        'host', default_value='0.0.0.0', description='API server bind address'
    )
    port_arg = DeclareLaunchArgument(
        'port', default_value='8090', description='API server port'
    )
    maps_dir_arg = DeclareLaunchArgument(
        'maps_dir',
        default_value=os.path.join(
            get_package_share_directory('h3_slam'), 'maps'
        ),
        description='Directory containing map YAML files',
    )
    charge_x_arg = DeclareLaunchArgument(
        'charge_x', default_value='0.0', description='Charging station X'
    )
    charge_y_arg = DeclareLaunchArgument(
        'charge_y', default_value='0.0', description='Charging station Y'
    )
    charge_yaw_arg = DeclareLaunchArgument(
        'charge_yaw_deg', default_value='0.0', description='Charging station yaw (degrees)'
    )

    api_server_node = Node(
        package='h3_api_server',
        executable='h3_api_server',
        name='h3_api_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'maps_dir': LaunchConfiguration('maps_dir'),
            'charge_x': LaunchConfiguration('charge_x'),
            'charge_y': LaunchConfiguration('charge_y'),
            'charge_yaw_deg': LaunchConfiguration('charge_yaw_deg'),
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        host_arg,
        port_arg,
        maps_dir_arg,
        charge_x_arg,
        charge_y_arg,
        charge_yaw_arg,
        api_server_node,
    ])
