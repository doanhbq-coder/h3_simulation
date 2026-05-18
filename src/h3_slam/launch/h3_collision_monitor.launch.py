import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Khai báo đường dẫn đến file cấu hình .yaml của bạn
    # Thay 'your_package_name' bằng tên package của bạn
    pkg_share = get_package_share_directory('h3_slam') 
    
    params_file = LaunchConfiguration('params_file')
    default_params_file = os.path.join(pkg_share, 'config', 'h3_collision_monitor_params_v2.yaml')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for the collision monitor node')

    # 2. Định nghĩa Node Collision Monitor
    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        emulate_tty=True,
        parameters=[params_file],
        # Ánh xạ lại các topic nếu cần thiết
        # remappings=[
        #     ('cmd_vel_in', 'cmd_vel_raw'),
        #     ('cmd_vel_out', 'cmd_vel')
        # ]
    )

    ld = LaunchDescription()

    # Thêm các hành động vào LaunchDescription
    ld.add_action(declare_params_file_cmd)
    ld.add_action(collision_monitor_node)

    return ld