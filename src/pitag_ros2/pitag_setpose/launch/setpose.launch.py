"""
Launch file for pitag_setpose node.

Typical usage:
    # Simulation only (URDF đã có TF camera):
    ros2 launch pitag_setpose setpose.launch.py

    # Robot thật (cần publish TF camera, cao 1.7m):
    ros2 launch pitag_setpose setpose.launch.py \
        publish_camera_tf:=true \
        api_host:=http://192.168.68.142

    # Robot thật + cob_fiducials cùng lúc:
    ros2 launch pitag_setpose setpose.launch.py \
        publish_camera_tf:=true \
        api_host:=http://192.168.1.100 \
        with_fiducials:=true \
        image_topic:=/camera/imx335/image_raw \
        camera_info_topic:=/camera/imx335/camera_info
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('pitag_setpose')
    default_config = os.path.join(pkg_share, 'config', 'pitags.yaml')

    # ── Arguments: setpose ────────────────────────────────────────────────────
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Absolute path to pitags.yaml with tag positions')

    detection_topic_arg = DeclareLaunchArgument(
        'detection_topic',
        default_value='/fiducials/detect_fiducials',
        description='DetectionArray topic published by cob_fiducials')

    initialpose_topic_arg = DeclareLaunchArgument(
        'initialpose_topic',
        default_value='/initialpose',
        description='Topic to publish PoseWithCovarianceStamped for AMCL')

    api_host_arg = DeclareLaunchArgument(
        'api_host',
        default_value='',
        description='Robot API base URL, e.g. http://192.168.1.100 (empty = disabled)')

    api_timeout_arg = DeclareLaunchArgument(
        'api_timeout',
        default_value='3.0',
        description='HTTP request timeout in seconds')

    # ── Arguments: camera TF (robot thật, không có URDF) ─────────────────────
    publish_camera_tf_arg = DeclareLaunchArgument(
        'publish_camera_tf',
        default_value='false',
        description=(
            'Publish static TF base_link→camera_frame→camera_frame_optical. '
            'Dùng cho robot thật (không có URDF). '
            'Khi chạy simulation thì tắt (URDF đã publish TF rồi).'))

    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='1.7',
        description='Chiều cao camera so với base_link (meters). Robot cao 1.7m → camera ở đỉnh.')

    camera_x_arg = DeclareLaunchArgument(
        'camera_x',
        default_value='0.0',
        description='Offset x của camera so với tâm base_link (meters, dương = về phía trước)')

    camera_y_arg = DeclareLaunchArgument(
        'camera_y',
        default_value='0.0',
        description='Offset y của camera so với tâm base_link (meters, dương = sang trái)')

    camera_yaw_arg = DeclareLaunchArgument(
        'camera_yaw',
        default_value='0.0',
        description='Góc xoay camera quanh trục Z (yaw, radians). 0 = camera_frame Y song song map Y.')

    # ── Arguments: fiducials ──────────────────────────────────────────────────
    with_fiducials_arg = DeclareLaunchArgument(
        'with_fiducials',
        default_value='false',
        description='Set true to also launch cob_fiducials detector')

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/imx335/image_mono8',
        description='Camera image topic (used only when with_fiducials=true)')

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/imx335/camera_info',
        description='Camera info topic (used only when with_fiducials=true)')

    # ── Static TF: base_link → camera_frame ──────────────────────────────────
    # xyz = vị trí vật lý camera trên robot
    # rpy = (0, -pi/2, camera_yaw):
    #   pitch = -1.5707963 → camera X-axis hướng lên trần (+Z world)
    #   yaw   = camera_yaw → xoay camera trong mặt phẳng nằm ngang nếu cần
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        output='screen',
        arguments=[
            '--x',         LaunchConfiguration('camera_x'),
            '--y',         LaunchConfiguration('camera_y'),
            '--z',         LaunchConfiguration('camera_height'),
            '--roll',      '0',
            '--pitch',     '-1.5707963',
            '--yaw',       LaunchConfiguration('camera_yaw'),
            '--frame-id',  'base_link',
            '--child-frame-id', 'camera_frame',
        ],
        condition=IfCondition(LaunchConfiguration('publish_camera_tf')),
    )

    # ── Static TF: camera_frame → camera_frame_optical ───────────────────────
    # Đây là chuẩn ROS: chuyển từ body frame (X-forward, Y-left, Z-up)
    # sang optical frame (X-right, Y-down, Z-forward/vào cảnh).
    # rpy = (-pi/2, 0, -pi/2) — cố định, không đổi.
    camera_optical_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_tf_publisher',
        output='screen',
        arguments=[
            '--x',         '0',
            '--y',         '0',
            '--z',         '0',
            '--roll',      '-1.5707963',
            '--pitch',     '0',
            '--yaw',       '-1.5707963',
            '--frame-id',  'camera_frame',
            '--child-frame-id', 'camera_frame_optical',
        ],
        condition=IfCondition(LaunchConfiguration('publish_camera_tf')),
    )

    # ── Setpose node ──────────────────────────────────────────────────────────
    setpose_node = Node(
        package='pitag_setpose',
        executable='setpose_node',
        name='pitag_setpose',
        output='screen',
        parameters=[{
            'config_file':        LaunchConfiguration('config_file'),
            'detection_topic':    LaunchConfiguration('detection_topic'),
            'initialpose_topic':  LaunchConfiguration('initialpose_topic'),
            'api_host':           LaunchConfiguration('api_host'),
            'api_timeout':        LaunchConfiguration('api_timeout'),
        }],
    )

    # ── Optional: launch cob_fiducials together ───────────────────────────────
    fiducials_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('cob_fiducials'), 'ros', 'launch', 'fiducials.launch.py'
            ])
        ]),
        launch_arguments={
            'image_topic':       LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'publish_tf':        'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('with_fiducials')),
    )

    return LaunchDescription([
        # setpose args
        config_file_arg,
        detection_topic_arg,
        initialpose_topic_arg,
        api_host_arg,
        api_timeout_arg,
        # camera TF args
        publish_camera_tf_arg,
        camera_height_arg,
        camera_x_arg,
        camera_y_arg,
        camera_yaw_arg,
        # fiducials args
        with_fiducials_arg,
        image_topic_arg,
        camera_info_topic_arg,
        # nodes
        camera_tf_node,
        camera_optical_tf_node,
        setpose_node,
        fiducials_launch,
    ])
