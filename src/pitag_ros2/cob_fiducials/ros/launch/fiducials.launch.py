import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Trailing slash is required: C++ node does model_directory + model_filename
    model_dir = os.path.join(
        get_package_share_directory('cob_fiducials'),
        'common', 'files', 'models', ''   # os.path.join adds trailing /
    )

    # ── Camera topics ────────────────────────────────────────────────────────
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/imx335/image_mono8',
        description='Full image topic to subscribe (e.g. /camera/imx335/image_mono8 or /camera/imx335/image_raw)'
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/imx335/camera_info',
        description='Full camera_info topic to subscribe'
    )

    # ── Detector settings ────────────────────────────────────────────────────
    fiducial_type_arg = DeclareLaunchArgument(
        'fiducial_type',
        default_value='TYPE_PI',
        description='Fiducial type: TYPE_PI or TYPE_ARUCO'
    )

    model_filename_arg = DeclareLaunchArgument(
        'model_filename',
        default_value='piTagIni_0.xml',
        description='XML model file (piTagIni_0.xml → ID 0-3 size 10cm, piTagIni_1.xml → ID 0-3 size 5cm, piTagIni_2.xml → ID 0-33)'
    )

    ros_node_mode_arg = DeclareLaunchArgument(
        'ros_node_mode',
        default_value='MODE_TOPIC_AND_SERVICE',
        description='MODE_TOPIC | MODE_SERVICE | MODE_TOPIC_AND_SERVICE'
    )

    ir_reflective_arg = DeclareLaunchArgument(
        'use_ir_reflective_markers',
        default_value='false',
        description='Invert image before detection — true for bright reflective dots on dark IR background'
    )

    # ── Output settings ──────────────────────────────────────────────────────
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Publish TF transforms for detected markers'
    )

    publish_2d_image_arg = DeclareLaunchArgument(
        'publish_2d_image',
        default_value='true',
        description='Publish debug image with axes drawn on detected markers'
    )

    publish_marker_array_arg = DeclareLaunchArgument(
        'publish_marker_array',
        default_value='true',
        description='Publish RViz marker array'
    )

    fiducials_node = Node(
        package='cob_fiducials',
        executable='fiducials',
        name='fiducials',
        namespace='fiducials',
        output='screen',
        parameters=[{
            'fiducial_type':                          LaunchConfiguration('fiducial_type'),
            'ros_node_mode':                          LaunchConfiguration('ros_node_mode'),
            'model_directory':                        model_dir,
            'model_filename':                         LaunchConfiguration('model_filename'),
            'compute_sharpness_measure':              False,
            'sharpness_calibration_parameter_m':      9139.749632393357,
            'sharpness_calibration_parameter_n':      -2670187.875850272,
            'log_or_calibrate_sharpness_measurements': False,
            'publish_marker_array':                   LaunchConfiguration('publish_marker_array'),
            'publish_tf':                             LaunchConfiguration('publish_tf'),
            'publish_2d_image':                       LaunchConfiguration('publish_2d_image'),
            'debug_verbosity':                        1,
            'use_ir_reflective_markers':              LaunchConfiguration('use_ir_reflective_markers'),
        }],
        remappings=[
            ('image_color',            LaunchConfiguration('image_topic')),
            ('camera_info',            LaunchConfiguration('camera_info_topic')),
            ('detect_fiducials',       '/fiducials/detect_fiducials'),
            ('image',                  '/fiducials/image'),
            ('get_fiducials',          '/fiducials/get_fiducials'),
            ('fiducial_marker_array',  '/fiducials/fiducial_marker_array'),
            ('fiducial_detection_array', '/fiducials/fiducial_detection_array'),
        ]
    )

    return LaunchDescription([
        image_topic_arg,
        camera_info_topic_arg,
        fiducial_type_arg,
        model_filename_arg,
        ros_node_mode_arg,
        ir_reflective_arg,
        publish_tf_arg,
        publish_2d_image_arg,
        publish_marker_array_arg,
        fiducials_node,
    ])
