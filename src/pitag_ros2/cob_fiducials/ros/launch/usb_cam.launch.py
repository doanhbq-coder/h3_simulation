from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    device_arg = DeclareLaunchArgument(
        'video_device', default_value='/dev/video0',
        description='V4L2 device path')

    width_arg = DeclareLaunchArgument(
        'image_width', default_value='640',
        description='Capture width in pixels')

    height_arg = DeclareLaunchArgument(
        'image_height', default_value='480',
        description='Capture height in pixels')

    fps_arg = DeclareLaunchArgument(
        'framerate', default_value='30.0',
        description='Frames per second')

    # ── MJPEG thay vì YUYV ─────────────────────────────────────────────────
    # YUYV raw ở 640×480@30fps ≈ 27 MB/s → vượt quá băng thông thực tế của USB 2.0.
    # MJPEG nén tại hardware camera → ~1-3 MB/s, không còn bị drop frame.
    # Cheese mặc định dùng MJPEG, đó là lý do nó mượt còn ROS2 thì lag.
    format_arg = DeclareLaunchArgument(
        'pixel_format', default_value='mjpeg2rgb',
        description='Pixel format: mjpeg (recommended for USB bandwidth), yuyv, rgb8')

    camera_name_arg = DeclareLaunchArgument(
        'camera_name', default_value='imx335',
        description='Camera name — sets topic namespace /camera/<name>/...')

    frame_id_arg = DeclareLaunchArgument(
        'camera_frame_id', default_value='camera_frame_optical',
        description='TF frame ID in image headers')

    calib_file_arg = DeclareLaunchArgument(
        'camera_info_url', default_value='file:///home/neo/.ros/camera_info/imx335.yaml',
        description='Camera calibration YAML, e.g. file:///home/user/cam.yaml')

    camera_name = LaunchConfiguration('camera_name')

    # ── USB cam node ────────────────────────────────────────────────────────
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace=['camera/', camera_name],
        output='screen',
        parameters=[{
            'video_device':       LaunchConfiguration('video_device'),
            'image_width':        LaunchConfiguration('image_width'),
            'image_height':       LaunchConfiguration('image_height'),
            'framerate':          LaunchConfiguration('framerate'),
            'pixel_format':       LaunchConfiguration('pixel_format'),
            'camera_name':        camera_name,
            'camera_frame_id':    LaunchConfiguration('camera_frame_id'),
            'frame_id':           LaunchConfiguration('camera_frame_id'),
            'camera_info_url':    LaunchConfiguration('camera_info_url'),
            'io_method':          'mmap',
            'buffer_count':       4,        # tăng buffer để tránh drop frame
            'auto_white_balance': True,
            'autoexposure':       True,
        }],
        remappings=[
            ('image_raw',   'image_raw'),
            ('camera_info', 'camera_info'),
        ]
    )

    # ── image_transport republish: MJPEG → raw ─────────────────────────────
    # Khi pixel_format=mjpeg, usb_cam publish topic compressed (sensor_msgs/CompressedImage).
    # Node này decode MJPEG → sensor_msgs/Image (raw) để các node downstream
    # (pitag detector, rqt image_view) subscribe bình thường qua /image_raw.
    #
    # Nếu node pitag của bạn dùng image_transport và subscribe "compressed"
    # trực tiếp được thì có thể bỏ node này để tiết kiệm CPU.
    decompress_node = Node(
        package='image_transport',
        executable='republish',
        name='mjpeg_decompress',
        output='screen',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed',  ['camera/', camera_name, '/image_raw/compressed']),
            ('out',            ['camera/', camera_name, '/image_raw']),
        ]
    )

        # ── YUYV → mono8 converter ──────────────────────────────────────────────
    # Camera phần cứng không xuất mono8 trực tiếp.
    # Node này lấy kênh Y (luminance) từ YUYV → mono8, giảm băng thông ~3×.
    mono8_converter_node = Node(
        package='cob_fiducials',
        executable='yuyv_to_mono8',
        name='yuyv_to_mono8',
        output='screen',
        parameters=[{
            'input_topic':  ['/camera/', camera_name, '/image_raw'],
            'output_topic': ['/camera/', camera_name, '/image_mono8'],
        }]
    )

    return LaunchDescription([
        device_arg,
        width_arg,
        height_arg,
        fps_arg,
        format_arg,
        camera_name_arg,
        frame_id_arg,
        calib_file_arg,
        usb_cam_node,
        decompress_node,
        mono8_converter_node,
    ])