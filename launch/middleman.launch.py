import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='arista1',
        description='Namespace for all camera middleman nodes.',
    )
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.64',
        description='Target robot IP for video streams.',
    )
    preview_arg = DeclareLaunchArgument(
        'preview',
        default_value='true',
        description='Set true to mirror each camera feed locally during streaming.',
    )

    namespace = LaunchConfiguration('namespace')
    robot_ip = LaunchConfiguration('robot_ip')
    preview = LaunchConfiguration('preview')
    profile_params = os.path.join(
        get_package_share_directory('arista_video_stream'),
        'config',
        'rpicam_stream.yaml',
    )

    can_control_node = Node(
        package='arista_camera_middleman',
        executable='can_control',
        name='can_control',
        namespace=namespace,
        output='screen',
    )
    zoom_control_node = Node(
        package='harshcam_camera_control',
        executable='camera_control_node',
        name='harshcam_camera_control',
        namespace=namespace,
        output='screen',
    )
    thermal_cam_stream = Node(
        package='arista_video_stream',
        executable='rpicam_stream.py',
        name='thermal_cam_stream',
        namespace=namespace,
        output='screen',
        parameters=[
            profile_params,
            {
                'robot_ip': robot_ip,
                'camera_id': 'thermal',
                'preview': preview,
            },
        ],
    )
    rgb_cam_stream = Node(
        package='arista_video_stream',
        executable='rpicam_stream.py',
        name='rgb_cam_stream',
        namespace=namespace,
        output='screen',
        parameters=[
            profile_params,
            {
                'robot_ip': robot_ip,
                'camera_id': 'rgb_zoom_harshcam',
                'preview': preview,
            },
        ],
    )
    webcam_stream = Node(
        package='arista_video_stream',
        executable='rpicam_stream.py',
        name='webcam_stream',
        namespace=namespace,
        output='screen',
        parameters=[
            profile_params,
            {
                'robot_ip': robot_ip,
                'camera_id': 'webcam',
                'preview': preview,
            },
        ],
    )
    xbox_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('arista_camera_middleman'),
                'launch',
                'xbox_controller.launch.py',
            )
        ),
        launch_arguments={
            'device_name': namespace,
        }.items(),
    )

    return LaunchDescription([
        namespace_arg,
        robot_ip_arg,
        preview_arg,
        # can_control_node,
        # zoom_control_node,
        # thermal_cam_stream,
        webcam_stream,
        # rgb_cam_stream,
        xbox_controller,
    ])
