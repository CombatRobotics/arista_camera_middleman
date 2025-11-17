from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():

    # Declare launch arguments

    # Joystick device name, Used as namespace for the joystick node
    device_name = DeclareLaunchArgument(
        'device_name',
        default_value='',
        description='Name of the joystick device'
    )

    # Used to specify if the robot namespace should be used, for command inputs
    use_robot_namespace = DeclareLaunchArgument(
        'use_robot_namespace',
        default_value='false',
        description='Use robot namespace'
    )

    # Used to specify if the joystick should be throttled
    throttle_joystick_arg = DeclareLaunchArgument(
        'throttle_joystick',
        default_value='false',
        description='Throttle joystick'
    )

    # Used for arming and cmd_vel priority
    priority_arg = DeclareLaunchArgument(
        'controller_priority',
        default_value='false',
        description='Check priority of the controller'
    )

    # Used to specify if the joy feedback should be used
    joy_feedback_available = DeclareLaunchArgument(
        'joy_feedback_available',
        default_value='false',
        description='Joy feedback available'
    )

    # Used to specify if the touch mode should be used
    touch_mode_available = DeclareLaunchArgument(
        'touch_mode_available',
        default_value='false',
        description='Touch mode available'
    )

    controller_type = DeclareLaunchArgument(
        'controller_type',
        default_value='XBOX',
        description='Controller type'
    )

    # ---------------------------------------------------------------------
    # 2.  Throttling
    # ---------------------------------------------------------------------
    throttle_node = Node(
        package="topic_tools",
        executable="throttle",
        name="joy_throttle",
        namespace=LaunchConfiguration('device_name'),
        condition=IfCondition(LaunchConfiguration('throttle_joystick')),
        arguments=[
            "messages",          # throttle type
            "joy",               # input
            "5",                 # Hz
            "joy/throttled"      # output
        ],
        output="screen",
    )

    # ---------------------------------------------------------------------
    # 3.  Joystick node
    # ---------------------------------------------------------------------
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=LaunchConfiguration('device_name'),
    )

    # ---------------------------------------------------------------------
    # 4.  Xbox controller node
    # ---------------------------------------------------------------------
    xbox_controller = Node(
        package='arista_joy_controller',
        executable='joy_controller',
        name='xbox_controller',
        output='screen',
        emulate_tty=True,
        namespace=LaunchConfiguration('device_name'),
        parameters=[{
            'throttle_joystick': LaunchConfiguration('throttle_joystick'),
            'use_robot_namespace': LaunchConfiguration('use_robot_namespace'),
            'controller_priority': LaunchConfiguration('controller_priority'),
            'joy_feedback_available': LaunchConfiguration('joy_feedback_available'),
            'touch_mode_available': LaunchConfiguration('touch_mode_available'),
            'controller_type': LaunchConfiguration('controller_type'),
        }]
    )

    # ---------------------------------------------------------------------
    # 5.  Launch description
    # ---------------------------------------------------------------------
    return LaunchDescription([
        use_robot_namespace,
        priority_arg,
        throttle_joystick_arg,
        joy_feedback_available,
        touch_mode_available,
        controller_type,
        device_name,
        throttle_node,
        joy_node,
        xbox_controller
    ])
