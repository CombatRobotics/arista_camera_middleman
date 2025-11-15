import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from std_msgs.msg import Empty
from arista_interfaces.msg import AvailableRobot
import subprocess
import platform
import subprocess
import re
import time
import statistics
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy,ReliabilityPolicy,HistoryPolicy
from rclpy.node import Node as rclNode
import os
def ping_host(host, count=4):
    # Determine OS-specific ping parameters
    param = '-n' if platform.system().lower() == 'windows' else '-c'
    command = ['ping', param, str(count), host]

    try:
        output = subprocess.check_output(command, stderr=subprocess.STDOUT, universal_newlines=True)
        
        latencies = [float(ms) for ms in re.findall(r'time[=<](\d+\.\d+)', output)]

        if latencies:
            mean_latency = statistics.mean(latencies)
            return mean_latency
        else:
            return -1
    except subprocess.CalledProcessError:
        return -2


class MiddlemanNode(rclNode):
    def __init__(self):
        super().__init__('middleman_node')
        qos_profile = QoSProfile(
            # depth=10,
            # durability=DurabilityPolicy.TRANSIENT_LOCAL  # or DurabilityPolicy.VOLATILE
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # self._publisher = self.create_publisher (Empty, '/request_available_robots', 10)
        # self._subscriber = self.create_subscription(AvailableRobot, '/available_robots', self._robot_callback, qos_profile)
        # self.robots = dict()
        self.get_logger().info('middleman node started')

    def _robot_callback(self, msg: AvailableRobot):
        self.robots[msg.robot_name] = msg

def get_default_config():
    msg = AvailableRobot()
    msg.robot_name = 'arista4'
    msg.robot_ip = '192.168.1.30'
    return msg

def get_robot_config():
    node = MiddlemanNode()
    start_time = None
    while True:
        node.get_logger().info('requesting available robots')
        node._publisher.publish(Empty())
        rclpy.spin_once(node, timeout_sec=1)
        if start_time is None:
            if len(node.robots) > 0:
                start_time = node.get_clock().now()
        else:
            if (node.get_clock().now().to_msg().sec - start_time.to_msg().sec) > 10:
                break
        time.sleep(1)
    min_ping = None
    probable_robot = None
    for name, robot in node.robots.items():
        _ping = ping_host(robot.robot_ip)
        if _ping >= 0:
            if min_ping is None or _ping < min_ping:
                min_ping = _ping
                probable_robot = robot
    if min_ping is not None and min_ping >=0 and min_ping < 1.5:
        return probable_robot
    else:
        return None

def make_launch_desc(robot_conf:AvailableRobot):
    ip = robot_conf.robot_ip
    namespace = robot_conf.robot_name
    print('ip: {}'.format(ip))
    can_control_node = Node(
        package='arista_camera_middleman',
        executable='can_control',
        name='can_control',
        namespace=namespace,
        output='screen',
        # respawn=True
    )
    zoom_control_node = Node(
        package='arista_camera_middleman',
        executable='zoom_control',
        name='zoom_control',
        namespace=namespace,
        output='screen',
        # respawn=True
    )
    thermal_cam_stream = Node(
        package='arista_video_stream',
        executable='rpicam_stream.py',
        name='thermal_cam_stream',
        namespace=namespace,
        output='screen',
        parameters=[{
            'robot_ip': ip,
            'camera_id': 'thermal',
            'port': 5032
        }],
        # respawn=True
    )
    rgb_cam_stream = Node(
        package='arista_video_stream',
        executable='rpicam_stream.py',
        name='rgb_cam_stream',
        namespace=namespace,
        output='screen',
        parameters=[{
            'robot_ip': ip,
            'camera_id': 'zoom_rgb',
            'port': 5035
        }],
        # respawn=True
    )
    xbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('arista_joy_controller'), 
                'launch', 'xbox_controller.launch.py'
            )
        ),
        launch_arguments={
            'device_name': namespace
        }.items()
    )
    return launch.LaunchDescription([
        can_control_node,
        zoom_control_node,
        thermal_cam_stream,
        rgb_cam_stream,
        xbox_node,
    ])

def generate_launch_description():
    rclpy.init()
    robot_conf = get_default_config()
    while robot_conf is None:
        robot_conf = get_robot_config()
        time.sleep(3)
    return make_launch_desc(robot_conf)

# if __name__ == '__main__':
    # generate_launch_description()
