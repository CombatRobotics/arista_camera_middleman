import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
        )\
        self.declare_parameter('max_latency', 1.5)
        self.max_latency = self.get_parameter('max_latency').value
        self._publisher = self.create_publisher(Empty, '/request_available_robots', 10)
        self._subscriber = self.create_subscription(AvailableRobot, '/available_robots', self._robot_callback, qos_profile)
        self.robots = dict()
        self.selected_robot = None  # Robot with latency < 1ms
        self.get_logger().info('middleman node started')

    def _robot_callback(self, msg: AvailableRobot):
        # Skip if we already found a suitable robot
        if self.selected_robot is not None:
            return

        # Skip if we already checked this robot
        if msg.robot_name in self.robots:
            return

        self.robots[msg.robot_name] = msg
        self.get_logger().info(f'Discovered robot: {msg.robot_name} at {msg.robot_ip}, pinging...')

        # Ping the robot immediately
        latency = ping_host(msg.robot_ip, count=2)  # Use fewer pings for faster check

        if latency >= 0:
            self.get_logger().info(f'Robot {msg.robot_name} latency: {latency:.2f}ms')
            if latency < self.max_latency:
                self.get_logger().info(f'Selected robot {msg.robot_name} with latency {latency:.2f}ms < {self.max_latency}ms')
                self.selected_robot = msg
        else:
            self.get_logger().warn(f'Failed to ping robot {msg.robot_name} at {msg.robot_ip}')

def get_robot_config():
    node = MiddlemanNode()
    max_wait_time = 30  # Maximum seconds to wait for a suitable robot
    start_time = node.get_clock().now()

    while True:
        node.get_logger().info('requesting available robots')
        node._publisher.publish(Empty())
        rclpy.spin_once(node, timeout_sec=1)

        # Check if we found a robot with latency < 1ms
        if node.selected_robot is not None:
            node.get_logger().info(f'Using robot: {node.selected_robot.robot_name}')
            return node.selected_robot

        # Check timeout
        elapsed = (node.get_clock().now() - start_time).nanoseconds / 1e9
        if elapsed > max_wait_time:
            node.get_logger().warn('Timeout waiting for robot with latency < 1ms')
            break

        time.sleep(0.5)

    # Fallback: if no robot with <1ms latency found, return None
    node.get_logger().error('No robot found with latency < 1ms')
    return None

def make_launch_desc(robot_conf:AvailableRobot):
    ip = robot_conf.robot_ip
    namespace = robot_conf.robot_name
    can_control_node = Node(
        package='arista_camera_middleman',
        executable='can_control',
        name='can_control',
        namespace=namespace,
        output='screen',
        respawn=True
    )
    zoom_control_node = Node(
        package='arista_camera_middleman',
        executable='zoom_control',
        name='zoom_control',
        namespace=namespace,
        output='screen',
        respawn=True
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
        respawn=True
    )
    rgb_cam_stream = Node(
        package='arista_video_stream',
        executable='rpicam_stream.py',
        name='rgb_cam_stream',
        namespace=namespace,
        output='screen',
        parameters=[{
            'robot_ip': ip,
            'camera_id': '/dev/video0',
            'port': 5035
        }],
        respawn=False
    )
    return launch.LaunchDescription([
        # can_control_node,
        # zoom_control_node,
        # thermal_cam_stream,
        rgb_cam_stream,
    ])

def generate_launch_description():
    rclpy.init()
    robot_conf = get_robot_config()
    while robot_conf is None:
        robot_conf = get_robot_config()
        time.sleep(3)
    return make_launch_desc(robot_conf)

# if __name__ == '__main__':
    # generate_launch_description()
