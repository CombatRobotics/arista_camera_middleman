import launch
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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
        )
        self._publisher = self.create_publisher (Empty, '/request_available_robots', 10)
        self._subscriber = self.create_subscription(AvailableRobot, '/available_robots', self._robot_callback, qos_profile)
        self.robots = dict()
        self.get_logger().info('middleman node started')

    def _robot_callback(self, msg: AvailableRobot):
        self.robots[msg.robot_name] = msg

def get_robot_config():
    node = MiddlemanNode()
    # Request available robots
    node._publisher.publish(Empty())
    
    # Wait for robots to be discovered
    while len(node.robots) == 0:
        rclpy.spin_once(node, timeout_sec=1.0)
        node._publisher.publish(Empty())  # Keep requesting
        node.get_logger().info('Waiting for available robots...')
    
    # Return first available robot
    robot = list(node.robots.values())[0]
    node.get_logger().info(f'Found robot: {robot.robot_name} at {robot.robot_ip}')
    node.destroy_node()
    return robot

def make_launch_desc(robot_conf:AvailableRobot):
    ip = robot_conf.robot_ip
    namespace = robot_conf.robot_name
    can_control_node = Node(
        package='arista_camera_middleman',
        executable='can_control',
        name='can_control',
        namespace=namespace,
        output='log',
        respawn=True
    )
    zoom_control_node = Node(
        package='harshcam_camera_control',
        executable='camera_control_node',
        name='camera_control_node',
        namespace=namespace,
        output='screen',
        parameters=[{
            'device': '/dev/harshcam_zoom_control',
            'baudrate': 115200
        }],
        respawn=True
    )
    thermal_cam_stream = ExecuteProcess(
        cmd=[
            'gst-launch-1.0',
            'v4l2src', 'device=/dev/thermal_cam_digitalcore', '!',
            'video/x-raw,width=640,height=480', '!',
            'videoconvert', '!',
            'x264enc', 'tune=zerolatency', 'bitrate=1000', 'speed-preset=ultrafast', '!',
            'rtph264pay', 'config-interval=1', 'pt=96', '!',
            'udpsink', f'host={ip}', 'port=5032',
        ],
        output='screen',
    )
    rgb_cam_stream = ExecuteProcess(
        cmd=[
            'gst-launch-1.0',
            'v4l2src', 'device=/dev/rgb_zoom_harshcam', '!',
            'video/x-raw,width=1920,height=1080', '!',
            'videoconvert', '!',
            'x264enc', 'tune=zerolatency', 'bitrate=4000', 'speed-preset=ultrafast', '!',
            'rtph264pay', 'config-interval=1', 'pt=96', '!',
            'udpsink', f'host={ip}', 'port=5035',
        ],
        output='screen',
    )
    return launch.LaunchDescription([
        can_control_node,
        zoom_control_node,
        thermal_cam_stream,
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
