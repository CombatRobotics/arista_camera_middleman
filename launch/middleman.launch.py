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
import statistics
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy

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


class MiddlemanNode(rclpy.Node):
    def __init__(self):
        super().__init__('middleman_node')
        qos_profile = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL  # or DurabilityPolicy.VOLATILE
        )
        self._publisher = self.create_publisher (Empty, '/request_available_robots', 10)
        self._subscriber = self.create_subscription(AvailableRobot, '/available_robots', self._robot_callback, qos_profile)
        self.robots = dict()
        self.get_logger().info('middleman node started')

    def _robot_callback(self, msg: AvailableRobot):
        self.robots[msg.robot_name] = msg

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
            if (node.get_clock().now().to_msg().sec - start_time.sec) > 5:
                break
    min_ping = None
    probable_robot = None
    for name, robot in node.robots.items():
        _ping = ping_host(robot.robot_ip)
        if _ping >= 0:
            if min_ping is None or _ping < min_ping:
                min_ping = _ping
                probable_robot = robot
    return probable_robot

def mak_launch_desc(robot_conf:AvailableRobot):
    ip = robot_conf.robot_ip
    namespace = robot_conf.robot_name
    print(robot_conf)
    return launch.LaunchDescription([
        
    ])

def generate_launch_description():
    robot_conf = get_robot_config()
    while robot_conf is None:
        robot_conf = get_robot_config()
    return mak_launch_desc(robot_conf)

# if __name__ == '__main__':
    # generate_launch_description()
