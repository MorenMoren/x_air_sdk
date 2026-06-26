from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="teleop_pad_bridge",
            executable="teleop_pad_bridge",
            name="teleop_pad_bridge",
            output="screen"
        ),
        Node(
            package="ros_tcp_endpoint",
            executable="default_server_endpoint",
            emulate_tty=True,
            parameters=[{"ROS_IP": "0.0.0.0"}, {"ROS_TCP_PORT": 10000}],
        )
    ])  