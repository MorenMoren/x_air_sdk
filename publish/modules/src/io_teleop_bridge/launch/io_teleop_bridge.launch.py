"""Launch file for io_teleop_bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="forward",
            description="Bridge mode: forward | trajectory",
        ),
        DeclareLaunchArgument(
            "rate",
            default_value="100.0",
            description="Update rate in Hz",
        ),
        DeclareLaunchArgument(
            "trajectory_time",
            default_value="0.5",
            description="Trajectory time_from_start in seconds",
        ),
        Node(
            package="io_teleop_bridge",
            executable="bridge_node",
            name="io_teleop_bridge",
            output="screen",
            parameters=[{
                "mode": LaunchConfiguration("mode"),
                "rate": LaunchConfiguration("rate"),
                "trajectory_time": LaunchConfiguration("trajectory_time"),
            }],
        ),
    ])
