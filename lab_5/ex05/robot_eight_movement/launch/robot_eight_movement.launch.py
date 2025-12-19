from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_prefix", default_value="crab_bot",
            description="prefix for /cmd_vel topic"
        ),
        Node(
            package="robot_eight_movement",
            executable="robot_eight_movement",
            name="robot_eight_movement",
            parameters=[
                {"robot_prefix": LaunchConfiguration("robot_prefix")}
            ]
        )
    ])
