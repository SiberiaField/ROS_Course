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
        DeclareLaunchArgument(
            "angular_vel", default_value="1.570796",
            description="robot's angular velocity"
        ),
        DeclareLaunchArgument(
            "linear_vel", default_value="0.3",
            description="circle linear_vel"
        ),
        Node(
            package="robot_circle_movement",
            executable="robot_circle_movement",
            name="robot_circle_movement",
            parameters=[
                {"robot_prefix": LaunchConfiguration("robot_prefix")},
                {"angular_vel": LaunchConfiguration("angular_vel")},
                {"linear_vel": LaunchConfiguration("linear_vel")}
            ]
        )
    ])
