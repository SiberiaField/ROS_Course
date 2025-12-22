import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('crab_bot_bringup')

    # Setup to launch the simulator and Gazebo world
    crab_bot_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_bringup, 'launch', 'crab_bot_sensors.launch.py'))
    )

    lidar_dodger = Node(
        package='crab_bot_dodge_lidar',
        executable='lidar_dodger',
        name='lidar_dodger'
    )

    delayed_lidar_dodger = TimerAction(
        period=2.0,
        actions=[lidar_dodger]
    )

    return LaunchDescription([
        crab_bot_sensors,
        delayed_lidar_dodger
    ])
