import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('crab_bot_bringup')
    pkg_project_description = get_package_share_directory('crab_bot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(pkg_project_description, 'urdf', 'crab_bot.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={"gz_args": "-r gpu_lidar_sensor.sdf"}.items(),
    )

    # Spawn robot
    create = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'crab_bot',
                   '-topic', '/robot_description',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.1'
                   ],
        output='screen',
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {
                'robot_description': robot_desc,
                'use_sim_time': True
            }
        ]
    )

    # RViz для визуализации
    rviz_config_path = os.path.join(pkg_project_bringup, 'config', 'crab_bot.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
        output="screen"
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'crab_bot_bridge_lidar.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # Исправим порядок запуска с помощью таймеров
    delayed_spawn = TimerAction(
        period=3.0,
        actions=[create]
    )

    delayed_bridge = TimerAction(
        period=5.0,  # Даем время появиться роботу в Gazebo
        actions=[bridge]
    )

    delayed_rviz = TimerAction(
        period=7.0,
        actions=[rviz]
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        gz_sim,
        robot_state_publisher,
        delayed_spawn,
        delayed_bridge,
        delayed_rviz
    ])
