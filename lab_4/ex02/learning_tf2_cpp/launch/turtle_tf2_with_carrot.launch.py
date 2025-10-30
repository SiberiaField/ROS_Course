from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('learning_tf2_cpp'), 'launch', 'turtle_tf2.launch.py']),
            launch_arguments={'target_frame': 'carrot1'}.items(),
        ),
        DeclareLaunchArgument(
            'radius', default_value='1.0',
            description='Radius for carrot frame'
        ),
        DeclareLaunchArgument(
            'direction_of_rotation', default_value='1',
            description='Direction of carrot rotation'
        ),
        Node(
            package='learning_tf2_cpp',
            executable='carrot_tf2_broadcaster',
            name='carrot_broadcaster',
            parameters=[
                {'radius': LaunchConfiguration('radius'),
                 'direction_of_rotation': LaunchConfiguration('direction_of_rotation')}
            ]
        )
    ])
