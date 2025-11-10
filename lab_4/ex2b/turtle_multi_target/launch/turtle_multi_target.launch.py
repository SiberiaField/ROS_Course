from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        DeclareLaunchArgument(
            'radius', default_value='1.0',
            description='Radius for carrot1 frame'
        ),
        DeclareLaunchArgument(
            'direction_of_rotation', default_value='1',
            description='Direction of rotation for carrot1 frame'
        ),
        DeclareLaunchArgument(
            'switch_threshold', default_value='1.0',
            description='Switch threshold for target_switcher node'
        ),
        Node(
            package='turtle_multi_target',
            executable='turtle_switcher',
            name='turtle_switcher',
            parameters=[
                {'radius': LaunchConfiguration('radius')},
                {'direction_of_rotation': LaunchConfiguration('direction_of_rotation')},
                {'switch_threshold': LaunchConfiguration('switch_threshold')}
            ]
        ),
        Node(
            package='turtle_multi_target',
            executable='turtle_controller',
            name='turtle_controller'
        )
    ])
