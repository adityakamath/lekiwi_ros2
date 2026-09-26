"""Launch the optional target tracker independently of Nav2 bringup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('lekiwi_navigation'), 'config', 'nav2', 'nav2_target.yaml',
        ])),
        Node(
            package='lekiwi_navigation', executable='nav2_target_node',
            name='nav2_target_node', output='screen',
            parameters=[LaunchConfiguration('params_file'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
