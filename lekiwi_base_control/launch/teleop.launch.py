#!/usr/bin/env python3
"""
Launch joystick teleoperation for LeKiwi robot.

Starts teleop_twist_joy node to convert joystick inputs to velocity commands.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description with configurable teleop parameters."""
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('lekiwi_base_control'),
            'config',
            'teleop_config.yaml'
        ]),
        description='Path to the teleop configuration file'
    )
    
    # Teleop twist joy node
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('/cmd_vel', '/base_controller/cmd_vel')
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        teleop_node,
    ])
