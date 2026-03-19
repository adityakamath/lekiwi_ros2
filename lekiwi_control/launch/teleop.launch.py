#!/usr/bin/env python3
"""
Launch joystick teleoperation for LeKiWi robot.

Starts joy_teleop node to convert joystick inputs to velocity commands.
teleop_twist_joy kept as fallback (currently commented out).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description with configurable teleop parameters."""
    # Declare launch arguments
    # config_file_arg = DeclareLaunchArgument(  # teleop_twist_joy fallback
    #     'config_file',
    #     default_value=PathJoinSubstitution([
    #         FindPackageShare('lekiwi_control'),
    #         'config',
    #         'teleop_config.yaml'
    #     ]),
    #     description='Path to the teleop_twist_joy configuration file'
    # )

    joy_teleop_config_arg = DeclareLaunchArgument(
        "joy_teleop_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("lekiwi_control"), "config", "joy_teleop_config.yaml"]
        ),
        description="Path to the joy_teleop configuration file",
    )

    # Teleop twist joy node (commented out)
    # teleop_node = Node(
    #     package='teleop_twist_joy',
    #     executable='teleop_node',
    #     name='teleop_node',
    #     parameters=[LaunchConfiguration('config_file')],
    #     remappings=[
    #         ('/cmd_vel', '/base_controller/cmd_vel')
    #     ]
    # )

    # Joy teleop node
    joy_teleop_node = Node(
        package="joy_teleop",
        executable="joy_teleop",
        name="joy_teleop",
        parameters=[LaunchConfiguration("joy_teleop_config")],
        output="screen",
    )

    return LaunchDescription(
        [
            # config_file_arg,  # teleop_twist_joy fallback
            joy_teleop_config_arg,
            # teleop_node,  # teleop_twist_joy (commented out)
            joy_teleop_node,
        ]
    )
