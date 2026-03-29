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

    teleop_config_arg = DeclareLaunchArgument(
        "teleop_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("lekiwi_control"), "config", "teleop_config.yaml"]
        ),
        description="Path to the joy_teleop configuration file",
    )

    # Joy teleop node
    joy_teleop_node = Node(
        package="joy_teleop",
        executable="joy_teleop",
        name="joy_teleop",
        parameters=[LaunchConfiguration("teleop_config")],
        output="screen",
    )

    return LaunchDescription(
        [
            teleop_config_arg,
            joy_teleop_node,
        ]
    )
