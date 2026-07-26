#!/usr/bin/env python3
"""Launch the LeKiwi audio indicator node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lekiwi_audio',
            executable='indicator_node',
            name='indicator_node',
            output='log',
            respawn=True,
        ),
    ])
