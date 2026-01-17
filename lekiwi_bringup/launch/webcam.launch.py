#!/usr/bin/env python3
"""
Launch HD webcam for LeKiwi robot.

Starts camera_node with configuration and calibration from webcam config files.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for HD webcam."""
    config_file = PathJoinSubstitution([
        FindPackageShare('lekiwi_bringup'),
        'config',
        'webcam_config.yaml'
    ])
    
    camera_info_file = PathJoinSubstitution([
        FindPackageShare('lekiwi_bringup'),
        'config',
        'webcam_calibration.yaml'
    ])

    # Create file:// URL for camera calibration
    camera_info_url = ['file://', camera_info_file]
    
    webcam_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='webcam_node',
        output='log',
        respawn=True,
        parameters=[
            config_file,
            {'camera_info_url': camera_info_url}
        ]
    )

    return LaunchDescription([
        webcam_node
    ])