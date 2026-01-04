#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('lekiwi_bringup'),
        'config',
        'laser_config.yaml'
    ])

    laser_node = Node(
        package='ldlidar_ros2',
        executable='ldlidar_ros2_node',
        name='laser_node',
        output='log',
        respawn=True,
        parameters=[config_file]
    )

    return LaunchDescription([
        laser_node
    ])
