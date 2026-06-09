#!/usr/bin/env python3
"""
Launch LD06 laser scanner for LeKiwi robot.

Starts ldlidar_ros2_node with configuration from laser.yaml.
"""

from launch import LaunchDescription

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare



def launch_setup(context):
    payload_value = LaunchConfiguration('payload').perform(context).lower()
    custom_filter_name = LaunchConfiguration('custom_filter').perform(context)
    config_file = PathJoinSubstitution([
        FindPackageShare('lekiwi_bringup'),
        'config',
        'laser.yaml'
    ])

    if payload_value == 'pantilt':
        filter_config_file = PathJoinSubstitution([
            FindPackageShare('lekiwi_bringup'),
            'config',
            'laser_filter.yaml'
        ])
        laser_node = Node(
            package='ldlidar_ros2',
            executable='ldlidar_ros2_node',
            name='laser_node',
            output='log',
            respawn=True,
            parameters=[config_file],
            remappings=[('scan', 'scan_raw')]
        )
        filter_node = Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_scan_filter_chain',
            output='log',
            parameters=[filter_config_file],
            remappings=[('scan', 'scan_raw'), ('scan_filtered', 'scan')]
        )
        return [laser_node, filter_node]
    elif custom_filter_name:
        custom_filter_file = PathJoinSubstitution([
            FindPackageShare('lekiwi_bringup'),
            'config',
            custom_filter_name
        ])
        laser_node = Node(
            package='ldlidar_ros2',
            executable='ldlidar_ros2_node',
            name='laser_node',
            output='log',
            respawn=True,
            parameters=[config_file],
            remappings=[('scan', 'scan_raw')]
        )
        filter_node = Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_scan_filter_chain',
            output='log',
            parameters=[custom_filter_file],
            remappings=[('scan', 'scan_raw'), ('scan_filtered', 'scan')]
        )
        return [laser_node, filter_node]
    else:
        laser_node = Node(
            package='ldlidar_ros2',
            executable='ldlidar_ros2_node',
            name='laser_node',
            output='log',
            respawn=True,
            parameters=[config_file]
        )
        return [laser_node]

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'payload',
            default_value='',
            description='Hardware payload: "" for base only, "pantilt" for base + pan-tilt',
        ),
        DeclareLaunchArgument(
            'custom_filter',
            default_value='',
            description='(Optional) Name of custom filter config YAML in lekiwi_bringup/config/',
        ),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
