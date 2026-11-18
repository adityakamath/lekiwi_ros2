#!/usr/bin/env python3
"""
Launch LD06 laser scanner for LeKiwi robot.

Starts ldlidar_ros2_node with configuration from laser.yaml. The optional custom_filter argument selects an explicit scan filter.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    """Launch ldlidar_ros2_node, plus a scan filter chain when a filter config is resolved."""
    custom_filter_name = LaunchConfiguration('custom_filter').perform(context)
    pkg_bringup = FindPackageShare('lekiwi_bringup').perform(context)
    config_file = PathJoinSubstitution([pkg_bringup, 'config', 'laser.yaml'])

    filter_config_file = ''
    if custom_filter_name:
        filter_config_file = os.path.join(pkg_bringup, 'config', custom_filter_name)
        if not os.path.exists(filter_config_file):
            raise RuntimeError(
                f"[laser.launch.py] custom_filter '{custom_filter_name}' not found: "
                f"expected {filter_config_file}."
            )
    laser_params = [config_file]
    laser_remappings = []
    if filter_config_file:
        laser_remappings = [('scan', 'scan_raw')]

    laser_node = Node(
        package='ldlidar_ros2',
        executable='ldlidar_ros2_node',
        name='laser_node',
        output='log',
        respawn=True,
        parameters=laser_params,
        remappings=laser_remappings,
    )

    if not filter_config_file:
        return [laser_node]

    filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_scan_filter_chain',
        output='log',
        parameters=[filter_config_file],
        remappings=[('scan', 'scan_raw'), ('scan_filtered', 'scan')],
    )
    return [laser_node, filter_node]

def generate_launch_description():
    """Declare custom_filter and launch the laser stack via launch_setup."""
    declared_arguments = [
        DeclareLaunchArgument(
            'custom_filter',
            default_value='',
            description='(Optional) Override filter config path relative to lekiwi_bringup/config/.',
        ),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
