#!/usr/bin/env python3
"""
Launch LD06 laser scanner for LeKiwi robot.

Starts ldlidar_ros2_node with configuration from laser.yaml. A payload-specific
laser filter is used automatically when one exists; custom_filter overrides it.
"""

import os

from launch import LaunchDescription

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare



def launch_setup(context):
    """Launch ldlidar_ros2_node, plus a scan filter chain when a filter config is resolved."""
    payload_value = LaunchConfiguration('payload').perform(context).lower()
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
    elif payload_value:
        payload_filter_file = os.path.join(
            pkg_bringup, 'config', 'payloads', payload_value, 'laser_filter.yaml'
        )
        if os.path.exists(payload_filter_file):
            filter_config_file = payload_filter_file

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
    """Declare payload/custom_filter and launch the laser stack via launch_setup."""
    declared_arguments = [
        DeclareLaunchArgument(
            'payload',
            default_value='',
            description='Hardware payload: "" for base only, "pantilt" for base + pan-tilt',
        ),
        DeclareLaunchArgument(
            'custom_filter',
            default_value='',
            description='(Optional) Override filter config path relative to lekiwi_bringup/config/.',
        ),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
