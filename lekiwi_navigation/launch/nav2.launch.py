#!/usr/bin/env python3
"""
Launch slam_toolbox for LeKiwi.

Launch arguments:
    slam_mode    map | localize   (default: map)
    map_name     subdirectory name under maps/ (required for localize)
    use_sim_time true | false     (default: false)

Invoked from navigation.launch.py for slam_mode:=map and slam_mode:=localize.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import yaml


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_mode    = LaunchConfiguration('slam_mode').perform(context)
    map_name     = LaunchConfiguration('map_name').perform(context)

    slam_params = PathJoinSubstitution([
        FindPackageShare('lekiwi_navigation'), 'config', 'nav2', 'slam_toolbox.yaml',
    ])

    # slam_toolbox internal mode names
    _st_mode = {'map': 'mapping', 'localize': 'localization'}
    extra_params = {'use_sim_time': use_sim_time, 'mode': _st_mode[slam_mode]}

    if slam_mode == 'localize':
        if not map_name:
            raise RuntimeError(
                "[nav2.launch.py] slam_mode:=localize requires map_name to be set. "
                "e.g. map_name:=livingroom1"
            )
        # os.path.realpath resolves the symlink created by --symlink-install so
        # that _pkg_src always points to the source tree where maps/ lives.
        # Maps are not installed — they remain in the source tree only.
        _pkg_src = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        map_file_path = os.path.join(_pkg_src, 'maps', map_name, 'map')
        extra_params['map_file_name'] = map_file_path

        # Load saved starting pose so slam_toolbox starts from the right position.
        pose_file = os.path.join(_pkg_src, 'maps', map_name, 'starting_pose.yaml')
        if os.path.exists(pose_file):
            with open(pose_file) as f:
                pose = yaml.safe_load(f)
            extra_params['map_start_pose'] = [pose['x'], pose['y'], pose['theta']]
        # If no starting_pose.yaml, slam_toolbox starts at map origin (0, 0, 0).

    return [
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, extra_params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['slam_toolbox'],
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_mode', default_value='map',
            description="slam_toolbox mode: 'map' (build new map) or 'localize' (re-use serialized map).",
        ),
        DeclareLaunchArgument(
            'map_name', default_value='',
            description=(
                'Subdirectory name of the map to load for localize mode '
                '(e.g. livingroom1). The full path is constructed automatically.'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock from a sim instead of system time.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
