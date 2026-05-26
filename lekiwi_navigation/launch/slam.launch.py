#!/usr/bin/env python3
"""
Launch localization and SLAM for LeKiwi.

Handles all three slam_mode cases:
    map      — slam_toolbox in mapping mode + map_saver_node (joystick R1 to save)
    localize — slam_toolbox in localization mode (re-use serialized map)
    amcl     — nav2_map_server + AMCL (localize on a static map)

Invoked from navigation.launch.py.

Launch arguments:
    slam_mode    map | localize | amcl  (default: map)
    map_name     subdirectory name under maps/ (required for localize and amcl)
    use_sim_time true | false  (default: false)
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_mode    = LaunchConfiguration('slam_mode').perform(context)
    map_name     = LaunchConfiguration('map_name').perform(context)

    if slam_mode not in ('map', 'localize', 'amcl'):
        raise RuntimeError(
            f"[nav2.launch.py] Unknown slam_mode '{slam_mode}'. "
            "Valid values: map, localize, amcl."
        )
    if slam_mode in ('localize', 'amcl') and not map_name:
        raise RuntimeError(
            f"[nav2.launch.py] slam_mode:={slam_mode} requires map_name to be set. "
            "e.g. map_name:=livingroom1"
        )

    pkg_nav = FindPackageShare('lekiwi_navigation').perform(context)

    if slam_mode in ('map', 'localize'):
        # ── slam_toolbox ────────────────────────────────────────────────────
        slam_params = PathJoinSubstitution([
            FindPackageShare('lekiwi_navigation'), 'config', 'nav2', 'slam_toolbox.yaml',
        ])

        _st_mode = {'map': 'mapping', 'localize': 'localization'}
        extra_params = {
            'use_sim_time': use_sim_time,
            'mode': _st_mode[slam_mode],
            # Prevent slam_toolbox from self-activating; let lifecycle_manager_slam
            # control configure/activate so lifecycle transitions are tracked.
            'use_lifecycle_manager': True,
        }

        if slam_mode == 'localize':
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

        actions = [
            LifecycleNode(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                namespace='',
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

        if slam_mode in ('map', 'localize'):
            # map_saver_node listens on /joy and saves the map when R1 (button 5) is pressed.
            actions.append(Node(
                package='lekiwi_navigation',
                executable='map_saver_node',
                name='map_saver_node',
                output='screen',
                parameters=[
                    f'{pkg_nav}/config/nav2/map_saver.yaml',
                    {'use_sim_time': use_sim_time},
                ],
            ))

        return actions

    else:  # amcl
        # ── map_server + AMCL ──────────────────────────────────────────────
        # os.path.realpath resolves the symlink created by --symlink-install so
        # that _pkg_src always points to the source tree where maps/ lives.
        # Maps are not installed — they remain in the source tree only.
        _pkg_src = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        map_yaml = os.path.join(_pkg_src, 'maps', map_name, 'map.yaml')

        return [
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'yaml_filename': map_yaml,
                }],
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[
                    f'{pkg_nav}/config/nav2/amcl.yaml',
                    {'use_sim_time': use_sim_time},
                ],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['map_server', 'amcl'],
                }],
            ),
        ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_mode', default_value='map',
            description="map = build new map; localize = slam_toolbox localization; amcl = AMCL on static map.",
        ),
        DeclareLaunchArgument(
            'map_name', default_value='',
            description=(
                'Subdirectory name of the map to load for localize or amcl mode '
                '(e.g. livingroom1). The full path is constructed automatically.'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock from a sim instead of system time.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
