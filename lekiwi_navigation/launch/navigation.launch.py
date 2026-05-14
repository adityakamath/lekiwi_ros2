#!/usr/bin/env python3
"""
Launch the LeKiwi navigation stack.

Includes ekf.launch.py (robot_localization EKF) and the selected navigation
mode via nav2.launch.py (slam_toolbox) or inline AMCL nodes.

Launch arguments:
    fusion_mode  (default: base)
        base — fuse wheel odometry (vx/vy) + BNO055 orientation + angular rates (ekf.yaml)
        imu  — fuse BNO055 only; no wheel odometry (ekf_imu.yaml).
        odom — fuse wheel odometry only; no IMU (ekf_odom.yaml).

    slam_mode    (default: map)
        map      — build a new map with slam_toolbox + map_saver_node (joystick R1 to save).
        localize — localize in an existing map with slam_toolbox (no map saving).
        amcl     — localize in an existing map with AMCL + nav2_map_server (no map saving).

    map_name     (default: '')
        Subdirectory name under lekiwi_navigation/maps/ for localize or amcl modes.
        e.g. map_name:=livingroom1  Required for localize and amcl.

    use_sim_time (default: false)
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    fusion_mode      = LaunchConfiguration('fusion_mode').perform(context)
    slam_mode        = LaunchConfiguration('slam_mode').perform(context)
    map_name         = LaunchConfiguration('map_name').perform(context)
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time     = use_sim_time_str.lower() in ('true', '1')

    if slam_mode not in ('map', 'localize', 'amcl'):
        raise RuntimeError(
            f"[navigation.launch.py] Unknown slam_mode '{slam_mode}'. "
            "Valid values: map, localize, amcl."
        )
    if slam_mode in ('localize', 'amcl') and not map_name:
        raise RuntimeError(
            f"[navigation.launch.py] slam_mode:={slam_mode} requires map_name to be set. "
            "e.g. map_name:=livingroom1"
        )

    pkg_nav = FindPackageShare('lekiwi_navigation')

    # ── EKF ──────────────────────────────────────────────────────────────────
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_nav, 'launch', 'ekf.launch.py']),
        ]),
        launch_arguments={
            'fusion_mode':  fusion_mode,
            'use_sim_time': use_sim_time_str,
        }.items(),
    )

    actions = [ekf]

    if slam_mode in ('map', 'localize'):
        # ── slam_toolbox ─────────────────────────────────────────────────────
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_nav, 'launch', 'nav2.launch.py']),
            ]),
            launch_arguments={
                'slam_mode':    slam_mode,
                'map_name':     map_name,
                'use_sim_time': use_sim_time_str,
            }.items(),
        ))

        if slam_mode == 'map':
            # map_saver_node — joystick R1 saves the map
            actions.append(Node(
                package='lekiwi_navigation',
                executable='map_saver_node',
                name='map_saver_node',
                output='screen',
                parameters=[
                    PathJoinSubstitution([pkg_nav, 'config', 'nav2', 'map_saver_config.yaml']),
                    {'use_sim_time': use_sim_time},
                ],
            ))

    else:
        # ── AMCL + map_server ────────────────────────────────────────────────
        # os.path.realpath resolves the symlink created by --symlink-install so
        # that _pkg_src always points to the source tree where maps/ lives.
        # Maps are not installed — they remain in the source tree only.
        _pkg_src = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        map_yaml = os.path.join(_pkg_src, 'maps', map_name, 'map.yaml')

        actions.append(Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
            }],
        ))
        actions.append(Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                PathJoinSubstitution([pkg_nav, 'config', 'nav2', 'amcl.yaml']),
                {'use_sim_time': use_sim_time},
            ],
        ))
        actions.append(Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'fusion_mode',
            default_value='base',
            description=(
                'EKF sensor fusion mode: '
                'base = wheel odom + BNO055 (ekf.yaml); '
                'imu = BNO055 only (ekf_imu.yaml); '
                'odom = wheel odom only (ekf_odom.yaml).'
            ),
        ),
        DeclareLaunchArgument(
            'slam_mode',
            default_value='map',
            description=(
                'Navigation mode: '
                'map = build new map with slam_toolbox + map_saver_node; '
                'localize = localize with slam_toolbox (no saving); '
                'amcl = localize with AMCL + nav2_map_server (no saving).'
            ),
        ),
        DeclareLaunchArgument(
            'map_name',
            default_value='',
            description=(
                'Subdirectory name under lekiwi_navigation/maps/ to load '
                '(e.g. livingroom1). Required for localize and amcl modes.'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock from a simulator instead of system time.',
        ),
        OpaqueFunction(function=launch_setup),
    ])


