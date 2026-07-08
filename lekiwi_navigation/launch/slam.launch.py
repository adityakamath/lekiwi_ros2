#!/usr/bin/env python3
"""
Launch localization and SLAM for LeKiwi.

nav_mode controls which backend is used:
    not set + no map_name — slam_toolbox mapping from scratch + map_saver_node
    not set + map_name    — nav2_map_server + AMCL (auto-detected)
    slam (no map_name)    — slam_toolbox mapping from scratch + map_saver_node
    slam + map_name       — slam_toolbox localization on existing map (can extend it)
    amcl + map_name       — nav2_map_server + AMCL (static map, no extension)
    amcl (no map_name)    — warning, falls back to slam_toolbox mapping

Invoked from navigation.launch.py.

Launch arguments:
    nav_mode     slam | amcl  (default: auto-detected from map_name)
    map_name     subdirectory name under maps/ (optional; triggers localization when set)
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
    """Build the slam_toolbox or map_server+AMCL nodes for the selected nav_mode."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav_mode     = LaunchConfiguration('nav_mode').perform(context)
    map_name     = LaunchConfiguration('map_name').perform(context)

    if nav_mode not in ('', 'slam', 'amcl'):
        raise RuntimeError(
            f"[slam.launch.py] Unknown nav_mode '{nav_mode}'. "
            "Valid values: slam, amcl."
        )

    # Auto-detect when nav_mode is not explicitly set.
    if not nav_mode:
        nav_mode = 'amcl' if map_name else 'slam'

    # amcl without a map → warn and fall back to slam_toolbox mapping.
    if nav_mode == 'amcl' and not map_name:
        import logging
        logging.getLogger('launch').warning(
            "[slam.launch.py] nav_mode:=amcl requires map_name but none was provided. "
            "Falling back to slam_toolbox mapping mode."
        )
        nav_mode = 'slam'

    # Translate user-facing nav_mode to internal slam_toolbox mode.
    # nav_mode==slam + map_name → localize (on existing map, can extend it)
    # nav_mode==slam            → map (mapping from scratch)
    if nav_mode == 'slam':
        slam_mode = 'localize' if map_name else 'map'
    else:
        slam_mode = 'amcl'

    pkg_nav = FindPackageShare('lekiwi_navigation').perform(context)

    if slam_mode in ('map', 'localize'):
        # ── slam_toolbox ────────────────────────────────────────────────────
        slam_params = PathJoinSubstitution([
            FindPackageShare('lekiwi_navigation'), 'config', 'nav2', 'slam_toolbox.yaml',
        ])

        # See slam_toolbox_yaml.md ("Runtime overrides") for why these three are set here
        # instead of in slam_toolbox.yaml.
        _st_mode = {'map': 'mapping', 'localize': 'localization'}
        extra_params = {
            'use_sim_time': use_sim_time,
            'mode': _st_mode[slam_mode],
            'use_lifecycle_manager': True,
        }

        if slam_mode == 'localize':
            # realpath resolves the --symlink-install symlink to the source tree - see
            # map_saver_node.md ("Path resolution").
            _pkg_src = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
            map_file_path = os.path.join(_pkg_src, 'maps', map_name, 'map')
            extra_params['map_file_name'] = map_file_path

            pose_file = os.path.join(_pkg_src, 'maps', map_name, 'starting_pose.yaml')
            if os.path.exists(pose_file):
                with open(pose_file) as f:
                    pose = yaml.safe_load(f)
                extra_params['map_start_pose'] = [pose['x'], pose['y'], pose['theta']]

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

        # map_saver_node exposes /save_map (std_srvs/srv/SetBool) - see map_saver_node.md.
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
        # realpath resolves the --symlink-install symlink to the source tree - see
        # map_saver_node.md ("Path resolution").
        _pkg_src = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        map_yaml = os.path.join(_pkg_src, 'maps', map_name, 'map.yaml')

        amcl_extra_params = {'use_sim_time': use_sim_time}
        pose_file = os.path.join(_pkg_src, 'maps', map_name, 'starting_pose.yaml')
        if os.path.exists(pose_file):
            with open(pose_file) as f:
                pose = yaml.safe_load(f)
            amcl_extra_params.update({
                'set_initial_pose': True,
                'initial_pose.x': float(pose['x']),
                'initial_pose.y': float(pose['y']),
                'initial_pose.yaw': float(pose['theta']),
            })
        else:
            import logging
            logging.getLogger('launch').warning(
                f"[slam.launch.py] No starting_pose.yaml found for map '{map_name}'. "
                "AMCL will not publish map->odom until you set an initial pose "
                "(e.g. via Foxglove's Publish Pose tool on the /initialpose topic)."
            )

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
                    amcl_extra_params,
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
    """Declare nav_mode/map_name/use_sim_time and launch via launch_setup."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'nav_mode', default_value='',
            description=(
                "Navigation mode (auto-detected from map_name if not set): "
                "not set + no map_name = slam_toolbox mapping; "
                "not set + map_name = amcl; "
                "slam = slam_toolbox mapping (or localization if map_name is set); "
                "amcl = AMCL on static map (requires map_name; falls back to mapping with a warning if not provided)."
            ),
        ),
        DeclareLaunchArgument(
            'map_name', default_value='',
            description=(
                'Subdirectory name of the map to load for amcl mode or slam_toolbox localization '
                '(e.g. livingroom1). The full path is constructed automatically.'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock from a sim instead of system time.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
