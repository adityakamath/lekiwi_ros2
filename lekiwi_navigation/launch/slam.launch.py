#!/usr/bin/env python3
"""
Launch localization and SLAM for LeKiwi.

mission controls which backend is used:
    not set + no map_name — slam_toolbox mapping from scratch + map_saver_node
    not set + map_name    — nav2_map_server + AMCL (auto-detected)
    map                   — slam_toolbox mapping from scratch + map_saver_node
    slam + map_name       — slam_toolbox localization on existing map (can extend it)
    amcl + map_name       — nav2_map_server + AMCL (static map, no extension)

Invoked from navigation.launch.py.

Launch arguments:
    mission      map | slam | amcl  (default: auto-detected from map_name)
    map_name     subdirectory name under maps/ (optional; triggers AMCL when mission is not set)
    use_sim_time true | false  (default: false)
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare


def _launch_arg_as_bool(context, name: str) -> bool:
    """Resolve a launch argument as a strict boolean."""
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ('true', '1'):
        return True
    if value in ('false', '0'):
        return False
    raise RuntimeError(
        f"[slam.launch.py] Launch argument '{name}' must be true/false or 1/0, got {value!r}."
    )


def launch_setup(context, *args, **kwargs):
    """Build the slam_toolbox or map_server+AMCL nodes for the selected mission."""
    use_sim_time = _launch_arg_as_bool(context, 'use_sim_time')
    mission      = LaunchConfiguration('mission').perform(context)
    map_name     = LaunchConfiguration('map_name').perform(context)

    if mission not in ('', 'map', 'slam', 'amcl'):
        raise RuntimeError(
            f"[slam.launch.py] Unknown mission '{mission}'. "
            "Valid values: map, slam, amcl."
        )

    # Preserve the current default behavior when mission is not explicitly set.
    if not mission:
        mission = 'amcl' if map_name else 'map'

    if mission in ('slam', 'amcl') and not map_name:
        raise RuntimeError(
            f"[slam.launch.py] mission:={mission} requires map_name to be set."
        )

    # Translate the user-facing mission to the internal backend/mode.
    if mission == 'map':
        slam_mode = 'map'
    elif mission == 'slam':
        slam_mode = 'localize'
    else:
        slam_mode = 'amcl'

    pkg_nav = FindPackageShare('lekiwi_navigation').perform(context)

    if slam_mode in ('map', 'localize'):
        # ── slam_toolbox ────────────────────────────────────────────────────
        slam_params = PathJoinSubstitution([
            FindPackageShare('lekiwi_navigation'), 'config', 'nav2', 'slam_toolbox.yaml',
        ])

        # Set here rather than in slam_toolbox.yaml since all three depend on
        # slam_mode/map_name, only known at launch time.
        _st_mode = {'map': 'mapping', 'localize': 'localization'}
        extra_params = {
            'use_sim_time': use_sim_time,
            'mode': _st_mode[slam_mode],
            'use_lifecycle_manager': True,
        }

        if slam_mode == 'localize':
            # realpath resolves the --symlink-install symlink to the source tree, so this
            # always points at the real maps/ directory rather than the install/build tree.
            _pkg_src = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
            map_dir = os.path.join(_pkg_src, 'maps', map_name)

            # Find the posegraph file - the stem may not be 'map'.
            posegraph_files = [
                f for f in os.listdir(map_dir) if f.endswith('.posegraph')
            ] if os.path.isdir(map_dir) else []
            if not posegraph_files:
                raise RuntimeError(
                    f"[slam.launch.py] No .posegraph file found in maps/{map_name}/ "
                    f"for slam_toolbox localization. "
                    "Run mission:=map first to build and save the map."
                )
            map_stem = os.path.splitext(posegraph_files[0])[0]
            map_file_path = os.path.join(map_dir, map_stem)
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

        # map_saver_node exposes /save_map (std_srvs/srv/SetBool).
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
        # realpath resolves the --symlink-install symlink to the source tree, so this
        # always points at the real maps/ directory rather than the install/build tree.
        _pkg_src = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        map_dir = os.path.join(_pkg_src, 'maps', map_name)

        # Find the map yaml file - the stem may not be 'map'.
        map_yaml_files = [
            f for f in os.listdir(map_dir)
            if f.endswith('.yaml') and not f.startswith('starting_pose')
        ] if os.path.isdir(map_dir) else []
        if not map_yaml_files:
            raise RuntimeError(
                f"[slam.launch.py] No map .yaml file found in maps/{map_name}/ for AMCL. "
                "Run mission:=map first to build and save the map."
            )
        map_yaml = os.path.join(map_dir, map_yaml_files[0])

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
    """Declare mission/map_name/use_sim_time and launch via launch_setup."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'mission', default_value='',
            description=(
                "Navigation mission (auto-detected from map_name if not set): "
                "not set + no map_name = slam_toolbox mapping; "
                "not set + map_name = amcl; "
                "map = slam_toolbox mapping; "
                "slam = slam_toolbox localization (requires map_name); "
                "amcl = AMCL on static map (requires map_name)."
            ),
        ),
        DeclareLaunchArgument(
            'map_name', default_value='',
            description=(
                'Subdirectory name of the map to load for amcl or slam_toolbox localization '
                '(e.g. livingroom1). The full path is constructed automatically.'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock from a sim instead of system time.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
