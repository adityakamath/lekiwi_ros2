#!/usr/bin/env python3
"""
Launch the Nav2 navigation nodes for LeKiwi.

Mirrors nav2_bringup/launch/navigation_launch.py but limited to the nodes
LeKiwi actually uses.  Unused nav2_bringup nodes are omitted:
    - smoother_server   (SmacPlanner2D has a built-in path smoother)
    - route_server      (graph-based routing — not needed for free-space nav)
    - docking_server    (no docking hardware)
    - following_server  (no following use-case)

Nodes launched (in lifecycle order):
    controller_server      MPPI Omni @ 10 Hz
    planner_server         SmacPlanner2D (A* + built-in smoother)
    behavior_server        Spin / BackUp / Wait recoveries
    velocity_smoother      rate-limits MPPI output
    collision_monitor      last-resort safety stop
    collision_toggle_node               R1 deadman disable for FootprintApproach (plain node)
    waypoint_follower                   patrol loop
    waypoint_recorder_node              record/follow/reset waypoint services (plain node)
    bt_navigator                        Behavior Tree orchestrator (started last)
    keepout/speed_filter_mask_server    no-go/speed zone masks
    keepout/speed_costmap_filter_info_server  (each has its own lifecycle manager)

Topic wiring (no namespace):
    controller_server  cmd_vel   → cmd_vel_raw      (MPPI output)
    behavior_server    cmd_vel   → cmd_vel_raw      (BackUp / Spin output)
    velocity_smoother  cmd_vel   → cmd_vel_raw      (input remap; output is cmd_vel_smoothed)
    twist_switch_node  switched input is cmd_vel_smoothed / cmd_vel_teleop, output is
                        cmd_vel_presafety (lekiwi_control's twist_switch.yaml)
    collision_monitor  reads cmd_vel_presafety, writes base_controller/cmd_vel
                        (positioned downstream of twist_switch_node, not Nav2's stock
                        position)

Launch arguments:
    params_file   full path to nav2.yaml
                  (default: lekiwi_navigation/config/nav2/nav2.yaml)
    map_name      subdirectory under maps/ to load filter masks from
                  (default: '' - no filters active, e.g. while actively mapping)
    use_sim_time  true | false  (default: false)
    autostart     true | false  (default: true)
    log_level     info | debug | warn | error  (default: info)

Invoked from navigation.launch.py.
"""

import os
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Build and return the Nav2 nodes, resolving map_name to filter mask paths."""
    pkg_nav = FindPackageShare('lekiwi_navigation').perform(context)
    map_name = LaunchConfiguration('map_name').perform(context)

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart    = LaunchConfiguration('autostart')
    log_level    = LaunchConfiguration('log_level')

    # bt_navigator must be last - it depends on the other servers being active. Costmap
    # filters get their own lifecycle managers below.
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'waypoint_follower',
        'bt_navigator',
    ]
    keepout_lifecycle_nodes = ['keepout_filter_mask_server', 'keepout_costmap_filter_info_server']
    speed_lifecycle_nodes = ['speed_filter_mask_server', 'speed_costmap_filter_info_server']

    # Standard Nav2 TF remappings — required when running without a namespace.
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # ParameterFile makes the yaml available to every node without duplicating
    # it.  allow_substs=True enables $(find ...) style substitutions inside the
    # yaml if ever needed.
    configured_params = ParameterFile(params_file, allow_substs=True)

    # realpath resolves the --symlink-install symlink to the source tree, so this always
    # points at the real maps/ directory rather than the install/build tree.
    _pkg_src = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

    # yaml_filename must be a real, existing path or ''.
    log_messages = []
    if map_name:
        filters_dir = os.path.join(_pkg_src, 'maps', map_name, 'filters')
        keepout_yaml = os.path.join(filters_dir, 'keepout_mask.yaml')
        speed_yaml = os.path.join(filters_dir, 'speed_mask.yaml')
        keepout_mask_path = keepout_yaml if os.path.exists(keepout_yaml) else ''
        speed_mask_path = speed_yaml if os.path.exists(speed_yaml) else ''
        if not keepout_mask_path or not speed_mask_path:
            log_messages.append(LogInfo(msg=(
                f"[nav2.launch.py] map_name='{map_name}' has no filters/ (or it's "
                'incomplete) - keepout/speed zones disabled for this map. '
                'map_saver_node creates this automatically for maps saved going forward; '
                f'older maps may need a one-time migration - check {filters_dir} if this '
                'is unexpected.'
            )))
    else:
        keepout_mask_path = ''
        speed_mask_path = ''

    # When filter masks are available, enable the filter plugins via an ephemeral param
    # overlay. The plugins default to enabled: false in nav2.yaml so no override is
    # needed (and no separate file is needed) when running without a map.
    filter_enable_params = []
    if keepout_mask_path:
        _filter_dict = {
            'local_costmap': {'local_costmap': {'ros__parameters': {
                'keepout_filter': {'enabled': True},
                'speed_filter': {'enabled': True},
            }}},
            'global_costmap': {'global_costmap': {'ros__parameters': {
                'keepout_filter': {'enabled': True},
            }}},
        }
        _tf = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
        yaml.dump(_filter_dict, _tf)
        _tf.close()
        filter_enable_params = [ParameterFile(_tf.name)]

    load_nodes = [
        SetParameter('use_sim_time', use_sim_time),
        # ── Controller Server ────────────────────────────────────────────
        # Remaps output cmd_vel → cmd_vel_raw so velocity_smoother can pick
        # it up without a circular dependency on cmd_vel_nav.
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params] + filter_enable_params,
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_raw')],
        ),
        # ── Planner Server ───────────────────────────────────────────────
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params] + filter_enable_params,
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        # ── Behavior Server ──────────────────────────────────────────────
        # BackUp and Spin also publish cmd_vel; remap to cmd_vel_raw so
        # recovery velocities go through the same smoother/safety path.
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_raw')],
        ),
        # ── BT Navigator ─────────────────────────────────────────────────
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        # ── Waypoint Follower ────────────────────────────────────────────
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        # ── Waypoint Recorder ────────────────────────────────────────────
        # Not a lifecycle node.
        Node(
            package='lekiwi_navigation',
            executable='waypoint_recorder_node',
            name='waypoint_recorder_node',
            output='log',
            parameters=[
                PathJoinSubstitution([pkg_nav, 'config', 'nav2', 'waypoint_recorder.yaml']),
            ],
            arguments=['--ros-args', '--log-level', log_level],
        ),
        # ── Velocity Smoother ────────────────────────────────────────────
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_raw')],
        ),
        # ── Collision Monitor ────────────────────────────────────────────
        # Topic wiring set via nav2.yaml's cmd_vel_in/out_topic (downstream of
        # twist_switch_node, not Nav2's stock position).
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        # ── Collision Toggle ─────────────────────────────────────────────
        # Not a lifecycle node.
        Node(
            package='lekiwi_navigation',
            executable='collision_toggle_node',
            name='collision_toggle_node',
            output='log',
            parameters=[
                PathJoinSubstitution([pkg_nav, 'config', 'nav2', 'collision_toggle.yaml']),
            ],
            arguments=['--ros-args', '--log-level', log_level],
        ),
        # ── Costmap Filters (no-go + speed zones) ────────────────────────
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='keepout_filter_mask_server',
            output='screen',
            parameters=[{'yaml_filename': keepout_mask_path, 'topic_name': 'keepout_filter_mask'}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='keepout_costmap_filter_info_server',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='speed_filter_mask_server',
            output='screen',
            parameters=[{'yaml_filename': speed_mask_path, 'topic_name': 'speed_filter_mask'}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='speed_costmap_filter_info_server',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        # ── Lifecycle Managers ───────────────────────────────────────────
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                configured_params,
                {'autostart': autostart},
                {'node_names': lifecycle_nodes},
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_keepout_zone',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                {'autostart': autostart},
                {'node_names': keepout_lifecycle_nodes},
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_speed_zone',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                {'autostart': autostart},
                {'node_names': speed_lifecycle_nodes},
            ],
        ),
    ]

    return log_messages + [GroupAction(actions=load_nodes)]


def generate_launch_description():
    """Declare Nav2 launch arguments and launch via launch_setup."""
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('lekiwi_navigation'), 'config', 'nav2', 'nav2.yaml',
            ]),
            description=(
                'Full path to the Nav2 parameters YAML file. '
                'Defaults to lekiwi_navigation/config/nav2/nav2.yaml.'
            ),
        ),
        DeclareLaunchArgument(
            'map_name',
            default_value='',
            description=(
                'Subdirectory under lekiwi_navigation/maps/ to load keepout/speed filter '
                'masks from (its filters/ subfolder). Empty disables both filters - the '
                'normal case while actively mapping, since there is no saved map yet.'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock from a simulator instead of system time.',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically activate the Nav2 lifecycle nodes on startup.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for all Nav2 nodes (info | debug | warn | error).',
        ),
        OpaqueFunction(function=launch_setup),
    ])
