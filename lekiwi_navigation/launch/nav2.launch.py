#!/usr/bin/env python3
"""
Launch the Nav2 navigation nodes for LeKiwi.

Mirrors nav2_bringup/launch/navigation_launch.py but limited to the six
nodes LeKiwi actually uses.  Unused nav2_bringup nodes are omitted:
    - smoother_server   (SmacPlanner2D has a built-in path smoother)
    - route_server      (graph-based routing — not needed for free-space nav)
    - waypoint_follower (single-goal navigation only)
    - docking_server    (no docking hardware)
    - following_server  (no following use-case)

Nodes launched (in lifecycle order):
    controller_server      MPPI Omni @ 10 Hz
    planner_server         SmacPlanner2D (A* + built-in smoother)
    behavior_server        Spin / BackUp / Wait recoveries
    velocity_smoother      rate-limits MPPI output
    collision_monitor      last-resort safety stop
    collision_toggle_node  R1 deadman, see collision_toggle_node.md (plain node)
    bt_navigator           Behavior Tree orchestrator (started last)

Topic wiring (no namespace):
    controller_server  cmd_vel   → cmd_vel_raw      (MPPI output)
    behavior_server    cmd_vel   → cmd_vel_raw      (BackUp / Spin output)
    velocity_smoother  cmd_vel   → cmd_vel_raw      (input remap; output is cmd_vel_smoothed)
    twist_switch_node  switched input is cmd_vel_smoothed / cmd_vel_teleop, output is
                        cmd_vel_presafety (lekiwi_control's twist_switch.yaml)
    collision_monitor  reads cmd_vel_presafety, writes base_controller/cmd_vel - see
                        nav2_part2_plan.md (Feature 1) for why it sits downstream of
                        twist_switch_node instead of Nav2's stock position

Launch arguments:
    params_file   full path to nav2.yaml
                  (default: lekiwi_navigation/config/nav2/nav2.yaml)
    use_sim_time  true | false  (default: false)
    autostart     true | false  (default: true)
    log_level     info | debug | warn | error  (default: info)

Invoked from navigation.launch.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_nav = FindPackageShare('lekiwi_navigation')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart    = LaunchConfiguration('autostart')
    log_level    = LaunchConfiguration('log_level')

    # Lifecycle manager brings up nodes in this order; bt_navigator must be last
    # because it depends on the other servers being active.
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
    ]

    # Standard Nav2 TF remappings — required when running without a namespace.
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # ParameterFile makes the yaml available to every node without duplicating
    # it.  allow_substs=True enables $(find ...) style substitutions inside the
    # yaml if ever needed.
    configured_params = ParameterFile(params_file, allow_substs=True)

    load_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            # ── Controller Server ────────────────────────────────────────────
            # Remaps output cmd_vel → cmd_vel_raw so velocity_smoother can pick
            # it up without a circular dependency on cmd_vel_nav.
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_raw')],
            ),
            # ── Planner Server ───────────────────────────────────────────────
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params],
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
            # Topic wiring set via nav2.yaml's cmd_vel_in/out_topic - see nav2_part2_plan.md
            # (Feature 1) for why this sits downstream of twist_switch_node.
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
            # Not a lifecycle node - see collision_toggle_node.md.
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
            # ── Lifecycle Manager ────────────────────────────────────────────
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
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                pkg_nav, 'config', 'nav2', 'nav2.yaml',
            ]),
            description=(
                'Full path to the Nav2 parameters YAML file. '
                'Defaults to lekiwi_navigation/config/nav2/nav2.yaml.'
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
        load_nodes,
    ])
