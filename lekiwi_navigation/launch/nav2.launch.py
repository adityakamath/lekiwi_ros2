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
    controller_server      MPPI Omni @ 20 Hz
    planner_server         SmacPlanner2D (A* + built-in smoother)
    behavior_server        Spin / BackUp / Wait recoveries
    velocity_smoother      rate-limits MPPI output
    collision_monitor      last-resort safety stop (R1 deadman toggle for its
                           FootprintApproach polygon lives in lekiwi_control's
                           control_support_node, not here - see collision_toggle_node.py)
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
from lekiwi_navigation.util_velocity_limits import velocity_overrides
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes, Node, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.substitutions import FindPackageShare

_VALID_LOG_LEVELS = {'info', 'debug', 'warn', 'error'}


def _launch_arg_as_bool(context, name: str) -> bool:
    """Resolve a launch argument as a strict boolean."""
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ('true', '1'):
        return True
    if value in ('false', '0'):
        return False
    raise RuntimeError(
        f"[nav2.launch.py] Launch argument '{name}' must be true/false or 1/0, got {value!r}."
    )


def _launch_arg_as_int(context, name: str) -> int:
    """Resolve a launch argument as a non-negative integer."""
    value = LaunchConfiguration(name).perform(context).strip()
    if not value.isdigit():
        raise RuntimeError(
            f"[nav2.launch.py] Launch argument '{name}' must be a non-negative integer, got {value!r}."
        )
    return int(value)


def launch_setup(context, *args, **kwargs):
    """Build and return the Nav2 nodes, resolving map_name to filter mask paths."""
    pkg_nav = FindPackageShare('lekiwi_navigation').perform(context)
    teleop_path = os.path.join(
        FindPackageShare('lekiwi_control').perform(context), 'config', 'base_teleop.yaml')
    speed_limits = velocity_overrides(teleop_path, LaunchConfiguration('params_file').perform(context))
    map_name = LaunchConfiguration('map_name').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    if log_level not in _VALID_LOG_LEVELS:
        raise RuntimeError(
            f"[nav2.launch.py] Unknown log_level {log_level!r}. "
            f"Valid values: {sorted(_VALID_LOG_LEVELS)}"
        )

    params_file  = LaunchConfiguration('params_file')
    use_sim_time = _launch_arg_as_bool(context, 'use_sim_time')
    autostart    = _launch_arg_as_bool(context, 'autostart')
    diagnostics  = _launch_arg_as_bool(context, 'diagnostics')
    wp_loops     = _launch_arg_as_int(context, 'wp_loops')
    use_composition = _launch_arg_as_bool(context, 'use_composition')
    container_name = LaunchConfiguration('container_name').perform(context)

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

    namespace = str(context.launch_configurations.get('namespace', '')).strip('/')
    prefix = f'{namespace}/'.lstrip('/')  # '' for the empty namespace
    ns_path = f'/{namespace}'.rstrip('/')  # '' for the empty namespace

    # No /tf remap: all robots share the one global /tf and tell themselves apart by frame prefix.
    remappings = [('/diagnostics', 'diagnostics')]

    # ParameterFile shares the yaml across every node without duplicating it;
    # allow_substs=True enables $(find ...) substitutions inside it if ever needed.
    configured_params = ParameterFile(params_file, allow_substs=True)

    # Overrides of what nav2.yaml writes unprefixed or absolute: the frames, and the scan and
    # odometry topics that the costmap child nodes cannot resolve relatively.
    base_params = [configured_params]
    frames = {'robot_base_frame': f'{prefix}base_footprint'}
    scan = {'topic': f'{ns_path}/scan'}  # copied per use: YAML aliases are not valid parameter files
    odom_topic = f'{ns_path}/odometry/filtered'
    namespace_overrides = {
        '/**/bt_navigator': {'ros__parameters': {
            **frames, 'global_frame': f'{prefix}map', 'odom_topic': odom_topic}},
        '/**/behavior_server': {'ros__parameters': {
            **frames, 'global_frame': f'{prefix}map', 'local_frame': f'{prefix}odom'}},
        '/**/velocity_smoother': {'ros__parameters': {'odom_topic': odom_topic}},
        '/**/collision_monitor': {'ros__parameters': {
            'base_frame_id': f'{prefix}base_footprint', 'odom_frame_id': f'{prefix}odom',
            'scan': dict(scan)}},
        '/**/local_costmap': {'local_costmap': {'ros__parameters': {
            **frames, 'global_frame': f'{prefix}odom', 'obstacle_layer': {'scan': dict(scan)}}}},
        '/**/global_costmap': {'global_costmap': {'ros__parameters': {
            **frames, 'global_frame': f'{prefix}map', 'obstacle_layer': {'scan': dict(scan)}}}},
    }
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as overrides:
        yaml.dump(namespace_overrides, overrides)
    base_params.append(ParameterFile(overrides.name))

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

    # Enables the filter plugins via an ephemeral param overlay when masks are available;
    # they default to enabled: false in nav2.yaml, so no override/file is needed otherwise.
    filter_enable_params = []
    if keepout_mask_path:
        _filter_dict = {
            '/**/local_costmap': {'local_costmap': {'ros__parameters': {
                'keepout_filter': {'enabled': True},
                'speed_filter': {'enabled': True},
            }}},
            '/**/global_costmap': {'global_costmap': {'ros__parameters': {
                'keepout_filter': {'enabled': True},
            }}},
        }
        _tf = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
        yaml.dump(_filter_dict, _tf)
        _tf.close()
        filter_enable_params = [ParameterFile(_tf.name)]

    _wp_overrides = {'/**/waypoint_recorder_node': {'ros__parameters': {
        'publish_diagnostics': diagnostics,
        'number_of_loops': wp_loops,
    }}}
    _wp_tf = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump(_wp_overrides, _wp_tf)
    _wp_tf.close()
    waypoint_recorder_params = ParameterFile(_wp_tf.name)

    # Every Nav2 node is described once and built either as a process (Node) or as a component
    # loaded into one shared container, so both modes get identical names, parameters and remaps.
    smoother_remaps = remappings + [('cmd_vel', 'cmd_vel_raw')]
    manager_remaps = [('/diagnostics', 'diagnostics')]
    nav2_nodes = [
        dict(package='nav2_controller', executable='controller_server',
             plugin='nav2_controller::ControllerServer', name='controller_server',
             parameters=[*base_params, speed_limits['controller_server']] + filter_enable_params,
             remappings=smoother_remaps),
        dict(package='nav2_planner', executable='planner_server',
             plugin='nav2_planner::PlannerServer', name='planner_server',
             parameters=[*base_params] + filter_enable_params, remappings=remappings),
        # BackUp and Spin also publish cmd_vel; remap to cmd_vel_raw so recovery velocities go
        # through the same smoother/safety path.
        dict(package='nav2_behaviors', executable='behavior_server',
             plugin='behavior_server::BehaviorServer', name='behavior_server',
             parameters=[*base_params, speed_limits['behavior_server']],
             remappings=smoother_remaps),
        dict(package='nav2_bt_navigator', executable='bt_navigator',
             plugin='nav2_bt_navigator::BtNavigator', name='bt_navigator',
             parameters=[*base_params], remappings=remappings),
        dict(package='nav2_waypoint_follower', executable='waypoint_follower',
             plugin='nav2_waypoint_follower::WaypointFollower', name='waypoint_follower',
             parameters=[*base_params], remappings=remappings),
        dict(package='nav2_velocity_smoother', executable='velocity_smoother',
             plugin='nav2_velocity_smoother::VelocitySmoother', name='velocity_smoother',
             parameters=[*base_params, speed_limits['velocity_smoother']],
             remappings=smoother_remaps),
        # Topic wiring comes from nav2.yaml's cmd_vel_in/out_topic (downstream of twist_switch_node).
        dict(package='nav2_collision_monitor', executable='collision_monitor',
             plugin='nav2_collision_monitor::CollisionMonitor', name='collision_monitor',
             parameters=[*base_params], remappings=remappings),
        # Costmap filters (no-go + speed zones).
        dict(package='nav2_map_server', executable='map_server',
             plugin='nav2_map_server::MapServer', name='keepout_filter_mask_server',
             parameters=[{'yaml_filename': keepout_mask_path, 'topic_name': 'keepout_filter_mask'}],
             remappings=remappings),
        dict(package='nav2_map_server', executable='costmap_filter_info_server',
             plugin='nav2_map_server::CostmapFilterInfoServer',
             name='keepout_costmap_filter_info_server',
             parameters=[*base_params], remappings=remappings),
        dict(package='nav2_map_server', executable='map_server',
             plugin='nav2_map_server::MapServer', name='speed_filter_mask_server',
             parameters=[{'yaml_filename': speed_mask_path, 'topic_name': 'speed_filter_mask'}],
             remappings=remappings),
        dict(package='nav2_map_server', executable='costmap_filter_info_server',
             plugin='nav2_map_server::CostmapFilterInfoServer',
             name='speed_costmap_filter_info_server',
             parameters=[*base_params], remappings=remappings),
        dict(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             plugin='nav2_lifecycle_manager::LifecycleManager',
             name='lifecycle_manager_navigation',
             parameters=[*base_params, {'autostart': autostart},
                         {'node_names': lifecycle_nodes}], remappings=manager_remaps),
        dict(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             plugin='nav2_lifecycle_manager::LifecycleManager',
             name='lifecycle_manager_keepout_zone',
             parameters=[{'autostart': autostart}, {'node_names': keepout_lifecycle_nodes}],
             remappings=manager_remaps),
        dict(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             plugin='nav2_lifecycle_manager::LifecycleManager',
             name='lifecycle_manager_speed_zone',
             parameters=[{'autostart': autostart}, {'node_names': speed_lifecycle_nodes}],
             remappings=manager_remaps),
    ]

    # Plain nodes outside Nav2 lifecycle management; never composed.
    plain_nodes = [
        # Not a lifecycle node.
        Node(
            package='lekiwi_navigation',
            executable='waypoint_recorder_node',
            name='waypoint_recorder_node',
            output='log',
            parameters=[
                PathJoinSubstitution([pkg_nav, 'config', 'nav2', 'waypoint_recorder.yaml']),
                waypoint_recorder_params,
                {'frame_id': f'{prefix}map', 'base_frame': f'{prefix}base_footprint'},
            ],
            arguments=['--ros-args', '--log-level', log_level],
        ),
        # Target tracker.
        Node(
            package='lekiwi_navigation',
            executable='nav2_target_node',
            name='nav2_target_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([pkg_nav, 'config', 'nav2', 'nav2_target.yaml']),
                {'use_sim_time': use_sim_time},
                {'map_frame': f'{prefix}map', 'base_frame': f'{prefix}base_footprint',
                 'target_frame': f'{prefix}nav2_target'},
            ],
            arguments=['--ros-args', '--log-level', log_level],
        ),
    ]

    if use_composition:
        # Isolated container: one executor thread per component, so MPPI cannot stall the rest.
        # Intra-process comms stay off, as in Nav2's own composition: with them on, a publisher
        # that checks get_subscription_count() sees none from the same container.
        load_nodes = [
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='rclcpp_components',
                executable='component_container_isolated',
                name=container_name,
                output='screen',
                # The costmaps are child nodes the controller/planner create themselves; they read
                # their settings from the process-wide params file, not from the component's.
                parameters=[*base_params] + filter_enable_params,
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            LoadComposableNodes(
                target_container=f'{ns_path}/{container_name}',
                composable_node_descriptions=[
                    ComposableNode(
                        package=n['package'], plugin=n['plugin'], name=n['name'],
                        parameters=n['parameters'], remappings=n['remappings'],
                    ) for n in nav2_nodes
                ],
            ),
            *plain_nodes,
        ]
    else:
        load_nodes = [
            SetParameter('use_sim_time', use_sim_time),
            *[Node(
                package=n['package'], executable=n['executable'], name=n['name'],
                output='screen', parameters=n['parameters'],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=n['remappings'],
            ) for n in nav2_nodes],
            *plain_nodes,
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
            'diagnostics',
            default_value='false',
            description=(
                "Publish waypoint_recorder_node's patrol status to /diagnostics - the same "
                'top-level flag that gates motor_diagnostics/bno055_diagnostics in '
                'control.launch.py, forwarded here so one argument covers every diagnostics '
                'publisher.'
            ),
        ),
        DeclareLaunchArgument(
            'wp_loops',
            default_value='0',
            description=(
                "Overrides waypoint_recorder.yaml's number_of_loops: total patrol passes per "
                'start. 0 = loop forever (default), N>0 = exactly N passes.'
            ),
        ),
        DeclareLaunchArgument(
            'use_composition',
            default_value='true',
            description=(
                'Load the Nav2 nodes as components of one container (fewer processes) '
                'instead of one process each. Names, parameters and '
                'remappings are identical either way.'
            ),
        ),
        DeclareLaunchArgument(
            'container_name',
            default_value='nav2_container',
            description='Container node name used when use_composition:=true.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for all Nav2 nodes (info | debug | warn | error).',
        ),
        OpaqueFunction(function=launch_setup),
    ])
