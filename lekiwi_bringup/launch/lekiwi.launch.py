#!/usr/bin/env python3
"""Launch the LeKiwi base and the assemblies selected in hardware.yaml."""


import tempfile

import yaml

from lekiwi_mujoco.hardware_config import load_hardware_config

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_arg_as_bool(context, name: str) -> bool:
    """Resolve a launch argument as a strict boolean."""
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ('true', '1'):
        return True
    if value in ('false', '0'):
        return False
    raise RuntimeError(
        f"[lekiwi.launch.py] Launch argument '{name}' must be true/false or 1/0, got {value!r}."
    )


def launch_setup(context):
    """Validate arguments and include control, navigation and the configured base sensors."""
    hardware_config = LaunchConfiguration('hardware_config').perform(context)
    hardware = load_hardware_config(hardware_config)
    enable_camera       = _launch_arg_as_bool(context, 'enable_camera')
    diagnostics         = _launch_arg_as_bool(context, 'diagnostics')
    use_mock            = LaunchConfiguration('use_mock').perform(context)
    fusion_mode         = LaunchConfiguration('fusion_mode').perform(context)
    map_name            = LaunchConfiguration('map_name').perform(context)
    pointcloud          = _launch_arg_as_bool(context, 'pointcloud')
    mission             = LaunchConfiguration('mission').perform(context)
    wp_loops            = LaunchConfiguration('wp_loops').perform(context)
    use_composition     = LaunchConfiguration('use_composition').perform(context)
    use_sim_time        = LaunchConfiguration('use_sim_time').perform(context)
    sim                 = _launch_arg_as_bool(context, 'sim')
    mujoco_gui          = LaunchConfiguration('mujoco_gui').perform(context)
    joy                 = LaunchConfiguration('joy').perform(context)
    sts_serial_port     = LaunchConfiguration('sts_serial_port').perform(context)
    mujoco_model        = LaunchConfiguration('mujoco_model').perform(context)
    mujoco_scene        = LaunchConfiguration('mujoco_scene').perform(context)

    tof, audio = (hardware[name] for name in ('sparkfun_sensor', 'respeaker'))
    enable_camera = enable_camera and hardware['gemini2']

    if not hardware['imu'] and fusion_mode in ('base', 'imu'):
        raise RuntimeError(
            f"[lekiwi.launch.py] imu: false in the hardware file requires fusion_mode:=odom "
            f"(fusion_mode:={fusion_mode!r} needs the BNO055)."
        )

    if sim:
        # Running in MuJoCo implies sim time and mock hardware - forced here defensively
        # rather than relying solely on control.launch.py's mujoco branch never opening the real port.
        use_sim_time = 'true'
        use_mock = use_mock or 'true'


    pkg_control = FindPackageShare('lekiwi_control').perform(context)
    pkg_nav     = FindPackageShare('lekiwi_navigation').perform(context)

    def include(pkg, path, args=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource([f'{pkg}/{path}']),
            launch_arguments=(args or {}).items(),
        )

    control_args = {
        'enable_camera':    str(enable_camera).lower(),
        'pointcloud':       str(pointcloud).lower(),
        'diagnostics':      str(diagnostics).lower(),
        'use_mock':         use_mock,
        'use_sim_time':     use_sim_time,
        'joy':              joy,
        'sts_serial_port':  sts_serial_port,
        'mujoco_model':     mujoco_model,
        'enable_odom_tf':   'false',  # the navigation EKF publishes odom -> base_footprint
    }
    control_args['hardware_config'] = hardware_config
    if sim:
        control_args['ros2_control_hardware_type'] = 'mujoco'
        control_args['mujoco_headless'] = 'false' if mujoco_gui.lower() in ('true', '1') else 'true'
        control_args['mujoco_scene'] = mujoco_scene
    control = include(pkg_control, 'launch/control.launch.py', control_args)

    if hardware['lidar']:
        nav = include(pkg_nav, 'launch/navigation.launch.py', {
            'fusion_mode':  fusion_mode,
            'mission':      mission,
            'map_name':     map_name,
            'use_sim_time': use_sim_time,
            'diagnostics':  str(diagnostics).lower(),
            'wp_loops':     wp_loops,
            'use_composition': use_composition,
        })
    else:
        # SLAM, Nav2 and the collision monitor need /scan; only the odometry EKF runs.
        nav = include(pkg_nav, 'launch/ekf.launch.py', {
            'fusion_mode':  fusion_mode,
            'use_sim_time': use_sim_time,
        })

    actions = [control, nav]

    if diagnostics:
        pkg_bringup = FindPackageShare('lekiwi_bringup').perform(context)
        analyzer_file = f'{pkg_bringup}/config/diagnostic_aggregator.yaml'
        enabled_analyzers = {
            'motors': not sim,
            'imu': hardware['imu'] and not sim,
            'battery': hardware['battery_monitor'] and not sim,
            'patrol': hardware['lidar'],
        }
        if any(enabled_analyzers.values()):
            if not all(enabled_analyzers.values()):
                with open(analyzer_file) as stream:
                    analyzer_config = yaml.safe_load(stream)
                sections = analyzer_config['analyzers']['ros__parameters']
                for name, enabled in enabled_analyzers.items():
                    if not enabled:
                        sections.pop(name, None)
                with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='lekiwi_diagnostics_', delete=False) as stream:
                    yaml.safe_dump(analyzer_config, stream)
                    analyzer_file = stream.name
            actions.append(Node(
                package='diagnostic_aggregator', executable='aggregator_node', name='analyzers',
                output='log', parameters=[analyzer_file,
                                        {'use_sim_time': use_sim_time.lower() in ('true', '1')}],
            ))

    if sim:
        # laser/audio/battery_monitor/real camera have no simulated equivalent, or are
        # already hosted directly by control.launch.py's mujoco control node instead.
        return actions

    pkg_bringup = FindPackageShare('lekiwi_bringup').perform(context)

    if hardware['lidar']:
        actions.append(include(pkg_bringup, 'launch/laser.launch.py'))

    if tof:
        # Its start-up stalls the shared I2C bus, so wait until the controllers are active.
        actions.append(TimerAction(period=20.0, actions=[include(pkg_bringup, 'launch/tof.launch.py')]))

    if audio:
        pkg_audio = FindPackageShare('lekiwi_audio').perform(context)
        actions.append(include(pkg_audio, 'launch/audio.launch.py'))

    # Starts both battery_monitor_node and battery_events_node (the latter exposes
    # /battery_low, /battery_critical, /battery_full, /charger_connected as SetBool
    # services for indicator_node). battery_params_file is left at its own default
    # (ina260_battery_monitor's own config/battery.yaml) deliberately - only the one
    # LeKiwi-specific override goes here, layered on top via battery_overrides_file, so
    # this never re-duplicates (and re-drifts from) the upstream package's own defaults.
    # Argument names are battery-prefixed, not the generic params_file/overrides_file -
    # nav2.launch.py (included above, as `nav`) declares its own params_file argument in
    # this same launch tree's shared LaunchConfiguration namespace, and a generic name
    # here would silently resolve to nav2's value instead of this package's own default
    # (found the hard way - see battery_monitor.launch.py's own module docstring).
    if hardware['battery_monitor']:
        pkg_ina260 = FindPackageShare('ina260_battery_monitor').perform(context)
        actions.append(include(pkg_ina260, 'launch/battery_monitor.launch.py', {
            'battery_overrides_file': f'{pkg_bringup}/config/battery_overrides.yaml',
        }))

    # Start the fitted Gemini 2 via the local driver launch.
    if hardware['gemini2']:
        camera = include(pkg_bringup, 'launch/gemini2.launch.py', {
            'enable_camera': str(enable_camera).lower(),
            'pointcloud': str(pointcloud).lower(),
        })
        actions.append(camera)

    return actions


def generate_launch_description():
    """Declare all top-level LeKiwi launch arguments and launch via launch_setup."""
    declared_arguments = [
        DeclareLaunchArgument('hardware_config',
                              default_value=PathJoinSubstitution([FindPackageShare('lekiwi_bringup'), 'config', 'hardware.yaml']),
                              description='Optional robot assemblies.'),

        DeclareLaunchArgument('enable_camera', default_value='true',
                              description='Start the selected real or simulated camera; false retains geometry.'),
        DeclareLaunchArgument(
            'diagnostics',
            default_value='false',
            description=(
                'Single flag for every /diagnostics publisher and consumer: launches motor '
                'and IMU diagnostics nodes in lekiwi_control, enables patrol status publishing '
                'in waypoint_recorder_node (lekiwi_navigation), and launches a '
                'diagnostic_aggregator node (see lekiwi_bringup/config/diagnostic_aggregator.yaml) '
                'that rolls all three up into one top-level status on /diagnostics_agg. '
                'false disables publishing from all of them, not just the aggregator.'
            ),
        ),
        DeclareLaunchArgument(
            'use_mock',
            default_value='',
            description='[advanced/debug] Mock mode override (true/false); empty means use urdf_config.yaml value.',
        ),
        DeclareLaunchArgument(
            'joy',
            default_value='false',
            description='Launch joy_node on this device. Set true when the joystick is connected locally; '
                        'leave false when /joy is published from a remote device.',
        ),
        DeclareLaunchArgument(
            'sts_serial_port',
            default_value='',
            description='[advanced/debug] Serial port override; empty string means use urdf_config.yaml value.',
        ),
        DeclareLaunchArgument(
            'mujoco_model',
            default_value='',
            description='[advanced, sim only] Path to a pre-built MJCF file to load; empty means '
                        'xacro-process it at launch time instead (base model).',
        ),
        DeclareLaunchArgument(
            'fusion_mode',
            default_value='base',
            description=(
                '[advanced] EKF sensor fusion mode: '
                'base = wheel odom + BNO055 (default); '
                'imu = BNO055 only (test platform); '
                'odom = wheel odom only (debug).'
            ),
        ),
        DeclareLaunchArgument(
            'pointcloud',
            default_value='false',
            description='Publish the Gemini 2 colored point cloud; the real camera also publishes the Cloudini-compressed cloud.',
        ),
        DeclareLaunchArgument(
            'mission',
            default_value='',
            description=(
                'Navigation mission: empty preserves current default behavior '
                '(no map_name = map, map_name set = amcl); '
                'map = slam_toolbox mapping; '
                'slam = slam_toolbox localization on an existing map; '
                'amcl = AMCL on a static map.'
            ),
        ),
        DeclareLaunchArgument(
            'map_name',
            default_value='',
            description=(
                'Subdirectory name of the map to load for slam or amcl missions '
                '(e.g. livingroom1). Required for slam and amcl.'
            ),
        ),
        DeclareLaunchArgument(
            'use_composition',
            default_value='true',
            description='Load the Nav2 nodes into one container instead of one process each '
                        '(same names, parameters and remappings).',
        ),
        DeclareLaunchArgument(
            'wp_loops',
            default_value='0',
            description=(
                "Overrides lekiwi_navigation/config/nav2/waypoint_recorder.yaml's "
                'number_of_loops (total patrol passes per start): 0 = loop forever (default), '
                'N>0 = exactly N passes.'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock from a simulator instead of system time (forwarded to control & navigation).',
        ),
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Run against MuJoCo instead of real hardware: forces use_sim_time and '
                        'use_mock, and skips laser/tof/audio/battery_monitor/real camera (laser is hosted '
                        'by the mujoco control node itself; the other three have no simulated '
                        'equivalent at all). The base model is generated from the hardware profile.',
        ),
        DeclareLaunchArgument(
            'mujoco_scene',
            default_value='flat',
            description='[sim only] World for the simulated robot: flat, arena, home (apartment with pushable '
                        'obstacles), maze, none, or a scene MJCF path. Ignored unless sim:=true.',
        ),
        DeclareLaunchArgument(
            'mujoco_gui',
            default_value='false',
            description='[sim only] Launch with the MuJoCo Simulate viewer attached.',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
