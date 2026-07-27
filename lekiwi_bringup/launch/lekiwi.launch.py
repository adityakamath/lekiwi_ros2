#!/usr/bin/env python3
"""
Launch the LeKiwi robot system.

The 'payload' argument selects which hardware payload is present:
  ""        — base drive + IMU + navigation + laser [no pan-tilt, no OAK-D]
  "pantilt" — base + pan-tilt + OAK-D + navigation + laser [default]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_VALID_PAYLOADS = {'', 'pantilt'}


def launch_setup(context):
    """Validate arguments and include control/navigation/laser (+ oakd for pantilt)."""
    payload             = LaunchConfiguration('payload').perform(context)
    pantilt_config      = LaunchConfiguration('pantilt_config').perform(context)
    diagnostics         = LaunchConfiguration('diagnostics').perform(context)
    control_mock        = LaunchConfiguration('control_mock').perform(context)
    fusion_mode         = LaunchConfiguration('fusion_mode').perform(context)
    map_name            = LaunchConfiguration('map_name').perform(context)
    pointcloud          = LaunchConfiguration('pointcloud').perform(context)
    octomap             = LaunchConfiguration('octomap').perform(context)
    mission             = LaunchConfiguration('mission').perform(context)
    use_sim_time        = LaunchConfiguration('use_sim_time').perform(context)
    sim                 = LaunchConfiguration('sim').perform(context).strip().lower() in ('true', '1')
    gui                 = LaunchConfiguration('gui').perform(context)

    if sim:
        # Running in MuJoCo implies sim time; keep any explicit control_mock override,
        # otherwise force it (control.launch.py's mujoco branch never opens the real serial
        # port, but this stays defensive/explicit rather than relying on that alone).
        use_sim_time = 'true'
        control_mock = control_mock or 'true'

    if payload not in _VALID_PAYLOADS:
        raise RuntimeError(
            f"[lekiwi.launch.py] Unknown payload '{payload}'. "
            f"Valid values: {sorted(_VALID_PAYLOADS)}"
        )
    if pointcloud.lower() in ('true', '1') and payload != 'pantilt':
        raise RuntimeError(
            "[lekiwi.launch.py] pointcloud:=true requires payload:=pantilt."
        )
    if octomap.lower() in ('true', '1') and pointcloud.lower() not in ('true', '1'):
        raise RuntimeError(
            "[lekiwi.launch.py] octomap:=true requires pointcloud:=true."
        )

    pkg_control = FindPackageShare('lekiwi_control').perform(context)
    pkg_nav     = FindPackageShare('lekiwi_navigation').perform(context)

    def include(pkg, path, args=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource([f'{pkg}/{path}']),
            launch_arguments=(args or {}).items(),
        )

    control_args = {
        'payload':        payload,
        'pantilt_config': pantilt_config,
        'diagnostics':  diagnostics,
        'use_mock':     control_mock,
        'use_sim_time': use_sim_time,
    }
    if sim:
        control_args['ros2_control_hardware_type'] = 'mujoco'
        control_args['mujoco_headless'] = 'false' if gui.lower() in ('true', '1') else 'true'
    control = include(pkg_control, 'launch/control.launch.py', control_args)

    nav = include(pkg_nav,     'launch/navigation.launch.py', {
        'fusion_mode':  fusion_mode,
        'mission':      mission,
        'map_name':     map_name,
        'use_sim_time': use_sim_time,
        'diagnostics':  diagnostics,
    })

    actions = [control, nav]

    if diagnostics.strip().lower() in ('true', '1'):
        pkg_bringup = FindPackageShare('lekiwi_bringup').perform(context)
        # name='analyzers' is required, not cosmetic - diagnostic_aggregator.yaml's own
        # top-level key is 'analyzers', and ROS 2 associates a YAML params file with a node
        # by matching that key to the node's name.
        actions.append(Node(
            package='diagnostic_aggregator',
            executable='aggregator_node',
            name='analyzers',
            parameters=[
                f'{pkg_bringup}/config/diagnostic_aggregator.yaml',
                {'use_sim_time': use_sim_time.lower() in ('true', '1')},
            ],
        ))

    if sim:
        # laser/audio/oakd are real-sensor drivers with no simulated equivalent (oakd) or
        # already-simulated equivalent hosted directly by control.launch.py's mujoco
        # control node (laser - see base.control.xacro's laser_frame <sensor>, which
        # publishes /scan itself), so none of them are needed here.
        return actions

    pkg_bringup = FindPackageShare('lekiwi_bringup').perform(context)
    laser = include(pkg_bringup, 'launch/laser.launch.py', {'payload': payload})
    actions.append(laser)

    pkg_audio = FindPackageShare('lekiwi_audio').perform(context)
    actions.append(include(pkg_audio, 'launch/audio.launch.py'))

    if payload == 'pantilt':
        pkg_pantilt = FindPackageShare('pt_bringup').perform(context)
        oakd = include(pkg_pantilt, 'launch/oakd.launch.py', {
            'pointcloud': pointcloud,
            'octomap':    octomap,
        })
        actions.append(oakd)

    return actions


def generate_launch_description():
    """Declare all top-level LeKiwi launch arguments and launch via launch_setup."""
    declared_arguments = [
        DeclareLaunchArgument(
            'payload',
            default_value='pantilt',
            description='Hardware payload: "" for base only, "pantilt" for base + pan-tilt',
        ),
        DeclareLaunchArgument(
            'pantilt_config',
            default_value='pt101',
            description='Pan-tilt mesh variant when payload:="pantilt": "pt100" or "pt101"',
        ),
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
            'control_mock',
            default_value='',
            description='[advanced/debug] Mock mode override (true/false); empty means use urdf_config.yaml value.',
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
            description='[pantilt only] Enable RGBD point cloud output from OAK-D.',
        ),
        DeclareLaunchArgument(
            'octomap',
            default_value='false',
            description='[pantilt only, requires pointcloud:=true] Run octomap_server on the OAK-D point cloud.',
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
            'use_sim_time',
            default_value='false',
            description='Use /clock from a simulator instead of system time (forwarded to control & navigation).',
        ),
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Run against MuJoCo instead of real hardware: forces use_sim_time and '
                        'control_mock, and skips laser/audio/oakd (laser is hosted by the mujoco '
                        'control node itself; oakd has no simulated equivalent yet - audio has no '
                        'simulated equivalent at all). Base and pantilt payload both supported.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='[sim only] Launch with the MuJoCo Simulate viewer attached.',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
