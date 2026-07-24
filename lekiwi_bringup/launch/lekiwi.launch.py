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
from launch_ros.substitutions import FindPackageShare

_VALID_PAYLOADS = {'', 'pantilt'}


def launch_setup(context):
    """Validate arguments and include control/navigation/laser (+ oakd for pantilt)."""
    payload             = LaunchConfiguration('payload').perform(context)
    pantilt_config      = LaunchConfiguration('pantilt_config').perform(context)
    diagnostics         = LaunchConfiguration('diagnostics').perform(context)
    control_mock        = LaunchConfiguration('control_mock').perform(context)
    fusion_mode         = LaunchConfiguration('fusion_mode').perform(context)
    joy                 = LaunchConfiguration('joy').perform(context)
    map_name            = LaunchConfiguration('map_name').perform(context)
    pointcloud          = LaunchConfiguration('pointcloud').perform(context)
    octomap             = LaunchConfiguration('octomap').perform(context)
    mission             = LaunchConfiguration('mission').perform(context)
    use_sim_time        = LaunchConfiguration('use_sim_time').perform(context)

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

    pkg_bringup = FindPackageShare('lekiwi_bringup').perform(context)
    pkg_control = FindPackageShare('lekiwi_control').perform(context)
    pkg_nav     = FindPackageShare('lekiwi_navigation').perform(context)
    pkg_pantilt = FindPackageShare('pt_bringup').perform(context)
    pkg_audio   = FindPackageShare('lekiwi_audio').perform(context)

    def include(pkg, path, args=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource([f'{pkg}/{path}']),
            launch_arguments=(args or {}).items(),
        )

    control = include(pkg_control, 'launch/control.launch.py', {
        'payload':        payload,
        'pantilt_config': pantilt_config,
        'diagnostics':  diagnostics,
        'use_mock':     control_mock,
        'joy':          joy,
        'use_sim_time': use_sim_time,
    })

    nav = include(pkg_nav,     'launch/navigation.launch.py', {
        'fusion_mode':  fusion_mode,
        'mission':      mission,
        'map_name':     map_name,
        'use_sim_time': use_sim_time,
    })
    laser = include(pkg_bringup, 'launch/laser.launch.py', {'payload': payload})
    audio = include(pkg_audio, 'launch/audio.launch.py')
    oakd = include(pkg_pantilt, 'launch/oakd.launch.py', {
        'pointcloud': pointcloud,
        'octomap':    octomap,
    })

    actions = [control, nav, laser, audio]
    if payload == 'pantilt':
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
            description='Pan-tilt mesh variant when payload:="pt100" or "pt101"',
        ),
        DeclareLaunchArgument(
            'diagnostics',
            default_value='false',
            description='Launch motor and IMU diagnostics nodes in lekiwi_control.',
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
            'joy',
            default_value='false',
            description='Launch joy_node on the robot. Set true when the joystick is connected locally; '
                        'leave false when /joy is published from a remote device.',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
