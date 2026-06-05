#!/usr/bin/env python3
"""
Launch the LeKiwi robot system.

The 'config' argument selects which subsystems to bring up:
  base     — base drive + IMU, laser, navigation [no pantilt]
  pantilt  — pan-tilt servos + OAK-D camera [no base, no navigation]
  k2       — full K2 (LeKiwi2) system: base + pantilt + all sensors + navigation [default]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    """
    Build and return the list of launch actions for the selected bringup config.

    Called at launch time by OpaqueFunction so all argument values are resolved
    before deciding which subsystems to include.
    """
    config              = LaunchConfiguration('config').perform(context)
    control_diagnostics = LaunchConfiguration('control_diagnostics').perform(context)
    control_mock        = LaunchConfiguration('control_mock').perform(context)
    fusion_mode         = LaunchConfiguration('fusion_mode').perform(context)
    joy                 = LaunchConfiguration('joy').perform(context)
    map_name            = LaunchConfiguration('map_name').perform(context)
    pointcloud          = LaunchConfiguration('pointcloud').perform(context)
    slam_mode           = LaunchConfiguration('slam_mode').perform(context)
    use_sim_time        = LaunchConfiguration('use_sim_time').perform(context)

    pkg_bringup = FindPackageShare('lekiwi_bringup').perform(context)
    pkg_control = FindPackageShare('lekiwi_control').perform(context)
    pkg_nav     = FindPackageShare('lekiwi_navigation').perform(context)

    def include(pkg, path, args=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource([f'{pkg}/{path}']),
            launch_arguments=(args or {}).items(),
        )

    # Control stack — config is forwarded directly to lekiwi_control's launch file
    control = include(pkg_control, 'launch/control.launch.py', {
        'config':           config,
        'diagnostics':      control_diagnostics,
        'use_mock':         control_mock,
        'joy':              joy,
        'use_sim_time':     use_sim_time,
    })

    nav    = include(pkg_nav,     'launch/navigation.launch.py', {
        'fusion_mode':  fusion_mode,
        'slam_mode':    slam_mode,
        'map_name':     map_name,
        'use_sim_time': use_sim_time,
    })
    laser  = include(pkg_bringup, 'launch/laser.launch.py', {'config': config})
    oakd   = include(pkg_bringup, 'launch/oakd.launch.py', {'pointcloud': pointcloud})

    # ── Config-specific subsystem selection ──────────────────────────────────

    if config == 'base':
        # Base drive + navigation; no pantilt hardware, no OAK-D
        return [control, nav, laser]

    elif config == 'pantilt':
        # Pan-tilt + OAK-D only; no base drive, no navigation, no 2D sensors
        return [control, oakd]

    else:  # k2 — full system
        return [control, nav, laser, oakd]


def generate_launch_description():
    """
    Declare launch arguments and hand off to launch_setup via OpaqueFunction.

    The 'config' argument mirrors the one in lekiwi_control/launch/lekiwi.launch.py
    and gates which sensors and navigation stack are brought up alongside it.
    """
    declared_arguments = [
        DeclareLaunchArgument(
            'config',
            default_value='k2',
            description='Bringup configuration: base, pantilt, or k2',
        ),
        DeclareLaunchArgument(
            'control_diagnostics',
            default_value='false',
            description='Launch motor and IMU diagnostics nodes in lekiwi_control',
        ),
        DeclareLaunchArgument(
            'control_mock',
            default_value='',
            description='Mock mode override (true/false) forwarded to lekiwi_control; empty means use urdf_config.yaml value',
        ),
        DeclareLaunchArgument(
            'fusion_mode',
            default_value='base',
            description=(
                'EKF sensor fusion mode (passed to navigation.launch.py): '
                'base = wheel odom + BNO055; '
                'imu = BNO055 only (test platform); '
                'odom = wheel odom only (mock / debug).'
            ),
        ),
        DeclareLaunchArgument(
            'pointcloud',
            default_value='false',
            description='Enable RGBD point cloud output from OAK-D (uses oakd_vio_pcl.yaml)',
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
                'Subdirectory name of the map to load for localize or amcl modes '
                '(e.g. livingroom1). Required for localize and amcl.'
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
