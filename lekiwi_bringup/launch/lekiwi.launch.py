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


def launch_setup(context):
    payload               = LaunchConfiguration('payload').perform(context)
    pantilt_config      = LaunchConfiguration('pantilt_config').perform(context)
    control_diagnostics = LaunchConfiguration('control_diagnostics').perform(context)
    control_mock        = LaunchConfiguration('control_mock').perform(context)
    fusion_mode         = LaunchConfiguration('fusion_mode').perform(context)
    joy                 = LaunchConfiguration('joy').perform(context)
    map_name            = LaunchConfiguration('map_name').perform(context)
    pointcloud          = LaunchConfiguration('pointcloud').perform(context)
    octomap             = LaunchConfiguration('octomap').perform(context)
    slam_mode           = LaunchConfiguration('slam_mode').perform(context)
    use_sim_time        = LaunchConfiguration('use_sim_time').perform(context)

    pkg_bringup = FindPackageShare('lekiwi_bringup').perform(context)
    pkg_control = FindPackageShare('lekiwi_control').perform(context)
    pkg_nav     = FindPackageShare('lekiwi_navigation').perform(context)
    pkg_pantilt = FindPackageShare('pt_bringup').perform(context)

    def include(pkg, path, args=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource([f'{pkg}/{path}']),
            launch_arguments=(args or {}).items(),
        )

    control = include(pkg_control, 'launch/control.launch.py', {
        'payload':        payload,
        'pantilt_config': pantilt_config,
        'diagnostics':  control_diagnostics,
        'use_mock':     control_mock,
        'joy':          joy,
        'use_sim_time': use_sim_time,
    })

    nav   = include(pkg_nav,     'launch/navigation.launch.py', {
        'fusion_mode':  fusion_mode,
        'slam_mode':    slam_mode,
        'map_name':     map_name,
        'use_sim_time': use_sim_time,
    })
    laser = include(pkg_bringup, 'launch/laser.launch.py', {'payload': payload})
    oakd = include(pkg_pantilt, 'launch/oakd.launch.py', {
        'pointcloud': pointcloud,
        'octomap':    octomap,
    })

    actions = [control, nav, laser]
    if payload == 'pantilt':
        actions.append(oakd)
    return actions


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'payload',
            default_value='pantilt',
            description='Hardware payload: "" for base only, "pantilt" for base + pan-tilt',
        ),
        DeclareLaunchArgument(
            'pantilt_config',
            default_value='pt100',
            description='Pan-tilt mesh variant when payload:=pantilt: "pt100" or "pt101"',
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
            description='Enable RGBD point cloud output from OAK-D (uses oakd_vio_pcl.yaml).',
        ),
        DeclareLaunchArgument(
            'octomap',
            default_value='false',
            description='Run octomap_server on /oak/rgbd/points to build a persistent 3D '
                        'octree. Only takes effect when pointcloud:=true. Forwarded to '
                        'pt_bringup/launch/oakd.launch.py.',
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
