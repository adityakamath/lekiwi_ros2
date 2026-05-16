#!/usr/bin/env python3
"""
Launch the LeKiwi navigation stack.

Thin wrapper: declares all navigation arguments and includes ekf.launch.py,
slam.launch.py, and nav2.launch.py.

    ekf.launch.py   — robot_localization EKF (wheel odom + IMU → /odometry/filtered)
    slam.launch.py  — slam_toolbox (map / localize) or AMCL + map_server (amcl)
    nav2.launch.py  — Nav2 nodes (controller, planner, behavior, bt_navigator,
                       velocity_smoother, collision_monitor)

Launch arguments:
    fusion_mode  base | imu | odom  (default: base)
    slam_mode    map | localize | amcl  (default: map)
    map_name     subdirectory under maps/ (required for localize/amcl)
    use_sim_time true | false  (default: false)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_nav = FindPackageShare('lekiwi_navigation')

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_nav, 'launch', 'ekf.launch.py']),
        ]),
        launch_arguments={
            'fusion_mode':  LaunchConfiguration('fusion_mode'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_nav, 'launch', 'slam.launch.py']),
        ]),
        launch_arguments={
            'slam_mode':    LaunchConfiguration('slam_mode'),
            'map_name':     LaunchConfiguration('map_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_nav, 'launch', 'nav2.launch.py']),
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'fusion_mode',
            default_value='base',
            description=(
                'EKF sensor fusion mode: '
                'base = wheel odom + BNO055 (ekf.yaml); '
                'imu = BNO055 only (ekf_imu.yaml); '
                'odom = wheel odom only (ekf_odom.yaml).'
            ),
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
                'Subdirectory name under lekiwi_navigation/maps/ to load '
                '(e.g. livingroom). Required for localize and amcl modes.'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock from a simulator instead of system time.',
        ),
        ekf,
        slam,
        nav2,
    ])


