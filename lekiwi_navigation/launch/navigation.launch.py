#!/usr/bin/env python3
"""
Launch the LeKiwi navigation stack.

Main entry point for the lekiwi_navigation package.

Declares shared navigation arguments and forwards them to ekf.launch.py,
slam.launch.py, and nav2.launch.py.

    ekf.launch.py   — robot_localization EKF (wheel odom + IMU → /odometry/filtered)
    slam.launch.py  — slam_toolbox (mapping or localization) or AMCL + map_server
    nav2.launch.py  — Nav2 nodes (controller, planner, behavior, bt_navigator,
                       velocity_smoother, collision_monitor)

Launch arguments:
    fusion_mode  base | imu | odom  (default: base)
    mission      map | slam | amcl  (default: auto-detected from map_name;
                   see slam.launch.py for full semantics)
    map_name     subdirectory under maps/
    use_sim_time true | false  (default: false)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Declare navigation arguments and include ekf/slam/nav2 launch files."""
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
            'mission':      LaunchConfiguration('mission'),
            'map_name':     LaunchConfiguration('map_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_nav, 'launch', 'nav2.launch.py']),
        ]),
        launch_arguments={
            'map_name':     LaunchConfiguration('map_name'),
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
            'mission',
            default_value='',
            description=(
                'Navigation mission selector. For exact mode behavior, '
                'map requirements, and validation rules see slam.launch.py.'
            ),
        ),
        DeclareLaunchArgument(
            'map_name',
            default_value='',
            description=(
                'Subdirectory name under lekiwi_navigation/maps/ to load '
                '(e.g. livingroom). Also tells '
                'nav2.launch.py where to load this map\'s keepout/speed filter masks '
                'from (its filters/ subfolder) - empty disables both filters.'
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


