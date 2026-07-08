#!/usr/bin/env python3
"""
Launch the LeKiwi navigation stack.

Thin wrapper: declares all navigation arguments and includes ekf.launch.py,
slam.launch.py, and nav2.launch.py.

    ekf.launch.py   — robot_localization EKF (wheel odom + IMU → /odometry/filtered)
    slam.launch.py  — slam_toolbox (mapping or localization) or AMCL + map_server
    nav2.launch.py  — Nav2 nodes (controller, planner, behavior, bt_navigator,
                       velocity_smoother, collision_monitor)

Launch arguments:
    fusion_mode  base | imu | odom  (default: base)
    nav_mode     slam | amcl  (default: slam)
                   slam            — slam_toolbox mapping from scratch
                   slam + map_name — slam_toolbox localization on existing map
                   amcl            — AMCL on static map (requires map_name)
    map_name     subdirectory under maps/ (optional; triggers localization when set)
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
            'nav_mode':    LaunchConfiguration('nav_mode'),
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
            'nav_mode',
            default_value='',
            description=(
                'Navigation mode (auto-detected from map_name if not set): '
                'not set + no map_name = slam_toolbox mapping; '
                'not set + map_name = amcl; '
                'slam = slam_toolbox mapping (or localization if map_name is set); '
                'amcl = AMCL on static map (requires map_name; warns and falls back to mapping if not provided).'
            ),
        ),
        DeclareLaunchArgument(
            'map_name',
            default_value='',
            description=(
                'Subdirectory name under lekiwi_navigation/maps/ to load '
                '(e.g. livingroom). Required for localize and amcl modes. Also tells '
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


