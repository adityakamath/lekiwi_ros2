#!/usr/bin/env python3
"""
Launch complete LeKiwi robot system with navigation stack.

Starts base control (motors, controllers), webcam, laser scanner,
and the navigation stack (EKF sensor fusion). The EKF owns the
odom → base_footprint TF; base_controller TF publishing is disabled.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for complete robot bringup with navigation."""
    declared_arguments = [
        DeclareLaunchArgument(
            'use_mock',
            default_value='false',
            description=(
                'Use mock/simulation mode (no hardware required). '
                'Automatically switches EKF to odom_only mode (ekf_odom.yaml).'
            ),
        ),
        DeclareLaunchArgument(
            'imu_only',
            default_value='false',
            description=(
                'Set to true when the robot is on a test platform (wheels spin but robot is '
                'stationary). Switches EKF to IMU-only mode. Cannot be combined with odom_only.'
            ),
        ),
        DeclareLaunchArgument(
            'odom_only',
            default_value='false',
            description=(
                'Fuse only wheel odometry, no IMU. Useful for debugging odometry in isolation. '
                'Cannot be set to true at the same time as imu_only. '
                'Automatically set to true when use_mock is true.'
            ),
        ),
    ]

    # Get package share directories
    lekiwi_bringup_share = FindPackageShare('lekiwi_bringup')
    lekiwi_control_share = FindPackageShare('lekiwi_control')
    lekiwi_navigation_share = FindPackageShare('lekiwi_navigation')

    # Include base control launch file
    base_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            lekiwi_control_share, '/launch/base.launch.py'
        ]),
        launch_arguments={'use_mock': LaunchConfiguration('use_mock')}.items()
    )

    # Include navigation launch file 
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            lekiwi_navigation_share, '/launch/nav.launch.py'
        ]),
        launch_arguments={
            'use_mock': LaunchConfiguration('use_mock'),
            'imu_only': LaunchConfiguration('imu_only'),
            'odom_only': LaunchConfiguration('odom_only'),
        }.items(),
    )

    # Include laser scanner launch file
    laser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            lekiwi_bringup_share, '/launch/laser.launch.py'
        ])
    )

    # Include webcam launch file
    webcam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            lekiwi_bringup_share, '/launch/webcam.launch.py'
        ])
    )

    return LaunchDescription(declared_arguments + [
        base_control_launch,
        nav_launch,
        laser_launch,
        webcam_launch,
    ])
