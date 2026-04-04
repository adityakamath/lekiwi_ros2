#!/usr/bin/env python3
"""
Launch the LeKiwi navigation stack.

Starts robot_localization EKF node that fuses wheel odometry (/base_controller/odom)
with BNO055 IMU data (/imu_sensor_broadcaster/imu) to produce a filtered odometry
estimate on /odometry/filtered and publishes the odom → base_footprint TF.

This launch file is intended to be used alongside lekiwi_control/base.launch.py
with enable_odom_tf:=false so that the EKF owns the odom → base_footprint transform.

Launch arguments:
    use_mock  (default: false) — when true, forces odom_only mode (ekf_odom.yaml) since
        the IMU is unavailable in mock mode. Overrides imu_only.
    imu_only  (default: false) — fuse only IMU (no wheel odometry). Visualization stays
        stationary on a test platform. Cannot be combined with odom_only.
    odom_only (default: false) — fuse only wheel odometry (no IMU). Useful for debugging
        odometry in isolation; heading will drift. Cannot be combined with imu_only.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Resolve args, enforce mutual exclusion, and return the EKF node."""
    use_mock = LaunchConfiguration('use_mock').perform(context)
    imu_only = LaunchConfiguration('imu_only').perform(context)
    odom_only = LaunchConfiguration('odom_only').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)

    # Mock mode has no IMU hardware — force odom-only regardless of other args.
    if use_mock == 'true':
        if imu_only == 'true':
            print(
                '[nav.launch.py] WARNING: imu_only:=true is ignored because '
                'use_mock:=true overrides it — switching to odom_only mode (ekf_odom.yaml).'
            )
        imu_only = 'false'
        odom_only = 'true'

    if imu_only == 'true' and odom_only == 'true':
        raise RuntimeError(
            "imu_only and odom_only cannot both be true. "
            "Set at most one of them to true."
        )

    if imu_only == 'true':
        config_file = 'ekf_imu.yaml'
    elif odom_only == 'true':
        config_file = 'ekf_odom.yaml'
    else:
        config_file = 'ekf.yaml'

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='log',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('lekiwi_navigation'),
                'config',
                config_file,
            ])
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    return [ekf_node]


def generate_launch_description():
    """Generate launch description for LeKiwi navigation stack."""
    declared_arguments = [
        DeclareLaunchArgument(
            'log_level',
            default_value='warn',
            description='Logging level for navigation nodes',
        ),
        DeclareLaunchArgument(
            'use_mock',
            default_value='false',
            description=(
                'When true, forces odom_only mode (ekf_odom.yaml) since the IMU is '
                'unavailable in mock mode. Overrides imu_only.'
            ),
        ),
        DeclareLaunchArgument(
            'imu_only',
            default_value='false',
            description=(
                'When true, use ekf_imu.yaml (IMU-only). Visualization stays stationary '
                'on a test platform. Cannot be combined with odom_only.'
            ),
        ),
        DeclareLaunchArgument(
            'odom_only',
            default_value='false',
            description=(
                'When true, use ekf_odom.yaml (wheel odometry only, no IMU). Useful for '
                'debugging odometry in isolation. Cannot be combined with imu_only.'
            ),
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

