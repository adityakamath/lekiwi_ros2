#!/usr/bin/env python3
"""
Launch the LeKiwi navigation stack.

Starts robot_localization EKF node that fuses wheel odometry (/base_controller/odom)
with BNO055 IMU data (/imu_sensor_broadcaster/imu) to produce a filtered odometry
estimate on /odometry/filtered and publishes the odom → base_footprint TF.

This launch file is intended to be used alongside lekiwi_control/lekiwi.launch.py
with enable_odom_tf:=false so that the EKF owns the odom → base_footprint transform.

Launch arguments:
    ekf_mode  (default: base)
        base — fuse wheel odometry (vx/vy) + BNO055 orientation + angular rates (ekf.yaml)
        imu  — fuse BNO055 only; no wheel odometry. Use on a test platform where wheels
               spin freely without moving the robot (ekf_imu.yaml).
        odom — fuse wheel odometry only; no IMU. Heading will drift. Use in mock mode or
               for isolated odometry debugging (ekf_odom.yaml).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_EKF_CONFIGS = {
    'base': 'ekf.yaml',
    'imu':  'ekf_imu.yaml',
    'odom': 'ekf_odom.yaml',
}

def launch_setup(context, *args, **kwargs):
    """Resolve ekf_mode and return the EKF node."""
    ekf_mode = LaunchConfiguration('ekf_mode').perform(context)

    config_file = _EKF_CONFIGS.get(ekf_mode)
    if config_file is None:
        raise RuntimeError(
            f"[nav.launch.py] Unknown ekf_mode '{ekf_mode}'. "
            f"Valid values: {list(_EKF_CONFIGS)}"
        )

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
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    return [ekf_node]


def generate_launch_description():
    """Generate launch description for LeKiwi navigation stack."""
    declared_arguments = [
        DeclareLaunchArgument(
            'ekf_mode',
            default_value='base',
            description=(
                'EKF sensor fusion mode: '
                'base = wheel odom + BNO055 (ekf.yaml); '
                'imu = BNO055 only, no wheel odom (ekf_imu.yaml); '
                'odom = wheel odom only, no IMU (ekf_odom.yaml).'
            ),
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
