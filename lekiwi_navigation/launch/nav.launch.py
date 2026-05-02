#!/usr/bin/env python3
"""
Launch the LeKiwi navigation stack.

Starts a sensor fusion node that produces a filtered odometry estimate on
/odometry/filtered and publishes the odom → base_footprint TF.

This launch file is intended to be used alongside lekiwi_control/lekiwi.launch.py
with enable_odom_tf:=false so that the fusion node owns the odom → base_footprint
transform.

Launch arguments:
    fusion_mode  (default: base)
        base — fuse wheel odometry (vx/vy) + BNO055 orientation + angular rates (ekf.yaml)
        imu  — fuse BNO055 only; no wheel odometry. Use on a test platform where wheels
               spin freely without moving the robot (ekf_imu.yaml).
        odom — fuse wheel odometry only; no IMU. Heading will drift. Use in mock mode or
               for isolated odometry debugging (ekf_odom.yaml).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_EKF_CONFIGS = {
    'base': 'ekf.yaml',
    'imu':  'ekf_imu.yaml',
    'odom': 'ekf_odom.yaml',
}

def launch_setup(context, *args, **kwargs):
    """Resolve fusion_mode and return the EKF node."""
    from launch.substitutions import LaunchConfiguration
    fusion_mode = LaunchConfiguration('fusion_mode').perform(context)

    config_file = _EKF_CONFIGS.get(fusion_mode)
    if config_file is None:
        raise RuntimeError(
            f"[nav.launch.py] Unknown fusion_mode '{fusion_mode}'. "
            f"Valid values: {list(_EKF_CONFIGS)}"
        )

    return [Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='log',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('lekiwi_navigation'),
                'config',
                'robot_localization',
                config_file,
            ])
        ],
        arguments=['--ros-args', '--log-level', 'rclcpp:=ERROR'],
    )]


def generate_launch_description():
    """Generate launch description for LeKiwi navigation stack."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'fusion_mode',
            default_value='base',
            description=(
                'EKF sensor fusion mode: '
                'base = wheel odom + BNO055 (ekf.yaml); '
                'imu = BNO055 only, no wheel odom (ekf_imu.yaml); '
                'odom = wheel odom only, no IMU (ekf_odom.yaml).'
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
