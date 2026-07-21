#!/usr/bin/env python3
"""
Launch the robot_localization EKF node for LeKiwi.

Selects the EKF config file based on the fusion_mode argument and starts
ekf_node, which publishes the odom → base_footprint TF and /odometry/filtered.

Launch arguments:
    fusion_mode  (default: base)
        base — fuse wheel odometry (vx/vy) + BNO055 orientation + angular rates (ekf.yaml)
        imu  — fuse BNO055 only; no wheel odometry (ekf_imu.yaml)
        odom — fuse wheel odometry only; no IMU (ekf_odom.yaml)

    use_sim_time (default: false)
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


def _launch_arg_as_bool(context, name: str) -> bool:
    """Resolve a launch argument as a strict boolean."""
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ('true', '1'):
        return True
    if value in ('false', '0'):
        return False
    raise RuntimeError(
        f"[ekf.launch.py] Launch argument '{name}' must be true/false or 1/0, got {value!r}."
    )


def launch_setup(context, *args, **kwargs):
    """Build the ekf_node Node for the selected fusion_mode config file."""
    fusion_mode  = LaunchConfiguration('fusion_mode').perform(context)
    use_sim_time = _launch_arg_as_bool(context, 'use_sim_time')

    config_file = _EKF_CONFIGS.get(fusion_mode)
    if config_file is None:
        raise RuntimeError(
            f"[ekf.launch.py] Unknown fusion_mode '{fusion_mode}'. "
            f"Valid values: {list(_EKF_CONFIGS)}"
        )

    return [
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='log',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('lekiwi_navigation'),
                    'config', 'robot_localization', config_file,
                ]),
                {'use_sim_time': use_sim_time},
            ],
            arguments=['--ros-args', '--log-level', 'rclcpp:=ERROR'],
        ),
    ]


def generate_launch_description():
    """Declare fusion_mode/use_sim_time and launch ekf_node via launch_setup."""
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
            'use_sim_time',
            default_value='false',
            description='Use /clock from a simulator instead of system time.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
