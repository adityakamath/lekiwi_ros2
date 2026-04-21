#!/usr/bin/env python3
"""
Launch the LeKiwi robot with pantilt enabled (use_pantilt:=true).
Loads lekiwi_control_config.yaml and the base URDF with pantilt.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttySERVO',
            description='Serial port for STS motor communication',
        ),
        DeclareLaunchArgument(
            'use_mock',
            default_value='false',
            description='Use mock/simulation mode (no hardware required)',
        ),
    ]

    serial_port = LaunchConfiguration('serial_port')
    use_mock = LaunchConfiguration('use_mock')

    # Get URDF via xacro with use_pantilt:=true
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('lekiwi_description'), 'urdf', 'base.urdf.xacro']),
        ' serial_port:=', serial_port,
        ' use_mock:=', use_mock,
        ' use_pantilt:=true',
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='log',
        parameters=[robot_description],
        name='robot_state_publisher',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            PathJoinSubstitution([FindPackageShare('lekiwi_control'), 'config', 'lekiwi_control_config.yaml']),
        ],
        remappings=[('/diagnostics', '/controller_manager/diagnostics')],
        output='log',
        emulate_tty=True,
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='both',
    )

    base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['base_controller', '-c', '/controller_manager'],
        output='both',
    )

    pantilt_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pantilt_controller', '-c', '/controller_manager'],
        output='both',
    )

    imu_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster', '-c', '/controller_manager'],
        output='both',
    )

    motor_diagnostics_launch = GroupAction([
        SetRemap('/diagnostics', '/base/diagnostics'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('sts_hardware_interface'), 'launch', 'motor_diagnostics.launch.py'])
            ])
        ),
    ])

    imu_diagnostics_launch = Node(
        package='bno055_hardware_interface',
        executable='bno055_diagnostics',
        name='bno055_diagnostics',
        output='log',
        parameters=[{
            'i2c_bus':          1,
            'i2c_addr':         '28',
            'sensor_mode':      'NDOF',
            'enable_mock_mode': use_mock,
        }],
        remappings=[('/diagnostics', '/imu/diagnostics')],
    )

    delayed_joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_base_controller_spawner = TimerAction(
        period=2.5,
        actions=[base_controller_spawner, pantilt_controller_spawner, imu_broadcaster_spawner],
    )

    delayed_motor_diagnostics_launch = TimerAction(
        period=3.0,
        actions=[motor_diagnostics_launch],
    )

    delayed_imu_diagnostics_launch = TimerAction(
        period=3.0,
        actions=[imu_diagnostics_launch],
    )



    return LaunchDescription(declared_arguments + [
        robot_state_publisher_node,
        controller_manager,
        delayed_joint_state_broadcaster_spawner,
        delayed_base_controller_spawner,
        delayed_motor_diagnostics_launch,
        delayed_imu_diagnostics_launch,
        # teleop_launch,  # Disabled: joy_teleop not installed
    ])
