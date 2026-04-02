#!/usr/bin/env python3
"""
Launch the complete LeKiwi robot control stack.

Starts robot_state_publisher, controller_manager, controllers (joint_state_broadcaster,
base_controller), motor diagnostics, and teleop with timed sequencing for initialization.
"""

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import GroupAction, TimerAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description with sequenced node startup."""
    
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttySERVO',
            description='Serial port for STS motor communication'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_mock',
            default_value='false',
            description='Use mock/simulation mode (no hardware required)'
        )
    )
    
    # Initialize Arguments
    serial_port = LaunchConfiguration('serial_port')
    use_mock = LaunchConfiguration('use_mock')
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("lekiwi_description"), "urdf", "base.urdf.xacro"]
            ),
            ' ',
            'serial_port:=', serial_port,
            ' ',
            'use_mock:=', use_mock,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Robot state publisher (set to WARN to reduce verbose logging)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[robot_description],
        name="robot_state_publisher",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", "WARN"],
    )

    # Controller manager (set to WARN to reduce verbose logging)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution(
                [FindPackageShare("lekiwi_control"), "config", "base_controllers.yaml"]
            ),
        ],
        arguments=["--ros-args", "--remap", "/diagnostics:=/controller_manager/diagnostics"],
        output="log",
        emulate_tty=True,
    )

    # Joint state broadcaster spawner - starts 2s after launch to ensure controller_manager is initialized
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="both",
    )

    # Omni wheel drive controller spawner - starts 2.5s after launch
    base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["base_controller", "-c", "/controller_manager"],
        output="both",
    )

    # IMU sensor broadcaster spawner - starts 2.5s after launch alongside base_controller
    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "-c", "/controller_manager"],
        output="both",
    )


    # Motor diagnostics node from sts_hardware_interface - starts after controller_manager is ready
    # Remaps /diagnostics -> /base/diagnostics to separate from IMU diagnostics
    motor_diagnostics_launch = GroupAction([
        SetRemap('/diagnostics', '/base/diagnostics'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('sts_hardware_interface'),
                    'launch',
                    'motor_diagnostics.launch.py'
                ])
            ])
        ),
    ])

    # IMU diagnostics node from bno055_hardware_interface - starts after IMU is initialized
    # Remaps /diagnostics -> /imu/diagnostics to separate from motor diagnostics
    # Not started in mock mode (requires real I2C hardware)
    imu_diagnostics_launch = Node(
        package='bno055_hardware_interface',
        executable='bno055_diagnostics',
        name='bno055_diagnostics',
        output='log',
        parameters=[{
            'i2c_bus':     1,
            'i2c_addr':    '28',
            'sensor_mode': 'NDOF',
        }],
        remappings=[('/diagnostics', '/imu/diagnostics')],
        condition=UnlessCondition(use_mock),
    )

    # Delay spawners to ensure controller_manager and hardware are fully initialized
    delayed_joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_base_controller_spawner = TimerAction(
        period=2.5,
        actions=[base_controller_spawner, imu_broadcaster_spawner],
    )

    # Delay diagnostics launches to ensure joint states are being published
    delayed_motor_diagnostics_launch = TimerAction(
        period=3.0,
        actions=[motor_diagnostics_launch],
    )

    delayed_imu_diagnostics_launch = TimerAction(
        period=3.0,
        actions=[imu_diagnostics_launch],
    )

    # Include teleop launch file
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lekiwi_control'),
                'launch',
                'teleop.launch.py'
            ])
        ])
    )


    nodes = [
        robot_state_publisher_node,
        controller_manager,
        delayed_joint_state_broadcaster_spawner,
        delayed_base_controller_spawner,
        delayed_motor_diagnostics_launch,
        delayed_imu_diagnostics_launch,
        teleop_launch,
    ]

    return LaunchDescription(declared_arguments + nodes)