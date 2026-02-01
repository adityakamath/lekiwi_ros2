#!/usr/bin/env python3
"""
Launch the complete LeKiwi robot control stack.

Starts robot_state_publisher, controller_manager, controllers (joint_state_broadcaster,
base_controller), motor diagnostics, and teleop with timed sequencing for initialization.
"""

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import TimerAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
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
                [FindPackageShare("lekiwi_base_control"), "config", "base_controllers.yaml"]
            ),
        ],
        arguments=["--ros-args", "-r", "/diagnostics:=/controller_manager/diagnostics"],
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

    # Motor diagnostics node - starts after controller_manager is ready
    motor_diagnostics_node = Node(
        package="lekiwi_base_control",
        executable="motor_diagnostics",
        output="log",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("lekiwi_base_control"), "config", "motor_diagnostics_config.yaml"]
            ),
        ],
        remappings=[
            ("/diagnostics", "/dynamic_joint_states/diagnostics"),
        ],
    )

    # Delay spawners to ensure controller_manager and hardware are fully initialized
    delayed_joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_base_controller_spawner = TimerAction(
        period=2.5,
        actions=[base_controller_spawner],
    )

    # Delay motor diagnostics node to ensure joint states are being published
    delayed_motor_diagnostics_node = TimerAction(
        period=3.0,
        actions=[motor_diagnostics_node],
    )

    # Include teleop launch file
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lekiwi_base_control'),
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
        delayed_motor_diagnostics_node,
        teleop_launch,
    ]

    return LaunchDescription(declared_arguments + nodes)