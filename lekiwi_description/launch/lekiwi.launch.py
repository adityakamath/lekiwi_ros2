#!/usr/bin/env python3
"""
Launch the LeKiwi robot description.

Starts robot_state_publisher and optionally joint_state_publisher_gui for manual
joint visualization without running the full control stack.

Example usage:
    ros2 launch lekiwi_description lekiwi.launch.py
    ros2 launch lekiwi_description lekiwi.launch.py use_mock:=true gui:=true
    ros2 launch lekiwi_description lekiwi.launch.py serial_port:=/dev/ttyACM0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
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
        DeclareLaunchArgument(
            'i2c_bus',
            default_value='1',
            description='I2C bus number for the BNO055 IMU (/dev/i2c-N)',
        ),
        DeclareLaunchArgument(
            'i2c_addr',
            default_value='28',
            description='I2C address for the BNO055 IMU (decimal: 28=0x28, 29=0x29)',
        ),
        DeclareLaunchArgument(
            'axis_remap',
            default_value='P1',
            description='BNO055 axis remapping (P0-P7, see datasheet Table 3-4)',
        ),
        DeclareLaunchArgument(
            'sensor_mode',
            default_value='NDOF',
            description='BNO055 fusion mode: NDOF, NDOF_FMC_OFF, or IMUPLUS',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='Start joint_state_publisher_gui for manual joint control',
        ),
    ]

    serial_port = LaunchConfiguration('serial_port')
    use_mock = LaunchConfiguration('use_mock')
    i2c_bus = LaunchConfiguration('i2c_bus')
    i2c_addr = LaunchConfiguration('i2c_addr')
    axis_remap = LaunchConfiguration('axis_remap')
    sensor_mode = LaunchConfiguration('sensor_mode')
    gui = LaunchConfiguration('gui')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('lekiwi_description'), 'urdf', 'base.urdf.xacro']),
        ' serial_port:=', serial_port,
        ' use_mock:=', use_mock,
        ' i2c_bus:=', i2c_bus,
        ' i2c_addr:=', i2c_addr,
        ' axis_remap:=', axis_remap,
        ' sensor_mode:=', sensor_mode,
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='log',
        parameters=[robot_description],
        name='robot_state_publisher',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='log',
        condition=IfCondition(gui),
    )

    return LaunchDescription(declared_arguments + [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
    ])
