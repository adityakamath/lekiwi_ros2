#!/usr/bin/env python3
# Copyright 2025 LeKiwi Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Launches the full ROS 2 stack for the LeKiwi omnidirectional robot.

This launch file starts:
- Robot State Publisher (URDF from xacro)
- Controller Manager (ros2_control_node)
- Joint State Broadcaster
- Omni Wheel Drive Controller

All configuration is loaded from the package's URDF and YAML files.
"""

import os
import sys
from launch import LaunchDescription, LaunchContext
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.actions import RegisterEventHandler, TimerAction, OpaqueFunction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the launch description for the LeKiwi base control stack."""
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("lekiwi_description"), "urdf", "base.urdf.xacro"]
            ),
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
        remappings=[
            ("/diagnostics", "/controller_manager/diagnostics"),
        ],
        output="log",
        name="controller_manager",
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

    return LaunchDescription(nodes)