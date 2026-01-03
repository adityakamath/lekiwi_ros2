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
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
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
                [FindPackageShare("lekiwi_base_control"), "urdf", "base.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Controller manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution(
                [FindPackageShare("lekiwi_base_control"), "config", "base_controllers.yaml"]
            ),
        ],
        output="both",
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # Omni wheel drive controller spawner
    omni_wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_wheel_controller"],
        remappings=[
            ("/omni_wheel_controller/cmd_vel", "/cmd_vel"),
        ],
    )

    nodes = [
        robot_state_publisher_node,
        controller_manager,
        joint_state_broadcaster_spawner,
        omni_wheel_controller_spawner,
    ]

    return LaunchDescription(nodes)
