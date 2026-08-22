#!/usr/bin/env python3
"""Launch LeKiwi's joystick teleop stack: joy_teleop (+ joy_node when running locally). Included
unconditionally from control.launch.py - only joy_node is gated by the 'joy' argument, false by
default when /joy is published remotely instead."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_arg_as_bool(context, name: str) -> bool:
    """Resolve a launch argument as a strict boolean."""
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ('true', '1'):
        return True
    if value in ('false', '0'):
        return False
    raise RuntimeError(
        f"[teleop.launch.py] Launch argument '{name}' must be true/false or 1/0, got {value!r}."
    )


def launch_setup(context):
    """Build joy_teleop (+ joy_node when 'joy' is true) for the selected payload."""
    payload      = LaunchConfiguration('payload').perform(context)
    use_sim_time = _launch_arg_as_bool(context, 'use_sim_time')
    launch_joy   = _launch_arg_as_bool(context, 'joy')

    pkg_ctrl = FindPackageShare('lekiwi_control').perform(context)

    teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[
            f'{pkg_ctrl}/config/base/teleop.yaml',
            *([] if not payload else [f'{pkg_ctrl}/config/payloads/{payload}/teleop.yaml']),
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    actions = [teleop_node]

    if launch_joy:
        actions.append(Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='log',
            parameters=[{'use_sim_time': use_sim_time}],
        ))

    return actions


def generate_launch_description():
    """Declare teleop launch arguments and launch via launch_setup."""
    declared_arguments = [
        DeclareLaunchArgument(
            'payload',
            default_value='',
            description='Hardware payload: "" for base only, "pantilt" for base + pan-tilt',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock from a simulator instead of system time.',
        ),
        DeclareLaunchArgument(
            'joy',
            default_value='false',
            description='Launch joy_node on this device. Set true when the joystick is connected locally; '
                        'leave false when /joy is published from a remote device.',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
