#!/usr/bin/env python3
"""
Launch the LeKiwi robot control stack.

The 'payload' argument selects which hardware is present alongside the base:
  ""        — base drive + IMU only  [default: no payload]
  "pantilt" — base drive + IMU + pan-tilt servos
"""

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _launch_arg_as_bool(context, name: str) -> bool:
    """Resolve a launch argument as a strict boolean."""
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ('true', '1'):
        return True
    if value in ('false', '0'):
        return False
    raise RuntimeError(
        f"[control.launch.py] Launch argument '{name}' must be true/false or 1/0, got {value!r}."
    )

def launch_setup(context):
    """Build the control-stack nodes for the selected payload."""
    payload        = LaunchConfiguration('payload').perform(context)
    pantilt_config = LaunchConfiguration('pantilt_config').perform(context)
    serial_port  = LaunchConfiguration('sts_serial_port').perform(context)
    use_mock     = LaunchConfiguration('use_mock').perform(context)
    diagnostics  = _launch_arg_as_bool(context, 'diagnostics')
    launch_joy   = _launch_arg_as_bool(context, 'joy')
    use_sim_time = _launch_arg_as_bool(context, 'use_sim_time')

    pkg_desc = FindPackageShare('lekiwi_description').perform(context)
    pkg_ctrl = FindPackageShare('lekiwi_control').perform(context)
    xacro    = FindExecutable(name='xacro').perform(context)

    urdf = f'{pkg_desc}/urdf/base_pantilt/base_pantilt.urdf.xacro' if payload == 'pantilt' else f'{pkg_desc}/urdf/base/base.urdf.xacro'

    _cfg = yaml.safe_load(open(f'{pkg_ctrl}/config/base/urdf_config.yaml'))
    if payload:
        _cfg.update(yaml.safe_load(open(f'{pkg_ctrl}/config/payloads/{payload}/urdf_config.yaml')))
    final_serial_port = serial_port if serial_port else _cfg['serial_port']
    final_use_mock    = use_mock if use_mock else str(_cfg['use_mock']).lower()

    xacro_cmd = (
        f'{xacro} {urdf}'
        f' serial_port:={final_serial_port}'
        f' use_mock:={final_use_mock}'
        f' baud_rate:={_cfg["baud_rate"]}'
        f' use_sync_write:={str(_cfg["use_sync_write"]).lower()}'
        f' left_motor_id:={_cfg["left_motor_id"]}'
        f' back_motor_id:={_cfg["back_motor_id"]}'
        f' right_motor_id:={_cfg["right_motor_id"]}'
        f' sts_max_velocity_steps:={_cfg["sts_max_velocity_steps"]}'
        f' proportional_acc_max:={_cfg["proportional_acc_max"]}'
    )
    if payload == 'pantilt':
        xacro_cmd += (
            f' pantilt_config:={pantilt_config}'
            f' pan_motor_id:={_cfg["pan_motor_id"]}'
            f' tilt_motor_id:={_cfg["tilt_motor_id"]}'
            f' pan_center_steps:={_cfg["pan_center_steps"]}'
            f' tilt_center_steps:={_cfg["tilt_center_steps"]}'
            f' proportional_vel_max:={_cfg["proportional_vel_max"]}'
            f' pan_joint_lower:={_cfg["pan_joint_lower"]}'
            f' pan_joint_upper:={_cfg["pan_joint_upper"]}'
            f' tilt_joint_lower:={_cfg["tilt_joint_lower"]}'
            f' tilt_joint_upper:={_cfg["tilt_joint_upper"]}'
        )

    robot_description = {
        'robot_description': ParameterValue(Command([xacro_cmd]), value_type=str)
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='log',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        name='robot_state_publisher',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            f'{pkg_ctrl}/config/base/control.yaml',
            *([] if not payload else [f'{pkg_ctrl}/config/payloads/{payload}/control.yaml']),
            {'use_sim_time': use_sim_time},
        ],
        output='log',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'rclcpp:=ERROR'],
    )

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

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='log',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    twist_switch_node = Node(
        package='lekiwi_control',
        executable='twist_switch_node',
        name='twist_switch',
        output='log',
        parameters=[
            f'{pkg_ctrl}/config/base/twist_switch.yaml',
            {'use_sim_time': use_sim_time},
        ],
    )

    bool_toggle_node = Node(
        package='lekiwi_control',
        executable='bool_toggle_node',
        name='bool_toggle_node',
        output='log',
        parameters=[
            f'{pkg_ctrl}/config/base/toggles.yaml',
            {'use_sim_time': use_sim_time},
        ],
    )

    extra_spawner_nodes = [
        Node(package='controller_manager', executable='spawner',
             arguments=['base_controller', '-c', '/controller_manager',
                        '--controller-manager-timeout', '30'], output='both'),
        Node(package='controller_manager', executable='spawner',
             arguments=['imu_sensor_broadcaster', '-c', '/controller_manager',
                        '--controller-manager-timeout', '30'], output='both'),
    ]
    if payload == 'pantilt':
        extra_spawner_nodes.append(
            Node(package='controller_manager', executable='spawner',
                 arguments=['pantilt_controller', '-c', '/controller_manager',
                            '--controller-manager-timeout', '30'], output='both'),
        )

    # Start spawners as soon as the controller_manager process starts.
    # joint_state_broadcaster first; other controllers follow after a brief stagger
    # so they do not all race to activate simultaneously.
    controller_spawner = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', '-c', '/controller_manager',
                               '--controller-manager-timeout', '30'],
                    output='both',
                ),
                TimerAction(period=1.0, actions=extra_spawner_nodes),
            ],
        )
    )

    motor_diagnostics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('sts_hardware_interface'), 'launch', 'motor_diagnostics.launch.py'])
        ])
    )

    actions = [
        robot_state_publisher,
        controller_manager,
        controller_spawner,
        teleop_node,
        twist_switch_node,
        bool_toggle_node,
    ]

    if launch_joy:
        actions.append(joy_node)

    if diagnostics:
        actions.append(TimerAction(period=3.0, actions=[motor_diagnostics]))
        actions.append(TimerAction(
            period=3.0,
            actions=[Node(
                package='bno055_hardware_interface',
                executable='bno055_diagnostics',
                name='bno055_diagnostics',
                output='log',
                parameters=[
                    f'{pkg_ctrl}/config/base/bno055_diagnostics.yaml',
                    {'enable_mock_mode': final_use_mock},
                ],
            )],
        ))

    return actions


def generate_launch_description():
    """Declare control-stack launch arguments and launch via launch_setup."""
    declared_arguments = [
        DeclareLaunchArgument(
            'payload',
            default_value='',
            description='Hardware payload: "" for base only, "pantilt" for base + pan-tilt',
        ),
        DeclareLaunchArgument(
            'pantilt_config',
            default_value='pt101',
            description='Pan-tilt mesh variant when payload:="pt100" or "pt101"',
        ),
        DeclareLaunchArgument(
            'sts_serial_port',
            default_value='',
            description='Serial port override; empty string means use urdf_config.yaml value',
        ),
        DeclareLaunchArgument(
            'use_mock',
            default_value='',
            description='Mock mode override (true/false); empty string means use urdf_config.yaml value',
        ),
        DeclareLaunchArgument(
            'diagnostics',
            default_value='true',
            description='Launch motor and IMU diagnostics nodes',
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
