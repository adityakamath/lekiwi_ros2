#!/usr/bin/env python3
"""
Launch the LeKiwi robot control stack.

The 'payload' argument selects which hardware is present alongside the base:
  ""        — base drive (+ IMU unless imu:=false)  [default: no payload]
  "pantilt" — base drive + pan-tilt servos (+ IMU unless imu:=false)

payload:="pantilt" merges pan-tilt onto this shared bus instead of including
pt_control/launch/pantilt.launch.py - see pantilt_ros2 README's "Launch-time bring-up".
"""

import subprocess
import tempfile

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
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
    imu          = _launch_arg_as_bool(context, 'imu')
    launch_joy   = _launch_arg_as_bool(context, 'joy')
    use_sim_time = _launch_arg_as_bool(context, 'use_sim_time')
    hw_type = LaunchConfiguration('ros2_control_hardware_type').perform(context)
    mujoco_model    = LaunchConfiguration('mujoco_model').perform(context)
    mujoco_headless = LaunchConfiguration('mujoco_headless').perform(context)

    pkg_desc = FindPackageShare('lekiwi_description').perform(context)
    pkg_ctrl = FindPackageShare('lekiwi_control').perform(context)
    xacro    = FindExecutable(name='xacro').perform(context)

    urdf = f'{pkg_desc}/urdf/base_pantilt/base_pantilt.urdf.xacro' if payload == 'pantilt' else f'{pkg_desc}/urdf/base/base.urdf.xacro'

    # MJCF must land on disk (not stay in-memory) since its <include> is filesystem-path-based -
    # one file per payload, unlike the urdf var above.
    if mujoco_model:
        final_mujoco_model = mujoco_model
    elif hw_type == 'mujoco':
        if payload == 'pantilt':
            mjcf_cmd = [xacro, f'{pkg_desc}/mjcf/base_pantilt.mjcf.xacro',
                        f'pantilt_config:={pantilt_config}', 'scene:=true']
        else:
            mjcf_cmd = [xacro, f'{pkg_desc}/mjcf/base.mjcf.xacro', 'scene:=true']
        mjcf_xml = subprocess.run(mjcf_cmd, capture_output=True, text=True, check=True).stdout
        mjcf_file = tempfile.NamedTemporaryFile(
            mode='w', suffix='.xml', prefix='lekiwi_mujoco_', delete=False)
        mjcf_file.write(mjcf_xml)
        mjcf_file.close()
        final_mujoco_model = mjcf_file.name
    else:
        final_mujoco_model = ''

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
        f' sts3215_max_vel_steps:={_cfg["sts3215_max_vel_steps"]}'
        f' proportional_acc_max:={_cfg["proportional_acc_max"]}'
        f' internal_max_vel:={_cfg["internal_max_vel"]}'
        f' internal_max_acc:={_cfg["internal_max_acc"]}'
        f' internal_acc_coeff:={_cfg["internal_acc_coeff"]}'
        f' internal_control_period:={_cfg["internal_control_period"]}'
        f' imu:={str(imu).lower()}'
    )
    if payload == 'pantilt':
        xacro_cmd += (
            f' pantilt_config:={pantilt_config}'
            f' proportional_vel_max:={_cfg["proportional_vel_max"]}'
            f' pantilt_internal_max_vel:={_cfg["pantilt_internal_max_vel"]}'
            f' pantilt_internal_max_acc:={_cfg["pantilt_internal_max_acc"]}'
            f' pantilt_internal_acc_coeff:={_cfg["pantilt_internal_acc_coeff"]}'
        )
    if hw_type == 'mujoco':
        xacro_cmd += (
            f' ros2_control_hardware_type:={hw_type}'
            f' mujoco_model:={final_mujoco_model}'
            f' mujoco_headless:={mujoco_headless}'
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

    # mujoco_ros2_control hosts its own simulation + /scan publishing (base.control.xacro) -
    # use_sim_time:true is required regardless of the launch arg.
    mujoco_control_node = Node(
        package='mujoco_ros2_control',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            f'{pkg_ctrl}/config/base/control.yaml',
            *([] if not payload else [f'{pkg_ctrl}/config/payloads/{payload}/control.yaml']),
            {'use_sim_time': True},
        ],
        output='both',
        emulate_tty=True,
    )

    control_node = mujoco_control_node if hw_type == 'mujoco' else controller_manager

    teleop_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([f'{pkg_ctrl}/launch/teleop.launch.py']),
        launch_arguments={
            'payload': payload,
            'use_sim_time': str(use_sim_time).lower(),
            'joy': str(launch_joy).lower(),
        }.items(),
    )

    # bool_toggle_node + twist_switch_node: one process, one MultiThreadedExecutor - both are
    # lightweight and always launch together. No name= here: each node keeps its own hardcoded
    # name in code (matching its own config section below), and launch_ros's name= would emit
    # a bare -r __node:=<name> remap that renames every rclpy.Node in the process to it.
    control_support_node = Node(
        package='lekiwi_control',
        executable='control_support_node',
        output='log',
        parameters=[
            f'{pkg_ctrl}/config/base/toggles.yaml',
            f'{pkg_ctrl}/config/base/twist_switch.yaml',
            {'use_sim_time': use_sim_time},
        ],
    )

    extra_spawner_nodes = [
        Node(package='controller_manager', executable='spawner',
             arguments=['base_controller', '-c', '/controller_manager',
                        '--controller-manager-timeout', '30'], output='both'),
    ]
    if imu:
        # Only spawn when the URDF declares the lekiwi_imu interface (imu:=true branch).
        extra_spawner_nodes.append(
            Node(package='controller_manager', executable='spawner',
                 arguments=['imu_sensor_broadcaster', '-c', '/controller_manager',
                            '--controller-manager-timeout', '30'], output='both'),
        )
    if payload == 'pantilt':
        extra_spawner_nodes.append(
            Node(package='controller_manager', executable='spawner',
                 arguments=['pantilt_controller', '-c', '/controller_manager',
                            '--controller-manager-timeout', '30'], output='both'),
        )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager',
                   '--controller-manager-timeout', '30'],
        output='both',
    )

    # Start spawners as soon as control_node (whichever variant) starts. joint_state_broadcaster
    # first; others follow after a brief stagger so they don't all race to activate at once.
    controller_spawner_actions = [
        RegisterEventHandler(
            OnProcessStart(
                target_action=control_node,
                on_start=[
                    joint_state_broadcaster_spawner,
                    TimerAction(period=1.0, actions=extra_spawner_nodes),
                ],
            )
        )
    ]

    motor_diagnostics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('sts_hardware_interface'), 'launch', 'motor_diagnostics.launch.py'])
        ])
    )

    actions = [
        robot_state_publisher,
        control_node,
        *controller_spawner_actions,
        teleop_include,
        control_support_node,
    ]

    if diagnostics:
        actions.append(TimerAction(period=3.0, actions=[motor_diagnostics]))
        if imu:
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
            description='Pan-tilt mesh variant when payload:="pantilt": "pt100" or "pt101"',
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
            default_value='false',
            description='Launch motor and IMU diagnostics nodes. Defaults to false everywhere '
                        'in the stack (lekiwi.launch.py included) so this default takes effect '
                        'identically whether control.launch.py is run standalone or included.',
        ),
        DeclareLaunchArgument(
            'imu',
            default_value='true',
            description='Whether a physical BNO055 IMU is present (honored on real and '
                        'mujoco hardware; gazebo is xacro-level only, not wired into this '
                        'launch file). false omits the IMU sensor block, skips spawning '
                        'imu_sensor_broadcaster, and skips bno055_diagnostics even when '
                        'diagnostics:=true.',
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
        DeclareLaunchArgument(
            'ros2_control_hardware_type',
            default_value='real',
            description='"real" for STS/BNO055 hardware plugins, "mujoco" for '
                        'mujoco_ros2_control/MujocoSystemInterface (base and pantilt payload both '
                        'supported). "gazebo" is also supported at the URDF/xacro level - '
                        'base.control.xacro/base_pantilt.control.xacro - but not wired into this '
                        'launch file.',
        ),
        DeclareLaunchArgument(
            'mujoco_model',
            default_value='',
            description='Path to a pre-built MJCF file to load; empty means xacro-process '
                        'lekiwi_description/mjcf/base.mjcf.xacro or base_pantilt.mjcf.xacro '
                        '(picked by payload, with pantilt_config) at launch time instead. Only '
                        'used when ros2_control_hardware_type:="mujoco".',
        ),
        DeclareLaunchArgument(
            'mujoco_headless',
            default_value='false',
            description='[mujoco only] Run without the MuJoCo Simulate viewer window.',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
