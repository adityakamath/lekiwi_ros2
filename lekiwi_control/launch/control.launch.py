#!/usr/bin/env python3
"""
Launch the LeKiwi robot control stack.

The 'config' argument selects 'base', 'pantilt', or 'k2' to load
the matching URDF, controller config, teleop config, and spawners:
  base     — wheel drive + IMU (no pantilt)
  pantilt  — pan/tilt servos only (no base drive, no IMU)
  k2       — full K2 (LeKiwi2) robot: base + pantilt + IMU [default]
"""

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    """
    Build and return the list of launch actions for the selected robot config.

    Called at launch time by OpaqueFunction, so all LaunchConfiguration values
    are fully resolved before any conditional branching happens. This avoids the
    complexity of composing runtime substitutions for config-dependent paths and
    xacro arguments.
    """
    config        = LaunchConfiguration('config').perform(context)
    serial_port   = LaunchConfiguration('sts_serial_port').perform(context)
    use_mock      = LaunchConfiguration('use_mock').perform(context)
    diagnostics   = LaunchConfiguration('diagnostics').perform(context)
    launch_joy    = LaunchConfiguration('joy').perform(context).lower() in ('true', '1')
    use_sim_time  = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1')

    pkg_desc = FindPackageShare('lekiwi_description').perform(context)
    pkg_ctrl = FindPackageShare('lekiwi_control').perform(context)
    xacro    = FindExecutable(name='xacro').perform(context)

    # Each config has its own URDF: for 'base' or 'pantilt', use subdirectory; otherwise use k2.urdf.xacro at root.
    if config in ('base', 'pantilt'):
        urdf = f'{pkg_desc}/urdf/{config}/{config}.urdf.xacro'
    else:
        urdf = f'{pkg_desc}/urdf/k2.urdf.xacro'

    # pan/tilt center steps are only declared in pantilt and k2 URDFs —
    # passing them to base.urdf.xacro would cause an "unused argument" xacro error.
    xacro_cmd = f'{xacro} {urdf} serial_port:={serial_port} use_mock:={use_mock}'
    if config in ('pantilt', 'k2'):
        # Read calibration values from the per-config pantilt_limits.yaml rather
        # than from launch arguments — callers don't need to know these values.
        _limits = yaml.safe_load(open(f'{pkg_ctrl}/config/{config}/urdf_config.yaml'))
        xacro_cmd += (
            f' pan_center_steps:={_limits["pan_center_steps"]}'
            f' tilt_center_steps:={_limits["tilt_center_steps"]}'
        )

    robot_description = {
        'robot_description': ParameterValue(Command([xacro_cmd]), value_type=str)
    }

    # ── Nodes common to all configs ──────────────────────────────────────────

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='log',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        name='robot_state_publisher',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    # Controller config lives at config/{config}/control.yaml — folder name
    # matches the config argument, so no extra branching is needed here.
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            f'{pkg_ctrl}/config/{config}/control.yaml',
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('/diagnostics', '/controller_manager/diagnostics')],
        output='log',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'rclcpp:=ERROR'],
    )

    teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[f'{pkg_ctrl}/config/{config}/teleop.yaml', {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # joy_node reads the gamepad and publishes /joy. Disabled by default because
    # the joystick is normally connected to and published from a remote device.
    # Set launch_joy:=true to run joy_node on the robot itself.
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='log',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # twist_switch_node bridges /cmd_vel_teleop (joy_teleop) and /cmd_vel_nav (nav stack)
    # onto /base_controller/cmd_vel. Only launched for configs that include the
    # base_controller (base and k2) — pantilt has no drive controller.
    twist_switch_node = Node(
        package='lekiwi_control',
        executable='twist_switch_node',
        name='twist_switch',
        output='log',
        parameters=[
            f'{pkg_ctrl}/config/twist_switch.yaml',
            {'use_sim_time': use_sim_time},
        ],
    )

    # joint_state_broadcaster must be active before the other controllers so
    # that the controller_manager can read joint states during activation.
    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
            output='both',
        )],
    )

    # Motor diagnostics reads STS servo telemetry (voltage, temperature, load)
    # from the shared serial bus and publishes to /base/diagnostics.
    motor_diagnostics = GroupAction([
        SetRemap('/diagnostics', '/base/diagnostics'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('sts_hardware_interface'), 'launch', 'motor_diagnostics.launch.py'])
            ])
        ),
    ])

    # ── Config-specific controller spawners ──────────────────────────────────

    if config == 'base':
        extra_spawners = TimerAction(
            period=2.5,
            actions=[
                Node(package='controller_manager', executable='spawner',
                     arguments=['base_controller', '-c', '/controller_manager'], output='both'),
                Node(package='controller_manager', executable='spawner',
                     arguments=['imu_sensor_broadcaster', '-c', '/controller_manager'], output='both'),
            ],
        )
    elif config == 'pantilt':
        extra_spawners = TimerAction(
            period=2.5,
            actions=[
                Node(package='controller_manager', executable='spawner',
                     arguments=['pantilt_controller', '-c', '/controller_manager'], output='both'),
            ],
        )
    else:  # k2 — all controllers
        extra_spawners = TimerAction(
            period=2.5,
            actions=[
                Node(package='controller_manager', executable='spawner',
                     arguments=['base_controller', '-c', '/controller_manager'], output='both'),
                Node(package='controller_manager', executable='spawner',
                     arguments=['imu_sensor_broadcaster', '-c', '/controller_manager'], output='both'),
                Node(package='controller_manager', executable='spawner',
                     arguments=['pantilt_controller', '-c', '/controller_manager'], output='both'),
            ],
        )

    # ── Assemble action list ─────────────────────────────────────────────────

    actions = [
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        extra_spawners,
        teleop_node,
    ]

    if launch_joy:
        actions.append(joy_node)

    # Add twist_switch_node for configs with a base_controller
    if config in ('base', 'k2'):
        actions.append(twist_switch_node)

    if diagnostics.lower() == 'true':
        # Motor diagnostics applies to all configs — all use STS servos.
        actions.append(TimerAction(period=3.0, actions=[motor_diagnostics]))

        # IMU diagnostics only for configs that include the BNO055 (base and k2).
        if config in ('base', 'k2'):
            actions.append(TimerAction(
                period=3.0,
                actions=[Node(
                    package='bno055_hardware_interface',
                    executable='bno055_diagnostics',
                    name='bno055_diagnostics',
                    output='log',
                    parameters=[
                        f'{pkg_ctrl}/config/bno055_diagnostics.yaml',
                        {'enable_mock_mode': use_mock},
                    ],
                    remappings=[('/diagnostics', '/imu/diagnostics')],
                )],
            ))

    return actions


def generate_launch_description():
    """
    Declare launch arguments and hand off to launch_setup via OpaqueFunction.

    All argument defaults are defined here; the actual node construction is
    deferred to launch_setup so that config-dependent logic can use plain
    Python conditionals rather than runtime substitution composition.
    """
    declared_arguments = [
        DeclareLaunchArgument(
            'config',
            default_value='k2',
            description='Robot configuration to launch: base, pantilt, or k2',
        ),
        DeclareLaunchArgument(
            'sts_serial_port',
            default_value='/dev/ttySERVO',
            description='Serial port for STS motor communication',
        ),
        DeclareLaunchArgument(
            'use_mock',
            default_value='false',
            description='Use mock/simulation mode (no hardware required)',
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
