#!/usr/bin/env python3
"""
Launch the LeKiwi robot control stack.

The 'config' argument selects 'base', 'pantilt', or 'lekiwi' to load
the matching URDF, controller config, teleop config, and spawners:
  base     — wheel drive + IMU (no pantilt)
  pantilt  — pan/tilt servos only (no base drive, no IMU)
  lekiwi   — full robot: base + pantilt + IMU  [default]
"""

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
    serial_port   = LaunchConfiguration('serial_port').perform(context)
    use_mock      = LaunchConfiguration('use_mock').perform(context)
    diagnostics   = LaunchConfiguration('diagnostics').perform(context)
    pan_center    = LaunchConfiguration('pan_center_steps').perform(context)
    tilt_center   = LaunchConfiguration('tilt_center_steps').perform(context)
    teleop_config = LaunchConfiguration('teleop_config').perform(context)

    pkg_desc = FindPackageShare('lekiwi_description').perform(context)
    pkg_ctrl = FindPackageShare('lekiwi_control').perform(context)
    xacro    = FindExecutable(name='xacro').perform(context)

    # Each config has its own URDF: base and pantilt live in subdirectories,
    # lekiwi.urdf.xacro is at the root of the urdf/ folder.
    if config == 'base':
        urdf = f'{pkg_desc}/urdf/base/base.urdf.xacro'
    elif config == 'pantilt':
        urdf = f'{pkg_desc}/urdf/pantilt/pantilt.urdf.xacro'
    else:
        urdf = f'{pkg_desc}/urdf/lekiwi.urdf.xacro'

    # pan/tilt center steps are only declared in pantilt and lekiwi URDFs —
    # passing them to base.urdf.xacro would cause an "unused argument" xacro error.
    xacro_cmd = f'{xacro} {urdf} serial_port:={serial_port} use_mock:={use_mock}'
    if config in ('pantilt', 'lekiwi'):
        xacro_cmd += f' pan_center_steps:={pan_center} tilt_center_steps:={tilt_center}'

    robot_description = {
        'robot_description': ParameterValue(Command([xacro_cmd]), value_type=str)
    }

    # ── Nodes common to all configs ──────────────────────────────────────────

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='log',
        parameters=[robot_description],
        name='robot_state_publisher',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    # Controller config lives at config/{config}/control.yaml — folder name
    # matches the config argument, so no extra branching is needed here.
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, f'{pkg_ctrl}/config/{config}/control.yaml'],
        remappings=[('/diagnostics', '/controller_manager/diagnostics')],
        output='log',
        emulate_tty=True,
    )

    teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[teleop_config],
        output='screen',
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
    else:  # lekiwi — all controllers
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

    if diagnostics.lower() == 'true':
        # Motor diagnostics applies to all configs — all use STS servos.
        actions.append(TimerAction(period=3.0, actions=[motor_diagnostics]))

        # IMU diagnostics only for configs that include the BNO055 (base and lekiwi).
        if config in ('base', 'lekiwi'):
            actions.append(TimerAction(
                period=3.0,
                actions=[Node(
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
            default_value='base',
            description='Robot configuration to launch: base, pantilt, or lekiwi',
        ),
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
            'diagnostics',
            default_value='true',
            description='Launch motor and IMU diagnostics nodes',
        ),
        DeclareLaunchArgument(
            'pan_center_steps',
            default_value='2048',
            description='Encoder step that maps to 0 rad for the pan joint',
        ),
        DeclareLaunchArgument(
            'tilt_center_steps',
            default_value='2646',
            description='Encoder step that maps to 0 rad for the tilt joint',
        ),
        # teleop_config defaults to config/{config}/teleop.yaml, but can be
        # overridden at launch time to point at a custom joystick mapping.
        DeclareLaunchArgument(
            'teleop_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('lekiwi_control'), 'config', LaunchConfiguration('config'), 'teleop.yaml']
            ),
            description='Path to the joy_teleop configuration file',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
