#!/usr/bin/env python3
"""Launch the LeKiwi base controller, configured sensors and teleoperation."""

import subprocess
import shlex
import sys
import tempfile

import yaml

from lekiwi_mujoco.hardware_config import load_hardware_config

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_ros.utilities import make_namespace_absolute, prefix_namespace


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
    """Build the control-stack nodes for the base."""
    hardware_config = LaunchConfiguration('hardware_config').perform(context)
    hardware = load_hardware_config(hardware_config)
    enable_camera = _launch_arg_as_bool(context, 'enable_camera') and hardware['gemini2']
    pointcloud = _launch_arg_as_bool(context, 'pointcloud')
    serial_port  = LaunchConfiguration('sts_serial_port').perform(context)
    use_mock     = LaunchConfiguration('use_mock').perform(context)
    diagnostics  = _launch_arg_as_bool(context, 'diagnostics')
    launch_joy   = _launch_arg_as_bool(context, 'joy')
    use_sim_time = _launch_arg_as_bool(context, 'use_sim_time')
    enable_odom_tf = _launch_arg_as_bool(context, 'enable_odom_tf')
    prefix = LaunchConfiguration('frame_prefix').perform(context)
    ros_namespace = context.launch_configurations.get('ros_namespace')

    def in_namespace(name):
        # The MuJoCo plugins create their topics from sub-nodes, so these names are spelled out
        # under the robot's namespace.
        return make_namespace_absolute(prefix_namespace(ros_namespace, name))

    # A params file, not a dotted key: the controller reads its own base_controller.ros__parameters.
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='lekiwi_odom_tf_', delete=False) as f:
        # Sensor frames are named in control.yaml; each robot's must carry its own prefix.
        controller_params = {
            '/**/base_controller': {'ros__parameters': {'enable_odom_tf': enable_odom_tf}},
            '/**/imu_sensor_broadcaster': {'ros__parameters': {'frame_id': f'{prefix}imu_frame'}},
            '/**/gemini2_imu_broadcaster': {'ros__parameters': {
                'frame_id': f'{prefix}gemini2_accel_gyro_optical_frame'}},
        }
        yaml.safe_dump(controller_params, f)
        odom_tf_params = f.name
    hw_type = LaunchConfiguration('ros2_control_hardware_type').perform(context)
    mujoco_model    = LaunchConfiguration('mujoco_model').perform(context)
    mujoco_scene = LaunchConfiguration('mujoco_scene').perform(context)
    mujoco_headless = LaunchConfiguration('mujoco_headless').perform(context)

    pkg_desc = FindPackageShare('lekiwi_description').perform(context)
    pkg_ctrl = FindPackageShare('lekiwi_control').perform(context)
    pkg_mujoco = FindPackageShare('lekiwi_mujoco').perform(context)
    xacro    = FindExecutable(name='xacro').perform(context)

    urdf = f'{pkg_desc}/urdf/base.urdf.xacro'

    # The control plugin loads MJCF from disk. Generate the configured base model.
    if mujoco_model:
        final_mujoco_model = mujoco_model
    elif hw_type == 'mujoco':
        # One compiler for committed assets, headless tests and ROS; it also
        # uses the configured base frames and sensor assemblies.
        with tempfile.NamedTemporaryFile(
                suffix='.xml', prefix='lekiwi_mujoco_', delete=False) as mjcf_file:
            final_mujoco_model = mjcf_file.name
        subprocess.run([
            sys.executable, '-m', 'lekiwi_mujoco.build_mujoco_models',
            '--control-package', pkg_ctrl, '--description-package', pkg_desc,
            '--output', final_mujoco_model, '--absolute', '--scene', mujoco_scene,
            '--lidar', 'plugin',
            '--hardware-config', hardware_config,
            '--frame-prefix', prefix,
        ], capture_output=True, text=True, check=True)
    else:
        final_mujoco_model = ''

    with open(f'{pkg_ctrl}/config/urdf_config.yaml') as stream:
        _cfg = yaml.safe_load(stream)
    final_serial_port = serial_port if serial_port else _cfg['serial_port']
    final_use_mock    = use_mock if use_mock else str(_cfg['use_mock']).lower()

    # Build argv before quoting, so profile/package paths containing spaces are valid.
    xacro_args = [
        xacro, urdf,
        f'base_controller_config:={pkg_ctrl}/config/control.yaml',
        f'serial_port:={final_serial_port}',
        f'use_mock:={final_use_mock}',
        *[f'{name}:={str(_cfg[name]).lower()}' for name in (
            'baud_rate', 'use_sync_write', 'left_motor_id', 'back_motor_id', 'right_motor_id',
            'sts3215_max_vel_steps', 'proportional_acc_max', 'internal_max_vel',
            'internal_max_acc', 'internal_acc_coeff', 'internal_control_period')],
        f'hardware_config:={hardware_config}',
        f'enable_camera:={str(enable_camera).lower()}',
    ]
    if hw_type == 'mujoco':
        xacro_args += [
            f'ros2_control_hardware_type:={hw_type}',
            f'mujoco_model:={final_mujoco_model}',
            f'mujoco_headless:={mujoco_headless}',
        ]
    xacro_cmd = shlex.join(xacro_args)

    robot_description = {
        'robot_description': ParameterValue(Command([xacro_cmd]), value_type=str)
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='log',
        parameters=[robot_description, {'use_sim_time': use_sim_time},
                    {'frame_prefix': prefix}],
        name='robot_state_publisher',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            f'{pkg_ctrl}/config/control.yaml',
            {'use_sim_time': use_sim_time},
            odom_tf_params,
        ],
        output='log',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'rclcpp:=ERROR'],
    )

    # Remove plugin entries for absent devices as well as their processing nodes.
    sensor_plugins = f'{pkg_mujoco}/config/mujoco_ros2_control_plugins.yaml'
    if hw_type == 'mujoco' and not (hardware['sparkfun_sensor'] and hardware['lidar']):
        with open(sensor_plugins) as stream:
            plugin_config = yaml.safe_load(stream)
        plugins = plugin_config['/**']['ros__parameters']['mujoco_plugins']
        lidar_plugin = plugins['mujoco_3d_lidar_plugin']
        for name in ('lidar', 'tof'):
            if not hardware['lidar' if name == 'lidar' else 'sparkfun_sensor']:
                lidar_plugin.pop(name, None)
        if not {'lidar', 'tof'} & lidar_plugin.keys():
            plugins.pop('mujoco_3d_lidar_plugin')
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='lekiwi_sensors_', delete=False) as stream:
            yaml.safe_dump(plugin_config, stream)
            sensor_plugins = stream.name

    # The plugins create topics from sub-nodes, so each robot's topic names and sensor frames are set here.
    plugins = {'emergency_stop_plugin': {'service_name': in_namespace('emergency_stop')}}
    if hardware['lidar']:
        plugins.setdefault('mujoco_3d_lidar_plugin', {})['lidar'] = {
            'frame_name': f'{prefix}laser_frame', 'topic': in_namespace('scan_raw')}
    if hardware['sparkfun_sensor']:
        plugins.setdefault('mujoco_3d_lidar_plugin', {})['tof'] = {
            'frame_name': f'{prefix}tof_frame', 'topic': in_namespace('tof/pointcloud_raw')}
    if enable_camera:
        plugins['mujoco_camera_plugin'] = {'gemini2_rgb': {
            'frame_name': f'{prefix}gemini2_color_optical_frame',
            'info_topic': in_namespace('gemini2/color/camera_info'),
            'image_topic': in_namespace('gemini2/color/image_raw'),
            'depth_topic': in_namespace('_gemini2/depth_raw')}}
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='lekiwi_plugins_', delete=False) as stream:
        yaml.safe_dump({'/**': {'ros__parameters': {'mujoco_plugins': plugins}}}, stream)
        plugin_overrides = stream.name

    # mujoco_ros2_control hosts its own simulation + /scan publishing (base.control.xacro) -
    # use_sim_time:true is required regardless of the launch arg.
    mujoco_control_node = Node(
        package='mujoco_ros2_control',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            f'{pkg_ctrl}/config/control.yaml',
            *([f'{pkg_ctrl}/config/gemini2_imu_broadcaster.yaml']
              if enable_camera else []),
            sensor_plugins,
            *([f'{pkg_mujoco}/config/mujoco_camera_plugin.yaml',
               f'{pkg_mujoco}/config/mujoco_camera_gemini2.yaml']
              if enable_camera else []),
            plugin_overrides,
            {'use_sim_time': True},
            odom_tf_params,
        ],
        output='both',
        emulate_tty=True,
    )

    control_node = mujoco_control_node if hw_type == 'mujoco' else controller_manager

    # Sim-only extras: the plugin's raw lidar scan goes through the same laser_filters
    # raw-to-/scan split the real LD06 uses, and the camera images need an optical frame.
    sim_nodes = []
    if hw_type == 'mujoco':
        filter_file = 'mujoco_laser_filter.yaml'
        if hardware['lidar']:
            sim_nodes.append(Node(
                package='laser_filters',
                executable='scan_to_scan_filter_chain',
                name='laser_scan_filter_chain',
                output='log',
                parameters=[f'{pkg_mujoco}/config/{filter_file}', {'use_sim_time': True}],
                remappings=[('scan', 'scan_raw'), ('scan_filtered', 'scan')],
            ))
        if hardware['sparkfun_sensor']:
            sim_nodes.append(Node(
                package='lekiwi_mujoco', executable='tof_pointcloud',
                name='tof_pointcloud', output='log',
                parameters=[{'use_sim_time': True}, {'frame_id': f'{prefix}tof_frame'}],
            ))
        if enable_camera:
            sim_nodes.extend([
                Node(
                    package='lekiwi_mujoco', executable='gemini2_depth',
                    name='gemini2_depth', output='log', parameters=[{'use_sim_time': True}],
                ),
                Node(
                    package='tf2_ros', executable='static_transform_publisher',
                    name='gemini2_imu_tf', output='log',
                    arguments=['--frame-id', f'{prefix}gemini2_color_optical_frame',
                               '--child-frame-id', f'{prefix}gemini2_accel_gyro_optical_frame'],
                    parameters=[{'use_sim_time': True}],
                ),
                Node(
                    package='tf2_ros', executable='static_transform_publisher',
                    name='gemini2_optical_frame_publisher', output='log',
                    arguments=['--frame-id', f'{prefix}gemini2_link',
                               '--child-frame-id', f'{prefix}gemini2_color_optical_frame',
                               '--roll', '-1.5707963', '--pitch', '0', '--yaw', '-1.5707963'],
                    parameters=[{'use_sim_time': True}],
                ),
            ])
            if pointcloud:
                sim_nodes.extend([
                    Node(
                        package='depth_image_proc', executable='point_cloud_xyzrgb_node',
                        name='gemini2_colored_pointcloud', output='log',
                        parameters=[{'use_sim_time': True}],
                        remappings=[('rgb/image_rect_color', 'gemini2/color/image_raw'),
                                    ('rgb/camera_info', 'gemini2/color/camera_info'),
                                    ('depth_registered/image_rect', 'gemini2/depth/image_raw'),
                                    ('points', '_gemini2/registered_points')],
                    ),
                    Node(
                        package='lekiwi_mujoco', executable='gemini2_cloud',
                        name='gemini2_cloud', output='log',
                        parameters=[{'use_sim_time': True}],
                    ),
                ])

    teleop_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([f'{pkg_ctrl}/launch/teleop.launch.py']),
        launch_arguments={
            'use_sim_time': str(use_sim_time).lower(),
            'joy': str(launch_joy).lower(),
        }.items(),
    )

    # bool_toggle_node + twist_switch_node + collision_toggle_node: one process, one
    # MultiThreadedExecutor - all three are lightweight and always launch together. No name=
    # here: each node keeps its own hardcoded name in code (matching its own config section
    # below), and launch_ros's name= would emit a bare -r __node:=<name> remap that renames
    # every rclpy.Node in the process to it. collision_toggle_node's target (collision_monitor,
    # in lekiwi_navigation) doesn't need to be running - it no-ops if it isn't.
    control_support_node = Node(
        package='lekiwi_control',
        executable='control_support_node',
        output='log',
        parameters=[
            f'{pkg_ctrl}/config/toggles.yaml',
            f'{pkg_ctrl}/config/twist_switch.yaml',
            f'{pkg_ctrl}/config/collision_toggle.yaml',
            {'use_sim_time': use_sim_time},
            # Without a LiDAR there is no collision monitor, so commands go straight to the base.
            *([] if hardware['lidar'] else [{'output_topic': 'base_controller/cmd_vel'}]),
            {'frame_id': f'{prefix}base_link'},
        ],
    )

    # Spawners start from event handlers, outside the namespace scope, so they carry it explicitly.
    extra_spawner_nodes = [
        Node(package='controller_manager', executable='spawner',
             arguments=['base_controller', '-c', 'controller_manager',
                        '--controller-manager-timeout', '30'], output='both', namespace=ros_namespace),
    ]
    if hardware['imu']:
        extra_spawner_nodes.append(
            Node(package='controller_manager', executable='spawner',
                 arguments=['imu_sensor_broadcaster', '-c', 'controller_manager',
                            '--controller-manager-timeout', '30'], output='both', namespace=ros_namespace),
        )
    if hw_type == 'mujoco' and enable_camera:
        extra_spawner_nodes.append(
            Node(package='controller_manager', executable='spawner',
                 arguments=['gemini2_imu_broadcaster', '-c', 'controller_manager',
                            '--controller-manager-timeout', '30',
                            '--controller-ros-args', '--ros-args -r ~/imu:=gemini2/gyro_accel/sample'],
                 output='both', namespace=ros_namespace),
        )
    # Real-hardware motor health for motor_diagnostics_node (/dynamic_joint_states is gone on Lyrical);
    # the MuJoCo joints export no voltage/temperature/current/is_moving interfaces.
    if diagnostics and hw_type != 'mujoco':
        sts_joints = ['left_wheel_joint', 'back_wheel_joint', 'right_wheel_joint']
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='lekiwi_state_ifaces_', delete=False) as f:
            yaml.safe_dump({'/**/state_interfaces_broadcaster': {'ros__parameters': {'interfaces': [
                f'{j}/{i}' for j in sts_joints
                for i in ('effort', 'voltage', 'temperature', 'current', 'is_moving')]}}}, f)
        extra_spawner_nodes.append(
            Node(package='controller_manager', executable='spawner',
                 arguments=['state_interfaces_broadcaster', '-c', 'controller_manager',
                            '--controller-manager-timeout', '30', '--param-file', f.name],
                 output='both', namespace=ros_namespace))

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', 'controller_manager',
                   '--controller-manager-timeout', '30'],
        output='both',
        namespace=ros_namespace,
    )

    # Spawners run one after another: concurrent load_controller calls race in the controller manager.
    spawners = [joint_state_broadcaster_spawner, *extra_spawner_nodes]
    controller_spawner_actions = [
        RegisterEventHandler(OnProcessStart(target_action=control_node, on_start=[spawners[0]])),
        *[RegisterEventHandler(OnProcessExit(target_action=done, on_exit=[nxt]))
          for done, nxt in zip(spawners, spawners[1:])],
    ]

    motor_diagnostics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('sts_hardware_interface'), 'launch', 'motor_diagnostics.launch.py'])
        ]),
    )

    actions = [
        robot_state_publisher,
        control_node,
        *controller_spawner_actions,
        teleop_include,
        control_support_node,
        *sim_nodes,
    ]

    if diagnostics and hw_type != 'mujoco':
        actions.append(TimerAction(period=3.0, actions=[motor_diagnostics]))
    if diagnostics and hw_type != 'mujoco' and hardware['imu']:
        actions.append(TimerAction(
            period=3.0,
            actions=[Node(
                package='bno055_hardware_interface',
                executable='bno055_diagnostics',
                name='bno055_diagnostics',
                output='log',
                parameters=[
                    f'{pkg_ctrl}/config/bno055_diagnostics.yaml',
                    {'enable_mock_mode': final_use_mock},
                ],
            )],
        ))

    return actions


def generate_launch_description():
    """Declare control-stack launch arguments and launch via launch_setup."""
    declared_arguments = [
        DeclareLaunchArgument('frame_prefix', default_value='',
                              description='TF frame prefix, e.g. robot1/; set by lekiwi.launch.py.'),
        DeclareLaunchArgument('hardware_config',
                              default_value=PathJoinSubstitution([FindPackageShare('lekiwi_bringup'), 'config', 'hardware.yaml']),
                              description='Optional robot assemblies.'),

        DeclareLaunchArgument('enable_camera', default_value='true',
                              description='Start simulated camera publishing; geometry remains selected.'),
        DeclareLaunchArgument('pointcloud', default_value='false',
                              description='[simulation] Publish one colored cloud (real hardware uses lekiwi_bringup Cloudini compression when enabled).'),
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
            'use_sim_time',
            default_value='false',
            description='Use /clock from a simulator instead of system time.',
        ),
        DeclareLaunchArgument(
            'enable_odom_tf',
            default_value='true',
            description='Let base_controller publish the odom -> base_footprint TF from wheel odometry. '
                        'Default true so this launch file is usable standalone; lekiwi.launch.py passes '
                        'false because the navigation EKF (robot_localization) publishes it there.',
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
                        'mujoco_ros2_control/MujocoSystemInterface (base '
                        'supported). "gazebo" is also supported at the URDF/xacro level - '
                        'base.control.xacro - but not wired into this '
                        'launch file.',
        ),
        DeclareLaunchArgument('mujoco_scene', default_value='flat',
                              description='flat, none, or scene MJCF path for generated models'),
        DeclareLaunchArgument(
            'mujoco_model',
            default_value='',
            description='Path to a pre-built MJCF file to load; empty means generate with MjSpec from '
                        'the base MJCF and URDF '
                        'at launch time instead. Only '
                        'used when ros2_control_hardware_type:="mujoco".',
        ),
        DeclareLaunchArgument(
            'mujoco_headless',
            default_value='false',
            description='[mujoco only] Run without the MuJoCo Simulate viewer window.',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
