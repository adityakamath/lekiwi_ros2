#!/usr/bin/env python3
"""Launch the base Orbbec Gemini 2 driver and optional cloud compressor."""

from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, SetParameter
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def _boolean(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value not in ('true', 'false', '1', '0'):
        raise RuntimeError(f'{name} must be true/false or 1/0, got {value!r}')
    return value in ('true', '1')


def launch_setup(context):
    if not _boolean(context, 'enable_camera'):
        return []
    config = Path(FindPackageShare('lekiwi_bringup').perform(context)) / 'config/gemini2.yaml'
    rates = yaml.safe_load(config.read_text())['/gemini2/gemini2']['ros__parameters']
    for stream in ('color_fps', 'depth_fps'):
        if type(rates[stream]) is not int or rates[stream] not in (5, 10, 15, 30):
            raise RuntimeError(f'{config}: {stream} must be 5, 10, 15, or 30; automatic FPS is disabled')
    for size in ('color_width', 'color_height', 'depth_width', 'depth_height'):
        if type(rates[size]) is not int or rates[size] <= 0:
            raise RuntimeError(f'{config}: {size} must be a positive integer')
    pointcloud = 'true' if _boolean(context, 'pointcloud') else 'false'
    flags = {}
    for flag in ('enable_imu', 'depth_registration', 'enable_noise_removal_filter',
                 'enable_frame_sync', 'enable_ldp'):
        if type(rates[flag]) is not bool:
            raise RuntimeError(f'{config}: {flag} must be true or false')
        flags[flag] = str(rates[flag]).lower()
    if type(rates['laser_energy_level']) is not int or rates['laser_energy_level'] < -1:
        raise RuntimeError(f'{config}: laser_energy_level must be an integer >= -1')
    if pointcloud == 'true' and flags['depth_registration'] != 'true':
        raise RuntimeError(f'{config}: pointcloud:=true needs depth_registration: true')
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('orbbec_camera'), 'launch', 'gemini2.launch.py',
        ])),
        launch_arguments={
            'camera_name': 'gemini2',
            'serial_number': LaunchConfiguration('serial_number'),
            'usb_port': LaunchConfiguration('usb_port'),
            'enable_color': 'true',
            'color_width': str(rates['color_width']),
            'color_height': str(rates['color_height']),
            'color_fps': str(rates['color_fps']),
            'enable_depth': 'true',
            'depth_width': str(rates['depth_width']),
            'depth_height': str(rates['depth_height']),
            'depth_fps': str(rates['depth_fps']),
            'enable_ir': 'false',
            'enable_accel': flags['enable_imu'],
            'enable_gyro': flags['enable_imu'],
            'enable_sync_output_accel_gyro': flags['enable_imu'],
            'depth_registration': flags['depth_registration'],
            'enable_noise_removal_filter': flags['enable_noise_removal_filter'],
            'enable_frame_sync': flags['enable_frame_sync'],
            'enable_ldp': flags['enable_ldp'],
            'laser_energy_level': str(rates['laser_energy_level']),
            'enable_point_cloud': 'false',
            'enable_colored_point_cloud': pointcloud,
            'publish_tf': 'true',
        }.items(),
    )
    # image_transport loads every installed plugin per topic; keep only the useful ones.
    decimation = LaunchConfiguration('pointcloud_decimation').perform(context).strip()
    if not decimation.isdigit() or int(decimation) < 1:
        raise RuntimeError(f'pointcloud_decimation must be an integer >= 1, got {decimation!r}')
    actions = [LogInfo(msg=f'Gemini 2: color={rates["color_fps"]} FPS, '
                           f'depth={rates["depth_fps"]} FPS; configuration: {config}')]
    if pointcloud == 'true':
        # Keep every Nth point per axis in the driver's cloud only; set before the driver starts
        actions.append(SetParameter('point_cloud_decimation_filter_factor', int(decimation)))
    actions += [
        SetParameter('color.image_raw.enable_pub_plugins',
                     ['image_transport/raw', 'image_transport/compressed']),
        SetParameter('depth.image_raw.enable_pub_plugins',
                     ['image_transport/raw', 'image_transport/compressedDepth']),
        driver,
    ]
    if pointcloud == 'true':
        actions.append(ComposableNodeContainer(
            name='gemini2_cloudini_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[ComposableNode(
                package='lekiwi_bringup',
                plugin='lekiwi_bringup::PCLCompressorNode',
                name='gemini2_cloudini_compressor',
                parameters=[PathJoinSubstitution([
                    FindPackageShare('lekiwi_bringup'), 'config', 'gemini2.yaml',
                ])],
            )],
            output='both',
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_camera', default_value='true',
                              description='Start the camera driver and streaming helpers.'),
        DeclareLaunchArgument('pointcloud', default_value='false',
                              description='Publish one colored Orbbec point cloud and compress it with Cloudini.'),
        DeclareLaunchArgument('pointcloud_decimation', default_value='2',
                              description='Keep every Nth point per axis in the point cloud (2 = a quarter of the points).'),
        DeclareLaunchArgument('serial_number', default_value='',
                              description='Orbbec device serial; empty selects the first device.'),
        DeclareLaunchArgument('usb_port', default_value='',
                              description='Optional Orbbec USB port selector.'),
        OpaqueFunction(function=launch_setup),
    ])
