#!/usr/bin/env python3
"""
Launch the LeKiwi robot system.

The 'config' argument selects which subsystems to bring up:
  base     — base drive + IMU, laser, webcam, navigation  [no pantilt]
  pantilt  — pan-tilt servos + OAK-D camera               [no base, no navigation]
  lekiwi   — full system: base + pantilt + all sensors + navigation  [default]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """
    Build and return the list of launch actions for the selected bringup config.

    Called at launch time by OpaqueFunction so all argument values are resolved
    before deciding which subsystems to include.
    """
    config          = LaunchConfiguration('config').perform(context)
    diagnostics     = LaunchConfiguration('diagnostics').perform(context)
    ekf_mode        = LaunchConfiguration('ekf_mode').perform(context)
    pointcloud      = LaunchConfiguration('pointcloud').perform(context)

    pkg_bringup = FindPackageShare('lekiwi_bringup').perform(context)
    pkg_control = FindPackageShare('lekiwi_control').perform(context)
    pkg_nav     = FindPackageShare('lekiwi_navigation').perform(context)

    def include(pkg, path, args=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource([f'{pkg}/{path}']),
            launch_arguments=(args or {}).items(),
        )

    # Control stack — config is forwarded directly to lekiwi_control's launch file
    control = include(pkg_control, 'launch/lekiwi.launch.py', {
        'config':      config,
        'diagnostics': diagnostics,
    })

    nav    = include(pkg_nav,     'launch/nav.launch.py', {'ekf_mode': ekf_mode})
    laser  = include(pkg_bringup, 'launch/laser.launch.py')
    webcam = include(pkg_bringup, 'launch/webcam.launch.py')
    oakd   = include(pkg_bringup, 'launch/oakd.launch.py', {'pointcloud': pointcloud})

    # ── Config-specific subsystem selection ──────────────────────────────────

    if config == 'base':
        # Base drive + navigation; no pantilt hardware, no OAK-D
        return [control, nav, laser, webcam]

    elif config == 'pantilt':
        # Pan-tilt + OAK-D only; no base drive, no navigation, no 2D sensors
        return [control, oakd]

    else:  # lekiwi — full system
        return [control, nav, laser, webcam, oakd]


def generate_launch_description():
    """
    Declare launch arguments and hand off to launch_setup via OpaqueFunction.

    The 'config' argument mirrors the one in lekiwi_control/launch/lekiwi.launch.py
    and gates which sensors and navigation stack are brought up alongside it.
    """
    declared_arguments = [
        DeclareLaunchArgument(
            'config',
            default_value='base',
            description='Bringup configuration: base, pantilt, or lekiwi',
        ),
        DeclareLaunchArgument(
            'diagnostics',
            default_value='true',
            description='Launch motor and IMU diagnostics nodes',
        ),
        DeclareLaunchArgument(
            'ekf_mode',
            default_value='base',
            description=(
                'EKF sensor fusion mode (passed to nav.launch.py): '
                'base = wheel odom + BNO055; '
                'imu = BNO055 only (test platform); '
                'odom = wheel odom only (mock / debug).'
            ),
        ),
        DeclareLaunchArgument(
            'pointcloud',
            default_value='false',
            description='Enable RGBD point cloud output from OAK-D (uses oakd_vio_pcl.yaml)',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
