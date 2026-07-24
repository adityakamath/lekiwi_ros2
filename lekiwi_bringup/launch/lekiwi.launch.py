#!/usr/bin/env python3
"""
Launch the LeKiwi robot system.

The 'payload' argument selects which hardware payload is present:
  ""        — base drive + IMU + navigation + laser [no pan-tilt, no OAK-D]
  "pantilt" — base + pan-tilt + OAK-D + navigation + laser [default]
"""

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_VALID_PAYLOADS = {'', 'pantilt'}


def launch_setup(context):
    """Validate arguments and include control/navigation/laser (+ oakd for pantilt)."""
    payload             = LaunchConfiguration('payload').perform(context)
    pantilt_config      = LaunchConfiguration('pantilt_config').perform(context)
    diagnostics         = LaunchConfiguration('diagnostics').perform(context)
    control_mock        = LaunchConfiguration('control_mock').perform(context)
    fusion_mode         = LaunchConfiguration('fusion_mode').perform(context)
    map_name            = LaunchConfiguration('map_name').perform(context)
    pointcloud          = LaunchConfiguration('pointcloud').perform(context)
    octomap             = LaunchConfiguration('octomap').perform(context)
    mission             = LaunchConfiguration('mission').perform(context)
    use_sim_time        = LaunchConfiguration('use_sim_time').perform(context)
    sim                 = LaunchConfiguration('sim').perform(context).strip().lower() in ('true', '1')
    gui                 = LaunchConfiguration('gui').perform(context)

    if sim:
        # Running in Gazebo implies sim time; keep any explicit control_mock override,
        # otherwise force it. Both base and pantilt payloads render the real STS/BNO055
        # plugin blocks entirely out of the URDF in gazebo mode (control.launch.py's
        # ros2_control_hardware_type), so this is mostly defensive at this point - it
        # only matters if a future payload doesn't have gz_ros2_control wired up yet and
        # silently falls back to the real hardware plugin, which would otherwise try to
        # open a real serial port that doesn't exist.
        use_sim_time = 'true'
        control_mock = control_mock or 'true'

    if payload not in _VALID_PAYLOADS:
        raise RuntimeError(
            f"[lekiwi.launch.py] Unknown payload '{payload}'. "
            f"Valid values: {sorted(_VALID_PAYLOADS)}"
        )
    if pointcloud.lower() in ('true', '1') and payload != 'pantilt':
        raise RuntimeError(
            "[lekiwi.launch.py] pointcloud:=true requires payload:=pantilt."
        )
    if octomap.lower() in ('true', '1') and pointcloud.lower() not in ('true', '1'):
        raise RuntimeError(
            "[lekiwi.launch.py] octomap:=true requires pointcloud:=true."
        )

    pkg_bringup = FindPackageShare('lekiwi_bringup').perform(context)
    pkg_control = FindPackageShare('lekiwi_control').perform(context)
    pkg_nav     = FindPackageShare('lekiwi_navigation').perform(context)
    # pt_bringup is only resolved down in the real-hardware/payload:="pantilt" branch below
    # (for oakd) - not needed at all in sim mode (oakd has no simulated equivalent yet), so
    # resolving it here unconditionally would fail any sim:=true run where it isn't built.

    def include(pkg, path, args=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource([f'{pkg}/{path}']),
            launch_arguments=(args or {}).items(),
        )

    control = include(pkg_control, 'launch/control.launch.py', {
        'payload':        payload,
        'pantilt_config': pantilt_config,
        'diagnostics':  diagnostics,
        'use_mock':     control_mock,
        'use_sim_time': use_sim_time,
        'ros2_control_hardware_type': 'gazebo' if sim else 'real',
    })

    nav = include(pkg_nav,     'launch/navigation.launch.py', {
        'fusion_mode':  fusion_mode,
        'mission':      mission,
        'map_name':     map_name,
        'use_sim_time': use_sim_time,
    })

    actions = [control, nav]

    if sim:
        # Gazebo itself, plus the bridge/spawn plumbing. Everything above the hardware
        # interface (controllers, teleop, twist_switch_node, collision_monitor, nav2) is
        # the same control/nav includes as real hardware - only the hardware layer differs,
        # via control.launch.py's ros2_control_hardware_type above. laser/audio/oakd are
        # real-sensor drivers with no simulated equivalent yet, so they're skipped here.
        # gz-sim's own bundled empty.sdf doesn't load the Sensors system plugin, so <sensor>
        # elements (lidar, IMU, camera) never produce data - use our own copy of it
        # with Sensors added instead.
        world_file = f'{pkg_bringup}/worlds/empty.sdf'
        gz_launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']
            ),
            launch_arguments={
                'gz_args': f' -r -v 4 {world_file}' if gui.lower() in ('true', '1') else f' -s -r -v 4 {world_file}',
            }.items(),
        )
        gz_sim_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                # LD06/LD19 lidar (laser_frame sensor, lekiwi_description/urdf/base/base.module.xacro)
                'scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                # BNO055 IMU (imu_frame sensor, same file) - published directly onto the
                # topic imu_sensor_broadcaster would use, since it isn't spawned in gazebo
                # mode (control.launch.py)
                'imu_sensor_broadcaster/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            ],
            output='screen',
        )
        # -topic robot_description subscribes and waits for control's robot_state_publisher
        # to publish it - no ordering dependency needed here.
        gz_spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=['-topic', 'robot_description', '-name', 'lekiwi', '-allow_renaming', 'true'],
        )
        actions += [gz_launch_description, gz_sim_bridge, gz_spawn_entity]
    else:
        laser = include(pkg_bringup, 'launch/laser.launch.py', {'payload': payload})
        actions.append(laser)
        # lekiwi_audio isn't committed to this repo yet (local-only, not ready to publish -
        # same reason CI excludes it), so it won't exist on a fresh clone. Skip it instead
        # of crashing the whole bringup over a missing optional package.
        try:
            pkg_audio = get_package_share_directory('lekiwi_audio')
            actions.append(include(pkg_audio, 'launch/audio.launch.py'))
        except PackageNotFoundError:
            print("[lekiwi.launch.py] lekiwi_audio not found - skipping audio (mic will not be available).")
        if payload == 'pantilt':
            pkg_pantilt = FindPackageShare('pt_bringup').perform(context)
            oakd = include(pkg_pantilt, 'launch/oakd.launch.py', {
                'pointcloud': pointcloud,
                'octomap':    octomap,
            })
            actions.append(oakd)

    return actions


def generate_launch_description():
    """Declare all top-level LeKiwi launch arguments and launch via launch_setup."""
    declared_arguments = [
        DeclareLaunchArgument(
            'payload',
            default_value='pantilt',
            description='Hardware payload: "" for base only, "pantilt" for base + pan-tilt',
        ),
        DeclareLaunchArgument(
            'pantilt_config',
            default_value='pt101',
            description='Pan-tilt mesh variant when payload:="pt100" or "pt101"',
        ),
        DeclareLaunchArgument(
            'diagnostics',
            default_value='false',
            description='Launch motor and IMU diagnostics nodes in lekiwi_control.',
        ),
        DeclareLaunchArgument(
            'control_mock',
            default_value='',
            description='[advanced/debug] Mock mode override (true/false); empty means use urdf_config.yaml value.',
        ),
        DeclareLaunchArgument(
            'fusion_mode',
            default_value='base',
            description=(
                '[advanced] EKF sensor fusion mode: '
                'base = wheel odom + BNO055 (default); '
                'imu = BNO055 only (test platform); '
                'odom = wheel odom only (debug).'
            ),
        ),
        DeclareLaunchArgument(
            'pointcloud',
            default_value='false',
            description='[pantilt only] Enable RGBD point cloud output from OAK-D.',
        ),
        DeclareLaunchArgument(
            'octomap',
            default_value='false',
            description='[pantilt only, requires pointcloud:=true] Run octomap_server on the OAK-D point cloud.',
        ),
        DeclareLaunchArgument(
            'mission',
            default_value='',
            description=(
                'Navigation mission: empty preserves current default behavior '
                '(no map_name = map, map_name set = amcl); '
                'map = slam_toolbox mapping; '
                'slam = slam_toolbox localization on an existing map; '
                'amcl = AMCL on a static map.'
            ),
        ),
        DeclareLaunchArgument(
            'map_name',
            default_value='',
            description=(
                'Subdirectory name of the map to load for slam or amcl missions '
                '(e.g. livingroom1). Required for slam and amcl.'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock from a simulator instead of system time (forwarded to control & navigation).',
        ),
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Run against Gazebo instead of real hardware: starts gz_sim, uses '
                        'gz_ros2_control for the base, forces use_sim_time and control_mock, '
                        'and skips laser/audio/oakd (no simulated equivalent yet).',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='[sim only] Launch Gazebo with the GUI client attached.',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
