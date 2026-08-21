#!/usr/bin/env python3
"""
Launch the LeKiwi robot system.

The 'payload' argument selects which hardware payload is present:
  ""        — base drive + navigation + laser [no pan-tilt, no OAK-D]
  "pantilt" — base + pan-tilt + OAK-D + navigation + laser [default]

The 'imu' argument (default true) selects whether a physical BNO055 IMU is present
on the base (real hardware and sim:=true/MuJoCo both honor it). imu:=false requires
fusion_mode:=odom, since base/imu fusion modes need the IMU.

The 'laser' argument (default true) selects whether a physical LD06 LiDAR is present on
the base. laser:=false skips laser.launch.py's include entirely.

The 'audio' argument (default true) selects whether a physical reSpeaker mic array is
present on the base. audio:=false skips lekiwi_audio's launch include entirely.

The 'battery_monitor' argument (default true) selects whether a physical INA260 current/
voltage sensor is present on the base. battery_monitor:=false skips ina260_ros2's launch
include entirely; if left true on a robot with no INA260 fitted, battery_monitor_node
itself warns and shuts down cleanly rather than crashing (see its module docstring).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_VALID_PAYLOADS = {'', 'pantilt'}


def _launch_arg_as_bool(context, name: str) -> bool:
    """Resolve a launch argument as a strict boolean."""
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ('true', '1'):
        return True
    if value in ('false', '0'):
        return False
    raise RuntimeError(
        f"[lekiwi.launch.py] Launch argument '{name}' must be true/false or 1/0, got {value!r}."
    )


def launch_setup(context):
    """Validate arguments and include control/navigation/laser (+ oakd for pantilt)."""
    payload             = LaunchConfiguration('payload').perform(context)
    pantilt_config      = LaunchConfiguration('pantilt_config').perform(context)
    diagnostics         = _launch_arg_as_bool(context, 'diagnostics')
    use_mock            = LaunchConfiguration('use_mock').perform(context)
    fusion_mode         = LaunchConfiguration('fusion_mode').perform(context)
    imu                 = _launch_arg_as_bool(context, 'imu')
    laser               = _launch_arg_as_bool(context, 'laser')
    audio               = _launch_arg_as_bool(context, 'audio')
    battery_monitor     = _launch_arg_as_bool(context, 'battery_monitor')
    map_name            = LaunchConfiguration('map_name').perform(context)
    pointcloud          = _launch_arg_as_bool(context, 'pointcloud')
    octomap             = _launch_arg_as_bool(context, 'octomap')
    mission             = LaunchConfiguration('mission').perform(context)
    use_sim_time        = LaunchConfiguration('use_sim_time').perform(context)
    sim                 = _launch_arg_as_bool(context, 'sim')
    gui                 = LaunchConfiguration('gui').perform(context)
    joy                 = LaunchConfiguration('joy').perform(context)
    sts_serial_port     = LaunchConfiguration('sts_serial_port').perform(context)
    mujoco_model        = LaunchConfiguration('mujoco_model').perform(context)

    if sim:
        # Running in MuJoCo implies sim time; keep any explicit use_mock override,
        # otherwise force it (control.launch.py's mujoco branch never opens the real serial
        # port, but this stays defensive/explicit rather than relying on that alone).
        use_sim_time = 'true'
        use_mock = use_mock or 'true'

    if payload not in _VALID_PAYLOADS:
        raise RuntimeError(
            f"[lekiwi.launch.py] Unknown payload '{payload}'. "
            f"Valid values: {sorted(_VALID_PAYLOADS)}"
        )
    if pointcloud and payload != 'pantilt':
        raise RuntimeError(
            "[lekiwi.launch.py] pointcloud:=true requires payload:=pantilt."
        )
    if octomap and not pointcloud:
        raise RuntimeError(
            "[lekiwi.launch.py] octomap:=true requires pointcloud:=true."
        )
    if not imu and fusion_mode in ('base', 'imu'):
        raise RuntimeError(
            f"[lekiwi.launch.py] imu:=false requires fusion_mode:=odom "
            f"(fusion_mode:={fusion_mode!r} needs the BNO055)."
        )

    pkg_control = FindPackageShare('lekiwi_control').perform(context)
    pkg_nav     = FindPackageShare('lekiwi_navigation').perform(context)

    def include(pkg, path, args=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource([f'{pkg}/{path}']),
            launch_arguments=(args or {}).items(),
        )

    control_args = {
        'payload':          payload,
        'pantilt_config':   pantilt_config,
        'diagnostics':      str(diagnostics).lower(),
        'use_mock':         use_mock,
        'use_sim_time':     use_sim_time,
        'imu':              str(imu).lower(),
        'joy':              joy,
        'sts_serial_port':  sts_serial_port,
        'mujoco_model':     mujoco_model,
    }
    if sim:
        control_args['ros2_control_hardware_type'] = 'mujoco'
        control_args['mujoco_headless'] = 'false' if gui.lower() in ('true', '1') else 'true'
    control = include(pkg_control, 'launch/control.launch.py', control_args)

    nav = include(pkg_nav,     'launch/navigation.launch.py', {
        'fusion_mode':  fusion_mode,
        'mission':      mission,
        'map_name':     map_name,
        'use_sim_time': use_sim_time,
        'diagnostics':  str(diagnostics).lower(),
    })

    actions = [control, nav]

    if diagnostics:
        pkg_bringup = FindPackageShare('lekiwi_bringup').perform(context)
        # name='analyzers' is required, not cosmetic - diagnostic_aggregator.yaml's own
        # top-level key is 'analyzers', and ROS 2 associates a YAML params file with a node
        # by matching that key to the node's name.
        actions.append(Node(
            package='diagnostic_aggregator',
            executable='aggregator_node',
            name='analyzers',
            output='log',
            parameters=[
                f'{pkg_bringup}/config/diagnostic_aggregator.yaml',
                {'use_sim_time': use_sim_time.lower() in ('true', '1')},
            ],
        ))

    if sim:
        # laser/audio/battery_monitor/oakd are real-sensor drivers with no simulated
        # equivalent (oakd, battery_monitor) or already-simulated equivalent hosted
        # directly by control.launch.py's mujoco control node (laser - see
        # base.control.xacro's laser_frame <sensor>, which publishes /scan itself), so
        # none of them are needed here.
        return actions

    pkg_bringup = FindPackageShare('lekiwi_bringup').perform(context)

    if laser:
        actions.append(include(pkg_bringup, 'launch/laser.launch.py', {'payload': payload}))

    if audio:
        pkg_audio = FindPackageShare('lekiwi_audio').perform(context)
        actions.append(include(pkg_audio, 'launch/audio.launch.py'))

    if battery_monitor:
        pkg_ina260 = FindPackageShare('ina260_ros2').perform(context)
        actions.append(include(pkg_ina260, 'launch/battery_monitor.launch.py', {
            'params_file': f'{pkg_bringup}/config/battery.yaml',
        }))

    if payload == 'pantilt':
        pkg_pantilt = FindPackageShare('pt_bringup').perform(context)
        oakd = include(pkg_pantilt, 'launch/oakd.launch.py', {
            'pointcloud': str(pointcloud).lower(),
            'octomap':    str(octomap).lower(),
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
            description='Pan-tilt mesh variant when payload:="pantilt": "pt100" or "pt101"',
        ),
        DeclareLaunchArgument(
            'diagnostics',
            default_value='false',
            description=(
                'Single flag for every /diagnostics publisher and consumer: launches motor '
                'and IMU diagnostics nodes in lekiwi_control, enables patrol status publishing '
                'in waypoint_recorder_node (lekiwi_navigation), and launches a '
                'diagnostic_aggregator node (see lekiwi_bringup/config/diagnostic_aggregator.yaml) '
                'that rolls all three up into one top-level status on /diagnostics_agg. '
                'false disables publishing from all of them, not just the aggregator.'
            ),
        ),
        DeclareLaunchArgument(
            'use_mock',
            default_value='',
            description='[advanced/debug] Mock mode override (true/false); empty means use urdf_config.yaml value.',
        ),
        DeclareLaunchArgument(
            'joy',
            default_value='false',
            description='Launch joy_node on this device. Set true when the joystick is connected locally; '
                        'leave false when /joy is published from a remote device.',
        ),
        DeclareLaunchArgument(
            'sts_serial_port',
            default_value='',
            description='[advanced/debug] Serial port override; empty string means use urdf_config.yaml value.',
        ),
        DeclareLaunchArgument(
            'mujoco_model',
            default_value='',
            description='[advanced, sim only] Path to a pre-built MJCF file to load; empty means '
                        'xacro-process it at launch time instead (picked by payload/pantilt_config).',
        ),
        DeclareLaunchArgument(
            'fusion_mode',
            default_value='base',
            description=(
                '[advanced] EKF sensor fusion mode: '
                'base = wheel odom + BNO055 (default); '
                'imu = BNO055 only (test platform); '
                'odom = wheel odom only (debug, required when imu:=false).'
            ),
        ),
        DeclareLaunchArgument(
            'imu',
            default_value='true',
            description='Whether a physical BNO055 IMU is present on the base (honored on '
                        'real hardware and sim:=true/MuJoCo). false requires '
                        'fusion_mode:=odom; omits the IMU from the URDF, the '
                        'imu_sensor_broadcaster controller, and bno055_diagnostics.',
        ),
        DeclareLaunchArgument(
            'laser',
            default_value='true',
            description='Whether a physical LD06 LiDAR is present on the base. '
                        'false skips laser.launch.py\'s include entirely.',
        ),
        DeclareLaunchArgument(
            'audio',
            default_value='true',
            description='Whether a physical reSpeaker mic array and speaker is present on the base. '
                        'false skips lekiwi_audio\'s launch include entirely.',
        ),
        DeclareLaunchArgument(
            'battery_monitor',
            default_value='true',
            description='Whether a physical INA260 current/voltage sensor is present on the base. '
                        'false skips ina260_ros2\'s launch include entirely; if left true with no '
                        'INA260 fitted, battery_monitor_node warns and shuts down cleanly instead '
                        'of crashing.',
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
            description='Run against MuJoCo instead of real hardware: forces use_sim_time and '
                        'use_mock, and skips laser/audio/battery_monitor/oakd (laser is hosted '
                        'by the mujoco control node itself; the other three have no simulated '
                        'equivalent at all). Base and pantilt payload both supported.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='[sim only] Launch with the MuJoCo Simulate viewer attached.',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
