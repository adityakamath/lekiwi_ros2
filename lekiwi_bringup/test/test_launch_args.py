#!/usr/bin/env python3
"""
Smoke tests for lekiwi_bringup launch argument surfaces.

CLI smoke checks cover argument declarations; validation and hardware routing
are inspected directly through launch setup functions.

These tests do NOT start any nodes - they only validate the launch surface.
"""

import subprocess

import pytest


def _show_arguments(package, launch_file, extra_args=None):
    """Run `ros2 launch --show-arguments` and return stdout + returncode."""
    cmd = ['ros2', 'launch', '--show-arguments', package, launch_file]
    if extra_args:
        cmd += extra_args
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    return result.stdout + result.stderr, result.returncode


def _validate_setup(package, filename, overrides):
    """Call validation directly; ros2 launch --noninteractive is not a dry-run."""
    import importlib.util
    from pathlib import Path
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument

    path = Path(__file__).resolve().parents[2] / package / 'launch' / filename
    spec = importlib.util.spec_from_file_location(package + filename, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(overrides)
    return module.launch_setup(context)


# ── lekiwi.launch.py argument surface ────────────────────────────────────────

class TestLekiwiBringupArgs:
    """Checks that the top-level launch declares all expected arguments."""

    EXPECTED_ARGS = [
        'enable_camera', 'diagnostics', 'use_mock', 'joy',
        'sts_serial_port', 'mujoco_model',
        'fusion_mode', 'hardware_config', 'pointcloud',
        'mission', 'map_name', 'wp_loops', 'use_sim_time',
    ]

    def test_expected_args_declared(self):
        output, _ = _show_arguments('lekiwi_bringup', 'lekiwi.launch.py')
        for arg in self.EXPECTED_ARGS:
            assert arg in output, f"Expected argument '{arg}' not found in lekiwi.launch.py"


# ── slam.launch.py argument surface ──────────────────────────────────────────

class TestSlamLaunchArgs:
    """Checks that slam.launch.py declares all expected arguments."""

    EXPECTED_ARGS = ['mission', 'map_name', 'use_sim_time']

    def test_expected_args_declared(self):
        output, _ = _show_arguments('lekiwi_navigation', 'slam.launch.py')
        for arg in self.EXPECTED_ARGS:
            assert arg in output, f"Expected argument '{arg}' not found in slam.launch.py"


@pytest.mark.parametrize('package,filename,overrides,error', [
    ('lekiwi_bringup', 'lekiwi.launch.py', {'diagnostics': 'yes'}, 'diagnostics'),
    ('lekiwi_navigation', 'slam.launch.py', {'mission': 'invalid'}, 'mission'),
    ('lekiwi_navigation', 'slam.launch.py', {'mission': 'slam'}, 'map_name'),
    ('lekiwi_navigation', 'slam.launch.py', {'mission': 'amcl'}, 'map_name'),
    ('lekiwi_navigation', 'slam.launch.py', {'use_sim_time': 'notabool'}, 'use_sim_time'),
    ('lekiwi_navigation', 'nav2.launch.py', {'log_level': 'verbose'}, 'log_level'),
    ('lekiwi_navigation', 'nav2.launch.py', {'diagnostics': 'yes'}, 'diagnostics'),
    ('lekiwi_navigation', 'nav2.launch.py', {'autostart': 'yes'}, 'autostart'),
])
def test_invalid_launch_values_are_rejected(package, filename, overrides, error):
    with pytest.raises(RuntimeError, match=error):
        _validate_setup(package, filename, overrides)


# ── laser.launch.py argument surface ─────────────────────────────────────────

class TestLaserLaunchArgs:
    """Checks that laser.launch.py declares all expected arguments."""

    def test_expected_args_declared(self):
        output, _ = _show_arguments('lekiwi_bringup', 'laser.launch.py')
        for arg in ('custom_filter',):
            assert arg in output, f"Expected argument '{arg}' not found in laser.launch.py"


@pytest.mark.parametrize('enabled,sim', [('true', 'false'), ('false', 'false'), ('true', 'true')])
def test_camera_enable_is_forwarded_and_driver_is_lazy(enabled, sim, monkeypatch):
    import importlib.util
    from pathlib import Path
    from launch import LaunchContext, LaunchDescription
    from launch.actions import DeclareLaunchArgument
    from launch_ros.substitutions import FindPackageShare

    path = Path(__file__).resolve().parents[1] / 'launch/lekiwi.launch.py'
    spec = importlib.util.spec_from_file_location('lekiwi_camera_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert context.launch_configurations['enable_camera'] == 'true'
    context.launch_configurations.update(enable_camera=enabled, sim=sim, diagnostics='false')
    lookups = []

    def find(self, package):
        lookups.append(package)
        assert package != 'depthai_ros_driver'
        return '/unused/' + package

    monkeypatch.setattr(FindPackageShare, 'find', find)
    monkeypatch.setattr(module, 'load_hardware_config', lambda path: dict(
        sparkfun_sensor=False, respeaker=False, gemini2=True, shelf=True,
        lidar=True, imu=True, battery_monitor=True))
    actions = module.launch_setup(context)
    control_args = dict(actions[0].launch_arguments)
    assert control_args['enable_camera'] == enabled
    assert control_args['pointcloud'] == 'false'
    streaming = sim == 'false'
    assert ('lekiwi_bringup' in lookups) == streaming
    assert len(actions) == (5 if streaming else 2)
    if streaming:
        assert dict(actions[-1].launch_arguments)['enable_camera'] == enabled
        source = actions[-1].launch_description_source
        monkeypatch.setattr(source, '_get_launch_description', lambda path: LaunchDescription())
        source.get_launch_description(context)
        assert source.location.endswith('gemini2.launch.py')


def test_base_uses_its_own_camera_launch(monkeypatch):
    """The recommended base + static Gemini 2 starts lekiwi_bringup's driver launch and uses the local camera wrapper."""
    import importlib.util
    from pathlib import Path
    from launch import LaunchContext, LaunchDescription
    from launch.actions import DeclareLaunchArgument
    from launch_ros.substitutions import FindPackageShare

    path = Path(__file__).resolve().parents[1] / 'launch/lekiwi.launch.py'
    spec = importlib.util.spec_from_file_location('lekiwi_base_camera_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(sim='false', diagnostics='false')
    lookups = []

    def find(self, package):
        lookups.append(package)
        return '/unused/' + package

    monkeypatch.setattr(FindPackageShare, 'find', find)
    monkeypatch.setattr(module, 'load_hardware_config', lambda path: dict(
        sparkfun_sensor=False, respeaker=False, gemini2=True, shelf=True,
        lidar=True, imu=True, battery_monitor=True))
    actions = module.launch_setup(context)
    source = actions[-1].launch_description_source
    monkeypatch.setattr(source, '_get_launch_description', lambda path: LaunchDescription())
    source.get_launch_description(context)
    assert source.location == '/unused/lekiwi_bringup/launch/gemini2.launch.py'


def test_gemini2_cloudini_compressor_only_with_pointcloud(monkeypatch):
    """The Cloudini compressor container starts only when the colored point cloud is enabled."""
    import importlib.util
    from pathlib import Path
    from launch import LaunchContext
    from launch_ros.actions import ComposableNodeContainer
    from launch_ros.substitutions import FindPackageShare

    path = Path(__file__).resolve().parents[1] / 'launch/gemini2.launch.py'
    spec = importlib.util.spec_from_file_location('lekiwi_gemini2_pointcloud_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(FindPackageShare, 'find', lambda self, package: str(Path(__file__).resolve().parents[1])
                        if package == 'lekiwi_bringup' else '/unused/' + package)

    def setup(pointcloud):
        context = LaunchContext()
        context.launch_configurations.update(enable_camera='true', pointcloud=pointcloud,
                                             pointcloud_decimation='2', serial_number='', usb_port='')
        return module.launch_setup(context)

    def containers(pointcloud):
        return [a for a in setup(pointcloud) if isinstance(a, ComposableNodeContainer)]

    assert containers('false') == []
    assert len(containers('true')) == 1
    # The decimation parameter must be set before the driver include to take effect
    actions = setup('true')
    names = [type(a).__name__ for a in actions]
    assert names.index('SetParameter') < names.index('IncludeLaunchDescription')


@pytest.mark.parametrize('disabled,absent_includes,retained_includes', [
    ('sparkfun_sensor', {'tof.launch.py'}, {'laser.launch.py', 'audio.launch.py'}),
    ('respeaker', {'audio.launch.py'}, {'laser.launch.py', 'tof.launch.py'}),
    ('gemini2', {'gemini2.launch.py'}, {'laser.launch.py', 'tof.launch.py'}),
    ('shelf', set(), {'laser.launch.py', 'tof.launch.py', 'audio.launch.py'}),
])
def test_hardware_config_controls_independent_driver_includes(disabled, absent_includes, retained_includes, tmp_path, monkeypatch):
    import importlib.util
    from pathlib import Path
    import yaml
    from launch import LaunchContext, LaunchDescription
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch_ros.substitutions import FindPackageShare

    path = Path(__file__).resolve().parents[1] / 'launch/lekiwi.launch.py'
    spec = importlib.util.spec_from_file_location('lekiwi_components_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    config = tmp_path / 'hardware.yaml'
    config.write_text(yaml.safe_dump({disabled: False}))
    context.launch_configurations['hardware_config'] = str(config)
    monkeypatch.setattr(FindPackageShare, 'find', lambda self, package: '/unused/' + package)
    actions = module.launch_setup(context)
    args = dict(actions[0].launch_arguments)
    assert args['hardware_config'] == str(config)
    assert not {'lidar', 'laser', 'tof', 'audio', 'respeaker', 'speaker', 'gemini2', 'shelf'} & args.keys()
    includes = set()
    from launch.actions import TimerAction
    flat = [a for action in actions
            for a in (action.actions if isinstance(action, TimerAction) else [action])]
    for action in flat:
        if isinstance(action, IncludeLaunchDescription):
            source = action.launch_description_source
            monkeypatch.setattr(source, '_get_launch_description', lambda path: LaunchDescription())
            source.get_launch_description(context)
            includes.add(Path(source.location).name)
    assert not absent_includes & includes
    assert retained_includes <= includes


def test_robot_composition_and_fusion_come_only_from_hardware_file( tmp_path, monkeypatch):
    import importlib.util
    from pathlib import Path
    import yaml
    from launch import LaunchContext, LaunchDescription
    from launch.actions import DeclareLaunchArgument
    from launch_ros.substitutions import FindPackageShare

    path = Path(__file__).resolve().parents[1] / 'launch/lekiwi.launch.py'
    spec = importlib.util.spec_from_file_location('hardware_composition_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    declared = []
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            declared.append(action.name)
            action.execute(context)
    assert not {'imu', 'battery_monitor', 'respeaker', 'lidar', 'sparkfun_sensor',
                'gemini2', 'shelf', 'rpi5', 'speaker', 'tray', 'payload'} & set(declared)
    config = tmp_path / 'hardware.yaml'
    config.write_text(yaml.safe_dump({'respeaker': False, 'sparkfun_sensor': False}))
    context.launch_configurations['hardware_config'] = str(config)
    monkeypatch.setattr(FindPackageShare, 'find', lambda self, package: '/unused/' + package)
    actions = module.launch_setup(context)
    assert dict(actions[0].launch_arguments)['hardware_config'] == str(config)
    assert dict(actions[1].launch_arguments)['fusion_mode'] == 'base'
    assert len(actions) == 5  # Control, navigation, default LiDAR/battery, selected camera.
    source = actions[-1].launch_description_source
    monkeypatch.setattr(source, '_get_launch_description', lambda path: LaunchDescription())
    source.get_launch_description(context)
    assert 'gemini2.launch.py' in source.location


@pytest.mark.parametrize('sim', [True, False])
def test_permanent_sensors_and_navigation_launch(sim, tmp_path, monkeypatch):
    import importlib.util
    from pathlib import Path
    import yaml
    from launch import LaunchContext, LaunchDescription
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch_ros.substitutions import FindPackageShare

    path = Path(__file__).resolve().parents[1] / 'launch/lekiwi.launch.py'
    spec = importlib.util.spec_from_file_location('sensor_fallback_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    config = tmp_path / 'hardware.yaml'
    config.write_text(yaml.safe_dump(dict(respeaker=False, sparkfun_sensor=False, gemini2=False)))
    # A debug override must not request a sensor that is not fitted.
    context.launch_configurations.update(hardware_config=str(config), sim=str(sim).lower(),
                                         fusion_mode='imu')
    lookups = []

    def find(self, package):
        lookups.append(package)
        return '/unused/' + package

    monkeypatch.setattr(FindPackageShare, 'find', find)
    actions = module.launch_setup(context)
    includes = {}
    for action in actions:
        if isinstance(action, IncludeLaunchDescription):
            source = action.launch_description_source
            monkeypatch.setattr(source, '_get_launch_description', lambda path: LaunchDescription())
            source.get_launch_description(context)
            includes[Path(source.location).name] = dict(action.launch_arguments)
    nav_name = 'navigation.launch.py'
    assert nav_name in includes
    assert 'navigation.launch.py' in includes
    assert 'ekf.launch.py' not in includes
    expected_fusion = 'imu'
    assert includes[nav_name]['fusion_mode'] == expected_fusion
    assert includes['control.launch.py']['enable_odom_tf'] == 'false'
    assert ('laser.launch.py' in includes) == (not sim)
    assert ('battery_monitor.launch.py' in includes) == (not sim)
    assert ('ina260_battery_monitor' in lookups) == (not sim)


def test_camera_fps_is_not_a_launch_argument_and_real_rates_come_from_yaml(monkeypatch):
    import importlib.util
    from pathlib import Path
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch_ros.substitutions import FindPackageShare

    root = Path(__file__).resolve().parents[2]
    for package, filename in [('lekiwi_bringup', 'lekiwi.launch.py'),
                              ('lekiwi_control', 'control.launch.py'),
                              ('lekiwi_bringup', 'gemini2.launch.py')]:
        path = root / package / 'launch' / filename
        spec = importlib.util.spec_from_file_location(package + filename, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        declarations = [a for a in module.generate_launch_description().entities
                        if isinstance(a, DeclareLaunchArgument)]
        assert 'camera_fps' not in {a.name for a in declarations}
    context = LaunchContext()
    for declaration in declarations:
        declaration.execute(context)
    monkeypatch.setattr(FindPackageShare, 'find', lambda self, package: str(Path(__file__).resolve().parents[1])
                        if package == 'lekiwi_bringup' else '/unused/' + package)
    driver = next(a for a in module.launch_setup(context) if isinstance(a, IncludeLaunchDescription))
    args = dict(driver.launch_arguments)
    assert 'camera_fps' not in args
    assert args['color_fps'] == '15'
    assert args['depth_fps'] == '5'


def test_simulated_camera_rates_are_configured_in_yaml():
    from pathlib import Path
    import yaml
    config = Path(__file__).resolve().parents[2] / 'lekiwi_mujoco/config'
    for filename in ('mujoco_camera_plugin.yaml',):
        values = yaml.safe_load((config / filename).read_text())
        assert values['/**']['ros__parameters']['mujoco_plugins']['mujoco_camera_plugin']['camera_publish_rate'] == 5.0


@pytest.mark.parametrize('settings,sim,expected', [
    ({}, False, {'motors', 'imu', 'battery', 'patrol'}),
    ({}, True, {'patrol'}),
])
def test_diagnostic_analyzers_follow_enabled_sources(settings, sim, expected, tmp_path, monkeypatch):
    import importlib.util
    from pathlib import Path
    import yaml
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument

    path = Path(__file__).resolve().parents[1] / 'launch/lekiwi.launch.py'
    spec = importlib.util.spec_from_file_location('diagnostic_source_audit', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    config = tmp_path / 'hardware.yaml'
    config.write_text(yaml.safe_dump(settings))
    context.launch_configurations.update(hardware_config=str(config), diagnostics='true', sim=str(sim).lower())
    nodes = []
    original = module.Node

    def record(**kwargs):
        nodes.append(kwargs)
        return original(**kwargs)

    monkeypatch.setattr(module, 'Node', record)
    module.launch_setup(context)
    analyzers = [node for node in nodes if node.get('name') == 'analyzers']
    assert bool(analyzers) == bool(expected)
    if analyzers:
        params = yaml.safe_load(Path(analyzers[0]['parameters'][0]).read_text())['analyzers']['ros__parameters']
        assert set(params) - {'path'} == expected


def test_orbbec_component_receives_configured_camera_rates():
    """Resolve the installed upstream launch without starting a container or camera."""
    import importlib.util
    from pathlib import Path
    import yaml
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch.utilities import perform_substitutions
    from launch_ros.actions import ComposableNodeContainer
    from launch_ros.utilities import evaluate_parameters

    try:
        get_package_share_directory('orbbec_camera')
    except PackageNotFoundError:
        pytest.skip('Orbbec driver is not installed')
    package = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location('resolved_camera_launch', package / 'launch/gemini2.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    include = next(a for a in module.launch_setup(context) if isinstance(a, IncludeLaunchDescription))
    for name, value in include.launch_arguments:
        context.launch_configurations[name] = (perform_substitutions(context, value)
                                                    if isinstance(value, list) else str(value))
    upstream = include.launch_description_source.get_launch_description(context)
    for action in upstream.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)

    def components(actions):
        for action in actions:
            if isinstance(action, ComposableNodeContainer):
                yield from action._ComposableNodeContainer__composable_node_descriptions
            elif hasattr(action, 'get_sub_entities'):
                yield from components(action.get_sub_entities())

    camera = next(components(upstream.entities))
    parameters = {}
    for values in evaluate_parameters(context, camera.parameters):
        parameters.update(values)
    configured = yaml.safe_load((package / 'config/gemini2.yaml').read_text())[
        '/gemini2/gemini2']['ros__parameters']
    assert parameters['color_fps'] == configured['color_fps'] == 15
    assert parameters['depth_fps'] == configured['depth_fps'] == 5
    assert parameters['enable_colored_point_cloud'] is False
    assert parameters['enable_point_cloud'] is False
