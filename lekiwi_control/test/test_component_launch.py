"""One hardware file selects independent optional groups in URDF and simulation."""
import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


def _launch(overrides, tmp_path):
    path = Path(__file__).resolve().parents[1] / 'launch/control.launch.py'
    spec = importlib.util.spec_from_file_location('component_control_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    config = tmp_path / 'hardware.yaml'
    config.write_text(yaml.safe_dump(overrides))
    context.launch_configurations.update(use_mock='true', hardware_config=str(config))
    return module, context, config


@pytest.mark.parametrize('disabled', [None, 'lidar', 'imu', 'respeaker', 'sparkfun_sensor', 'gemini2', 'shelf', 'rpi5'])
def test_hardware_file_reaches_xacro(disabled, tmp_path, monkeypatch):
    module, context, config = _launch({disabled: False} if disabled else {}, tmp_path)
    commands = []
    original = module.Command

    def record(command):
        commands.append(command)
        return original(command)

    monkeypatch.setattr(module, 'Command', record)
    module.launch_setup(context)
    args = dict(part.split(':=', 1) for part in commands[0][0].split() if ':=' in part)
    assert args['hardware_config'] == str(config)
    assert args['enable_camera'] == ('false' if disabled == 'gemini2' else 'true')
    assert not {'audio', 'laser', 'tof', 'speaker', 'tray', 'payload', 'rpi5'} & args.keys()


@pytest.mark.parametrize('settings,expected', [
    ({'sparkfun_sensor': False}, {'lidar'}),
    ({'lidar': False}, {'tof'}),
    ({'lidar': False, 'sparkfun_sensor': False}, set()),
])
def test_simulation_uses_same_file_and_only_present_sensor_plugins(settings, expected, tmp_path, monkeypatch):
    module, context, config = _launch(settings, tmp_path)
    context.launch_configurations.update(ros2_control_hardware_type='mujoco')
    commands, nodes = [], []
    monkeypatch.setattr(module.subprocess, 'run', lambda command, **kwargs: commands.append(command))
    original = module.Node

    def record(**kwargs):
        nodes.append(kwargs)
        return original(**kwargs)

    monkeypatch.setattr(module, 'Node', record)
    module.launch_setup(context)
    command = commands[0]
    assert command[command.index('--hardware-config') + 1] == str(config)
    control = next(node for node in nodes if node['package'] == 'mujoco_ros2_control')
    plugin_file = next(p for p in control['parameters'] if isinstance(p, str) and 'lekiwi_sensors_' in p)
    plugins = yaml.safe_load(Path(plugin_file).read_text())['/**']['ros__parameters']['mujoco_plugins']
    plugin = plugins.get('mujoco_3d_lidar_plugin', {})
    assert {name for name in ('lidar', 'tof') if name in plugin} == expected
    names = {node.get('name') for node in nodes}
    assert ('laser_scan_filter_chain' in names) == ('lidar' in expected)
    assert ('tof_pointcloud' in names) == ('tof' in expected)




def test_profile_path_with_spaces_reaches_xacro(tmp_path, monkeypatch):
    import shlex
    import subprocess
    import xml.etree.ElementTree as ET
    module, context, config = _launch({'gemini2': False}, tmp_path)
    spaced = tmp_path / 'robot hardware profile.yaml'
    config.rename(spaced)
    context.launch_configurations['hardware_config'] = str(spaced)
    commands = []
    original = module.Command

    def record(command):
        commands.append(command)
        return original(command)

    monkeypatch.setattr(module, 'Command', record)
    module.launch_setup(context)
    args = shlex.split(commands[0][0])
    assert f'hardware_config:={spaced}' in args
    result = subprocess.run(args, capture_output=True, text=True, check=True)
    assert ET.fromstring(result.stdout).find("link[@name='gemini2_link']") is None


@pytest.mark.parametrize('lidar', [True, False])
def test_without_a_lidar_commands_go_straight_to_the_base(lidar, tmp_path, monkeypatch):
    module, context, _ = _launch({'lidar': lidar}, tmp_path)
    nodes = []
    original = module.Node

    def record(**kwargs):
        nodes.append(kwargs)
        return original(**kwargs)

    monkeypatch.setattr(module, 'Node', record)
    module.launch_setup(context)
    support = next(node for node in nodes if node.get('executable') == 'control_support_node')
    overrides = [p for p in support['parameters'] if isinstance(p, dict) and 'output_topic' in p]
    assert overrides == ([] if lidar else [{'output_topic': 'base_controller/cmd_vel'}])


@pytest.mark.parametrize('imu', [True, False])
def test_imu_broadcaster_and_diagnostics_follow_the_imu_setting(imu, tmp_path, monkeypatch):
    module, context, _ = _launch({'imu': imu}, tmp_path)
    context.launch_configurations.update(diagnostics='true')
    nodes = []
    original = module.Node

    def record(**kwargs):
        nodes.append(kwargs)
        return original(**kwargs)

    monkeypatch.setattr(module, 'Node', record)
    module.launch_setup(context)
    spawned = {node['arguments'][0] for node in nodes if node.get('executable') == 'spawner'}
    assert ('imu_sensor_broadcaster' in spawned) == imu
    assert 'base_controller' in spawned
    assert any(node.get('name') == 'bno055_diagnostics' for node in nodes) == imu


def test_simulation_does_not_start_physical_diagnostic_nodes(tmp_path, monkeypatch):
    module, context, _ = _launch({}, tmp_path)
    context.launch_configurations.update(diagnostics='true', ros2_control_hardware_type='mujoco',
                                         mujoco_model='/unused/model.xml')
    nodes = []
    original = module.Node

    def record(**kwargs):
        nodes.append(kwargs)
        return original(**kwargs)

    monkeypatch.setattr(module, 'Node', record)
    actions = module.launch_setup(context)
    assert not any(node.get('name') == 'bno055_diagnostics' for node in nodes)
    from launch.actions import TimerAction
    assert not any(isinstance(action, TimerAction) for action in actions)




@pytest.mark.parametrize('namespace,prefix', [('', ''), ('robot1', 'robot1/')])
def test_model_builder_receives_the_frame_prefix(namespace, prefix, tmp_path, monkeypatch):
    module, context, _ = _launch({}, tmp_path)
    context.launch_configurations.update(ros2_control_hardware_type='mujoco', namespace=namespace)
    commands = []
    monkeypatch.setattr(module.subprocess, 'run', lambda command, **kwargs: commands.append(command))
    module.launch_setup(context)
    command = commands[0]
    assert command[command.index('--frame-prefix') + 1] == prefix
