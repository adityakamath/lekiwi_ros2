"""Simulation camera options start only their selected processing nodes."""
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
import pytest
import yaml


@pytest.mark.parametrize('enabled,cloud', [
    ('true', 'false'), ('true', 'true'), ('false', 'true'),
])
def test_sim_camera_pipeline(enabled, cloud, monkeypatch, tmp_path):
    path = Path(__file__).resolve().parents[1] / 'launch/control.launch.py'
    spec = importlib.util.spec_from_file_location('sim_camera_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(enable_camera=enabled, pointcloud=cloud,
                                         ros2_control_hardware_type='mujoco',
                                         mujoco_model='/unused/model.xml')
    config = tmp_path / 'hardware.yaml'
    config.write_text('shelf: true\n')
    context.launch_configurations['hardware_config'] = str(config)
    nodes = []
    original = module.Node

    def record(**kwargs):
        nodes.append(kwargs)
        return original(**kwargs)

    monkeypatch.setattr(module, 'Node', record)
    module.launch_setup(context)
    names = {n.get('name') for n in nodes}
    imu_name = 'gemini2'
    imu_spawners = [n for n in nodes if n.get('package') == 'controller_manager'
                    and f'{imu_name}_imu_broadcaster' in n.get('arguments', [])]
    assert bool(imu_spawners) == (enabled == 'true')
    if imu_spawners:
        assert any(str(arg).endswith(f'{imu_name}_imu_broadcaster.yaml')
                   for arg in imu_spawners[0]['arguments'])
    remaps = [v for n in nodes for _, v in n.get('remappings', [])]
    control = next(n for n in nodes if n['package'] == 'mujoco_ros2_control')
    parameters = [str(p) for p in control['parameters']]
    assert any(p.endswith(f'{imu_name}_imu_broadcaster.yaml') for p in parameters) == (enabled == 'true')
    assert any(p.endswith('mujoco_camera_plugin.yaml') for p in parameters) == (enabled == 'true')
    assert not any(isinstance(p, dict) and 'mujoco_plugins' in p for p in control['parameters'])  # Rates come from plugin YAML.
    if enabled == 'false':
        assert not any('camera_gemini2.yaml' in p for p in parameters)
        assert not any(name and 'gemini2' in name for name in names)
        return
    assert 'gemini2_rgb_compressor' in names
    assert 'gemini2_mount_tf' not in names
    assert {'gemini2_depth'} <= names
    assert '/_gemini2/depth_raw' not in remaps  # It is configured on the camera plugin.
    assert any(p.endswith('mujoco_camera_gemini2.yaml') for p in parameters)
    assert ('gemini2_colored_pointcloud' in names) == (cloud == 'true')
    assert ('gemini2_cloud' in names) == (cloud == 'true')
    assert ('/_gemini2/registered_points' in remaps) == (cloud == 'true')
    assert 'gemini2_pointcloud' not in names
    assert 'gemini2_pointcloud_to_scan' not in names


@pytest.mark.parametrize('hw_type,diagnostics,expected_joints', [
    ('real', 'true', ['left_wheel_joint', 'back_wheel_joint', 'right_wheel_joint']),
    ('real', 'false', None), ('mujoco', 'true', None),
])
def test_state_interfaces_broadcaster_for_motor_diagnostics(
        hw_type, diagnostics, expected_joints, monkeypatch, tmp_path):
    path = Path(__file__).resolve().parents[1] / 'launch/control.launch.py'
    spec = importlib.util.spec_from_file_location('state_ifaces_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(diagnostics=diagnostics,
                                         enable_camera='false',
                                         ros2_control_hardware_type=hw_type,
                                         mujoco_model='/unused/model.xml')
    config = tmp_path / 'hardware.yaml'
    config.write_text('shelf: true\n')
    context.launch_configurations['hardware_config'] = str(config)
    nodes = []
    original = module.Node

    def record(**kwargs):
        nodes.append(kwargs)
        return original(**kwargs)

    monkeypatch.setattr(module, 'Node', record)
    module.launch_setup(context)
    spawners = [n for n in nodes if n.get('package') == 'controller_manager'
                and 'state_interfaces_broadcaster' in n.get('arguments', [])]
    if expected_joints is None:
        assert not spawners
        return
    assert len(spawners) == 1
    args = spawners[0]['arguments']
    params = yaml.safe_load(open(args[args.index('--param-file') + 1]))
    interfaces = params['/**/state_interfaces_broadcaster']['ros__parameters']['interfaces']
    assert interfaces == [f'{j}/{i}' for j in expected_joints
                          for i in ('effort', 'voltage', 'temperature', 'current', 'is_moving')]
