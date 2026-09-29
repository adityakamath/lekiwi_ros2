"""Simulation camera options start only their selected processing nodes."""
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
import pytest


@pytest.mark.parametrize('camera', ['gemini2', 'oakd_s2'])
@pytest.mark.parametrize('enabled,cloud', [
    ('true', 'false'), ('true', 'true'), ('false', 'true'),
])
def test_sim_camera_pipeline(camera, enabled, cloud, monkeypatch):
    path = Path(__file__).resolve().parents[1] / 'launch/control.launch.py'
    spec = importlib.util.spec_from_file_location('sim_camera_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(payload='pantilt', camera_config=camera,
                                         enable_camera=enabled, pointcloud=cloud,
                                         ros2_control_hardware_type='mujoco',
                                         mujoco_model='/unused/model.xml')
    nodes = []
    original = module.Node

    def record(**kwargs):
        nodes.append(kwargs)
        return original(**kwargs)

    monkeypatch.setattr(module.FindPackageShare, 'find',
                        lambda self, package: str(Path(__file__).resolve().parents[2] /
                                                  ('payloads/pantilt_ros2' if package.startswith('pt_') else '') /
                                                  package))
    monkeypatch.setattr(module, 'Node', record)
    module.launch_setup(context)
    names = {n.get('name') for n in nodes}
    imu_name = 'gemini2' if camera == 'gemini2' else 'oak'
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
    assert any('pt_mujoco/config/mujoco_ros2_control_plugins.yaml' in p for p in parameters) == (enabled == 'true')
    assert any(p.endswith('mujoco_camera_pantilt.yaml') for p in parameters) == (enabled == 'true')
    assert any(isinstance(p, dict) and 'mujoco_plugins' in p for p in control['parameters']) == (enabled == 'true')
    if enabled == 'false':
        assert not any('camera_gemini2.yaml' in p for p in parameters)
        assert not any(name and ('gemini2' in name or 'oak_' in name or name == 'depth_to_scan')
                       for name in names)
        return
    assert ('gemini2_rgb_compressor' if camera == 'gemini2' else 'oak_rgb_compressor') in names
    if camera == 'gemini2':
        assert {'gemini2_depth', 'gemini2_depth_to_scan'} <= names
        assert '/gemini2/scan' in remaps
        assert '/_gemini2/depth_raw' not in remaps  # It is configured on the camera plugin.
        assert any(p.endswith('mujoco_camera_gemini2.yaml') for p in parameters)
        assert ('gemini2_colored_pointcloud' in names) == (cloud == 'true')
        assert ('gemini2_cloud' in names) == (cloud == 'true')
        assert ('/_gemini2/registered_points' in remaps) == (cloud == 'true')
        assert 'gemini2_pointcloud' not in names
        assert 'gemini2_pointcloud_to_scan' not in names
    else:
        assert 'depth_to_scan' in names
        assert not any(v.startswith('/gemini2/') for v in remaps)
        assert '/oak/scan' in remaps
        assert ('oak_colored_pointcloud' in names) == (cloud == 'true')
        assert ('oak_cloud' in names) == (cloud == 'true')
        assert ('/oak/rgbd/points' in remaps) == (cloud == 'true')
