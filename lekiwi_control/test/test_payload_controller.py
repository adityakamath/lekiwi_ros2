"""The pan-tilt controller is spawned from pt_control's own parameter file, not merged into lekiwi's."""
from pathlib import Path

import yaml

SOURCE = Path(__file__).resolve().parents[1]
PAYLOAD = SOURCE.parent / 'payloads/pantilt_ros2/pt_control'


def test_the_payload_ships_its_controller_and_lekiwi_has_no_copy():
    config = yaml.safe_load((PAYLOAD / 'config/pantilt_controller.yaml').read_text())['pantilt_controller']['ros__parameters']
    assert config['type'] == 'forward_command_controller/ForwardCommandController'
    assert config['joints'] == ['shoulder_pan_joint', 'tilt_joint'] and config['interface_name'] == 'position'
    assert not (SOURCE / 'config/payloads/pantilt/control.yaml').exists()
    assert not (SOURCE / 'config/payloads/pantilt/urdf_config.yaml').exists()


def test_base_manager_config_does_not_define_the_payload_controller():
    manager = yaml.safe_load((SOURCE / 'config/base/control.yaml').read_text())
    assert 'pantilt_controller' not in manager and 'pantilt_controller' not in manager['controller_manager']['ros__parameters']
    assert 'joint_limits' not in manager['controller_manager']['ros__parameters']


def test_launch_spawns_the_pantilt_controller_with_the_payloads_file_and_loads_no_payload_overlay():
    source = (SOURCE / 'launch/control.launch.py').read_text()
    assert "'--param-file', f'{pkg_pt_control}/config/pantilt_controller.yaml'" in source
    assert "config/payloads/{payload}/control.yaml" not in source
    assert "config/payloads/{payload}/urdf_config.yaml" not in source


def test_launch_runs_the_same_depth_to_scan_slice_for_the_pantilt_in_simulation():
    source = (SOURCE / 'launch/control.launch.py').read_text()
    assert "executable='depthimage_to_laserscan_node'" in source
    assert "f'{pkg_pt_mujoco}/config/mujoco_depth_to_scan.yaml'" in source and "('scan', '/oak/scan')" in source
