"""Config validation prevents typos and quoted false values from enabling devices."""
import pytest

from lekiwi_mujoco.hardware_config import HARDWARE_DEFAULTS, load_hardware_config


def test_default_hardware_file_preserves_all_groups():
    assert load_hardware_config() == HARDWARE_DEFAULTS.copy()


def test_partial_profile_keeps_groups_independent(tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text('respeaker: false\n')
    expected = HARDWARE_DEFAULTS.copy()
    expected['respeaker'] = False
    assert load_hardware_config(path) == expected


@pytest.mark.parametrize('contents', ['lidar: false', 'imu: false', 'battery_monitor: false', 'speaker: false', 'respeaker: "false"',
                                     'shelf: 0', 'imu: 0', 'battery_monitor: \"false\"', '- lidar', ''])
def test_invalid_hardware_config_is_rejected(tmp_path, contents):
    path = tmp_path / 'hardware.yaml'
    path.write_text(contents)
    with pytest.raises(ValueError):
        load_hardware_config(path)


def test_builder_uses_hardware_file_without_a_model_selector(tmp_path, monkeypatch):
    from lekiwi_mujoco import build_mujoco_models as builder
    config = tmp_path / 'hardware.yaml'
    config.write_text('shelf: true\n')
    output = tmp_path / 'robot.xml'
    calls = []
    monkeypatch.setattr(builder, 'build', lambda *args, **kwargs: calls.append((args, kwargs)))
    monkeypatch.setattr('sys.argv', ['build_mujoco_models', '--hardware-config', str(config), '--output', str(output)])
    builder.main()
    assert calls[0][0][0] == output
    assert calls[0][1]['hardware_config'] == config
