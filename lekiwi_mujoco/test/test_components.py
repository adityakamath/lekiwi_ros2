"""Split MuJoCo assemblies must match the configurable URDF, including sensors."""
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest
import yaml

from lekiwi_mujoco import build_mujoco_models as builder

PACKAGE = Path(__file__).resolve().parents[1]
DESCRIPTION = PACKAGE.parent / 'lekiwi_description'
SPLIT_LINKS = ['rpi5_case_link', 'gemini2_mount_link',
               'respeaker_board_link', 'speaker_link', 'lidar_plate_link',
               'tray_link']


def test_split_visuals_follow_urdf_and_fit_mesh_loader():
    spec = builder.build_robot_spec()
    urdf, _ = builder.expand_urdf({'lekiwi_description': DESCRIPTION}, builder.control_package())
    for name in SPLIT_LINKS:
        joint = next(j for j in urdf.findall('joint') if j.find('child').get('link') == name)
        body = spec.body(name)
        assert body is not None
        assert np.allclose(body.pos, np.fromstring(joint.find('origin').get('xyz'), sep=' '))
        visual = urdf.find(f"link[@name='{name}']/visual")
        expected = type('Pose', (), {'alt': type('Orientation', (), {})()})()
        builder.set_origin(expected, visual.find('origin'))
        geom = next(iter(body.geoms))
        assert np.allclose(geom.pos, expected.pos) and np.allclose(geom.quat, expected.quat)
    assert Path(spec.mesh('lekiwi_base_visual').file).name == 'lekiwi_base.stl'
    assert Path(spec.mesh('tray_assembly_visual').file).name == 'tray_assembly.stl'
    for mesh in spec.meshes:
        path = Path(mesh.file)
        if path.suffix == '.stl':
            with path.open('rb') as stream:
                stream.seek(80)
                assert struct.unpack('<I', stream.read(4))[0] <= 200000
        assert path.name != 'lekiwi_base_mujoco.stl'
    spec.compile()


@pytest.mark.parametrize('components,removed,retained', [
    ({'rpi5': False}, ['rpi5_case_link'], ['base_link', 'tray_link']),
    ({'respeaker': False}, ['mic_link', 'respeaker_board_link',
                            'speaker_link'], ['laser_link', 'tof_link']),
    ({'shelf': False}, ['tray_link'], ['laser_link']),
    ({'gemini2': False},
     ['gemini2_link', 'gemini2_mount_link'], ['laser_frame', 'imu_link', 'imu_frame', 'rpi5_case_link']),
])
def test_disabled_assemblies_are_absent_and_model_steps(components, removed, retained, tmp_path):
    config = tmp_path / 'hardware.yaml'
    config.write_text(yaml.safe_dump(components))
    spec = builder.build_robot_spec(hardware_config=config)
    for name in removed:
        assert spec.body(name) is None
    for name in retained:
        assert spec.body(name) is not None
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data, nstep=20)
    assert np.isfinite(data.qpos).all()
    assert (spec.body('laser_link') is not None) is True
    assert (spec.body('imu_link') is not None) is True
    if components.get('gemini2') is False:
        assert not list(spec.cameras)
        assert not any(sensor.name.startswith('gemini2_') for sensor in spec.sensors)


@pytest.mark.parametrize('components,expected', [
    ({'sparkfun_sensor': False}, {'lidar'}), ({}, {'lidar', 'tof'}),
])
def test_native_plugins_follow_present_sensor_frames(tmp_path, components, expected):
    config = tmp_path / 'hardware.yaml'
    config.write_text(yaml.safe_dump(components))
    output = builder.build(tmp_path / 'model.xml', absolute=True, scene=False,
                           lidar='plugin', hardware_config=config)
    root = ET.parse(output).getroot()
    assert {node.get('name') for node in root.findall('sensor/plugin')} == expected


@pytest.mark.parametrize('lidar,tof,expected', [
    (True, True, True),
    (True, False, True),
    (False, True, True),
    (False, False, False),
])
def test_sensor_plate_follows_either_sensor(lidar, tof, expected, tmp_path):
    config = tmp_path / 'hardware.yaml'
    config.write_text(yaml.safe_dump({'lidar': lidar, 'sparkfun_sensor': tof}))
    spec = builder.build_robot_spec(hardware_config=config)
    urdf, _ = builder.expand_urdf({'lekiwi_description': DESCRIPTION}, builder.control_package(), config)
    assert (urdf.find("link[@name='lidar_plate_link']") is not None) is expected
    assert (spec.body('lidar_plate_link') is not None) is expected
    for name in ('ld06_lidar_plate_link', 'sparkfun_sensor_plate_link'):
        assert urdf.find(f"link[@name='{name}']") is None
        assert spec.body(name) is None


def test_disabled_lidar_removes_its_bodies_but_keeps_the_shared_plate(tmp_path):
    config = tmp_path / 'hardware.yaml'
    config.write_text(yaml.safe_dump({'lidar': False}))
    spec = builder.build_robot_spec(hardware_config=config)
    for name in ('laser_link', 'laser_frame'):
        assert spec.body(name) is None
    assert spec.body('lidar_plate_link') is not None
    assert spec.body('tof_link') is not None
    assert not [s.name for s in spec.sensors if s.name.startswith('lidar-')]
    spec.compile()


def test_disabled_imu_removes_its_body_and_sensors(tmp_path):
    config = tmp_path / 'hardware.yaml'
    config.write_text(yaml.safe_dump({'imu': False}))
    spec = builder.build_robot_spec(hardware_config=config)
    assert spec.body('imu_link') is None
    assert not [s.name for s in spec.sensors if s.name.startswith('bno055_')]
    assert spec.body('laser_link') is not None
    spec.compile()


def test_enabled_imu_keeps_its_body_and_sensors(tmp_path):
    config = tmp_path / 'hardware.yaml'
    config.write_text(yaml.safe_dump({'imu': True}))
    spec = builder.build_robot_spec(hardware_config=config)
    assert spec.body('imu_link') is not None
    assert [s.name for s in spec.sensors if s.name.startswith('bno055_')]


def test_disabling_all_optional_assemblies_keeps_only_the_base(tmp_path):
    from lekiwi_mujoco.hardware_config import HARDWARE_DEFAULTS
    config = tmp_path / 'hardware.yaml'
    config.write_text(yaml.safe_dump(dict.fromkeys(HARDWARE_DEFAULTS, False)))
    spec = builder.build_robot_spec(hardware_config=config)
    for name in ('lidar_plate_link', 'laser_link', 'laser_frame', 'imu_link', 'imu_frame', 'tof_link', 'mic_link', 'speaker_link', 'respeaker_board_link',
                 'gemini2_link', 'gemini2_mount_link', 'tray_link', 'rpi5_case_link'):
        assert spec.body(name) is None
    spec.compile()
