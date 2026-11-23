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
               'tray_link', 'shelf_spacers_link']


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
    assert Path(spec.mesh('shelf_tray_visual').file).name == 'shelf_tray.stl'
    assert Path(spec.mesh('shelf_spacers_35mm_visual').file).name == 'shelf_spacers_35mm.stl'
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
    ({'shelf': False}, ['tray_link', 'shelf_spacers_link'], ['laser_link']),
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
                 'gemini2_link', 'gemini2_mount_link', 'tray_link', 'shelf_spacers_link', 'rpi5_case_link'):
        assert spec.body(name) is None
    spec.compile()


@pytest.mark.parametrize('x,y', [(0.05, 0.0), (0.1, 0.0), (0.02, 0.1)])
def test_block_rests_on_the_tray_plate_inside_its_rim(x, y):
    import mujoco
    spec = builder.build_spec('flat')
    block = spec.worldbody.add_body(name='block')
    block.add_freejoint()
    block.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[.015, .015, .015], mass=.05)
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    base = model.body('base_link').id
    rotation = data.xmat[base].reshape(3, 3)
    start = model.jnt_qposadr[model.body_jntadr[model.body('block').id]]
    data.qpos[start:start + 3] = data.xpos[base] + rotation @ [x, y, 0.3]
    for _ in range(2000):
        mujoco.mj_step(model, data)
    rest = rotation.T @ (data.xpos[model.body('block').id] - data.xpos[base])
    assert rest[2] == pytest.approx(0.0915 + 0.015, abs=2e-3)  # plate top plus half the block


def test_tray_collision_pieces_follow_the_stl_volume():
    from scipy.spatial import ConvexHull
    model = builder.build_spec('flat').compile()
    pieces = [geom for geom in range(model.ngeom) if (model.geom(geom).name or '').startswith('tray_link_collision')]
    assert pieces
    volume = 0
    for geom in pieces:
        mesh = model.geom_dataid[geom]
        start = model.mesh_vertadr[mesh]
        volume += ConvexHull(model.mesh_vert[start:start + model.mesh_vertnum[mesh]]).volume
    triangles = builder.stl_triangles(DESCRIPTION / 'meshes/shelf_tray.stl')
    stl_volume = abs(np.einsum('ij,ij->i', triangles[:, 0], np.cross(triangles[:, 1], triangles[:, 2])).sum() / 6)
    assert stl_volume <= volume <= 1.15 * stl_volume  # Hulls only add material, within the cell chord error.
