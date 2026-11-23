"""Physics and portable model checks, runnable without ROS."""
import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

mujoco = pytest.importorskip('mujoco')
np = pytest.importorskip('numpy')
PACKAGE = Path(__file__).resolve().parents[1]
DESCRIPTION = PACKAGE.parent / 'lekiwi_description'
sys.path.insert(0, str(PACKAGE))
from lekiwi_mujoco.simulation import Simulation  # noqa: E402


def runtime_from_file(path, settle=True):
    runtime = Simulation(mujoco.MjModel.from_xml_path(str(path)))
    runtime.reset(.5 if settle else 0.)
    return runtime

MODEL_FILENAME = 'lekiwi_base.xml'


def test_model_settles_and_preserves_interfaces_and_wheel_mass():
    runtime = runtime_from_file(PACKAGE / 'mjcf' / MODEL_FILENAME)
    model, data = runtime.model, runtime.data
    assert np.isfinite(data.qpos).all()
    assert .038 < data.qpos[2] < .043
    assert np.linalg.norm(data.qvel[:6]) < .002
    assert model.nu == 3
    assert sum('roller_joint' in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) or '') for i in range(model.njnt)) == 48
    from lekiwi_mujoco.build_mujoco_models import package_paths
    import xacro
    source = 'base.urdf.xacro'
    with package_paths({'lekiwi_description': DESCRIPTION}):
        doc = xacro.process_file(str(DESCRIPTION / 'urdf' / source), mappings={'hardware_config': str(PACKAGE.parent / 'lekiwi_bringup/config/hardware.yaml'), 'base_controller_config': str(PACKAGE.parent / 'lekiwi_control/config/control.yaml'), 'use_mock': 'true'})
    urdf = ET.fromstring(doc.toxml())
    masses = {link.get('name'): float(link.find('inertial/mass').get('value'))
              for link in urdf.findall('link') if link.find('inertial/mass') is not None}
    assert model.body_mass.sum() == pytest.approx(sum(masses.values()))
    for prefix in ['left', 'back', 'right']:
        hub = model.body(prefix + '_wheel_link').id
        children = [i for i in range(model.nbody) if model.body_parentid[i] == hub]
        assert model.body_mass[hub] + model.body_mass[children].sum() == pytest.approx(masses[prefix + '_wheel_link'])
    assert model.sensor('lidar-180').id >= 0
    assert model.sensor('bno055_quat').id >= 0


@pytest.mark.parametrize('command,axis,target', [((.15, 0, 0), 0, .3), ((0, .15, 0), 1, .3), ((0, 0, .5), 2, 1.), ((-.15, 0, 0), 0, -.3)])
def test_measured_base_motion(command, axis, target):
    runtime = runtime_from_file(PACKAGE / 'mjcf' / MODEL_FILENAME)
    start = runtime.pose()
    runtime.command(command)
    runtime.step(1000)
    delta = runtime.pose() - start
    assert abs(delta[axis] - target) < .05
    assert np.max(np.abs(np.delete(delta, axis))) < .035


def test_wheel_command_speed_limit():
    runtime = runtime_from_file(PACKAGE / 'mjcf/lekiwi_base.xml')
    rates = runtime.control.wheel_speeds([100, 100, 100])
    assert np.all(abs(rates) <= runtime.control.wheel_limits + 1e-10)


def test_generated_assets_are_current_and_portable(tmp_path):
    pytest.importorskip('xacro')
    spec = importlib.util.spec_from_file_location('builder', PACKAGE / 'lekiwi_mujoco/build_mujoco_models.py')
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    output = builder.build(tmp_path / MODEL_FILENAME, absolute=True)
    expected = ET.parse(output).getroot()
    actual = ET.parse(PACKAGE / 'mjcf' / MODEL_FILENAME).getroot()
    for root, directory in [(expected, tmp_path), (actual, PACKAGE / 'mjcf')]:
        for mesh in root.iter('mesh'):
            if 'file' not in mesh.attrib:
                continue  # Generated collision pieces carry their vertices inline.
            mesh.set('file', str((directory / mesh.attrib['file']).resolve()))
        for node in root.iter():
            node.text = (node.text or '').strip()
            node.tail = ''
    assert ET.tostring(expected) == ET.tostring(actual)
    assert not actual.findall('.//include')
    mujoco.MjModel.from_xml_path(str(output))


def test_limits_regenerate_from_edited_configuration_and_urdf(tmp_path, monkeypatch):
    import shutil
    import yaml
    from lekiwi_mujoco import build_mujoco_models as builder
    description = tmp_path / 'lekiwi_description'
    shutil.copytree(DESCRIPTION / 'urdf', description / 'urdf')
    # Geometry tests edit Xacro, never mesh assets; avoid duplicating every STL.
    (description / 'meshes').symlink_to(DESCRIPTION / 'meshes', target_is_directory=True)
    simulation = tmp_path / 'lekiwi_mujoco'
    shutil.copytree(PACKAGE / 'config', simulation / 'config')
    (simulation / 'mjcf').symlink_to(PACKAGE / 'mjcf', target_is_directory=True)
    control = tmp_path / 'lekiwi_control/config'
    shutil.copytree(PACKAGE.parent / 'lekiwi_control/config', control)
    config = simulation / 'config/mujoco.yaml'
    config_data = yaml.safe_load(config.read_text())
    config_data['command']['base_velocity'] = [.08, .06, .25]
    config.write_text(yaml.safe_dump(config_data))
    motors = control / 'urdf_config.yaml'
    motor_data = yaml.safe_load(motors.read_text())
    motor_data['sts3215_max_vel_steps'] = 2000
    motors.write_text(yaml.safe_dump(motor_data))
    monkeypatch.setattr(builder, 'PACKAGE', description)
    monkeypatch.setattr(builder, 'SIM_PACKAGE', simulation)
    monkeypatch.setattr(builder, 'control_package', lambda: tmp_path / 'lekiwi_control')
    root = ET.parse(builder.build(tmp_path / 'first.xml')).getroot()
    assert root.find("custom/numeric[@name='base_velocity_limits']").get('data') == '0.08 0.06 0.25'
    expected = int(2000 * .85) * 2 * np.pi / 4096
    assert float(root.find("actuator/general[@name='left_wheel_joint']").get('ctrlrange').split()[1]) == pytest.approx(expected, abs=1e-5)
    module = description / 'urdf/base.module.xacro'
    module.write_text(module.read_text().replace('velocity="${max_velocity_rads}"', 'velocity="0.9"'))
    root = ET.parse(builder.build(tmp_path / 'second.xml')).getroot()
    assert root.find("actuator/general[@name='left_wheel_joint']").get('ctrlrange') == '-0.9 0.9'


@pytest.fixture
def parameter_workspace(tmp_path, monkeypatch):
    """Isolate source edits so tests exercise regeneration, not committed snapshots."""
    import shutil
    from lekiwi_mujoco import build_mujoco_models as builder
    description = tmp_path / 'lekiwi_description'
    for folder in ('urdf', 'meshes'):
        shutil.copytree(DESCRIPTION / folder, description / folder)
    simulation = tmp_path / 'lekiwi_mujoco'
    for folder in ('mjcf', 'config'):
        shutil.copytree(PACKAGE / folder, simulation / folder)
    shutil.copytree(PACKAGE.parent / 'lekiwi_control/config', tmp_path / 'lekiwi_control/config')
    monkeypatch.setattr(builder, 'PACKAGE', description)
    monkeypatch.setattr(builder, 'SIM_PACKAGE', simulation)
    monkeypatch.setattr(builder, 'control_package', lambda: tmp_path / 'lekiwi_control')
    return description


def test_robot_scene_boundary_and_native_composition(tmp_path):
    from lekiwi_mujoco.build_mujoco_models import build_robot_spec, compose_scene
    robot = build_robot_spec()
    model = robot.compile()
    assert robot.geom('floor') is None
    assert not list(robot.lights) and not list(robot.textures)
    assert robot.material('groundplane') is None
    world = compose_scene(robot, 'flat')
    combined = world.compile()
    assert world.geom('floor') is not None
    assert combined.ngeom == model.ngeom + 1
    assert combined.nu == model.nu
    assert combined.body_mass.sum() == pytest.approx(model.body_mass.sum())
    # Attachment must serialize to independently loadable XML, including defaults.
    path = tmp_path / 'composed.xml'
    path.write_text(world.to_xml())
    loaded = mujoco.MjModel.from_xml_path(str(path))
    assert loaded.actuator('left_wheel_joint').id >= 0


def test_custom_scene_resolves_its_own_assets(tmp_path):
    from lekiwi_mujoco.build_mujoco_models import build
    folder = tmp_path / 'scene'
    assets = folder / 'assets'
    assets.mkdir(parents=True)
    (assets / 'tetra.obj').write_text('v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n')
    scene = folder / 'world.xml'
    scene.write_text('<mujoco><compiler meshdir="assets"/><asset><mesh name="fixture" file="tetra.obj"/></asset>'
                     '<worldbody><geom name="fixture_geom" type="mesh" mesh="fixture" pos="3 0 0"/></worldbody></mujoco>')
    path = build(tmp_path / 'different_directory' / 'model.xml', scene=scene)
    model = mujoco.MjModel.from_xml_path(str(path))
    assert model.geom('fixture_geom').id >= 0
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'floor') == -1


def _inertia_about_body_origin(model, body, parent_rotation=None, position=None):
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, model.body_iquat[body])
    rotation = rotation.reshape(3, 3)
    center = model.body_ipos[body]
    if parent_rotation is not None:
        rotation = parent_rotation @ rotation
        center = position + parent_rotation @ center
    return (rotation @ np.diag(model.body_inertia[body]) @ rotation.T
            + model.body_mass[body] * (np.dot(center, center)*np.eye(3) - np.outer(center, center)))


def test_changed_geometry_and_inertia_propagate(parameter_workspace):
    import yaml
    from lekiwi_mujoco.build_mujoco_models import build_robot_spec
    path = parameter_workspace.parent / 'lekiwi_control/config/control.yaml'
    settings = yaml.safe_load(path.read_text())
    geometry = settings['/**/base_controller']['ros__parameters']
    geometry.update(wheel_radius=.0612, robot_radius=.15, wheel_offset=.9)
    path.write_text(yaml.safe_dump(settings))
    module = parameter_workspace / 'urdf/base.module.xacro'
    module.write_text(module.read_text().replace('<mass value="2.0"/>', '<mass value="2.7"/>'))
    robot = build_robot_spec()
    model = robot.compile()
    assert model.body_mass[model.body('base_link').id] == pytest.approx(2.7)
    assert robot.body('base_link').pos[2] == pytest.approx(.0612 - .01036)
    assert np.allclose(model.numeric('wheel_kinematics').data, [.0612, .15, .9])
    for i, prefix in enumerate(('left', 'back', 'right')):
        hub = model.body(prefix + '_wheel_link').id
        assert np.allclose(model.body_pos[hub, :2], .15*np.array([np.cos(.9+i*2*np.pi/3), np.sin(.9+i*2*np.pi/3)]))
        visual = next(g for g in robot.body(prefix + '_wheel_link').geoms if g.meshname)
        assert np.allclose(robot.mesh(visual.meshname).scale, [1.2]*3)
        geom = model.geom(prefix + '_roller_collision_0').id
        roller = model.body(prefix + '_roller_0').id
        assert model.body_pos[roller, 0] + model.geom_size[geom, 0] == pytest.approx(.0612)
        children = np.flatnonzero(model.body_parentid == hub)
        assert model.body_mass[hub] + model.body_mass[children].sum() == pytest.approx(.1)
        total = _inertia_about_body_origin(model, hub)
        for child in children:
            rotation = np.empty(9)
            mujoco.mju_quat2Mat(rotation, model.body_quat[child])
            total += _inertia_about_body_origin(model, child, rotation.reshape(3, 3), model.body_pos[child])
        assert np.allclose(total, np.diag([.001]*3), atol=1e-12)


def test_profile_and_urdf_effort_own_actuation(parameter_workspace):
    import yaml
    from lekiwi_mujoco.build_mujoco_models import build_robot_spec
    path = parameter_workspace.parent / 'lekiwi_mujoco/config/mujoco.yaml'
    settings = yaml.safe_load(path.read_text())
    settings['actuators']['wheel'].update(velocity_gain=7., armature=.2)
    settings['contact']['friction'] = [.8, .002, .00002]
    settings['physics']['timestep'] = .001
    path.write_text(yaml.safe_dump(settings))
    module = parameter_workspace / 'urdf/base.module.xacro'
    module.write_text(module.read_text().replace('effort="${max_torque_nm}"', 'effort="1.7"'))
    model = build_robot_spec().compile()
    assert model.opt.timestep == pytest.approx(.001)
    actuator = model.actuator('left_wheel_joint').id
    assert model.actuator_gainprm[actuator, 0] == 7.
    assert np.allclose(model.actuator_forcerange[actuator], [-1.7, 1.7])
    assert np.allclose(model.geom_friction[model.geom('left_roller_collision_0').id], [.8, .002, .00002])
    assert model.dof_armature[model.joint('left_wheel_joint').dofadr[0]] == .2


@pytest.mark.parametrize('failure', ['radius', 'layout', 'inertia', 'roller_mass'])
def test_inconsistent_parameters_fail_generation(parameter_workspace, failure):
    import yaml
    from lekiwi_mujoco.build_mujoco_models import build_robot_spec
    if failure == 'radius':
        path = parameter_workspace.parent / 'lekiwi_control/config/control.yaml'
        settings = yaml.safe_load(path.read_text())
        settings['/**/base_controller']['ros__parameters']['wheel_radius'] = -1
        path.write_text(yaml.safe_dump(settings))
    elif failure == 'roller_mass':
        path = parameter_workspace.parent / 'lekiwi_mujoco/config/mujoco.yaml'
        settings = yaml.safe_load(path.read_text())
        settings['contact']['roller_mass_fraction'] = .1
        path.write_text(yaml.safe_dump(settings))
    else:
        path = parameter_workspace / 'urdf/base.module.xacro'
        text = path.read_text()
        if failure == 'layout':
            text = text.replace('wheel_base_radius * cos(offset)', '0.4 * cos(offset)')
        else:
            text = text.replace('ixx="0.001"', 'ixx="0.1"')
        path.write_text(text)
    with pytest.raises(ValueError):
        build_robot_spec()


def test_urdf_mass_edits_replace_every_base_mass(parameter_workspace):
    import re
    from lekiwi_mujoco.build_mujoco_models import build_robot_spec
    before = build_robot_spec().compile()
    for module in (parameter_workspace / 'urdf').glob('*.module.xacro'):
        module.write_text(re.sub(r'<mass value="([0-9.]+)"/>',
                                 lambda match: f'<mass value="{float(match[1]) * 1.5}"/>', module.read_text()))
    after = build_robot_spec().compile()
    assert after.body_mass.sum() == pytest.approx(1.5 * before.body_mass.sum())
    for name in ('base_link', 'laser_link', 'imu_link', 'mic_link'):
        assert after.body(name).mass[0] == pytest.approx(1.5 * before.body(name).mass[0])
    for prefix in ('left', 'back', 'right'):
        for name in [prefix + '_wheel_link', *[f'{prefix}_roller_{i}' for i in range(16)]]:
            assert after.body(name).mass[0] == pytest.approx(1.5 * before.body(name).mass[0])


def test_urdf_inertial_origin_and_tensor_are_preserved(parameter_workspace):
    from lekiwi_mujoco.build_mujoco_models import build_robot_spec
    module = parameter_workspace / 'urdf/base.module.xacro'
    origin = '<origin xyz="0.01 -0.02 0.03" rpy="0.2 -0.3 0.4"/>'
    text = module.read_text().replace('<inertial>', '<inertial>' + origin, 1)
    text = text.replace('ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"',
                        'ixx="0.012" ixy="0.001" ixz="0.002" iyy="0.013" iyz="0.001" izz="0.014"', 1)
    module.write_text(text)
    model = build_robot_spec().compile()
    body = model.body('base_link').id
    assert np.allclose(model.body_ipos[body], [.01, -.02, .03])
    r = _urdf_transform(ET.fromstring(origin))[:3, :3]
    tensor = np.array([[.012,.001,.002],[.001,.013,.001],[.002,.001,.014]])
    orientation = np.empty(9)
    mujoco.mju_quat2Mat(orientation, model.body_iquat[body])
    orientation = orientation.reshape(3,3)
    assert np.allclose(orientation @ np.diag(model.body_inertia[body]) @ orientation.T, r @ tensor @ r.T)


def test_static_camera_intrinsics_and_imu():
    model = mujoco.MjModel.from_xml_path(str(PACKAGE / 'mjcf' / MODEL_FILENAME))
    data = mujoco.MjData(model)
    camera = model.camera('gemini2_rgb').id
    assert model.cam_resolution[camera].tolist() == [640, 360]
    assert model.cam_fovy[camera] == pytest.approx(55.)
    quat = model.sensor('gemini2_quat').id
    assert model.sensor_type[quat] == mujoco.mjtSensor.mjSENS_FRAMEQUAT
    accel = model.sensor('gemini2_accel').id
    gyro = model.sensor('gemini2_gyro').id
    assert model.sensor_type[accel] == mujoco.mjtSensor.mjSENS_ACCELEROMETER
    assert model.sensor_type[gyro] == mujoco.mjtSensor.mjSENS_GYRO
    assert model.sensor_objid[quat] == model.sensor_objid[accel] == model.sensor_objid[gyro]
    site = model.sensor_objid[gyro]
    assert model.site_bodyid[site] == model.body('gemini2_link').id
    data.qvel[5] = .7
    mujoco.mj_forward(model, data)
    address = model.sensor_adr[gyro]
    assert np.linalg.norm(data.sensordata[address:address + 3]) == pytest.approx(.7)
    # The IMU optical axes agree with the render camera (+Z versus MuJoCo -Z).
    optical = data.site_xmat[site].reshape(3, 3)
    render = data.cam_xmat[camera].reshape(3, 3)
    np.testing.assert_allclose(optical, render @ np.diag([1, -1, -1]), atol=1e-6)


def test_static_camera_pose_and_sensor_interfaces():
    from lekiwi_mujoco.build_mujoco_models import build_robot_spec
    model = build_robot_spec().compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    camera = model.camera('gemini2_rgb').id
    base = data.body('base_link')
    rotation = base.xmat.reshape(3, 3)
    assert np.allclose(rotation.T @ (data.cam_xpos[camera] - base.xpos), [0.09365, 0, 0.0723], atol=1e-6)
    assert np.allclose(rotation.T @ -data.cam_xmat[camera].reshape(3, 3)[:, 2], [1, 0, 0], atol=1e-6)
    assert model.cam_resolution[camera].tolist() == [640, 360]
    for name in ('gemini2_quat', 'gemini2_accel', 'gemini2_gyro'):
        assert model.sensor(name).id >= 0


def _urdf_transform(origin):
    """Independent matrix implementation of URDF's fixed-axis RPY convention."""
    import math
    transform = np.eye(4)
    if origin is None:
        return transform
    transform[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
    roll, pitch, yaw = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    transform[:3, :3] = rz @ ry @ rx
    return transform


@pytest.mark.parametrize('prefix', ['', 'robot1/'])
def test_frame_prefix_names_the_lidar_and_tof_bodies(prefix, tmp_path):
    import mujoco
    from lekiwi_mujoco.build_mujoco_models import build
    model = mujoco.MjModel.from_xml_path(str(build(tmp_path / 'model.xml', absolute=True, frame_prefix=prefix)))
    bodies = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(model.nbody)}
    assert {f'{prefix}laser_frame', f'{prefix}tof_frame'} <= bodies
    assert ('laser_frame' in bodies) == (prefix == '')
