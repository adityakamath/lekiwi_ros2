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

VARIANTS = ['lekiwi_base.xml', 'lekiwi_pt100_oakd_s2.xml', 'lekiwi_pt101_oakd_s2.xml']


@pytest.mark.parametrize('filename', VARIANTS)
def test_models_settle_and_preserve_interfaces_and_wheel_mass(filename):
    runtime = runtime_from_file(PACKAGE / 'mjcf' / filename)
    model, data = runtime.model, runtime.data
    assert np.isfinite(data.qpos).all()
    assert .038 < data.qpos[2] < .043
    assert np.linalg.norm(data.qvel[:6]) < .002
    assert model.nu == (3 if filename == VARIANTS[0] else 5)
    assert sum('roller_joint' in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) or '') for i in range(model.njnt)) == 48
    from lekiwi_mujoco.build_mujoco_models import package_paths
    import xacro
    variant = 'base' if filename == VARIANTS[0] else ('pt100' if filename == VARIANTS[1] else 'pt101')
    source = 'base/base.urdf.xacro' if variant == 'base' else 'base_pantilt/base_pantilt.urdf.xacro'
    with package_paths({'lekiwi_description': DESCRIPTION,
                        'pt_description': PACKAGE.parent / 'payloads/pantilt_ros2/pt_description'}):
        doc = xacro.process_file(str(DESCRIPTION / 'urdf' / source), mappings={'base_controller_config': str(PACKAGE.parent / 'lekiwi_control/config/base/control.yaml'), 'pantilt_config': variant, 'use_mock': 'true'})
    urdf = ET.fromstring(doc.toxml())
    masses = {link.get('name'): float(link.find('inertial/mass').get('value'))
              for link in urdf.findall('link') if link.find('inertial/mass') is not None}
    assert model.body_mass.sum() == pytest.approx(sum(masses.values()))
    for prefix in ['left', 'back', 'right']:
        hub = model.body(prefix + '_wheel_link').id
        children = [i for i in range(model.nbody) if model.body_parentid[i] == hub]
        assert model.body_mass[hub] + model.body_mass[children].sum() == pytest.approx(masses[prefix + '_wheel_link'])
    assert model.sensor('lidar-359').id >= 0
    assert model.sensor('bno055_quat').id >= 0


@pytest.mark.parametrize('filename', VARIANTS)
@pytest.mark.parametrize('command,axis,target', [((.15, 0, 0), 0, .3), ((0, .15, 0), 1, .3), ((0, 0, .5), 2, 1.), ((-.15, 0, 0), 0, -.3)])
def test_measured_motion_in_all_payloads(filename, command, axis, target):
    runtime = runtime_from_file(PACKAGE / 'mjcf' / filename)
    start = runtime.pose()
    runtime.command(command)
    runtime.step(1000)
    delta = runtime.pose() - start
    assert abs(delta[axis] - target) < .05
    assert np.max(np.abs(np.delete(delta, axis))) < .035


@pytest.mark.parametrize('filename', VARIANTS[1:])
def test_pan_tilt_and_optical_axes(filename):
    runtime = runtime_from_file(PACKAGE / 'mjcf' / filename)
    model, data = runtime.model, runtime.data
    camera = model.camera('oak_rgb').id
    assert np.dot(-data.cam_xmat[camera].reshape(3, 3)[:, 2], [1, 0, 0]) > .999
    for name, goal in [('shoulder_pan_joint', .6), ('tilt_joint', .3)]:
        data.ctrl[model.actuator(name).id] = goal
    runtime.step(2000)
    for name, goal in [('shoulder_pan_joint', .6), ('tilt_joint', .3)]:
        assert abs(data.qpos[model.joint(name).qposadr[0]] - goal) < .03
    assert np.isfinite(data.qpos).all()
    assert np.dot(-data.cam_xmat[camera].reshape(3, 3)[:, 2], [1, 0, 0]) < .95


def test_wheel_command_speed_limit():
    runtime = runtime_from_file(PACKAGE / 'mjcf/lekiwi_base.xml')
    rates = runtime.control.wheel_speeds([100, 100, 100])
    assert np.all(abs(rates) <= runtime.control.wheel_limits + 1e-10)


@pytest.mark.parametrize('variant,filename', list(zip(['base', 'pt100', 'pt101'], VARIANTS)))
def test_generated_assets_are_current_and_portable(tmp_path, variant, filename):
    pytest.importorskip('xacro')
    spec = importlib.util.spec_from_file_location('builder', PACKAGE / 'lekiwi_mujoco/build_mujoco_models.py')
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    output = builder.build(variant, tmp_path / filename, absolute=True)
    expected = ET.parse(output).getroot()
    actual = ET.parse(PACKAGE / 'mjcf' / filename).getroot()
    for root, directory in [(expected, tmp_path), (actual, PACKAGE / 'mjcf')]:
        for mesh in root.iter('mesh'):
            mesh.set('file', str((directory / mesh.attrib['file']).resolve()))
        for node in root.iter():
            node.text = (node.text or '').strip()
            node.tail = ''
    assert ET.tostring(expected) == ET.tostring(actual)
    assert not actual.findall('.//include')
    mujoco.MjModel.from_xml_path(str(output))


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


@pytest.mark.parametrize('variant', ['pt100', 'pt101'])
@pytest.mark.parametrize('pan,tilt', [(0., 0.), (.6, -.4), (-.7, .5)])
def test_payload_frames_and_visual_meshes_match_urdf(variant, pan, tilt):
    import math
    xacro = pytest.importorskip('xacro')
    from lekiwi_mujoco.build_mujoco_models import package_paths
    payload = PACKAGE.parent / 'payloads/pantilt_ros2/pt_description'
    with package_paths({'lekiwi_description': DESCRIPTION, 'pt_description': payload}):
        doc = xacro.process_file(str(DESCRIPTION / 'urdf/base_pantilt/base_pantilt.urdf.xacro'),
                                 mappings={'base_controller_config': str(PACKAGE.parent / 'lekiwi_control/config/base/control.yaml'), 'pantilt_config': variant, 'use_mock': 'true',
                                           'simulation_controllers': '', 'payload_simulation_controllers': ''})
    urdf = ET.fromstring(doc.toxml())
    transforms = {'base_footprint': np.eye(4)}
    remaining = list(urdf.findall('joint'))
    while remaining:
        progress = False
        for joint in list(remaining):
            parent = joint.find('parent').get('link')
            if parent not in transforms:
                continue
            local = _urdf_transform(joint.find('origin'))
            angle = {'shoulder_pan_joint': pan, 'tilt_joint': tilt}.get(joint.get('name'), 0.)
            if angle:
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                axis /= np.linalg.norm(axis)
                x, y, z = axis
                skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                motion = np.eye(4)
                motion[:3, :3] = np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*(skew @ skew)
                local = local @ motion
            transforms[joint.find('child').get('link')] = transforms[parent] @ local
            remaining.remove(joint)
            progress = True
        assert progress, 'Disconnected URDF joint tree'
    runtime = runtime_from_file(PACKAGE / 'mjcf' / f'lekiwi_{variant}_oakd_s2.xml', settle=False)
    model, data = runtime.model, runtime.data
    for name, value in [('shoulder_pan_joint', pan), ('tilt_joint', tilt)]:
        data.qpos[model.joint(name).qposadr[0]] = value
    mujoco.mj_forward(model, data)
    base_inverse = np.linalg.inv(transforms['base_link'])
    for name in ['pantilt_base_link', 'shoulder_link', 'tilt_link', 'oak_link', 'oak_link_model_origin']:
        expected = base_inverse @ transforms[name]
        body = model.body(name).id
        actual = np.eye(4)
        actual[:3, :3] = data.xmat[body].reshape(3, 3)
        actual[:3, 3] = data.xpos[body] - data.xpos[runtime.robot.base]
        assert np.allclose(actual, expected, atol=1e-8), name
    # Compare actual compiled/rendered mesh bounds with original STL vertices transformed
    # through the URDF. This catches wrong visual offsets as well as frame-only errors.
    for link_name, mesh_name in [('tilt_link', 'tilt_joint_oakd_s2'),
                                  ('oak_link_model_origin', 'oakd_s2')]:
        visual = urdf.find(f"link[@name='{link_name}']/visual")
        transform = transforms[link_name] @ _urdf_transform(visual.find('origin'))
        path = payload / 'meshes' / (mesh_name + '.stl')
        dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
        records = np.fromfile(path, dtype=dtype, offset=84)
        vertices = records['vertices'].reshape(-1, 3).astype(float)
        expected_vertices = vertices @ transform[:3, :3].T + transform[:3, 3]
        mesh_id = model.mesh(mesh_name).id
        geom_id = next(i for i in range(model.ngeom)
                       if model.geom_dataid[i] == mesh_id and model.geom_group[i] == 2)
        start, count = model.mesh_vertadr[mesh_id], model.mesh_vertnum[mesh_id]
        compiled = model.mesh_vert[start:start+count]
        actual_vertices = compiled @ data.geom_xmat[geom_id].reshape(3, 3).T + data.geom_xpos[geom_id]
        assert np.allclose(actual_vertices.min(axis=0), expected_vertices.min(axis=0), atol=2e-7)
        assert np.allclose(actual_vertices.max(axis=0), expected_vertices.max(axis=0), atol=2e-7)


def test_limits_regenerate_from_edited_configuration_and_urdf(tmp_path, monkeypatch):
    import shutil
    import yaml
    from lekiwi_mujoco import build_mujoco_models as builder
    description = tmp_path / 'lekiwi_description'
    for folder in ['urdf', 'meshes']:
        shutil.copytree(DESCRIPTION / folder, description / folder)
    simulation = tmp_path / 'lekiwi_mujoco'
    for folder in ['mjcf', 'config']:
        shutil.copytree(PACKAGE / folder, simulation / folder)
    control = tmp_path / 'lekiwi_control/config/base'
    shutil.copytree(PACKAGE.parent / 'lekiwi_control/config/base', control)
    config = control / 'control.yaml'
    config_data = yaml.safe_load(config.read_text())
    base = config_data['base_controller']['ros__parameters']
    base['linear']['x']['max_velocity'] = .08
    base['linear']['y']['max_velocity'] = .06
    base['angular']['z']['max_velocity'] = .25
    config.write_text(yaml.safe_dump(config_data))
    motors = control / 'urdf_config.yaml'
    motor_data = yaml.safe_load(motors.read_text())
    motor_data['sts3215_max_vel_steps'] = 2000
    motors.write_text(yaml.safe_dump(motor_data))
    monkeypatch.setattr(builder, 'PACKAGE', description)
    monkeypatch.setattr(builder, 'SIM_PACKAGE', simulation)
    monkeypatch.setattr(builder, 'control_package', lambda: tmp_path / 'lekiwi_control')
    root = ET.parse(builder.build('base', tmp_path / 'first.xml')).getroot()
    assert root.find("custom/numeric[@name='base_velocity_limits']").get('data') == '0.08 0.06 0.25'
    expected = int(2000 * .85) * 2 * np.pi / 4096
    assert float(root.find("actuator/general[@name='left_wheel_joint']").get('ctrlrange').split()[1]) == pytest.approx(expected, abs=1e-5)
    module = description / 'urdf/base/base.module.xacro'
    module.write_text(module.read_text().replace('velocity="${max_velocity_rads}"', 'velocity="0.9"'))
    root = ET.parse(builder.build('base', tmp_path / 'second.xml')).getroot()
    assert root.find("actuator/general[@name='left_wheel_joint']").get('ctrlrange') == '-0.9 0.9'


def test_payload_config_overrides_velocity_and_position_limits(tmp_path, monkeypatch):
    import shutil
    import yaml
    from lekiwi_mujoco import build_mujoco_models as builder
    description = tmp_path / 'lekiwi_description'
    for folder in ['urdf', 'meshes']:
        shutil.copytree(DESCRIPTION / folder, description / folder)
    simulation = tmp_path / 'lekiwi_mujoco'
    for folder in ['mjcf', 'config']:
        shutil.copytree(PACKAGE / folder, simulation / folder)
    control = tmp_path / 'lekiwi_control/config'
    shutil.copytree(PACKAGE.parent / 'lekiwi_control/config', control)
    config = control / 'payloads/pantilt/control.yaml'
    settings = yaml.safe_load(config.read_text())
    limits = settings['controller_manager']['ros__parameters']['joint_limits']
    for name, speed, low, high in [('shoulder_pan_joint', .7, -.4, .5), ('tilt_joint', .9, -.6, .3)]:
        limits[name].update(has_velocity_limits=True, max_velocity=speed,
                            min_position=low, max_position=high)
    config.write_text(yaml.safe_dump(settings))
    monkeypatch.setattr(builder, 'PACKAGE', description)
    monkeypatch.setattr(builder, 'SIM_PACKAGE', simulation)
    monkeypatch.setattr(builder, 'control_package', lambda: tmp_path / 'lekiwi_control')
    root = ET.parse(builder.build('pt101', tmp_path / 'payload.xml',
        pt_package=PACKAGE.parent / 'payloads/pantilt_ros2/pt_description')).getroot()
    for name, speed, low, high in [('shoulder_pan_joint', .7, -.4, .5), ('tilt_joint', .9, -.6, .3)]:
        assert float(root.find(f"custom/numeric[@name='velocity_limit_{name}']").get('data')) == speed
        assert root.find(f"actuator/general[@name='{name}']").get('ctrlrange') == f'{low} {high}'
        assert root.find(f".//worldbody//joint[@name='{name}']").get('range') == f'{low} {high}'


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
    robot = build_robot_spec('base')
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
    path = build('base', tmp_path / 'different_directory' / 'model.xml', scene=scene)
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


@pytest.mark.parametrize('variant', ['base', 'pt100', 'pt101'])
def test_changed_geometry_and_inertia_propagate(parameter_workspace, variant):
    import yaml
    from lekiwi_mujoco.build_mujoco_models import build_robot_spec
    path = parameter_workspace.parent / 'lekiwi_control/config/base/control.yaml'
    settings = yaml.safe_load(path.read_text())
    geometry = settings['base_controller']['ros__parameters']
    geometry.update(wheel_radius=.0612, robot_radius=.15, wheel_offset=.9)
    path.write_text(yaml.safe_dump(settings))
    module = parameter_workspace / 'urdf/base/base.module.xacro'
    module.write_text(module.read_text().replace('<mass value="2.0"/>', '<mass value="2.7"/>'))
    robot = build_robot_spec(variant, pt_package=PACKAGE.parent / 'payloads/pantilt_ros2/pt_description')
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
    module = parameter_workspace / 'urdf/base/base.module.xacro'
    module.write_text(module.read_text().replace('effort="${max_torque_nm}"', 'effort="1.7"'))
    model = build_robot_spec('base').compile()
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
        path = parameter_workspace.parent / 'lekiwi_control/config/base/control.yaml'
        settings = yaml.safe_load(path.read_text())
        settings['base_controller']['ros__parameters']['wheel_radius'] = -1
        path.write_text(yaml.safe_dump(settings))
    elif failure == 'roller_mass':
        path = parameter_workspace.parent / 'lekiwi_mujoco/config/mujoco.yaml'
        settings = yaml.safe_load(path.read_text())
        settings['contact']['roller_mass_fraction'] = .1
        path.write_text(yaml.safe_dump(settings))
    else:
        path = parameter_workspace / 'urdf/base/base.module.xacro'
        text = path.read_text()
        if failure == 'layout':
            text = text.replace('wheel_base_radius * cos(offset)', '0.4 * cos(offset)')
        else:
            text = text.replace('ixx="0.001"', 'ixx="0.1"')
        path.write_text(text)
    with pytest.raises(ValueError):
        build_robot_spec('base')


def test_urdf_mass_edits_replace_every_base_mass(parameter_workspace):
    import re
    from lekiwi_mujoco.build_mujoco_models import build_robot_spec
    before = build_robot_spec('base').compile()
    module = parameter_workspace / 'urdf/base/base.module.xacro'
    text = re.sub(r'<mass value="([0-9.]+)"/>',
                  lambda match: f'<mass value="{float(match[1]) * 1.5}"/>', module.read_text())
    module.write_text(text)
    after = build_robot_spec('base').compile()
    assert after.body_mass.sum() == pytest.approx(1.5 * before.body_mass.sum())
    for name in ('base_link', 'laser_link', 'imu_link', 'mic_link'):
        assert after.body(name).mass[0] == pytest.approx(1.5 * before.body(name).mass[0])
    for prefix in ('left', 'back', 'right'):
        for name in [prefix + '_wheel_link', *[f'{prefix}_roller_{i}' for i in range(16)]]:
            assert after.body(name).mass[0] == pytest.approx(1.5 * before.body(name).mass[0])


def test_urdf_inertial_origin_and_tensor_are_preserved(parameter_workspace):
    from lekiwi_mujoco.build_mujoco_models import build_robot_spec
    module = parameter_workspace / 'urdf/base/base.module.xacro'
    origin = '<origin xyz="0.01 -0.02 0.03" rpy="0.2 -0.3 0.4"/>'
    text = module.read_text().replace('<inertial>', '<inertial>' + origin, 1)
    text = text.replace('ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"',
                        'ixx="0.012" ixy="0.001" ixz="0.002" iyy="0.013" iyz="0.001" izz="0.014"', 1)
    module.write_text(text)
    model = build_robot_spec('base').compile()
    body = model.body('base_link').id
    assert np.allclose(model.body_ipos[body], [.01, -.02, .03])
    r = _urdf_transform(ET.fromstring(origin))[:3, :3]
    tensor = np.array([[.012,.001,.002],[.001,.013,.001],[.002,.001,.014]])
    orientation = np.empty(9)
    mujoco.mju_quat2Mat(orientation, model.body_iquat[body])
    orientation = orientation.reshape(3,3)
    assert np.allclose(orientation @ np.diag(model.body_inertia[body]) @ orientation.T, r @ tensor @ r.T)


def test_payload_mass_changes_and_new_camera_inertia(parameter_workspace):
    import shutil
    from lekiwi_mujoco.build_mujoco_models import build_robot_spec
    source = PACKAGE.parent / 'payloads/pantilt_ros2/pt_description'
    payload = parameter_workspace.parent / 'payload'
    shutil.copytree(source / 'urdf', payload / 'urdf')
    shutil.copytree(source / 'mjcf', payload / 'mjcf')
    shutil.copytree(source / 'meshes', payload / 'meshes')
    module = payload / 'urdf/pantilt.module.xacro'
    text = module.read_text().replace('<mass value="1.0"/>', '<mass value="1.3"/>')
    module.write_text(text)
    module = payload / 'urdf/oakd_s2.module.xacro'
    text = module.read_text()
    # Adding missing inertial data later must replace the massless frame automatically.
    text = text.replace('<link name="${camera_name}_model_origin">',
                        '<link name="${camera_name}_model_origin"><inertial><mass value="0.2"/>'
                        '<inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial>')
    module.write_text(text)
    model = build_robot_spec('pt101', pt_package=payload).compile()
    assert model.body('pantilt_base_link').mass[0] == pytest.approx(1.3)
    assert model.body('oak_link_model_origin').mass[0] == pytest.approx(.2)
