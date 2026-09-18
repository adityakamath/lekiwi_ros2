#!/usr/bin/env python3
"""Generate portable base/PT100/PT101 MJCFs from shared xacro + pinned payload.

Also used by ROS launch: --absolute keeps mesh paths valid in temporary files.
Native MjSpec parsing resolves XML includes; payload integration fixes are applied
consistently, without modifying the pinned pantilt_ros2 dependency.

Standalone, no install needed: `python3 -m lekiwi_mujoco.build_mujoco_models --variant pt101
--output /tmp/robot.xml`, run from this package's root dir (-m puts the cwd on sys.path).
After `pip install -e .` (or a colcon build), the same tool is also `build_mujoco_models`
on PATH / `ros2 run lekiwi_mujoco build_mujoco_models`.
"""
import argparse
from contextlib import contextmanager
import os
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
from lekiwi_mujoco.mujoco_parameters import positive, sync_robot_parameters
import yaml
import xacro
import xacro.substitution_args

from lekiwi_mujoco.paths import package_share

PACKAGE = None  # Optional description override; resolve lazily so explicit paths work.
SIM_PACKAGE = package_share('lekiwi_mujoco')


@contextmanager
def package_paths(packages):
    """Resolve the two known packages without requiring a ROS Python installation."""
    packages = {'lekiwi_mujoco': SIM_PACKAGE, **packages}
    original = xacro.substitution_args._eval_find
    xacro.substitution_args._eval_find = lambda name: str(packages[name]) if name in packages else original(name)
    try:
        yield
    finally:
        xacro.substitution_args._eval_find = original


def set_origin(element, origin):
    """Translate URDF fixed-axis Rz(yaw) Ry(pitch) Rx(roll) to MJCF quaternion."""
    xyz = origin.get('xyz', '0 0 0') if origin is not None else '0 0 0'
    rpy = origin.get('rpy', '0 0 0') if origin is not None else '0 0 0'
    r, p, y = [float(value) / 2 for value in rpy.split()]
    cr, cp, cy = math.cos(r), math.cos(p), math.cos(y)
    sr, sp, sy = math.sin(r), math.sin(p), math.sin(y)
    quat = (cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy)
    element.pos = list(map(float, xyz.split()))
    element.alt.type = mujoco.mjtOrientation.mjORIENTATION_QUAT
    element.quat = quat


def sync_payload_geometry(spec, packages, variant, control):
    """Use the robot's URDF as the single source for payload frames and mesh origins.

    The pinned payload MJCF has a stale tilt-joint Z (0.1025 vs URDF 0.0541441).
    Reconcile the whole payload chain instead of maintaining another copied offset.
    Empty controller paths avoid requiring control packages for geometry generation.
    """
    with package_paths(packages):
        doc = xacro.process_file(
            str(packages['lekiwi_description'] / 'urdf/base_pantilt/base_pantilt.urdf.xacro'),
            mappings={'pantilt_config': variant, 'use_mock': 'true',
                      'base_controller_config': str(control / 'config/base/control.yaml'),
                      'simulation_controllers': '', 'payload_simulation_controllers': ''})
    urdf = ET.fromstring(doc.toxml())
    joints = [('pantilt_mount_joint', 'pantilt_mount'),
              ('shoulder_pan_joint', 'shoulder_link'), ('tilt_joint', 'tilt_link'),
              ('oak_link_center_joint', 'oak_link'),
              ('oak_link_model_origin_joint', 'oak_link_model_origin')]
    for joint_name, body_name in joints:
        joint = urdf.find(f"joint[@name='{joint_name}']")
        body = spec.body(body_name)
        if joint is None or body is None:
            raise ValueError(f'Missing payload frame: {joint_name} / {body_name}')
        set_origin(body, joint.find('origin'))
        mj_joint = spec.joint(joint_name)
        if mj_joint is not None:
            mj_joint.axis = list(map(float, joint.find('axis').get('xyz').split()))
            limit = joint.find('limit')
            mj_joint.range = [float(limit.get('lower')), float(limit.get('upper'))]
    meshes = {mesh.name: Path(mesh.file).name for mesh in spec.meshes if mesh.file}
    for name in ('pantilt_base_link', 'shoulder_link', 'tilt_link', 'oak_link_model_origin'):
        body = spec.body(name)
        link = urdf.find(f"link[@name='{name}']")
        for geom in body.geoms:
            filename = meshes.get(geom.meshname)
            role = 'collision' if geom.classname.name == 'collision' else 'visual'
            matches = [entry for entry in link.findall(role)
                       if entry.find('geometry/mesh') is not None
                       and Path(entry.find('geometry/mesh').get('filename')).name == filename]
            if len(matches) != 1:
                raise ValueError(f'Ambiguous or missing URDF {role}: {name}/{filename}')
            set_origin(geom, matches[0].find('origin'))
            spec.mesh(geom.meshname).scale = list(map(float, matches[0].find('geometry/mesh').get('scale', '1 1 1').split()))


def sync_velocity_limits(spec, packages, variant, control):
    """Embed configured command limits so the viewer needs only the generated XML."""
    config = yaml.safe_load((control / 'config/base/control.yaml').read_text())['base_controller']['ros__parameters']
    motor = yaml.safe_load((control / 'config/base/urdf_config.yaml').read_text())
    payload_limits = {}
    if variant != 'base':
        motor.update(yaml.safe_load((control / 'config/payloads/pantilt/urdf_config.yaml').read_text()))
        payload_limits = yaml.safe_load((control / 'config/payloads/pantilt/control.yaml').read_text())['controller_manager']['ros__parameters'].get('joint_limits', {})
    source = 'base/base.urdf.xacro' if variant == 'base' else 'base_pantilt/base_pantilt.urdf.xacro'
    with package_paths(packages):
        doc = xacro.process_file(str(packages['lekiwi_description'] / 'urdf' / source), mappings={
            'pantilt_config': variant, 'use_mock': 'true',
            'base_controller_config': str(control / 'config/base/control.yaml'),
            **{key: str(value).lower() if isinstance(value, bool) else str(value) for key, value in motor.items()},
            'simulation_controllers': '', 'payload_simulation_controllers': ''})
    urdf = ET.fromstring(doc.toxml())
    simulation = yaml.safe_load((SIM_PACKAGE / 'config/mujoco.yaml').read_text())
    sync_robot_parameters(spec, urdf, config, simulation, set_origin)
    def numeric(name, values, positive=True):
        values = [float(value) for value in values]
        if not all(math.isfinite(v) and (not positive or v > 0) for v in values):
            raise ValueError(f'Invalid command limits: {name}')
        spec.add_numeric(name=name, data=values)
    numeric('base_velocity_limits', [config['linear']['x']['max_velocity'],
                                    config['linear']['y']['max_velocity'],
                                    config['angular']['z']['max_velocity']])
    numeric('wheel_kinematics', [config['wheel_radius'], config['robot_radius'], config['wheel_offset']], positive=False)
    for actuator in spec.actuators:
        name = actuator.target
        if name in ('left_wheel_joint', 'back_wheel_joint', 'right_wheel_joint'):
            limit = float(urdf.find(f"joint[@name='{name}']/limit").get('velocity'))
            actuator.ctrllimited = True
            actuator.ctrlrange = [-limit, limit]
        else:
            # Real-mode URDF <limit velocity> is deliberately 1e6; use the
            # hardware's actual max_velocity parameter, not that sentinel.
            param = urdf.find(f".//ros2_control/joint[@name='{name}']/param[@name='max_velocity']")
            if param is None:
                raise ValueError(f'Missing hardware velocity limit for {name}')
            hardware_limit = float(param.text)
            urdf_limit = float(urdf.find(f"joint[@name='{name}']/limit").get('velocity'))
            limit = min(hardware_limit, urdf_limit)
            configured = payload_limits.get(name, {})
            if configured.get('has_velocity_limits', False):
                limit = min(limit, float(configured['max_velocity']))
            joint_limit = urdf.find(f"joint[@name='{name}']/limit")
            hardware_joint = urdf.find(f".//ros2_control/joint[@name='{name}']")
            low = max(float(joint_limit.get('lower')),
                      float(hardware_joint.find("param[@name='min_position']").text))
            high = min(float(joint_limit.get('upper')),
                       float(hardware_joint.find("param[@name='max_position']").text))
            if configured.get('has_position_limits', False):
                low = max(low, float(configured['min_position']))
                high = min(high, float(configured['max_position']))
            if not math.isfinite(low) or not math.isfinite(high) or low >= high:
                raise ValueError(f'Invalid position limits for {name}: {low}, {high}')
            spec.joint(name).range = [low, high]
            actuator.inheritrange = 0
            actuator.ctrllimited = True
            actuator.ctrlrange = [low, high]
        numeric('velocity_limit_' + name, [limit])


def build_robot_spec(variant, pt_package=None, *, control_dir=None, description_dir=None):
    """Return an editable native spec with absolute assets for future scene composition."""
    if variant not in ('base', 'pt100', 'pt101'):
        raise ValueError(f'Unknown variant: {variant}')
    control = Path(control_dir).resolve() if control_dir else control_package()
    packages = {'lekiwi_description': Path(description_dir).resolve() if description_dir else (PACKAGE or package_share('lekiwi_description'))}
    if variant != 'base':
        packages['pt_description'] = Path(pt_package).resolve() if pt_package else package_share('pt_description')
    source = SIM_PACKAGE / 'mjcf' / ('base.mjcf.xacro' if variant == 'base' else 'base_pantilt.mjcf.xacro')
    with package_paths(packages):
        doc = xacro.process_file(str(source), mappings={
            'pantilt_config': variant, 'standalone': 'false'})
    # MuJoCo parses includes and maintains model references; no custom XML assembly.
    spec = mujoco.MjSpec.from_string(doc.toxml())
    if variant != 'base':
        sync_payload_geometry(spec, packages, variant, control)
    sync_velocity_limits(spec, packages, variant, control)
    camera = spec.camera('oak_rgb')
    if camera is not None:
        camera.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES
        camera.alt.xyaxes = [0, -1, 0, 0, 0, 1]
    configure_physics(spec)
    return spec


def control_package():
    # Optional checkout/share discovery; installed ROS launch passes this explicitly.
    return package_share('lekiwi_control')


def configure_physics(spec):
    settings = yaml.safe_load((SIM_PACKAGE / 'config/mujoco.yaml').read_text())['physics']
    spec.option.timestep = positive(settings['timestep'], 'physics timestep')
    iterations = positive(settings['iterations'], 'solver iterations')
    if int(iterations) != iterations:
        raise ValueError('solver iterations must be an integer')
    spec.option.iterations = int(iterations)
    integrators = {'implicitfast': mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
                   'implicit': mujoco.mjtIntegrator.mjINT_IMPLICIT,
                   'Euler': mujoco.mjtIntegrator.mjINT_EULER, 'RK4': mujoco.mjtIntegrator.mjINT_RK4}
    spec.option.integrator = integrators[settings['integrator']]


def resolve_assets(spec):
    """Resolve external scene assets before native attachment changes their context."""
    for assets, directory in ((spec.meshes, spec.meshdir), (spec.textures, spec.texturedir)):
        root = Path(spec.modelfiledir or '.') / directory
        for asset in assets:
            if asset.file:
                asset.file = str((root / asset.file).resolve())
    for texture in spec.textures:
        texture.cubefiles = [str((Path(spec.modelfiledir or '.') / spec.texturedir / name).resolve())
                             if name else '' for name in texture.cubefiles]
    spec.meshdir = ''
    spec.texturedir = ''


def compose_scene(robot, scene='flat'):
    """One robot in a separate scene. Native attachment owns references and assets.

    The simulation profile owns global physics options; scenes own world geometry,
    visual settings and lighting. No fleet, multi-instance or terrain generator is defined.
    """
    no_scene = scene is False or scene is None or scene == 'none'
    if no_scene:
        return robot
    if scene is True or scene == 'flat':
        path = SIM_PACKAGE / 'mjcf/scenes/flat.xml'
    else:
        path = Path(scene).expanduser().resolve()
    world = mujoco.MjSpec.from_file(str(path))
    resolve_assets(world)
    configure_physics(world)
    # A named root default stays valid when serialized below the scene's default.
    robot.default.name = 'lekiwi_robot'
    world.attach(robot, prefix='', frame=world.worldbody.add_frame(name='lekiwi_spawn'))
    return world


def build_spec(variant, pt_package=None, scene='flat', *, control_dir=None, description_dir=None):
    return compose_scene(build_robot_spec(variant, pt_package, control_dir=control_dir, description_dir=description_dir), scene)


def build(variant, output, absolute=False, pt_package=None, scene=True, *, control_dir=None, description_dir=None):
    output = Path(output).resolve()
    spec = build_spec(variant, pt_package=pt_package, scene=scene, control_dir=control_dir, description_dir=description_dir)
    for mesh in [*spec.meshes, *spec.textures]:
        if mesh.file:
            path = Path(mesh.file).resolve()
            mesh.file = str(path) if absolute else os.path.relpath(path, output.parent).replace(os.sep, '/')
            # Let MuJoCo compile portable paths without changing process cwd.
            if not absolute:
                spec.assets[mesh.file] = path.read_bytes()
    for texture in spec.textures:
        files = []
        for name in texture.cubefiles:
            if name and not absolute:
                path = Path(name).resolve()
                name = os.path.relpath(path, output.parent).replace(os.sep, '/')
                spec.assets[name] = path.read_bytes()
            files.append(name)
        texture.cubefiles = files
    output.parent.mkdir(parents=True, exist_ok=True)
    spec.compile()
    content = '<!-- Generated by lekiwi_mujoco.build_mujoco_models using MjSpec; edit sources, not this file. -->\n'
    output.write_text(content + spec.to_xml())
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant', choices=['base', 'pt100', 'pt101'])
    parser.add_argument('--output', type=Path)
    parser.add_argument('--absolute', action='store_true')
    parser.add_argument('--pt-package', type=Path)
    parser.add_argument('--control-package', type=Path, help='Directory containing control config/ (data only)')
    parser.add_argument('--description-package', type=Path, help='Robot description source/share directory')
    parser.add_argument('--scene', default='flat', help='flat, none, or a scene MJCF path')
    args = parser.parse_args()
    if bool(args.variant) != bool(args.output):
        parser.error('--variant and --output must be used together')
    if args.variant:
        build(args.variant, args.output, args.absolute, args.pt_package, scene=args.scene, control_dir=args.control_package, description_dir=args.description_package)
    else:
        for variant in ('base', 'pt100', 'pt101'):
            filename = 'lekiwi_base.xml' if variant == 'base' else f'lekiwi_{variant}_oakd_s2.xml'
            print(build(variant, SIM_PACKAGE / 'mjcf' / filename, args.absolute, args.pt_package, scene=args.scene, control_dir=args.control_package, description_dir=args.description_package))


if __name__ == '__main__':
    main()
