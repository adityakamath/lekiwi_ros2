#!/usr/bin/env python3
"""Generate portable base/PT100/PT101 MJCFs from shared xacro + the pan-tilt payload from pt_mujoco.

Also used by ROS launch: --absolute keeps mesh paths valid in temporary files.
Native MjSpec parsing resolves XML includes; the payload is built by pt_mujoco from this robot's
URDF and attached at the URDF's mount, without modifying the pinned pantilt_ros2 dependency.

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

from lekiwi_mujoco.paths import package_share, payload_package

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


def expand_urdf(variant, packages, control):
    """The robot URDF in mock mode, plus the controller geometry and payload joint limits it is built from."""
    config = yaml.safe_load((control / 'config/control.yaml').read_text())['base_controller']['ros__parameters']
    motor = yaml.safe_load((control / 'config/urdf_config.yaml').read_text())
    payload_limits = {}
    if variant != 'base':
        # Optional limit overrides come from the payload's own control package, like the real robot's.
        payload_package()
        from pt_mujoco.build_mujoco_models import joint_limits
        payload_limits = joint_limits(package_share('pt_control'))
    source = 'base/base.urdf.xacro' if variant == 'base' else 'base_pantilt/base_pantilt.urdf.xacro'
    with package_paths(packages):
        doc = xacro.process_file(str(packages['lekiwi_description'] / 'urdf' / source), mappings={
            'pantilt_config': variant, 'use_mock': 'true',
            'base_controller_config': str(control / 'config/control.yaml'),
            **{key: str(value).lower() if isinstance(value, bool) else str(value) for key, value in motor.items()},
            'simulation_controllers': '', 'payload_simulation_controllers': ''})
    return ET.fromstring(doc.toxml()), config, payload_limits


def attach_payload(spec, variant, urdf, payload_limits, packages):
    """Mount the pan-tilt payload at the URDF's mount joint.

    pt_mujoco builds the payload (frames, inertias, limits, servo parameters, camera) from this
    robot's own URDF; the base spec only owns where it is mounted.
    """
    joint = urdf.find("joint[@name='pantilt_mount_joint']")
    if joint is None:
        raise ValueError('Missing payload mount: pantilt_mount_joint')
    parent = spec.body(joint.find('parent').get('link'))
    if parent is None:
        raise ValueError(f"Missing payload mount body: {joint.find('parent').get('link')}")
    payload_package()
    from pt_mujoco.build_mujoco_models import build_payload_spec
    payload = build_payload_spec(variant, urdf, payload_limits, packages['pt_description'])
    frame = parent.add_frame(name='pantilt_mount')
    set_origin(frame, joint.find('origin'))
    # A named root default stays valid when serialized below the robot's default.
    payload.default.name = 'pt_payload'
    spec.attach(payload, prefix='', frame=frame)


def sync_velocity_limits(spec, urdf, config):
    """Embed the wheel command limits and metadata so the viewer needs only the generated XML."""
    simulation = yaml.safe_load((SIM_PACKAGE / 'config/mujoco.yaml').read_text())
    sync_robot_parameters(spec, urdf, config, simulation, set_origin)
    def numeric(name, values, positive=True):
        values = [float(value) for value in values]
        if not all(math.isfinite(v) and (not positive or v > 0) for v in values):
            raise ValueError(f'Invalid command limits: {name}')
        spec.add_numeric(name=name, data=values)
    # Speed scale for the standalone keyboard viewer, owned by this package. The ROS
    # sim reads none of it: it follows the commands it receives (teleop and Nav2 set the speeds).
    numeric('base_velocity_limits', simulation['command']['base_velocity'])
    numeric('wheel_kinematics', [config['wheel_radius'], config['robot_radius'], config['wheel_offset']], positive=False)
    for actuator in spec.actuators:
        name = actuator.target
        # The payload's limits are synced by pt_mujoco when it is attached.
        if name in ('left_wheel_joint', 'back_wheel_joint', 'right_wheel_joint'):
            limit = float(urdf.find(f"joint[@name='{name}']/limit").get('velocity'))
            actuator.ctrllimited = True
            actuator.ctrlrange = [-limit, limit]
            numeric('velocity_limit_' + name, [limit])


def build_robot_spec(variant, pt_package=None, *, control_dir=None, description_dir=None):
    """Return an editable native spec with absolute assets for future scene composition."""
    if variant not in ('base', 'pt100', 'pt101'):
        raise ValueError(f'Unknown variant: {variant}')
    control = Path(control_dir).resolve() if control_dir else control_package()
    packages = {'lekiwi_description': Path(description_dir).resolve() if description_dir else (PACKAGE or package_share('lekiwi_description'))}
    if variant != 'base':
        packages['pt_description'] = Path(pt_package).resolve() if pt_package else package_share('pt_description')
    with package_paths(packages):
        doc = xacro.process_file(str(SIM_PACKAGE / 'mjcf/base.mjcf.xacro'))
    # MuJoCo parses includes and maintains model references; no custom XML assembly.
    spec = mujoco.MjSpec.from_string(doc.toxml())
    urdf, config, payload_limits = expand_urdf(variant, packages, control)
    if variant != 'base':
        attach_payload(spec, variant, urdf, payload_limits, packages)
    sync_velocity_limits(spec, urdf, config)
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
    if scene is True:
        scene = 'flat'
    bundled = SIM_PACKAGE / 'mjcf/scenes' / f'{scene}.xml'
    path = bundled if bundled.is_file() else Path(scene).expanduser().resolve()
    world = mujoco.MjSpec.from_file(str(path))
    resolve_assets(world)
    configure_physics(world)
    # A named root default stays valid when serialized below the scene's default.
    robot.default.name = 'lekiwi_robot'
    world.attach(robot, prefix='', frame=world.worldbody.add_frame(name='lekiwi_spawn'))
    return world


def build_spec(variant, pt_package=None, scene='flat', *, control_dir=None, description_dir=None):
    return compose_scene(build_robot_spec(variant, pt_package, control_dir=control_dir, description_dir=description_dir), scene)


def use_native_lidar(xml):
    """Swap the per-ray rangefinders for one native mujoco.plugin.lidar sensor.

    The rangefinders cast every ray on every physics step (about 99% of step time on a Pi
    for a 5 Hz scan); the plugin casts at its own update_rate. Applied to the written XML
    because the plugin library (mujoco_3d_lidar) is only registered inside the ROS
    mujoco_vendor MuJoCo, not the Python one, so the spec cannot compile it here.
    """
    lidar = yaml.safe_load((SIM_PACKAGE / 'config/mujoco.yaml').read_text())['lidar']
    root = ET.fromstring(xml)
    for sensor in root.iter('sensor'):
        for ray in [e for e in sensor if e.tag == 'rangefinder']:
            sensor.remove(ray)
    parents = {child: parent for parent in root.iter() for child in parent}
    for site in [e for e in root.iter('site') if e.get('name', '').startswith('lidar-')]:
        parents[site].remove(site)
    frame = next(e for e in root.iter('body') if e.get('name') == 'laser_frame')
    # The plugin excludes no body from its rays (the rangefinders excluded their own), so the
    # visual LD06 mesh around the site would be hit at zero range and blank the whole scan.
    for geom in [e for e in frame if e.tag == 'geom']:
        frame.remove(geom)
    ET.SubElement(frame, 'site', name='lidar', pos='0 0 0', group='4')
    plugin = ET.Element('plugin', plugin='mujoco.plugin.lidar')
    instance = ET.SubElement(plugin, 'instance', name='lidar')
    for key, value in {
            'resolution': f"{lidar['resolution']} 1", 'azimuth_range': '-3.1415926 3.1415926',
            'elevation_range': '0 0', 'max_range': lidar['max_range'],
            'min_range': lidar['min_range'], 'update_rate': lidar['update_rate'],
            'async': int(lidar['async'])}.items():
        ET.SubElement(instance, 'config', key=key, value=str(value))
    extension = ET.Element('extension')
    extension.append(plugin)
    root.insert(0, extension)
    sensor = next(root.iter('sensor'))
    ET.SubElement(sensor, 'plugin', name='lidar', instance='lidar', objtype='site', objname='lidar')
    return ET.tostring(root, encoding='unicode')


def build(variant, output, absolute=False, pt_package=None, scene=True, *, control_dir=None, description_dir=None, lidar='rangefinder'):
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
    xml = spec.to_xml()
    if lidar == 'plugin':
        xml = use_native_lidar(xml)
    elif lidar != 'rangefinder':
        raise ValueError("lidar must be 'rangefinder' or 'plugin'")
    output.write_text(content + xml)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant', choices=['base', 'pt100', 'pt101'])
    parser.add_argument('--output', type=Path)
    parser.add_argument('--absolute', action='store_true')
    parser.add_argument('--pt-package', type=Path)
    parser.add_argument('--control-package', type=Path, help='Directory containing control config/ (data only)')
    parser.add_argument('--description-package', type=Path, help='Robot description source/share directory')
    parser.add_argument('--scene', default='flat', help='flat, arena, none, or a scene MJCF path')
    parser.add_argument('--lidar', choices=['rangefinder', 'plugin'], default='rangefinder',
                        help='plugin: one native mujoco.plugin.lidar sensor (ROS/mujoco_vendor only)')
    args = parser.parse_args()
    if bool(args.variant) != bool(args.output):
        parser.error('--variant and --output must be used together')
    if args.variant:
        build(args.variant, args.output, args.absolute, args.pt_package, scene=args.scene, control_dir=args.control_package, description_dir=args.description_package, lidar=args.lidar)
    else:
        for variant in ('base', 'pt100', 'pt101'):
            filename = 'lekiwi_base.xml' if variant == 'base' else f'lekiwi_{variant}_oakd_s2.xml'
            print(build(variant, SIM_PACKAGE / 'mjcf' / filename, args.absolute, args.pt_package, scene=args.scene, control_dir=args.control_package, description_dir=args.description_package, lidar=args.lidar))


if __name__ == '__main__':
    main()
