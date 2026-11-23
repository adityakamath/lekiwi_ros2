#!/usr/bin/env python3
"""Generate the configured LeKiwi base MJCF from URDF and simulation settings.

Run python3 -m lekiwi_mujoco.build_mujoco_models to regenerate the base model.
--absolute keeps mesh paths valid when ROS launch generates a temporary model.
"""
import argparse
from contextlib import contextmanager
import os
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
from lekiwi_mujoco.hardware_config import load_hardware_config
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


def expand_urdf(packages, control, hardware_config=None):
    """The robot URDF in mock mode, plus its controller geometry."""
    config = yaml.safe_load((control / 'config/control.yaml').read_text())['/**/base_controller']['ros__parameters']
    motor = yaml.safe_load((control / 'config/urdf_config.yaml').read_text())
    hardware_config = hardware_config or package_share('lekiwi_bringup') / 'config/hardware.yaml'
    source = 'base.urdf.xacro'
    load_hardware_config(hardware_config)  # Validate Python API callers as well as CLI profiles.
    with package_paths(packages):
        doc = xacro.process_file(str(packages['lekiwi_description'] / 'urdf' / source), mappings={
            'use_mock': 'true',
            'base_controller_config': str(control / 'config/control.yaml'),
            **{key: str(value).lower() if isinstance(value, bool) else str(value) for key, value in motor.items()},
            'simulation_controllers': '',
            'hardware_config': str(hardware_config)})
    return ET.fromstring(doc.toxml()), config


def add_static_camera(spec, urdf, packages):
    """Attach the static camera using URDF geometry and frames."""
    mount = spec.body('base_link').add_body(name='gemini2_link')
    set_origin(mount, urdf.find("joint[@name='gemini2_link_center_joint']/origin"))
    model = mount.add_body(name='gemini2_link_model_origin')
    set_origin(model, urdf.find("joint[@name='gemini2_link_model_origin_joint']/origin"))
    spec.add_mesh(name='gemini2', file=str(packages['lekiwi_description'] / 'meshes/gemini2.stl'))
    geom = model.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname='gemini2', contype=0, conaffinity=0,
                          group=2, mass=0, rgba=[0.15, 0.15, 0.15, 1.0])
    set_origin(geom, urdf.find("link[@name='gemini2_link_model_origin']/visual/origin"))
    # Gemini 2 16:9 RGB profile, nominal V55/H86; right is the frame's -Y and up its +Z.
    camera = mount.add_camera(name='gemini2_rgb', fovy=55, resolution=[640, 360])
    camera.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES
    camera.alt.xyaxes = [0, -1, 0, 0, 0, 1]
    # Ideal co-located six-axis IMU in optical axes; device extrinsics need calibration.
    mount.add_site(name='gemini2_imu', size=[0.001, 0, 0], quat=[0.5, -0.5, 0.5, -0.5])
    for name, kind in (('gemini2_accel', mujoco.mjtSensor.mjSENS_ACCELEROMETER),
                       ('gemini2_gyro', mujoco.mjtSensor.mjSENS_GYRO),
                       ('gemini2_quat', mujoco.mjtSensor.mjSENS_FRAMEQUAT)):
        spec.add_sensor(name=name, type=kind, objtype=mujoco.mjtObj.mjOBJ_SITE, objname='gemini2_imu')


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
        if name in ('left_wheel_joint', 'back_wheel_joint', 'right_wheel_joint'):
            limit = float(urdf.find(f"joint[@name='{name}']/limit").get('velocity'))
            actuator.ctrllimited = True
            actuator.ctrlrange = [-limit, limit]
            numeric('velocity_limit_' + name, [limit])


def configure_base_components(spec, urdf, packages):
    """Draw split fixed parts from the same expanded URDF used for TF and hardware.

    Keep the simulation's wheel contacts and sensor sites; remove absent sensor
    bodies before compilation so no disabled device survives as hidden geometry.
    """
    links = {link.get('name'): link for link in urdf.findall('link')}
    for body_name, prefix in [('laser_link', 'lidar-'), ('imu_link', 'bno055_'),
                              ('mic_link', None), ('tof_link', None), ('rpi5_case_link', None)]:
        if body_name in links:
            continue
        if prefix:
            for sensor in list(spec.sensors):
                if sensor.name.startswith(prefix):
                    spec.delete(sensor)
        body = spec.body(body_name)
        if body is not None:
            spec.delete(body)
    # The plain chassis fits MuJoCo's STL limit; never draw the old combined mesh.
    filename = links['base_link'].find('visual/geometry/mesh').get('filename').split('/')[-1]
    spec.mesh('lekiwi_base_visual').file = str(packages['lekiwi_description'] / 'meshes' / filename)
    for joint in urdf.findall('joint'):
        if joint.get('type') != 'fixed' or joint.find('parent').get('link') != 'base_link':
            continue
        name = joint.find('child').get('link')
        if spec.body(name) is not None or name == 'gemini2_link':
            continue
        link = links[name]
        visual = link.find('visual')
        if visual is None:
            continue
        body = spec.body('base_link').add_body(name=name)
        set_origin(body, joint.find('origin'))
        filename = visual.find('geometry/mesh').get('filename').split('/')[-1]
        mesh_name = Path(filename).stem + '_visual'
        if spec.mesh(mesh_name) is None:
            spec.add_mesh(name=mesh_name, file=str(packages['lekiwi_description'] / 'meshes' / filename))
        geom = body.add_geom(name=name + '_visual', type=mujoco.mjtGeom.mjGEOM_MESH,
                             meshname=mesh_name, contype=0, conaffinity=0, group=2, mass=0)
        set_origin(geom, visual.find('origin'))
        geom.rgba = list(map(float, visual.find('material/color').get('rgba').split()))
        spec.mesh(mesh_name).scale = list(map(float, visual.find('geometry/mesh').get('scale', '1 1 1').split()))
    used_meshes = {geom.meshname for geom in spec.geoms if geom.meshname}
    for mesh in list(spec.meshes):
        if mesh.name not in used_meshes:
            spec.delete(mesh)


def build_robot_spec(*, control_dir=None, description_dir=None, hardware_config=None):
    """Return an editable native spec with absolute assets for future scene composition."""
    control = Path(control_dir).resolve() if control_dir else control_package()
    packages = {'lekiwi_description': Path(description_dir).resolve() if description_dir else (PACKAGE or package_share('lekiwi_description'))}
    with package_paths(packages):
        doc = xacro.process_file(str(SIM_PACKAGE / 'mjcf/base.mjcf.xacro'))
    # MuJoCo parses includes and maintains model references; no custom XML assembly.
    spec = mujoco.MjSpec.from_string(doc.toxml())
    urdf, config = expand_urdf(packages, control, hardware_config)
    configure_base_components(spec, urdf, packages)
    if urdf.find("link[@name='gemini2_link']") is not None:
        add_static_camera(spec, urdf, packages)
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


def build_spec(scene='flat', *, control_dir=None, description_dir=None, hardware_config=None):
    return compose_scene(build_robot_spec(control_dir=control_dir, description_dir=description_dir, hardware_config=hardware_config), scene)


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
    frame = next((e for e in root.iter('body') if e.get('name') == 'laser_frame'), None)
    if frame is None:
        return xml
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


def use_native_tof(xml):
    """Add a VL53L5CX mujoco.plugin.lidar instance on tof_frame, independently of the LD06.

    The site's x axis points along tof_frame's +z, the sensor's boresight. It sits site_height above the chip, over
    the chassis collision cylinder, which the plugin's rays would otherwise hit from inside. Rays span the sensor's
    square field of view, one per zone centre.
    """
    tof = yaml.safe_load((SIM_PACKAGE / 'config/mujoco.yaml').read_text())['tof']
    root = ET.fromstring(xml)
    frame = next((e for e in root.iter('body') if e.get('name') == 'tof_frame'), None)
    if frame is None:
        return xml
    # R_y(-90 deg): site x -> tof_frame z, site y -> y, site z -> -x.
    ET.SubElement(frame, 'site', name='tof', pos=f"0 0 {tof['site_height']}", quat='0.70710678 0 -0.70710678 0', group='4')
    zones = tof['resolution']
    half = math.radians(tof['fov_degrees']) / 2 * (1 - 1 / zones)  # first/last zone centres
    plugin = root.find('extension/plugin[@plugin="mujoco.plugin.lidar"]')
    if plugin is None:
        extension = root.find('extension')
        if extension is None:
            extension = ET.SubElement(root, 'extension')
        plugin = ET.SubElement(extension, 'plugin', plugin='mujoco.plugin.lidar')
    instance = ET.SubElement(plugin, 'instance', name='tof')
    for key, value in {
            'resolution': f'{zones} {zones}', 'azimuth_range': f'{-half:.7f} {half:.7f}',
            'elevation_range': f'{-half:.7f} {half:.7f}', 'max_range': tof['max_range'],
            'min_range': tof['min_range'], 'update_rate': tof['update_rate'],
            'async': 0}.items():
        ET.SubElement(instance, 'config', key=key, value=str(value))
    sensor = next(root.iter('sensor'))
    ET.SubElement(sensor, 'plugin', name='tof', instance='tof', objtype='site', objname='tof')
    return ET.tostring(root, encoding='unicode')


def prefix_frames(xml, prefix):
    """Rename the bodies the lidar and ToF plugins publish their scans in, so each robot's frames are its own."""
    root = ET.fromstring(xml)
    for body in root.iter('body'):
        if body.get('name') in ('laser_frame', 'tof_frame'):
            body.set('name', prefix + body.get('name'))
    return ET.tostring(root, encoding='unicode')


def build(output, absolute=False, scene=True, *, control_dir=None, description_dir=None, lidar='rangefinder', hardware_config=None, frame_prefix=''):
    output = Path(output).resolve()
    spec = build_spec(scene=scene, control_dir=control_dir, description_dir=description_dir, hardware_config=hardware_config)
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
        xml = use_native_tof(use_native_lidar(xml))
    elif lidar != 'rangefinder':
        raise ValueError("lidar must be 'rangefinder' or 'plugin'")
    if frame_prefix:
        xml = prefix_frames(xml, frame_prefix)
    output.write_text(content + xml)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--absolute', action='store_true')
    parser.add_argument('--control-package', type=Path, help='Directory containing control config/ (data only)')
    parser.add_argument('--description-package', type=Path, help='Robot description source/share directory')
    parser.add_argument('--scene', default='flat', help='flat, arena, home, maze, none, or a scene MJCF path')
    parser.add_argument('--lidar', choices=['rangefinder', 'plugin'], default='rangefinder',
                        help='plugin: one native mujoco.plugin.lidar sensor (ROS/mujoco_vendor only)')
    parser.add_argument('--hardware-config', type=Path, help='YAML file selecting optional hardware groups')
    parser.add_argument('--frame-prefix', default='', help='Prefix for the lidar and ToF frame names, e.g. robot1/')
    args = parser.parse_args()
    hardware_config = args.hardware_config
    load_hardware_config(hardware_config)
    output = args.output or SIM_PACKAGE / 'mjcf/lekiwi_base.xml'
    print(build(output, args.absolute, scene=args.scene,
                control_dir=args.control_package, description_dir=args.description_package,
                lidar=args.lidar, hardware_config=hardware_config, frame_prefix=args.frame_prefix))


if __name__ == '__main__':
    main()
