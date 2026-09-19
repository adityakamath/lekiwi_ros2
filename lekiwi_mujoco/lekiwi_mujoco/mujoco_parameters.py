"""LeKiwi parameter ownership and URDF-to-MjSpec physical conversion."""
from types import SimpleNamespace

import mujoco
import numpy as np

WHEELS = ('left', 'back', 'right')


def positive(value, name, allow_zero=False):
    value = float(value)
    if not np.isfinite(value) or value < 0 or (value == 0 and not allow_zero):
        raise ValueError(f'{name} must be finite and {"nonnegative" if allow_zero else "positive"}')
    return value


def pose(origin, set_origin):
    target = SimpleNamespace(alt=SimpleNamespace())
    set_origin(target, origin)
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, np.array(target.quat))
    return np.array(target.pos), rotation.reshape(3, 3)


def parallel_axis(position):
    return np.dot(position, position) * np.eye(3) - np.outer(position, position)


def set_inertia(body, mass, center, inertia):
    """Explicit principal inertia; reject invalid sources instead of silently repairing."""
    positive(mass, f'{body.name} mass')
    values, axes = np.linalg.eigh(inertia)
    if not np.isfinite(values).all() or values.min() <= 0 or values.max() > values.sum() - values.max() + 1e-12:
        raise ValueError(f'Nonphysical inertia for {body.name}; check URDF and roller allocation')
    if np.linalg.det(axes) < 0:
        axes[:, 0] *= -1
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, axes.ravel())
    body.explicitinertial = True
    body.mass, body.ipos, body.iquat, body.inertia = mass, center, quaternion, values


def sync_robot_parameters(spec, urdf, geometry, simulation, set_origin):
    """URDF owns frames/inertias; YAML owns wheel geometry and simulation assumptions."""
    radius = positive(geometry['wheel_radius'], 'wheel_radius')
    base_radius = positive(geometry['robot_radius'], 'robot_radius')
    offset = float(geometry['wheel_offset'])
    if not np.isfinite(offset):
        raise ValueError('wheel_offset must be finite')
    # All represented base joints follow the expanded URDF. Payload chain is handled
    # separately because its mount uses an extra fixed intermediate MJCF body.
    base_joints = ['lekiwi_mount_joint', 'laser_joint', 'laser_frame_joint',
                   'imu_joint', 'imu_frame_joint', 'mic_joint'] + [p + '_wheel_joint' for p in WHEELS]
    for name in base_joints:
        joint = urdf.find(f"joint[@name='{name}']")
        body = spec.body(joint.find('child').get('link'))
        if body is None:
            raise ValueError(f'Missing MuJoCo body for {name}')
        set_origin(body, joint.find('origin'))
        mj_joint = spec.joint(name)
        if mj_joint is not None:
            mj_joint.axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
    # Fail rather than silently accept an independently edited URDF wheel layout.
    for i, prefix in enumerate(WHEELS):
        position = spec.body(prefix + '_wheel_link').pos
        expected = base_radius * np.array([np.cos(offset + i*2*np.pi/3), np.sin(offset + i*2*np.pi/3)])
        if not np.allclose(position[:2], expected, atol=1e-10, rtol=0):
            raise ValueError('URDF wheel layout disagrees with controller geometry')
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, spec.body(prefix + '_wheel_link').quat)
        axis = np.array(spec.joint(prefix + '_wheel_joint').axis)
        axis = rotation.reshape(3, 3) @ axis / np.linalg.norm(axis)
        if not np.allclose(axis, [*expected / base_radius, 0], atol=1e-10, rtol=0):
            raise ValueError('URDF wheel axis disagrees with controller kinematics')
    # Sync visual mesh origins and scales, with explicit decimated mesh mapping.
    # Actual asset name for the wheel is read by geoms rather than assumed.
    mesh_scales = {}
    for body_name in ('base_link', 'left_wheel_link', 'back_wheel_link', 'right_wheel_link', 'imu_link', 'mic_link'):
        visual = urdf.find(f"link[@name='{body_name}']/visual")
        for geom in spec.body(body_name).geoms:
            if geom.meshname:
                geom.mass, geom.density = 0, 0
                set_origin(geom, visual.find('origin'))
                scale = visual.find('geometry/mesh').get('scale', '1 1 1')
                scale = np.fromstring(scale, sep=' ')
                if geom.meshname in mesh_scales and not np.allclose(mesh_scales[geom.meshname], scale):
                    raise ValueError(f'Inconsistent URDF scales for shared mesh {geom.meshname}')
                mesh_scales[geom.meshname] = scale
                spec.mesh(geom.meshname).scale = scale
    # Lidar mesh is intentionally on laser_frame to avoid rangefinder self-hits.
    visual = urdf.find("link[@name='laser_link']/visual")
    p, r = pose(visual.find('origin'), set_origin)
    frame = urdf.find("joint[@name='laser_frame_joint']/origin")
    fp, fr = pose(frame, set_origin)
    for geom in spec.body('laser_frame').geoms:
        if geom.meshname:
            geom.mass, geom.density = 0, 0
            geom.pos = fr.T @ (p - fp)
            quaternion = np.empty(4)
            mujoco.mju_mat2Quat(quaternion, (fr.T @ r).ravel())
            geom.alt.type = mujoco.mjtOrientation.mjORIENTATION_QUAT
            geom.quat = quaternion
            spec.mesh(geom.meshname).scale = np.fromstring(visual.find('geometry/mesh').get('scale', '1 1 1'), sep=' ')
    contact = simulation['contact']
    small = positive(contact['roller_radius_fraction'], 'roller_radius_fraction') * radius
    width = positive(contact['roller_half_width_fraction'], 'roller_half_width_fraction') * radius
    if small >= radius:
        raise ValueError('roller_radius_fraction must be less than one')
    fraction = positive(contact['roller_mass_fraction'], 'roller_mass_fraction')
    if 16 * fraction >= 1:
        raise ValueError('Rollers must leave positive hub mass (16 * roller_mass_fraction < 1)')
    friction = [positive(v, 'contact friction', allow_zero=True) for v in contact['friction']]
    if len(friction) != 3:
        raise ValueError('contact friction needs three coefficients')
    # No robot mass is inferred from CAD volume or retained from the imported MJCF.
    # Virtual rollers receive a share of the URDF wheel mass below. Fixed frames
    # without URDF inertia stay massless; missing moving-link inertia is an error.
    for body in spec.bodies:
        if body.name == 'world' or '_roller_' in body.name:
            continue
        link = urdf.find(f"link[@name='{body.name}']")
        element = link.find('inertial') if link is not None else None
        for geom in body.geoms:
            geom.mass, geom.density = 0, 0
        if element is None:
            if list(body.joints):
                raise ValueError(f'Missing URDF inertia for moving body {body.name}')
            body.explicitinertial = True
            body.mass, body.ipos, body.inertia = 0, [0, 0, 0], [0, 0, 0]
    # URDF inertias are authoritative, including sensors previously inferred from mesh density.
    for link in urdf.findall('link'):
        element = link.find('inertial')
        body = spec.body(link.get('name'))
        if element is None:
            continue
        if body is None:
            raise ValueError(f"URDF inertia has no MuJoCo body: {link.get('name')}")
        mass = positive(element.find('mass').get('value'), link.get('name') + ' mass')
        center, rotation = pose(element.find('origin'), set_origin)
        raw = element.find('inertia')
        xx, yy, zz, xy, xz, yz = [float(raw.get(k)) for k in ('ixx','iyy','izz','ixy','ixz','iyz')]
        inertia = rotation @ np.array([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]]) @ rotation.T
        if link.get('name') in [p + '_wheel_link' for p in WHEELS]:
            prefix = link.get('name').split('_')[0]
            roller_mass = mass * fraction
            roller_inertia = roller_mass / 5 * np.array([width**2 + small**2, 2*small**2, small**2 + width**2])
            remaining = inertia + mass * parallel_axis(center)
            for i in range(16):
                roller = spec.body(f'{prefix}_roller_{i}')
                angle = i * 2 * np.pi / 16
                roller.pos = [(radius-small)*np.cos(angle), (radius-small)*np.sin(angle), 0]
                roller.quat = [np.cos(angle/2), 0, 0, np.sin(angle/2)]
                geom = spec.geom(f'{prefix}_roller_collision_{i}')
                geom.size = [small, width, small]
                geom.mass, geom.friction = roller_mass, friction
                joint = spec.joint(f'{prefix}_roller_joint_{i}')
                joint.damping = [positive(contact['damping'], 'roller damping', True), 0, 0]
                joint.armature = positive(contact['armature'], 'roller armature', True)
                matrix = np.empty(9)
                mujoco.mju_quat2Mat(matrix, roller.quat)
                matrix = matrix.reshape(3,3)
                remaining -= matrix @ np.diag(roller_inertia) @ matrix.T + roller_mass*parallel_axis(roller.pos)
            hub_mass = mass - 16 * roller_mass
            center = mass * center / hub_mass
            inertia = remaining - hub_mass * parallel_axis(center)
            mass = hub_mass
        set_inertia(body, mass, center, inertia)
    chassis = spec.geom('base_link_collision')
    chassis.size = [positive(contact['chassis_radius'], 'chassis_radius'),
                    positive(contact['chassis_half_height'], 'chassis_half_height'), 0]
    chassis.pos = contact['chassis_center']
    for actuator in spec.actuators:
        name = actuator.target
        # Payload servos are configured by the payload's own package (pt_mujoco) when attached.
        if not name.endswith('wheel_joint'):
            continue
        joint = spec.joint(name)
        settings = simulation['actuators']['wheel']
        joint.armature = positive(settings['armature'], name + ' armature', True)
        joint.frictionloss = positive(settings['frictionloss'], name + ' frictionloss', True)
        effort = positive(urdf.find(f"joint[@name='{name}']/limit").get('effort'), name + ' effort')
        actuator.forcelimited = True
        actuator.forcerange = [-effort, effort]
        gain = positive(settings['velocity_gain'], 'velocity_gain')
        actuator.gainprm[0], actuator.biasprm[2] = gain, -gain
