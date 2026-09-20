"""ROS/GUI-independent commands, stepping, reset and state for a single robot."""
import math
from types import MappingProxyType

import mujoco
import numpy as np
import yaml

from lekiwi_mujoco.paths import package_share, payload_package

WHEELS = ('left_wheel_joint', 'back_wheel_joint', 'right_wheel_joint')


def _laser_filter_params(has_payload):
    """Read the same filter chain the real robot's laser_filters node applies
    (lekiwi_mujoco/config/mujoco_laser_filter[_pantilt].yaml), so the standalone sim's
    lidar_scan() matches what /scan looks like on hardware or in the ROS sim - without
    duplicating the threshold/mask numbers here and letting them drift out of sync."""
    name = 'mujoco_laser_filter_pantilt.yaml' if has_payload else 'mujoco_laser_filter.yaml'
    filters = yaml.safe_load((package_share('lekiwi_mujoco') / 'config' / name).read_text())
    filters = filters['laser_scan_filter_chain']['ros__parameters']
    no_hit_threshold, mask = None, None
    for value in filters.values():
        if not isinstance(value, dict) or 'type' not in value:
            continue
        if value['type'] == 'laser_filters/LaserScanRangeFilter':
            no_hit_threshold = float(value['params']['lower_threshold'])
        elif value['type'] == 'laser_filters/LaserScanAngularBoundsFilterInPlace':
            mask = (float(value['params']['lower_angle']), float(value['params']['upper_angle']))
    if no_hit_threshold is None:
        raise ValueError(f'{name}: no LaserScanRangeFilter (no_hit_to_inf) entry found')
    return no_hit_threshold, mask


def lidar_scan(bindings, sensordata):
    """Filtered LD06 ranges, or None if this model has no lidar.

    Mirrors the real robot's laser_filters chain: MuJoCo's -1 'no hit' sentinel becomes
    +inf per REP-117 (bindings.lidar_filter's no_hit_to_inf threshold). The payload's blind
    arc needs no masking here - when a payload is mounted, build_mujoco_models's
    mask_payload_lidar() already removed those rangefinders from the model entirely (see
    its docstring), so MuJoCo's own native rangefinder visualization is correct un-aided,
    and bindings.lidar_adr simply doesn't include them.
    """
    if bindings.lidar_filter is None:
        return None
    no_hit_threshold, _mask = bindings.lidar_filter
    ranges = sensordata[bindings.lidar_adr].copy()
    ranges[ranges < no_hit_threshold] = np.inf
    return ranges


def payload_names():
    # pt_mujoco is only needed for models that carry the payload, so the base runs without it
    try:
        payload_package()
        from pt_mujoco.simulation import PAYLOAD
    except (ImportError, FileNotFoundError):
        return None
    return PAYLOAD


def finite_vector(value, size, label):
    value = np.asarray(value, dtype=float)
    if value.shape != (size,) or not np.isfinite(value).all():
        raise ValueError(f'{label} must contain {size} finite values')
    return value


class RobotBindings:
    """Resolve the single robot's named elements once against a compiled model."""
    def __init__(self, model):
        self.model = model

        def required(kind, name):
            result = mujoco.mj_name2id(model, kind, name)
            if result < 0:
                raise ValueError(f'Missing robot element: {name}')
            return result

        self.base = required(mujoco.mjtObj.mjOBJ_BODY, 'base_link')
        self.wheels = tuple(required(mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in WHEELS)
        payload = payload_names()
        if payload is None and model.nu > len(WHEELS):
            raise ValueError('This model has a payload but pt_mujoco is not available')
        payload = payload or ()
        self.payload = tuple(name for name in payload
                             if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) >= 0)
        if self.payload and self.payload != payload:
            raise ValueError('Pan-tilt model must contain both payload actuators')
        names = WHEELS + self.payload
        self.actuator_ids = MappingProxyType({name: required(mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in names})
        self.joint_ids = MappingProxyType({name: required(mujoco.mjtObj.mjOBJ_JOINT, name) for name in names})
        for name in names:
            if model.actuator_trnid[self.actuator_ids[name], 0] != self.joint_ids[name]:
                raise ValueError(f'Actuator/joint mismatch: {name}')
        self.sensor_ids = self._named_ids(mujoco.mjtObj.mjOBJ_SENSOR, model.nsensor)
        self.camera_ids = self._named_ids(mujoco.mjtObj.mjOBJ_CAMERA, model.ncam)
        # lidar-0..lidar-N-1, one rangefinder per degree CCW from +X (base_shared.xml);
        # sensor_adr (not the sensor id) indexes sensordata, matching base_subtree.xml's comment.
        lidar = sorted(((int(name.rsplit('-', 1)[1]), i) for name, i in self.sensor_ids.items()
                       if name.startswith('lidar-')))
        self.lidar_adr = np.array([model.sensor_adr[i] for _, i in lidar], dtype=int)
        self.lidar_filter = _laser_filter_params(bool(self.payload)) if lidar else None

    def _named_ids(self, kind, count):
        return MappingProxyType({name: i for i in range(count)
                                 if (name := mujoco.mj_id2name(self.model, kind, i))})

    def numeric(self, name):
        result = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_NUMERIC, name)
        if result < 0:
            raise ValueError(f'Missing robot metadata: {name}; regenerate the model')
        return self.model.numeric(result).data.copy()


class RobotControl:
    """Common command semantics. Limits constrain commands, not physical state."""
    def __init__(self, model, bindings=None):
        self.model = model
        self.bindings = bindings or RobotBindings(model)
        self.wheels = list(self.bindings.wheels)
        self.base_limits = finite_vector(self.bindings.numeric('base_velocity_limits'), 3, 'base limits')
        self.wheel_limits = np.array([self.bindings.numeric('velocity_limit_' + name)[0] for name in WHEELS])
        radius, base_radius, offset = finite_vector(self.bindings.numeric('wheel_kinematics'), 3, 'kinematics')
        if min(radius, base_radius) <= 0 or np.any(self.base_limits <= 0) or not np.isfinite(self.wheel_limits).all() or np.any(self.wheel_limits <= 0):
            raise ValueError('Radii and velocity limits must be positive')
        angles = offset + np.arange(3) * 2 * np.pi / 3
        self.kinematics = np.column_stack((np.sin(angles), -np.cos(angles), np.full(3, -base_radius))) / radius
        self.payload_limits = {name: float(self.bindings.numeric('velocity_limit_' + name)[0]) for name in self.bindings.payload}
        if any(not np.isfinite(v) or v <= 0 for v in self.payload_limits.values()):
            raise ValueError('Payload velocity limits must be positive and finite')
        self.previous_targets = {name: float(np.clip(0, *model.actuator_ctrlrange[self.bindings.actuator_ids[name]]))
                                 for name in self.payload_limits}

    def wheel_speeds(self, command):
        command = finite_vector(command, 3, 'body velocity')
        rates = self.kinematics @ command
        return self.limit_wheels(rates)

    def limit_wheels(self, rates):
        rates = finite_vector(rates, 3, 'wheel rates')
        body = np.linalg.solve(self.kinematics, rates)
        scale = max(1., float(np.max(np.abs(rates) / self.wheel_limits)),
                    float(np.max(np.abs(body) / self.base_limits)))
        return rates / scale

    def action(self, value):
        return np.clip(finite_vector(value, 3 + len(self.payload_limits), 'action'), -1, 1)

    def integrate_payload(self, data, rates, dt):
        """Form requested targets from normalized rates; physics enforces them separately."""
        if not np.isfinite(dt) or dt < 0:
            raise ValueError('Target interval must be finite and nonnegative')
        rates = finite_vector(rates, len(self.payload_limits), 'payload rates')
        for rate, (name, limit) in zip(np.clip(rates, -1, 1), self.payload_limits.items()):
            actuator = self.bindings.actuator_ids[name]
            data.ctrl[actuator] = np.clip(data.ctrl[actuator] + rate * limit * dt,
                                          *self.model.actuator_ctrlrange[actuator])

    def apply(self, data, requested, dt):
        if not np.isfinite(dt) or dt < 0:
            raise ValueError('Control interval must be finite and nonnegative')
        requested = finite_vector(requested, self.model.nu, 'actuator commands')
        data.ctrl[self.wheels] = self.limit_wheels(requested[self.wheels])
        for name, limit in self.payload_limits.items():
            actuator = self.bindings.actuator_ids[name]
            low, high = self.model.actuator_ctrlrange[actuator]
            previous = self.previous_targets[name]
            target = np.clip(requested[actuator], max(low, previous-limit*dt), min(high, previous+limit*dt))
            data.ctrl[actuator] = target
            self.previous_targets[name] = float(target)

    def reset_targets(self, data):
        self.previous_targets = {name: float(np.clip(data.ctrl[self.bindings.actuator_ids[name]],
                                                    *self.model.actuator_ctrlrange[self.bindings.actuator_ids[name]]))
                                 for name in self.payload_limits}
        for name, value in self.previous_targets.items():
            data.ctrl[self.bindings.actuator_ids[name]] = value


class Simulation:
    """One model/data owner. No wall-clock pacing, rendering, keyboard or ROS imports.

    step(count, action) holds a normalized rate action for count physics ticks.
    step(count) accepts actuator edits in data.ctrl (e.g. viewer sliders); position
    requests persist until reached or replaced, with slew limits at each tick.
    """
    def __init__(self, model):
        self.model = model
        self.data = mujoco.MjData(model)
        self.robot = RobotBindings(model)
        self.control = RobotControl(model, self.robot)
        self.ready = False
        self._requested = self.data.ctrl.copy()
        self._applied = self.data.ctrl.copy()

    def steps_for(self, seconds):
        if not np.isfinite(seconds) or seconds <= 0:
            raise ValueError('Control interval must be finite and positive')
        ratio = seconds / self.model.opt.timestep
        if abs(ratio - round(ratio)) > 1e-8 or round(ratio) < 1:
            raise ValueError('Control interval must be an integer multiple of the physics timestep')
        return round(ratio)

    def reset(self, settle_seconds=.5):
        if not np.isfinite(settle_seconds) or settle_seconds < 0:
            raise ValueError('Settling duration must be finite and nonnegative')
        mujoco.mj_resetData(self.model, self.data)
        self.control.reset_targets(self.data)
        self._requested = self.data.ctrl.copy()
        self._applied = self.data.ctrl.copy()
        self.ready = True
        # Settling uses the same bounded controls and stepping as normal execution.
        self.step(round(settle_seconds / self.model.opt.timestep))
        self.data.time = 0.
        mujoco.mj_forward(self.model, self.data)
        return self.observation()

    def command(self, body_velocity):
        self.data.ctrl[self.control.wheels] = self.control.wheel_speeds(body_velocity)
        self._requested[self.control.wheels] = self.data.ctrl[self.control.wheels]

    def stop(self):
        """Zero wheel command and hold current payload targets; do not teleport state."""
        self.data.ctrl[self.control.wheels] = 0.
        self._requested = self.data.ctrl.copy()
        self._applied = self.data.ctrl.copy()
        self.control.reset_targets(self.data)

    def step(self, count=1, action=None):
        if not self.ready:
            raise RuntimeError('Call reset() before stepping')
        if not isinstance(count, (int, np.integer)) or isinstance(count, bool) or count < 0:
            raise ValueError('Physics step count must be a nonnegative integer')
        action = self.control.action(action) if action is not None else None
        if not np.isfinite(self.data.ctrl).all():
            raise ValueError('Actuator commands must be finite')
        changed = self.data.ctrl != self._applied
        self._requested[changed] = self.data.ctrl[changed]
        for _ in range(count):
            if action is not None:
                self.data.ctrl[self.control.wheels] = self.control.wheel_speeds(action[:3] * self.control.base_limits)
                self.control.integrate_payload(self.data, action[3:], self.model.opt.timestep)
                self._requested = self.data.ctrl.copy()
            self.control.apply(self.data, self._requested, self.model.opt.timestep)
            mujoco.mj_step(self.model, self.data)
        self._applied = self.data.ctrl.copy()
        mujoco.mj_forward(self.model, self.data)
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all() or not np.isfinite(self.data.sensordata).all():
            raise FloatingPointError('Non-finite MuJoCo state')

    def pose(self):
        rotation = self.data.xmat[self.robot.base].reshape(3, 3)
        return np.r_[self.data.xpos[self.robot.base, :2], math.atan2(rotation[1, 0], rotation[0, 0])]

    def scan(self):
        """Filtered LD06 ranges (see lidar_scan()), or None if this model has no lidar."""
        return lidar_scan(self.robot, self.data.sensordata)

    def observation(self):
        return {'qpos': self.data.qpos.copy(), 'qvel': self.data.qvel.copy(),
                'sensors': self.data.sensordata.copy(), 'scan': self.scan()}

    def info(self):
        return {'sim_time': float(self.data.time), 'actuator_commands': self.data.ctrl.copy(),
                'contacts': int(self.data.ncon), 'warning_count': sum(int(w.number) for w in self.data.warning)}
