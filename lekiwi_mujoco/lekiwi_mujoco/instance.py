"""Description and resolved handles for one robot; no fleet manager."""
from dataclasses import dataclass
from types import MappingProxyType

import mujoco
import numpy as np

WHEELS = ('left_wheel_joint', 'back_wheel_joint', 'right_wheel_joint')
PAYLOAD = ('shoulder_pan_joint', 'tilt_joint')


@dataclass(frozen=True)
class RobotInstance:
    """Spawn transform relative to the robot's nominal ground footprint, wxyz quaternion.

    Applied during MjSpec composition only. Binding an existing model never moves it;
    its compiled qpos0 remains the reset pose.
    """
    name: str = 'lekiwi'
    prefix: str = ''
    position: tuple = (0., 0., 0.)
    quaternion: tuple = (1., 0., 0., 0.)

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name or not isinstance(self.prefix, str):
            raise ValueError('Robot name must be nonempty and prefix must be a string')
        p = np.asarray(self.position, dtype=float)
        q = np.asarray(self.quaternion, dtype=float)
        if p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or not np.isfinite(q).all():
            raise ValueError('Spawn requires finite xyz position and wxyz quaternion')
        norm = np.linalg.norm(q)
        if not np.isfinite(norm) or norm < 1e-12:
            raise ValueError('Spawn quaternion cannot be zero')
        object.__setattr__(self, 'position', tuple(p))
        object.__setattr__(self, 'quaternion', tuple(q / norm))

    def resolve(self, model):
        return RobotBindings(model, self)


class RobotBindings:
    """Resolve names once against a model; canonical keys map to prefixed model IDs."""
    def __init__(self, model, instance):
        self.model, self.instance = model, instance
        prefix = instance.prefix

        def required(kind, name):
            result = mujoco.mj_name2id(model, kind, prefix + name)
            if result < 0:
                raise ValueError(f'Missing robot element: {prefix + name}')
            return result

        self.base = required(mujoco.mjtObj.mjOBJ_BODY, 'base_link')
        joints = range(model.body_jntadr[self.base], model.body_jntadr[self.base] + model.body_jntnum[self.base])
        self.free_joint = next((j for j in joints if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE), None)
        if self.free_joint is None:
            raise ValueError('Robot base must have a free joint')
        self.free_qpos = int(model.jnt_qposadr[self.free_joint])
        self.spawn_qpos = tuple(model.qpos0[self.free_qpos:self.free_qpos + 7])
        self.wheels = tuple(required(mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in WHEELS)
        self.payload = tuple(name for name in PAYLOAD
                             if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, prefix + name) >= 0)
        if self.payload and self.payload != PAYLOAD:
            raise ValueError('Pan-tilt model must contain both payload actuators')
        names = WHEELS + self.payload
        self.actuator_ids = MappingProxyType({name: required(mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in names})
        self.joint_ids = MappingProxyType({name: required(mujoco.mjtObj.mjOBJ_JOINT, name) for name in names})
        for name in names:
            if model.actuator_trnid[self.actuator_ids[name], 0] != self.joint_ids[name]:
                raise ValueError(f'Actuator/joint mismatch: {prefix + name}')
        self.sensor_ids = self._named_ids(mujoco.mjtObj.mjOBJ_SENSOR, model.nsensor)
        self.camera_ids = self._named_ids(mujoco.mjtObj.mjOBJ_CAMERA, model.ncam)

    def _named_ids(self, kind, count):
        prefix = self.instance.prefix
        return MappingProxyType({name[len(prefix):]: i for i in range(count)
                                 if (name := mujoco.mj_id2name(self.model, kind, i)) and name.startswith(prefix)})

    def numeric(self, name):
        name = self.instance.prefix + name
        result = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_NUMERIC, name)
        if result < 0:
            raise ValueError(f'Missing robot metadata: {name}; regenerate the model')
        return self.model.numeric(result).data.copy()
