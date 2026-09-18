"""Task-neutral Gymnasium interface for one LeKiwi, without ROS or a training task.

Import this module to register LeKiwi/Mujoco-v0. Actions are normalized base
vx/vy/yaw rates followed, for payload models, by pan/tilt rates. Observations are
named ideal sensors by default; raw simulator state requires privileged mode.
Neither mode claims calibrated real-robot equivalence.
"""
from pathlib import Path
from tempfile import TemporaryDirectory

import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np

from lekiwi_mujoco.build_mujoco_models import build
from lekiwi_mujoco.simulation import Simulation
from lekiwi_mujoco.observations import ObservationContract


class LeKiwiEnv(gym.Env):
    metadata = {'render_modes': ['rgb_array'], 'render_fps': 50}

    def __init__(self, variant='pt101', model_path=None, render_mode=None,
                 control_dt=.02, width=640, height=480, scene='flat', control_dir=None, description_dir=None, pt_package=None, instance=None, observation_mode="sensors"):
        if render_mode not in (None, 'rgb_array'):
            raise ValueError('Use render_mode=None or rgb_array; use mujoco_preview.py for manual inspection')
        self._temporary = TemporaryDirectory(prefix='lekiwi_gym_') if model_path is None else None
        path = Path(model_path) if model_path else build(
            variant, Path(self._temporary.name) / 'model.xml', absolute=True, scene=scene, control_dir=control_dir, description_dir=description_dir, pt_package=pt_package, instance=instance)
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.simulation = Simulation(self.model, instance)
        self.data = self.simulation.data
        if observation_mode not in ("sensors", "privileged"):
            raise ValueError("observation_mode must be sensors or privileged")
        self.observation_mode = observation_mode
        self.contract = ObservationContract(self.model, self.simulation.robot)
        self.frame_skip = self.simulation.steps_for(control_dt)
        self.dt = self.frame_skip * self.model.opt.timestep
        self.metadata = {**type(self).metadata, 'render_fps': round(1 / self.dt)}
        self.render_mode = render_mode
        self.width, self.height = width, height
        self._renderer = None
        self.wheels = self.simulation.control.wheels
        self.payload = self.simulation.robot.payload
        self.base_limits = self.simulation.control.base_limits
        self.payload_limits = self.simulation.control.payload_limits
        self.action_space = spaces.Box(-1., 1., shape=(3 + len(self.payload),), dtype=np.float64)
        self.observation_space = spaces.Dict({
            'qpos': spaces.Box(-np.inf, np.inf, shape=(self.model.nq,), dtype=np.float64),
            'qvel': spaces.Box(-np.inf, np.inf, shape=(self.model.nv,), dtype=np.float64),
            'sensors': spaces.Box(-np.inf, np.inf, shape=(self.model.nsensordata,), dtype=np.float64),
        })

        if observation_mode == 'sensors':
            sample = self.contract.read(self.data)
            self.observation_space = spaces.Dict({
                key: (spaces.MultiBinary(value.shape) if key == 'lidar_valid' else
                      spaces.Box(-np.inf, np.inf, shape=value.shape, dtype=np.float64))
                for key, value in sample.items()})

    def _observation(self):
        if self.observation_mode == 'sensors':
            return self.contract.read(self.data)
        return self.simulation.observation()

    def _info(self):
        return {**self.simulation.info(), 'control_dt': self.dt}

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if options:
            raise ValueError('No task-specific reset options are defined')
        self.simulation.reset()
        return self._observation(), self._info()

    def step(self, action):
        if not self.simulation.ready:
            raise gym.error.ResetNeeded('Call reset() before step()')
        self.simulation.step(self.frame_skip, action)
        # Task rewards, termination and horizons belong in future wrappers.
        return self._observation(), 0., False, False, self._info()

    def render(self):
        if self.render_mode != 'rgb_array':
            return None
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)
        option = mujoco.MjvOption()
        option.geomgroup[3] = 0
        option.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
        camera = mujoco.MjvCamera()
        camera.lookat[:] = self.data.xpos[self.simulation.robot.base] + [0, 0, .18]
        camera.distance, camera.azimuth, camera.elevation = 1.25, 135, -22
        self._renderer.update_scene(self.data, camera, scene_option=option)
        return self._renderer.render().copy()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
        if self._temporary is not None:
            self._temporary.cleanup()
            self._temporary = None
        self.simulation.ready = False


if 'LeKiwi/Mujoco-v0' not in gym.registry:
    gym.register(id='LeKiwi/Mujoco-v0', entry_point='lekiwi_mujoco.mujoco_env:LeKiwiEnv')
