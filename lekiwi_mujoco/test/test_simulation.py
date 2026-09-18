"""Cross-client behavior and prefixed single-instance regression checks."""
from pathlib import Path

import glfw
import mujoco
import numpy as np
import pytest

from lekiwi_mujoco.build_mujoco_models import build
from lekiwi_mujoco.instance import RobotInstance
from lekiwi_mujoco.mujoco_env import LeKiwiEnv
from lekiwi_mujoco.mujoco_preview import KeyboardControl
from lekiwi_mujoco.simulation import Simulation


@pytest.fixture(scope='module')
def model():
    return mujoco.MjModel.from_xml_path(str(Path(__file__).resolve().parents[1] / 'mjcf/lekiwi_pt101_oakd_s2.xml'))


def same(a, b):
    for key in ('qpos', 'qvel', 'ctrl', 'sensordata'):
        np.testing.assert_allclose(getattr(a, key), getattr(b, key), atol=1e-12, rtol=0)
    assert a.time == pytest.approx(b.time)


@pytest.mark.parametrize('keys,action', [
    ({glfw.KEY_UP}, [1, 0, 0, 0, 0]),
    ({glfw.KEY_LEFT}, [0, 1, 0, 0, 0]),
    ({glfw.KEY_LEFT_SHIFT, glfw.KEY_LEFT}, [0, 0, 1, 0, 0]),
    ({glfw.KEY_LEFT_ALT, glfw.KEY_LEFT, glfw.KEY_UP}, [0, 0, 0, 1, 1]),
])
def test_keyboard_and_gym_share_physics(model, keys, action):
    with LeKiwiEnv(model_path=Path(__file__).resolve().parents[1] / 'mjcf/lekiwi_pt101_oakd_s2.xml') as env:
        sim = Simulation(model)
        sim.reset()
        env.reset()
        keyboard = KeyboardControl(model, sim.control)
        for _ in range(4):
            env.step(action)
            for _ in range(env.frame_skip):
                keyboard.update(sim.data, keys, model.opt.timestep)
                sim.step()
        same(sim.data, env.data)
        keyboard.update(sim.data, set(), model.opt.timestep)
        sim.step()
        env.simulation.step(action=np.zeros(5))
        same(sim.data, env.data)


def test_batching_reset_and_slider_goal(model):
    a, b = Simulation(model), Simulation(model)
    a.reset()
    b.reset()
    action = [.3, -.5, .2, 1, -1]
    a.step(20, action)
    for _ in range(20):
        b.step(action=action)
    same(a.data, b.data)
    pan = a.robot.actuator_ids['shoulder_pan_joint']
    a.data.ctrl[pan] = .8
    start = a.control.previous_targets['shoulder_pan_joint']
    a.step(3)
    assert a.data.ctrl[pan] == pytest.approx(start + 3 * model.opt.timestep * a.control.payload_limits['shoulder_pan_joint'])
    a.stop()
    held = a.data.ctrl[pan]
    a.step(4)
    assert a.data.ctrl[pan] == held
    assert np.all(a.data.ctrl[a.control.wheels] == 0)
    a.reset()
    b.reset()
    same(a.data, b.data)
    a.step(20, action)
    b.step(20, action)
    same(a.data, b.data)


@pytest.mark.parametrize('variant', ['base', 'pt100', 'pt101'])
def test_prefixed_spawn_serialization_and_reset(tmp_path, variant):
    instance = RobotInstance(prefix='robot/', position=(1, 2, .1), quaternion=(1, 0, 0, 1))
    path = build(variant, tmp_path / 'robot.xml', absolute=True, instance=instance)
    model = mujoco.MjModel.from_xml_path(str(path))
    sim = Simulation(model, instance)
    sim.reset(settle_seconds=0)
    assert sim.pose() == pytest.approx([1, 2, np.pi / 2])
    root = sim.robot.free_qpos
    np.testing.assert_allclose(sim.data.qpos[root:root+7], sim.robot.spawn_qpos)
    assert sim.robot.sensor_ids
    assert bool(sim.robot.camera_ids) == (variant != 'base')
    sim.step(5, np.ones(3 + len(sim.robot.payload)))
    sim.reset(settle_seconds=0)
    np.testing.assert_allclose(sim.data.qpos[root:root+7], sim.robot.spawn_qpos)
    with pytest.raises(ValueError, match='Missing robot element'):
        Simulation(model)
    with LeKiwiEnv(model_path=path, instance=instance) as env:
        env.reset()
        env.step(np.ones(env.action_space.shape))
        assert env.data.time == pytest.approx(.02)


def test_invalid_execution_inputs(model):
    sim = Simulation(model)
    with pytest.raises(RuntimeError):
        sim.step()
    sim.reset(0)
    for count in (-1, .5, True):
        with pytest.raises(ValueError):
            sim.step(count)
    for interval in (0, np.nan, .003):
        with pytest.raises(ValueError):
            sim.steps_for(interval)
    with pytest.raises(ValueError):
        sim.step(action=[0, 0, 0, np.nan, 0])
    with pytest.raises(ValueError):
        RobotInstance(quaternion=(0, 0, 0, 0))
