"""Cross-client behavior and single-instance regression checks."""
from pathlib import Path

import mujoco
import numpy as np
import pytest

from lekiwi_mujoco.simulation import Simulation


@pytest.fixture(scope='module')
def model():
    return mujoco.MjModel.from_xml_path(str(Path(__file__).resolve().parents[1] / 'mjcf/lekiwi_base.xml'))


def same(a, b):
    for key in ('qpos', 'qvel', 'ctrl', 'sensordata'):
        np.testing.assert_allclose(getattr(a, key), getattr(b, key), atol=1e-12, rtol=0)
    assert a.time == pytest.approx(b.time)


def test_batching_reset_and_wheel_commands(model):
    a, b = Simulation(model), Simulation(model)
    a.reset()
    b.reset()
    action = [.3, -.5, .2]
    a.step(20, action)
    for _ in range(20):
        b.step(action=action)
    same(a.data, b.data)
    a.stop()
    a.step(4)
    assert np.all(a.data.ctrl[a.control.wheels] == 0)
    a.reset()
    b.reset()
    same(a.data, b.data)
    a.step(20, action)
    b.step(20, action)
    same(a.data, b.data)


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
        sim.step(action=[0, np.nan, 0])
