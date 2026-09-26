"""Apartment props must remain pushable when composed with the actual robot."""
from pathlib import Path
import sys

import pytest

mujoco = pytest.importorskip('mujoco')
np = pytest.importorskip('numpy')
PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from lekiwi_mujoco.build_mujoco_models import build  # noqa: E402
from lekiwi_mujoco.simulation import Simulation  # noqa: E402


@pytest.fixture(scope='module')
def apartment_model(tmp_path_factory):
    path = tmp_path_factory.mktemp('apartment') / 'robot.xml'
    return mujoco.MjModel.from_xml_path(str(build('base', path, absolute=True, scene='home')))


@pytest.mark.parametrize('prop', ['cardboard_box', 'foam_block', 'toy_ball'])
def test_robot_pushes_prop_and_reset_restores_it(apartment_model, prop):
    sim = Simulation(apartment_model)
    sim.reset()
    body = apartment_model.body('home_' + prop).id
    joint = apartment_model.joint('home_' + prop + '_free').id
    adr = apartment_model.jnt_qposadr[joint]
    home = sim.data.qpos[adr:adr + 7].copy()
    # Put each prop on the same clear approach, leaving its settled height intact.
    sim.data.qpos[adr:adr + 2] = [0.8, 0.0]
    # Park the other props away from this controlled interaction.
    for other, xy in [('cardboard_box', [-.8, .8]),
                      ('foam_block', [-.6, .8]), ('toy_ball', [-.4, .8])]:
        if other != prop:
            j = apartment_model.joint('home_' + other + '_free').id
            a = apartment_model.jnt_qposadr[j]
            sim.data.qpos[a:a + 2] = xy
    mujoco.mj_forward(apartment_model, sim.data)
    start = sim.data.xpos[body].copy()
    sim.command([0.15, 0., 0.])
    sim.step(3500)
    assert sim.data.xpos[body, 0] - start[0] > .15
    assert np.isfinite(sim.data.qpos).all()
    assert abs(sim.data.xpos[body, 2] - start[2]) < .1
    sim.reset()
    np.testing.assert_allclose(sim.data.qpos[adr:adr + 7], home, atol=1e-6)
