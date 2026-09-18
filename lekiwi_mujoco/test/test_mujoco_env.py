"""Gymnasium contract checks; no learning task or algorithm."""
from pathlib import Path
import sys

import pytest

gym = pytest.importorskip('gymnasium')
np = pytest.importorskip('numpy')
pytest.importorskip('mujoco')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lekiwi_mujoco.mujoco_env import LeKiwiEnv  # noqa: E402
from gymnasium.utils.env_checker import check_env  # noqa: E402


@pytest.mark.parametrize('variant', ['base', 'pt100', 'pt101'])
def test_gymnasium_contract_and_seeded_reset(variant):
    with LeKiwiEnv(variant=variant) as env:
        check_env(env, skip_render_check=True)
        initial, _ = env.reset(seed=123)
        observation, reward, terminated, truncated, info = env.step(np.ones(env.action_space.shape))
        assert env.observation_space.contains(observation)
        assert reward == 0. and not terminated and not truncated
        assert info['sim_time'] == pytest.approx(.02)
        again, _ = env.reset(seed=123)
        for key in initial:
            assert np.array_equal(initial[key], again[key])
        assert np.all(env.data.ctrl == 0)


def test_registration_limits_and_invalid_actions():
    with gym.make('LeKiwi/Mujoco-v0', variant='pt101', max_episode_steps=2) as env:
        env.reset(seed=1)
        raw = env.unwrapped
        for i in range(2):
            _, _, terminated, truncated, _ = env.step(np.full(5, 100.))
            assert not terminated and truncated == (i == 1)
        assert np.all(np.abs(raw.data.ctrl[raw.wheels]) <= 4.433205)
        for name in raw.payload:
            actuator = raw.model.actuator(name).id
            assert raw.data.ctrl[actuator] == pytest.approx(raw.payload_limits[name] * .04)
        with pytest.raises(ValueError):
            raw.step(np.full(5, np.nan))


def test_step_requires_reset_and_observations_are_copies():
    with LeKiwiEnv(variant='base') as env:
        with pytest.raises(gym.error.ResetNeeded):
            env.step(np.zeros(3))
        observation, _ = env.reset()
        observation['joint_position'][:] = 100
        assert not np.all(env.data.qpos == 100)
