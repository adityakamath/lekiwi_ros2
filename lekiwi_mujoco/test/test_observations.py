"""Observable state excludes free-base and passive-roller privileged state."""
import numpy as np
import pytest
from lekiwi_mujoco.mujoco_env import LeKiwiEnv


@pytest.mark.parametrize('variant,count', [('base', 3), ('pt101', 5)])
def test_sensor_contract(variant, count):
    with LeKiwiEnv(variant=variant) as env:
        obs, _ = env.reset()
        assert 'qpos' not in obs and 'qvel' not in obs
        assert len(obs['joint_position']) == count
        assert len(obs['lidar_range']) == 360
        assert env.observation_space.contains(obs)
        meta = env.contract.metadata()
        assert len(meta['lidar_rays']) == 360
        assert all('roller' not in n for n in meta['joint_names'])
        # MuJoCo negative no-hit sentinel is represented by an explicit mask.
        sensor = env.simulation.robot.sensor_ids['lidar-0']
        env.data.sensordata[env.model.sensor_adr[sensor]] = -1
        sample = env.contract.read(env.data)
        assert sample['lidar_valid'][0] == 0 and sample['lidar_range'][0] == 0
        obs, *_ = env.step(np.zeros(count))
        assert obs['time'][0] == pytest.approx(env.dt)
        obs, _ = env.reset()
        assert obs['time'][0] == 0


def test_privileged_state_is_explicit():
    with LeKiwiEnv(variant='base', observation_mode='privileged') as env:
        obs, _ = env.reset()
        assert set(obs) == {'qpos', 'qvel', 'sensors'}
        assert env.observation_space.contains(obs)
