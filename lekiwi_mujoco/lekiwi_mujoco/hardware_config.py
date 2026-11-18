"""Shared optional-hardware configuration for bringup, control and model generation."""
from pathlib import Path

import yaml

from lekiwi_mujoco.paths import package_share

HARDWARE_GROUPS = ('lidar', 'imu', 'battery_monitor', 'respeaker', 'sparkfun_sensor', 'gemini2', 'shelf', 'rpi5')
HARDWARE_DEFAULTS = dict.fromkeys(HARDWARE_GROUPS, True)


def load_hardware_config(path=None):
    path = Path(path) if path else package_share('lekiwi_bringup') / 'config/hardware.yaml'
    with path.open() as stream:
        settings = yaml.safe_load(stream)
    if not isinstance(settings, dict):
        raise ValueError(f'{path}: hardware configuration must be a mapping')
    unknown = set(settings) - set(HARDWARE_DEFAULTS)
    if unknown:
        raise ValueError(f'{path}: unknown hardware groups: {sorted(unknown)}')
    settings = {**HARDWARE_DEFAULTS, **settings}
    for name, enabled in settings.items():
        if not isinstance(enabled, bool):
            raise ValueError(f'{path}: {name} must be a YAML boolean (true or false)')
    return settings
