"""Derive Nav2 speed limits from the joystick configuration's axis magnitudes."""
import math

import yaml


def velocity_overrides(teleop_path):
    """Return per-node parameter overrides; reject missing or unsafe axis scales."""
    with open(teleop_path) as stream:
        config = yaml.safe_load(stream)
    axes = config['joy_teleop']['ros__parameters']['teleop']['axis_mappings']
    limits = []
    for name in ('twist-linear-x', 'twist-linear-y', 'twist-angular-z'):
        scale = axes[name]['scale']
        if isinstance(scale, bool) or not isinstance(scale, (int, float)):
            raise ValueError(f'{name}.scale must be numeric')
        if not math.isfinite(scale) or scale == 0:
            raise ValueError(f'{name}.scale must be finite and nonzero')
        if axes[name].get('offset', 0.0) != 0:
            raise ValueError(f'{name}.offset must be zero for symmetric velocity limits')
        limits.append(abs(float(scale)))
    x, y, yaw = limits
    return {
        'controller_server': {
            'FollowPath.vx_max': x, 'FollowPath.vx_min': -x,
            'FollowPath.vy_max': y, 'FollowPath.vy_min': -y,
            'FollowPath.wz_max': yaw, 'FollowPath.wz_min': -yaw,
        },
        'behavior_server': {'max_rotational_vel': yaw},
        'velocity_smoother': {'max_velocity': limits, 'min_velocity': [-v for v in limits]},
    }
