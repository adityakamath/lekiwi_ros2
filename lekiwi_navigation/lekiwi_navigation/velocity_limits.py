"""Derive Nav2 speed limits from the joystick configuration's axis magnitudes."""
import math

import yaml


def velocity_overrides(teleop_path, params_path=None):
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
    plugin_ids = ['FollowPath']
    if params_path is not None:
        with open(params_path) as stream:
            nav_params = yaml.safe_load(stream)['controller_server']['ros__parameters']
        plugin_ids = [plugin_id for plugin_id in nav_params['controller_plugins']
                      if nav_params[plugin_id]['plugin'] == 'nav2_mppi_controller::MPPIController']
        if not plugin_ids:
            raise ValueError('params_file must configure an MPPI controller plugin')
    controller_limits = {}
    for plugin_id in plugin_ids:
        controller_limits.update({
            f'{plugin_id}.vx_max': x, f'{plugin_id}.vx_min': -x,
            f'{plugin_id}.vy_max': y, f'{plugin_id}.vy_min': -y,
            f'{plugin_id}.wz_max': yaw, f'{plugin_id}.wz_min': -yaw,
        })
    return {
        'controller_server': controller_limits,
        'behavior_server': {'max_rotational_vel': yaw},
        'velocity_smoother': {'max_velocity': limits, 'min_velocity': [-v for v in limits]},
    }
