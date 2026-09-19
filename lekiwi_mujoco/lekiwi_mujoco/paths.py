"""Locate data in a checkout or installed prefix without importing ROS."""
import os
from pathlib import Path
import sys


def package_share(name):
    override = os.environ.get(name.upper() + '_SHARE')
    if override:
        path = Path(override).expanduser().resolve()
        if not path.is_dir():
            raise FileNotFoundError(f'{name} data directory does not exist: {path}')
        return path
    root = Path(__file__).resolve().parents[2]
    candidates = [root / name, root / 'payloads/pantilt_ros2' / name]
    for prefix in [*os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep), sys.prefix]:
        if prefix:
            candidates.append(Path(prefix) / 'share' / name)
    for path in candidates:
        if (path / 'package.xml').is_file():
            return path.resolve()
    raise FileNotFoundError(f'Cannot locate {name}; set {name.upper()}_SHARE to its source/share directory')


def payload_package():
    """Make pt_mujoco importable: from an install or, without one, from the source checkout."""
    try:
        import pt_mujoco  # noqa: F401
    except ImportError:
        sys.path.insert(0, str(package_share('pt_mujoco')))
