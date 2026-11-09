"""Shared pytest fixtures for lekiwi_control tests."""
from pathlib import Path

import pytest
import rclpy


@pytest.fixture(scope='session', autouse=True)
def ros_init():
    """Initialize rclpy once for the whole test session, idempotently."""
    if not rclpy.ok():
        rclpy.init()
    yield
    rclpy.try_shutdown()


def pantilt_available():
    """True when the legacy pantilt_ros2 payload is checked out or installed."""
    if (Path(__file__).resolve().parents[2] / 'payloads/pantilt_ros2/pt_description/package.xml').is_file():
        return True
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('pt_description')
        return True
    except Exception:
        return False


def pytest_configure(config):
    config.addinivalue_line('markers', 'pantilt: needs the pantilt_ros2 payload (skipped when it is not installed)')


def pytest_collection_modifyitems(config, items):
    """Skip tests marked `pantilt`, and any parametrized `pt101` case, when pantilt_ros2 is missing."""
    if pantilt_available():
        return
    skip = pytest.mark.skip(reason='pantilt_ros2 is not installed (legacy payload)')
    for item in items:
        if 'pantilt' in item.keywords or 'pt101' in item.nodeid:
            item.add_marker(skip)


@pytest.fixture(scope='session')
def pantilt():
    """Depend on this fixture to skip when pantilt_ros2 is missing."""
    if not pantilt_available():
        pytest.skip('pantilt_ros2 is not installed (legacy payload)')
