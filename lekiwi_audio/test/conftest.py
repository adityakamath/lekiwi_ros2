"""Shared pytest fixtures for lekiwi_audio tests."""
import pytest
import rclpy


@pytest.fixture(scope='session', autouse=True)
def ros_init():
    """Initialize rclpy once for the whole test session, idempotently."""
    if not rclpy.ok():
        rclpy.init()
    yield
    rclpy.try_shutdown()
