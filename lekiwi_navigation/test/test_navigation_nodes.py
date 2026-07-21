#!/usr/bin/env python3
"""
Node-level unit tests for lekiwi_navigation Python nodes.

CollisionToggleNode button edge detection.
Tests the observable state changes in _last_state without needing a live service.
"""

import pytest
import rclpy
from sensor_msgs.msg import Joy
from unittest.mock import MagicMock

from lekiwi_navigation.collision_toggle_node import CollisionToggleNode


# rclpy is initialized by conftest.py (session-scoped, idempotent).
_BUTTON = 10   # R1, matches collision_toggle.yaml


def _joy(buttons):
    msg = Joy()
    msg.buttons = list(buttons)
    return msg


def _pressed():
    """Joy message with button 10 pressed."""
    return _joy([0] * _BUTTON + [1])


def _released():
    """Joy message with all buttons released (including button 10)."""
    return _joy([0] * (_BUTTON + 1))


@pytest.fixture
def node():
    n = CollisionToggleNode()
    # Replace client so _set_enabled does not try to contact a live service.
    mock_client = MagicMock()
    mock_client.service_is_ready.return_value = False
    n._client = mock_client
    yield n
    n.destroy_node()


class TestCollisionToggleButtonEdge:
    """Tests the _last_state gating inside _joy_callback."""

    def test_initial_state_is_released(self, node):
        assert node._last_state is False

    def test_press_updates_last_state(self, node):
        node._joy_callback(_pressed())
        assert node._last_state is True

    def test_hold_does_not_change_last_state(self, node):
        node._joy_callback(_pressed())
        node._joy_callback(_pressed())   # same state
        assert node._last_state is True  # still True

    def test_release_updates_last_state(self, node):
        node._joy_callback(_pressed())
        node._joy_callback(_released())
        assert node._last_state is False

    def test_out_of_range_button_ignored(self, node):
        node._joy_callback(_joy([]))     # empty buttons list
        assert node._last_state is False  # unchanged

    def test_press_calls_set_enabled_false(self, node):
        """On press, _set_enabled(False) disables the collision stop."""
        calls = []
        node._set_enabled = lambda enabled: calls.append(enabled)
        node._joy_callback(_pressed())
        assert calls == [False]

    def test_release_calls_set_enabled_true(self, node):
        """On release, _set_enabled(True) re-enables the collision stop."""
        # First press so _last_state=True, then patch, then release.
        node._joy_callback(_pressed())   # state -> True (service_is_ready=False so no call)
        calls = []
        node._set_enabled = lambda enabled: calls.append(enabled)
        node._joy_callback(_released())
        assert calls == [True]
