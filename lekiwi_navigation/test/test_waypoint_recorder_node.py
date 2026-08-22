#!/usr/bin/env python3
"""Node-level unit tests for WaypointRecorderNode's _on_nav_feedback guard: feedback arriving
while paused for an external-detour handoff must not be attributed to patrol progress."""

from unittest.mock import MagicMock

import pytest

from lekiwi_navigation.waypoint_recorder_node import WaypointRecorderNode

# rclpy is initialized by conftest.py (session-scoped, idempotent).


@pytest.fixture
def node():
    n = WaypointRecorderNode()
    yield n
    n.destroy_node()


class TestOnNavFeedbackGuard:

    def test_ignored_when_not_patrolling(self, node):
        node._goal_handle = None
        msg = MagicMock()
        node._on_nav_feedback(msg)
        assert node._latest_nav_feedback is None

    def test_ignored_while_paused_for_detour(self, node):
        node._goal_handle = MagicMock()
        node._resume_timer = MagicMock()
        msg = MagicMock()
        node._on_nav_feedback(msg)
        assert node._latest_nav_feedback is None

    def test_processed_while_actively_patrolling(self, node):
        node._goal_handle = MagicMock()
        node._resume_timer = None
        node._publish_diagnostics = MagicMock()
        msg = MagicMock()
        node._on_nav_feedback(msg)
        assert node._latest_nav_feedback is msg.feedback
        node._publish_diagnostics.assert_called_once()
