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


class TestRecordFrames:

    def test_frames_default_to_the_unprefixed_names(self, node):
        assert node.get_parameter('frame_id').value == 'map'
        assert node.get_parameter('base_frame').value == 'base_footprint'

    @pytest.mark.parametrize('prefix', ['', 'robot1/'])
    def test_waypoint_is_recorded_between_the_configured_frames(self, node, prefix):
        from geometry_msgs.msg import TransformStamped
        from unittest.mock import patch
        from std_srvs.srv import SetBool
        node._frame_id = f'{prefix}map'
        node._base_frame = f'{prefix}base_footprint'
        node._goal_handle = None
        pose = TransformStamped()
        pose.transform.translation.x, pose.transform.translation.y = 1.0, 2.0
        pose.transform.rotation.w = 1.0
        with patch('lekiwi_navigation.waypoint_recorder_node.lookup_transform', return_value=pose) as lookup:
            response = node._handle_record(SetBool.Request(data=True), SetBool.Response())
        lookup.assert_called_once_with(f'{prefix}map', f'{prefix}base_footprint')
        assert response.success
        assert node._waypoints[-1].header.frame_id == f'{prefix}map'
