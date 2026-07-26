#!/usr/bin/env python3
"""Node-level unit tests for lekiwi_audio's IndicatorNode.

Uses rclpy directly (no launch_testing, no live service graph) by initializing
rclpy once per session via a session-scoped fixture (conftest.py). Tests
exercise pure logic methods that don't require a live ROS graph - no service
calls, no subscribers actually firing.
"""

import pytest
from action_msgs.msg import GoalStatus, GoalStatusArray
from lekiwi_audio.indicator_node import IndicatorNode, _phrase_filename
from lekiwi_audio.render_phrases import phrase_filename as render_phrase_filename

# rclpy is initialized by conftest.py (session-scoped, idempotent).


@pytest.fixture
def node():
    """Create an IndicatorNode with default parameters and destroy it after the test."""
    n = IndicatorNode()
    yield n
    n.destroy_node()


class TestPhraseFilenameConsistency:
    """Verify the two independent hash implementations always agree.

    indicator_node._phrase_filename() and render_phrases.phrase_filename()
    must always agree - they're the only link between build-time rendering
    and runtime lookup, with no separate manifest file.
    """

    def test_same_hash_for_same_phrase(self):
        for phrase in ('Emergency stop enabled', 'Waypoint recorded', 'Error'):
            assert _phrase_filename(phrase) == render_phrase_filename(phrase)

    def test_different_phrases_produce_different_filenames(self):
        assert _phrase_filename('Teleop mode') != _phrase_filename('Autonomous mode')


class TestPhraseFor:

    def test_true_success_returns_configured_phrase(self, node):
        assert node._phrase_for('/emergency_stop', True, True) == 'Emergency stop enabled'

    def test_false_success_returns_configured_phrase(self, node):
        assert node._phrase_for('/emergency_stop', False, True) == 'Emergency stop disabled'

    def test_false_with_no_entry_is_silent(self, node):
        # /record_waypoint's false case is a deliberate no-op (confirmed in the
        # real handler) - no phrase configured, nothing should be announced.
        assert node._phrase_for('/record_waypoint', False, True) is None

    def test_true_failure_falls_back_to_configured_error_phrase(self, node):
        assert node._phrase_for('/record_waypoint', True, False) == 'Error'

    def test_true_failure_with_no_failure_entry_falls_back_to_generic_error(self, node):
        # /reset_waypoints has no explicit "failure" key in phrases.yaml -
        # should still announce something rather than go silent.
        assert node._phrase_for('/reset_waypoints', True, False) == 'Error'

    def test_unknown_service_is_silent(self, node):
        assert node._phrase_for('/not_a_real_service', True, True) is None


class TestPhraseForNavGoal:

    def test_succeeded_returns_configured_phrase(self, node):
        assert node._phrase_for_nav_goal(GoalStatus.STATUS_SUCCEEDED) == 'Goal reached'

    def test_aborted_returns_configured_phrase(self, node):
        assert node._phrase_for_nav_goal(GoalStatus.STATUS_ABORTED) == 'Goal failed'

    def test_canceled_is_silent(self, node):
        assert node._phrase_for_nav_goal(GoalStatus.STATUS_CANCELED) is None

    def test_executing_is_silent(self, node):
        assert node._phrase_for_nav_goal(GoalStatus.STATUS_EXECUTING) is None


def _status_array(status_code, goal_id_byte):
    """Build a GoalStatusArray with one GoalStatus entry for testing."""
    status = GoalStatus()
    status.status = status_code
    status.goal_info.goal_id.uuid = [goal_id_byte] * 16
    msg = GoalStatusArray()
    msg.status_list = [status]
    return msg


class TestOnNavGoalStatus:

    def test_succeeded_goal_speaks_once(self, node, monkeypatch):
        spoken = []
        monkeypatch.setattr(node._speech, 'speak', spoken.append)
        msg = _status_array(GoalStatus.STATUS_SUCCEEDED, 1)
        node._on_nav_goal_status(msg)
        node._on_nav_goal_status(msg)  # same goal republished - must not double-announce
        assert spoken == ['Goal reached']

    def test_different_goals_each_speak(self, node, monkeypatch):
        spoken = []
        monkeypatch.setattr(node._speech, 'speak', spoken.append)
        node._on_nav_goal_status(_status_array(GoalStatus.STATUS_SUCCEEDED, 1))
        node._on_nav_goal_status(_status_array(GoalStatus.STATUS_ABORTED, 2))
        assert spoken == ['Goal reached', 'Goal failed']

    def test_canceled_goal_is_silent(self, node, monkeypatch):
        spoken = []
        monkeypatch.setattr(node._speech, 'speak', spoken.append)
        node._on_nav_goal_status(_status_array(GoalStatus.STATUS_CANCELED, 1))
        assert spoken == []
