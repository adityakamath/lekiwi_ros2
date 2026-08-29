#!/usr/bin/env python3
"""Node-level unit tests for lekiwi_audio's IndicatorNode.

Uses rclpy directly (no launch_testing, no live service graph) by initializing
rclpy once per session via a session-scoped fixture (conftest.py). Tests
exercise pure logic methods that don't require a live ROS graph - no service
calls, no subscribers actually firing.
"""

import pytest
from action_msgs.msg import GoalStatus, GoalStatusArray
from lekiwi_audio.indicator_node import GoalStatusWatcher, IndicatorNode, _phrase_filename
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


def _make_status(status_code, goal_id_byte):
    """Build a single GoalStatus entry for testing."""
    status = GoalStatus()
    status.status = status_code
    status.goal_info.goal_id.uuid = [goal_id_byte] * 16
    return status


def _status_array(status_code, goal_id_byte):
    """Build a GoalStatusArray with one GoalStatus entry for testing."""
    msg = GoalStatusArray()
    msg.status_list = [_make_status(status_code, goal_id_byte)]
    return msg


_NAV_GOAL_PHRASES = {
    'success': 'Goal reached', 'failure': 'Goal failed', 'canceled': 'Goal canceled',
}


class TestGoalStatusWatcher:
    """GoalStatusWatcher is plain Python - no rclpy Node needed to exercise its logic."""

    def test_succeeded_speaks(self):
        watcher = GoalStatusWatcher('nav_goal', _NAV_GOAL_PHRASES)
        assert watcher.resolve(_status_array(GoalStatus.STATUS_SUCCEEDED, 1)) == ['Goal reached']

    def test_aborted_speaks(self):
        watcher = GoalStatusWatcher('nav_goal', _NAV_GOAL_PHRASES)
        assert watcher.resolve(_status_array(GoalStatus.STATUS_ABORTED, 1)) == ['Goal failed']

    def test_executing_is_silent(self):
        watcher = GoalStatusWatcher('nav_goal', _NAV_GOAL_PHRASES)
        assert watcher.resolve(_status_array(GoalStatus.STATUS_EXECUTING, 1)) == []

    def test_missing_phrase_entry_is_silent(self):
        watcher = GoalStatusWatcher('nav_goal', {})
        assert watcher.resolve(_status_array(GoalStatus.STATUS_SUCCEEDED, 1)) == []

    def test_same_goal_not_reannounced(self):
        watcher = GoalStatusWatcher('nav_goal', _NAV_GOAL_PHRASES)
        msg = _status_array(GoalStatus.STATUS_SUCCEEDED, 1)
        assert watcher.resolve(msg) == ['Goal reached']
        assert watcher.resolve(msg) == []  # same goal republished - must not double-announce

    def test_different_goals_each_speak(self):
        watcher = GoalStatusWatcher('nav_goal', _NAV_GOAL_PHRASES)
        assert watcher.resolve(_status_array(GoalStatus.STATUS_SUCCEEDED, 1)) == ['Goal reached']
        assert watcher.resolve(_status_array(GoalStatus.STATUS_ABORTED, 2)) == ['Goal failed']

    def test_canceled_goal_with_no_successor_speaks(self):
        watcher = GoalStatusWatcher('nav_goal', _NAV_GOAL_PHRASES)
        assert watcher.resolve(_status_array(GoalStatus.STATUS_CANCELED, 1)) == ['Goal canceled']

    def test_canceled_goal_superseded_by_active_goal_is_silent(self):
        # A newer goal is already ACCEPTED/EXECUTING alongside the CANCELED one - this is the
        # action server preempting the old goal, not a deliberate stop.
        watcher = GoalStatusWatcher('nav_goal', _NAV_GOAL_PHRASES)
        msg = GoalStatusArray()
        msg.status_list = [
            _make_status(GoalStatus.STATUS_CANCELED, 1),
            _make_status(GoalStatus.STATUS_EXECUTING, 2),
        ]
        assert watcher.resolve(msg) == []

    def test_superseded_cancel_is_not_reannounced_once_successor_finishes(self):
        watcher = GoalStatusWatcher('nav_goal', _NAV_GOAL_PHRASES)
        msg = GoalStatusArray()
        msg.status_list = [
            _make_status(GoalStatus.STATUS_CANCELED, 1),
            _make_status(GoalStatus.STATUS_EXECUTING, 2),
        ]
        assert watcher.resolve(msg) == []  # goal 1 superseded, marked handled, stays silent
        assert watcher.resolve(_status_array(GoalStatus.STATUS_SUCCEEDED, 2)) == ['Goal reached']


class TestIndicatorNodeGoalWatcherWiring:
    """Confirms IndicatorNode builds watchers from parameters/phrases.yaml and wires them up."""

    def test_default_nav_goal_watcher_is_built(self, node):
        assert 'nav_goal' in node._goal_watchers

    def test_callback_speaks_resolved_phrases(self, node, monkeypatch):
        spoken = []
        monkeypatch.setattr(node._speech, 'speak', spoken.append)
        callback = node._make_goal_status_callback(node._goal_watchers['nav_goal'])
        callback(_status_array(GoalStatus.STATUS_SUCCEEDED, 1))
        assert spoken == ['Goal reached']
