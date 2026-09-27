"""Goal lifecycle tests for execution on a ROS 2 device (no Nav2 server needed)."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
import pytest
from std_srvs.srv import SetBool

from lekiwi_navigation.nav2_target_node import Nav2TargetNode


@pytest.fixture
def tracker():
    node = Nav2TargetNode()
    node._nav_client = Mock()
    node._nav_client.server_is_ready.return_value = True
    node._publish_goal_status = Mock()
    node._submitted_pub = Mock()
    node._mode = True
    node._pose = [1.0, 2.0, math.pi / 2]
    yield node
    node.destroy_node()


def send(node, value=True):
    return node._send_goal(SetBool.Request(data=value), SetBool.Response())


def test_false_is_noop_even_without_mode(tracker):
    tracker._mode = None
    assert send(tracker, False).success
    tracker._nav_client.send_goal_async.assert_not_called()


@pytest.mark.parametrize('mode,pose,ready', [
    (False, [1.0, 2.0, 0.0], True),
    (None, [1.0, 2.0, 0.0], True),
    (True, None, True),
    (True, [float('nan'), 0.0, 0.0], True),
    (True, [1.0, 2.0, 0.0], False),
])
def test_submission_preconditions(tracker, mode, pose, ready):
    tracker._mode, tracker._pose = mode, pose
    tracker._nav_client.server_is_ready.return_value = ready
    assert not send(tracker).success
    tracker._nav_client.send_goal_async.assert_not_called()


def test_snapshot_and_busy_slot_survive_edit_and_mode_change(tracker):
    assert send(tracker).success
    goal = tracker._nav_client.send_goal_async.call_args.args[0]
    token = tracker._goal_token
    tracker._pose[:] = [9.0, 8.0, 0.0]
    assert goal.pose.header.frame_id == 'map'
    assert goal.pose.pose.position.x == 1.0
    assert goal.pose.pose.position.y == 2.0
    assert goal.pose.pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert goal.pose.pose.orientation.w == pytest.approx(math.sqrt(0.5))
    assert not send(tracker).success
    tracker._set_mode(False)
    assert tracker._goal_token is token
    tracker._nav_client.send_goal_async.assert_called_once()


def test_rejection_releases_slot(tracker):
    send(tracker)
    tracker._on_goal_response(tracker._goal_token, Mock(result=lambda: SimpleNamespace(accepted=False)))
    assert tracker._goal_token is None
    tracker._publish_goal_status.assert_called_with('rejected')
    assert send(tracker).success


@pytest.mark.parametrize('status,state', [
    (GoalStatus.STATUS_SUCCEEDED, 'succeeded'),
    (GoalStatus.STATUS_ABORTED, 'failed'),
    (GoalStatus.STATUS_CANCELED, 'canceled'),
])
def test_terminal_result_releases_slot(tracker, status, state):
    send(tracker)
    token = tracker._goal_token
    handle = Mock(accepted=True)
    tracker._on_goal_response(token, Mock(result=lambda: handle))
    assert tracker._goal_handle is handle
    handle.get_result_async.assert_called_once()
    result = SimpleNamespace(status=status, result=SimpleNamespace(error_code=0, error_msg=''))
    tracker._on_goal_result(token, Mock(result=lambda: result))
    assert tracker._goal_token is None
    assert tracker._goal_handle is None
    tracker._publish_goal_status.assert_called_with(state, '')
    assert send(tracker).success
    new_token = tracker._goal_token
    tracker._on_goal_result(token, Mock(result=lambda: result))
    assert tracker._goal_token is new_token


def test_uncertain_transport_outcome_keeps_busy_slot(tracker):
    tracker._nav_client.send_goal_async.side_effect = RuntimeError('transport lost')
    assert not send(tracker).success
    assert tracker._goal_token is not None
    tracker._publish_goal_status.assert_called_with('unknown', 'transport lost')
    assert not send(tracker).success
    tracker._nav_client.send_goal_async.assert_called_once()


@pytest.mark.parametrize('stage', ['response', 'result'])
def test_async_exception_keeps_slot(tracker, stage):
    send(tracker)
    token = tracker._goal_token
    future = Mock()
    future.result.side_effect = RuntimeError('connection lost')
    callback = tracker._on_goal_response if stage == 'response' else tracker._on_goal_result
    callback(token, future)
    assert tracker._goal_token is token
    tracker._publish_goal_status.assert_called_with('unknown', 'connection lost')


def test_unrecognized_result_does_not_allow_duplicate(tracker):
    send(tracker)
    token = tracker._goal_token
    result = SimpleNamespace(status=GoalStatus.STATUS_UNKNOWN)
    tracker._on_goal_result(token, Mock(result=lambda: result))
    assert tracker._goal_token is token
    assert not send(tracker).success
