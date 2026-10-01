"""Goal lifecycle tests for execution on a ROS 2 device (no Nav2 server needed)."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
import pytest
from std_srvs.srv import SetBool

from lekiwi_navigation.nav2_target_node import Nav2TargetNode


@pytest.fixture
def tracker():
    node = Nav2TargetNode()
    node._nav_client = Mock()
    node._nav_client.server_is_ready.return_value = True
    node._publish_goal_status = Mock()
    node._goal_input_status_pub = Mock()
    node._submitted_pub = Mock()
    node._mode = True
    node._pose = [1.0, 2.0, math.pi / 2]
    yield node
    node.destroy_node()


def send(node, value=True):
    return node._send_goal(SetBool.Request(data=value), SetBool.Response())


def map_goal(x=4.0, y=-2.0, yaw=math.pi / 2):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.orientation.w = math.cos(yaw / 2)
    return msg


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


def test_external_map_goal_moves_target_and_submits_snapshot(tracker):
    tracker._pose = None
    tracker._on_external_goal(map_goal())

    assert tracker._pose == pytest.approx([4.0, -2.0, math.pi / 2])
    goal = tracker._nav_client.send_goal_async.call_args.args[0]
    assert goal.pose.header.frame_id == 'map'
    assert goal.pose.pose.position.x == 4.0
    assert goal.pose.pose.position.y == -2.0
    assert goal.pose.pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert goal.pose.pose.orientation.w == pytest.approx(math.sqrt(0.5))
    assert tracker._submitted_pub.publish.call_args.args[0].pose.position.x == 4.0
    assert tracker._goal_input_status_pub.publish.call_args.args[0].data.startswith('submitted:')


@pytest.mark.parametrize('frame,zero_quaternion', [('odom', False), ('map', True)])
def test_external_goal_rejects_wrong_frame_or_invalid_orientation(
        tracker, frame, zero_quaternion):
    msg = map_goal()
    msg.header.frame_id = frame
    if zero_quaternion:
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 0.0
    original_pose = tracker._pose[:]

    tracker._on_external_goal(msg)

    assert tracker._pose == original_pose
    tracker._nav_client.send_goal_async.assert_not_called()
    assert tracker._goal_input_status_pub.publish.call_args.args[0].data.startswith('rejected:')


def test_external_goal_rejected_outside_nav_mode_or_while_busy(tracker):
    tracker._mode = False
    tracker._on_external_goal(map_goal())
    tracker._nav_client.send_goal_async.assert_not_called()
    assert tracker._pose == [1.0, 2.0, math.pi / 2]

    tracker._mode = True
    assert send(tracker).success
    tracker._on_external_goal(map_goal())
    assert tracker._nav_client.send_goal_async.call_count == 1
    assert tracker._pose == [1.0, 2.0, math.pi / 2]


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


def test_operator_can_clear_only_unknown_goal_state(tracker):
    assert send(tracker).success
    active_token = tracker._goal_token
    assert send(tracker, False).success
    assert tracker._goal_token is active_token
    tracker._mark_goal_unknown('transport lost')
    assert tracker._goal_unknown
    response = send(tracker, False)
    assert response.success and 'Cleared unknown' in response.message
    assert tracker._goal_token is None
    assert tracker._goal_handle is None
    assert not tracker._goal_unknown
    tracker._publish_goal_status.assert_called_with(
        'idle', 'unknown outcome cleared by operator')
    assert send(tracker).success
    new_token = tracker._goal_token
    tracker._on_goal_response(active_token, Mock())
    assert tracker._goal_token is new_token
