"""Deterministic node tests; run with ROS 2 installed, without robot hardware."""
import math
from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import TransformStamped, TwistStamped
import pytest
from rclpy.time import Time
from service_msgs.msg import ServiceEventInfo
from std_srvs.srv import SetBool, SetBool_Event
import tf2_ros
from visualization_msgs.msg import Marker

from lekiwi_navigation.nav2_target_node import Nav2TargetNode


@pytest.fixture
def target(monkeypatch):
    node = Nav2TargetNode()
    clock = Mock()
    clock.now.return_value = Time(seconds=10)
    monkeypatch.setattr(node, 'get_clock', lambda: clock)
    node._buffer = Mock()
    node._broadcaster = Mock()
    node._pose_pub = Mock()
    node._marker_pub = Mock()
    node._status_pub = Mock()
    yield node, clock
    node.destroy_node()


def event(kind, seq=1, value=True, success=True):
    msg = SetBool_Event()
    msg.info.event_type = kind
    msg.info.sequence_number = seq
    if kind == ServiceEventInfo.REQUEST_RECEIVED:
        msg.request = [SetBool.Request(data=value)]
    else:
        msg.response = [SetBool.Response(success=success)]
    return msg


def test_mode_requires_matching_success_and_duplicate_preserves_draft(target):
    n, _ = target
    n._on_mode_event(event(ServiceEventInfo.RESPONSE_SENT))
    assert n._mode is None
    n._on_mode_event(event(ServiceEventInfo.REQUEST_RECEIVED))
    n._on_mode_event(event(ServiceEventInfo.RESPONSE_SENT, success=False))
    assert n._mode is None
    n._on_mode_event(event(ServiceEventInfo.REQUEST_RECEIVED, seq=2))
    n._on_mode_event(event(ServiceEventInfo.RESPONSE_SENT, seq=2))
    assert n._mode is True
    n._pose = [1., 2., 3.]
    n._set_mode(True)
    assert n._pose == [1., 2., 3.]
    n._command = (0, (1, 0, 0))
    n._on_liveliness(SimpleNamespace(alive_count=0))
    assert n._mode is None and n._command is None and n._pose is None


def test_attached_marker_identity_without_map_lookup(target):
    n, _ = target
    n._set_mode(False)
    n._on_twist(TwistStamped())
    n._tick()
    n._buffer.lookup_transform.assert_not_called()
    tf = n._broadcaster.sendTransform.call_args.args[0]
    assert tf.header.frame_id == 'base_footprint'
    assert tf.child_frame_id == 'nav2_target'
    assert tf.transform.translation.x == tf.transform.translation.y == 0
    assert tf.transform.rotation.w == 1
    marker = n._marker_pub.publish.call_args.args[0]
    assert marker.type == Marker.SPHERE and marker.frame_locked
    assert marker.header.frame_id == 'nav2_target'
    assert marker.scale.x == marker.scale.y == marker.scale.z == n.get_parameter('marker_scale').value
    assert marker.lifetime.sec > 0 or marker.lifetime.nanosec > 0
    assert n._command is None


def test_map_transition_seeds_once_and_reattaches(target):
    n, _ = target
    tf = TransformStamped()
    tf.transform.translation.x = 3.
    tf.transform.translation.y = 4.
    tf.transform.rotation.z = math.sin(math.pi / 4)
    tf.transform.rotation.w = math.cos(math.pi / 4)
    n._buffer.lookup_transform.return_value = tf
    n._set_mode(True)
    n._tick()
    assert n._pose == pytest.approx([3, 4, math.pi / 2])
    assert n._pose_pub.publish.call_args.args[0].header.frame_id == 'map'
    n._tick()
    n._buffer.lookup_transform.assert_called_once()
    n._set_mode(False)
    n._tick()
    assert n._pose_pub.publish.call_args.args[0].header.frame_id == 'base_footprint'


def test_missing_tf_waits_without_publishing_invented_pose(target):
    n, _ = target
    n._buffer.lookup_transform.side_effect = tf2_ros.TransformException('missing')
    n._set_mode(True)
    n._tick()
    assert n._pose is None and n._status == 'waiting_for_map_transform'
    n._pose_pub.publish.assert_not_called()
    n._marker_pub.publish.assert_not_called()


@pytest.mark.parametrize('vx,vy,expected', [(1., 0., [0., .05]), (0., 1., [-.05, 0.])])
def test_translation_uses_target_heading(target, vx, vy, expected):
    n, clock = target
    n._mode, n._pose = True, [0., 0., math.pi / 2]
    n._tick()
    msg = TwistStamped()
    msg.header.frame_id = 'base_link'
    msg.twist.linear.x, msg.twist.linear.y = vx, vy
    n._on_twist(msg)
    clock.now.return_value = Time(seconds=10.05)
    n._tick()
    assert n._pose[:2] == pytest.approx(expected)


def test_rotation_wrap_and_no_autosubmit(target):
    n, clock = target
    n._mode, n._pose = True, [0., 0., math.pi - .01]
    n._nav_client = Mock()
    n._tick()
    msg = TwistStamped()
    msg.twist.angular.z = 1.
    n._on_twist(msg)
    clock.now.return_value = Time(seconds=10.05)
    n._tick()
    assert n._pose[2] == pytest.approx(-math.pi + .04)
    n._nav_client.send_goal_async.assert_not_called()


@pytest.mark.parametrize('now', [9., 10.15, 10.5])
def test_clock_jump_large_step_and_timeout_clear_input(target, now):
    n, clock = target
    n._mode, n._pose = True, [0., 0., 0.]
    n._tick()
    msg = TwistStamped()
    msg.twist.linear.x = 1.
    n._on_twist(msg)
    clock.now.return_value = Time(seconds=now)
    n._tick()
    assert n._pose == [0., 0., 0.]
    assert n._command is None


def test_nonfinite_input_clears_previous_command(target):
    n, _ = target
    n._mode, n._pose = True, [0., 0., 0.]
    n._command = (0, (1, 0, 0))
    msg = TwistStamped()
    msg.twist.angular.z = float('nan')
    n._on_twist(msg)
    assert n._command is None


@pytest.mark.parametrize('input_hz', [10, 30, 50, 100])
@pytest.mark.parametrize('command_first', [True, False])
@pytest.mark.parametrize('axis', ['x', 'y', 'yaw'])
def test_continuous_input_integrates_full_duration(target, input_hz, command_first, axis):
    n, clock = target
    n._mode, n._pose = True, [0., 0., 0.]
    msg = TwistStamped()
    if axis == 'yaw':
        msg.twist.angular.z = 1.
    else:
        setattr(msg.twist.linear, axis, 1.)
    # Three seconds at 1 m/s or rad/s, with independent 30 Hz publication.
    events = [(i * 1_000_000_000 // 30, 'tick') for i in range(91)]
    events += [(i * 1_000_000_000 // input_hz, 'command')
               for i in range(input_hz * 3 + 1)]
    for elapsed, kind in sorted(events, key=lambda e: (e[0], (e[1] == 'command')
                                                      != command_first)):
        clock.now.return_value = Time(nanoseconds=10_000_000_000 + elapsed)
        if kind == 'tick':
            n._tick()
        else:
            n._on_twist(msg)
    expected = [0., 0., 0.]
    expected[['x', 'y', 'yaw'].index(axis)] = 3.
    assert n._pose == pytest.approx(expected)


def test_command_changes_between_ticks_preserve_each_interval(target):
    n, clock = target
    n._mode, n._pose = True, [0., 0., 0.]
    n._tick()
    msg = TwistStamped()
    # Start between ticks, reverse, stop, then restart after an idle interval.
    for elapsed, velocity in [(20, 1.), (40, -2.), (70, 0.), (90, 1.)]:
        clock.now.return_value = Time(nanoseconds=10_000_000_000 + elapsed * 1_000_000)
        msg.twist.linear.x = velocity
        n._on_twist(msg)
    clock.now.return_value = Time(seconds=10.1)
    n._tick()
    assert n._pose == pytest.approx([.02 - .06 + .01, 0., 0.])


@pytest.mark.parametrize('now', [9., 10.15, 10.5])
def test_new_command_after_clock_gap_does_not_integrate_stale_input(target, now):
    n, clock = target
    n._mode, n._pose = True, [0., 0., 0.]
    msg = TwistStamped()
    msg.twist.linear.x = 1.
    n._on_twist(msg)
    clock.now.return_value = Time(seconds=now)
    n._on_twist(msg)
    assert n._pose == [0., 0., 0.]
    clock.now.return_value = Time(nanoseconds=clock.now.return_value.nanoseconds + 50_000_000)
    n._tick()
    assert n._pose == pytest.approx([.05, 0., 0.])
