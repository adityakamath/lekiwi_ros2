#!/usr/bin/env python3
"""
Node-level unit tests for lekiwi_control Python nodes.

Uses rclpy directly (no hardware, no launch_testing) by initializing rclpy once
per session via a session-scoped fixture. Tests exercise pure logic methods that
do not require a live ROS graph (no service calls, no subscribers).

Nodes under test:
  - TeleopSwitchNode  (_convert, _handle, _switch_cb)
  - BoolToggleNode    (configuration loading, toggle state)
"""

import pytest
import rclpy
from geometry_msgs.msg import Twist, TwistStamped

from lekiwi_control.twist_switch_node import TeleopSwitchNode
from lekiwi_control.bool_toggle_node import BoolToggle

# rclpy is initialized by conftest.py (session-scoped, idempotent).

# ── TeleopSwitchNode._convert ─────────────────────────────────────────────────

@pytest.fixture
def switch_node():
    """Create a TeleopSwitchNode with default parameters and destroy it after the test."""
    node = TeleopSwitchNode()
    yield node
    node.destroy_node()


class TestTwistSwitchConvert:
    """Tests for the Twist/TwistStamped conversion logic."""

    def test_twist_to_twiststamped(self, switch_node):
        """Twist input → TwistStamped output: header is added."""
        switch_node._output_stamped = True
        twist = Twist()
        twist.linear.x = 0.5
        result = switch_node._convert(twist, in_stamped=False)
        assert isinstance(result, TwistStamped)
        assert result.twist.linear.x == 0.5
        assert result.header.frame_id == switch_node._frame_id

    def test_twiststamped_to_twist(self, switch_node):
        """TwistStamped input → Twist output: header is stripped."""
        switch_node._output_stamped = False
        ts = TwistStamped()
        ts.twist.linear.y = 0.3
        result = switch_node._convert(ts, in_stamped=True)
        assert isinstance(result, Twist)
        assert result.linear.y == 0.3

    def test_twiststamped_to_twiststamped(self, switch_node):
        """TwistStamped → TwistStamped: frame_id is refreshed."""
        switch_node._output_stamped = True
        ts = TwistStamped()
        ts.twist.angular.z = 1.0
        ts.header.frame_id = 'old_frame'
        result = switch_node._convert(ts, in_stamped=True)
        assert isinstance(result, TwistStamped)
        assert result.header.frame_id == switch_node._frame_id
        assert result.twist.angular.z == 1.0

    def test_twist_to_twist_passthrough(self, switch_node):
        """Twist → Twist: message is returned unchanged."""
        switch_node._output_stamped = False
        twist = Twist()
        twist.linear.x = 0.1
        result = switch_node._convert(twist, in_stamped=False)
        assert isinstance(result, Twist)
        assert result.linear.x == 0.1


class TestTwistSwitchHandle:
    """Tests for the topic selection gate logic."""

    def test_default_input_forwarded_when_not_switched(self, switch_node):
        """Messages from the default input (switched=False) are forwarded when not switched."""
        switch_node._switched = False
        published = []
        switch_node._pub.publish = lambda msg: published.append(msg)
        ts = TwistStamped()
        switch_node._handle(ts, switched=False, in_stamped=True)
        assert len(published) == 1

    def test_switched_input_blocked_when_not_switched(self, switch_node):
        """Messages from the switched input are blocked when mode is default."""
        switch_node._switched = False
        published = []
        switch_node._pub.publish = lambda msg: published.append(msg)
        ts = TwistStamped()
        switch_node._handle(ts, switched=True, in_stamped=True)
        assert len(published) == 0

    def test_switched_input_forwarded_when_switched(self, switch_node):
        """Messages from the switched input are forwarded when mode is switched."""
        switch_node._switched = True
        published = []
        switch_node._pub.publish = lambda msg: published.append(msg)
        ts = TwistStamped()
        switch_node._handle(ts, switched=True, in_stamped=True)
        assert len(published) == 1

    def test_default_input_blocked_when_switched(self, switch_node):
        """Messages from the default input are blocked when mode is switched."""
        switch_node._switched = True
        published = []
        switch_node._pub.publish = lambda msg: published.append(msg)
        ts = TwistStamped()
        switch_node._handle(ts, switched=False, in_stamped=True)
        assert len(published) == 0


class TestTwistSwitchCb:
    """Tests for the /twist_switch service callback."""

    def test_switch_to_switched(self, switch_node):
        switch_node._switched = False
        req = type('Req', (), {'data': True})()
        resp = type('Resp', (), {'success': None, 'message': ''})()
        result = switch_node._switch_cb(req, resp)
        assert switch_node._switched is True
        assert result.success is True

    def test_switch_to_default(self, switch_node):
        switch_node._switched = True
        req = type('Req', (), {'data': False})()
        resp = type('Resp', (), {'success': None, 'message': ''})()
        result = switch_node._switch_cb(req, resp)
        assert switch_node._switched is False
        assert result.success is True


# ── BoolToggle state machine ──────────────────────────────────────────────────

class TestBoolToggleState:
    """Tests for the toggle state and initial_state logic."""

    def test_initial_state_false(self):
        """A toggle with initial_state=False starts inactive."""
        node = rclpy.create_node('test_bool_toggle_state_false')
        try:
            toggle = BoolToggle(node, 'test', '/trigger', '/target', initial_state=False)
            assert toggle._active is False
        finally:
            node.destroy_node()

    def test_initial_state_true(self):
        """A toggle with initial_state=True starts active."""
        node = rclpy.create_node('test_bool_toggle_state_true')
        try:
            toggle = BoolToggle(node, 'test2', '/trigger2', '/target2', initial_state=True)
            assert toggle._active is True
        finally:
            node.destroy_node()


class TestBoolToggleNodeConfig:
    """Tests that BoolToggleNode reads toggles config correctly."""

    def test_emergency_stop_toggle_configured(self):
        """BoolToggleNode with teleop config must have the emergency_stop toggle."""
        import os
        params_file = os.path.join(
            os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
            'config', 'base', 'toggles.yaml'
        )
        import yaml
        cfg = yaml.safe_load(open(params_file))
        toggle_names = cfg['bool_toggle_node']['ros__parameters']['toggles']
        assert 'emergency_stop' in toggle_names
        assert 'twist_switch' in toggle_names
        assert 'waypoint_follow' in toggle_names
