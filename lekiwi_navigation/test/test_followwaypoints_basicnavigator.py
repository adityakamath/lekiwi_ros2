#!/usr/bin/env python3
"""
Verifies BasicNavigator.followWaypoints() transmits number_of_loops/goal_index correctly.

Runs entirely against a stub FollowWaypoints action server in-process - no real nav2 stack,
no hardware, no robot motion. This is the check to run before any live-robot test: it proves
the patched nav2_simple_commander actually puts number_of_loops/goal_index on the outgoing
goal message, which is the one thing a live test can't isolate from everything else that can
go wrong on real hardware (controller state, DDS discovery, motor power).
"""

import threading
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from nav2_simple_commander.robot_navigator import BasicNavigator
import pytest
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class _StubWaypointFollower(Node):
    """Minimal FollowWaypoints action server: records the goal it receives, then succeeds."""

    def __init__(self):
        super().__init__('stub_waypoint_follower')
        self.last_goal = None
        self._server = ActionServer(self, FollowWaypoints, 'follow_waypoints', self._execute)

    def _execute(self, goal_handle):
        self.last_goal = goal_handle.request
        goal_handle.succeed()
        return FollowWaypoints.Result()


@pytest.fixture
def stub_and_navigator():
    """Spin a stub action server and a BasicNavigator on a shared background executor."""
    stub = _StubWaypointFollower()
    navigator = BasicNavigator(node_name='test_basic_navigator')

    executor = MultiThreadedExecutor()
    executor.add_node(stub)
    executor.add_node(navigator)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    yield stub, navigator

    executor.shutdown()
    stub.destroy_node()
    navigator.destroy_node()


def _wait_for(predicate, timeout=5.0):
    deadline = time.time() + timeout
    while not predicate() and time.time() < deadline:
        time.sleep(0.05)
    return predicate()


def test_followwaypoints_transmits_number_of_loops_and_goal_index(stub_and_navigator):
    """The exact regression this branch exists to check: both new fields reach the server."""
    stub, navigator = stub_and_navigator

    pose = PoseStamped()
    pose.header.frame_id = 'map'
    task = navigator.followWaypoints([pose, pose], number_of_loops=3, goal_index=2)

    assert task is not None, 'goal was rejected'
    assert _wait_for(lambda: stub.last_goal is not None), 'stub server never received a goal'
    assert stub.last_goal.number_of_loops == 3
    assert stub.last_goal.goal_index == 2
    assert len(stub.last_goal.poses) == 2


def test_followwaypoints_default_matches_old_behavior(stub_and_navigator):
    """Omitting the new kwargs must behave exactly like the pre-patch signature (0, 0)."""
    stub, navigator = stub_and_navigator

    pose = PoseStamped()
    pose.header.frame_id = 'map'
    task = navigator.followWaypoints([pose])

    assert task is not None, 'goal was rejected'
    assert _wait_for(lambda: stub.last_goal is not None), 'stub server never received a goal'
    assert stub.last_goal.number_of_loops == 0
    assert stub.last_goal.goal_index == 0
