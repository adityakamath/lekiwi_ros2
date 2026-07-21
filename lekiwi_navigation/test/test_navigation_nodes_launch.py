#!/usr/bin/env python3
"""
launch_testing integration tests for lekiwi_navigation nodes.

Tests that waypoint_recorder_node and map_saver_node start cleanly,
advertise their expected services, and respond correctly to basic calls.
"""

import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from std_srvs.srv import SetBool


@pytest.mark.launch_test
def generate_test_description():
    waypoint_recorder = launch_ros.actions.Node(
        package="lekiwi_navigation",
        executable="waypoint_recorder_node",
        name="waypoint_recorder_node",
        parameters=[{"number_of_loops": 0, "frame_id": "map",
                     "marker_topic": "/waypoint_markers", "max_retries": 3}],
        output="screen",
    )
    map_saver = launch_ros.actions.Node(
        package="lekiwi_navigation",
        executable="map_saver_node",
        name="map_saver_node",
        parameters=[{"save_timeout": 5.0}],
        output="screen",
    )
    return launch.LaunchDescription([
        waypoint_recorder,
        map_saver,
        launch_testing.actions.ReadyToTest(),
    ]), {"waypoint_recorder": waypoint_recorder, "map_saver": map_saver}


def _make_node(name):
    """Create an rclpy node, initializing rclpy if not already done."""
    if not rclpy.ok():
        rclpy.init()
    return rclpy.create_node(name)


def _wait_for_service(client, service_name, timeout=15.0):
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(f"{service_name} did not become ready in {timeout}s")


def _call_service(node, client, data):
    req = SetBool.Request()
    req.data = data
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    return future.result()


class TestWaypointRecorderServices(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._node = _make_node("test_waypoint_recorder")
        services = ["/record_waypoint", "/waypoint_follow", "/reset_waypoints"]
        cls._clients = {s: cls._node.create_client(SetBool, s) for s in services}
        for svc, client in cls._clients.items():
            _wait_for_service(client, svc)

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_node()

    def _call(self, service, data):
        return _call_service(self._node, self._clients[service], data)

    def test_record_waypoint_noop_on_false(self):
        result = self._call("/record_waypoint", False)
        self.assertIsNotNone(result)
        self.assertTrue(result.success)

    def test_waypoint_follow_fails_without_waypoints(self):
        result = self._call("/waypoint_follow", True)
        self.assertIsNotNone(result)
        self.assertFalse(result.success)

    def test_reset_waypoints_always_succeeds(self):
        result = self._call("/reset_waypoints", True)
        self.assertIsNotNone(result)
        self.assertTrue(result.success)

    def test_waypoint_follow_noop_on_false(self):
        result = self._call("/waypoint_follow", False)
        self.assertIsNotNone(result)
        self.assertTrue(result.success)


class TestMapSaverServices(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._node = _make_node("test_map_saver")
        cls._client = cls._node.create_client(SetBool, "/save_map")
        _wait_for_service(cls._client, "/save_map")

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_node()

    def _call(self, data):
        return _call_service(self._node, self._client, data)

    def test_save_map_noop_on_false(self):
        result = self._call(False)
        self.assertIsNotNone(result)
        self.assertTrue(result.success)
