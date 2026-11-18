"""The on-demand TF lookup finds a transform, times out cleanly and leaves no thread behind."""

import threading

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
import tf2_ros

from lekiwi_navigation.tf_on_demand import lookup_transform


def _broadcast(node, parent, child, x):
    msg = TransformStamped()
    msg.header.frame_id, msg.child_frame_id = parent, child
    msg.transform.translation.x = x
    msg.transform.rotation.w = 1.0
    tf2_ros.StaticTransformBroadcaster(node).sendTransform(msg)


def test_lookup_returns_the_published_transform():
    node = rclpy.create_node('tf_on_demand_source')
    _broadcast(node, 'od_map', 'od_base', 2.5)
    spinner = threading.Thread(target=lambda: rclpy.spin_once(node, timeout_sec=1.0))
    spinner.start()
    try:
        assert lookup_transform('od_map', 'od_base', timeout=5.0).transform.translation.x == 2.5
    finally:
        spinner.join()
        node.destroy_node()


def test_missing_transform_times_out_and_stops_its_thread():
    before = threading.active_count()
    with pytest.raises(tf2_ros.TransformException):
        lookup_transform('od_nowhere', 'od_nothing', timeout=0.3)
    assert threading.active_count() == before
