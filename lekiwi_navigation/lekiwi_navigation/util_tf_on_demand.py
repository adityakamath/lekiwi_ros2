"""Look up one transform with a short-lived TF listener instead of a permanent one."""

from rclpy.duration import Duration
from rclpy.time import Time
import tf2_ros


def lookup_transform(target, source, timeout=2.0):
    """Block until target <- source is available, using a listener on its own node and thread.

    A permanent listener decodes every /tf message (the EKF alone publishes 30 Hz); this one
    exists only for the lookup, so idle nodes pay nothing.
    """
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, None, spin_thread=True)
    try:
        return buffer.lookup_transform(target, source, Time(), Duration(seconds=timeout))
    finally:
        listener.executor.shutdown()
        listener.dedicated_listener_thread.join()
        listener.unregister()
        listener.node.destroy_node()
