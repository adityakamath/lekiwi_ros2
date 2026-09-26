#!/usr/bin/env python3
"""Editable planar target; phase one deliberately does not submit navigation goals."""

import math

from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
import rclpy
from rclpy.duration import Duration
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from service_msgs.msg import ServiceEventInfo
from std_msgs.msg import String
from std_srvs.srv import SetBool_Event
import tf2_ros
from visualization_msgs.msg import Marker


class Nav2TargetNode(Node):
    """Attach to the base in teleop; edit a map-fixed target in navigation mode."""

    def __init__(self):
        super().__init__('nav2_target_node')
        defaults = {
            'map_frame': 'map', 'base_frame': 'base_footprint',
            'target_frame': 'nav2_target', 'teleop_topic': 'cmd_vel_teleop',
            'mode_service': 'twist_switch', 'publish_rate': 30.0,
            'command_timeout': 0.2, 'max_dt': 0.1, 'marker_scale': 0.08,
            'translation_scale': 1.0, 'rotation_scale': 1.0,
            'teleop_color': [0.5, 0.5, 0.5, 0.9],
            'editing_color': [0.0, 1.0, 1.0, 0.9],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._params = {name: self.get_parameter(name).value for name in defaults}
        for name in ('publish_rate', 'command_timeout', 'max_dt', 'marker_scale'):
            value = self._params[name]
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('translation_scale', 'rotation_scale'):
            if not math.isfinite(self._params[name]) or self._params[name] < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        for name in ('teleop_color', 'editing_color'):
            color = self._params[name]
            if len(color) != 4 or any(not math.isfinite(v) or not 0 <= v <= 1 for v in color):
                raise ValueError(f'{name} must contain four RGBA values in [0, 1]')
        frames = [self._params[n] for n in ('map_frame', 'base_frame', 'target_frame')]
        if any(not f or f.startswith('/') for f in frames) or len(set(frames)) != 3:
            raise ValueError('Frames must be nonempty, distinct, and have no leading slash')
        self._mode = None
        self._pose = None
        self._pending = {}
        self._command = None
        self._last_tick = None
        self._status = None
        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer, self)
        self._broadcaster = tf2_ros.TransformBroadcaster(self)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self._pose_pub = self.create_publisher(PoseStamped, 'nav2_target_pose', retained)
        self._status_pub = self.create_publisher(String, 'nav2_target_status', retained)
        self._marker_pub = self.create_publisher(Marker, 'nav2_target_marker', 1)
        event_qos = QoSProfile(depth=32, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE,
                               liveliness_lease_duration=Duration(seconds=1))
        self.create_subscription(
            SetBool_Event, self._params['mode_service'] + '/_service_event',
            self._on_mode_event, event_qos,
            event_callbacks=SubscriptionEventCallbacks(liveliness=self._on_liveliness),
        )
        self.create_subscription(TwistStamped, self._params['teleop_topic'], self._on_twist, 1)
        self.create_timer(1.0 / self._params['publish_rate'], self._tick)

    def _on_liveliness(self, event):
        if event.alive_count == 0:
            self._pending.clear()
            self._set_mode(None)

    def _set_mode(self, mode):
        if mode == self._mode:
            return
        self._mode = mode
        self._pose = None
        self._command = None
        self._last_tick = None

    def _on_mode_event(self, msg):
        key = (bytes(msg.info.client_gid), msg.info.sequence_number)
        if msg.info.event_type == ServiceEventInfo.REQUEST_RECEIVED and msg.request:
            self._pending[key] = msg.request[0].data
            if len(self._pending) > 32:
                self._pending.pop(next(iter(self._pending)))
        elif msg.info.event_type == ServiceEventInfo.RESPONSE_SENT:
            mode = self._pending.pop(key, None)
            if mode is not None and msg.response and msg.response[0].success:
                self._set_mode(mode)

    def _on_twist(self, msg):
        if self._mode is not True or self._pose is None:
            return
        values = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)
        if not all(math.isfinite(v) for v in values):
            self._command = None
            return
        # This input is a target-control vector, regardless of its original base_link header.
        self._command = (self.get_clock().now().nanoseconds, values)

    def _publish_status(self, value):
        if value != self._status:
            self._status = value
            self._status_pub.publish(String(data=value))
            self.get_logger().info(value)

    def _tick(self):
        now = self.get_clock().now()
        previous = self._last_tick
        self._last_tick = now.nanoseconds
        if previous is not None and now.nanoseconds < previous:
            self._command = None
        if self._mode is True and self._pose is None:
            try:
                # The attached target is identity in base_frame: this is the one-time
                # attachment-to-map conversion, not continuous robot tracking.
                tf = self._buffer.lookup_transform(
                    self._params['map_frame'], self._params['base_frame'], Time())
            except tf2_ros.TransformException:
                self._publish_status('waiting_for_map_transform')
                return
            q = tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            self._pose = [tf.transform.translation.x, tf.transform.translation.y, yaw]
            self._command = None
        attached = self._mode is not True
        if not attached and self._command is not None and previous is not None:
            received, (vx, vy, wz) = self._command
            age = (now.nanoseconds - received) * 1e-9
            dt = (now.nanoseconds - max(previous, received)) * 1e-9
            if 0 <= age <= self._params['command_timeout'] and 0 < dt <= self._params['max_dt']:
                x, y, yaw = self._pose
                scale = self._params['translation_scale']
                x += (math.cos(yaw) * vx - math.sin(yaw) * vy) * dt * scale
                y += (math.sin(yaw) * vx + math.cos(yaw) * vy) * dt * scale
                yaw += wz * dt * self._params['rotation_scale']
                self._pose = [x, y, math.atan2(math.sin(yaw), math.cos(yaw))]
            else:
                self._command = None
        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = self._params['base_frame' if attached else 'map_frame']
        x, y, yaw = (0.0, 0.0, 0.0) if attached else self._pose
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        tf = TransformStamped()
        tf.header = pose.header
        tf.child_frame_id = self._params['target_frame']
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation = pose.pose.orientation
        self._broadcaster.sendTransform(tf)
        self._pose_pub.publish(pose)
        marker = Marker()
        marker.header.stamp = now.to_msg()
        marker.header.frame_id = self._params['target_frame']
        marker.ns, marker.id = 'nav2_target', 0
        marker.type, marker.action = Marker.SPHERE, Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.frame_locked = True
        marker.scale.x = marker.scale.y = marker.scale.z = self._params['marker_scale']
        color = self._params['teleop_color' if attached else 'editing_color']
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.lifetime = Duration(seconds=3.0 / self._params['publish_rate']).to_msg()
        self._marker_pub.publish(marker)
        self._publish_status('waiting_for_mode' if self._mode is None else
                             ('teleop_attached' if attached else 'editing'))


def main(args=None):
    rclpy.init(args=args)
    node = Nav2TargetNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
