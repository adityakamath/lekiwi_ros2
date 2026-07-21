#!/usr/bin/env python3
"""
twist_switch_node — generic velocity command switcher.

Two input topics feed a single output topic. The active input is selected via a SetBool
service. Input/output types are independently configurable as either Twist or TwistStamped;
the node converts between them as needed. Only messages from the active input are forwarded -
if it stops publishing, nothing is published (no fallback, no zero-velocity injection).

Service:
    /twist_switch  (std_srvs/SetBool)
        false — activate default input  (initial state)
        true  — activate switched input

Parameters:
    input_topic_default          str   Default input topic name  (default: /cmd_vel_teleop)
    input_topic_default.stamped  bool  True = TwistStamped, False = Twist  (default: true)
    input_topic_switched         str   Switched input topic name (default: /cmd_vel_smoothed)
    input_topic_switched.stamped bool  True = TwistStamped, False = Twist  (default: true)
    output_topic                 str   Output topic name  (default: /base_controller/cmd_vel)
    output_topic.stamped         bool  True = TwistStamped, False = Twist  (default: true)
    frame_id                     str   frame_id for stamped output  (default: base_link)
"""

import rclpy
from rclpy._rclpy_pybind11.service_introspection import ServiceIntrospectionState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default

from geometry_msgs.msg import Twist, TwistStamped
from std_srvs.srv import SetBool


class TeleopSwitchNode(Node):
    """Forwards whichever of two velocity input topics is currently selected to one output."""

    def __init__(self):
        """Declare parameters, wire up the publisher/subscribers, and create /twist_switch."""
        super().__init__('twist_switch_node')

        self.declare_parameter('input_topic_default',          '/cmd_vel_teleop')
        self.declare_parameter('input_topic_default.stamped',  True)
        self.declare_parameter('input_topic_switched',         '/cmd_vel_smoothed')
        self.declare_parameter('input_topic_switched.stamped', True)
        self.declare_parameter('output_topic',                 '/base_controller/cmd_vel')
        self.declare_parameter('output_topic.stamped',         True)
        self.declare_parameter('frame_id',                     'base_link')

        topic_default    = self.get_parameter('input_topic_default').value
        default_stamped  = self.get_parameter('input_topic_default.stamped').value
        topic_switched   = self.get_parameter('input_topic_switched').value
        switched_stamped = self.get_parameter('input_topic_switched.stamped').value
        topic_output     = self.get_parameter('output_topic').value
        self._output_stamped = self.get_parameter('output_topic.stamped').value
        self._frame_id   = self.get_parameter('frame_id').value

        self._switched = False

        if self._output_stamped:
            self._pub = self.create_publisher(TwistStamped, topic_output, 10)
        else:
            self._pub = self.create_publisher(Twist, topic_output, 10)

        if default_stamped:
            self.create_subscription(
                TwistStamped, topic_default,
                lambda msg: self._handle(msg, switched=False, in_stamped=True),
                10)
        else:
            self.create_subscription(
                Twist, topic_default,
                lambda msg: self._handle(msg, switched=False, in_stamped=False),
                10)

        if switched_stamped:
            self.create_subscription(
                TwistStamped, topic_switched,
                lambda msg: self._handle(msg, switched=True, in_stamped=True),
                10)
        else:
            self.create_subscription(
                Twist, topic_switched,
                lambda msg: self._handle(msg, switched=True, in_stamped=False),
                10)

        self._srv = self.create_service(SetBool, '/twist_switch', self._switch_cb)
        # Lets a listener (e.g. the audio indicator node) observe every call's
        # request+response on /twist_switch/_service_event, regardless of caller.
        self._srv.configure_introspection(
            self.get_clock(), qos_profile_services_default, ServiceIntrospectionState.CONTENTS)

        self.get_logger().info(
            f'twist_switch_node ready\n'
            f'  default:  {topic_default} ({"TwistStamped" if default_stamped else "Twist"})\n'
            f'  switched: {topic_switched} ({"TwistStamped" if switched_stamped else "Twist"})\n'
            f'  output:   {topic_output} ({"TwistStamped" if self._output_stamped else "Twist"})\n'
            f'  service:  /twist_switch'
        )

    def _handle(self, msg, *, switched: bool, in_stamped: bool) -> None:
        """Forward msg to output only if it is from the currently active input."""
        if switched != self._switched:
            return
        self._pub.publish(self._convert(msg, in_stamped))

    def _convert(self, msg, in_stamped: bool):
        """Convert msg to the output type, stamping or stripping header as needed."""
        if self._output_stamped and in_stamped:
            # TwistStamped → TwistStamped: refresh header
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._frame_id
            return msg

        if self._output_stamped and not in_stamped:
            # Twist → TwistStamped: add header
            out = TwistStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self._frame_id
            out.twist = msg
            return out

        if not self._output_stamped and in_stamped:
            # TwistStamped → Twist: strip header
            return msg.twist

        # Twist → Twist: pass through
        return msg

    def _switch_cb(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        """Select the active input per request.data and report the new mode."""
        self._switched = request.data
        response.success = True
        if self._switched:
            response.message = 'Switched to switched input'
            self.get_logger().info('Mode → switched')
        else:
            response.message = 'Switched to default input'
            self.get_logger().info('Mode → default')
        return response


def main(args=None):
    """Initialize rclpy, spin TeleopSwitchNode, and shut down cleanly."""
    rclpy.init(args=args)
    node = TeleopSwitchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Received keyboard interrupt!')
    except ExternalShutdownException:
        print('Received external shutdown request!')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
