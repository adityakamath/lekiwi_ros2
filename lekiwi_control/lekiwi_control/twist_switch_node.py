#!/usr/bin/env python3
"""
twist_switch_node — generic velocity command switcher.

Two input topics feed a single output topic.  The active input is selected via
a SetBool service.  Input/output types are independently configurable as either
Twist or TwistStamped; the node converts between them as needed.

Subscribes to:
    <input_topic_default>   (Twist or TwistStamped — see input_topic_default.stamped)
    <input_topic_switched>  (Twist or TwistStamped — see input_topic_switched.stamped)

Publishes:
    <output_topic>          (Twist or TwistStamped — see output_topic.stamped)

Service:
    /twist_switch  (std_srvs/SetBool)
        false — activate default input  (initial state)
        true  — activate switched input

Behaviour:
    Only messages from the currently active input are forwarded to the output.
    If the active input stops publishing, nothing is published — there is no
    fallback to the other input and no zero-velocity injection.
    Call service(false) to return to the default input at any time.

Conversion rules:
    Twist in  + TwistStamped out → stamp with now() and frame_id
    TwistStamped in + Twist out  → strip header, forward twist only
    Same type in/out             → pass through (TwistStamped header refreshed)

Parameters:
    input_topic_default          str   Default input topic name  (default: /cmd_vel_teleop)
    input_topic_default.stamped  bool  True = TwistStamped, False = Twist  (default: true)
    input_topic_switched         str   Switched input topic name (default: /cmd_vel_nav)
    input_topic_switched.stamped bool  True = TwistStamped, False = Twist  (default: true)
    output_topic                 str   Output topic name  (default: /base_controller/cmd_vel)
    output_topic.stamped         bool  True = TwistStamped, False = Twist  (default: true)
    frame_id                     str   frame_id for stamped output  (default: base_link)
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped
from std_srvs.srv import SetBool


class TeleopSwitchNode(Node):

    def __init__(self):
        super().__init__('twist_switch_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('input_topic_default',          '/cmd_vel_teleop')
        self.declare_parameter('input_topic_default.stamped',  True)
        self.declare_parameter('input_topic_switched',         '/cmd_vel_nav')
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

        # ── Publisher ─────────────────────────────────────────────────────────
        if self._output_stamped:
            self._pub = self.create_publisher(TwistStamped, topic_output, 10)
        else:
            self._pub = self.create_publisher(Twist, topic_output, 10)

        # ── Subscribers ───────────────────────────────────────────────────────
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

        # ── Service: /twist_switch ────────────────────────────────────────────
        self._srv = self.create_service(SetBool, '/twist_switch', self._switch_cb)

        self.get_logger().info(
            f'twist_switch_node ready\n'
            f'  default:  {topic_default} ({"TwistStamped" if default_stamped else "Twist"})\n'
            f'  switched: {topic_switched} ({"TwistStamped" if switched_stamped else "Twist"})\n'
            f'  output:   {topic_output} ({"TwistStamped" if self._output_stamped else "Twist"})\n'
            f'  service:  /twist_switch'
        )

    # ── Message handling ──────────────────────────────────────────────────────

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

    # ── Service callback ──────────────────────────────────────────────────────

    def _switch_cb(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
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
