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
        false — activate default input
        true  — activate switched input

        Switching to the switched input is rejected if no node is publishing on
        input_topic_switched (e.g. the navigation stack is not running).
        - If already in switched mode (nav died mid-session): stays in switched
          mode, continues publishing zeros. Call service(false) to return to default.
        - If in default mode: remains on default input.

Conversion rules:
    Twist in  + TwistStamped out → stamp with now() and frame_id
    TwistStamped in + Twist out  → strip header, forward twist only
    Same type in/out             → pass through (TwistStamped header refreshed)

Timeout behaviour:
    While the switched input is active, if no message arrives for longer than
    switched_timeout_s seconds, a zero velocity is published.  The node remains in
    switched mode — an explicit switch(false) call is required to return to the
    default input.

Parameters:
    input_topic_default          str   Default input topic name  (default: /cmd_vel_teleop)
    input_topic_default.stamped  bool  True = TwistStamped, False = Twist  (default: true)
    input_topic_switched         str   Switched input topic name (default: /cmd_vel_nav)
    input_topic_switched.stamped bool  True = TwistStamped, False = Twist  (default: false)
    output_topic                 str   Output topic name  (default: /base_controller/cmd_vel)
    output_topic.stamped         bool  True = TwistStamped, False = Twist  (default: true)
    frame_id                     str   frame_id for stamped output  (default: base_link)
    switched_timeout_s           float Silence timeout in switched mode, seconds  (default: 0.5)
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
        self.declare_parameter('input_topic_switched.stamped', False)
        self.declare_parameter('output_topic',                 '/base_controller/cmd_vel')
        self.declare_parameter('output_topic.stamped',         True)
        self.declare_parameter('frame_id',                     'base_link')
        self.declare_parameter('switched_timeout_s',           0.5)

        topic_default    = self.get_parameter('input_topic_default').value
        default_stamped  = self.get_parameter('input_topic_default.stamped').value
        topic_switched   = self.get_parameter('input_topic_switched').value
        switched_stamped = self.get_parameter('input_topic_switched.stamped').value
        topic_output     = self.get_parameter('output_topic').value
        self._output_stamped = self.get_parameter('output_topic.stamped').value
        self._frame_id   = self.get_parameter('frame_id').value
        self._timeout    = self.get_parameter('switched_timeout_s').value

        self._switched           = False
        self._last_switched_time = None
        self._timeout_warned     = False
        self._topic_switched     = topic_switched

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

        # ── Watchdog timer: 10 Hz ─────────────────────────────────────────────
        self.create_timer(0.1, self._watchdog_cb)

        self.get_logger().info(
            f'teleop_switch_node ready\n'
            f'  default:  {topic_default} ({"TwistStamped" if default_stamped else "Twist"})\n'
            f'  switched: {topic_switched} ({"TwistStamped" if switched_stamped else "Twist"})\n'
            f'  output:   {topic_output} ({"TwistStamped" if self._output_stamped else "Twist"})\n'
            f'  timeout:  {self._timeout} s | service: /twist_switch'
        )

    # ── Message handling ──────────────────────────────────────────────────────

    def _handle(self, msg, *, switched: bool, in_stamped: bool) -> None:
        """Forward msg to output if it is from the currently active input."""
        if switched != self._switched:
            return
        if switched:
            self._last_switched_time = self.get_clock().now()
            self._timeout_warned = False  # nav is alive again
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
        if request.data:
            if self.count_publishers(self._topic_switched) == 0:
                # No publisher on switched topic — leave _switched unchanged.
                # If already switched: stay in switched mode, zeros continue.
                # If on default: remain on default.
                response.success = False
                response.message = (
                    f'No publishers on {self._topic_switched} — '
                    + ('staying in switched mode, call service(false) to return to default'
                       if self._switched else 'remaining on default input')
                )
                self.get_logger().warn(response.message)
                return response

            self._switched = True
            self._last_switched_time = None
            self._timeout_warned = False
            response.success = True
            response.message = 'Switched to switched input'
            self.get_logger().info('Mode → switched')
        else:
            self._switched = False
            response.success = True
            response.message = 'Switched to default input'
            self.get_logger().info('Mode → default')

        return response

    # ── Watchdog ──────────────────────────────────────────────────────────────

    def _watchdog_cb(self) -> None:
        """Publish zeros if switched input has gone silent."""
        if not self._switched:
            return

        timed_out = (
            self._last_switched_time is None
            or (self.get_clock().now() - self._last_switched_time).nanoseconds * 1e-9
            > self._timeout
        )

        if not timed_out:
            return

        if not self._timeout_warned:
            self.get_logger().warn(
                f'Switched input silent for >{self._timeout}s — publishing zeros. '
                f'Call service(false) or press the teleop button to return to default.'
            )
            self._timeout_warned = True

        if self._output_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._frame_id
            self._pub.publish(msg)
        else:
            self._pub.publish(Twist())


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
