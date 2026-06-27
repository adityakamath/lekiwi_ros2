#!/usr/bin/env python3
"""
collision_toggle_node — deadman-style disable for collision_monitor's predictive stop.

Watches one /joy button: while held, disables collision_monitor's FootprintApproach polygon
via rcl_interfaces/srv/SetParameters; on release, re-enables it. Momentary, not a toggle - it
can't be left disabled by accident.

Parameters:
    button          int     /joy button index to watch (default: 10 = R1)
    target_node     string  Node to call set_parameters on (default: /collision_monitor)
    parameter_name  string  Parameter to disable on press / re-enable on release
                            (default: FootprintApproach.enabled)
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Joy


class CollisionToggleNode(Node):
    """Disables a collision_monitor parameter while a joystick button is held."""

    def __init__(self):
        """Declare parameters, create the SetParameters client, and subscribe to /joy."""
        super().__init__('collision_toggle_node')

        self.declare_parameter('button', 10)
        self.declare_parameter('target_node', '/collision_monitor')
        self.declare_parameter('parameter_name', 'FootprintApproach.enabled')

        self._button = self.get_parameter('button').value
        self._parameter_name = self.get_parameter('parameter_name').value
        target_node = self.get_parameter('target_node').value

        self._last_state = False  # assumed released at startup
        self._client = self.create_client(SetParameters, f'{target_node}/set_parameters')

        self.create_subscription(Joy, '/joy', self._joy_callback, 10)
        self.get_logger().info(
            f'collision_toggle_node ready: holding button {self._button} disables '
            f'{target_node} {self._parameter_name}, releasing re-enables it'
        )

    def _joy_callback(self, msg: Joy):
        """On a button state change, disable on press and re-enable on release."""
        if self._button >= len(msg.buttons):
            return

        pressed = bool(msg.buttons[self._button])
        if pressed == self._last_state:
            return
        self._last_state = pressed

        # Held -> disabled (False), released -> enabled (True).
        self._set_enabled(not pressed)

    def _set_enabled(self, enabled: bool):
        """Call set_parameters on the target node with the new enabled value."""
        if not self._client.service_is_ready():
            self.get_logger().error(
                'collision_toggle_node: set_parameters service not available'
            )
            return

        req = SetParameters.Request()
        req.parameters = [
            Parameter(self._parameter_name, Parameter.Type.BOOL, enabled).to_parameter_msg()
        ]
        future = self._client.call_async(req)
        future.add_done_callback(lambda f: self._on_done(f, enabled))

    def _on_done(self, future, enabled: bool):
        """Log whether the set_parameters call succeeded."""
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f'collision_toggle_node: call failed: {e}')
            return

        if result.results and result.results[0].successful:
            self.get_logger().info(f'{self._parameter_name} -> {enabled}')
        else:
            reason = result.results[0].reason if result.results else 'no result returned'
            self.get_logger().error(f'collision_toggle_node: rejected ({reason})')


def main(args=None):
    """Initialize rclpy, spin CollisionToggleNode, and shut down cleanly."""
    rclpy.init(args=args)
    node = CollisionToggleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
