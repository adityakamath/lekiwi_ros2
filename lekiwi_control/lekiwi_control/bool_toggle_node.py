#!/usr/bin/env python3
"""
bool_toggle_node — generic SetBool toggle wrapper, configurable for any number of targets.

For each name listed in the 'toggles' parameter, exposes a std_srvs/srv/Trigger service that
flips an internal boolean and calls a target std_srvs/srv/SetBool service with the new value.
The flip only commits once the target service confirms success.

Each toggle also watches target_service's own _service_event introspection topic and keeps its
belief about current state in sync with every successful call to it, from any caller - not just
calls made through this node's own trigger_service. That closes two related gaps: a caller that
bypasses the toggle (e.g. Foxglove calling /waypoint_follow directly) can no longer desync it,
and if the target publishes its introspection with transient-local durability (see
STATE_SERVICE_QOS below, and the matching QoS on /emergency_stop, /twist_switch,
/waypoint_follow), a fresh toggle instance learns the target's real current state on startup
instead of blindly trusting initial_state. If the target isn't configured that way, this
subscription simply never receives anything and initial_state is used as before - a silent,
safe fallback, not a failure.

Parameters:
    toggles                   string[]  Names of toggles to create (each is a namespace below)
    <name>.trigger_service    string    Service this node exposes (std_srvs/srv/Trigger)
    <name>.target_service     string    Service this node calls (std_srvs/srv/SetBool)
    <name>.initial_state      bool      Assumed starting state, used until a real state event
                                         arrives (default: false)
"""

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, LivelinessPolicy, QoSProfile, ReliabilityPolicy
from service_msgs.msg import ServiceEventInfo
from std_srvs.srv import SetBool, SetBool_Event, Trigger

# Matches the QoS that /emergency_stop, /twist_switch, and /waypoint_follow configure on their
# own introspection (see sts_hardware_interface, twist_switch_node, waypoint_recorder_node).
# Requesting transient-local durability here is what makes replay work, not just a preference -
# a volatile request would connect fine but never receive the target's cached last event.
STATE_SERVICE_QOS = QoSProfile(
    depth=2,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    liveliness=LivelinessPolicy.AUTOMATIC,
    liveliness_lease_duration=Duration(seconds=1),
)


class BoolToggle:
    """One configured toggle: owns a Trigger server and a SetBool client."""

    def __init__(
        self,
        node: Node,
        name: str,
        trigger_service: str,
        target_service: str,
        initial_state: bool,
    ):
        """Create the Trigger server and SetBool client for this toggle."""
        self._node = node
        self._name = name
        self._active = initial_state
        self._target_service = target_service

        # Needs its own reentrant group, not the default - the Trigger handler below calls
        # and awaits this same client, so the default group would hang after the first call.
        self._client = node.create_client(
            SetBool, target_service, callback_group=ReentrantCallbackGroup()
        )
        node.create_service(Trigger, trigger_service, self._handle_trigger)

        # (client_gid, sequence_number) -> request value, populated on REQUEST_RECEIVED and
        # consumed on RESPONSE_SENT - same correlation lekiwi_audio's indicator_node uses.
        self._pending_target_calls = {}
        node.create_subscription(
            SetBool_Event,
            f'{target_service}/_service_event',
            self._on_target_service_event,
            STATE_SERVICE_QOS,
        )

        node.get_logger().info(
            f"toggle '{name}' ready: {trigger_service} -> {target_service} "
            f'(initial_state={initial_state})'
        )

    def _on_target_service_event(self, msg: SetBool_Event) -> None:
        """Sync _active to every successful call to target_service, from any caller."""
        key = (bytes(msg.info.client_gid), msg.info.sequence_number)

        if msg.info.event_type == ServiceEventInfo.REQUEST_RECEIVED:
            if msg.request:
                self._pending_target_calls[key] = msg.request[0].data
            if len(self._pending_target_calls) > 16:  # defensive cap, shouldn't normally fill
                self._pending_target_calls.pop(next(iter(self._pending_target_calls)), None)
            return

        if msg.info.event_type == ServiceEventInfo.RESPONSE_SENT:
            if key not in self._pending_target_calls or not msg.response:
                return
            request_value = self._pending_target_calls.pop(key)
            if not msg.response[0].success:
                return
            if request_value != self._active:
                self._node.get_logger().info(
                    f"toggle '{self._name}': synced to {self._target_service} -> "
                    f'{"ACTIVE" if request_value else "INACTIVE"}'
                )
            self._active = request_value

    async def _handle_trigger(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Flip the toggle and call target_service with the new state."""
        new_state = not self._active

        if not self._client.service_is_ready():
            response.success = False
            response.message = (
                f"toggle '{self._name}': target service '{self._target_service}' not available"
            )
            self._node.get_logger().error(response.message)
            return response

        req = SetBool.Request()
        req.data = new_state
        result = await self._client.call_async(req)

        if not result.success:
            response.success = False
            response.message = (
                f"toggle '{self._name}': target service rejected the request: {result.message}"
            )
            self._node.get_logger().error(response.message)
            return response

        # Only commit the flip once the target has confirmed it - keeps this node's belief
        # from drifting from the target's actual state on a failed call.
        self._active = new_state
        response.success = True
        response.message = f"toggle '{self._name}' -> {'ACTIVE' if new_state else 'INACTIVE'}"
        self._node.get_logger().info(response.message)
        return response


class BoolToggleNode(Node):
    """Owns one BoolToggle per name listed in the 'toggles' parameter."""

    def __init__(self):
        """Declare the 'toggles' parameter and construct each configured BoolToggle."""
        super().__init__('bool_toggle_node')

        # Default ['']  rather than [] - rclpy can't infer an element type from a truly empty
        # list default. Filtered below.
        self.declare_parameter('toggles', [''])
        toggle_names = [name for name in self.get_parameter('toggles').value if name]

        self._toggles = []
        for name in toggle_names:
            self.declare_parameter(f'{name}.trigger_service', '')
            self.declare_parameter(f'{name}.target_service', '')
            self.declare_parameter(f'{name}.initial_state', False)

            trigger_service = self.get_parameter(f'{name}.trigger_service').value
            target_service = self.get_parameter(f'{name}.target_service').value
            initial_state = self.get_parameter(f'{name}.initial_state').value

            if not trigger_service or not target_service:
                raise ValueError(
                    f"toggle '{name}' is missing trigger_service or target_service"
                )

            self._toggles.append(
                BoolToggle(self, name, trigger_service, target_service, initial_state)
            )

        if not self._toggles:
            self.get_logger().warn('bool_toggle_node started with no toggles configured')


def main(args=None):
    """Initialize rclpy, spin BoolToggleNode, and shut down cleanly."""
    rclpy.init(args=args)
    node = BoolToggleNode()
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
