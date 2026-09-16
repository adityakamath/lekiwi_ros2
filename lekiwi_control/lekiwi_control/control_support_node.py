#!/usr/bin/env python3
"""Combined process for lekiwi_control's non-real-time support nodes: bool_toggle_node,
twist_switch_node, collision_toggle_node. All three are lightweight, non-blocking, and
always launch together from control.launch.py - sharing one process/executor cuts DDS
participant count and per-process overhead for a group that has no reason to run as three
separate processes. collision_toggle_node's target (collision_monitor, in lekiwi_navigation)
doesn't need to exist for this process to run - it no-ops gracefully if it doesn't, see its
own _set_enabled.
"""

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from lekiwi_control.bool_toggle_node import BoolToggleNode
from lekiwi_control.collision_toggle_node import CollisionToggleNode
from lekiwi_control.twist_switch_node import TeleopSwitchNode


def main(args=None):
    """Initialize rclpy, spin all three nodes on one MultiThreadedExecutor, and shut down cleanly."""
    rclpy.init(args=args)
    nodes = [BoolToggleNode(), TeleopSwitchNode(), CollisionToggleNode()]
    executor = MultiThreadedExecutor(num_threads=3)
    for node in nodes:
        executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        print('Received keyboard interrupt!')
    except ExternalShutdownException:
        print('Received external shutdown request!')
    finally:
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
