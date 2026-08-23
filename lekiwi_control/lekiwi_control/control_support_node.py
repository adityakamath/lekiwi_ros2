#!/usr/bin/env python3
"""Combined process for lekiwi_control's non-real-time support nodes: bool_toggle_node,
twist_switch_node. Both are lightweight, non-blocking, and always launch together from
control.launch.py - sharing one process/executor cuts DDS participant count and per-process
overhead for a group that has no reason to run as two separate processes.
"""

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from lekiwi_control.bool_toggle_node import BoolToggleNode
from lekiwi_control.twist_switch_node import TeleopSwitchNode


def main(args=None):
    """Initialize rclpy, spin both nodes on one MultiThreadedExecutor, and shut down cleanly."""
    rclpy.init(args=args)
    nodes = [BoolToggleNode(), TeleopSwitchNode()]
    executor = MultiThreadedExecutor(num_threads=2)
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
