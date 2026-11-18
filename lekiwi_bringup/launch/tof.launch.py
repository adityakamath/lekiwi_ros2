#!/usr/bin/env python3
"""Launch the SparkFun VL53L5CX ToF imager for LeKiwi.

Starts tof_imager_ros's lifecycle node with config/tof.yaml and takes it through configure and
activate; its point cloud is published on tof/pointcloud in tof_frame."""

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """Launch tof_imager_publisher as a lifecycle node and configure then activate it."""
    tof_node = LifecycleNode(
        package='tof_imager_ros',
        executable='tof_imager_publisher',
        name='tof_imager',
        namespace='',
        output='log',
        parameters=[PathJoinSubstitution([FindPackageShare('lekiwi_bringup'), 'config', 'tof.yaml'])],
        remappings=[('pointcloud', 'tof/pointcloud')],
    )
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(tof_node),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))
    activate_when_configured = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=tof_node,
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(tof_node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        ))],
    ))
    return LaunchDescription([tof_node, configure, activate_when_configured])
