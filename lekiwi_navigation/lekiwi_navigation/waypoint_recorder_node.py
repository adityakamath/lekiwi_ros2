#!/usr/bin/env python3
"""
waypoint_recorder_node — service-driven waypoint recording and patrol trigger.

TEST BUILD, not the production implementation: uses nav2_simple_commander.BasicNavigator's
followWaypoints(..., number_of_loops=, goal_index=) instead of a raw FollowWaypoints
ActionClient, specifically to exercise those two fields (added upstream in navigation2) against
a real robot. main branch keeps the raw-ActionClient version - this build deliberately drops
per-goal identity tracking (safe against a goal preempted mid-flight), external-detour
auto-resume via /navigate_to_pose status, retry/unreachable-waypoint removal, and diagnostics,
none of which are needed to validate number_of_loops/goal_index. Do not merge to main.

Three std_srvs/srv/SetBool services, usable from joy_teleop button bindings or Foxglove's
"Call Service" panel:

  /record_waypoint
      true  - record the robot's current pose (map -> base_footprint TF) as the next waypoint,
              and publish a visualization_msgs/MarkerArray marker for it.
      false - no-op

  /waypoint_follow
      true  - send the recorded waypoints via BasicNavigator.followWaypoints(), looping per
              number_of_loops. If already patrolling, this is a no-op. Resumes from the last
              polled waypoint index rather than restarting from 0 (goal_index).
      false - cancel the active patrol via BasicNavigator.cancelTask(), if any.

  /reset_waypoints
      true  - cancel any active patrol, clear the recorded waypoints + their markers, and
              forget the resume position.
      false - no-op

number_of_loops is the total number of passes through the waypoint list per start; 0 means
loop forever.

Parameters:
    number_of_loops  int    Total passes through the waypoint list per start
                             (default: 0, i.e. loop forever; N>0 = exactly N passes)
    frame_id         str    TF frame waypoints are recorded in (default: map)
    marker_topic     str    Topic for the waypoint MarkerArray (default: /waypoint_markers)
    poll_period      float  Feedback poll interval while patrolling, in seconds (default: 0.5) -
                             BasicNavigator has no feedback callback, only getFeedback() for the
                             last-seen message, so progress/resume tracking is polled instead of
                             pushed.
"""

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import SetBool
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

# FollowWaypoints.Goal.number_of_loops is a uint32 - this is its max value, used as the
# practical stand-in for "loop forever" (the field has no literal infinite option).
_UINT32_MAX = 4294967295


class WaypointRecorderNode(Node):
    """Records waypoints from the robot's pose and drives a looping patrol via BasicNavigator."""

    def __init__(self):
        """Declare parameters, set up TF/publishers/BasicNavigator, and create the services."""
        super().__init__('waypoint_recorder_node')

        self.declare_parameter('number_of_loops', 0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('marker_topic', '/waypoint_markers')
        self.declare_parameter('poll_period', 0.5)

        self._number_of_loops = self.get_parameter('number_of_loops').value
        self._frame_id = self.get_parameter('frame_id').value
        marker_topic = self.get_parameter('marker_topic').value
        self._poll_period = self.get_parameter('poll_period').value

        self._waypoints = []
        self._patrolling = False
        self._resume_index = 0
        self._poll_timer = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)

        # A separate node from `self`, deliberately: BasicNavigator's blocking calls
        # (rclpy.spin_until_future_complete with no explicit executor) fall back to rclpy's
        # implicit global default executor, not whichever executor this node itself is added
        # to. Keeping them as two node objects, and spinning this one on its own explicit
        # executor (see main()) instead of that same global one, means those blocking calls
        # can't re-enter this node's own callback dispatch while already inside one of its
        # callbacks (every service handler here calls into self._navigator).
        self._navigator = BasicNavigator(node_name='waypoint_recorder_navigator')

        self.create_service(SetBool, '/record_waypoint', self._handle_record)
        self.create_service(SetBool, '/waypoint_follow', self._handle_set_following)
        self.create_service(SetBool, '/reset_waypoints', self._handle_reset)

        self.get_logger().info(
            'waypoint_recorder_node (BasicNavigator test build) ready: /record_waypoint, '
            '/waypoint_follow, /reset_waypoints'
        )

    def destroy_node(self) -> bool:
        """Tear down the BasicNavigator's own node before this one."""
        self._navigator.destroy_node()
        return super().destroy_node()

    def _handle_record(self, request: SetBool.Request, response: SetBool.Response):
        """Record the robot's current pose as the next waypoint."""
        if not request.data:
            response.success = True
            response.message = 'No-op (set data: true to record).'
            return response

        try:
            tf = self._tf_buffer.lookup_transform(
                self._frame_id, 'base_footprint', Time(), Duration(seconds=0.5))
        except Exception as e:
            response.success = False
            response.message = f'TF lookup failed: {e}'
            self.get_logger().error(response.message)
            return response

        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = 0.0  # map's ground plane is aligned with base_footprint
        pose.pose.orientation = tf.transform.rotation

        self._waypoints.append(pose)
        index = len(self._waypoints) - 1
        self._publish_markers()

        response.success = True
        response.message = (
            f'Waypoint {index} recorded at '
            f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
        )
        self.get_logger().info(response.message)
        return response

    def _publish_markers(self):
        """Publish an arrow for every recorded waypoint. Leading DELETEALL clears stale ones."""
        markers = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)
        for i, pose in enumerate(self._waypoints):
            arrow = Marker()
            arrow.header.frame_id = self._frame_id
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = 'waypoints'
            arrow.id = i
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose = pose.pose
            arrow.scale.x = 0.3
            arrow.scale.y = 0.06
            arrow.scale.z = 0.06
            arrow.color.g = 1.0
            arrow.color.a = 1.0
            markers.markers.append(arrow)
        self._marker_pub.publish(markers)

    def _clear_markers(self):
        """Delete every published waypoint marker."""
        markers = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)
        self._marker_pub.publish(markers)

    def _total_passes(self) -> int:
        """Return total passes (the number_of_loops param's 0 maps to the action field max)."""
        return self._number_of_loops if self._number_of_loops > 0 else _UINT32_MAX

    def _handle_set_following(self, request: SetBool.Request, response: SetBool.Response):
        """Dispatch to _start_patrol or _stop_patrol based on request.data."""
        if request.data:
            return self._start_patrol(response)
        return self._stop_patrol(response)

    def _start_patrol(self, response: SetBool.Response):
        """Send a followWaypoints goal, resuming from self._resume_index if applicable."""
        if self._patrolling:
            response.success = True
            response.message = 'Already patrolling.'
            self.get_logger().info(response.message)
            return response

        if not self._waypoints:
            response.success = False
            response.message = 'No waypoints recorded - call /record_waypoint first.'
            self.get_logger().error(response.message)
            return response

        start_index = self._resume_index if self._resume_index < len(self._waypoints) else 0
        # number_of_loops is loops *after* the first pass, not total passes - see
        # _total_passes for the number_of_loops parameter's actual semantics.
        task = self._navigator.followWaypoints(
            list(self._waypoints),
            number_of_loops=self._total_passes() - 1,
            goal_index=start_index,
        )
        if task is None:
            response.success = False
            response.message = 'FollowWaypoints goal was rejected.'
            self.get_logger().error(response.message)
            return response

        self._patrolling = True
        self._poll_timer = self.create_timer(self._poll_period, self._poll_patrol)

        passes_desc = (
            'continuous' if self._number_of_loops == 0 else f'{self._number_of_loops} pass(es)'
        )
        response.success = True
        response.message = (
            f'Patrol started at waypoint {start_index}, {len(self._waypoints)} waypoints '
            f'total, {passes_desc}.'
        )
        self.get_logger().info(response.message)
        return response

    def _stop_patrol(self, response: SetBool.Response):
        """Cancel the active patrol, if any."""
        if not self._patrolling:
            response.success = True
            response.message = 'No active patrol to stop.'
            self.get_logger().info(response.message)
            return response

        self._navigator.cancelTask()
        self._patrolling = False
        if self._poll_timer is not None:
            self._poll_timer.cancel()
            self._poll_timer = None

        response.success = True
        response.message = (
            f'Patrol cancellation requested (resuming from waypoint {self._resume_index}).'
        )
        self.get_logger().info(response.message)
        return response

    def _poll_patrol(self):
        """Poll BasicNavigator for feedback/completion - it has no push callback for either."""
        feedback = self._navigator.getFeedback()
        if feedback is not None and feedback.current_waypoint != self._resume_index:
            self._resume_index = feedback.current_waypoint
            self.get_logger().info(f'Patrol: heading to waypoint {self._resume_index}.')

        if not self._navigator.isTaskComplete():
            return

        result = self._navigator.getResult()
        self._patrolling = False
        self._poll_timer.cancel()
        self._poll_timer = None

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Patrol finished.')
            self._resume_index = 0
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Patrol stopped.')
        else:
            self.get_logger().error(f'Patrol failed: {result}')

    def _handle_reset(self, request: SetBool.Request, response: SetBool.Response):
        """Cancel any active patrol and clear waypoints/markers/resume state."""
        if not request.data:
            response.success = True
            response.message = 'No-op (set data: true to reset).'
            return response

        if self._patrolling:
            self._navigator.cancelTask()
            self._patrolling = False
        if self._poll_timer is not None:
            self._poll_timer.cancel()
            self._poll_timer = None

        count = len(self._waypoints)
        self._waypoints.clear()
        self._resume_index = 0
        self._clear_markers()

        response.success = True
        response.message = f'Reset: cancelled any active patrol, cleared {count} waypoint(s).'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    """Initialize rclpy, spin WaypointRecorderNode on its own executor, and shut down cleanly.

    Deliberately not the bare rclpy.spin(node) shortcut - that adds this node to rclpy's
    implicit global default executor, which is exactly the executor BasicNavigator's blocking
    calls fall back to internally (see the comment on self._navigator in __init__). Spinning on
    an explicit executor here keeps the two separate.
    """
    rclpy.init(args=args)
    node = WaypointRecorderNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
