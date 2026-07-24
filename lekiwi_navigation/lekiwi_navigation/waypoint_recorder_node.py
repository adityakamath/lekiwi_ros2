#!/usr/bin/env python3
"""
waypoint_recorder_node — service-driven waypoint recording and patrol trigger.

Three std_srvs/srv/SetBool services, usable from joy_teleop button bindings or Foxglove's
"Call Service" panel:

  /record_waypoint
      true  - record the robot's current pose (map -> base_footprint TF) as the next waypoint,
              and publish a visualization_msgs/MarkerArray marker for it. If already
              patrolling, queues it instead of adding it immediately - it joins the patrol at
              the start of the next loop, marked with a different marker color until then.
      false - no-op

  /waypoint_follow
      true  - send the recorded waypoints to the /follow_waypoints action, looping
              continuously (see number_of_loops). If already patrolling, this is a no-op. If
              resuming after a stop, continues toward the same waypoint it was last heading to
              rather than restarting from 0 - pause/resume, not stop/restart.
      false - cancel the active /follow_waypoints goal, if any (does not clear waypoints, and
              does not forget which waypoint it was heading to)

  /reset_waypoints
      true  - cancel any active patrol, clear the recorded waypoints + their markers, and
              forget the pause/resume position - the next start is a true fresh start
      false - no-op

number_of_loops is the total number of passes through the waypoint list per start; 0 means
loop forever.

If something else sends the robot a /navigate_to_pose goal mid-patrol (RViz's "2D Nav Goal",
Foxglove's goal-pose panel), it's treated as a deliberate detour rather than an error: the
current leg aborts, the robot follows the new goal, and once /navigate_to_pose is free again
the patrol automatically resumes toward the interrupted waypoint. If the same waypoint fails
max_retries times in a row instead (no detour resolving it - genuinely unreachable), it's
removed from the list and the patrol continues to the next one; later loops never target it.

Progress (current loop, from/to waypoint, ETA, distance, recovery count) is published as
diagnostic_msgs/msg/DiagnosticArray on /patrol_diagnostics rather than a visualization - WARN while
paused waiting to auto-resume, STALE when stopped deliberately, OK/WARN(recoveries>0) while
actively patrolling.

Cancelling or resetting also clears nav2's own plan-visualization topics (/plan and friends),
which otherwise keep showing the last path received even after the goal that produced it is
gone.

Parameters:
    number_of_loops  int  Total passes through the waypoint list per start
                          (default: 0, i.e. loop forever; N>0 = exactly N passes)
    frame_id         str  TF frame waypoints are recorded in (default: map)
    marker_topic     str  Topic for the waypoint MarkerArray (default: /waypoint_markers)
    max_retries      int  Consecutive failures at the same waypoint before giving up on it
                          and removing it from the list (default: 3)
"""

import copy

from action_msgs.msg import GoalStatus, GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy._rclpy_pybind11.service_introspection import ServiceIntrospectionState
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    LivelinessPolicy,
    qos_profile_services_default,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from nav_msgs.msg import Path
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

# /waypoint_follow reflects ongoing patrol state (running vs. stopped), not a one-shot action
# like /record_waypoint and /reset_waypoints - a late-joining subscriber (e.g. the audio
# indicator node, or bool_toggle_node syncing its own belief about current state) should learn
# whether a patrol is active on connect rather than waiting for the next start/stop. Depth 2 is
# enough to cache the last request+response pair. The liveliness lease lets a subscriber notice
# if this node dies without a clean exit.
STATE_SERVICE_QOS = QoSProfile(
    depth=2,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    liveliness=LivelinessPolicy.AUTOMATIC,
    liveliness_lease_duration=Duration(seconds=1),
)

# nav2's own plan-visualization topics - cleared explicitly on pause/reset (see
# _clear_plan_visualizations), since nav2 leaves them showing a stale path after a cancel.
_PLAN_TOPICS = [
    '/plan',
    '/unsmoothed_plan',
    '/controller_server/transformed_global_plan',
    '/controller_server/optimal_path',
]

# FollowWaypoints.Goal.number_of_loops is a uint32 - this is its max value, used as the
# practical stand-in for "loop forever" (the field has no literal infinite option).
_UINT32_MAX = 4294967295


class WaypointRecorderNode(Node):
    """Records waypoints from the robot's pose and drives a looping patrol between them."""

    def __init__(self):
        """Declare parameters, set up TF/publishers/the action client, and create the services."""
        super().__init__('waypoint_recorder_node')

        self.declare_parameter('number_of_loops', 0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('marker_topic', '/waypoint_markers')
        self.declare_parameter('max_retries', 3)

        self._number_of_loops = self.get_parameter('number_of_loops').value
        self._frame_id = self.get_parameter('frame_id').value
        marker_topic = self.get_parameter('marker_topic').value
        self._max_retries = self.get_parameter('max_retries').value

        self._waypoints = []
        self._pending_waypoints = []
        self._goal_handle = None
        self._current_loop = 0
        self._last_feedback_waypoint = None
        self._from_waypoint = -1
        self._to_waypoint = 0
        self._leg_durations = []
        self._leg_start_time = None
        self._latest_nav_feedback = None
        self._latest_nav_status = None
        self._resume_timer = None
        self._superseded_goal_handle_ids = set()
        self._failed_waypoint = None
        self._consecutive_failures = 0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, '/patrol_diagnostics', 10)
        self._plan_pubs = [self.create_publisher(Path, topic, 10) for topic in _PLAN_TOPICS]
        self._action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            '/navigate_to_pose/_action/feedback',
            self._on_nav_feedback,
            10,
        )
        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self._on_nav_status,
            10,
        )

        # Introspection lets a listener (e.g. the audio indicator node) observe every
        # call's request+response on <service>/_service_event, regardless of caller.
        record_srv = self.create_service(SetBool, '/record_waypoint', self._handle_record)
        follow_srv = self.create_service(SetBool, '/waypoint_follow', self._handle_set_following)
        reset_srv = self.create_service(SetBool, '/reset_waypoints', self._handle_reset)
        for srv in (record_srv, reset_srv):
            srv.configure_introspection(
                self.get_clock(), qos_profile_services_default, ServiceIntrospectionState.CONTENTS)
        # /waypoint_follow gets the transient-local state QoS (see STATE_SERVICE_QOS above) -
        # record/reset are one-shot actions where replaying a stale call on a late-joining
        # subscriber would announce something that didn't just happen.
        follow_srv.configure_introspection(
            self.get_clock(), STATE_SERVICE_QOS, ServiceIntrospectionState.CONTENTS)

        self.get_logger().info(
            'waypoint_recorder_node ready: /record_waypoint, /waypoint_follow, /reset_waypoints'
        )

    def _handle_record(self, request: SetBool.Request, response: SetBool.Response):
        """Record the robot's pose as the next waypoint, queuing it if a patrol is active."""
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

        response.success = True

        if self._goal_handle is not None:
            # Queued, not spliced in immediately - an immediate splice would preempt the
            # in-flight goal, and nav2_waypoint_follower resets goal_index to 0 on any
            # preemption, jumping the patrol to the wrong waypoint.
            self._pending_waypoints.append(pose)
            index = len(self._waypoints) + len(self._pending_waypoints) - 1
            response.message = (
                f'Waypoint {index} recorded at ({pose.pose.position.x:.2f}, '
                f'{pose.pose.position.y:.2f}) and queued for the next loop'
            )
        else:
            self._waypoints.append(pose)
            index = len(self._waypoints) - 1
            response.message = (
                f'Waypoint {index} recorded at '
                f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
            )

        self._publish_markers()
        self.get_logger().info(response.message)
        return response

    def _publish_markers(self):
        """Publish an arrow + index label for every recorded waypoint, including queued ones.

        Leading DELETEALL ensures stale markers (e.g. orange ghosts left over when pending
        waypoints are merged into the active patrol) are cleared atomically before the fresh
        state is added.  RViz2 and Foxglove both process MarkerArrays sequentially within a
        single message, so this produces no visible flicker.
        """
        markers = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)
        self._append_waypoint_markers(markers, self._waypoints, 'waypoints', (0.0, 1.0, 0.0))
        self._append_waypoint_markers(
            markers, self._pending_waypoints, 'pending_waypoints', (1.0, 0.65, 0.0),
            index_offset=len(self._waypoints))
        self._marker_pub.publish(markers)

    def _append_waypoint_markers(self, markers, waypoints, ns, color, index_offset=0):
        """Append an arrow + index label for each pose in waypoints to markers, in-place."""
        r, g, b = color
        for i, pose in enumerate(waypoints):
            label_index = index_offset + i
            arrow = Marker()
            arrow.header.frame_id = self._frame_id
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = ns
            arrow.id = i * 2
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose = copy.deepcopy(pose.pose)
            arrow.scale.x = 0.3
            arrow.scale.y = 0.06
            arrow.scale.z = 0.06
            arrow.color.r = r
            arrow.color.g = g
            arrow.color.b = b
            arrow.color.a = 1.0
            markers.markers.append(arrow)

            label = Marker()
            label.header.frame_id = self._frame_id
            label.header.stamp = self.get_clock().now().to_msg()
            label.ns = ns
            label.id = i * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            # deepcopy - without it, label.pose and the waypoint's own stored pose are the same
            # object, and the += 0.3 below would mutate the waypoint's actual stored z in place.
            label.pose = copy.deepcopy(pose.pose)
            label.pose.position.z += 0.3
            label.scale.z = 0.2
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = str(label_index)
            markers.markers.append(label)

    def _clear_markers(self):
        """Delete every published waypoint marker."""
        markers = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)
        self._marker_pub.publish(markers)

    def _handle_set_following(self, request: SetBool.Request, response: SetBool.Response):
        """Dispatch to _start_patrol or _stop_patrol based on request.data."""
        if request.data:
            return self._start_patrol(response)
        return self._stop_patrol(response)

    def _start_patrol(self, response: SetBool.Response):
        """Validate preconditions, then send a FollowWaypoints goal via _send_patrol_goal."""
        if self._goal_handle is not None:
            response.success = True
            response.message = 'Already patrolling.'
            self.get_logger().info(response.message)
            return response

        if not self._waypoints:
            response.success = False
            response.message = 'No waypoints recorded - call /record_waypoint first.'
            self.get_logger().error(response.message)
            return response

        if not self._action_client.server_is_ready():
            response.success = False
            response.message = '/follow_waypoints action server not available.'
            self.get_logger().error(response.message)
            return response

        resuming, start_index = self._send_patrol_goal()
        passes_desc = 'continuous' if self._number_of_loops == 0 else f'{self._number_of_loops} pass(es)'
        response.success = True
        response.message = (
            f'Patrol {"resumed at" if resuming else "requested:"} waypoint {start_index}, '
            f'{len(self._waypoints)} waypoints total, {passes_desc}.'
        )
        self.get_logger().info(response.message)
        return response

    def _total_passes(self) -> int:
        """Total passes through the waypoint list (the number_of_loops parameter's 0 means
        loop forever, mapped here to the action field's max representable value)."""
        return self._number_of_loops if self._number_of_loops > 0 else _UINT32_MAX

    def _send_patrol_goal(self):
        """Send a FollowWaypoints goal, resuming from self._to_waypoint if applicable.

        Shared by _start_patrol (the /waypoint_follow service handler), the automatic
        resume-after-external-detour path in _on_nav_status, and the loop-boundary merge in
        _on_feedback - all need the same resume-aware goal construction. Returns
        (resuming, start_index).
        """
        total = len(self._waypoints)
        resuming = self._last_feedback_waypoint is not None
        start_index = self._to_waypoint if resuming else 0
        if start_index >= total:
            start_index = 0  # stale resume point - fall back

        goal = FollowWaypoints.Goal()
        goal.poses = list(self._waypoints)
        goal.goal_index = start_index
        # FollowWaypoints.Goal.number_of_loops is loops *after* the first pass, not total
        # passes - see _total_passes for the number_of_loops parameter's actual semantics.
        goal.number_of_loops = self._total_passes() - 1

        if not resuming:
            self._current_loop = 0
            self._leg_durations = []
        self._last_feedback_waypoint = None
        self._leg_start_time = self.get_clock().now()
        self._latest_nav_feedback = None

        future = self._action_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)
        return resuming, start_index

    def _on_goal_response(self, future):
        """Record the accepted goal handle and start waiting for its result."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Patrol goal was rejected.')
            return
        self._goal_handle = goal_handle
        self.get_logger().info('Patrol goal accepted.')
        goal_handle.get_result_async().add_done_callback(
            lambda f: self._on_result(f, goal_handle))

    def _on_feedback(self, feedback_msg):
        """Track loop/waypoint progress, merge queued waypoints at a loop boundary, and publish
        diagnostics on every change."""
        current_waypoint = feedback_msg.feedback.current_waypoint
        if current_waypoint == self._last_feedback_waypoint:
            return  # no change since the last feedback tick - nothing new to report

        now = self.get_clock().now()
        loop_wrapped = False
        if self._last_feedback_waypoint is not None:
            leg_duration = (now - self._leg_start_time).nanoseconds * 1e-9
            self._leg_durations.append(leg_duration)
            # current_waypoint resets to 0 at the start of each new loop (nav2 doesn't report
            # the loop number itself) - a drop to a lower value is exactly that reset.
            if current_waypoint < self._last_feedback_waypoint:
                self._current_loop += 1
                loop_wrapped = True

        if current_waypoint != self._to_waypoint:
            # Genuinely advanced, not a retry - clear any failure streak for the old target.
            self._failed_waypoint = None
            self._consecutive_failures = 0

        self._from_waypoint = self._last_feedback_waypoint if (
            self._last_feedback_waypoint is not None) else -1
        self._to_waypoint = current_waypoint
        self._last_feedback_waypoint = current_waypoint
        self._leg_start_time = now

        total = len(self._waypoints)
        self.get_logger().info(
            f'Patrol loop {self._current_loop}: heading to waypoint {current_waypoint}/'
            f'{total - 1}'
        )
        self._publish_diagnostics()

        if loop_wrapped and self._pending_waypoints:
            self._merge_pending_waypoints()

    def _merge_pending_waypoints(self):
        """Fold queued recordings into self._waypoints and resend the goal.

        Called from a loop wrap in _on_feedback, and from _remove_unreachable_waypoint when
        removing the last active waypoint would otherwise orphan pending ones.
        """
        count = len(self._pending_waypoints)
        self._waypoints.extend(self._pending_waypoints)
        self._pending_waypoints = []
        self._publish_markers()
        self.get_logger().info(
            f'{count} queued waypoint(s) joined the patrol at the start of loop '
            f'{self._current_loop}.'
        )
        if self._goal_handle is not None:
            self._superseded_goal_handle_ids.add(id(self._goal_handle))
        self._send_patrol_goal()

    def _on_nav_feedback(self, msg):
        """Cache the latest /navigate_to_pose feedback for the diagnostics ETA fields."""
        if self._goal_handle is None:
            return  # not patrolling - this /navigate_to_pose goal isn't ours
        self._latest_nav_feedback = msg.feedback
        self._publish_diagnostics()

    def _on_result(self, future, goal_handle):
        """Handle the FollowWaypoints result: clean stop, finish, auto-resume, or give up.

        goal_handle identifies which specific goal this result belongs to, since a spliced-in
        recording (_handle_record) can preempt the current one with a new goal.
        """
        result = future.result()

        if id(goal_handle) in self._superseded_goal_handle_ids:
            self._superseded_goal_handle_ids.discard(id(goal_handle))
            return  # expected - this goal was deliberately preempted by a splice

        if goal_handle is not self._goal_handle:
            return  # stale result for a goal we no longer track - ignore defensively

        self._goal_handle = None

        if result.status == GoalStatus.STATUS_CANCELED:
            self._clear_plan_visualizations()
            self._publish_diagnostics(active=False)
            self.get_logger().info('Patrol stopped.')
            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._clear_plan_visualizations()
            self._publish_diagnostics(active=False)
            self.get_logger().info('Patrol finished.')
            return

        # Not a deliberate stop - wait until /navigate_to_pose is free, then resume. Track
        # consecutive failures at the same waypoint index so a genuinely unreachable waypoint
        # eventually gets dropped instead of retried forever (see _remove_unreachable_waypoint).
        if self._to_waypoint == self._failed_waypoint:
            self._consecutive_failures += 1
        else:
            self._failed_waypoint = self._to_waypoint
            self._consecutive_failures = 1

        if self._consecutive_failures >= self._max_retries:
            self._remove_unreachable_waypoint()
            return

        self._publish_diagnostics(active=False, waiting=True)
        self.get_logger().warn(
            f'Patrol interrupted (status={result.status}) - resuming once /navigate_to_pose '
            f'is free (attempt {self._consecutive_failures}/{self._max_retries}).'
        )
        if self._resume_timer is None:
            self._resume_timer = self.create_timer(0.5, self._try_resume_after_handoff)

    def _remove_unreachable_waypoint(self):
        """Drop the waypoint that's failed max_retries times in a row and resume past it."""
        index = self._failed_waypoint
        total_before = len(self._waypoints)
        if 0 <= index < total_before:
            del self._waypoints[index]
        self._failed_waypoint = None
        self._consecutive_failures = 0

        self.get_logger().error(
            f'Waypoint {index} unreachable after {self._max_retries} attempts - removed from '
            f'the patrol ({len(self._waypoints)} waypoint(s) left).'
        )
        self._publish_markers()  # DELETEALL + redraw — removal shifts every later marker ID

        if not self._waypoints:
            if self._pending_waypoints:
                self._merge_pending_waypoints()
                return

            self._clear_plan_visualizations()
            self._current_loop = 0
            self._last_feedback_waypoint = None
            self._from_waypoint = -1
            self._to_waypoint = 0
            self._leg_durations = []
            self._publish_diagnostics(active=False)
            self.get_logger().error('No waypoints left - patrol stopped.')
            return

        # self._to_waypoint (left untouched) now naturally refers to the waypoint that shifted
        # into the removed index, since list indices shift down by one after a deletion.
        self._send_patrol_goal()

    def _try_resume_after_handoff(self):
        """If /navigate_to_pose is free, cancel the recheck timer and resume the patrol."""
        if self._navigate_to_pose_busy():
            return  # still occupied - try again next timer tick or status update
        self._resume_timer.cancel()
        self._resume_timer = None
        self.get_logger().info('/navigate_to_pose is free - resuming patrol.')
        self._send_patrol_goal()

    def _navigate_to_pose_busy(self):
        """Return whether any goal is currently active on /navigate_to_pose."""
        if self._latest_nav_status is None:
            return False
        active = (
            GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING,
        )
        return any(status.status in active for status in self._latest_nav_status.status_list)

    def _on_nav_status(self, msg):
        """Cache the latest /navigate_to_pose status and retry a pending resume if one's due."""
        self._latest_nav_status = msg
        if self._resume_timer is not None:
            self._try_resume_after_handoff()

    def _stop_patrol(self, response: SetBool.Response):
        """Cancel the active goal or pending auto-resume, and clear plan visualizations."""
        if self._resume_timer is not None:
            # Was waiting to auto-resume after an external-goal detour - an explicit stop
            # request here means give up on that, not keep waiting.
            self._resume_timer.cancel()
            self._resume_timer = None
            self._clear_plan_visualizations()
            self._publish_diagnostics(active=False)
            response.success = True
            response.message = 'Patrol cancellation requested (was waiting to auto-resume).'
            self.get_logger().info(response.message)
            return response

        if self._goal_handle is None:
            response.success = True
            response.message = 'No active patrol to stop.'
            self.get_logger().info(response.message)
            return response

        # _goal_handle is cleared by _on_result once cancellation is confirmed, not here - a
        # second stop request just re-requests it (harmless), and a start request cleanly
        # preempts it.
        self._clear_plan_visualizations()
        self._goal_handle.cancel_goal_async()
        response.success = True
        response.message = 'Patrol cancellation requested.'
        self.get_logger().info(response.message)
        return response

    def _estimate_loop_duration(self):
        """Estimate total loop duration from observed leg times, or the current ETA as a fallback."""
        total = len(self._waypoints)
        if total == 0:
            return None
        if self._leg_durations:
            return (sum(self._leg_durations) / len(self._leg_durations)) * total
        if self._latest_nav_feedback is not None:
            eta = self._latest_nav_feedback.estimated_time_remaining
            return (eta.sec + eta.nanosec * 1e-9) * total
        return None

    def _clear_plan_visualizations(self):
        """Publish an empty Path to every nav2 plan topic to clear stale visualizations."""
        empty_path = Path()
        for pub in self._plan_pubs:
            pub.publish(empty_path)

    def _publish_diagnostics(self, active: bool = True, waiting: bool = False):
        """Publish the current patrol status to /patrol_diagnostics."""
        status = DiagnosticStatus()
        status.name = 'waypoint_recorder: patrol'
        status.hardware_id = ''

        if waiting:
            status.level = DiagnosticStatus.WARN
            status.message = (
                f'Patrol interrupted - waiting for /navigate_to_pose to free up '
                f'(attempt {self._consecutive_failures}/{self._max_retries})'
            )
        elif not active:
            status.level = DiagnosticStatus.STALE
            status.message = 'Not patrolling'
        else:
            total = len(self._waypoints)
            values = [
                KeyValue(key='current_loop', value=str(self._current_loop)),
                KeyValue(key='from_waypoint', value=str(self._from_waypoint)),
                KeyValue(key='to_waypoint', value=str(self._to_waypoint)),
                KeyValue(key='total_waypoints', value=str(total)),
            ]

            nav_feedback = self._latest_nav_feedback
            recoveries = 0
            if nav_feedback is not None:
                eta = nav_feedback.estimated_time_remaining
                eta_sec = eta.sec + eta.nanosec * 1e-9
                recoveries = nav_feedback.number_of_recoveries
                values.append(
                    KeyValue(key='estimated_time_to_waypoint_sec', value=f'{eta_sec:.1f}'))
                values.append(KeyValue(
                    key='distance_to_waypoint_m',
                    value=f'{nav_feedback.distance_remaining:.2f}'))
                values.append(KeyValue(key='number_of_recoveries', value=str(recoveries)))

            estimated_loop = self._estimate_loop_duration()
            if estimated_loop is not None:
                values.append(
                    KeyValue(key='estimated_loop_duration_sec', value=f'{estimated_loop:.1f}'))

            status.level = DiagnosticStatus.WARN if recoveries > 0 else DiagnosticStatus.OK
            status.message = (
                f'Loop {self._current_loop}: heading to waypoint {self._to_waypoint}/'
                f'{total - 1}'
            )
            status.values = values

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self._diagnostics_pub.publish(msg)

    def _handle_reset(self, request: SetBool.Request, response: SetBool.Response):
        """Cancel any active patrol, clear waypoints/markers/plans, and reset patrol state."""
        if not request.data:
            response.success = True
            response.message = 'No-op (set data: true to reset).'
            return response

        if self._resume_timer is not None:
            self._resume_timer.cancel()
            self._resume_timer = None

        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

        count = len(self._waypoints) + len(self._pending_waypoints)
        self._waypoints.clear()
        self._pending_waypoints.clear()
        self._clear_markers()  # DELETEALL
        self._clear_plan_visualizations()

        # Forget the pause/resume position too - otherwise the next start would try to resume
        # toward a now-meaningless index from before this reset.
        self._current_loop = 0
        self._last_feedback_waypoint = None
        self._from_waypoint = -1
        self._to_waypoint = 0
        self._leg_durations = []
        self._latest_nav_feedback = None
        self._failed_waypoint = None
        self._consecutive_failures = 0

        response.success = True
        response.message = f'Reset: cancelled any active patrol, cleared {count} waypoint(s).'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    """Initialize rclpy, spin WaypointRecorderNode, and shut down cleanly."""
    rclpy.init(args=args)
    node = WaypointRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
