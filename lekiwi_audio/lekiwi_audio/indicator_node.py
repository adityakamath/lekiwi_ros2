#!/usr/bin/env python3
"""Generic spoken-announcement node: watches ROS 2 introspection/status topics named entirely
via parameters, and speaks the phrase configured for each outcome in phrases.yaml. No service
or action name is hard-coded into the node itself.

- `services` (+ `state_services` for late-joiner state replay): SetBool services, watched via
  their `<service>/_service_event` introspection topic.
- `goal_status_watchers`: action `_action/status` (GoalStatusArray) topics, each named and
  configured via a `<name>.topic` parameter plus a phrases.yaml `goal_status.<name>` entry.

Ships configured by default for e-stop, mode switching, waypoint/map services, ad-hoc Nav2
goals, and battery events - see the DEFAULT_* constants below.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
import queue
import subprocess
import threading

from action_msgs.msg import GoalStatus, GoalStatusArray
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    LivelinessPolicy,
    qos_profile_services_default,
    QoSProfile,
    ReliabilityPolicy,
)
from service_msgs.msg import ServiceEventInfo
from std_srvs.srv import SetBool_Event
import yaml

DEFAULT_SPEAKER_DEVICE = 'plughw:CARD=C16K6Ch,DEV=0'

DEFAULT_SERVICES = [
    '/emergency_stop',
    '/twist_switch',
    '/record_waypoint',
    '/reset_waypoints',
    '/waypoint_follow',
    '/save_map',
    '/battery_low',
    '/battery_critical',
    '/battery_full',
]

# Transient-local: late joiners get the last call replayed instead of silence.
DEFAULT_STATE_SERVICES = [
    '/emergency_stop', '/twist_switch', '/waypoint_follow',
    '/battery_low', '/battery_critical', '/battery_full',
]
STATE_SERVICE_QOS = QoSProfile(
    depth=2,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    liveliness=LivelinessPolicy.AUTOMATIC,
    liveliness_lease_duration=Duration(seconds=1),
)

# Each name here gets a <name>.topic parameter (default below if listed, else required).
# Add another action's status topic by extending this list - no code change needed.
DEFAULT_GOAL_STATUS_WATCHERS = ['nav_goal']
DEFAULT_GOAL_STATUS_TOPICS = {'nav_goal': '/navigate_to_pose/_action/status'}

# Fixed by the action_msgs/GoalStatus wire format, not deployment-specific.
_GOAL_OUTCOME_KEYS = {
    GoalStatus.STATUS_SUCCEEDED: 'success',
    GoalStatus.STATUS_ABORTED: 'failure',
    GoalStatus.STATUS_CANCELED: 'canceled',
}

# CANCELED alongside another still-active goal means the action server preempted it, not a
# deliberate stop - see GoalStatusWatcher.resolve.
_GOAL_ACTIVE_STATUSES = (
    GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING,
)

_READ_ONLY = ParameterDescriptor(read_only=True)  # every parameter is consumed once at startup


def _phrase_filename(phrase: str) -> str:
    """Hash a phrase to its rendered filename - must match render_phrases.phrase_filename()."""
    return hashlib.sha1(phrase.encode('utf-8')).hexdigest()[:12] + '.wav'


def _has_other_active_goal(status_list, exclude_goal_id: bytes) -> bool:
    """True if some goal besides exclude_goal_id in status_list is still in flight."""
    return any(
        s.status in _GOAL_ACTIVE_STATUSES and bytes(s.goal_info.goal_id.uuid) != exclude_goal_id
        for s in status_list
    )


class GoalStatusWatcher:
    """Resolves one action's GoalStatusArray updates to phrases, announcing each goal once."""

    def __init__(self, name: str, phrases: dict[str, str]) -> None:
        """Store the watcher's name (for logging) and its outcome -> phrase map."""
        self.name = name
        self._phrases = phrases
        self._announced: dict[bytes, None] = {}  # goal_id -> None, oldest-first, capped at 16

    def resolve(self, msg: GoalStatusArray) -> list[str]:
        """Return phrases for goals in msg that just reached an announceable outcome."""
        phrases = []
        for status in msg.status_list:
            goal_id = bytes(status.goal_info.goal_id.uuid)
            if goal_id in self._announced:
                continue
            if (status.status == GoalStatus.STATUS_CANCELED
                    and _has_other_active_goal(msg.status_list, goal_id)):
                self._mark(goal_id)  # superseded by a newer goal - its outcome speaks instead
                continue
            outcome = _GOAL_OUTCOME_KEYS.get(status.status)
            phrase = None if outcome is None else self._phrases.get(outcome)
            if phrase is None:
                continue
            self._mark(goal_id)
            phrases.append(phrase)
        return phrases

    def _mark(self, goal_id: bytes) -> None:
        """Record goal_id as handled so a later republish of the same status is a no-op."""
        self._announced[goal_id] = None
        if len(self._announced) > 16:  # defensive cap, shouldn't normally fill
            self._announced.pop(next(iter(self._announced)), None)


class SpeechQueue:
    """Play announcements one at a time from pre-rendered audio files (see render_phrases.py).

    Queue depth capped with drop-oldest - the most recent state matters, not a stale backlog.
    """

    def __init__(self, sounds_dir: Path, speaker_device: str, logger, max_depth: int = 4) -> None:
        """Set the mixer volume and start the worker thread that plays queued phrases."""
        # Sets its own volume rather than assuming voice_pipeline.py already did; missing
        # amixer (CI, no audio hardware) shouldn't prevent startup - playback just no-ops.
        try:
            subprocess.run(['amixer', '-c', '0', 'sset', 'PCM', '100%'], capture_output=True)
        except FileNotFoundError:
            logger.warning('amixer not found - skipping mixer volume setup.')
        self._sounds_dir = sounds_dir
        self._speaker_device = speaker_device
        self._logger = logger
        self._queue: queue.Queue[str | None] = queue.Queue(maxsize=max_depth)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def speak(self, text: str) -> None:
        """Queue a phrase for playback, dropping the oldest pending one if the queue is full."""
        try:
            self._queue.put_nowait(text)
        except queue.Full:
            try:
                self._queue.get_nowait()  # drop oldest
            except queue.Empty:
                pass
            self._queue.put_nowait(text)

    def stop(self) -> None:
        """Signal the worker thread to exit and wait briefly for it."""
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            pass
        self._thread.join(timeout=1.0)

    def _run(self) -> None:
        """Play queued phrases one at a time until stop() signals this thread to exit."""
        while True:
            text = self._queue.get()
            if text is None:
                return
            wav_path = self._sounds_dir / _phrase_filename(text)
            if not wav_path.exists():
                # phrases.yaml was edited but the package wasn't rebuilt since -
                # skip rather than crash the node over a stale/missing asset.
                self._logger.warning(
                    f'no rendered audio for {text!r} ({wav_path.name}) - rebuild the package.'
                )
                continue
            try:
                subprocess.run(
                    ['aplay', '-D', self._speaker_device, str(wav_path)],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except FileNotFoundError:
                self._logger.warning('aplay not found - cannot play audio.')


class IndicatorNode(Node):
    """Watches service introspection and configured goal-status topics, speaking phrases.yaml."""

    def __init__(self) -> None:
        """Declare parameters, load phrases.yaml, and subscribe to every watched service/topic."""
        super().__init__('indicator_node')

        self.declare_parameter('speaker_device', DEFAULT_SPEAKER_DEVICE, _READ_ONLY)
        self.declare_parameter('services', DEFAULT_SERVICES, _READ_ONLY)
        self.declare_parameter('state_services', DEFAULT_STATE_SERVICES, _READ_ONLY)
        self.declare_parameter('goal_status_watchers', DEFAULT_GOAL_STATUS_WATCHERS, _READ_ONLY)
        speaker_device = self.get_parameter('speaker_device').value
        services = self.get_parameter('services').value
        state_services = set(self.get_parameter('state_services').value)
        watcher_names = self.get_parameter('goal_status_watchers').value

        share_dir = Path(get_package_share_directory('lekiwi_audio'))
        with open(share_dir / 'config' / 'phrases.yaml') as f:
            phrases_data = yaml.safe_load(f)
        self._phrases = phrases_data['services']
        self._goal_status_phrases = phrases_data.get('goal_status', {})

        self._speech = SpeechQueue(share_dir / 'sounds', speaker_device, self.get_logger())

        # Per-service correlation cache: (client_gid, sequence_number) -> request value,
        # populated on REQUEST_RECEIVED and consumed on RESPONSE_SENT.
        self._pending: dict[str, dict[tuple, bool]] = {}

        for service in services:
            qos = STATE_SERVICE_QOS if service in state_services else qos_profile_services_default
            self.create_subscription(
                SetBool_Event,
                f'{service}/_service_event',
                self._make_callback(service),
                qos,
            )

        self._goal_watchers = self._build_goal_watchers(watcher_names)

        self.get_logger().info(
            f"indicator_node ready, watching services: {', '.join(services)}; "
            f"goal status: {', '.join(self._goal_watchers) or 'none'}"
        )

    def _build_goal_watchers(self, watcher_names: list[str]) -> dict[str, GoalStatusWatcher]:
        """Declare each watcher's <name>.topic parameter and subscribe it, skipping unset ones."""
        watchers = {}
        for name in watcher_names:
            default_topic = DEFAULT_GOAL_STATUS_TOPICS.get(name, '')
            topic = self.declare_parameter(f'{name}.topic', default_topic, _READ_ONLY).value
            if not topic:
                self.get_logger().warning(f'{name}.topic not set - skipping this watcher.')
                continue
            watcher = GoalStatusWatcher(name, self._goal_status_phrases.get(name, {}))
            watchers[name] = watcher
            self.create_subscription(
                GoalStatusArray, topic, self._make_goal_status_callback(watcher), 10)
        return watchers

    def destroy_node(self) -> bool:
        """Stop the speech worker thread before the node itself is torn down."""
        self._speech.stop()
        return super().destroy_node()

    def _phrase_for(self, service: str, request_value: bool, success: bool) -> str | None:
        """Look up the configured phrase for a service call's request value and outcome."""
        state_phrases = self._phrases.get(service, {}).get('true' if request_value else 'false')
        if state_phrases is None:
            return None  # no entry for this request value - e.g. a deliberate false/no-op
        if success:
            return state_phrases.get('success')
        return state_phrases.get('failure', 'Error')  # unconfigured failure still announces

    def _make_goal_status_callback(self, watcher: GoalStatusWatcher):
        """Build a GoalStatusArray callback that speaks every phrase watcher.resolve() returns."""
        def callback(msg: GoalStatusArray) -> None:
            for phrase in watcher.resolve(msg):
                self.get_logger().info(f'{watcher.name}: -> {phrase!r}')
                self._speech.speak(phrase)
        return callback

    def _make_callback(self, service: str):
        """Build a _service_event callback that correlates request/response pairs for service."""
        pending = self._pending.setdefault(service, {})

        def callback(msg: SetBool_Event) -> None:
            """Cache the request on REQUEST_RECEIVED, then speak on the matching RESPONSE_SENT."""
            key = (bytes(msg.info.client_gid), msg.info.sequence_number)

            if msg.info.event_type == ServiceEventInfo.REQUEST_RECEIVED:
                if msg.request:
                    pending[key] = msg.request[0].data
                if len(pending) > 16:  # defensive cap, shouldn't normally fill
                    pending.pop(next(iter(pending)), None)
                return

            if msg.info.event_type == ServiceEventInfo.RESPONSE_SENT:
                if key not in pending or not msg.response:
                    return
                request_value = pending.pop(key)
                success = msg.response[0].success
                phrase = self._phrase_for(service, request_value, success)
                if phrase:
                    self.get_logger().info(
                        f'{service}: data={request_value} success={success} -> {phrase!r}'
                    )
                    self._speech.speak(phrase)

        return callback


def main(args=None):
    """Initialize rclpy, spin IndicatorNode, and shut down cleanly."""
    rclpy.init(args=args)
    node = IndicatorNode()
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
