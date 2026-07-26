#!/usr/bin/env python3
"""Spoken status announcements for e-stop, mode switching, waypoint/map services, and Nav2 goals.

Watches ROS 2 service introspection and /navigate_to_pose status, not button presses or the
services' own effects, so announcements fire identically regardless of caller.

For the state services (see STATE_SERVICES below), this also announces current state once on
startup or reconnect - the corresponding server replays its last request+response pair to a
late-joining subscriber, rather than staying silent until the next transition.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
import queue
import subprocess
import threading

from action_msgs.msg import GoalStatus, GoalStatusArray
from ament_index_python.packages import get_package_share_directory
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
]

# Services whose owning node publishes _service_event with transient-local durability (see
# STATE_SERVICE_QOS in sts_hardware_interface, twist_switch_node, waypoint_recorder_node) -
# these represent ongoing state, so subscribing with matching durability replays the last
# request+response pair on startup, and this node announces current state instead of staying
# silent until the next transition. Durability is a compatibility requirement, not just a
# preference: a VOLATILE subscription would never receive the replay even if it connects fine.
# The other three services (record/reset/save) are one-shot actions - replaying a stale call
# there would announce something that didn't just happen, so they stay on the plain default.
STATE_SERVICES = {'/emergency_stop', '/twist_switch', '/waypoint_follow'}
STATE_SERVICE_QOS = QoSProfile(
    depth=2,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    liveliness=LivelinessPolicy.AUTOMATIC,
    liveliness_lease_duration=Duration(seconds=1),
)

DEFAULT_NAV_GOAL_STATUS_TOPIC = '/navigate_to_pose/_action/status'

# Only terminal, non-canceled outcomes get announced - see phrases.yaml's nav_goal comment.
_NAV_GOAL_OUTCOME_KEYS = {
    GoalStatus.STATUS_SUCCEEDED: 'success',
    GoalStatus.STATUS_ABORTED: 'failure',
}


def _phrase_filename(phrase: str) -> str:
    """Hash a phrase to its rendered filename.

    Must match render_phrases.phrase_filename() exactly - this is the only
    link between the two, there's no separate manifest file.
    """
    return hashlib.sha1(phrase.encode('utf-8')).hexdigest()[:12] + '.wav'


class SpeechQueue:
    """Play announcements one at a time from pre-rendered audio files.

    Every distinct phrase was already rendered to sounds/*.wav at build time
    (see render_phrases.py), so this is just an aplay of an existing file,
    never live synthesis. Queue depth capped with drop-oldest: if events ever
    outpace speech, the most recent state is what should be announced, not a
    backlog of stale ones.
    """

    def __init__(self, sounds_dir: Path, speaker_device: str, logger, max_depth: int = 4) -> None:
        """Set the mixer volume and start the worker thread that plays queued phrases."""
        # Not assumed to be handled elsewhere (e.g. by voice_pipeline.py, if
        # that's even running) - this node sets its own audible volume.
        subprocess.run(['amixer', '-c', '0', 'sset', 'PCM', '100%'], capture_output=True)
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
            subprocess.run(
                ['aplay', '-D', self._speaker_device, str(wav_path)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )


class IndicatorNode(Node):
    """Watches service introspection and nav goal status, and speaks the matching phrase."""

    def __init__(self) -> None:
        """Declare parameters, load phrases.yaml, and subscribe to every watched service/topic."""
        super().__init__('indicator_node')

        self.declare_parameter('speaker_device', DEFAULT_SPEAKER_DEVICE)
        self.declare_parameter('services', DEFAULT_SERVICES)
        self.declare_parameter('nav_goal_status_topic', DEFAULT_NAV_GOAL_STATUS_TOPIC)
        speaker_device = self.get_parameter('speaker_device').value
        services = self.get_parameter('services').value
        nav_goal_status_topic = self.get_parameter('nav_goal_status_topic').value

        share_dir = Path(get_package_share_directory('lekiwi_audio'))
        with open(share_dir / 'config' / 'phrases.yaml') as f:
            phrases_data = yaml.safe_load(f)
        self._phrases = phrases_data['services']
        self._nav_goal_phrases = phrases_data.get('nav_goal', {})

        self._speech = SpeechQueue(share_dir / 'sounds', speaker_device, self.get_logger())

        # Per-service correlation cache: (client_gid, sequence_number) -> request value,
        # populated on REQUEST_RECEIVED and consumed on RESPONSE_SENT.
        self._pending: dict[str, dict[tuple, bool]] = {}

        for service in services:
            qos = STATE_SERVICE_QOS if service in STATE_SERVICES else qos_profile_services_default
            self.create_subscription(
                SetBool_Event,
                f'{service}/_service_event',
                self._make_callback(service),
                qos,
            )

        # goal_id (bytes) -> None, oldest-first; tracks which /navigate_to_pose goals have
        # already been announced, since the status topic republishes every tracked goal's
        # current state on every message, not just deltas.
        self._announced_nav_goals: dict[bytes, None] = {}
        if nav_goal_status_topic:
            self.create_subscription(
                GoalStatusArray,
                nav_goal_status_topic,
                self._on_nav_goal_status,
                10,
            )

        self.get_logger().info(f"indicator_node ready, watching: {', '.join(services)}")

    def destroy_node(self) -> bool:
        """Stop the speech worker thread before the node itself is torn down."""
        self._speech.stop()
        return super().destroy_node()

    def _phrase_for(self, service: str, request_value: bool, success: bool) -> str | None:
        """Look up the configured phrase for a service call's request value and outcome."""
        state_phrases = self._phrases.get(service, {}).get('true' if request_value else 'false')
        if state_phrases is None:
            # No entry at all for this request value - e.g. the false/no-op calls on
            # /record_waypoint, /reset_waypoints, /save_map (confirmed in each handler:
            # false changes nothing, it's not a real state). Deliberately silent.
            return None
        if success:
            return state_phrases.get('success')
        # A failure with no phrase configured still gets announced generically,
        # rather than silently dropped just because it wasn't anticipated.
        return state_phrases.get('failure', 'Error')

    def _phrase_for_nav_goal(self, status: int) -> str | None:
        """Look up the configured phrase for a terminal /navigate_to_pose status, if any."""
        outcome = _NAV_GOAL_OUTCOME_KEYS.get(status)
        if outcome is None:
            return None
        return self._nav_goal_phrases.get(outcome)

    def _on_nav_goal_status(self, msg: GoalStatusArray) -> None:
        """Announce each ad-hoc /navigate_to_pose goal's outcome once.

        Every goal here is a genuine ad-hoc goal, never a waypoint-patrol leg: the patrol drives
        through /follow_waypoints instead, and waypoint_recorder_node's own external-detour
        detection depends on /navigate_to_pose staying idle during a normal patrol leg.
        """
        for status in msg.status_list:
            phrase = self._phrase_for_nav_goal(status.status)
            if phrase is None:
                continue
            goal_id = bytes(status.goal_info.goal_id.uuid)
            if goal_id in self._announced_nav_goals:
                continue
            self._announced_nav_goals[goal_id] = None
            if len(self._announced_nav_goals) > 16:  # defensive cap, shouldn't normally fill
                self._announced_nav_goals.pop(next(iter(self._announced_nav_goals)), None)
            self.get_logger().info(f'nav goal status={status.status} -> {phrase!r}')
            self._speech.speak(phrase)

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
