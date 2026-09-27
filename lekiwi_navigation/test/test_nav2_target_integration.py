"""Real ROS service/action/introspection round trip with a local fake Nav2 server."""
import time

from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default
from service_msgs.msg import ServiceEventInfo
from std_srvs.srv import SetBool, SetBool_Event

from lekiwi_navigation.nav2_target_node import Nav2TargetNode


def test_send_service_action_result_and_audio_events():
    tracker = Nav2TargetNode()
    peer = Node('nav2_target_test_peer')
    executor = SingleThreadedExecutor()
    executor.add_node(tracker)
    executor.add_node(peer)
    received, events = [], []

    def execute(handle):
        received.append(handle.request.pose)
        handle.succeed()
        return NavigateToPose.Result()

    server = ActionServer(peer, NavigateToPose, 'navigate_to_pose', execute)
    client = peer.create_client(SetBool, 'nav2_send_goal')
    subscription = peer.create_subscription(
        SetBool_Event, 'nav2_send_goal/_service_event', events.append,
        qos_profile_services_default)

    def until(predicate):
        deadline = time.monotonic() + 10
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert predicate(), 'Timed out waiting for local ROS communication'

    try:
        until(lambda: client.service_is_ready() and tracker._nav_client.server_is_ready()
              and peer.count_publishers('nav2_send_goal/_service_event') > 0)
        tracker._mode, tracker._pose = True, [1., 2., 0.]
        response = client.call_async(SetBool.Request(data=True))
        until(lambda: response.done() and received and tracker._goal_token is None
              and any(e.info.event_type == ServiceEventInfo.RESPONSE_SENT for e in events))
        assert response.result().success
        assert received[0].header.frame_id == 'map'
        assert received[0].pose.position.x == 1.
        assert received[0].pose.position.y == 2.
        assert received[0].pose.orientation.w == 1.
        requests = [e for e in events if e.info.event_type == ServiceEventInfo.REQUEST_RECEIVED]
        responses = [e for e in events if e.info.event_type == ServiceEventInfo.RESPONSE_SENT]
        assert requests and requests[0].request[0].data
        assert responses[0].response[0].success
        assert requests[0].info.sequence_number == responses[0].info.sequence_number
        assert bytes(requests[0].info.client_gid) == bytes(responses[0].info.client_gid)
    finally:
        peer.destroy_subscription(subscription)
        server.destroy()
        executor.remove_node(tracker)
        executor.remove_node(peer)
        tracker.destroy_node()
        peer.destroy_node()
        executor.shutdown()
