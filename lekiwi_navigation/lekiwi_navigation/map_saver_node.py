#!/usr/bin/env python3
"""
map_saver_node — save the slam_toolbox map on request.

One std_srvs/srv/SetBool service, usable from joy_teleop button bindings or Foxglove's
"Call Service" panel:

  /save_map
      true  - save the map (slam_toolbox save_map + serialize_map) into a timestamped
              directory under maps/, plus the robot's current map-frame pose for
              localization startup. Rejected if a save is already in progress.
      false - no-op

Parameters:
    save_timeout  float  Seconds before a stuck save/serialize chain is forcibly cleared,
                          so /save_map doesn't refuse every later call (default: 15.0)
"""

import math
import os
import yaml
from datetime import datetime

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_srvs.srv import SetBool
import tf2_ros


class MapSaverNode(Node):
    """Saves the slam_toolbox map and starting pose on request."""

    def __init__(self):
        """Resolve the maps directory and create the /save_map service."""
        super().__init__('map_saver_node')

        # realpath resolves the --symlink-install symlink, so this always points to the
        # source tree (src/lekiwi_ros2/lekiwi_navigation/maps/), not the install directory.
        _src_pkg = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        self._maps_dir = os.path.join(_src_pkg, 'maps')
        os.makedirs(self._maps_dir, exist_ok=True)

        self.declare_parameter('save_timeout', 15.0)
        self._save_timeout = self.get_parameter('save_timeout').value

        self._save_client      = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self._serialize_client = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._saving = False
        self._save_watchdog = None

        self.create_service(SetBool, '/save_map', self._handle_save)
        self.get_logger().info(f'map_saver_node ready: /save_map -> {self._maps_dir!r}')

    def _handle_save(self, request: SetBool.Request, response: SetBool.Response):
        """Start a map save, rejecting if one is already in progress."""
        if not request.data:
            response.success = True
            response.message = 'No-op (set data: true to save).'
            return response

        if self._saving:
            response.success = False
            response.message = (
                f'Save already in progress (clears automatically after '
                f'{self._save_timeout:.0f}s if stuck).'
            )
            return response

        self._trigger_save()
        response.success = True
        response.message = f'Save started: {self._map_name!r}'
        self.get_logger().info(response.message)
        return response

    def _trigger_save(self):
        """Create the timestamped output directory and call /slam_toolbox/save_map."""
        self._saving = True
        timestamp = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        save_dir = os.path.join(self._maps_dir, timestamp)
        os.makedirs(save_dir, exist_ok=True)
        self._map_name = os.path.join(save_dir, 'map')
        self.get_logger().info(f'Saving map to {self._map_name!r} …')

        if not self._save_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('/slam_toolbox/save_map service not available')
            self._saving = False
            return
        if not self._serialize_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('/slam_toolbox/serialize_map service not available')
            self._saving = False
            return

        self._save_watchdog = self.create_timer(self._save_timeout, self._on_save_watchdog)

        save_req = SaveMap.Request()
        save_req.name.data = self._map_name

        future = self._save_client.call_async(save_req)
        future.add_done_callback(self._on_save_done)

    def _on_save_watchdog(self):
        """Force-clear a save/serialize chain that never finished (e.g. a slam_toolbox
        lifecycle hiccup dropping the response), so /save_map isn't stuck refusing forever."""
        self._clear_saving()
        self.get_logger().error(
            f'Save watchdog fired after {self._save_timeout:.0f}s - the chain never '
            'completed. Cleared the in-progress flag; check whether map/pose-graph files '
            'were actually written before retrying.'
        )

    def _clear_saving(self):
        """Cancel the watchdog if still pending and clear the in-progress flag."""
        if self._save_watchdog is not None:
            self._save_watchdog.cancel()
            self._save_watchdog = None
        self._saving = False

    def _on_save_done(self, future):
        """Log the save_map result, then serialize."""
        try:
            result = future.result()
            if result.result == 0:
                self.get_logger().info(f'Map saved: {self._map_name}.pgm / .yaml')
            else:
                self.get_logger().error(f'save_map returned error code {result.result}')
        except Exception as e:
            self.get_logger().error(f'save_map call failed: {e}')

        # Now serialize the pose graph for use with slam_mode:=localization
        ser_req = SerializePoseGraph.Request()
        ser_req.filename = self._map_name

        future2 = self._serialize_client.call_async(ser_req)
        future2.add_done_callback(self._on_serialize_done)

    def _on_serialize_done(self, future):
        """Log the serialize_map result, then save the starting pose."""
        try:
            result = future.result()
            if result.result == 0:
                self.get_logger().info(f'Pose graph serialized: {self._map_name}.posegraph / .data')
            else:
                self.get_logger().error(f'serialize_map returned error code {result.result}')
        except Exception as e:
            self.get_logger().error(f'serialize_map call failed: {e}')
        finally:
            self._save_starting_pose()
            self._clear_saving()

    def _save_starting_pose(self):
        """Look up map→base_footprint and write starting_pose.yaml next to the map files."""
        save_dir = os.path.dirname(self._map_name)
        try:
            tf_stamped = self._tf_buffer.lookup_transform(
                'map', 'base_footprint', Time(), Duration(seconds=0.5)
            )
            t = tf_stamped.transform.translation
            q = tf_stamped.transform.rotation
            theta = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            pose_file = os.path.join(save_dir, 'starting_pose.yaml')
            with open(pose_file, 'w') as f:
                yaml.dump({'x': float(t.x), 'y': float(t.y), 'theta': float(theta)}, f)
            self.get_logger().info(
                f'Starting pose saved: x={t.x:.3f} y={t.y:.3f} theta={theta:.3f}'
            )
        except Exception as e:
            self.get_logger().warn(
                f'Could not save starting pose (localization will start at map origin): {e}'
            )


def main(args=None):
    """Initialize rclpy, spin MapSaverNode, and shut down cleanly."""
    rclpy.init(args=args)
    node = MapSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
