#!/usr/bin/env python3
"""
map_saver_node — save the slam_toolbox map on joystick button press.

Subscribes to /joy and on R1 button (button 5) press:
  1. Creates maps/<YYYY_MM_DD_HH_MM_SS>/ in the source tree
  2. Calls /slam_toolbox/save_map      → writes map.pgm + map.yaml
  3. Calls /slam_toolbox/serialize_map → writes map.posegraph + map.data
                                         (needed for slam_mode:=localization)
  4. Looks up map→base_footprint TF    → writes starting_pose.yaml
                                         (used by nav2.launch.py for map_start_pose)

Parameters:
    button  int  Button index for save trigger (default: 5 = R1)
"""

import math
import os
import yaml
from datetime import datetime

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Joy
from slam_toolbox.srv import SaveMap, SerializePoseGraph
import tf2_ros


class MapSaverNode(Node):

    def __init__(self):
        super().__init__('map_saver_node')

        # os.path.realpath resolves the symlink created by --symlink-install
        # so _maps_dir always points to the source tree:
        #   src/lekiwi_ros2/lekiwi_navigation/maps/
        _src_pkg = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        self._maps_dir = os.path.join(_src_pkg, 'maps')
        os.makedirs(self._maps_dir, exist_ok=True)

        self.declare_parameter('button', 5)
        self._button = self.get_parameter('button').value

        self._save_client      = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self._serialize_client = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._last_button_state = False
        self._saving = False

        self._joy_sub = self.create_subscription(Joy, '/joy', self._joy_callback, 10)
        self.get_logger().info(
            f'map_saver_node ready — press button {self._button} (R1) to save map to {self._maps_dir!r}'
        )

    def _joy_callback(self, msg: Joy):
        if self._button >= len(msg.buttons):
            return

        pressed = bool(msg.buttons[self._button])
        # Rising edge only
        if pressed and not self._last_button_state and not self._saving:
            self._trigger_save()
        self._last_button_state = pressed

    def _trigger_save(self):
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

        save_req = SaveMap.Request()
        save_req.name.data = self._map_name

        future = self._save_client.call_async(save_req)
        future.add_done_callback(self._on_save_done)

    def _on_save_done(self, future):
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
            self._saving = False

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
