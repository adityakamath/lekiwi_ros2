"""Convert ideal rendered depth to the Orbbec registered-depth wire format."""
import copy

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def depth_millimeters(msg):
    """Range-gate 32FC1 meters to 16UC1 millimeters, with zero for invalid pixels."""
    if msg.encoding != '32FC1':
        raise ValueError(f'Expected rendered 32FC1 depth, got {msg.encoding}')
    depth = np.ndarray((msg.height, msg.width), buffer=msg.data,
                       dtype='>f4' if msg.is_bigendian else '<f4', strides=(msg.step, 4))
    valid = np.isfinite(depth) & (depth >= .15) & (depth <= 10.)
    result = np.zeros(depth.shape, dtype='<u2')
    result[valid] = np.rint(depth[valid] * 1000).astype('<u2')
    return result


class GeminiDepth(Node):
    def __init__(self):
        super().__init__('gemini2_depth')
        self.depth_pub = self.create_publisher(Image, '/gemini2/depth/image_raw', 1)
        self.info_pub = self.create_publisher(CameraInfo, '/gemini2/depth/camera_info', 1)
        self.create_subscription(Image, '/_gemini2/depth_raw', self.on_depth,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/gemini2/color/camera_info', self.on_info,
                                 qos_profile_sensor_data)

    def on_info(self, msg):
        # D2C registration uses exactly the simulated color intrinsics and frame.
        self.info_pub.publish(msg)

    def on_depth(self, msg):
        out = copy.copy(msg)
        out.encoding = '16UC1'
        out.is_bigendian = False
        out.step = msg.width * 2
        out.data = depth_millimeters(msg).tobytes()
        self.depth_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GeminiDepth()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
