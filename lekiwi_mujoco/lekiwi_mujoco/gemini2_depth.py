"""Convert ideal rendered depth to the Orbbec registered-depth wire format, and JPEG-compress the color image."""
import copy

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


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


def jpeg_compressed(msg, quality=95):
    """Encode an rgb8 or bgr8 color image as a JPEG CompressedImage, as image_transport's compressed plugin does."""
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'Expected rgb8 or bgr8 color, got {msg.encoding}')
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    pixels = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    bgr = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else pixels
    ok, jpeg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise ValueError('JPEG encoding failed')
    out = CompressedImage()
    out.header = msg.header
    out.format = 'jpeg'
    out.data = jpeg.tobytes()
    return out


class GeminiDepth(Node):
    def __init__(self):
        super().__init__('gemini2_depth')
        self.depth_pub = self.create_publisher(Image, 'gemini2/depth/image_raw', 1)
        self.info_pub = self.create_publisher(CameraInfo, 'gemini2/depth/camera_info', 1)
        self.create_subscription(Image, '_gemini2/depth_raw', self.on_depth,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, 'gemini2/color/camera_info', self.on_info,
                                 qos_profile_sensor_data)
        self.color_pub = self.create_publisher(CompressedImage, 'gemini2/color/image_raw/compressed', qos_profile_sensor_data)
        self.create_subscription(Image, 'gemini2/color/image_raw', self.on_color, qos_profile_sensor_data)

    def on_info(self, msg):
        # D2C registration uses exactly the simulated color intrinsics and frame.
        self.info_pub.publish(msg)

    def on_color(self, msg):
        if self.color_pub.get_subscription_count():  # Encode only while someone is listening.
            self.color_pub.publish(jpeg_compressed(msg))

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
