"""Publish compact Gemini clouds without invalid depth pixels."""
import copy

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


_FIELD_NAMES = ('x', 'y', 'z')


def finite_cloud(message):
    """Drop nonfinite XYZ records while preserving each valid point's raw fields."""
    fields = {field.name: field for field in message.fields}
    if any(name not in fields or fields[name].datatype != PointField.FLOAT32
           for name in _FIELD_NAMES):
        raise ValueError('Point cloud must contain float32 x, y, z fields')
    shape = (message.height, message.width)
    strides = (message.row_step, message.point_step)
    byte_order = '>' if message.is_bigendian else '<'
    valid = np.ones(shape, dtype=bool)
    for name in _FIELD_NAMES:
        coordinate = np.ndarray(shape, buffer=message.data, dtype=byte_order + 'f4',
                                offset=fields[name].offset, strides=strides)
        valid &= np.isfinite(coordinate)
    points = np.ndarray(shape, buffer=message.data, dtype=f'V{message.point_step}',
                        strides=strides)
    packed = points[valid].tobytes()
    result = copy.copy(message)
    result.height = 1
    result.width = int(valid.sum())
    result.row_step = result.width * message.point_step
    result.data = packed
    result.is_dense = True
    return result


class GeminiCloud(Node):
    def __init__(self):
        super().__init__('gemini2_cloud')
        self.rgb_pub = self.create_publisher(
            PointCloud2, '/gemini2/depth_registered/points', 5)
        self.create_subscription(PointCloud2, '/_gemini2/registered_points',
                                 lambda msg: self.rgb_pub.publish(finite_cloud(msg)),
                                 qos_profile_sensor_data)


def main(args=None):
    rclpy.init(args=args)
    node = GeminiCloud()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
