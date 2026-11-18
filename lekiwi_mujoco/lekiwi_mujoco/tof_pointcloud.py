"""Rotate the simulated ToF cloud from the lidar plugin's site axes into tof_frame."""
import copy

import numpy as np
import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

from lekiwi_mujoco.paths import package_share

_FIELD_NAMES = ('x', 'y', 'z')


def to_tof_frame(message, site_height, frame_id='tof_frame'):
    """Map site axes (x forward) to tof_frame (z forward): (x, y, z) -> (-z, y, x + site_height); NaN no-hits are kept."""
    fields = {field.name: field for field in message.fields}
    if any(name not in fields or fields[name].datatype != PointField.FLOAT32 for name in _FIELD_NAMES):
        raise ValueError('Point cloud must contain float32 x, y, z fields')
    byte_order = '>' if message.is_bigendian else '<'
    shape = (message.height, message.width)
    strides = (message.row_step, message.point_step)
    x, y, z = (np.ndarray(shape, buffer=message.data, dtype=byte_order + 'f4', offset=fields[name].offset,
                          strides=strides) for name in _FIELD_NAMES)
    points = np.stack((-z, y, x + site_height), axis=-1).astype('<f4')
    result = copy.copy(message)
    result.header = copy.copy(message.header)
    result.header.frame_id = frame_id
    result.fields = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                     for i, name in enumerate(_FIELD_NAMES)]
    result.is_bigendian = False
    result.point_step = 12
    result.row_step = 12 * message.width
    result.data = points.tobytes()
    return result


class TofPointCloud(Node):
    def __init__(self):
        super().__init__('tof_pointcloud')
        site_height = yaml.safe_load((package_share('lekiwi_mujoco') / 'config/mujoco.yaml').read_text())['tof']['site_height']
        publisher = self.create_publisher(PointCloud2, '/tof/pointcloud', qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/tof/pointcloud_raw',
                                 lambda msg: publisher.publish(to_tof_frame(msg, site_height)), qos_profile_sensor_data)


def main(args=None):
    rclpy.init(args=args)
    node = TofPointCloud()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
