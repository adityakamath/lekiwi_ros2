"""The simulated VL53L5CX: its plugin instance in the generated model and the cloud rotated into tof_frame."""
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

from lekiwi_mujoco.build_mujoco_models import use_native_tof

PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = yaml.safe_load((PACKAGE / 'config/mujoco.yaml').read_text())['tof']


def _model():
    return ('<mujoco><extension><plugin plugin="mujoco.plugin.lidar"><instance name="lidar"/></plugin></extension>'
            '<worldbody><body name="tof_frame"/></worldbody><sensor/></mujoco>')


def test_native_tof_is_an_8x8_instance_on_a_site_looking_along_tof_frame_z():
    root = ET.fromstring(use_native_tof(_model()))
    config = {c.get('key'): c.get('value') for c in root.find('.//instance[@name="tof"]')}
    zone = math.radians(CONFIG['fov_degrees']) / CONFIG['resolution']
    half = float(config['azimuth_range'].split()[1])
    assert config['resolution'] == '8 8'
    assert config['azimuth_range'] == config['elevation_range']
    assert 2 * half == pytest.approx(math.radians(CONFIG['fov_degrees']) - zone, abs=1e-6)  # zone centres, not edges
    assert float(config['max_range']) == CONFIG['max_range'] and float(config['update_rate']) == CONFIG['update_rate']
    site = root.find('.//body[@name="tof_frame"]/site[@name="tof"]')
    assert site.get('pos') == f"0 0 {CONFIG['site_height']}"
    w, x, y, z = map(float, site.get('quat').split())
    # The first column of the rotation matrix is the site's x axis in tof_frame: +z.
    assert (1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)) == pytest.approx((0, 0, 1), abs=1e-6)
    assert root.find('sensor/plugin[@name="tof"]').get('objname') == 'tof'


def test_plugin_params_publish_the_raw_cloud_in_tof_frame_for_the_relay():
    plugins = yaml.safe_load((PACKAGE / 'config/mujoco_ros2_control_plugins.yaml').read_text())
    tof = plugins['/**']['ros__parameters']['mujoco_plugins']['mujoco_3d_lidar_plugin']['tof']
    assert tof == {'frame_name': 'tof_frame', 'topic': '/tof/pointcloud_raw'}


def test_cloud_is_rotated_into_tof_frame_and_keeps_no_hits():
    pytest.importorskip('sensor_msgs.msg')
    from sensor_msgs.msg import PointCloud2, PointField
    from lekiwi_mujoco.tof_pointcloud import to_tof_frame

    fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
    nan = float('nan')
    points = [(1., 0., 0.), (1., 0.5, -0.25), (nan, nan, nan), (2., 0., 0.)]
    data = b''.join(struct.pack('<fff', *p) for p in points)
    message = PointCloud2(width=2, height=2, fields=fields, point_step=12, row_step=24, data=data)
    message.header.frame_id = 'raw'
    out = to_tof_frame(message, 0.011)
    assert out.header.frame_id == 'tof_frame' and (out.height, out.width) == (2, 2)
    result = np.frombuffer(out.data, dtype='<f4').reshape(4, 3)
    # Boresight (site +x) becomes tof_frame +z, offset back by the ray origin's height above tof_frame.
    assert result[0] == pytest.approx((0, 0, 1.011))
    assert result[1] == pytest.approx((0.25, 0.5, 1.011))  # site elevation -> -x, azimuth -> y
    assert np.isnan(result[2]).all()
    assert result[3] == pytest.approx((0, 0, 2.011))
