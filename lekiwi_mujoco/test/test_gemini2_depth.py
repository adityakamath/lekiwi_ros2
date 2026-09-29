"""Gemini's registered depth uses millimeters and invalid pixels are zero."""
import numpy as np
import pytest
from sensor_msgs.msg import Image

from lekiwi_mujoco.gemini2_depth import depth_millimeters


@pytest.mark.parametrize('bigendian', [False, True])
def test_depth_range_units_and_padded_rows(bigendian):
    pixels = np.array([[.149, .15, 1.234, 99], [10., 10.01, np.nan, 99],
                       [np.inf, -1., 0., 99]], dtype='>f4' if bigendian else '<f4')
    msg = Image(width=3, height=3, step=16, encoding='32FC1',
                is_bigendian=int(bigendian), data=pixels.tobytes())
    converted = depth_millimeters(msg)
    assert converted.dtype == np.dtype('<u2')
    assert converted.tolist() == [[0, 150, 1234], [10000, 0, 0], [0, 0, 0]]


def test_unexpected_depth_encoding_is_rejected():
    with pytest.raises(ValueError, match='32FC1'):
        depth_millimeters(Image(encoding='16UC1'))
