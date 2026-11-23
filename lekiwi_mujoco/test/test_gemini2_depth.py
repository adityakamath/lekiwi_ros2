"""Gemini's registered depth uses millimeters and invalid pixels are zero; the color image is JPEG-compressed."""
import cv2
import numpy as np
import pytest

pytest.importorskip('sensor_msgs.msg')
from sensor_msgs.msg import Image

from lekiwi_mujoco.gemini2_depth import depth_millimeters, jpeg_compressed


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


@pytest.mark.parametrize('encoding', ['rgb8', 'bgr8'])
def test_color_is_jpeg_compressed_with_its_colors_and_padded_rows(encoding):
    red = (255, 0, 0) if encoding == 'rgb8' else (0, 0, 255)
    pixels = np.zeros((16, 16, 3), np.uint8)
    pixels[:] = red
    rows = np.zeros((16, 16 * 3 + 8), np.uint8)  # rows padded to 56 bytes
    rows[:, :48] = pixels.reshape(16, 48)
    msg = Image(width=16, height=16, step=56, encoding=encoding, data=rows.tobytes())
    msg.header.frame_id = 'robot1/gemini2_color_optical_frame'
    out = jpeg_compressed(msg)
    assert (out.format, out.header.frame_id) == ('jpeg', 'robot1/gemini2_color_optical_frame')
    decoded = cv2.imdecode(np.frombuffer(bytes(out.data), np.uint8), cv2.IMREAD_COLOR)[..., ::-1]
    assert decoded.shape == (16, 16, 3)
    assert np.abs(decoded.astype(int) - (255, 0, 0)).max() < 12  # JPEG is lossy, but the color is red


def test_unexpected_color_encoding_is_rejected():
    with pytest.raises(ValueError, match='rgb8 or bgr8'):
        jpeg_compressed(Image(encoding='mono8'))
