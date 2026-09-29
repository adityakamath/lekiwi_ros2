"""Keep point fields intact while removing only nonfinite 3D samples."""
import math
import struct

import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField

from lekiwi_mujoco.gemini2_cloud import finite_cloud


@pytest.mark.parametrize('colored', [False, True])
@pytest.mark.parametrize('bigendian', [False, True])
def test_compacts_invalid_points_and_preserves_colors(colored, bigendian):
    order = '>' if bigendian else '<'
    stride = 20 if colored else 12
    fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
              for i, name in enumerate(('x', 'y', 'z'))]
    if colored:
        fields.append(PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1))
    records = []
    for x, color in [(1., 0x00FF1020), (float('nan'), 0x00FF2030),
                     (float('inf'), 0x00FF3040), (2., 0x00010203)]:
        raw = struct.pack(order + 'fff', x, 3., 4.)
        if colored:
            raw += bytes(4) + struct.pack(order + 'I', color)
        records.append(raw)
    data = b''.join(records[:2]) + bytes(8) + b''.join(records[2:]) + bytes(8)
    msg = PointCloud2(width=2, height=2, fields=fields, point_step=stride,
                      row_step=2 * stride + 8, data=data, is_bigendian=bigendian,
                      is_dense=False)
    out = finite_cloud(msg)
    assert out.height == 1 and out.width == 2
    assert out.row_step == 2 * stride and out.is_dense
    assert bytes(out.data) == records[0] + records[3]
    assert [(f.name, f.offset) for f in out.fields] == [(f.name, f.offset) for f in fields]
    assert math.isfinite(np.frombuffer(out.data, dtype=order + 'f4')[0])


def test_rejects_missing_xyz():
    with pytest.raises(ValueError, match='x, y, z'):
        finite_cloud(PointCloud2())
