"""Vectorized PointCloud2 parsing, including organized clouds with row padding."""
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


def read_xyzi(message):
    fields = {field.name: field for field in message.fields}
    if not all(name in fields for name in ('x', 'y', 'z')):
        raise ValueError('PointCloud2 must contain x, y, z')
    types = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
    endian = '>' if message.is_bigendian else '<'
    names = [name for name in ('x', 'y', 'z', 'intensity') if name in fields]
    dtype = np.dtype({'names': names,
                      'formats': [endian + types[fields[n].datatype] for n in names],
                      'offsets': [fields[n].offset for n in names],
                      'itemsize': message.point_step})
    if not message.width or not message.height:
        return np.empty((0, 4), dtype=np.float32)
    cloud = np.ndarray((message.height, message.width), dtype=dtype,
                       buffer=message.data, strides=(message.row_step, message.point_step))
    out = np.zeros((message.height*message.width, 4), dtype=np.float32)
    for i, name in enumerate(names):
        out[:, i] = cloud[name].ravel()
    return out[np.isfinite(out).all(axis=1)]


def make_cloud(points, header):
    data = np.ascontiguousarray(points, dtype='<f4').reshape((-1, 4))
    msg = PointCloud2()
    msg.header = header
    msg.height, msg.width = 1, len(data)
    msg.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z', 'intensity'))]
    msg.is_bigendian = False
    msg.point_step, msg.row_step = 16, 16*len(data)
    msg.is_dense = True
    msg.data = data.tobytes()
    return msg


def flatten(points, sensor_world_position, plane_z=0.1):
    """Input axes are world-aligned, but origin is the physical LiDAR origin."""
    out = points.copy()
    out[:, :3] += np.asarray(sensor_world_position)
    out[:, 2] = plane_z
    return out
