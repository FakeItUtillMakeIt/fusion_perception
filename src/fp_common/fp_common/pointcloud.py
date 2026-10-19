import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

_FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]


def read_xyz(msg):
    # handles both unorganized (height==1) and organized (height>1, e.g. gz
    # gpu_lidar 360x16) clouds: points are row-major, point_step apart.
    fields = {f.name: f for f in msg.fields}
    for name in ('x', 'y', 'z'):
        if fields[name].datatype != PointField.FLOAT32:
            raise ValueError(f'field {name} not FLOAT32')
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    n = msg.width * msg.height
    step = msg.point_step
    out = np.empty((n, 3), dtype=np.float32)
    for i, name in enumerate(('x', 'y', 'z')):
        off = fields[name].offset
        view = np.lib.stride_tricks.as_strided(
            buf[off:], shape=(n, 4), strides=(step, 1), writeable=False)
        out[:, i] = view.view(np.float32)[:, 0]
    out = out[np.isfinite(out).all(axis=1)]
    return np.ascontiguousarray(out)


def read_field_names(msg):
    return [f.name for f in msg.fields]


def make_cloud(header, xyz, extra=None):
    fields = list(_FIELDS_XYZ)
    n = xyz.shape[0]
    dt = [('x', 'f4'), ('y', 'f4'), ('z', 'f4')]
    if extra:
        for name, dtype_str, arr in extra:
            fields.append(PointField(name=name, offset=0, datatype=dtype_str, count=1))
            dt.append((name, _NP_DTYPES[dtype_str]))
    structured = np.zeros(n, dtype=np.dtype(dt, align=False))
    structured['x'] = xyz[:, 0]
    structured['y'] = xyz[:, 1]
    structured['z'] = xyz[:, 2]
    if extra:
        for name, _, arr in extra:
            structured[name] = arr
    if extra:
        off = 12
        for f in fields[3:]:
            f.offset = off
            off += np.dtype(_NP_DTYPES[f.datatype]).itemsize
    return PointCloud2(
        header=header,
        height=1,
        width=n,
        is_dense=True,
        fields=fields,
        point_step=structured.dtype.itemsize,
        row_step=structured.dtype.itemsize * n,
        data=structured.tobytes(),
    )


_NP_DTYPES = {
    PointField.INT8: 'i1', PointField.UINT8: 'u1',
    PointField.INT16: 'i2', PointField.UINT16: 'u2',
    PointField.INT32: 'i4', PointField.UINT32: 'u4',
    PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8',
}


def pack_rgb(r, g, b):
    rgb = (r.astype(np.uint32) << 16) | (g.astype(np.uint32) << 8) | b.astype(np.uint32)
    return np.ascontiguousarray(rgb, dtype=np.uint32).view(np.float32)


def read_xyz_label(msg):
    """read x,y,z (float32) + label (uint8) fields; returns (N,3) f32, (N,) u8"""
    fields = {f.name: f for f in msg.fields}
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    n = msg.width * msg.height
    step = msg.point_step
    xyz = np.empty((n, 3), dtype=np.float32)
    for i, name in enumerate(('x', 'y', 'z')):
        off = fields[name].offset
        view = np.lib.stride_tricks.as_strided(
            buf[off:], shape=(n, 4), strides=(step, 1), writeable=False)
        xyz[:, i] = view.view(np.float32)[:, 0]
    lab = np.lib.stride_tricks.as_strided(
        buf[fields['label'].offset:], shape=(n, 1), strides=(step, 1), writeable=False)[:, 0].copy()
    keep = np.isfinite(xyz).all(axis=1)
    return xyz[keep], lab[keep]
