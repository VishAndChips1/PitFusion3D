"""Zero-copy PointCloud2 <-> numpy helpers.

sensor_msgs_py.point_cloud2 builds an intermediate structured array and
re-packs it field by field, which dominates per-callback cost on an ARM
CPU. These helpers map msg.data directly with a numpy dtype carrying the
message's real field offsets, so reading is a view rather than a copy.
"""

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


_DATATYPES = {
    PointField.INT8: np.int8,
    PointField.UINT8: np.uint8,
    PointField.INT16: np.int16,
    PointField.UINT16: np.uint16,
    PointField.INT32: np.int32,
    PointField.UINT32: np.uint32,
    PointField.FLOAT32: np.float32,
    PointField.FLOAT64: np.float64,
}

XYZI_FIELDS = ('x', 'y', 'z', 'intensity')
XYZIRT_FIELDS = ('x', 'y', 'z', 'intensity', 'rgb', 'thermal')


def structured_view(msg):
    """View msg.data as a structured array honouring the real field offsets.

    Zero-copy: padded layouts (e.g. gz-sim's 32-byte stride) are described
    by the dtype's itemsize rather than being repacked.
    """
    names, formats, offsets = [], [], []
    for field in msg.fields:
        if field.count != 1 or field.datatype not in _DATATYPES:
            continue
        if field.name in names:
            continue
        names.append(field.name)
        formats.append(_DATATYPES[field.datatype])
        offsets.append(field.offset)
    dtype = np.dtype({
        'names': names,
        'formats': formats,
        'offsets': offsets,
        'itemsize': msg.point_step,
    })
    count = msg.width * msg.height
    return np.frombuffer(msg.data, dtype=dtype, count=count)


def read_fields(msg, field_names, dtype=np.float32):
    """Return an (N, len(field_names)) array of the requested fields.

    When the fields are consecutive float32 values (the layout every cloud
    in this package uses) the whole block is taken as one strided slice,
    which costs a single bulk copy instead of one gather per field.
    """
    count = msg.width * msg.height
    offsets = {f.name: f for f in msg.fields}
    fast = (
        dtype == np.float32
        and msg.point_step % 4 == 0
        and not msg.is_bigendian
        and all(
            name in offsets
            and offsets[name].datatype == PointField.FLOAT32
            and offsets[name].count == 1
            for name in field_names
        )
    )
    if fast:
        first = offsets[field_names[0]].offset
        contiguous = all(
            offsets[name].offset == first + 4 * i for i, name in enumerate(field_names)
        )
        if contiguous and first % 4 == 0:
            block = np.frombuffer(msg.data, dtype=np.float32, count=count * (msg.point_step // 4))
            block = block.reshape(count, msg.point_step // 4)
            start = first // 4
            # Returned as a view: forcing contiguity here costs a full copy
            # that most callers never need.
            return block[:, start:start + len(field_names)]

    view = structured_view(msg)
    out = np.empty((view.shape[0], len(field_names)), dtype=dtype)
    for i, name in enumerate(field_names):
        out[:, i] = view[name]
    return out


def make_cloud(header, field_names, payload):
    """Build a dense PointCloud2 from an (N, F) float32 array."""
    payload = np.ascontiguousarray(payload, dtype=np.float32)
    n = payload.shape[0]
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
        for i, name in enumerate(field_names)
    ]
    msg.is_bigendian = False
    msg.point_step = 4 * payload.shape[1]
    msg.row_step = msg.point_step * n
    msg.data = payload.tobytes()
    msg.is_dense = False
    return msg
