"""Unit tests for the core logic of the runtime nodes (no ROS graph needed)."""
import os
import struct
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

import fusion_web_streamer as streamer  # noqa: E402
import registration_merge_node as merge  # noqa: E402
import sensor_fusion_node as fusion  # noqa: E402
from sensor_msgs.msg import Image, PointCloud2, PointField  # noqa: E402


def make_cloud_msg(points):
    """Build an x, y, z, rgb, thermal PointCloud2 from an (N, 5) float32 array."""
    msg = PointCloud2()
    msg.height, msg.width = 1, len(points)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(['x', 'y', 'z', 'rgb', 'thermal'])]
    msg.point_step = 20
    msg.row_step = 20 * len(points)
    msg.data = np.asarray(points, dtype=np.float32).tobytes()
    return msg


def test_voxel_downsample_keeps_one_point_per_voxel():
    xyz = np.array([[0.0, 0.0, 0.0], [0.01, 0.01, 0.01], [1.0, 1.0, 1.0]], dtype=np.float32)
    assert len(merge.voxel_downsample_indices(xyz, 0.1)) == 2
    assert len(merge.voxel_downsample_indices(xyz, 0.0)) == 3      # voxel size 0 keeps everything


def test_icp_recovers_a_small_known_motion():
    rng = np.random.default_rng(1)
    target = rng.uniform(-3, 3, (3000, 3))
    shift = np.array([0.03, -0.02, 0.01])
    r, t, converged = merge.icp_point_to_point(target - shift, target, 20, 0.5, 200)
    assert converged
    assert np.allclose(t, shift, atol=0.005) and np.allclose(r, np.eye(3), atol=1e-3)


def test_icp_with_too_few_points_returns_identity():
    r, t, converged = merge.icp_point_to_point(np.zeros((10, 3)), np.zeros((10, 3)), 10, 0.5, 200)
    assert not converged and np.allclose(r, np.eye(3)) and np.allclose(t, 0)


def test_projection_centre_and_out_of_view():
    pts = np.array([[2.0, 0.0, 0.0], [2.0, 5.0, 0.0], [-1.0, 0.0, 0.0]], dtype=np.float32)
    ui, vi, valid = fusion.project_to_pixels(pts, 1000.0, 1000.0, 640.0, 360.0, 1280, 720)
    assert (ui[0], vi[0]) == (640, 360) and valid[0]
    assert not valid[1]            # far to the left: outside the image
    assert not valid[2]            # behind the camera


def test_projection_with_lens_distortion_matches_opencv():
    cv2 = pytest.importorskip('cv2')
    rng = np.random.default_rng(1)
    pts = np.column_stack([rng.uniform(1.5, 5.0, 300), rng.uniform(-1.5, 1.5, 300), rng.uniform(-1.0, 1.0, 300)]).astype(np.float32)
    k, dist = (1330.0, 1330.0, 617.0, 369.0), (-0.476, 0.223, 0.001, -0.002, 0.0)
    ui, vi, valid = fusion.project_to_pixels(pts, *k, 1280, 720, dist)
    optical = np.column_stack([-pts[:, 1], -pts[:, 2], pts[:, 0]])
    matrix = np.array([[k[0], 0, k[2]], [0, k[1], k[3]], [0, 0, 1.0]])
    ref, _ = cv2.projectPoints(optical.astype(np.float64), np.zeros(3), np.zeros(3), matrix, np.array(dist))
    ref = ref.reshape(-1, 2)
    assert valid.any()
    assert np.abs(ui[valid] - ref[valid, 0]).max() <= 1 and np.abs(vi[valid] - ref[valid, 1]).max() <= 1


def test_projection_survives_near_sideways_points_with_distortion():
    pts = np.array([[0.06, 5.0, 0.0], [0.06, -5.0, 3.0], [0.05001, 80.0, -80.0], [2.0, 0.0, 0.0]], dtype=np.float32)
    ui, vi, valid = fusion.project_to_pixels(pts, 1330.0, 1330.0, 617.0, 369.0, 1280, 720, (-0.476, 0.223, 0.0, 0.0, 0.0))
    assert not valid[:3].any()          # far outside the lens' view: never a wrapped-around "valid" pixel
    assert valid[3] and abs(ui[3] - 617) <= 1 and abs(vi[3] - 369) <= 1


def test_projection_without_distortion_is_unchanged():
    pts = np.array([[2.0, 0.3, -0.2]], dtype=np.float32)
    plain = fusion.project_to_pixels(pts, 1000.0, 1000.0, 640.0, 360.0, 1280, 720)
    zero = fusion.project_to_pixels(pts, 1000.0, 1000.0, 640.0, 360.0, 1280, 720, (0.0,) * 5)
    assert all(np.array_equal(a, b) for a, b in zip(plain, zero))


def test_lidar_to_camera_identity_without_extrinsics():
    m, b = fusion.lidar_to_camera_transform({}, 'lidar', 'camera')
    assert np.allclose(m, np.eye(3)) and np.allclose(b, 0)


def test_boresight_correction_rotates_the_transform():
    m, b = np.eye(3, dtype=np.float32), np.zeros(3, dtype=np.float32)
    m2, _ = fusion.apply_boresight_correction((m, b), 90.0, 0.0)
    assert np.allclose(np.array([1.0, 0.0, 0.0]) @ m2, [0.0, 1.0, 0.0], atol=1e-6)   # a +90 degree yaw turns the x axis onto y


def test_image_to_array_honours_row_padding():
    msg = Image()
    msg.height, msg.width, msg.step = 2, 3, 8        # 3 pixels of mono8 plus 5 bytes of padding per row
    msg.data = bytes(range(16))
    arr = fusion.image_to_array(msg, 1, np.uint8)
    assert arr.shape == (2, 3, 1) and arr[1, 0, 0] == 8


def test_image_to_array_rejects_short_buffer():
    msg = Image()
    msg.height, msg.width, msg.step, msg.data = 4, 4, 4, bytes(8)
    with pytest.raises(ValueError):
        fusion.image_to_array(msg, 1, np.uint8)


def test_encode_cloud_wire_format_roundtrip():
    rng = np.random.default_rng(2)
    n = 500
    packed = (rng.integers(0, 256, n).astype(np.uint32) << 16 | 7 << 8 | 9).view(np.float32)
    pts = np.column_stack([rng.uniform(0, 10, n), rng.uniform(-3, 3, n), rng.uniform(-1, 2, n), packed,
                           rng.uniform(250, 320, n)]).astype(np.float32)
    count, payload = streamer._encode_cloud(make_cloud_msg(pts), 10000, np.random.default_rng(0))
    assert count == n and len(payload) == 16 + 11 * n
    ox, oy, oz, scale = struct.unpack_from('<ffff', payload, 0)
    body = np.frombuffer(payload, dtype=np.dtype([('xyz', '<i2', 3), ('rgb', 'u1', 3), ('t', '<i2')]), offset=16)
    decoded = np.array([ox, oy, oz]) + body['xyz'] * scale
    assert np.abs(decoded - pts[:, :3]).max() < 2 * scale          # sub-millimetre quantisation
    assert np.allclose(body['t'] / 100.0, pts[:, 4], atol=0.006)


def test_encode_cloud_caps_points_and_marks_missing_thermal():
    pts = np.zeros((300, 5), dtype=np.float32)
    pts[:, 0] = np.arange(300)
    pts[:, 4] = np.nan
    count, payload = streamer._encode_cloud(make_cloud_msg(pts), 100, np.random.default_rng(0))
    assert count == 100
    body = np.frombuffer(payload, dtype=np.dtype([('xyz', '<i2', 3), ('rgb', 'u1', 3), ('t', '<i2')]), offset=16)
    assert (body['t'] == -32768).all()
