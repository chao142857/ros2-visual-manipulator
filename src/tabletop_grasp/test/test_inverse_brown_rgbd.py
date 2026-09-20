"""Check explicit inverse-Brown deprojection against the SDK without devices."""

from types import SimpleNamespace

import numpy as np
import pytest

from tabletop_grasp.config import PerceptionConfig
from tabletop_grasp.rgbd import _inverse_brown_rays, rgbd_to_xyz


def calibration():
    # Synthetic intrinsics for offline math tests; not a recorded device calibration.
    fx, fy, cx, cy = 400., 405., 320., 240.
    k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
    return SimpleNamespace(
        width=640, height=480, k=k, p=[fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.],
        r=np.eye(3).ravel().tolist(), distortion_model='plumb_bob',
        d=[-.05, .06, -.0005, -.00001, -.02])


def sdk_intrinsics(rs, info):
    result = rs.intrinsics()
    result.width, result.height = info.width, info.height
    result.fx, result.fy, result.ppx, result.ppy = info.k[0], info.k[4], info.k[2], info.k[5]
    result.coeffs = info.d
    result.model = rs.distortion.inverse_brown_conrady
    return result


@pytest.mark.parametrize('distance', [.08, .25, .59])
def test_inverse_brown_matches_sdk_across_field_of_view(distance):
    rs = pytest.importorskip('pyrealsense2')
    info = calibration()
    pixels = [(u, v) for v in np.linspace(0, 479, 7, dtype=int)
              for u in np.linspace(0, 639, 9, dtype=int)]
    depth = np.zeros((info.height, info.width), dtype=np.float32)
    for u, v in pixels:
        depth[v, u] = distance
    xyz = rgbd_to_xyz(np.zeros((*depth.shape, 3), np.uint8), depth, info,
                      PerceptionConfig(), inverse_brown_conrady=True)
    intr = sdk_intrinsics(rs, info)
    # This pure SDK math function constructs no context, sensor or pipeline.
    expected = [rs.rs2_deproject_pixel_to_point(intr, [float(u), float(v)], float(depth[v, u]))
                for u, v in pixels]
    assert np.allclose(xyz, expected, rtol=0., atol=2e-7)


def test_cached_rays_reuse_calibration_but_never_reuse_depth_or_points():
    info = calibration()
    depth = np.zeros((480, 640), np.float32)
    color = np.zeros((480, 640, 3), np.uint8)
    depth[100, 100] = .2
    original = depth.copy()
    _inverse_brown_rays.cache_clear()
    first = rgbd_to_xyz(color, depth, info, PerceptionConfig(), inverse_brown_conrady=True)
    assert np.array_equal(original, depth)
    depth[100, 100] = .4
    second = rgbd_to_xyz(color, depth, info, PerceptionConfig(), inverse_brown_conrady=True)
    assert np.allclose(second, first * 2)
    assert _inverse_brown_rays.cache_info().hits == 1
    assert _inverse_brown_rays.cache_info().currsize == 1
    info.d[0] *= 2
    changed = rgbd_to_xyz(color, depth, info, PerceptionConfig(), inverse_brown_conrady=True)
    assert not np.allclose(changed, second)
    assert _inverse_brown_rays.cache_info().currsize == 2


@pytest.mark.parametrize('invalid_d', [[], [0.] * 4, [0.] * 8, [0., 0., np.nan, 0., 0.]])
def test_inverse_brown_rejects_invalid_coefficients(invalid_d):
    info = calibration()
    info.d = invalid_d
    with pytest.raises(ValueError, match='five finite coefficients'):
        rgbd_to_xyz(np.zeros((480, 640, 3), np.uint8), np.zeros((480, 640), np.float32),
                    info, PerceptionConfig(), inverse_brown_conrady=True)


def test_inverse_brown_refuses_rectified_double_correction():
    with pytest.raises(ValueError, match='rgbd_rectified=false'):
        rgbd_to_xyz(np.zeros((480, 640, 3), np.uint8), np.zeros((480, 640), np.float32),
                    calibration(), PerceptionConfig(), already_rectified=True,
                    inverse_brown_conrady=True)


@pytest.mark.parametrize('dtype', [np.uint16, np.float32])
def test_inverse_brown_zero_coefficients_preserves_default_validity_and_units(dtype):
    info = calibration()
    info.d = [0.] * 5
    color = np.zeros((480, 640, 3), np.uint8)
    depth = np.zeros((480, 640), dtype=dtype)
    values = np.array([.01, .07, .20, .59, .60, .65])
    depth[200, 250:256] = values * 1000 if dtype == np.uint16 else values
    if dtype == np.float32:
        depth[201, :3] = [np.nan, np.inf, -np.inf]
    nominal = rgbd_to_xyz(color, depth, info, PerceptionConfig())
    explicit = rgbd_to_xyz(color, depth, info, PerceptionConfig(), inverse_brown_conrady=True)
    assert nominal.shape == explicit.shape
    assert np.allclose(nominal, explicit, atol=1e-7)


