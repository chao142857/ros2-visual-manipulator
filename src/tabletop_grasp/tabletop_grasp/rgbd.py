"""Registered RGB-D conversion through cv_bridge, OpenCV and Open3D."""

from functools import lru_cache

import cv2
import numpy as np
import open3d as o3d


@lru_cache(maxsize=4)
def _inverse_brown_rays(width, height, intrinsics, coefficients):
    """Cache calibration rays only; no depth, point cloud, pose or timestamp.

    Vectorized librealsense 2.56.2 rs2_deproject_pixel_to_point inverse-Brown
    branch (src/rs.cpp): identical 10 iterations including its xq/yq terms.
    Float32 arithmetic matches the SDK intrinsics and deprojection API.
    """
    fx, fy, cx, cy = np.asarray(intrinsics, dtype=np.float32)
    d = np.asarray(coefficients, dtype=np.float32)
    v, u = np.indices((height, width), dtype=np.float32)
    x, y = (u - cx) / fx, (v - cy) / fy
    xo, yo = x.copy(), y.copy()
    for _ in range(10):
        r2 = x * x + y * y
        icdist = 1 / (1 + ((d[4] * r2 + d[1]) * r2 + d[0]) * r2)
        xq, yq = x / icdist, y / icdist
        delta_x = 2 * d[2] * xq * yq + d[3] * (r2 + 2 * xq * xq)
        delta_y = 2 * d[3] * xq * yq + d[2] * (r2 + 2 * yq * yq)
        x, y = (xo - delta_x) * icdist, (yo - delta_y) * icdist
    rays = np.stack((x, y, np.ones_like(x)), axis=-1)
    if not np.isfinite(rays).all():
        raise ValueError('inverse Brown-Conrady calibration produced invalid rays')
    rays.setflags(write=False)
    return rays


def rgbd_to_xyz(color, depth, info, cfg, already_rectified=False,
                inverse_brown_conrady=False):
    if depth.ndim != 2 or color.shape[:2] != depth.shape:
        raise ValueError('RGB and registered depth dimensions differ')
    height, width = depth.shape
    if (width, height) != (info.width, info.height):
        raise ValueError('CameraInfo dimensions differ from registered images')
    k = np.asarray(info.k, dtype=float).reshape(3, 3)
    if not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0:
        raise ValueError('invalid camera intrinsics')
    r = np.asarray(info.r).reshape(3, 3)
    if not np.allclose(r, np.eye(3), atol=1e-6):
        raise ValueError('non-identity rectification rotation requires a matching optical frame')
    if depth.dtype == np.uint16:
        depth = depth.astype(np.float32) / 1000.0
    elif depth.dtype == np.float32:
        depth = depth.copy()
    else:
        raise ValueError('depth must be 16UC1 millimetres or 32FC1 metres')
    valid = np.isfinite(depth) & (depth >= cfg.min_depth_m) & (depth <= cfg.max_depth_m)
    depth[~valid] = 0.0
    if inverse_brown_conrady:
        if already_rectified:
            raise ValueError('inverse Brown-Conrady requires rgbd_rectified=false')
        # RealSense ROS reports plumb_bob for these profiles, losing the SDK
        # model distinction. This explicit override requires verified SDK data;
        # the topic name image_rect_raw alone does not establish rectification.
        if info.distortion_model not in ('plumb_bob', 'inverse_brown_conrady'):
            raise ValueError('inverse Brown-Conrady requires a compatible CameraInfo model')
        distortion = np.asarray(info.d, dtype=float)
        if distortion.shape != (5,) or not np.isfinite(distortion).all():
            raise ValueError('inverse Brown-Conrady requires five finite coefficients')
        rays = _inverse_brown_rays(
            width, height, (k[0, 0], k[1, 1], k[0, 2], k[1, 2]), tuple(distortion))
        # Match the Open3D RGBDImage depth_trunc boundary in the default path.
        valid &= depth < cfg.max_depth_m
        return rays[valid] * depth[valid, None]
    if already_rectified:
        k = np.asarray(info.p, dtype=float).reshape(3, 4)[:, :3]
        if not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0:
            raise ValueError('invalid rectified projection matrix')
    elif np.any(info.d):
        if info.distortion_model not in ('plumb_bob', 'rational_polynomial'):
            raise ValueError(f'unsupported distortion model {info.distortion_model}')
        distortion = np.asarray(info.d, dtype=float)
        if not np.isfinite(distortion).all():
            raise ValueError('invalid distortion coefficients')
        maps = cv2.initUndistortRectifyMap(
            k, distortion, np.eye(3), k, (width, height), cv2.CV_32FC1)
        depth = cv2.remap(depth, *maps, interpolation=cv2.INTER_NEAREST)
        color = cv2.remap(color, *maps, interpolation=cv2.INTER_LINEAR)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(np.ascontiguousarray(color)),
        o3d.geometry.Image(np.ascontiguousarray(depth)),
        depth_scale=1.0, depth_trunc=cfg.max_depth_m, convert_rgb_to_intensity=False)
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        width, height, k[0, 0], k[1, 1], k[0, 2], k[1, 2])
    cloud = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
    return np.asarray(cloud.points)
