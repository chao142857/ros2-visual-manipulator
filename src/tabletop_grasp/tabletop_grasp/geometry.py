"""Table-constrained cuboid geometry using Open3D and SciPy rotations/PCA.

This estimates visible horizontal support and infers the bottom at the table.
It does not claim to reconstruct occluded faces. All lengths are metres.
"""

from dataclasses import dataclass

import cv2
import numpy as np
import open3d as o3d
from scipy.linalg import eigh
from scipy.spatial.transform import Rotation


@dataclass
class Box:
    center: np.ndarray
    rotation: np.ndarray  # Columns: long, short, up, in processing frame.
    dimensions: np.ndarray
    point_count: int = 0
    quality: float = 0.0
    yaw_observable: bool = True
    observed_bounds: 'Box | None' = None  # Full depth support, never silently shrunk for collision.

    def obb(self):
        result = o3d.geometry.OrientedBoundingBox(self.center, self.rotation, self.dimensions)
        # Some Open3D builds leave this Eigen color vector uninitialized.
        # LineSet.from_obb copies it and otherwise emits invalid-color warnings.
        result.color = [0.5, 0.5, 0.5]
        return result


def plane_basis(normal):
    normal = np.asarray(normal, dtype=float)
    normal = normal / np.linalg.norm(normal)
    # Select from the dominant normal component, so tiny X/Y normal noise
    # on a horizontal table cannot alternate the tangent frame by 90 degrees.
    reference = np.eye(3)[(int(np.argmax(np.abs(normal))) + 1) % 3]
    first = reference - normal * np.dot(reference, normal)
    first /= np.linalg.norm(first)
    return np.column_stack((first, np.cross(normal, first), normal))


def canonical_axes(long_axis, normal, previous=None):
    """Keep the equivalent long-axis sign nearest the previous observation."""
    normal = np.asarray(normal, dtype=float)
    normal /= np.linalg.norm(normal)
    axis = long_axis - normal * np.dot(long_axis, normal)
    axis /= np.linalg.norm(axis)
    reference = previous[:, 0] if previous is not None else np.eye(3)[np.argmax(np.abs(axis))]
    if np.dot(axis, reference) < 0:
        axis = -axis
    return np.column_stack((axis, np.cross(normal, axis), normal))


def estimate_box(points, plane, cfg, previous=None):
    """Horizontal PCA, table-normal vertical, then Open3D box construction.

    Projecting before PCA avoids a vertical/short-axis swap for tall boxes.
    Near-square footprints are reported as yaw-unobservable and never grasped
    using an arbitrary long/short assignment.
    """
    points = np.asarray(points, dtype=float)
    if len(points) < cfg.target_min_points or not np.isfinite(points).all():
        raise ValueError('target has too few finite points')
    basis = plane_basis(plane[:3])
    projected = points @ basis[:, :2]
    _, vectors = eigh(np.cov(projected, rowvar=False))
    rotation = canonical_axes(basis[:, :2] @ vectors[:, -1], plane[:3])
    local = points @ rotation
    low, high = local.min(axis=0), local.max(axis=0)
    if high[1] - low[1] > high[0] - low[0]:
        rotation = canonical_axes(rotation[:, 1], plane[:3])
    if previous is not None and np.linalg.norm(points.mean(axis=0) - previous.center) <= \
            cfg.axis_tracking_distance_m:
        rotation = canonical_axes(rotation[:, 0], plane[:3], previous.rotation)
    local = points @ rotation
    low, high = local.min(axis=0), local.max(axis=0)
    low[2] = -plane[3]  # The cuboid rests on the fitted plane.
    dimensions = high - low
    if np.any(dimensions < cfg.target_min_dimension_m):
        raise ValueError('target dimensions below minimum')
    if np.any(dimensions > cfg.target_max_dimension_m):
        raise ValueError('target dimensions above maximum')
    box = Box(rotation @ ((low + high) / 2), rotation, dimensions, len(points))
    box.yaw_observable = bool(dimensions[0] / dimensions[1] >= cfg.min_horizontal_aspect)
    # A documented support heuristic, not a neural confidence score.
    box.quality = min(1.0, len(points) / (4.0 * cfg.target_min_points))
    return box


def refine_box_from_top(points, plane, cfg, raw_box, metrics, previous=None):
    """Camera-only cuboid metrology from a supported table-parallel top plane.

    Depth discontinuities can spread side pixels beyond the real footprint.
    Preserve their entire original envelope separately. This measured top
    support is not an obstacle/collision replacement or an occlusion guarantee.
    No expected physical dimensions or temporal averaging enter the estimate.
    """
    points = np.asarray(points, dtype=float)
    metrics.update({'mode': 'camera_top_surface_only', 'full_cluster_points': len(points),
                    'full_cluster_dimensions_m': raw_box.dimensions.tolist(),
                    'full_cluster_center_m': raw_box.center.tolist(),
                    'full_cluster_axes_columns': raw_box.rotation.tolist(),
                    'collision_geometry_replaced': False})
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    best = None
    for _ in range(cfg.plane_search_count):
        if len(cloud.points) < cfg.target_min_points:
            break
        coefficients, indices = cloud.segment_plane(
            distance_threshold=cfg.top_surface_distance_m, ransac_n=3,
            num_iterations=cfg.plane_iterations)
        coefficients = np.asarray(coefficients, dtype=float)
        coefficients /= np.linalg.norm(coefficients[:3])
        if np.dot(coefficients[:3], plane[:3]) < 0:
            coefficients *= -1
        tilt = float(np.degrees(np.arccos(np.clip(np.dot(coefficients[:3], plane[:3]), -1, 1))))
        # Peeling discovers hypotheses only. Earlier rejected planes can share
        # real top points, so score every hypothesis on the original cluster.
        residuals = np.abs(points @ coefficients[:3] + coefficients[3])
        inliers = points[residuals <= cfg.top_surface_distance_m]
        metrics.setdefault('plane_candidates', []).append({
            'points': len(inliers), 'discovery_points': len(indices),
            'support_reference': 'full_original_cluster', 'normal_error_deg': tilt})
        if tilt <= cfg.top_surface_max_tilt_deg and (best is None or len(inliers) > len(best[0])):
            best = (inliers, coefficients, tilt)
        cloud = cloud.select_by_index(indices, invert=True)
    if best is None:
        raise ValueError('cuboid top unobservable: no plane agrees with support-plane normal')
    top, coefficients, tilt = best
    ratio = len(top) / len(points)
    residuals = top @ coefficients[:3] + coefficients[3]
    metrics.update({'top_points': len(top), 'top_inlier_ratio': ratio,
                    'top_normal_error_deg': tilt,
                    'top_plane_rmse_m': float(np.sqrt(np.mean(residuals ** 2)))})
    if len(top) < cfg.target_min_points or ratio < cfg.top_surface_min_ratio:
        raise ValueError('cuboid top unobservable: insufficient plane support')
    basis = plane_basis(plane[:3])
    projected = np.asarray(top @ basis[:, :2], dtype=np.float32)
    rectangle = cv2.minAreaRect(projected)
    if min(rectangle[1]) < cfg.target_min_dimension_m:
        metrics['top_rectangle_dimensions_m'] = sorted(rectangle[1], reverse=True)
        raise ValueError('cuboid top unobservable: degenerate or too narrow planar support')
    corners = cv2.boxPoints(rectangle)
    sides = np.roll(corners, -1, axis=0) - corners
    index = int(np.argmax(np.linalg.norm(sides, axis=1)))
    previous_rotation = (previous.rotation if previous is not None
                         and np.linalg.norm(raw_box.center - previous.center) <= cfg.axis_tracking_distance_m
                         else None)
    rotation = canonical_axes(basis[:, :2] @ sides[index], plane[:3], previous_rotation)
    projected = top @ rotation
    low, high = projected.min(axis=0), projected.max(axis=0)
    heights = top @ plane[:3] + plane[3]
    height = float(np.median(heights))
    low[2], high[2] = -plane[3], -plane[3] + height
    dimensions = high - low
    hull_area = cv2.contourArea(cv2.convexHull(np.asarray(top @ basis[:, :2], dtype=np.float32)))
    fill = float(hull_area / max(dimensions[0] * dimensions[1], np.finfo(float).eps))
    all_local = points @ rotation
    lateral_excess = np.max(np.maximum(low[:2] - all_local[:, :2], all_local[:, :2] - high[:2]), axis=1)
    above_top_ratio = float(np.mean(points @ plane[:3] + plane[3] > height + cfg.top_surface_max_above_m))
    lateral_outlier_ratio = float(np.mean(lateral_excess > cfg.top_surface_max_lateral_excess_m))
    metrics.update({'top_height_m': height, 'top_footprint_fill_ratio': fill,
                    'top_support_dimensions_m': dimensions.tolist(),
                    'max_lateral_depth_excess_m': float(max(0., lateral_excess.max())),
                    'outside_top_footprint_ratio': float(np.mean(lateral_excess > cfg.top_surface_distance_m)),
                    'above_top_outlier_ratio': above_top_ratio,
                    'lateral_outlier_ratio': lateral_outlier_ratio,
                    'unseen_dimensions_verified': False})
    if fill < cfg.top_surface_min_footprint_fill:
        raise ValueError('cuboid top unobservable: insufficient rectangular footprint support')
    if max(above_top_ratio, lateral_outlier_ratio) > cfg.top_surface_max_outlier_ratio:
        raise ValueError('cuboid top conflicts with substantial noncuboid depth support')
    if np.any(dimensions < cfg.target_min_dimension_m) or np.any(dimensions > cfg.target_max_dimension_m):
        raise ValueError('cuboid top dimensions outside configured observability bounds')
    box = Box(rotation @ ((low + high) / 2), rotation, dimensions, len(points),
              quality=min(raw_box.quality, ratio), observed_bounds=raw_box)
    box.yaw_observable = bool(dimensions[0] / dimensions[1] >= cfg.min_horizontal_aspect)
    metrics['refinement_valid'] = True
    return box


def matrix_from_pose(pose):
    q, p = pose.orientation, pose.position
    quat = np.array([q.x, q.y, q.z, q.w])
    pos = np.array([p.x, p.y, p.z])
    if (not np.isfinite(np.r_[quat, pos]).all()
            or not np.isclose(np.linalg.norm(quat), 1., atol=1e-3)):
        raise ValueError('pose must be finite with unit quaternion')
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(quat).as_matrix()
    matrix[:3, 3] = pos
    return matrix


def pose_from_matrix(matrix):
    from geometry_msgs.msg import Pose
    result = Pose()
    result.position.x, result.position.y, result.position.z = map(float, matrix[:3, 3])
    q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = \
        map(float, q)
    return result


def pose_from_box(box):
    matrix = np.eye(4)
    matrix[:3, :3], matrix[:3, 3] = box.rotation, box.center
    return pose_from_matrix(matrix)


def transform_points(points, transform):
    """Apply a tf2 lookup result with SciPy; no optical-axis remapping."""
    rotation, position = transform.transform.rotation, transform.transform.translation
    quat = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
    xyz = np.array([position.x, position.y, position.z])
    if (not np.isfinite(np.r_[quat, xyz]).all()
            or not np.isclose(np.linalg.norm(quat), 1., atol=1e-3)):
        raise ValueError('invalid TF transform')
    return Rotation.from_quat(quat).apply(points) + xyz
