"""Open3D tabletop pipeline independent of ROS callbacks and motion planning."""

from dataclasses import dataclass, field

import numpy as np
import open3d as o3d

from .geometry import Box, estimate_box, plane_basis, refine_box_from_top
from .segmentation import GeometrySegmenter, select_target


def cloud_from_xyz(points):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=float))
    return cloud


@dataclass
class Observation:
    filtered: np.ndarray
    table_points: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    target_points: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    table: Box | None = None
    plane: np.ndarray | None = None
    plane_ratio: float = 0.0
    boxes: list = field(default_factory=list)
    target_index: int = -1
    reason: str = ''
    geometry_metrics: list = field(default_factory=list)
    metrology_metrics: dict = field(default_factory=dict)

    @property
    def valid(self):
        return self.target_index >= 0 and not self.reason


class TabletopPipeline:
    def __init__(self, config, segmenter=None, frame_mode='base', camera_max_incidence_deg=75.0):
        config.validate()
        if frame_mode not in ('base', 'camera'):
            raise ValueError('frame_mode must be base or camera')
        if config.refine_top_surface and frame_mode != 'camera':
            raise ValueError('top-surface refinement is camera-only; planning collision semantics are unverified')
        if not np.isfinite(camera_max_incidence_deg) or not 0 < camera_max_incidence_deg < 90:
            raise ValueError('camera_max_incidence_deg must be between 0 and 90')
        self.cfg = config
        self.segmenter = segmenter or GeometrySegmenter(config)
        self.frame_mode = frame_mode
        self.camera_max_incidence_deg = camera_max_incidence_deg
        self.previous = None
        self.metrology_reference = None
        self.metrology_pending = None
        self.metrology_pending_dimensions = []
        self.metrology_count = 0
        self.metrology_last_stamp = None

    def reset(self, expire=False):
        self.previous = None
        self.metrology_pending = None
        self.metrology_pending_dimensions = []
        self.metrology_count = 0
        if expire:
            self.metrology_reference = None
            self.metrology_last_stamp = None

    def begin_metrology_frame(self, stamp_sec):
        """Timestamp every input, including rejected geometry, for loss handling."""
        if not self.cfg.metrology_consistency_enabled:
            return
        if stamp_sec is None or not np.isfinite(stamp_sec) or stamp_sec <= 0:
            self.reset(expire=True)
            raise ValueError('camera metrology requires a finite positive source timestamp')
        if self.metrology_last_stamp is not None:
            gap = stamp_sec - self.metrology_last_stamp
            if gap <= 0:
                self.reset(expire=True)
                self.metrology_last_stamp = stamp_sec
                raise ValueError('camera metrology source timestamp did not advance')
            if gap > self.cfg.metrology_max_gap_sec:
                self.reset(expire=True)
        self.metrology_last_stamp = stamp_sec

    def check_metrology_box(self, box, metrics):
        """Rejection only: dimensions are compared, never substituted or averaged.

        Keep a dimension reference through ordinary geometry failures. Stream
        loss or relocation starts fresh acquisition. Rotation remains free.
        Consistency cannot certify that a partially observed top is complete.
        """
        cfg = self.cfg
        metrics.update({'enabled': cfg.metrology_consistency_enabled,
                        'full_extent_verified': False, 'measurement_kind': 'visible_top_support',
                        'output_is_fresh_measurement': True})
        if not cfg.metrology_consistency_enabled:
            return ''
        if self.metrology_reference is not None:
            if np.linalg.norm(box.center - self.metrology_reference.center) > cfg.axis_tracking_distance_m:
                self.metrology_reference = None
                self.metrology_pending = None
                self.metrology_pending_dimensions = []
                self.metrology_count = 0
                metrics['reference_reset_reason'] = 'object center relocated beyond tracking radius'
            else:
                delta = np.abs(box.dimensions - self.metrology_reference.dimensions)
                metrics['reference_dimensions_m'] = self.metrology_reference.dimensions.tolist()
                metrics['dimension_delta_m'] = delta.tolist()
                if np.any(delta > cfg.metrology_dimension_tolerance_m):
                    self.metrology_pending = None
                    self.metrology_pending_dimensions = []
                    self.metrology_count = 0
                    metrics['accepted'] = False
                    return 'visible top support changed dimensions beyond metrology tolerance'
        pending = self.metrology_pending
        need_new = (pending is None
                    or np.linalg.norm(box.center - pending.center) > cfg.axis_tracking_distance_m
                    or np.any(np.abs(box.dimensions - pending.dimensions) > cfg.metrology_dimension_tolerance_m))
        dimensions_window = (self.metrology_pending_dimensions[-(cfg.metrology_required_observations - 1):]
                             + [box.dimensions.copy()])
        span = np.ptp(dimensions_window, axis=0)
        need_new = need_new or np.any(span > cfg.metrology_dimension_tolerance_m)
        if need_new:
            self.metrology_pending = box
            self.metrology_pending_dimensions = [box.dimensions.copy()]
            self.metrology_count = 1
        else:
            self.metrology_pending_dimensions = dimensions_window
            self.metrology_count = min(cfg.metrology_required_observations, self.metrology_count + 1)
        metrics['agreement_span_m'] = np.ptp(self.metrology_pending_dimensions, axis=0).tolist()
        metrics['consecutive_observations'] = self.metrology_count
        metrics['required_observations'] = cfg.metrology_required_observations
        if self.metrology_count < cfg.metrology_required_observations:
            metrics['accepted'] = False
            return f'awaiting consistent fresh top support ({self.metrology_count}/{cfg.metrology_required_observations})'
        if self.metrology_reference is None:
            self.metrology_reference = self.metrology_pending
        metrics['reference_dimensions_m'] = self.metrology_reference.dimensions.tolist()
        metrics['accepted'] = True
        return ''

    def process(self, xyz, stamp_sec=None):
        cfg = self.cfg
        points = np.asarray(xyz, dtype=float).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        points = points[np.all((points >= cfg.roi_min) & (points <= cfg.roi_max), axis=1)]
        result = Observation(points)
        try:
            self.begin_metrology_frame(stamp_sec)
            if len(points) < cfg.plane_min_points + cfg.target_min_points:
                raise ValueError('insufficient points in workspace ROI')
            cloud = cloud_from_xyz(points).voxel_down_sample(cfg.voxel_m)
            if len(cloud.points) <= cfg.outlier_neighbors:
                raise ValueError('insufficient points after voxel filtering')
            cloud, _ = cloud.remove_statistical_outlier(
                nb_neighbors=cfg.outlier_neighbors, std_ratio=cfg.outlier_std_ratio)
            if cfg.radius_neighbors:
                cloud, _ = cloud.remove_radius_outlier(
                    nb_points=cfg.radius_neighbors, radius=cfg.outlier_radius_m)
            result.filtered = np.asarray(cloud.points)
            if len(result.filtered) < cfg.plane_min_points:
                raise ValueError('insufficient filtered scene points')
            self._table(cloud, result)
            heights = result.filtered @ result.plane[:3] + result.plane[3]
            above = cloud.select_by_index(np.flatnonzero(
                (heights >= cfg.object_above_plane_m) & (heights <= cfg.object_max_height_m)))
            clusters = []
            rejected = []
            for indices in self.segmenter.segment(above):
                points = np.asarray(above.select_by_index(list(indices)).points)
                metrics = {}
                try:
                    box = estimate_box(points, result.plane, cfg, self.previous)
                    if cfg.refine_top_surface:
                        result.geometry_metrics.append(metrics)
                        box = refine_box_from_top(points, result.plane, cfg, box, metrics, self.previous)
                    result.boxes.append(box)
                    clusters.append(points)
                except ValueError as error:
                    # Unmodelled clusters are never silently ignored for planning.
                    rejected.append(str(error))
                    if metrics:
                        metrics['rejection_reason'] = str(error)
            if rejected:
                raise ValueError('unmodelled above-table cluster(s): ' + '; '.join(rejected))
            result.target_index = select_target(result.boxes, cfg.selection)
            result.target_points = clusters[result.target_index]
            if self.frame_mode == 'camera' and not result.boxes[result.target_index].yaw_observable:
                raise ValueError('long/short axes unobservable: near-square visible footprint')
            if cfg.refine_top_surface:
                reason = self.check_metrology_box(result.boxes[result.target_index], result.metrology_metrics)
                if reason:
                    result.reason = reason
                    result.target_index = -1
                    self.previous = None
                    return result
            self.previous = result.boxes[result.target_index]
        except (ValueError, RuntimeError) as error:
            result.reason = str(error)
            result.target_index = -1
            self.reset()
        return result

    def _table(self, cloud, result):
        cfg = self.cfg
        remaining = cloud
        best = None
        for _ in range(cfg.plane_search_count):
            if len(remaining.points) < cfg.plane_min_points:
                break
            coefficients, indices = remaining.segment_plane(
                distance_threshold=cfg.plane_distance_m, ransac_n=3,
                num_iterations=cfg.plane_iterations)
            plane = np.asarray(coefficients)
            plane /= np.linalg.norm(plane[:3])
            points = np.asarray(remaining.select_by_index(indices).points)
            if self.frame_mode == 'camera':
                # Origin is the observing camera. Positive signed distance means
                # the visible side of this candidate support plane; gravity and
                # the identity of this plane as a physical table are unverified.
                if plane[3] < 0:
                    plane *= -1
                center = points.mean(axis=0)
                incidence = np.degrees(np.arccos(np.clip(
                    plane[3] / max(np.linalg.norm(center), np.finfo(float).eps), 0, 1)))
                supported = (plane[3] >= cfg.min_depth_m
                             and incidence <= self.camera_max_incidence_deg)
            else:
                if plane[2] < 0:
                    plane *= -1
                tilt = np.degrees(np.arccos(np.clip(plane[2], -1, 1)))
                height = -plane[3] / max(plane[2], np.finfo(float).eps)
                supported = (tilt <= cfg.plane_max_tilt_deg
                             and cfg.table_height_min_m <= height <= cfg.table_height_max_m)
            if len(indices) >= cfg.plane_min_points and supported:
                if best is None or len(indices) > len(best[1]):
                    best = (plane, points)
            remaining = remaining.select_by_index(indices, invert=True)
        if best is None:
            if self.frame_mode == 'camera':
                raise ValueError('no sufficiently supported camera-facing plane candidate')
            raise ValueError('no supported horizontal table plane in configured height range')
        plane, points = best
        ratio = len(points) / len(cloud.points)
        if ratio < cfg.plane_min_ratio:
            raise ValueError('table plane support ratio below threshold')
        rotation = plane_basis(plane[:3])
        projected = points @ rotation
        low, high = projected.min(axis=0), projected.max(axis=0)
        if np.min((high - low)[:2]) < cfg.table_min_extent_m:
            raise ValueError('table footprint too small')
        low[:2] -= cfg.table_padding_m
        high[:2] += cfg.table_padding_m
        high[2] = -plane[3]
        low[2] = high[2] - cfg.table_thickness_m
        result.table = Box(rotation @ ((low + high) / 2), rotation, high - low, len(points))
        result.table_points, result.plane, result.plane_ratio = points, plane, ratio
