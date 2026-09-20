"""Typed algorithm configuration; each field is exposed as a ROS parameter."""

from dataclasses import dataclass, fields
import math

import numpy as np


@dataclass
class PerceptionConfig:
    min_depth_m: float = 0.07
    max_depth_m: float = 0.60
    roi_min: tuple = (-0.55, -0.55, -0.06)
    roi_max: tuple = (0.55, 0.55, 0.65)
    voxel_m: float = 0.003
    outlier_neighbors: int = 20
    outlier_std_ratio: float = 2.0
    radius_neighbors: int = 0  # 0 disables the optional second filter.
    outlier_radius_m: float = 0.012
    plane_distance_m: float = 0.004
    plane_iterations: int = 700
    plane_search_count: int = 4
    plane_min_points: int = 150
    plane_min_ratio: float = 0.15
    plane_max_tilt_deg: float = 15.0
    table_height_min_m: float = -0.05
    table_height_max_m: float = 0.10
    table_min_extent_m: float = 0.10
    table_thickness_m: float = 0.03
    table_padding_m: float = 0.03
    object_above_plane_m: float = 0.006
    object_max_height_m: float = 0.20
    cluster_eps_m: float = 0.012
    cluster_min_points: int = 12
    target_min_points: int = 70
    target_min_dimension_m: float = 0.012
    target_max_dimension_m: float = 0.25
    min_horizontal_aspect: float = 1.15
    axis_tracking_distance_m: float = 0.04
    selection: str = 'isolated'
    # Camera-only metrology option. It never replaces a planning collision body.
    refine_top_surface: bool = False
    top_surface_distance_m: float = 0.002
    top_surface_max_tilt_deg: float = 10.0
    top_surface_min_ratio: float = 0.20
    top_surface_min_footprint_fill: float = 0.65
    top_surface_max_above_m: float = 0.006
    top_surface_max_lateral_excess_m: float = 0.012
    top_surface_max_outlier_ratio: float = 0.05
    # Reject inconsistent camera metrology; never average or reuse output poses.
    metrology_consistency_enabled: bool = False
    metrology_required_observations: int = 3
    metrology_dimension_tolerance_m: float = 0.006
    metrology_max_gap_sec: float = 1.0

    def validate(self):
        validate_numbers(self)
        if len(self.roi_min) != 3 or len(self.roi_max) != 3:
            raise ValueError('ROI bounds must be 3-vectors')
        if not np.all(np.asarray(self.roi_min) < self.roi_max):
            raise ValueError('ROI minimum must be below maximum')
        if not 0 < self.min_depth_m < self.max_depth_m:
            raise ValueError('invalid depth range')
        for name in ('voxel_m', 'plane_distance_m', 'cluster_eps_m', 'outlier_std_ratio',
                     'outlier_radius_m', 'table_thickness_m', 'table_min_extent_m',
                     'target_min_dimension_m', 'axis_tracking_distance_m'):
            if getattr(self, name) <= 0:
                raise ValueError(f'{name} must be positive')
        for name in ('plane_iterations', 'plane_search_count', 'plane_min_points',
                     'cluster_min_points', 'target_min_points', 'outlier_neighbors'):
            if getattr(self, name) < 3:
                raise ValueError(f'{name} must be at least 3')
        if self.radius_neighbors < 0 or self.table_padding_m < 0:
            raise ValueError('negative filter count or padding')
        if not 0 < self.plane_min_ratio <= 1 or not 0 < self.plane_max_tilt_deg < 90:
            raise ValueError('invalid plane acceptance thresholds')
        if self.table_height_min_m >= self.table_height_max_m:
            raise ValueError('invalid table height range')
        if not self.plane_distance_m < self.object_above_plane_m < self.object_max_height_m:
            raise ValueError('object height thresholds must exceed plane noise')
        if self.target_max_dimension_m <= self.target_min_dimension_m:
            raise ValueError('invalid object dimension range')
        if self.min_horizontal_aspect <= 1 or self.selection not in ('isolated', 'largest'):
            raise ValueError('invalid aspect ratio or target selection policy')
        if self.top_surface_distance_m <= 0 or not 0 < self.top_surface_max_tilt_deg < 90:
            raise ValueError('invalid top-surface plane thresholds')
        if not 0 < self.top_surface_min_ratio <= 1 or not 0 < self.top_surface_min_footprint_fill <= 1:
            raise ValueError('invalid top-surface support thresholds')
        if min(self.top_surface_max_above_m, self.top_surface_max_lateral_excess_m) <= 0:
            raise ValueError('top-surface residual tolerances must be positive')
        if not 0 <= self.top_surface_max_outlier_ratio < 1:
            raise ValueError('invalid top-surface residual outlier ratio')
        if self.metrology_required_observations < 2:
            raise ValueError('metrology consistency needs at least two observations')
        if min(self.metrology_dimension_tolerance_m, self.metrology_max_gap_sec) <= 0:
            raise ValueError('metrology consistency tolerances must be positive')
        if self.metrology_consistency_enabled and not self.refine_top_surface:
            raise ValueError('metrology consistency requires explicit top-surface refinement')


@dataclass
class GraspConfig:
    # Unknown physical measurements MUST remain unverified. No pulse mapping.
    # Deprecated compatibility flag: never sufficient to authorize planning.
    geometry_verified: bool = False
    geometry_source: str = 'unverified'  # cad or synthetic_fixture
    cad_geometry_verified: bool = False
    grasp_dimensions_verified: bool = False
    jaw_opening_range_verified: bool = False
    jaw_configuration_verified: bool = False
    tcp_verified: bool = False
    cad_attachment_verified: bool = False
    camera_extrinsics_verified: bool = False
    cad_collision_proxy_path: str = ''
    # Native CAD-frame bounds are reporting metadata, not jaw/TCP parameters.
    cad_envelope_xyz_m: tuple = (0.0, 0.0, 0.0)
    cad_tip_pad_envelope_xyz_m: tuple = (0.0, 0.0, 0.0)
    cad_static_inner_gap_m: float = 0.0
    # T_tcp_cad is a separately measured attachment, NOT inferred from the STEP.
    tcp_from_cad_xyz_m: tuple = (0.0, 0.0, 0.0)
    tcp_from_cad_xyzw: tuple = (0.0, 0.0, 0.0, 1.0)
    min_opening_m: float = 0.0
    max_opening_m: float = 0.0
    current_opening_m: float = 0.0  # Verified static opening; no jaw commands occur.
    finger_thickness_m: float = 0.0  # Along closing axis.
    finger_width_m: float = 0.0      # Along object long axis.
    finger_length_m: float = 0.0     # Along approach axis.
    fingertip_below_tcp_m: float = 0.0
    palm_depth_m: float = 0.0
    palm_width_m: float = 0.0
    opening_margin_m: float = 0.004
    edge_margin_m: float = 0.005
    table_clearance_m: float = 0.006
    palm_clearance_m: float = 0.005
    grasp_depth_from_top_m: float = 0.012
    pregrasp_distance_m: float = 0.06
    retreat_distance_m: float = 0.08
    long_axis_offsets_m: tuple = (0.0,)
    # R_grasp_tcp: TCP +X closes, TCP -Y approaches; grasp +Z approaches.
    grasp_from_tcp_xyzw: tuple = (-0.7071067811865476, 0.0, 0.0, 0.7071067811865476)

    def validate(self):
        validate_numbers(self)
        if self.geometry_source not in ('unverified', 'cad', 'synthetic_fixture'):
            raise ValueError('geometry_source must be unverified, cad or synthetic_fixture')
        for field in fields(self):
            value = getattr(self, field.name)
            if field.name.endswith('_m') and isinstance(value, (float, int)) and value < 0:
                raise ValueError(f'{field.name} must be nonnegative')
        if not self.long_axis_offsets_m:
            raise ValueError('at least one grasp offset is required')
        for name in ('grasp_from_tcp_xyzw', 'tcp_from_cad_xyzw'):
            q = np.asarray(getattr(self, name))
            if q.shape != (4,) or not np.isclose(np.linalg.norm(q), 1.0, atol=1e-6):
                raise ValueError(f'{name} must be a unit xyzw quaternion')
        if np.asarray(self.tcp_from_cad_xyz_m).shape != (3,):
            raise ValueError('tcp_from_cad_xyz_m must be a 3-vector')
        for name in ('cad_envelope_xyz_m', 'cad_tip_pad_envelope_xyz_m'):
            value = np.asarray(getattr(self, name))
            if value.shape != (3,) or np.any(value < 0):
                raise ValueError(f'{name} must be a nonnegative 3-vector')
        if min(self.pregrasp_distance_m, self.retreat_distance_m) <= 0:
            raise ValueError('approach/retreat distances must be positive')
        if self.jaw_opening_range_verified:
            if not 0 <= self.min_opening_m < self.max_opening_m:
                raise ValueError('invalid usable opening range')
        if self.jaw_configuration_verified and self.current_opening_m <= 0:
            raise ValueError('verified current opening must be positive')
        if self.grasp_dimensions_verified:
            if min(self.finger_thickness_m, self.finger_width_m, self.finger_length_m,
                   self.palm_depth_m, self.palm_width_m) <= 0:
                raise ValueError('verified finger/palm dimensions must be positive')
            if self.tcp_verified and self.fingertip_below_tcp_m >= self.finger_length_m:
                raise ValueError('TCP must lie within finger length')

    def verification(self):
        """Independent claims; a CAD file or legacy flag cannot validate kinematics."""
        names = ('cad_geometry_verified', 'grasp_dimensions_verified',
                 'jaw_opening_range_verified', 'jaw_configuration_verified',
                 'tcp_verified', 'cad_attachment_verified', 'camera_extrinsics_verified')
        return {name: getattr(self, name) for name in names}

    def planning_rejections(self):
        reasons = [f'{name} is unverified' for name, verified in self.verification().items()
                   if verified is not True]
        if self.geometry_source == 'unverified':
            reasons.append('gripper geometry source is unverified')
        if self.geometry_source == 'cad' and not self.cad_collision_proxy_path:
            reasons.append('CAD collision proxy is missing')
        return reasons


def validate_numbers(config):
    for field in fields(config):
        value = getattr(config, field.name)
        if isinstance(value, (int, float)) and not math.isfinite(value):
            raise ValueError(f'{field.name} must be finite')
        if isinstance(value, (list, tuple)) and not np.all(np.isfinite(value)):
            raise ValueError(f'{field.name} must be finite')


def declare_config(node, cls, prefix):
    defaults = cls()
    values = {}
    for field in fields(defaults):
        value = getattr(defaults, field.name)
        node.declare_parameter(f'{prefix}.{field.name}', list(value) if isinstance(value, tuple)
                               else value)
        values[field.name] = node.get_parameter(f'{prefix}.{field.name}').value
    result = cls(**values)
    result.validate()
    return result
