"""Short-dimension parallel-jaw grasps; no inverse kinematics or commands."""

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class Candidate:
    id: str
    grasp: np.ndarray       # T_base_tcp
    pregrasp: np.ndarray
    retreat: np.ndarray
    required_opening_m: float
    long_offset_m: float
    symmetry: int


def generate_grasps(box, cfg):
    cfg.validate()
    reasons = cfg.planning_rejections()
    if reasons:
        return [], reasons
    if not box.yaw_observable:
        return [], ['long/short axes are ambiguous (near-square footprint)']
    long_size, short_size, height = box.dimensions
    opening = short_size + 2 * cfg.opening_margin_m
    if short_size < cfg.min_opening_m or opening > cfg.max_opening_m:
        return [], [f'short dimension/opening outside usable range: '
                    f'width={short_size:.4f}, required={opening:.4f} m']
    if not cfg.min_opening_m <= cfg.current_opening_m <= cfg.max_opening_m:
        return [], ['verified current opening is outside the verified usable range']
    if opening > cfg.current_opening_m:
        return [], ['current static jaw opening is insufficient; this planner cannot move jaws']
    grasp_height = height - cfg.grasp_depth_from_top_m
    if grasp_height - cfg.fingertip_below_tcp_m < cfg.table_clearance_m:
        return [], ['finger/table clearance insufficient']
    if cfg.finger_length_m - cfg.fingertip_below_tcp_m - cfg.grasp_depth_from_top_m < \
            cfg.palm_clearance_m:
        return [], ['palm would collide with object top']
    up, long_axis, short_axis = box.rotation[:, 2], box.rotation[:, 0], box.rotation[:, 1]
    candidates = []
    for offset in cfg.long_axis_offsets_m:
        if abs(offset) + cfg.finger_width_m / 2 + cfg.edge_margin_m >= long_size / 2:
            reasons.append(f'offset {offset:.4f} m: finger pad too close to long-axis edge')
            continue
        position = box.center + offset * long_axis + \
            (height / 2 - cfg.grasp_depth_from_top_m) * up
        for symmetry in range(2):
            closing = short_axis * (1 if symmetry == 0 else -1)
            approach = -up
            grasp_axes = np.column_stack((closing, np.cross(approach, closing), approach))
            pose = np.eye(4)
            pose[:3, :3] = grasp_axes @ Rotation.from_quat(cfg.grasp_from_tcp_xyzw).as_matrix()
            pose[:3, 3] = position
            pregrasp, retreat = pose.copy(), pose.copy()
            pregrasp[:3, 3] += cfg.pregrasp_distance_m * up
            retreat[:3, 3] += cfg.retreat_distance_m * up
            candidates.append(Candidate(f'offset_{offset:.4f}_pi_{symmetry}', pose, pregrasp,
                                        retreat, opening, offset, symmetry))
    return candidates, reasons


def gripper_boxes(cfg):
    """Synthetic fixture fingers and palm in the grasp frame.

    Physical CAD uses its own static proxy. Do not synthesize a physical palm
    from the jaw range, nor equate the supplied CAD gap to the maximum opening.
    """
    cfg.validate()
    if cfg.geometry_source != 'synthetic_fixture' or cfg.planning_rejections():
        raise ValueError('analytic gripper boxes require an explicitly verified synthetic fixture')
    opening = cfg.current_opening_m
    z = cfg.fingertip_below_tcp_m - cfg.finger_length_m / 2
    finger_size = [cfg.finger_thickness_m, cfg.finger_width_m, cfg.finger_length_m]
    shapes = [(finger_size, [sign * (opening + cfg.finger_thickness_m) / 2, 0, z])
              for sign in (-1, 1)]
    shapes.append(([opening + 2 * cfg.finger_thickness_m, cfg.palm_width_m, cfg.palm_depth_m],
                   [0, 0, cfg.fingertip_below_tcp_m - cfg.finger_length_m - cfg.palm_depth_m / 2]))
    return shapes


def collision_boxes_in_tcp(cfg):
    """Return conservative static boxes as (dimensions, T_tcp_box).

    This planning-layer attachment leaves the physical URDF unchanged. CAD
    proxy axes retain the CAD frame until a measured T_tcp_cad is supplied.
    """
    cfg.validate()
    reasons = cfg.planning_rejections()
    if reasons:
        raise ValueError('; '.join(reasons))
    if cfg.geometry_source == 'synthetic_fixture':
        tcp_from_grasp = np.eye(4)
        tcp_from_grasp[:3, :3] = Rotation.from_quat(cfg.grasp_from_tcp_xyzw).inv().as_matrix()
        result = []
        for dimensions, center in gripper_boxes(cfg):
            local = np.eye(4)
            local[:3, 3] = center
            result.append((dimensions, tcp_from_grasp @ local))
        return result
    path = cfg.cad_collision_proxy_path
    package_prefix = 'package://tabletop_grasp/'
    if path.startswith(package_prefix):
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('tabletop_grasp')) / path[len(package_prefix):]
    with Path(path).open(encoding='utf-8') as stream:
        proxy = json.load(stream)
    if (proxy.get('units') != 'm' or proxy.get('frame_id') != 'gripper_cad'
            or proxy.get('conservative') is not True):
        raise ValueError('CAD proxy must be conservative, in metres, in gripper_cad')
    margin = proxy.get('safety_margin_m')
    if not isinstance(margin, (int, float)) or not np.isfinite(margin) or margin < 0:
        raise ValueError('CAD proxy must record a finite nonnegative safety margin')
    boxes = proxy.get('boxes')
    if not isinstance(boxes, list) or not boxes:
        raise ValueError('CAD proxy contains no collision boxes')
    tcp_from_cad = np.eye(4)
    tcp_from_cad[:3, :3] = Rotation.from_quat(cfg.tcp_from_cad_xyzw).as_matrix()
    tcp_from_cad[:3, 3] = cfg.tcp_from_cad_xyz_m
    result = []
    for box in boxes:
        dimensions = np.asarray(box.get('dimensions', []), dtype=float)
        center = np.asarray(box.get('center', []), dtype=float)
        quaternion = np.asarray(box.get('quaternion_xyzw', []), dtype=float)
        if (dimensions.shape != (3,) or not np.isfinite(dimensions).all()
                or np.any(dimensions <= 0) or center.shape != (3,)
                or not np.isfinite(center).all() or quaternion.shape != (4,)
                or not np.isfinite(quaternion).all()
                or not np.isclose(np.linalg.norm(quaternion), 1., atol=1e-6)):
            raise ValueError('invalid CAD collision box dimensions, center or quaternion')
        cad_from_box = np.eye(4)
        cad_from_box[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
        cad_from_box[:3, 3] = center
        result.append((dimensions.tolist(), tcp_from_cad @ cad_from_box))
    return result
