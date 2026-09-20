#!/usr/bin/env python3
"""Process a synthetic cloud numerically, without initializing a ROS node."""

import json

import numpy as np

from tabletop_grasp.config import GraspConfig, PerceptionConfig
from tabletop_grasp.grasps import generate_grasps
from tabletop_grasp.pipeline import TabletopPipeline
from tabletop_grasp.synthetic_scene import cuboid_scene


def main():
    """Print measured fixture geometry and explicitly synthetic candidates."""
    result = TabletopPipeline(PerceptionConfig()).process(cuboid_scene(seed=7))
    if not result.valid:
        raise SystemExit(result.reason)
    fixture = GraspConfig(
        geometry_source='synthetic_fixture', cad_geometry_verified=True,
        grasp_dimensions_verified=True, jaw_opening_range_verified=True,
        jaw_configuration_verified=True, tcp_verified=True,
        cad_attachment_verified=True, camera_extrinsics_verified=True,
        current_opening_m=0.075, min_opening_m=0.01, max_opening_m=0.075,
        finger_thickness_m=0.006, finger_width_m=0.012, finger_length_m=0.035,
        fingertip_below_tcp_m=0.005, palm_depth_m=0.015, palm_width_m=0.025)
    box = result.boxes[result.target_index]
    candidates, rejected = generate_grasps(box, fixture)
    print(json.dumps({
        'input': 'synthetic fixture; no hardware, no ROS node',
        'dimensions_m': np.round(box.dimensions, 5).tolist(),
        'candidate_count': len(candidates), 'rejections': rejected,
        'physical_geometry_verified': False,
    }, indent=2))
    return 0 if len(candidates) == 2 else 1


if __name__ == '__main__':
    raise SystemExit(main())
