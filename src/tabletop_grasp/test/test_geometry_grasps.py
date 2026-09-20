from dataclasses import replace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from tabletop_grasp.config import GraspConfig, PerceptionConfig
from tabletop_grasp.geometry import Box, canonical_axes, estimate_box, plane_basis
from tabletop_grasp.grasps import generate_grasps, gripper_boxes
from tabletop_grasp.pipeline import TabletopPipeline
from tabletop_grasp.synthetic_scene import cuboid_scene


@pytest.fixture
def gripper():
    return GraspConfig(geometry_source='synthetic_fixture', cad_geometry_verified=True,
                       grasp_dimensions_verified=True, jaw_opening_range_verified=True,
                       jaw_configuration_verified=True, tcp_verified=True,
                       cad_attachment_verified=True, camera_extrinsics_verified=True,
                       current_opening_m=0.075, min_opening_m=0.01, max_opening_m=0.075,
                       finger_thickness_m=0.006, finger_width_m=0.012, finger_length_m=0.035,
                       fingertip_below_tcp_m=0.005, palm_depth_m=0.015, palm_width_m=0.025)


def box_at(yaw=0., dimensions=(0.08, 0.034, 0.04)):
    return Box(np.array([0.03, -0.28, 0.008]), Rotation.from_euler('z', yaw).as_matrix(),
               np.array(dimensions))


@pytest.mark.parametrize('yaw', [-3.13, -1.57, -0.6, 0., 0.3, 1.56, 3.13])
def test_noisy_cuboid_short_dimension_and_orientation(yaw, gripper):
    pipeline = TabletopPipeline(PerceptionConfig())
    scene = pipeline.process(cuboid_scene(yaw=yaw))
    assert scene.valid, scene.reason
    box = scene.boxes[scene.target_index]
    assert np.allclose(box.dimensions, [0.08, 0.034, 0.04], atol=0.004)
    expected_long = Rotation.from_euler('z', yaw).apply([1., 0., 0.])
    assert abs(np.dot(box.rotation[:, 0], expected_long)) > 0.995
    assert np.linalg.det(box.rotation) == pytest.approx(1.)
    candidates, reasons = generate_grasps(box, gripper)
    assert len(candidates) == 2, reasons
    for candidate in candidates:
        # Physical TCP +X is the closing direction; TCP -Y approaches the table.
        assert abs(np.dot(candidate.grasp[:3, 0], box.rotation[:, 1])) > 0.999
        assert np.dot(-candidate.grasp[:3, 1], box.rotation[:, 2]) < -0.999
        assert candidate.required_opening_m < 0.05  # Long edge would require > 0.08.
    relative = candidates[0].grasp[:3, :3].T @ candidates[1].grasp[:3, :3]
    assert Rotation.from_matrix(relative).magnitude() == pytest.approx(np.pi)


def test_axis_sign_is_stable_at_canonical_boundary():
    previous = canonical_axes(np.array([1., 1.01, 0.]), np.array([0., 0., 1.]))
    updated = canonical_axes(np.array([-1.01, -1., 0.]), np.array([0., 0., 1.]), previous)
    assert np.dot(previous[:, 0], updated[:, 0]) > 0.99


def test_table_basis_does_not_flip_under_tiny_normal_noise():
    a = plane_basis([1e-6, 1e-5, 1.])
    b = plane_basis([1e-5, 1e-6, 1.])
    assert np.dot(a[:, 0], b[:, 0]) > 0.999


def test_vertical_axis_does_not_become_short_axis():
    cfg = PerceptionConfig()
    rng = np.random.default_rng(4)
    xyz = rng.uniform([-0.04, -0.015, 0.006], [0.04, 0.015, 0.14], (1000, 3))
    box = estimate_box(xyz, np.array([0., 0., 1., 0.]), cfg)
    assert box.dimensions[2] > box.dimensions[0] > box.dimensions[1]
    assert np.dot(box.rotation[:, 2], [0, 0, 1]) > 0.999


def test_noisy_sequence_has_no_pi_flip():
    pipeline = TabletopPipeline(PerceptionConfig())
    previous = None
    for seed in range(5):
        scene = pipeline.process(cuboid_scene(seed=seed, yaw=1.57, noise=0.0005))
        assert scene.valid, scene.reason
        rotation = scene.boxes[0].rotation
        if previous is not None:
            assert np.dot(rotation[:, 0], previous[:, 0]) > 0.99
        previous = rotation


def test_invalid_scene_clears_previous_geometry():
    pipeline = TabletopPipeline(PerceptionConfig())
    assert pipeline.process(cuboid_scene()).valid
    assert not pipeline.process([[np.nan, 0, 0]]).valid
    assert pipeline.previous is None


def test_multiple_objects_fail_isolated_selection():
    scene = cuboid_scene()
    extra = cuboid_scene(center=(-0.10, -0.28))
    extra = extra[extra[:, 2] > 0]
    result = TabletopPipeline(PerceptionConfig()).process(np.vstack([scene, extra]))
    assert not result.valid
    assert 'ambiguous scene' in result.reason


@pytest.mark.parametrize('changes,fragment', [
    ({'grasp_dimensions_verified': False}, 'unverified'),
    ({'max_opening_m': 0.035}, 'opening'),
    ({'min_opening_m': 0.04}, 'opening'),
    ({'fingertip_below_tcp_m': 0.025}, 'table clearance'),
    ({'finger_length_m': 0.017}, 'palm'),
    ({'long_axis_offsets_m': [0.039]}, 'edge'),
])
def test_gripper_constraints_reject(gripper, changes, fragment):
    candidates, reasons = generate_grasps(box_at(), replace(gripper, **changes))
    assert not candidates
    assert fragment in ' '.join(reasons)


def test_near_square_is_explicitly_unobservable(gripper):
    scene = TabletopPipeline(PerceptionConfig()).process(cuboid_scene(size=(0.04, 0.039, 0.04)))
    assert scene.valid, scene.reason
    assert not scene.boxes[0].yaw_observable
    assert not generate_grasps(scene.boxes[0], gripper)[0]


def test_offsets_and_table_safe_open_gripper(gripper):
    candidates, _ = generate_grasps(box_at(), replace(gripper, long_axis_offsets_m=[-0.01, 0., 0.01]))
    assert len(candidates) == 6
    shapes = gripper_boxes(gripper)
    assert len(shapes) == 3
    assert shapes[0][1][0] < 0 < shapes[1][1][0]


@pytest.mark.parametrize('config', [
    PerceptionConfig(voxel_m=-1), PerceptionConfig(plane_distance_m=np.nan),
    PerceptionConfig(roi_min=(1., 1., 1.)), GraspConfig(pregrasp_distance_m=0),
    GraspConfig(jaw_opening_range_verified=True)])
def test_bad_configuration_fails(config):
    with pytest.raises(ValueError):
        config.validate()
