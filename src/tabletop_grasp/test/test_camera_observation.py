"""Camera coordinates must remain useful without fabricated base/TCP state."""

from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header
from tf2_ros import TransformException
from visualization_msgs.msg import Marker

from tabletop_grasp.config import PerceptionConfig
from tabletop_grasp.geometry import Box, estimate_box, refine_box_from_top
from tabletop_grasp.perception_node import PerceptionNode
from tabletop_grasp.pipeline import TabletopPipeline
from tabletop_grasp.ros_geometry import orientation_preview_markers
from tabletop_grasp.synthetic_scene import cuboid_scene


def camera_scene(yaw=0., seed=7, size=(0.08, 0.034, 0.04)):
    rotation = Rotation.from_euler('xyz', [np.pi, 0.25, -0.3]).as_matrix()
    origin = rotation @ [0.03, -0.28, -0.012]
    xyz = cuboid_scene(yaw=yaw, seed=seed, size=size) @ rotation.T - origin + [0., 0., 0.4]
    return xyz, rotation


def camera_pipeline():
    config = replace(PerceptionConfig(), roi_min=(-0.45, -0.35, 0.07),
                     roi_max=(0.45, 0.35, 0.60))
    return TabletopPipeline(config, frame_mode='camera')


def test_camera_geometry_does_not_require_base_gravity_or_tf():
    xyz, rotation = camera_scene()
    result = camera_pipeline().process(xyz)
    assert result.valid, result.reason
    box = result.boxes[result.target_index]
    assert np.allclose(box.dimensions, [0.08, 0.034, 0.04], atol=0.004)
    assert np.dot(result.plane[:3], rotation[:, 2]) > 0.999
    assert result.plane[3] > 0  # Camera origin lies on the positive side.
    assert np.dot(result.plane[:3], -result.table_points.mean(axis=0)) > 0
    assert not TabletopPipeline(PerceptionConfig()).process(xyz).valid


def test_camera_rotation_sequence_preserves_long_short_and_pi_preview():
    pipeline = camera_pipeline()
    previous = None
    for seed, degrees in enumerate((0, 20, 45, 70, 90)):
        xyz, camera_rotation = camera_scene(np.radians(degrees), seed)
        result = pipeline.process(xyz)
        assert result.valid, result.reason
        box = result.boxes[result.target_index]
        expected = camera_rotation @ Rotation.from_euler('z', degrees, degrees=True).apply([1., 0., 0.])
        assert abs(np.dot(box.rotation[:, 0], expected)) > 0.995
        assert box.dimensions[0] > box.dimensions[1]
        if previous is not None:
            assert np.dot(box.rotation[:, 0], previous[:, 0]) > 0.89
        previous = box.rotation
        markers = orientation_preview_markers(Header(frame_id='camera_optical'), result, 1.5)
        arrows = [m for m in markers.markers if m.type == Marker.ARROW and m.action == Marker.ADD]
        assert len(arrows) == 2
        for sign, marker in zip((1, -1), arrows):
            points = np.array([[p.x, p.y, p.z] for p in marker.points])
            assert np.allclose(points[0], box.center)
            assert np.dot((points[1] - points[0]) / 0.065, box.rotation[:, 1]) == pytest.approx(sign)


def test_camera_ambiguous_footprint_invalidates_orientation_preview():
    xyz, _ = camera_scene(size=(0.04, 0.039, 0.04))
    pipeline = camera_pipeline()
    result = pipeline.process(xyz)
    assert not result.valid and 'unobservable' in result.reason
    assert len(result.target_points) > 0  # Retain evidence for diagnosis.
    assert not result.boxes[0].yaw_observable
    assert pipeline.previous is None
    markers = orientation_preview_markers(Header(), result, 1.5)
    assert len(markers.markers) == 1 and markers.markers[0].action == Marker.DELETEALL


def test_unavailable_base_tf_is_diagnostic_not_camera_geometry_failure():
    import json

    class MissingTF:
        def lookup_transform(self, *args):
            raise TransformException('no live joint/TCP chain')

    header = Header(frame_id='camera_optical')
    header.stamp.sec = 10
    fake = SimpleNamespace(
        cfg=PerceptionConfig(),
        p={'frame_mode': 'camera', 'base_frame': 'base_link',
           'max_age_sec': 1.0, 'future_tolerance_sec': 0.05},
        last_source_header=header, buffer=MissingTF(),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10000000000)))
    result = camera_pipeline().process(camera_scene()[0])
    values = {v.key: json.loads(v.value) for v in PerceptionNode._diagnostic_values(fake, header, result)}
    assert values['geometry_valid'] is True
    assert values['base_tf_available'] is False
    assert values['base_tf_trustworthy'] is False
    assert values['source_header_fresh'] is True
    assert values['orientation_preview_count'] == 2
    assert len(values['center_camera_m']) == 3
    assert 'center_base_m' not in values
    assert 'gravity unverified' in values['normal_semantics']


def test_camera_launch_contains_only_camera_perception_and_rviz():
    path = Path(__file__).parents[1] / 'launch/camera_observation.launch.py'
    source = path.read_text()
    assert "'frame_mode': 'camera'" in source
    assert "'output_namespace': '/tabletop_camera'" in source
    for forbidden in ('arm_driver', 'gripper_driver', 'robot_state_publisher',
                      "executable='planner'", "executable='extrinsics'", 'move_group'):
        assert forbidden not in source


def test_top_surface_refinement_cannot_enter_base_collision_pipeline():
    with pytest.raises(ValueError, match='camera-only'):
        TabletopPipeline(replace(PerceptionConfig(), refine_top_surface=True))


def test_top_surface_reconstructs_uneven_depth_skirt_without_shrinking_full_envelope():
    rng = np.random.default_rng(21)
    top = rng.uniform([-.03, -.018, .03], [.03, .018, .03], (700, 3))
    side = rng.uniform([-.03, -.018, .006], [.03, .018, .025], (1100, 3))
    side[:550, 0] = .03 + .015 * (1 - side[:550, 2] / .03)
    side[550:, 1] = -.018 - .015 * (1 - side[550:, 2] / .03)
    points = np.vstack((top, side))
    config = replace(PerceptionConfig(), refine_top_surface=True)
    plane = np.array([0., 0., 1., 0.])
    raw = estimate_box(points, plane, config)
    metrics = {}
    refined = refine_box_from_top(points, plane, config, raw, metrics)
    assert np.allclose(refined.dimensions, [.06, .036, .03], atol=.002)
    assert refined.observed_bounds is raw
    assert np.prod(raw.dimensions[:2]) > 1.15 * np.prod(refined.dimensions[:2])
    assert metrics['collision_geometry_replaced'] is False
    assert metrics['outside_top_footprint_ratio'] > 0


def test_partial_triangular_top_has_insufficient_rectangular_support():
    rng = np.random.default_rng(3)
    xy = rng.uniform([0., 0.], [.08, .04], (2000, 2))
    xy = xy[xy[:, 0] / .08 + xy[:, 1] / .04 < 1.]
    points = np.column_stack((xy, np.full(len(xy), .03)))
    plane = np.array([0., 0., 1., 0.])
    cfg = replace(PerceptionConfig(), refine_top_surface=True)
    raw = estimate_box(points, plane, cfg)
    metrics = {}
    with pytest.raises(ValueError, match='rectangular footprint'):
        refine_box_from_top(points, plane, cfg, raw, metrics)
    assert metrics['top_footprint_fill_ratio'] < cfg.top_surface_min_footprint_fill


def test_top_surface_does_not_hide_a_non_cuboid_obstruction():
    rng = np.random.default_rng(7)
    top = rng.uniform([-.03, -.018, .03], [.03, .018, .03], (700, 3))
    bump = rng.uniform([-.01, -.01, .05], [.01, .01, .08], (200, 3))
    points = np.vstack((top, bump))
    cfg = replace(PerceptionConfig(), refine_top_surface=True)
    plane = np.array([0., 0., 1., 0.])
    raw = estimate_box(points, plane, cfg)
    metrics = {}
    with pytest.raises(ValueError, match='noncuboid'):
        refine_box_from_top(points, plane, cfg, raw, metrics)
    assert metrics['above_top_outlier_ratio'] > .05


def test_narrow_visible_top_strip_is_rejected_before_axis_estimation():
    rng = np.random.default_rng(11)
    top = rng.uniform([-.03, -.002, .03], [.03, .002, .03], (700, 3))
    sides = rng.uniform([-.03, -.018, .006], [.03, .018, .015], (300, 3))
    points = np.vstack((top, sides))
    cfg = replace(PerceptionConfig(), refine_top_surface=True)
    plane = np.array([0., 0., 1., 0.])
    raw = estimate_box(points, plane, cfg)
    metrics = {}
    with pytest.raises(ValueError, match='unobservable'):
        refine_box_from_top(points, plane, cfg, raw, metrics)
    assert not metrics.get('refinement_valid', False)


def test_refined_camera_sequence_preserves_short_side_at_manual_test_angles():
    pipeline = camera_pipeline()
    pipeline.cfg.refine_top_surface = True
    previous = None
    for degrees in (0, 20, 45, 70, 90):
        points, camera_rotation = camera_scene(np.radians(degrees))
        result = pipeline.process(points)
        assert result.valid, result.reason
        box = result.boxes[result.target_index]
        assert np.allclose(box.dimensions, [.08, .034, .04], atol=.004)
        expected_short = camera_rotation @ Rotation.from_euler('z', degrees, degrees=True).apply([0., 1., 0.])
        assert abs(np.dot(box.rotation[:, 1], expected_short)) > .995
        if previous is not None:
            assert np.dot(box.rotation[:, 0], previous[:, 0]) > .89
        previous = box.rotation


def metrology_gate():
    config = replace(PerceptionConfig(), refine_top_surface=True,
                     metrology_consistency_enabled=True)
    return TabletopPipeline(config, frame_mode='camera')


def metrology_sample(gate, stamp, dimensions=(.064, .038, .028), degrees=0., center=None):
    gate.begin_metrology_frame(stamp)
    box = Box(np.array([0., .1, .25] if center is None else center),
              Rotation.from_euler('z', degrees, degrees=True).as_matrix(), np.array(dimensions))
    metrics = {}
    reason = gate.check_metrology_box(box, metrics)
    return reason, metrics, box


def test_metrology_requires_three_fresh_fits_and_never_substitutes_measurements():
    gate = metrology_gate()
    assert '1/3' in metrology_sample(gate, 1.)[0]
    assert '2/3' in metrology_sample(gate, 1.25)[0]
    reason, metrics, output = metrology_sample(gate, 1.5, dimensions=(.066, .039, .027))
    assert not reason and metrics['accepted']
    assert np.array_equal(output.dimensions, [.066, .039, .027])
    assert not metrics['full_extent_verified']


@pytest.mark.parametrize('dimensions', [(.085, .049, .026), (.062, .027, .028)])
def test_metrology_rejects_observed_growth_and_partial_width_without_replacing_reference(dimensions):
    gate = metrology_gate()
    for stamp in (1., 1.25, 1.5):
        metrology_sample(gate, stamp)
    reason, metrics, _ = metrology_sample(gate, 1.75, dimensions=dimensions)
    assert 'changed dimensions' in reason
    assert np.array_equal(gate.metrology_reference.dimensions, [.064, .038, .028])
    assert not metrics['accepted']
    assert '1/3' in metrology_sample(gate, 2.)[0]
    assert '2/3' in metrology_sample(gate, 2.25)[0]
    assert not metrology_sample(gate, 2.5)[0]


def test_metrology_preserves_dimension_reference_across_bad_geometry_and_allows_rotation():
    gate = metrology_gate()
    for stamp in (1., 1.25, 1.5):
        metrology_sample(gate, stamp)
    gate.begin_metrology_frame(1.75)
    gate.reset()  # Geometric fit failed, but the input stream stayed fresh.
    assert gate.metrology_reference is not None
    for index, angle in enumerate((45., 70., 90.)):
        reason, _, box = metrology_sample(gate, 2. + .25 * index, degrees=angle)
        assert bool(reason) == (index < 2)
        assert np.allclose(box.rotation, Rotation.from_euler('z', angle, degrees=True).as_matrix())


def test_metrology_reacquires_on_timeout_and_object_relocation():
    gate = metrology_gate()
    for stamp in (1., 1.25, 1.5):
        metrology_sample(gate, stamp)
    reason, _, _ = metrology_sample(gate, 3., dimensions=(.07, .048, .035))
    assert '1/3' in reason and gate.metrology_reference is None
    metrology_sample(gate, 3.25, dimensions=(.07, .048, .035))
    assert not metrology_sample(gate, 3.5, dimensions=(.07, .048, .035))[0]
    reason, metrics, _ = metrology_sample(gate, 3.75, center=[.1, .1, .25])
    assert '1/3' in reason and 'relocated' in metrics['reference_reset_reason']
    gate.reset(expire=True)
    assert gate.metrology_reference is None and gate.metrology_count == 0


def test_metrology_rejects_replayed_or_missing_source_timestamps():
    gate = metrology_gate()
    metrology_sample(gate, 1.)
    with pytest.raises(ValueError, match='did not advance'):
        metrology_sample(gate, 1.)
    assert gate.metrology_count == 0
    with pytest.raises(ValueError, match='source timestamp'):
        gate.begin_metrology_frame(None)


def test_metrology_requires_pairwise_agreement_not_only_agreement_to_first_sample():
    gate = metrology_gate()
    metrology_sample(gate, 1., dimensions=(.064, .038, .028))
    metrology_sample(gate, 1.25, dimensions=(.069, .038, .028))
    reason, metrics, _ = metrology_sample(gate, 1.5, dimensions=(.059, .038, .028))
    assert '1/3' in reason  # 10 mm span despite each being 5 mm from first.
    assert not metrics['accepted'] and gate.metrology_reference is None
