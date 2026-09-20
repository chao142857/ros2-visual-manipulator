"""Offline contracts for detached display transforms and rejected parameter edits."""

import ast
from copy import deepcopy
import math
from pathlib import Path
from types import SimpleNamespace

from builtin_interfaces.msg import Time
import pytest
from rclpy.parameter import Parameter
from rclpy.serialization import serialize_message
from visualization_msgs.msg import Marker

from tabletop_grasp.gripper_motion_preview import (
    FRAME_ID, GROUP_NAMES, GripperMotionPreviewNode, motion_markers, validate_parts,
)


def fixture_parts():
    return {'groups': [
        {'name': name, 'mesh': f'meshes/gripper_motion/{name}.stl',
         'bounds_mm': [10., 20., 30., 14., 26., 38.]}
        for name in GROUP_NAMES]}


def fixture_state():
    return {
        'status': 'approximate_unverified', 'pulse_us': 1300.,
        'opening_mm': 34., 'tcp_extension_mm': 127., 'theta_rad': 0.5,
        'transforms': {name: {'translation_mm': [4., 5., 6.],
                              'rotation_z_rad': math.pi / 2.0} for name in GROUP_NAMES},
    }


def test_original_mesh_and_candidate_box_share_the_same_rigid_transform():
    markers = motion_markers(fixture_parts(), fixture_state(), Time(sec=2))
    assert serialize_message(markers)
    assert {marker.header.frame_id for marker in markers.markers} == {FRAME_ID}
    mesh = next(marker for marker in markers.markers if marker.type == Marker.MESH_RESOURCE)
    assert mesh.mesh_resource == 'package://tabletop_grasp/meshes/gripper_motion/fixed.stl'
    assert (mesh.scale.x, mesh.scale.y, mesh.scale.z) == (1., 1., 1.)
    assert (mesh.pose.position.x, mesh.pose.position.y, mesh.pose.position.z) == (.004, .005, .006)
    assert mesh.pose.orientation.z == pytest.approx(math.sqrt(0.5))
    box = next(marker for marker in markers.markers if marker.type == Marker.CUBE)
    assert (box.pose.position.x, box.pose.position.y, box.pose.position.z) == pytest.approx(
        (-.019, .017, .040))
    assert (box.scale.x, box.scale.y, box.scale.z) == pytest.approx((.004, .006, .008))
    assert box.pose.orientation == mesh.pose.orientation
    label = next(marker for marker in markers.markers if marker.type == Marker.TEXT_VIEW_FACING)
    for expected in ('UNVERIFIED', '1300 us', 'opening 34.0 mm', 'TCP extension 127.0 mm',
                     'candidate group bounds', 'collision checks not performed'):
        assert expected in label.text


def test_hiding_candidate_bounds_removes_previously_published_boxes():
    shown = motion_markers(fixture_parts(), fixture_state(), Time(), show_bounds=True)
    hidden = motion_markers(fixture_parts(), fixture_state(), Time(), show_bounds=False)
    shown_ids = {(marker.ns, marker.id) for marker in shown.markers
                 if marker.type == Marker.CUBE}
    deleted_ids = {(marker.ns, marker.id) for marker in hidden.markers
                   if marker.action == Marker.DELETE}
    assert deleted_ids == shown_ids
    assert len(deleted_ids) == 7
    assert sum(marker.type == Marker.MESH_RESOURCE for marker in hidden.markers) == 7


def test_invalid_manifest_cannot_reference_external_mesh_or_ambiguous_groups():
    parts = fixture_parts()
    parts['groups'][0]['mesh'] = '../outside.stl'
    with pytest.raises(ValueError, match='package gripper_motion'):
        validate_parts(parts)
    parts = fixture_parts()
    parts['groups'][1] = deepcopy(parts['groups'][0])
    with pytest.raises(ValueError, match='seven gripper groups'):
        validate_parts(parts)
    parts = fixture_parts()
    parts['groups'][0]['bounds_mm'][3] = float('nan')
    with pytest.raises(ValueError, match='finite min XYZ'):
        validate_parts(parts)


class BoundedModel:
    def state(self, pulse_us):
        if not 600 <= pulse_us <= 1700:
            raise ValueError('pulse_us outside the configured estimate range')
        state = fixture_state()
        state['pulse_us'] = pulse_us
        return state


def fake_node():
    values = {'pulse_us': 1300., 'show_bounds': True}
    refreshes = []
    node = SimpleNamespace(
        model=BoundedModel(),
        get_parameter=lambda name: SimpleNamespace(value=values[name]),
        _refresh=SimpleNamespace(trigger=lambda: refreshes.append('refresh')))
    return node, values, refreshes


@pytest.mark.parametrize('value', [599, 1701, float('nan'), float('inf'), True, '1300', None])
def test_invalid_parameter_is_rejected_without_enqueuing_a_display_change(value):
    node, values, refreshes = fake_node()
    result = GripperMotionPreviewNode._validate_parameters(
        node, [Parameter('pulse_us', value=value)])
    assert not result.successful and result.reason
    assert values == {'pulse_us': 1300., 'show_bounds': True}
    assert refreshes == []


def test_valid_parameter_edit_only_enqueues_post_commit_refresh():
    node, values, refreshes = fake_node()
    result = GripperMotionPreviewNode._validate_parameters(
        node, [Parameter('pulse_us', value=600), Parameter('show_bounds', value=False)])
    assert result.successful and refreshes == ['refresh']
    assert values == {'pulse_us': 1300., 'show_bounds': True}


def test_preview_keeps_a_detached_ros_interface_and_launches_only_display_nodes():
    package = Path(__file__).parents[1]
    tree = ast.parse((package / 'tabletop_grasp/gripper_motion_preview.py').read_text())
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
             and isinstance(node.func, ast.Attribute)]
    assert not any(node.func.attr in ('create_subscription', 'create_client', 'create_timer')
                   for node in calls)
    topics = [node.args[1].value for node in calls if node.func.attr == 'create_publisher']
    assert set(topics) == {'/gripper_estimate/markers', '/gripper_estimate/state'}
    launch = ast.parse((package / 'launch/gripper_motion_preview.launch.py').read_text())
    executables = [keyword.value.value for node in ast.walk(launch) if isinstance(node, ast.Call)
                   and isinstance(node.func, ast.Name) and node.func.id == 'Node'
                   for keyword in node.keywords if keyword.arg == 'executable']
    assert set(executables) == {'gripper_motion_preview', 'rviz2'}
