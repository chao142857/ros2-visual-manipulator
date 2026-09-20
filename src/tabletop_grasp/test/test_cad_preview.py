import json
from pathlib import Path

from builtin_interfaces.msg import Time
import numpy as np
import pytest
from visualization_msgs.msg import Marker

from tabletop_grasp.cad_preview import cad_markers


def test_preview_retains_native_frame_metres_and_explicit_unknowns():
    path = Path(__file__).parents[1] / 'config/cad_collision_proxy.json'
    if not path.exists() and (Path(__file__).parents[3] / 'PUBLICATION_MANIFEST.json').is_file():
        pytest.skip('Public software release excludes CAD collision proxy and mesh assets')
    proxy = json.loads(path.read_text())
    markers = cad_markers(proxy, Time(sec=1)).markers
    assert {marker.header.frame_id for marker in markers} == {'gripper_cad'}
    mesh = next(marker for marker in markers if marker.type == Marker.MESH_RESOURCE)
    assert mesh.mesh_resource == 'package://tabletop_grasp/meshes/cad_gripper_visual.stl'
    assert (mesh.scale.x, mesh.scale.y, mesh.scale.z) == (1., 1., 1.)
    cubes = [marker for marker in markers if marker.type == Marker.CUBE]
    assert len(cubes) == len(proxy['boxes'])
    for cube, definition in zip(cubes, proxy['boxes']):
        assert np.allclose([cube.scale.x, cube.scale.y, cube.scale.z], definition['dimensions'])
        point = cube.pose.position
        assert np.allclose([point.x, point.y, point.z], definition['center'])
    label = next(marker for marker in markers if marker.type == Marker.TEXT_VIEW_FACING)
    assert 'STATIC CAD' in label.text and 'UNVERIFIED' in label.text
