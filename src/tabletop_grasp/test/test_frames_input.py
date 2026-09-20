from pathlib import Path

from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo
from tf2_geometry_msgs import do_transform_pose_stamped

from tabletop_grasp.config import PerceptionConfig
from tabletop_grasp.extrinsics import load_extrinsics
from tabletop_grasp.geometry import matrix_from_pose, pose_from_matrix, transform_points
from tabletop_grasp.rgbd import rgbd_to_xyz
from tabletop_grasp.ros_geometry import cloud_message, cloud_xyz


def test_pointcloud2_structured_numpy_roundtrip():
    from std_msgs.msg import Header
    xyz = np.array([[0.01, 0.02, 0.3], [0.1, 0.2, 0.4]])
    assert np.allclose(cloud_xyz(cloud_message(Header(), xyz)), xyz)
    assert cloud_xyz(cloud_message(Header(), np.empty((0, 3)))).shape == (0, 3)


def test_tf_points_agree_with_standard_tf2_pose_transform():
    transform = TransformStamped()
    transform.header.frame_id = 'base_link'
    transform.transform.translation.x = 0.3
    transform.transform.translation.z = 0.2
    q = Rotation.from_euler('xyz', [0.2, -0.7, 1.1]).as_quat()
    rotation = transform.transform.rotation
    rotation.x, rotation.y, rotation.z, rotation.w = map(float, q)
    points = np.array([[0.1, 0.02, 0.4]])
    source = PoseStamped()
    source.pose.position.x, source.pose.position.y, source.pose.position.z = points[0]
    source.pose.orientation.w = 1.
    expected = do_transform_pose_stamped(source, transform).pose.position
    assert np.allclose(transform_points(points, transform)[0], [expected.x, expected.y, expected.z])


def test_tcp_to_tool_transform_preserves_passive_extension():
    # Actual URDF TCP offset is along -Y, not a tag-derived Z correction.
    base_tcp = np.eye(4)
    base_tcp[:3, :3] = Rotation.from_euler('x', np.pi / 2).as_matrix()
    base_tcp[:3, 3] = [0.03, -0.28, 0.02]
    tcp_tool = np.eye(4)
    tcp_tool[1, 3] = 0.126
    base_tool = base_tcp @ tcp_tool
    assert np.allclose(base_tool[:3, 3], [0.03, -0.28, 0.146])
    assert np.allclose(matrix_from_pose(pose_from_matrix(base_tool)), base_tool)


@pytest.mark.parametrize('dtype', [np.uint16, np.float32])
def test_registered_depth_validity_and_pinhole_projection(dtype):
    info = CameraInfo(width=3, height=3, k=[100., 0., 1., 0., 100., 1., 0., 0., 1.],
                      r=np.eye(3).ravel().tolist())
    depth = np.zeros((3, 3), dtype=dtype)
    depth[1, 1] = 200 if dtype == np.uint16 else 0.2
    depth[1, 2] = 400 if dtype == np.uint16 else 0.4
    depth[0, 0] = 650 if dtype == np.uint16 else np.nan
    xyz = rgbd_to_xyz(np.zeros((3, 3, 3), dtype=np.uint8), depth, info, PerceptionConfig())
    assert np.allclose(xyz, [[0., 0., 0.2], [0.004, 0., 0.4]])


def test_registered_input_refuses_wrong_calibration_shape():
    with pytest.raises(ValueError, match='dimensions'):
        rgbd_to_xyz(np.zeros((3, 3, 3), np.uint8), np.zeros((3, 3), np.uint16),
                    CameraInfo(width=640, height=480), PerceptionConfig())


def test_existing_mount_is_read_without_apriltag_import():
    path = Path(__file__).parents[2] / 'wrist_camera_bringup/config/d405_mount_calibration.yaml'
    if not path.exists() and (Path(__file__).parents[3] / 'PUBLICATION_MANIFEST.json').is_file():
        pytest.skip('Public software release excludes site-specific mount calibration')
    parent, child, matrix = load_extrinsics(path)
    assert (parent, child) == ('tool0', 'wrist_d405_link')
    assert np.linalg.det(matrix[:3, :3]) == pytest.approx(1.)


def test_bad_extrinsic_matrix_fails(tmp_path):
    import yaml
    path = tmp_path / 'bad.yaml'
    data = {'schema': 'dual_camera_foundation.hand_eye_calibration',
            'tool_frame': 'tool0', 'camera_frame': 'camera',
            'validation': {'sample_count': 15, 'translation_rms_m': 0.001, 'rotation_rms_deg': 1.},
            'T_tool_camera': (np.eye(4) * 2).tolist()}
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='homogeneous'):
        load_extrinsics(path)
