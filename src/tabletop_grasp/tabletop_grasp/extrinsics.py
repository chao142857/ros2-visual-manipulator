"""Load a persistent calibrated transform without running a tag detector."""

from pathlib import Path

from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from tf2_ros import StaticTransformBroadcaster
import yaml


def load_extrinsics(path):
    """Support existing hand-eye and fixed-camera calibration file schemas."""
    data = yaml.safe_load(Path(path).read_text())
    if data.get('schema') == 'dual_camera_foundation.hand_eye_calibration':
        parent, child = data['tool_frame'], data['camera_frame']
        matrix = np.asarray(data['T_tool_camera'], dtype=float)
        metrics = data.get('validation') or {}
        if metrics.get('sample_count', 0) < 1:
            raise ValueError('hand-eye file has no calibration samples')
        if not np.isfinite([metrics.get('translation_rms_m', np.nan),
                            metrics.get('rotation_rms_deg', np.nan)]).all():
            raise ValueError('invalid hand-eye calibration validation metrics')
    else:
        parameters = data['camera_extrinsics_publisher_node']['ros__parameters']
        if parameters.get('calibrated') is not True:
            raise ValueError('fixed-camera extrinsics are not calibrated')
        parent, child = parameters['parent_frame'], parameters['child_frame']
        matrix = np.eye(4)
        quaternion = np.asarray(parameters['quaternion'], dtype=float)
        if quaternion.shape != (4,) or not np.isclose(np.linalg.norm(quaternion), 1., atol=1e-3):
            raise ValueError('invalid extrinsic quaternion')
        matrix[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
        matrix[:3, 3] = parameters['xyz']
    if (not parent or not child or parent == child or parent.startswith('/') or child.startswith('/')
            or any(c.isspace() for c in parent + child)):
        raise ValueError('invalid extrinsic frame names')
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all() or \
            not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError('invalid homogeneous camera extrinsic')
    r = matrix[:3, :3]
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(r), 1., atol=1e-5):
        raise ValueError('camera extrinsic rotation is not proper orthonormal')
    return parent, child, matrix


class ExtrinsicsNode(Node):
    def __init__(self):
        super().__init__('tabletop_extrinsics')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('expected_parent', 'tool0')
        self.declare_parameter('expected_child', 'wrist_d405_link')
        parent, child, matrix = load_extrinsics(self.get_parameter('calibration_file').value)
        if (parent, child) != (self.get_parameter('expected_parent').value,
                               self.get_parameter('expected_child').value):
            raise ValueError('calibration parent/child does not match requested camera mount')
        message = TransformStamped(child_frame_id=child)
        message.header.frame_id = parent
        message.header.stamp = self.get_clock().now().to_msg()
        t = message.transform.translation
        t.x, t.y, t.z = map(float, matrix[:3, 3])
        q = message.transform.rotation
        q.x, q.y, q.z, q.w = map(float, Rotation.from_matrix(matrix[:3, :3]).as_quat())
        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(message)
        self.get_logger().info(f'Loaded persistent {parent} -> {child}; calibration accuracy '
                               'still requires physical verification. No runtime tag detector.')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ExtrinsicsNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
