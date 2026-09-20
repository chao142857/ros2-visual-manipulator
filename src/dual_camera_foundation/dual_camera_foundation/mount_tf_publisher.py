"""Publish a validated ``tool0 -> wrist_d405_link`` static transform."""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from .hand_eye_calibration import load_calibration


def quaternion_from_matrix(matrix):
    """Convert a proper rotation matrix to an xyzw quaternion."""
    trace = float(np.trace(matrix))
    if trace > 0:
        scale = 2 * np.sqrt(trace + 1)
        result = np.array([
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        index = int(np.argmax(np.diag(matrix)))
        nxt, last = (index + 1) % 3, (index + 2) % 3
        scale = 2 * np.sqrt(max(
            1e-15,
            1 + matrix[index, index]
            - matrix[nxt, nxt]
            - matrix[last, last],
        ))
        result = np.zeros(4)
        result[index] = scale / 4
        result[3] = (matrix[last, nxt] - matrix[nxt, last]) / scale
        result[nxt] = (matrix[nxt, index] + matrix[index, nxt]) / scale
        result[last] = (matrix[last, index] + matrix[index, last]) / scale
    norm = float(np.linalg.norm(result))
    if not np.all(np.isfinite(result)) or norm <= 1.0e-12:
        raise ValueError('calibration rotation produced an invalid quaternion')
    return result / norm


def validate_mount_calibration(
    calibration_file,
    parent_frame='tool0',
    child_frame='wrist_d405_link',
    minimum_sample_count=15,
    max_translation_rms_m=0.01,
    max_rotation_rms_deg=2.0,
    max_outlier_count=0,
):
    """Load and validate a mount file without publishing any TF."""
    path = str(calibration_file).strip()
    if not path:
        raise ValueError('calibration_file is required')
    parent = str(parent_frame).strip().strip('/')
    child = str(child_frame).strip().strip('/')
    if not parent or not child or parent == child:
        raise ValueError('mount parent/child frames must be distinct and non-empty')
    minimum_sample_count = int(minimum_sample_count)
    max_translation_rms_m = float(max_translation_rms_m)
    max_rotation_rms_deg = float(max_rotation_rms_deg)
    max_outlier_count = int(max_outlier_count)
    if minimum_sample_count < 1:
        raise ValueError('minimum_sample_count must be positive')
    if (
        not np.isfinite(max_translation_rms_m)
        or max_translation_rms_m < 0
    ):
        raise ValueError(
            'max_translation_rms_m must be finite and non-negative'
        )
    if (
        not np.isfinite(max_rotation_rms_deg)
        or max_rotation_rms_deg < 0
    ):
        raise ValueError(
            'max_rotation_rms_deg must be finite and non-negative'
        )
    if max_outlier_count < 0:
        raise ValueError('max_outlier_count must be non-negative')

    calibration = load_calibration(path)
    metrics = calibration.metrics
    if metrics is None:
        raise ValueError('calibration lacks validation metrics')
    if metrics.sample_count < minimum_sample_count:
        raise ValueError(
            f'calibration has only {metrics.sample_count} samples; '
            f'need at least {minimum_sample_count}'
        )
    if not np.isfinite(metrics.translation_rms_m):
        raise ValueError('calibration translation RMS is not finite')
    if not np.isfinite(metrics.rotation_rms_deg):
        raise ValueError('calibration rotation RMS is not finite')
    if metrics.translation_rms_m > max_translation_rms_m:
        raise ValueError(
            'calibration translation RMS exceeds limit: '
            f'{metrics.translation_rms_m:.6g} m > '
            f'{max_translation_rms_m:.6g} m'
        )
    if metrics.rotation_rms_deg > max_rotation_rms_deg:
        raise ValueError(
            'calibration rotation RMS exceeds limit: '
            f'{metrics.rotation_rms_deg:.6g} deg > '
            f'{max_rotation_rms_deg:.6g} deg'
        )
    outlier_count = len(metrics.outlier_indices)
    if outlier_count > max_outlier_count:
        raise ValueError(
            f'calibration has {outlier_count} outlier samples; '
            f'maximum allowed is {max_outlier_count}'
        )
    if (
        calibration.tool_frame.strip().strip('/') != parent
        or calibration.camera_frame.strip().strip('/') != child
    ):
        raise ValueError(
            'frame mismatch: file '
            f'{calibration.tool_frame}->{calibration.camera_frame}, '
            f'requested {parent}->{child}'
        )
    return calibration


def validate_mount_contract(
    start_d405_transform,
    use_mount_calibration,
    calibration_file,
    parent_frame,
    child_frame,
    expected_child_frame,
    minimum_sample_count=15,
    max_translation_rms_m=0.01,
    max_rotation_rms_deg=2.0,
    max_outlier_count=0,
):
    """Fail closed for launch combinations that could expose bad base points."""
    if start_d405_transform and not use_mount_calibration:
        raise ValueError(
            'start_d405_transform:=true requires '
            'use_mount_calibration:=true'
        )
    if not use_mount_calibration:
        return None
    child = str(child_frame).strip().strip('/')
    expected = str(expected_child_frame).strip().strip('/')
    if child != expected:
        raise ValueError(
            f'D405 mount child frame must be {expected!r}, got {child!r}'
        )
    return validate_mount_calibration(
        calibration_file,
        parent_frame,
        child,
        minimum_sample_count,
        max_translation_rms_m,
        max_rotation_rms_deg,
        max_outlier_count,
    )


class MountTfPublisher(Node):
    """Refuse missing or unvalidated calibration rather than publish identity."""

    def __init__(self):
        super().__init__('calibrated_wrist_camera_mount_publisher')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('parent_frame', 'tool0')
        self.declare_parameter('child_frame', 'wrist_d405_link')
        self.declare_parameter('minimum_sample_count', 15)
        self.declare_parameter('max_translation_rms_m', 0.01)
        self.declare_parameter('max_rotation_rms_deg', 2.0)
        self.declare_parameter('max_outlier_count', 0)
        value = self.get_parameter
        path = str(value('calibration_file').value).strip()
        parent = str(value('parent_frame').value).strip().strip('/')
        child = str(value('child_frame').value).strip().strip('/')
        calibration = validate_mount_calibration(
            path,
            parent,
            child,
            value('minimum_sample_count').value,
            value('max_translation_rms_m').value,
            value('max_rotation_rms_deg').value,
            value('max_outlier_count').value,
        )
        matrix = calibration.T_tool_camera
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = parent
        message.child_frame_id = child
        message.transform.translation.x = float(matrix[0, 3])
        message.transform.translation.y = float(matrix[1, 3])
        message.transform.translation.z = float(matrix[2, 3])
        quaternion = quaternion_from_matrix(matrix[:3, :3])
        message.transform.rotation.x = float(quaternion[0])
        message.transform.rotation.y = float(quaternion[1])
        message.transform.rotation.z = float(quaternion[2])
        message.transform.rotation.w = float(quaternion[3])
        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(message)
        self.get_logger().info(
            f'published validated {parent} -> {child}: {path}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MountTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = [
    'MountTfPublisher',
    'main',
    'quaternion_from_matrix',
    'validate_mount_calibration',
    'validate_mount_contract',
]
