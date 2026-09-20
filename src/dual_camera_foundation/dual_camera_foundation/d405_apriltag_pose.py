            
"""GUI-free AprilTag pose publisher for the wrist-mounted D405.

The node owns a dedicated target frame and never publishes the legacy
``tag_<id>`` frames used by the Astra pipeline. OpenCV's PnP result is
``T_camera_target`` and is exposed as a timestamped PoseStamped and, when
requested, a dynamic TF edge. The detected centre pixel is also exposed as
`PointStamped` (`x=u`, `y=v`) for the generic RGB-D provider.
"""

from __future__ import annotations

from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster

from camera_perception.apriltag_detector import (
    create_apriltag_detector,
    detect_apriltags,
)
from camera_perception.camera_geometry import intrinsics_from_camera_info
from camera_perception.tag_pose import (
    TAG_SIZES,
    estimate_tag_pose,
    rotation_matrix_to_quaternion,
)


def normalize_topic_prefix(prefix: str) -> str:
    """Return an absolute, slash-normalized ROS topic prefix."""
    value = str(prefix or '').strip()
    return '/' + value.strip('/') if value else ''


def join_topic(prefix: str, suffix: str) -> str:
    """Join a topic prefix and relative suffix without duplicate slashes."""
    clean_suffix = str(suffix or '').strip().strip('/')
    clean_prefix = normalize_topic_prefix(prefix)
    if not clean_prefix:
        return '/' + clean_suffix
    return clean_prefix if not clean_suffix else f'{clean_prefix}/{clean_suffix}'


def normalize_frame(frame: str) -> str:
    """Normalize a TF frame ID (TF frame IDs must not start with ``/``)."""
    return str(frame or '').strip().strip('/')


def apply_frame_prefix(frame: str, prefix: str) -> str:
    """Apply a prefix to a generated child frame without double-prefixing."""
    clean_frame = normalize_frame(frame)
    clean_prefix = normalize_frame(prefix)
    if not clean_frame or not clean_prefix or clean_frame == clean_prefix:
        return clean_frame
    return clean_frame if clean_frame.startswith(clean_prefix + '/') else f'{clean_prefix}/{clean_frame}'


def _finite_matrix(values, shape=(3, 3)) -> Optional[np.ndarray]:
    """Convert a CameraInfo matrix to a finite NumPy array if possible."""
    try:
        array = np.asarray(values, dtype=np.float64).reshape(shape)
    except (TypeError, ValueError):
        return None
    return array if np.all(np.isfinite(array)) else None


def camera_model_from_info(msg: CameraInfo):
    """Build camera matrix and distortion, preserving shared K-to-P fallback."""
    intrinsics = intrinsics_from_camera_info(msg)
    if intrinsics is None:
        return None
    fx, fy, cx, cy, source = intrinsics
    matrix = _finite_matrix(msg.k)
    if source != 'K' or matrix is None or matrix[0, 0] == 0.0 or matrix[1, 1] == 0.0:
        matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    if matrix[0, 0] == 0.0 or matrix[1, 1] == 0.0:
        return None
    distortion = np.asarray(list(msg.d), dtype=np.float64)
    if distortion.size == 0:
        distortion = np.zeros(5, dtype=np.float64)
    if not np.all(np.isfinite(distortion)):
        return None
    return matrix, distortion.reshape(-1, 1), source


class D405AprilTagPoseNode(Node):
    """Publish one configured AprilTag pose from a D405 color stream."""

    def __init__(self):
        super().__init__('d405_apriltag_pose')
        self.declare_parameter('topic_prefix', '/wrist_camera/wrist_d405')
        self.declare_parameter('frame_prefix', '')
        self.declare_parameter('rgb_topic', '')
        self.declare_parameter('camera_info_topic', '')
        self.declare_parameter('pose_topic', '')
        self.declare_parameter('target_pixel_topic', '')
        self.declare_parameter('camera_frame', '')
        self.declare_parameter('target_frame', 'wrist_calibration_target_6')
        self.declare_parameter('target_tag_id', 6)
        self.declare_parameter('tag_size_m', 0.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_target_pixel', True)
        self.declare_parameter('pixel_frame', '')
        self.declare_parameter('queue_depth', 5)
        self.declare_parameter('min_distance_m', 0.03)
        self.declare_parameter('max_distance_m', 3.0)

        value = self.get_parameter
        topic_prefix = str(value('topic_prefix').value)
        self.frame_prefix = str(value('frame_prefix').value)
        self.rgb_topic = self._resolved_topic(str(value('rgb_topic').value), topic_prefix, 'color/image_rect_raw')
        self.camera_info_topic = self._resolved_topic(str(value('camera_info_topic').value), topic_prefix, 'color/camera_info')
        self.pose_topic = self._resolved_topic(str(value('pose_topic').value), topic_prefix, 'calibration/target_pose')
        self.target_pixel_topic = self._resolved_topic(
            str(value('target_pixel_topic').value), topic_prefix,
            'calibration/target_pixel',
        )
        self.camera_frame = normalize_frame(str(value('camera_frame').value))
        self.pixel_frame = normalize_frame(str(value('pixel_frame').value))
        self.target_frame = apply_frame_prefix(str(value('target_frame').value), self.frame_prefix)
        self.target_tag_id = int(value('target_tag_id').value)
        configured_size = float(value('tag_size_m').value)
        self.tag_size_m = configured_size if configured_size > 0.0 else float(TAG_SIZES.get(self.target_tag_id, 0.0))
        self.publish_tf = bool(value('publish_tf').value)
        self.publish_target_pixel = bool(value('publish_target_pixel').value)
        self.queue_depth = max(1, int(value('queue_depth').value))
        self.min_distance_m = max(0.0, float(value('min_distance_m').value))
        self.max_distance_m = max(self.min_distance_m, float(value('max_distance_m').value))
        if self.tag_size_m <= 0.0:
            raise ValueError(f'tag_size_m must be supplied for unknown target_tag_id {self.target_tag_id}')
        if not self.target_frame:
            raise ValueError('target_frame must not be empty')
        if self.publish_target_pixel and not self.target_pixel_topic:
            raise ValueError('target_pixel_topic must not be empty when enabled')

        self.bridge = CvBridge()
        self.detector = create_apriltag_detector()
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.pose_publisher = self.create_publisher(PoseStamped, self.pose_topic, self.queue_depth)
        self.pixel_publisher = (
            self.create_publisher(PointStamped, self.target_pixel_topic, self.queue_depth)
            if self.publish_target_pixel else None
        )
        self.info_subscription = self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, qos_profile_sensor_data)
        self.image_subscription = self.create_subscription(Image, self.rgb_topic, self.image_callback, qos_profile_sensor_data)
        self.camera_matrix = None
        self.distortion = None
        self.camera_info_frame = ''
        self._last_invalid_info_log_ns = 0
        self._published_count = 0
        self._processed_frame_count = 0
        self._last_waiting_log_ns = 0
        self.get_logger().info(
            f'D405 AprilTag pose node started: rgb={self.rgb_topic}, camera_info={self.camera_info_topic}, '
            f'pose={self.pose_topic}, pixel={self.target_pixel_topic}, '
            f'tag_id={self.target_tag_id}, target_frame={self.target_frame}, '
            f'publish_tf={self.publish_tf}, publish_target_pixel={self.publish_target_pixel}'
        )

    @staticmethod
    def _resolved_topic(explicit: str, prefix: str, suffix: str) -> str:
        return normalize_topic_prefix(explicit) if explicit.strip() else join_topic(prefix, suffix)

    def camera_info_callback(self, msg: CameraInfo):
        model = camera_model_from_info(msg)
        if model is None:
            now = self.get_clock().now().nanoseconds
            if now - self._last_invalid_info_log_ns > 5_000_000_000:
                self.get_logger().warning(f'Ignoring invalid CameraInfo on {self.camera_info_topic}')
                self._last_invalid_info_log_ns = now
            return
        self.camera_matrix, self.distortion, source = model
        self.camera_info_frame = normalize_frame(msg.header.frame_id)
        self.get_logger().debug(f'CameraInfo ready ({source}) frame={self.camera_info_frame}')

    def _source_frame(self, image_msg: Image) -> str:
        return self.camera_frame or normalize_frame(image_msg.header.frame_id) or self.camera_info_frame

    def image_callback(self, msg: Image):
        if self.camera_matrix is None or self.distortion is None:
            return
        source_frame = self._source_frame(msg)
        if not source_frame:
            self.get_logger().warning('Dropping detection: image frame_id is empty')
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            detections = detect_apriltags(image, self.detector)
            self._processed_frame_count += 1
        except Exception as error:
            self.get_logger().error(f'AprilTag image processing failed: {error}')
            return
        target = next((item for item in detections if item['id'] == self.target_tag_id), None)
        if target is None:
            now = self.get_clock().now().nanoseconds
            if now - self._last_waiting_log_ns > 5_000_000_000:
                visible_ids = [item['id'] for item in detections]
                self.get_logger().info(
                    f'Waiting for AprilTag ID {self.target_tag_id}: '
                    f'processed_frames={self._processed_frame_count}, '
                    f'visible_ids={visible_ids}'
                )
                self._last_waiting_log_ns = now
            return
        try:
            estimate = estimate_tag_pose(target['corners'], self.tag_size_m, self.camera_matrix, self.distortion)
            if estimate is None:
                return
            translation = np.asarray(estimate['tvec'], dtype=np.float64).reshape(3)
            rotation = np.asarray(estimate['rotation_matrix'], dtype=np.float64).reshape(3, 3)
            if not np.all(np.isfinite(translation)) or not np.all(np.isfinite(rotation)):
                return
            distance = float(np.linalg.norm(translation))
            if translation[2] <= 0.0 or distance < self.min_distance_m or distance > self.max_distance_m:
                return
            quaternion = np.asarray(rotation_matrix_to_quaternion(rotation), dtype=np.float64).reshape(4)
            if not np.all(np.isfinite(quaternion)) or np.linalg.norm(quaternion) <= 1.0e-12:
                return
            quaternion /= np.linalg.norm(quaternion)
        except (TypeError, ValueError, cv2.error, FloatingPointError) as error:
            self.get_logger().warning(f'AprilTag PnP rejected: {error}')
            return

        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = source_frame
        pose_msg.pose.position.x = float(translation[0])
        pose_msg.pose.position.y = float(translation[1])
        pose_msg.pose.position.z = float(translation[2])
        pose_msg.pose.orientation.x = float(quaternion[0])
        pose_msg.pose.orientation.y = float(quaternion[1])
        pose_msg.pose.orientation.z = float(quaternion[2])
        pose_msg.pose.orientation.w = float(quaternion[3])
        self.pose_publisher.publish(pose_msg)
        if self.pixel_publisher is not None:
            center_u, center_v = target['center']
            if (
                np.isfinite(float(center_u))
                and np.isfinite(float(center_v))
                and 0 <= int(center_u) < msg.width
                and 0 <= int(center_v) < msg.height
            ):
                pixel_msg = PointStamped()
                pixel_msg.header = pose_msg.header
                if self.pixel_frame:
                    pixel_msg.header.frame_id = self.pixel_frame
                pixel_msg.point.x = float(center_u)
                pixel_msg.point.y = float(center_v)
                pixel_msg.point.z = 0.0
                self.pixel_publisher.publish(pixel_msg)
        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header = pose_msg.header
            tf_msg.child_frame_id = self.target_frame
            tf_msg.transform.translation.x = pose_msg.pose.position.x
            tf_msg.transform.translation.y = pose_msg.pose.position.y
            tf_msg.transform.translation.z = pose_msg.pose.position.z
            tf_msg.transform.rotation = pose_msg.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)
        self._published_count += 1
        if self._published_count == 1:
            self.get_logger().info(
                f'AprilTag ID {self.target_tag_id} acquired after '
                f'{self._processed_frame_count} processed frames'
            )


def main(args=None):
    """Run the D405 AprilTag pose node."""
    rclpy.init(args=args)
    node = D405AprilTagPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ['D405AprilTagPoseNode', 'apply_frame_prefix', 'camera_model_from_info', 'join_topic', 'main', 'normalize_frame', 'normalize_topic_prefix']


